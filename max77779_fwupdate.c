/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023, Google Inc
 *
 * MAX77779 firmware updater
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>

#include "google_bms.h"
#include "max77779_regs.h"
#include "max77779.h"
#include "maxfg_common.h"

#define MAX77779_FIRMWARE_BINARY_PREFIX "batt_fw_adi_79"

#define FW_UPDATE_RETRY_CPU_RESET             100
#define FW_UPDATE_RETRY_FW_UPDATE             1000
#define FW_UPDATE_RETRY_RISCV_REBOOT          20
#define FW_UPDATE_WAIT_INTERVAL_MS            250
#define FW_UPDATE_WAIT_LOAD_BIN_MS            100
#define FW_UPDATE_TIMER_CHECK_INTERVAL_MS     1000
#define FW_UPDATE_CONDITION_CHECK_INTERVAL_MS (60 * 1000)

#define FW_UPDATE_MAXIMUM_PAGE_SIZE            (PAGE_SIZE*10)

/* b/308445917: adding device tree for voltage threshold */
#define MAX77779_FW_UPDATE_MIN_VOLTAGE 4100

#define MAX77779_FW_IMG_SZ_HEADER 8
#define MAX77779_FW_IMG_SZ_PACKET 42
#define MAX77779_FW_IMG_SZ_FRAME (MAX77779_FW_IMG_SZ_PACKET * 20)

/* FG's reg 0x40 and status value of 0x82 are not documented */
#define MAX77779_FG_BOOT_CHECK_REG 0x40
#define MAX77779_FG_BOOT_CHECK_SUCCESS 0x82

/* vimon's memory mapped to 0x80 */
#define MAX77779_VIMON_MEM_BASE_ADDR 0x80
#define MAX77779_VIMON_PG_SIZE 256
#define MAX77779_VIMON_PG3_SIZE (MAX77779_VIMON_PG_SIZE - 32)

#define MAX77779_OFFSET_VER_MAJOR 7
#define MAX77779_OFFSET_VER_MINOR 6

#define MAX77779_FW_UPDATE_STRING_MAX 32

#define MAX77779_GET_DATA_FRAME_SIZE(filelen) \
	(filelen - MAX77779_FW_IMG_SZ_HEADER - 3*MAX77779_FW_IMG_SZ_PACKET)

#define MAX77779_ABORT_ON_ERROR(result, name, err_op) { \
	if (result) { \
		dev_err(fwu->dev, "[%s] failed: %s (%d)\n", name, err_op, result); \
		return result; \
	} \
}

#define MARK_IN_PROGRESS() do {} while (0);

enum max77779_fwupdate_fg_lock {
	FG_ST_LOCK_ALL_SECTION = 0x0e,
	FG_ST_UNLOCK_ALL_SECTION = 0x00,
};

enum max77779_fg_operation_status {
	FGST_NOT_CACHED = 0x01,
	FGST_FWUPDATE = 0x02,
	FGST_ERR_READTAG = 0x10,
	FGST_NORMAL = 0xff,
};

enum max77779_fwupdate_intr {
	MAX77779_INTR_CLEAR = 0x00,
	MAX77779_INTR_SESSION_START = 0x70,
	MAX77779_INTR_TRANSFER_FRAMES = 0x72,
	MAX77779_INTR_APP_VALID = 0x77,
	MAX77779_INTR_SESSION_END = 0x74,
};

enum max77779_fwupdate_rsp_code {
	MAX77779_RSP_CODE_OK = 0x00,
	MAX77779_RSP_CODE_UNEXPECTED = 0xF0,
	MAX77779_RSP_CODE_CMD_SEC_FAIL = 0xF1,
	MAX77779_RSP_CODE_INVALID_PARAM = 0xF4,
	MAX77779_RSP_CODE_NOT_READY = 0xFF,
};

enum max77779_fwupdate_cmd {
	MAX77779_CMD_CLEAR_ALL = 0x00,
	MAX77779_CMD_REBOOT_RISCV = 0x080F,
};

struct max77779_version_info {
	u8 major;
	u8 minor;
};

struct max77779_fwupdate_custom_data {
	ssize_t size;
	char*  data;
};

struct max77779_fwupdate {
	struct device *dev;
	struct dentry *de;

	struct delayed_work update_work;

	struct i2c_client *pmic;
	struct i2c_client *fg;
	struct i2c_client *vimon;
	struct i2c_client *chg;

	struct platform_device *batt;

	bool can_update;

	struct max77779_version_info v_cur;
	struct max77779_version_info v_new;

	char fw_name[MAX77779_FW_UPDATE_STRING_MAX];

	u32 restrict_level_critical;
	size_t data_frame_size;
	u32 crc_val;

	u8 op_st;

	u8* scratch_buffer;
	u8* zero_filled_buffer;

	int minimum_voltage;

	struct max77779_version_info minimum;

	struct max77779_fwupdate_custom_data debug_image;
};

static int max77779_fwupdate_init(struct max77779_fwupdate *fwu)
{
	struct device* dev = fwu->dev;
	struct device_node *dn;
	int val = 0;

	if (!dev)
		return -EINVAL;

	fwu->minimum_voltage = MAX77779_FW_UPDATE_MIN_VOLTAGE;

	fwu->debug_image.data = NULL;
	fwu->debug_image.size = 0;

	if (!fwu->pmic) {
		dn = of_parse_phandle(dev->of_node, "max77779,pmic", 0);
		if (!dn)
			return -ENXIO;

		fwu->pmic = of_find_i2c_device_by_node(dn);
		if (!fwu->pmic)
			return -EPROBE_DEFER;
	}

	if (!fwu->fg) {
		dn = of_parse_phandle(dev->of_node, "max77779,fg", 0);
		if (!dn)
			return -ENXIO;

		fwu->fg = of_find_i2c_device_by_node(dn);
		if (!fwu->fg)
			return -EPROBE_DEFER;
	}

	if (!fwu->vimon) {
		dn = of_parse_phandle(dev->of_node, "max77779,vimon", 0);
		if (!dn)
			return -ENXIO;

		fwu->vimon = of_find_i2c_device_by_node(dn);
		if (!fwu->vimon)
			return -EPROBE_DEFER;
	}

	if (!fwu->chg) {
		dn = of_parse_phandle(dev->of_node, "max77779,chg", 0);
		if (!dn)
			return -ENXIO;

		fwu->chg = of_find_i2c_device_by_node(dn);
		if (!fwu->chg)
			return -EPROBE_DEFER;
	}

	if (!fwu->batt) {
		dn = of_parse_phandle(dev->of_node, "google,battery", 0);
		if (!dn)
			return -ENXIO;

		fwu->batt = of_find_device_by_node(dn);
		if (!fwu->batt)
			return -EPROBE_DEFER;
	}

	if (of_property_read_u32(dev->of_node, "fwu,enabled", &val) == 0)
		fwu->can_update = val;

	if (of_property_read_u32(dev->of_node, "minimum-voltage", &val) == 0)
		fwu->minimum_voltage = val;

	if (of_property_read_u32(dev->of_node, "version-major", &val) == 0)
		fwu->minimum.major = (u8)val;

	if (of_property_read_u32(dev->of_node, "version-minor", &val) == 0)
		fwu->minimum.minor = (u8)val;


	return 0;
}

static int max77779_wait_cpu_reset(struct max77779_fwupdate *fwu)
{
	int ret;
	int cnt = 0;
	unsigned int val;

	dev_info(fwu->dev, "waiting for cpu reset\n");

	while (cnt < FW_UPDATE_RETRY_CPU_RESET) {
		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
		ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_FG_AP_DATAIN0, &val);
		if (ret == 0) {
			if (val != MAX77779_RSP_CODE_UNEXPECTED) {
				MARK_IN_PROGRESS();
			} else {
				dev_info(fwu->dev, "cpu reset completed\n");
				return 0;
			}
		}
		cnt++;
	}

	dev_err(fwu->dev, "timeout for max77779_wait_cpu_reset\n");
	return -ETIMEDOUT;
}

static int max77779_wait_fw_update(struct max77779_fwupdate *fwu)
{
	int ret;
	int cnt = 0;
	unsigned int val;

	dev_info(fwu->dev, "waiting for firmware update\n");

	do {
		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
		cnt++;

		ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_FG_AP_DATAIN0, &val);
		if (ret != 0 || val == MAX77779_RSP_CODE_NOT_READY) {
			MARK_IN_PROGRESS();
			continue;
		} else if (val == MAX77779_RSP_CODE_UNEXPECTED) {
			dev_err(fwu->dev, "failed to firmware update rsp %02x\n", val);
			return -EBADFD;
		}

		dev_info(fwu->dev, "firmware update completed: rsp %02x\n", val);
		return 0;

	} while (cnt < FW_UPDATE_RETRY_FW_UPDATE);

	dev_err(fwu->dev, "timeout for max77779_wait_fw_update\n");
	return -ETIMEDOUT;
}

static int max77779_wait_riscv_reboot(struct max77779_fwupdate *fwu)
{
	int ret;
	int cnt = 0;
	unsigned int val;

	dev_info(fwu->dev, "waiting for riscv reboot\n");

	while (cnt < FW_UPDATE_RETRY_RISCV_REBOOT) {
		msleep(FW_UPDATE_WAIT_INTERVAL_MS);
		ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_FG_INT_STS, &val);
		if (!ret && (val & MAX77779_FG_FG_INT_MASK_POR_m_MASK)) {
			dev_info(fwu->dev, "wait_risc_reboot POR interrupt received\n");
			return 0;
		}

		cnt++;
	}

	dev_err(fwu->dev, "timeout for POR interrupt\n");
	return -ETIMEDOUT;
}

static int check_boot_completed(struct max77779_fwupdate *fwu)
{
	int ret;
	unsigned int val;

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_BOOT_CHECK_REG, &val);
	if (ret) {
		dev_err(fwu->dev, "failed to read %02x (%d) in check boot completed\n",
			MAX77779_FG_BOOT_CHECK_REG, ret);
		return ret;
	}

	if ((val & 0xff) != MAX77779_FG_BOOT_CHECK_SUCCESS) {
		dev_err(fwu->dev, "Boot NOT completed successfully: %04x\n", val);
		return -EIO;
	}

	dev_info(fwu->dev, "Boot completed successfully\n");
	return 0;
}

static int max77779_check_timer_refresh(struct max77779_fwupdate *fwu)
{
	int ret;
	unsigned int val0, val1;

	dev_info(fwu->dev, "check for timer refresh\n");

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_Timer, &val0);
	if (ret)
		goto check_timer_error;

	msleep(FW_UPDATE_TIMER_CHECK_INTERVAL_MS);

	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_Timer, &val1);
	if (ret)
		goto check_timer_error;

	if (val1 <= val0) {
		dev_err(fwu->dev, "Timer NOT updating correctly\n");
		return -EIO;
	}

	dev_info(fwu->dev, "Timer updating correctly\n");
	return 0;

check_timer_error:
	dev_err(fwu->dev, "failed to read %02x (%d) in max77779_check_timer_refresh\n",
		MAX77779_FG_Timer, ret);
	return ret;
}

static int max77779_send_command(struct max77779_fwupdate *fwu,
				 enum max77779_fwupdate_cmd cmd)
{
	int ret;

	ret = max77779_external_fg_reg_write(fwu->fg, MAX77779_FG_Command_fw, cmd);
	if (ret)
		dev_err(fwu->dev, "failed to write fg reg %02x (%d) in max77779_send_command\n",
			MAX77779_FG_Command_fw, ret);

	return ret;
}

static int max77779_trigger_interrupt(struct max77779_fwupdate *fwu,
				      enum max77779_fwupdate_intr intr)
{
	int ret;

	ret = max77779_external_pmic_reg_write(fwu->pmic, MAX77779_FG_AP_DATAOUT_OPCODE,
					       (unsigned int)intr);
	if (ret)
		dev_err(fwu->dev, "failed to write pmic reg %02x (%d) in trigger_interrupt\n",
			MAX77779_FG_AP_DATAOUT_OPCODE, ret);

	return ret;
}

static int max77779_get_firmware_version(struct max77779_fwupdate *fwu,
					 struct max77779_version_info *ver)
{
	int ret;
	unsigned int major, minor;

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_FG_FW_REV, &major);
	if (ret) {
		dev_err(fwu->dev, "failed to read pmic reg %02x (%d) in read firmware version\n",
			MAX77779_FG_FW_REV, ret);
		goto max77779_get_firmware_version_done;
	}

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_FG_FW_SUB_REV, &minor);
	if (ret)
		dev_err(fwu->dev, "failed to read pmic reg %02x (%d) in read firmware version\n",
			MAX77779_FG_FW_SUB_REV, ret);

	ver->major = (u8)major;
	ver->minor = (u8)minor;

max77779_get_firmware_version_done:
	return ret;
}


static int max77779_change_fg_lock(struct max77779_fwupdate *fwu,
				   const enum max77779_fwupdate_fg_lock st)
{
	int ret;

	/*
	 * change FG's lock status
	 * - write status_value 2 times to MAX77779_FG_USR
	 */
	ret = max77779_external_fg_reg_write(fwu->fg, MAX77779_FG_USR, st);
	if (ret == 0)
		ret = max77779_external_fg_reg_write(fwu->fg, MAX77779_FG_USR, st);

	if (ret)
		dev_err(fwu->dev, "failed to write fg reg %02x (%d) in change lock status\n",
			MAX77779_FG_USR, ret);

	return ret;
}

static int max77779_copy_to_vimon_mem(struct max77779_fwupdate *fwu,
				      const u16 page, const u8* data,
				      const size_t data_len)
{
	int ret;
	size_t idx;

	ret = max77779_external_vimon_reg_write(fwu->vimon, MAX77779_BVIM_PAGE_CTRL,
						(u8*)&page, 2);
	if (ret) {
		dev_err(fwu->dev, "failed to set page %x (%d)\n", page, ret);
		goto max77779_copy_to_vimon_mem_done;
	}

	ret = max77779_external_vimon_reg_write(fwu->vimon, MAX77779_VIMON_MEM_BASE_ADDR,
						data, data_len);
	if (ret) {
		dev_err(fwu->dev,
			"failed to write data to vimon's memory page %x (%d)\n",
			page, ret);
		goto max77779_copy_to_vimon_mem_done;
	}

	/* check data whether it's actually written to vimon's memory */
	ret = max77779_external_vimon_reg_read(fwu->vimon, MAX77779_VIMON_MEM_BASE_ADDR,
					       fwu->scratch_buffer, data_len);
	if (ret) {
		dev_err(fwu->dev, "failed to read data from vimon's memory page %x (%d)\n",
			page, ret);
		goto max77779_copy_to_vimon_mem_done;
	}

	for (idx=0; idx<data_len; idx+=2) {
		if (*(u16*)&data[idx] != *(u16*)&fwu->scratch_buffer[idx]) {
			dev_err(fwu->dev, "data mismatch at page %x offset %zu\n", page, idx);
			break;
		}
	}

max77779_copy_to_vimon_mem_done:
	return ret;
}

static int max77779_load_fw_binary(struct max77779_fwupdate *fwu,
				   const u8* data, const size_t data_len)
{
	u16 page;
	int ret;
	size_t cp_len;
	ssize_t remains = (ssize_t)data_len;

	/* clear all vimon's memory */
	for (page = 0; page < 4; page++) {
		if (page == 3)
			cp_len = MAX77779_VIMON_PG3_SIZE;
		else
			cp_len = MAX77779_VIMON_PG_SIZE;

		ret = max77779_copy_to_vimon_mem(fwu, page, fwu->zero_filled_buffer, cp_len);
		if (ret) {
			dev_err(fwu->dev, "failed load binary in erase data\n");
			goto max77779_load_fw_binary_done;
		}
	}

	/* copy firmware binary to vimon's memory */
	for (page = 0; (page < 4) && remains > 0; page++) {
		if (remains > MAX77779_VIMON_PG_SIZE)
			cp_len = MAX77779_VIMON_PG_SIZE;
		else
			cp_len = remains;

		ret = max77779_copy_to_vimon_mem(fwu, page, data, cp_len);
		if (ret) {
			dev_err(fwu->dev, "failed load binary in copy data in page %d\n",
				(int)page);
			goto max77779_load_fw_binary_done;
		}

		data += MAX77779_VIMON_PG_SIZE;
		remains -= MAX77779_VIMON_PG_SIZE;
	}

max77779_load_fw_binary_done:
	return ret;
}

static inline int max77779_clear_state_for_update(struct max77779_fwupdate *fwu)
{
	int ret;
	unsigned int val;

	/* clear POR bits*/
	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_FG_INT_STS, &val);
	if (ret) {
		dev_err(fwu->dev,
			"failed to read reg %02x (%d) in max77779_clear_state_for_update\n",
			MAX77779_FG_FG_INT_STS, ret);
		return ret;
	}

	ret = max77779_external_fg_reg_write(fwu->fg, MAX77779_FG_FG_INT_STS, val);
	if (ret) {
		dev_err(fwu->dev,
			"failed to write reg %02x (%d) in max77779_clear_state_for_update\n",
			MAX77779_FG_FG_INT_STS, ret);
		return ret;
	}

	/* clear commands */
	max77779_send_command(fwu, MAX77779_CMD_CLEAR_ALL);

	/* corner case, handles commands still present in AP_REQUEST_OPCODE */
	ret = max77779_trigger_interrupt(fwu, MAX77779_INTR_CLEAR);
	return ret;
}

static inline int max77779_session_start(struct max77779_fwupdate *fwu,
					 const u8* fw_binary_data,
					 const char* name)
{
	int ret;

	dev_info(fwu->dev, "[%s] begins\n", name);

	ret = max77779_trigger_interrupt(fwu, MAX77779_INTR_SESSION_START);
	MAX77779_ABORT_ON_ERROR(ret, name, "interrupt trigger");

	max77779_wait_cpu_reset(fwu);

	ret = max77779_load_fw_binary(fwu, fw_binary_data, MAX77779_FW_IMG_SZ_PACKET);
	MAX77779_ABORT_ON_ERROR(ret, name, "load_binary");

	ret = max77779_wait_fw_update(fwu);
	MAX77779_ABORT_ON_ERROR(ret, name, "max77779_wait_fw_update");

	dev_info(fwu->dev, "[%s] ends\n", name);

	return ret;
}

static int max77779_transfer_binary_data(struct max77779_fwupdate *fwu,
					 const u8* fw_binary_data,
					 const size_t data_size,
					 enum max77779_fwupdate_intr intr,
					 const char* name)
{
	int ret;
	ssize_t frame_len;
	ssize_t remains = (ssize_t)data_size;

	dev_info(fwu->dev, "[%s] begins\n", name);

	while (remains > 0) {
		frame_len = (remains > MAX77779_FW_IMG_SZ_FRAME)?
			    MAX77779_FW_IMG_SZ_FRAME : remains;

		ret = max77779_load_fw_binary(fwu, fw_binary_data, frame_len);
		MAX77779_ABORT_ON_ERROR(ret, name, "load_binary");

		msleep(FW_UPDATE_WAIT_LOAD_BIN_MS);

		ret = max77779_trigger_interrupt(fwu, intr);
		MAX77779_ABORT_ON_ERROR(ret, name, "max77779_trigger_interrupt");

		ret = max77779_wait_fw_update(fwu);
		MAX77779_ABORT_ON_ERROR(ret, name, "max77779_wait_fw_update");

		fw_binary_data += frame_len;
		remains -= frame_len;

		dev_info(fwu->dev, "transferred data (%zu/%zu)\n", (data_size - remains),
			 data_size);
	}

	dev_info(fwu->dev, "[%s] ends\n", name);

	return ret;
}

/* TODO: b/303731272 condtion check */
static int max77779_can_update(struct max77779_fwupdate *fwu, struct max77779_version_info* target)
{
	/* Is this device ellgabie to update firmware? */
	if (!fwu->can_update)
		return -EACCES;

	/* check version */
	if (target->major < fwu->v_cur.major ||
	    (target->major == fwu->v_cur.major && target->minor <= fwu->v_cur.minor))
		return -EINVAL;

	return 0;
}

static int max77779_fwl_prepare(struct max77779_fwupdate *fwu,
				const u8 *data, u32 size)
{
	int ret;
	size_t data_frame_size;

	fwu->zero_filled_buffer = kzalloc(MAX77779_VIMON_PG_SIZE, GFP_KERNEL);
	fwu->scratch_buffer = kmalloc(MAX77779_VIMON_PG_SIZE, GFP_KERNEL);
	if (!fwu->zero_filled_buffer || !fwu->scratch_buffer) {
		dev_err(fwu->dev, "failed to allocate temporay work buffer\n");
		return -ENOMEM;
	}

	dev_info(fwu->dev, "prepare firmware update (image size: %d)\n", size);

	data_frame_size = MAX77779_GET_DATA_FRAME_SIZE(size);
	if (data_frame_size % MAX77779_FW_IMG_SZ_PACKET) {
		dev_err(fwu->dev, "incorrect image size (data section size: %zu)\n",
			data_frame_size);
		return -EINVAL;
	}
	fwu->data_frame_size = data_frame_size;
	fwu->op_st = (u8)FGST_FWUPDATE;

	ret = gbms_storage_write(GBMS_TAG_FGST, &fwu->op_st, sizeof(fwu->op_st));
	if (ret != sizeof(fwu->op_st)) {
		fwu->op_st = (u8)FGST_ERR_READTAG;
		dev_err(fwu->dev, "failed to update eeprom:GBMS_TAG_FGST (%d)\n", ret);
		return -EIO;
	}

	ret = max77779_get_firmware_version(fwu, &fwu->v_cur);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read version information\n");

	ret = max77779_fg_enable_firmware_update(fwu->fg, true);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to set fg_enable_firmware_update");

	dev_info(fwu->dev, "the current installed firmware version %u.%u\n",
		 (unsigned int)fwu->v_cur.major,
		 (unsigned int)fwu->v_cur.minor);

	ret = max77779_change_fg_lock(fwu, FG_ST_UNLOCK_ALL_SECTION);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed unlock FG");

	ret = max77779_clear_state_for_update(fwu);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed clear command / POR  interrupt");

	/* TODO: 308211733 need to prevent mode change  */
	ret = max77779_chg_mode_write(fwu->chg, MAX77779_CHGR_MODE_BOOST_ON);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to set mode BOOST_ON");

	/* do unlock again */
	ret = max77779_change_fg_lock(fwu, FG_ST_UNLOCK_ALL_SECTION);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed unlock FG");

	ret = max77779_send_command(fwu, MAX77779_CMD_REBOOT_RISCV);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed send command CMD_REBOOT_RISCV");

	/* wait_riscv_reboot may be timedout even following update will be fine */
	max77779_wait_riscv_reboot(fwu);

	return ret;
}

/* TODO: b/303132973 - consider: "offset" */
static int max77779_fwl_write(struct max77779_fwupdate *fwu, const u8 *fw_binary_data, u32 offset,
			      u32 size, u32 *written)
{
	int ret;
	unsigned int crc;

	dev_info(fwu->dev, "perform firmware update\n");

	/* skip header*/
	fw_binary_data += MAX77779_FW_IMG_SZ_HEADER;
	*written += MAX77779_FW_IMG_SZ_HEADER;

	/* Session Start */
	ret = max77779_session_start(fwu, fw_binary_data, "Session Start");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "Session Start");

	fw_binary_data += MAX77779_FW_IMG_SZ_PACKET;
	*written += MAX77779_FW_IMG_SZ_PACKET;

	/* Transfer Frame */
	ret = max77779_transfer_binary_data(fwu, fw_binary_data, fwu->data_frame_size,
					    MAX77779_INTR_TRANSFER_FRAMES, "Transfer Frame");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "Transfer Frame");

	fw_binary_data += fwu->data_frame_size;
	*written += fwu->data_frame_size;

	/* App Valid: CRC check */
	ret = max77779_transfer_binary_data(fwu, fw_binary_data, MAX77779_FW_IMG_SZ_PACKET,
					    MAX77779_INTR_APP_VALID, "App Valid");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "App Valid");

	fw_binary_data += MAX77779_FW_IMG_SZ_PACKET;
	*written += MAX77779_FW_IMG_SZ_PACKET;

	fwu->crc_val = 0;
	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_FG_AP_DATAIN0, &crc);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read crc information");
	fwu->crc_val = (u8)crc;

	ret = max77779_external_pmic_reg_read(fwu->pmic, MAX77779_FG_AP_DATAIN1, &crc);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to read crc information");
	fwu->crc_val |= (crc << 8);


	/* Session End */
	ret = max77779_transfer_binary_data(fwu, fw_binary_data, MAX77779_FW_IMG_SZ_PACKET,
					    MAX77779_INTR_SESSION_END, "Session End");
	MAX77779_ABORT_ON_ERROR(ret, __func__, "Session End");

	*written += MAX77779_FW_IMG_SZ_PACKET;

	return ret;
}

static int max77779_fwl_poll_complete(struct max77779_fwupdate *fwu)
{
	int ret;

	dev_info(fwu->dev, "max77779_fwl_poll_complete\n");

	/* check firmware update status */
	dev_info(fwu->dev, "firmware update CRC: %x\n", fwu->crc_val);
	if (fwu->crc_val ==0) {
		dev_info(fwu->dev, "bad CRC value returns");
		return -EIO;
	}

	ret = max77779_chg_mode_write(fwu->chg, MAX77779_CHGR_MODE_BUCK_ON);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to set MAX77779_CHGR_MODE_BUCK_ON");

	max77779_wait_riscv_reboot(fwu);

	ret = max77779_get_firmware_version(fwu, &fwu->v_new);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed to get firmware version\n");
	dev_info(fwu->dev, "updated firmware version: %u.%u\n",
		 fwu->v_new.major, fwu->v_new.minor);

	ret = check_boot_completed(fwu);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed on check_boot_completed\n");

	ret = max77779_check_timer_refresh(fwu);
	MAX77779_ABORT_ON_ERROR(ret, __func__, "failed on max77779_check_timer_refresh\n");

	fwu->op_st = FGST_NORMAL;

	return ret;
}

static void max77779_fwl_cleanup(struct max77779_fwupdate *fwu)
{
	int ret;

	dev_info(fwu->dev, "max77779_fwl_cleanup\n");

	if (fwu->zero_filled_buffer)
		kfree(fwu->zero_filled_buffer);
	if (fwu->scratch_buffer)
		kfree(fwu->scratch_buffer);

	ret = max77779_fg_enable_firmware_update(fwu->fg, false);
	if (ret)
		dev_err(fwu->dev, "failed to set max77779_fg_enable_firmware_update (%d)\n", ret);

	if (fwu->op_st == FGST_NORMAL) {
		ret = gbms_storage_write(GBMS_TAG_FGST, &fwu->op_st, sizeof(fwu->op_st));
		if (ret != sizeof(fwu->op_st))
			dev_err(fwu->dev, "failed to update eeprom:GBMS_TAG_FGST (%d)\n", ret);
	}
}

static inline int perform_firmware_update(struct max77779_fwupdate *fwu, const char* data,
					  const size_t count)
{
	u32 written = 0;
	int ret;

	ret = max77779_fwl_prepare(fwu, data, count);
	if (ret)
		goto perform_firmware_update_cleanup;

	ret = max77779_fwl_write(fwu, data, 0, count, &written);
	if (ret || written != count)
		goto perform_firmware_update_cleanup;

	max77779_fwl_poll_complete(fwu);

perform_firmware_update_cleanup:
	max77779_fwl_cleanup(fwu);

	return ret;
}

static void firmware_update_work(struct work_struct* work)
{
	int ret;
	unsigned int val;
	struct max77779_version_info target_version;
	const struct firmware* fw_data;
	struct max77779_fwupdate *fwu = NULL;

	fwu = container_of(work, struct max77779_fwupdate, update_work.work);
	if (!fwu)
		return;

	/* check current voltage */
	ret = max77779_external_fg_reg_read(fwu->fg, MAX77779_FG_AvgVCell, &val);
	if (ret || reg_to_micro_volt(val) < fwu->minimum_voltage) {
		schedule_delayed_work(&fwu->update_work,
				      msecs_to_jiffies(FW_UPDATE_CONDITION_CHECK_INTERVAL_MS));
		return;
	}

	ret = request_firmware(&fw_data, fwu->fw_name, fwu->dev);
	if (ret) {
		dev_err(fwu->dev, "fails on request_firmware %d\n", ret);
		goto firmware_update_work_cleanup;
	}

	target_version.major = fw_data->data[MAX77779_OFFSET_VER_MAJOR];
	target_version.minor = fw_data->data[MAX77779_OFFSET_VER_MINOR];

	ret = max77779_can_update(fwu, &target_version);
	if (ret) {
		dev_err(fwu->dev, "can not update firmware %d\n", ret);
		goto firmware_update_work_cleanup;
	}

	ret = perform_firmware_update(fwu, fw_data->data, fw_data->size);
	if (ret)
		dev_err(fwu->dev, "firmware update failed %d\n", ret);

firmware_update_work_cleanup:
	release_firmware(fw_data);
}

/*
 * expects firmware_name doesn't contain newline, for example
 *  - echo -n "firmware_xxxx.bin" > update_firmware
 */
static ssize_t trigger_update_firmware(struct device *dev,
				       struct device_attribute *attr,
				       const char *firmware_name, size_t count)
{
	struct max77779_fwupdate *fwu = dev_get_drvdata(dev);

	if (!fwu)
		return -EAGAIN;

	if (!fwu->can_update) {
		dev_err(fwu->dev, "not allowed to update firmware\n");
		return -EACCES;
	}

	if (count > 1)
		strncpy(fwu->fw_name, firmware_name, MAX77779_FW_UPDATE_STRING_MAX - 1);
	else
		scnprintf(fwu->fw_name, MAX77779_FW_UPDATE_STRING_MAX, "%s_%d_%d.bin",
			  MAX77779_FIRMWARE_BINARY_PREFIX, (int)fwu->minimum.major,
			  (int)fwu->minimum.minor);

	schedule_delayed_work(&fwu->update_work,
			      msecs_to_jiffies(FW_UPDATE_TIMER_CHECK_INTERVAL_MS));

	return count;
}

static DEVICE_ATTR(update_firmware, 0660, NULL, trigger_update_firmware);

static int debug_enable_update(void *data, u64 val)
{
	struct max77779_fwupdate *fwu = data;

	if (fwu)
		fwu->can_update = val != 0;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_enable_update_fops, NULL, debug_enable_update, "%llu\n");

/*
 * Using the same pattern as FW_LOADER
 *  echo 1 > loading
 *  cat FW_IMG > data
 *  echo 0 > loading
 */
static int debug_update_firmware_loading(void *data, u64 val)
{
	struct max77779_fwupdate *fwu = data;
	int ret = 0;

	if (!fwu)
		return -EAGAIN;

	if (val) {
		if (!fwu->debug_image.data)
			fwu->debug_image.data = kzalloc(FW_UPDATE_MAXIMUM_PAGE_SIZE, GFP_KERNEL);

		if (!fwu->debug_image.data)
			return -ENOMEM;

		fwu->debug_image.size = 0;
	} else {
		if (fwu->debug_image.size > 0)
			ret = perform_firmware_update(fwu, fwu->debug_image.data, fwu->debug_image.size);

		if (fwu->debug_image.data) {
			kfree(fwu->debug_image.data);
			fwu->debug_image.data = NULL;
			fwu->debug_image.size = 0;
		}
	}

	fwu->can_update = (val != 0);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_update_firmware_loading_fops, NULL, debug_update_firmware_loading,
			"%llu\n");


static ssize_t debug_update_firmware_data(struct file *filp, const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct max77779_fwupdate *fwu;
	ssize_t ret;

	fwu = (struct max77779_fwupdate *)filp->private_data;
	if (!fwu)
		return -EAGAIN;

	if (!fwu->debug_image.data)
		return -EINVAL;

	if ((FW_UPDATE_MAXIMUM_PAGE_SIZE - fwu->debug_image.size) < count)
		return -EFBIG;

	ret = simple_write_to_buffer(fwu->debug_image.data, FW_UPDATE_MAXIMUM_PAGE_SIZE, ppos,
				     user_buf, count);

	if (ret >= 0)
		fwu->debug_image.size += ret;
	else
		fwu->debug_image.size = -EINVAL;

	return ret;
}

BATTERY_DEBUG_ATTRIBUTE(debug_update_firmware_data_fops, NULL, debug_update_firmware_data);


static int max77779_fwupdate_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct dentry *de;
	struct max77779_fwupdate *fwu;

	fwu = devm_kzalloc(&pdev->dev, sizeof(*fwu), GFP_KERNEL);
	if (!fwu)
		return -ENOMEM;

	fwu->dev = &pdev->dev;
	platform_set_drvdata(pdev, fwu);

	ret = max77779_fwupdate_init(fwu);
	if (ret)
		dev_err(fwu->dev, "error to set max77779_fwupdate\n");

	INIT_DELAYED_WORK(&fwu->update_work, firmware_update_work);

	ret = max77779_get_firmware_version(fwu, &fwu->v_cur);
	if (ret)
		dev_err(fwu->dev, "failed to read version information\n");

	ret = device_create_file(fwu->dev, &dev_attr_update_firmware);
	if (ret != 0) {
		pr_err("Failed to create update_firmware files, ret=%d\n", ret);
		return ret;
	}

	de = debugfs_create_dir("max77779_fwupdate", 0);
	if (!de)
		return 0;

	debugfs_create_file("enable_update", 0400, de, fwu, &debug_enable_update_fops);
	debugfs_create_file("loading", 0400, de, fwu, &debug_update_firmware_loading_fops);
	debugfs_create_file("data", 0444, de, fwu, &debug_update_firmware_data_fops);

	fwu->de = de;

	return ret;
}

static int max77779_fwupdate_remove(struct platform_device *pdev)
{
	struct max77779_fwupdate *fwu = platform_get_drvdata(pdev);
	if (!fwu)
		return 0;

	if (fwu->debug_image.data)
		kfree(fwu->debug_image.data);

	if (fwu->de)
		debugfs_remove(fwu->de);

	return 0;
}

static const struct of_device_id max77779_fwupdate_of_match[] = {
	{.compatible = "maxim,max77779fwu"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_fwupdate_of_match);


static const struct platform_device_id max77779_fwupdate_id[] = {
	{"max77779_fwupdate", 0},
	{}
};
MODULE_DEVICE_TABLE(platform, max77779_fwupdate_id);

static struct platform_driver max77779_fwupdate_driver = {
	.driver = {
		.name = "max77779_fwupdate",
		.owner = THIS_MODULE,
		.of_match_table = max77779_fwupdate_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		},
	.id_table = max77779_fwupdate_id,
	.probe = max77779_fwupdate_probe,
	.remove = max77779_fwupdate_remove,
};

module_platform_driver(max77779_fwupdate_driver);

MODULE_DESCRIPTION("MAX77779 Firmware Update Driver");
MODULE_AUTHOR("Chungro Lee <chungro@google.com>");
MODULE_LICENSE("GPL");