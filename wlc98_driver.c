// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ST WLC98
 * Based on sample linux driver for ST WLC98 from ST
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <asm/uaccess.h>

#include "wlc98_driver.h"

static int wlc68_i2c_read(struct i2c_client *client, void *cmd, u8 cmd_length,
			void *read_data, u8 read_count)
{
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].buf = cmd;
	msg[0].len = cmd_length;
	msg[0].flags = 0;

	msg[1].addr = client->addr;
	msg[1].buf = read_data;
	msg[1].len = read_count;
	msg[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "i2c transfer failed! err: %d\n", ret);
		return ret;
	}

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "[WLC] WR-W: ", DUMP_PREFIX_NONE,
			16, 1, cmd, cmd_length, 0);

	print_hex_dump(KERN_ERR, "[WLC] WR-R: ", DUMP_PREFIX_NONE,
			16, 1, read_data, read_count, 0);
#endif

	return 0;
}

static int wlc68_i2c_write(struct i2c_client *client, void *cmd, u8 cmd_length)
{
	struct i2c_msg msg[1];
	int ret;

	msg[0].addr = client->addr;
	msg[0].buf = cmd;
	msg[0].len = cmd_length;
	msg[0].flags = 0;

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "[WLC] W: ", DUMP_PREFIX_NONE,
			16, 1, cmd, cmd_length, 0);
#endif

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "i2c transfer failed! err: %d\n", ret);
		return ret;
	}

	return ret;
}

static int fw_i2c_read(struct i2c_client *client, u16 addr, u8 *read_buff, u8 read_count)
{
	u16 cmd;
	int err;

	cmd = cpu_to_be16(addr);
	err = wlc68_i2c_read(client, &cmd, 2, read_buff, read_count);
	if (err) {
		return E_BUS_WR;
	}

	return 0;
}

static int hw_i2c_write(struct i2c_client *client, u32 addr, u8 *data, u32 data_length)
{
	u8 cmd[MAX_CMD_SIZE];
	int err;

	cmd[0] = OPCODE_WRITE;
	addr = cpu_to_be32(addr);
	memcpy(&cmd[1], &addr, 4);
	memcpy(&cmd[5], data, data_length);

	err = wlc68_i2c_write(client, cmd, (5 + data_length));
	if (err) {
		return E_BUS_W;
	}

	return 0;
}

static int hw_i2c_read(struct i2c_client *client, u32 addr, u8 *read_buff, int read_count)
{
	u8 cmd[5];
	int err;

	cmd[0] = OPCODE_WRITE;
	addr = cpu_to_be32(addr);
	memcpy(&cmd[1], &addr, 4);

	err = wlc68_i2c_read(client, cmd, 5 , read_buff, read_count);
	if (err) {
		return E_BUS_WR;
	}

	return 0;
}

static int get_chip_info(struct i2c_client *client, struct wlc_chip_info *info)
{
	u8 read_buff[14];
	int err;

	err = fw_i2c_read(client, FWREG_CHIP_ID_REG, read_buff, 14);
	if (err) {
		return E_BUS_R;
	}

	info->chip_id = *((u16 *)read_buff);
	info->chip_revision = read_buff[2];
	info->customer_id = read_buff[3];
	info->project_id = *((u16 *)(read_buff + 4));
	info->ftp_patch_id = *((u16 *)(read_buff + 6));
	info->ram_patch_id = *((u16 *)(read_buff + 8));
	info->config_id = *((u16 *)(read_buff + 10));
	info->pe_id = *((u16 *)(read_buff + 12));

	err = hw_i2c_read(client, HWREG_HW_VER_ADDR, read_buff, 1);
	if (err){
		return E_BUS_R;
	}

	info->cut_id = read_buff[0];

	dev_info(&client->dev, "ChipID: %04X Chip Revision: %02X CustomerID: %02X\n",
		info->chip_id, info->chip_revision, info->customer_id);
	dev_info(&client->dev, "RomID: %04X FTPPatchID: %04X RAMPatchID: %04X CFG: %04X PE: %04X\n",
		info->project_id, info->ftp_patch_id,
		info->ram_patch_id, info->config_id, info->pe_id);

	return 0;
}

static ssize_t chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct wlc_ts_info *info = dev_get_drvdata(dev);

	ret = get_chip_info(info->client, &info->chip_info);
	if (ret) {
		dev_err(dev, "Error while getting chip info\n");
		return ret;
	}

	ret = scnprintf(buf, PAGE_SIZE, "Chip Id : %#04X, Chip Revision : %#02X\n",
			info->chip_info.chip_id, info->chip_info.chip_revision);
	return ret;
}

static DEVICE_ATTR_RO(chip_info);

static struct attribute *wlc_attr_group[] = {
	&dev_attr_chip_info.attr,
	NULL
};

static bool wlc98_is_reg(struct device *dev, unsigned int reg)
{
	switch(reg) {
	case 0x0 ... FWREG_MAX_REGISTER:
		return true;
	default:
		return false;
	}
}

static struct regmap_config wlc98_regmap = {
	.name		= "wlc98-mains",
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= FWREG_MAX_REGISTER,
	.readable_reg = wlc98_is_reg,
	.volatile_reg = wlc98_is_reg,
};

static int read_reg(void *data, u64 *val)
{
	struct wlc_ts_info *chip = data;
	int rc;
	unsigned int temp;

	if (chip->debug_address >= HWREG_HW_VER_ADDR) {
		u8 read_buff[1];
		rc = hw_i2c_read(chip->client, chip->debug_address, read_buff, 1);
		if (rc) {
			dev_err(chip->dev, "Error reading address: %04X\n",
				chip->debug_address);
			return E_BUS_R;
		}

		*val = read_buff[0];
	} else {
		rc = regmap_read(chip->regmap, chip->debug_address, &temp);
		if (rc) {
			dev_err(chip->dev, "Couldn't read reg %x rc = %d\n",
				chip->debug_address, rc);
			return -EAGAIN;
		}
		*val = temp;
	}
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct wlc_ts_info *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;

	if (chip->debug_address >= HWREG_HW_VER_ADDR) {
		rc = hw_i2c_write(chip->client, chip->debug_address, &temp, 1);
		if (rc) {
			dev_err(chip->dev, "Error writing address: %04X with value %01X\n",
				chip->debug_address, temp);
			return -EAGAIN;
		}
	} else {
		rc = regmap_write(chip->regmap, chip->debug_address, temp);
		if (rc) {
			dev_err(chip->dev, "Couldn't write %#02x to %#02x rc = %d\n",
				temp, chip->debug_address, rc);
			return -EAGAIN;
		}
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "%#02llx\n");

static int wlc98_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0;
	struct wlc_ts_info *info = NULL;

	info = devm_kzalloc(&client->dev, sizeof(struct wlc_ts_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	info->client = client;
	info->dev = &client->dev;
	dev_set_drvdata(info->dev, info);

	error = get_chip_info(client, &info->chip_info);
	if (error) {
		dev_err(info->dev, "ERROR: Cannot read chip info!\n");
		error = -ENODEV;
		goto probe_exit_error0;
	}

	/* sysfs stuff */
	info->attrs.attrs = wlc_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		dev_err(info->dev, "ERROR: Cannot create sysfs structure!\n");
		error = -ENODEV;
		goto probe_exit_error0;
	}

	info->regmap = devm_regmap_init_i2c(client, &wlc98_regmap);
	if (IS_ERR(info->regmap)) {
		error = -EINVAL;
		goto probe_exit_error1;
	}

	info->debug_root = debugfs_create_dir("charger-stwlc98", NULL);
	if (IS_ERR_OR_NULL(info->debug_root)) {
		dev_err(info->dev, "Couldn't create debug dir\n");
		error = -ENOENT;
		goto probe_exit_error1;
	}

	debugfs_create_file("data", 0644, info->debug_root, info, &register_debug_ops);
	debugfs_create_x32("address", 0644, info->debug_root, &info->debug_address);

	dev_info(info->dev, "Driver Version %s\n", WLC_DRV_VERSION);
	return 0;

probe_exit_error1:
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
probe_exit_error0:
	devm_kfree(info->dev, info);
	dev_err(&client->dev, "probe failed error: %d\n", error);
	return error;
}

static void wlc98_remove(struct i2c_client *client)
{
	struct wlc_ts_info *info = dev_get_drvdata(&(client->dev));
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
	devm_kfree(info->dev, info);
}

static const struct i2c_device_id id[] = {
	{ "STM_WLC", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, id);

#ifdef CONFIG_OF
static struct of_device_id stwlc98_match_table[] = {
	{ .compatible = "st_wlc98",},
	{ },
};
#endif

static struct i2c_driver driver = {
	.probe		= wlc98_probe,
	.remove		= wlc98_remove,
	.id_table	= id,
	.driver = {
		.name   = "STM_WLC",
#ifdef CONFIG_OF
		.of_match_table = stwlc98_match_table,
#endif
	},
};

static int __init stwlc98_wlc_init(void)
{
	return i2c_add_driver(&driver);
}

static void __exit stwlc98_wlc_exit(void)
{
	i2c_del_driver(&driver);
}

late_initcall(stwlc98_wlc_init);
module_exit(stwlc98_wlc_exit);

MODULE_AUTHOR("Prasanna Prapancham <prapancham@google.com>");
MODULE_DESCRIPTION("STWLC98 Wireless Charger Driver");
MODULE_LICENSE("GPL");
