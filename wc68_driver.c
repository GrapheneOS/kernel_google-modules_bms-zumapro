// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ST WC68 Direct charger
 * Based on sample linux driver for ST WLC98 from ST
 * and existing PCA9468 driver
 */


#include <linux/err.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#include "wc68_regs.h"
#include "wc68_charger.h"

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

/* Timer definition */
#define WC68_VBATMIN_CHECK_T	1000	/* 1000ms */
#define WC68_CCMODE_CHECK1_T	5000	/* 10000ms -> 500ms */
#define WC68_CCMODE_CHECK2_T	5000	/* 5000ms */
#define WC68_CVMODE_CHECK_T 	10000	/* 10000ms */
#define WC68_ENABLE_DELAY_T	150	/* 150ms */
#define WC68_CVMODE_CHECK2_T	1000	/* 1000ms */
#define WC68_ENABLE_WLC_DELAY_T	300	/* 300ms */

/* Battery Threshold */
#define WC68_DC_VBAT_MIN		3400000 /* uV */
/* Input Current Limit default value */
#define WC68_IIN_CFG_DFT		2500000 /* uA*/
/* Charging Float Voltage default value */
#define WC68_VFLOAT_DFT		4350000	/* uV */
/* Charging Sub Float Voltage default value */
#define WC68_VFLOAT_SUB_DFT		5000000	/* 5000000uV */
/* Charging Float Voltage max voltage for comp */
#define WC68_COMP_VFLOAT_MAX		4450000	/* uV */

/* Charging Done Condition */
#define WC68_IIN_DONE_DFT	500000		/* uA */
/* parallel charging done conditoin */
#define WC68_IIN_P_DONE	1000000		/* uA */
/* Parallel charging default threshold */
#define WC68_IIN_P_TH_DFT	4000000		/* uA */
/* Single charging default threshold */
#define WC68_IIN_S_TH_DFT	10000000	/* uA */

/* Maximum TA voltage threshold */
#define WC68_TA_MAX_VOL		9800000 /* uV */
/* Maximum TA current threshold, set to max(cc_max) / 2 */
#define WC68_TA_MAX_CUR		2600000	 /* uA */
/* Minimum TA current threshold */
#define WC68_TA_MIN_CUR		1000000	/* uA - PPS minimum current */

/* Minimum TA voltage threshold in Preset mode */
#define WC68_TA_MIN_VOL_PRESET	8000000	/* uV */
/* TA voltage threshold starting Adjust CC mode */
#define WC68_TA_MIN_VOL_CCADJ	8500000	/* 8000000uV --> 8500000uV */

#define WC68_TA_VOL_PRE_OFFSET	500000	 /* uV */
/* Adjust CC mode TA voltage step */
#define WC68_TA_VOL_STEP_ADJ_CC	40000	/* uV */
/* Pre CV mode TA voltage step */
#define WC68_TA_VOL_STEP_PRE_CV	20000	/* uV */

/* IIN_CC adc offset for accuracy */
#define WC68_IIN_ADC_OFFSET		20000	/* uA */
/* IIN_CC compensation offset */
#define WC68_IIN_CC_COMP_OFFSET	50000	/* uA */
/* IIN_CC compensation offset in Power Limit Mode(Constant Power) TA */
#define WC68_IIN_CC_COMP_OFFSET_CP	20000	/* uA */
/* TA maximum voltage that can support CC in Constant Power Mode */
#define WC68_TA_MAX_VOL_CP		9800000	/* 9760000uV --> 9800000uV */
/* Offset for cc_max / 2 */
#define WC68_IIN_MAX_OFFSET		0


/* maximum retry counter for restarting charging */
#define WC68_MAX_RETRY_CNT		3	/* retries */
/* TA IIN tolerance */
#define WC68_TA_IIN_OFFSET		100000	/* uA */
/* IIN_CC upper protection offset in Power Limit Mode TA */
#define WC68_IIN_CC_UPPER_OFFSET	50000	/* 50mA */

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP		20000	/* uV */
#define PD_MSG_TA_CUR_STEP		50000	/* uA */

/* Maximum WCRX voltage threshold */
#define WC68_WCRX_MAX_VOL		9750000 /* uV */
/* WCRX voltage Step */
#define WCRX_VOL_STEP			100000	/* uV */

#define WC68_OTV_MARGIN		12000	/* uV */

/* irdrop default limit */
#define WC68_IRDROP_LIMIT_CNT	3	/* tiers */
#define WC68_IRDROP_LIMIT_TIER1	-30000	/* uV */
#define WC68_IRDROP_LIMIT_TIER2	-19000	/* uV */
#define WC68_IRDROP_LIMIT_TIER3	0	/* uV */

/* Spread Spectrum default settings */
#define WC68_SC_CLK_DITHER_RATE_DEF	0	/* 25kHz */
#define WC68_SC_CLK_DITHER_LIMIT_DEF	0xF	/* 10% */

#define CBUS_UCP_DFT			400000	/* 400 mV, actual value TBD */

/* Status */
enum {
	STS_MODE_CHG_LOOP,	/* TODO: There is no such thing */
	STS_MODE_VFLT_LOOP,
	STS_MODE_IIN_LOOP,
	STS_MODE_LOOP_INACTIVE,
	STS_MODE_CHG_DONE,
	STS_MODE_VIN_UVLO,
};

/* Timer ID */
enum {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_PRESET_DC,
	TIMER_PRESET_CONFIG,
	TIMER_CHECK_ACTIVE,
	TIMER_ADJUST_CCMODE,
	TIMER_CHECK_CCMODE,
	TIMER_ENTER_CVMODE,
	TIMER_CHECK_CVMODE, /* 8 */
	TIMER_PDMSG_SEND,   /* 9 */
	TIMER_ADJUST_TAVOL,
	TIMER_ADJUST_TACUR,
};


/* TA increment Type */
enum {
	INC_NONE,	/* No increment */
	INC_TA_VOL,	/* TA voltage increment */
	INC_TA_CUR,	/* TA current increment */
};

/* BATT info Type */
enum {
	BATT_CURRENT,
	BATT_VOLTAGE,
};

static int wc68_hw_init(struct wc68_charger *wc68);
static int wc68_irq_init(struct wc68_charger *wc68,
			    struct i2c_client *client);

/* From STWC68 driver ***********************/
static int wc68_i2c_read(struct i2c_client *client, void *cmd, u8 cmd_length,
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
		return -EIO;
	}

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "[WC] WR-W: ", DUMP_PREFIX_NONE,
			16, 1, cmd, cmd_length, 0);

	print_hex_dump(KERN_ERR, "[WC] WR-R: ", DUMP_PREFIX_NONE,
			16, 1, read_data, read_count, 0);
#endif

	return 0;
}

static int wc68_i2c_write(struct i2c_client *client, void *cmd, u8 cmd_length)
{
	struct i2c_msg msg[1];
	int ret;

	msg[0].addr = client->addr;
	msg[0].buf = cmd;
	msg[0].len = cmd_length;
	msg[0].flags = 0;

#ifdef DEBUG
	print_hex_dump(KERN_INFO, "[WC] W: ", DUMP_PREFIX_NONE,
			16, 1, cmd, cmd_length, 0);
#endif

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "i2c transfer failed! err: %d\n", ret);
		return -EIO;
	}

	return 0;
}

static ssize_t chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	u16 read_buff[2];
	u16 cmd = cpu_to_be16(FWREG_CHIP_ID_REG);

	struct wc68_charger *info = dev_get_drvdata(dev);

	ret = wc68_i2c_read(info->client, (u8 *)&cmd, 2, read_buff, 4);
	if (ret) {
		scnprintf(buf, PAGE_SIZE, "%02X\n", ret);
		return -EIO;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%#04X, %#04X\n",
			read_buff[0], read_buff[1]);

	return ret;
}

static DEVICE_ATTR_RO(chip_info);

static struct attribute *wc_attr_group[] = {
	&dev_attr_chip_info.attr,
	NULL
};

static bool wc68_is_reg(u16 reg)
{
	switch(reg) {
	case 0x0 ... FWREG_MAX_REGISTER:
		return true;
	default:
		return false;
	}
}

static int read_reg_wc68(void *data, u64 *val)
{
	struct wc68_charger *chip = data;
	int rc;
	u16 cmd;
	u64 read_val = 0;

	if (chip->debug_count < 1 || chip->debug_count > 8) {
		dev_err(chip->dev, "Invalid count: %d. Valid: 1-8\n", chip->debug_count);
		return -EINVAL;
	}

	if (!wc68_is_reg(chip->debug_address)) {
		dev_err(chip->dev, "Invalid register address: %#04X\n", chip->debug_address);
		return -EINVAL;
	}

	cmd = cpu_to_be16(chip->debug_address);

	rc = wc68_i2c_read(chip->client, &cmd, 2, &read_val, chip->debug_count);
	if (rc) {
		dev_err(chip->dev, "Couldn't read reg %#x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}

	*val = be64_to_cpu(read_val) >> ((8 - chip->debug_count) * 8);
	return 0;
}

static int write_reg_wc68(void *data, u64 val)
{
	struct wc68_charger *chip = data;
	u8 cmd[10];

	if (chip->debug_count < 1 || chip->debug_count > 8) {
		dev_err(chip->dev, "Invalid count: %d. Valid: 1-8\n", chip->debug_count);
		return -EINVAL;
	}

	if (!wc68_is_reg(chip->debug_address)) {
		dev_err(chip->dev, "Invalid register address: %#04X\n", chip->debug_address);
		return -EINVAL;
	}

	cmd[0] = chip->debug_address >> 8;
	cmd[1] = chip->debug_address & 0xFF;

	val = cpu_to_be64(val) >> ((8 - chip->debug_count) * 8);

	memcpy(&cmd[2], &val, chip->debug_count);

	return wc68_i2c_write(chip->client, cmd, 2 + chip->debug_count);
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops_wc68, read_reg_wc68, write_reg_wc68, "%#llx\n");

/* ------------------------------------------------------------------------ */

/* ADC Read function, return uV or uA */
int wc68_read_adc(struct wc68_charger *wc68, u8 adc_ch)
{
	u16 addr;
	u16 raw_adc = 0;
	int conv_adc = -1;
	int ret;

	switch (adc_ch) {
	case ADCCH_VOUT:
		conv_adc = 0;
		break;

	case ADCCH_VIN:
		conv_adc = 0;
		break;

	case ADCCH_VBAT:
		addr = cpu_to_be16(VBAT1_ADC);
		ret = wc68_i2c_read(wc68->client, &addr, 2, &raw_adc, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		conv_adc = raw_adc * VBAT_STEP; /* unit - uV */
		break;

	case ADCCH_IIN:
		addr = cpu_to_be16(IBUS_ADC);
		ret = wc68_i2c_read(wc68->client, &addr, 2, &raw_adc, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		if (raw_adc <= 2047)
			conv_adc = raw_adc * IIN_STEP;
		else if (raw_adc >= 63488)
			conv_adc = (raw_adc - 65536) * IIN_STEP;
		else
			conv_adc = 0;
		break;

	case ADCCH_DIETEMP:
		addr = cpu_to_be16(TDIE_ADC);
		ret = wc68_i2c_read(wc68->client, &addr, 2, &raw_adc, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}

		if (raw_adc <= 2047)
			conv_adc = 371 + (raw_adc + 2048) * DIETEMP_STEP;
		else if (raw_adc >= 63488)
			conv_adc = 371 + (raw_adc - 65536 + 2048) * DIETEMP_STEP;
		else
			conv_adc = 0;
		if (conv_adc > DIETEMP_MAX)
			conv_adc = DIETEMP_MAX;
		else if (conv_adc < DIETEMP_MIN)
			conv_adc = DIETEMP_MIN;
		break;

	case ADCCH_NTC:
		conv_adc = 0;
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	/* if disabled a channel, re-enable it in -> WC68_REG_ADC_CFG */

	dev_dbg(wc68->dev, "%s: adc_ch=%u, raw_adc=%x convert_val=%d\n", __func__,
		 adc_ch, raw_adc, conv_adc);

	return conv_adc;
}

/* v float voltage (5 mV) resolution */
static int wc68_set_vfloat(struct wc68_charger *wc68,
			      unsigned int v_float)
{
	int ret = 0;
	u16 cmd[2];
	u16 val;

	cmd[0] = cpu_to_be16(VBAT_OVP_WARN_THRES);
	cmd[1] = v_float / VFLOAT_STEP;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 4);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error setting vfloat: %d\n", __func__, ret);
		return ret;
	}

	ret = wc68_i2c_read(wc68->client, &cmd[0], 2, &val, 2);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error reading vfloat: %d\n", __func__, ret);
		return ret;
	}

	dev_info(wc68->dev, "%s: v_float=%u, reg: %d (%d) Read back: raw: %d, conv: %d\n",
		__func__, v_float, v_float / VFLOAT_STEP, ret, val, val * VFLOAT_STEP);

	return ret;
}

static int wc68_set_input_current(struct wc68_charger *wc68,
				     unsigned int iin)
{
	int ret = 0, val;
	u16 cmd[2];

	/* round-up and increase one step */
	iin = iin + PD_MSG_TA_CUR_STEP;
	val = WC68_IIN_CFG(iin);

	/* Set IIN_CFG to one step higher */
	val = val + 1;
	if (val > 2040)
		val = 2040; /* maximum value is 7.469A  (7469/3.662) = 2039 */
	cmd[0] = cpu_to_be16(CBUS_OCP_WARN_THRES);
	cmd[1] = val;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 4);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error setting IIN: %d\n", __func__, ret);
	}

	dev_info(wc68->dev, "%s: iin=%d real iin_cfg=%d (%d)\n", __func__,
		 iin, val * WC68_IIN_CFG_STEP, ret);

	return ret;
}

/* Returns the enable or disable value. into 1 or 0. */
static int wc68_get_charging_enabled(struct wc68_charger *wc68)
{
	int ret;
	u16 cmd = cpu_to_be16(SYS_CFG_2);
	u8 val;

	ret = wc68_i2c_read(wc68->client, &cmd, 2, &val, 1);
	if (ret < 0) {
		dev_err(wc68->dev, "Error reading SC_EN err: %d\n", ret);
		return ret;
	}

	return val & SYS_CFG_2_SC_EN;
}


/* b/194346461 ramp down IIN */
static int wc68_wlc_ramp_down_iin(struct wc68_charger *wc68,
				     struct power_supply *wlc_psy)
{
	const int ramp_down_step = WC68_IIN_CFG_STEP;
	int ret = 0, iin;

	if (!wc68->wlc_ramp_out_iin)
		return 0;

	iin = wc68_input_current_limit(wc68);
	for ( ; iin >= WC68_IIN_CFG_MIN; iin -= ramp_down_step) {
		int iin_adc, wlc_iout = -1;

		iin_adc = wc68_read_adc(wc68, ADCCH_IIN);
		if (wlc_psy) {
			union power_supply_propval pro_val;

			ret = power_supply_get_property(wlc_psy,
					POWER_SUPPLY_PROP_ONLINE,
					&pro_val);
			if (ret < 0 || pro_val.intval != PPS_PSY_PROG_ONLINE)
				break;

			ret = power_supply_get_property(wlc_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW,
					&pro_val);
			if (ret == 0)
				wlc_iout = pro_val.intval;
		}

		ret = wc68_set_input_current(wc68, iin);
		if (ret < 0) {
			dev_err(wc68->dev, "%s: ramp down iin=%d (%d)\n", __func__,
				iin, ret);
			break;
		}

		dev_dbg(wc68->dev, "%s: iin_adc=%d, wlc_iout-%d ramp down iin=%d\n",
				__func__, iin_adc, wlc_iout, iin);
		msleep(wc68->wlc_ramp_out_delay);
	}

	return ret;
}

/* b/194346461 ramp down VOUT */
#define WLC_VOUT_CFG_STEP	40000

/* the caller will set to vbatt * 4 */
static int wc68_wlc_ramp_down_vout(struct wc68_charger *wc68,
				struct power_supply *wlc_psy)
{
	const int ramp_down_step = WLC_VOUT_CFG_STEP;
	union power_supply_propval pro_val;
	int vout = 0, vout_target = wc68->wlc_ramp_out_vout_target;
	int ret, vbatt;

	while (true) {
		vbatt = wc68_read_adc(wc68, ADCCH_VBAT);
		if (vbatt <= 0) {
			dev_err(wc68->dev, "%s: invalid vbatt %d\n", __func__, vbatt);
			break;
		}

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&pro_val);
		if (ret < 0) {
			dev_err(wc68->dev, "%s: invalid vout %d\n", __func__, ret);
			break;
		}

		if (!wc68->wlc_ramp_out_vout_target)
			vout_target = vbatt * 4;

		if (!vout)
			vout = pro_val.intval;
		if (vout < vout_target) {
			dev_dbg(wc68->dev, "%s: underflow vout=%d, vbatt=%d (target=%d)\n",
				__func__, vout, vbatt, vout_target);
			return 0;
		}

		pro_val.intval = vout - ramp_down_step;

		dev_dbg(wc68->dev, "%s: vbatt=%d, wlc_vout=%d->%d\n", __func__, vbatt,
			 vout, pro_val.intval);

		ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&pro_val);
		if (ret < 0) {
			dev_err(wc68->dev, "%s: cannot set vout %d\n", __func__, ret);
			break;
		}

		msleep(wc68->wlc_ramp_out_delay);
		vout = pro_val.intval;
	}

	return -EIO;
}

/* call holding mutex_lock(&wc68->lock); */
static int wc68_set_charging(struct wc68_charger *wc68, bool enable)
{
	int ret;

	dev_dbg(wc68->dev, "%s: enable=%d ta_type=%d\n", __func__,  enable, wc68->ta_type);

	if (enable && wc68_get_charging_enabled(wc68) == enable) {
		dev_dbg(wc68->dev, "%s: no op, already enabled\n", __func__);
		return 0;
	}

	if (enable) {
		u16 cmd[2];
		/* Start charging */
		cmd[0] = cpu_to_be16(SYS_CFG_2);
		cmd[1] = SYS_CFG_2_SC_EN;
		ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
		if (ret < 0) {
			dev_err(wc68->dev, "Error enabling SC_EN err: %d\n", ret);
			goto error;
		}
	} else {
		u16 cmd[2];

		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			struct power_supply *wlc_psy;

			wlc_psy = wc68_get_rx_psy(wc68);
			if (wlc_psy) {
				int ret;

				ret = wc68_wlc_ramp_down_iin(wc68, wlc_psy);
				if (ret < 0)
					dev_err(wc68->dev, "cannot ramp out iin (%d)\n", ret);

				ret = wc68_wlc_ramp_down_vout(wc68, wlc_psy);
				if (ret < 0)
					dev_err(wc68->dev, "cannot ramp out vout (%d)\n", ret);
			}
		}

		/* turn off charging */
		cmd[0] = cpu_to_be16(SYS_CFG_2);
		cmd[1] = 0;
		ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
		if (ret < 0) {
			dev_err(wc68->dev, "Error disabling SC_EN err: %d\n", ret);
			goto error;
		}

		/* Disable CBUS_UCP prot for replug detection */
		cmd[0] = cpu_to_be16(PROT_EN_0);
		cmd[1] = 0;
		ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
		if (ret) {
			dev_err(wc68->dev, "Error disabling UCP protection: %d\n", ret);
			goto error;
		}
	}

error:
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int wc68_check_state(u8 val[8], struct wc68_charger *wc68, int loglevel)
{
	int ret;
	u16 cmd;

	cmd = cpu_to_be16(INTR_FLG_0);
	/* Dump register */
	ret = wc68_i2c_read(wc68->client, &cmd, 2, val, 8);
	if (ret < 0)
		return ret;

	logbuffer_prlog(wc68, loglevel,
			"%s: INTR_FLG reg[1]=%#x,[2]=%#x,[3]=%#x,[4]=%#x,[5]=%#x,[6]=%#x,[7]=%#x",
			__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);

	return 0;
}

/* WC68 is not active state  - standby or shutdown */
/* Stop charging in timer_work */
/* return 0 when no error is detected */
static int wc68_check_not_active(struct wc68_charger *wc68)
{
	u8 val[8];
	u32 prot_sts;
	int ret;
	u16 cmd;

	cmd = cpu_to_be16(PROT_STS_0);
	ret = wc68_i2c_read(wc68->client, &cmd, 2, &prot_sts, 4);
	if (ret < 0) {
		dev_err(wc68->dev, "Error reading PROT_STS err: %d\n", ret);
		return ret;
	}

	if (prot_sts != 0) {
		dev_err(wc68->dev, "One or more protections tripped %#08X\n", prot_sts);
		return -EINVAL;
	}

	ret = wc68_check_state(val, wc68, LOGLEVEL_WARNING);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: cannot read state\n", __func__);
		return ret;
	}

	return 0;
}

/* Keep the current charging state, check STS_B again */
/* return 0 if VIN is still present, -EAGAIN if needs to retry, -EINVAL oth */
int wc68_check_standby(struct wc68_charger *wc68)
{
	int ret;
	u16 cmd;
	u8 val;

	cmd = cpu_to_be16(CHARGE_STS);
	ret = wc68_i2c_read(wc68->client, &cmd, 2, &val, 1);
	if (ret) {
		dev_err(wc68->dev, "Error reading CHARGE_STS : %d\n", ret);
		return ret;
	}

	return (val & CHARGE_STS_CHARGING) ? 0 : 1;
}

/*
 * Check Active status, 0 is active (or in RCP), <0 indicates a problem.
 * The function is called from different contexts/functions, errors are fatal
 * (i.e. stop charging) from all contexts except when this is called from
 * wc68_check_active_state().
 *
 * Other contexts:
 * . wc68_charge_adjust_ccmode
 * . wc68_charge_ccmode
 * . wc68_charge_start_cvmode
 * . wc68_charge_cvmode
 *
 * call holding mutex_lock(&wc68->lock)
 */
static int wc68_check_error(struct wc68_charger *wc68)
{
	int ret;
	u16 cmd;
	u8 val;

	cmd = cpu_to_be16(CHARGE_STS);
	ret = wc68_i2c_read(wc68->client, &cmd, 2, &val, 1);
	if (ret < 0)
		goto done;

	/* WC68 is active state */
	if (val & CHARGE_STS_CHARGING) {
		int vbatt;

		/* WC68 is charging */

		/* Check whether the battery voltage is over the minimum */
		vbatt = wc68_read_adc(wc68, ADCCH_VBAT);
		if (vbatt > WC68_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			cmd = cpu_to_be16(INTR_FLG_0);
			ret = wc68_i2c_read(wc68->client, &cmd, 2, &val, 1);
			if (ret < 0) {
				dev_err(wc68->dev, "%s: cannot read INTR_FLG_0 (%d)\n",
					__func__, ret);
			} else if (val & TDIE_OVTP_INTR_MSK) {
				/* Over temperature protection */
				dev_err(wc68->dev, "%s: Device is in temperature regulation\n",
					__func__);
				ret = -EINVAL;
			}
		} else {
			/* Abnormal battery level */
			dev_err(wc68->dev, "%s: Error abnormal battery voltage=%d\n",
				__func__, vbatt);
			ret = -EINVAL;
		}

		dev_dbg(wc68->dev, "%s: Active Status ok=%d (ret=%d)\n", __func__,
			 ret == 0, ret);
		return ret;
	}

	/* not in error but in standby or shutdown */

	ret = wc68_check_not_active(wc68);
	if (ret < 0) {
		/* There was an error, done... */
		goto done;
	}

	if (!wc68_check_standby(wc68)) {
		/* WC68 is in shutdown state */
		dev_err(wc68->dev, "%s: WC68 is in shutdown\n", __func__);
		ret = -EINVAL;
	} else if (wc68->charging_state == DC_STATE_NO_CHARGING) {
		/*
		 * Sometimes battery driver might call set_property function
		 * to stop charging during msleep. At this case, charging
		 * state would change DC_STATE_NO_CHARGING. WC68 should
		 * stop checking RCP condition and exit timer_work
		 */
		dev_err(wc68->dev, "%s: other driver forced stop\n", __func__);
		ret = -EINVAL;
	} else {

		/* Check the RCP condition, T_REVI_DET is 300ms */
		msleep(200);

		/*
		 * return 0 if VIN is still present, -EAGAIN if needs to retry,
		 * -EINVAL on error.
		 */
		ret = wc68_check_standby(wc68);
	}

done:
	dev_dbg(wc68->dev, "%s: Not Active Status=%d\n", __func__, ret);
	return ret;
}

static int wc68_get_iin(struct wc68_charger *wc68, int *iin)
{
	const int offset = 0;
	int temp;

	temp = wc68_read_adc(wc68, ADCCH_IIN);
	if (temp < 0)
		return temp;

	if (temp < offset)
		temp = offset;

	*iin = (temp - offset) * 2;
	return 0;
}

/* only needed for irdrop compensation ane maybe not even that... */
static int wc68_get_batt_info(struct wc68_charger *wc68, int info_type, int *info)
{
	union power_supply_propval val;
	enum power_supply_property psp;
	int ret;

	if (!wc68->batt_psy)
		wc68->batt_psy = power_supply_get_by_name("battery");
	if (!wc68->batt_psy)
		return -EINVAL;

	if (info_type == BATT_CURRENT)
		psp = POWER_SUPPLY_PROP_CURRENT_NOW;
	else
		psp = POWER_SUPPLY_PROP_VOLTAGE_NOW;

	ret = power_supply_get_property(wc68->batt_psy, psp, &val);
	if (ret == 0)
		*info = val.intval;

	return ret;
}

/* only needed for irdrop compensation ane maybe not even that... */
static int wc68_get_ibatt(struct wc68_charger *wc68, int *info)
{
	return wc68_get_batt_info(wc68, BATT_CURRENT, info);
}

static int wc68_get_current_adcs(struct wc68_charger *wc68, int *pibat, int *picn, int *piin)
{
	int rc = wc68_get_ibatt(wc68, pibat);
	if (rc)
		goto error;

	rc = wc68_get_iin(wc68, picn);
	if (rc)
		goto error;

	*piin = wc68_read_adc(wc68, ADCCH_IIN);
	return 0;

error:
	logbuffer_prlog(wc68, LOGLEVEL_ERR, "%s: Error: rc=%d", __func__, rc);
	return rc;
}

static void wc68_prlog_state(struct wc68_charger *wc68, const char *fn)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;
	int vbat = wc68_read_adc(wc68, ADCCH_VBAT);

	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		goto error;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: vbat=%d, iin=%d, iin_cc=%d, icn=%d ibat=%d, cc_max=%d rc=%d",
			fn, vbat, iin, wc68->iin_cc, icn, ibat, wc68->cc_max, rc);
	return;

error:
	dev_info(wc68->dev, "Error reading ibatt or icn: rc: %d, ibatt: %d, icn: %d\n",
		 rc, ibat, icn);
}

static int wc68_read_status(struct wc68_charger *wc68)
{
	int ret = 0;
	u16 addr;
	u8 reg_val;

	addr = cpu_to_be16(PROT_STS_1);
	ret = wc68_i2c_read(wc68->client, &addr, 2, &reg_val, 1);
	if (ret < 0) {
		dev_err(wc68->dev, "Error reading PROT_STS_1: %d\n", ret);
		return ret;
	}

	if (reg_val & PROT_STS_1_CBUS_OCP_WARN_STS) {
		ret = STS_MODE_IIN_LOOP;
	} else if (reg_val & PROT_STS_1_VBAT_OVP_WARN_STS) {
		ret = STS_MODE_VFLT_LOOP;
	} else {
		ret = STS_MODE_LOOP_INACTIVE; /* lower IIN or TA to enter CC? */
	}

	return ret;
}

/*
 * TODO: add formula and/or use device tree entries to configure. Can use
 * delta = WC68_COMP_VFLOAT_MAX to reduce the limit as float voltage
 * increases.
 * NOTE: how does this change with temperature, battery age?
 */
static int wc68_irdrop_limit(struct wc68_charger *wc68, int fv_uv)
{
	int delta = wc68->pdata->irdrop_limits[1];

	if (fv_uv < 4300000)
		delta = wc68->pdata->irdrop_limits[0];
	if (fv_uv >= WC68_COMP_VFLOAT_MAX)
		delta = wc68->pdata->irdrop_limits[2];

	return delta;
}

/* use max limit,  */
static int wc68_apply_irdrop(struct wc68_charger *wc68, int fv_uv)
{
	const int delta_limit = wc68_irdrop_limit(wc68, fv_uv);
	int ret, vbat, dc_vbat = 0, delta = 0;
	const bool adaptive = false;

	/* use classic irdrop */
	if (wc68->irdrop_comp_ok)
		goto error_done;

	ret = wc68_get_batt_info(wc68, BATT_VOLTAGE, &vbat);
	if (ret < 0)
		goto error_done;

	dc_vbat = wc68_read_adc(wc68, ADCCH_VBAT);
	if (dc_vbat < 0 || dc_vbat < vbat)
		goto error_done;

	if (adaptive) {
		delta = dc_vbat - vbat;
		if (delta > delta_limit)
			delta = delta_limit;
	} else {
		delta = delta_limit;
	}

	if (fv_uv + delta > WC68_COMP_VFLOAT_MAX)
		delta = WC68_COMP_VFLOAT_MAX - fv_uv;

error_done:
	dev_dbg(wc68->dev, "%s: fv_uv=%d->%d dc_vbat=%d, vbat=%d delta_v=%d\n",
		 __func__, fv_uv, fv_uv + delta, dc_vbat,
		 ret < 0 ? ret : vbat, delta);

	if (fv_uv + delta < dc_vbat) {
		dev_err(wc68->dev, "%s: fv_uv=%d, comp_fv_uv=%d is lower than VBAT=%d\n",
		       __func__, fv_uv, fv_uv + delta, dc_vbat);
		return -EINVAL;
	}

	return fv_uv + delta;
}

static int wc68_const_charge_voltage(struct wc68_charger *wc68);

/* irdrop compensation for the wc68 V_FLOAT, will only raise it */
static int wc68_comp_irdrop(struct wc68_charger *wc68)
{
	int ret = 0, v_float, fv_uv;

	v_float = wc68_const_charge_voltage(wc68);
	if (v_float < 0)
		return -EIO;

	fv_uv = wc68_apply_irdrop(wc68, wc68->fv_uv);
	if (fv_uv < 0)
		return -EIO;

	fv_uv = (fv_uv / VFLOAT_STEP) * VFLOAT_STEP;
	/* do not back down */
	if (fv_uv > v_float) {
		ret = wc68_set_vfloat(wc68, fv_uv);
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: v_float=%u->%u (%d)", __func__,
				v_float, fv_uv, ret);
	}

	return ret;
}

static int wc68_check_status(struct wc68_charger *wc68)
{
	int icn = -EINVAL, ibat = -EINVAL, vbat = -EINVAL;
	int rc, status;

	status = wc68_read_status(wc68);
	if (status < 0)
		goto error;

	rc = wc68_get_iin(wc68, &icn);
	if (rc)
		goto error;

	rc = wc68_get_batt_info(wc68, BATT_CURRENT, &ibat);
	if (rc)
		goto error;

	rc = wc68_get_batt_info(wc68, BATT_VOLTAGE, &vbat);

error:
	dev_dbg(wc68->dev, "%s: status=%d rc=%d icn:%d ibat:%d delta_c=%d, vbat:%d, fv:%d, cc_max:%d\n",
		 __func__, status, rc, icn, ibat, icn - ibat, vbat,
		 wc68->fv_uv, wc68->cc_max);

	return status;
}

/* hold mutex_lock(&wc68->lock); */
static int wc68_recover_ta(struct wc68_charger *wc68)
{
	int ret;

	if (wc68->ta_type == TA_TYPE_WIRELESS) {
		wc68->ta_vol = 0; /* set to a value to change rx vol */
		ret = wc68_send_rx_voltage(wc68, MSG_REQUEST_FIXED_PDO);
	} else {
		/* TODO: recover TA to value before handoff, or use DT */
		wc68->ta_vol = 9000000;
		wc68->ta_cur = 2200000;
		wc68->ta_objpos = 1; /* PDO1 - fixed 5V */
		ret = wc68_send_pd_message(wc68, MSG_REQUEST_FIXED_PDO);
	}

	/* will not be able to recover if TA is offline */
	if (ret < 0)
		dev_dbg(wc68->dev, "%s: cannot recover TA (%d)\n", __func__, ret);

	return 0;
}

/* Stop Charging */
static int wc68_stop_charging(struct wc68_charger *wc68)
{
	int ret = 0;

	/* mark the end with \n in logbuffer */
	logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
			"%s: wc68->charging_state=%d ret=%d\n",
			__func__, wc68->charging_state, ret);

	mutex_lock(&wc68->lock);

	/* Check the current state */
	if (wc68->charging_state == DC_STATE_NO_CHARGING)
		goto done;

	/* Stop Direct charging  */
	cancel_delayed_work(&wc68->timer_work);
	cancel_delayed_work(&wc68->pps_work);
	wc68->timer_id = TIMER_ID_NONE;
	wc68->timer_period = 0;

	/* Clear parameter */
	wc68->charging_state = DC_STATE_NO_CHARGING;
	wc68->ret_state = DC_STATE_NO_CHARGING;
	wc68->prev_iin = 0;
	wc68->prev_inc = INC_NONE;
	wc68->chg_mode = CHG_NO_DC_MODE;

	/* restore to config */
	wc68->pdata->iin_cfg = wc68->pdata->iin_cfg_max;
	wc68->pdata->v_float = wc68->pdata->v_float_dt;

	/*
	 * Clear charging configuration
	 * TODO: use defaults when these are negative or zero at startup
	 * NOTE: cc_max is twice of IIN + headroom
	 */
	wc68->cc_max = -1;
	wc68->fv_uv = -1;

	/* Clear requests for new Vfloat and new IIN */
	wc68->new_vfloat = 0;
	wc68->new_iin = 0;

	/* used to start DC and during errors */
	wc68->retry_cnt = 0;

	/* close stats */
	wc68_chg_stats_done(&wc68->chg_data, wc68);
	wc68_chg_stats_dump(wc68);

	/* TODO: something here to prep TA for the switch */

	ret = wc68_set_charging(wc68, false);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error-set_charging(main)\n", __func__);
		goto error;
	}

	/* stop charging and recover TA voltage */
	if (wc68->mains_online == true)
		wc68_recover_ta(wc68);

	power_supply_changed(wc68->mains);

done:
error:
	mutex_unlock(&wc68->lock);
	__pm_relax(wc68->monitor_wake_lock);
	dev_dbg(wc68->dev, "%s: END, ret=%d\n", __func__, ret);
	return ret;
}

#define FCC_TOLERANCE_RATIO		99
#define FCC_POWER_INCREASE_THRESHOLD	99

/*
 * Compensate TA current for the target input current called from
 * wc68_charge_ccmode() when loop becomes not active.
 *
 * wc68_charge_ccmode() ->
 * 	-> wc68_set_rx_voltage_comp()
 * 	-> wc68_set_ta_voltage_comp()
 * 	-> wc68_set_ta_current_comp2()
 *
 * NOTE: call holding mutex_lock(&wc68->lock);
 */
static int wc68_set_ta_current_comp(struct wc68_charger *wc68)
{
	const int iin_high = wc68->iin_cc + wc68->pdata->iin_cc_comp_offset;
	const int iin_low = wc68->iin_cc - wc68->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d prev_iin=%d",
			__func__, iin, iin_low, wc68->iin_cc, iin_high,
			icn, ibat, wc68->cc_max, rc,
			wc68->prev_iin);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {

		/* TA current is higher than the target input current */
		if (wc68->ta_cur > wc68->iin_cc) {
			/* TA current is over than IIN_CC */
			/* Decrease TA current (50mA) */
			wc68->ta_cur = wc68->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont1: ta_cur=%u",
					wc68->ta_cur);

		/* TA current is already less than IIN_CC */
		/* Compara IIN_ADC with the previous IIN_ADC */
		} else if (iin < (wc68->prev_iin - WC68_IIN_ADC_OFFSET)) {
			/* Assume that TA operation mode is CV mode */
			/* Decrease TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2-1: ta_vol=%u",
					wc68->ta_vol);
		} else {
			/* Assume TA operation mode is CL mode */
			/* Decrease TA current (50mA) */
			wc68->ta_cur = wc68->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2-2: ta_cur=%u",
					wc68->ta_cur);
		}

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

	} else if (iin < iin_low) {

		/* compare IIN ADC with previous IIN ADC + 20mA */
		if (iin > (wc68->prev_iin + WC68_IIN_ADC_OFFSET)) {
			/*
			 * TA voltage is not enough to supply the operating
			 * current of RDO: increase TA voltage
			 */

			/* Compare TA max voltage */
			if (wc68->ta_vol == wc68->ta_max_vol) {
				/* TA voltage is already the maximum voltage */
				/* Compare TA max current */
				if (wc68->ta_cur == wc68->ta_max_cur) {
					/* TA voltage and current are at max */
					logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
							"End1: ta_vol=%u, ta_cur=%u",
							wc68->ta_vol, wc68->ta_cur);

					/* Set timer */
					wc68->timer_id = TIMER_CHECK_CCMODE;
					wc68->timer_period = WC68_CCMODE_CHECK1_T;
				} else {
					/* Increase TA current (50mA) */
					wc68->ta_cur = wc68->ta_cur + PD_MSG_TA_CUR_STEP;

					logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
							"Cont3: ta_cur=%u",
							wc68->ta_cur);

					/* Send PD Message */
					wc68->timer_id = TIMER_PDMSG_SEND;
					wc68->timer_period = 0;

					/* Set TA increment flag */
					wc68->prev_inc = INC_TA_CUR;
				}
			} else {
				/* Increase TA voltage (20mV) */
				wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"Cont4: ta_vol=%u", wc68->ta_vol);

				/* Send PD Message */
				wc68->timer_id = TIMER_PDMSG_SEND;
				wc68->timer_period = 0;

				/* Set TA increment flag */
				wc68->prev_inc = INC_TA_VOL;
			}

		/* TA current is lower than the target input current */
		/* Check the previous TA increment */
		} else if (wc68->prev_inc == INC_TA_VOL) {
			/*
			 * The previous increment is TA voltage, but
			 * input current does not increase.
			 */

			/* Try to increase TA current */
			/* Compare TA max current */
			if (wc68->ta_cur == wc68->ta_max_cur) {

				/* TA current is already the maximum current */
				/* Compare TA max voltage */
				if (wc68->ta_vol == wc68->ta_max_vol) {
					/*
					 * TA voltage and current are already
					 * the maximum values
					 */
					logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
							"End2: ta_vol=%u, ta_cur=%u",
							wc68->ta_vol, wc68->ta_cur);

					wc68->timer_id = TIMER_CHECK_CCMODE;
					wc68->timer_period = WC68_CCMODE_CHECK1_T;
				} else {
					/* Increase TA voltage (20mV) */
					wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP;
					logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
							"Cont5: ta_vol=%u",
							wc68->ta_vol);

					/* Send PD Message */
					wc68->timer_id = TIMER_PDMSG_SEND;
					wc68->timer_period = 0;

					/* Set TA increment flag */
					wc68->prev_inc = INC_TA_VOL;
				}
			} else {
				const unsigned int ta_cur = wc68->ta_cur +
							    PD_MSG_TA_CUR_STEP;

				/* Increase TA current (50mA) */
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"Cont6: ta_cur=%u->%u",
						wc68->ta_cur, ta_cur);

				wc68->ta_cur = wc68->ta_cur + PD_MSG_TA_CUR_STEP;
				wc68->timer_id = TIMER_PDMSG_SEND;
				wc68->timer_period = 0;

				wc68->prev_inc = INC_TA_CUR;
			}

		/*
		 * The previous increment was TA current, but input current
		 * did not increase. Try to increase TA voltage.
		 */
		} else if (wc68->ta_vol == wc68->ta_max_vol) {
			/* TA voltage is already the maximum voltage */

			/* Compare TA maximum current */
			if (wc68->ta_cur == wc68->ta_max_cur) {
				/*
				* TA voltage and current are already at the
				 * maximum values
				 */
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"End3: ta_vol=%u, ta_cur=%u",
						 wc68->ta_vol, wc68->ta_cur);

				wc68->timer_id = TIMER_CHECK_CCMODE;
				wc68->timer_period = WC68_CCMODE_CHECK1_T;
			} else {
				/* Increase TA current (50mA) */
				wc68->ta_cur = wc68->ta_cur + PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"Cont7: ta_cur=%u", wc68->ta_cur);

				/* Send PD Message */
				wc68->timer_id = TIMER_PDMSG_SEND;
				wc68->timer_period = 0;

				/* Set TA increment flag */
				wc68->prev_inc = INC_TA_CUR;
			}
		} else {
			/* Increase TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"Comp. Cont8: ta_vol=%u->%u",
					wc68->ta_vol, wc68->ta_vol);

			/* Send PD Message */
			wc68->timer_id = TIMER_PDMSG_SEND;
			wc68->timer_period = 0;

			/* Set TA increment flag */
			wc68->prev_inc = INC_TA_VOL;
		}

	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"Comp. End4(valid): ta_vol=%u, ta_cur=%u",
				wc68->ta_vol, wc68->ta_cur);
		/* Set timer */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = WC68_CCMODE_CHECK1_T;

		/* b/186969924: reset increment state on valid */
		wc68->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	wc68->prev_iin = iin;
	return 0;
}

/*
 * max iin for 2:1 mode given cc_max and iin_cfg.
 * TODO: maybe use pdata->iin_cfg if cc_max is zero or negative.
 */
static int wc68_get_iin_max(const struct wc68_charger *wc68, int cc_max)
{
	const int cc_limit = wc68->pdata->iin_max_offset + cc_max / 2;
	int iin_max;

	iin_max = min(wc68->pdata->iin_cfg_max, (unsigned int)cc_limit);

	dev_dbg(wc68->dev, "%s: iin_max=%d iin_cfg=%u iin_cfg_max=%d cc_max=%d cc_limit=%d\n",
		 __func__, iin_max, wc68->pdata->iin_cfg,
		 wc68->pdata->iin_cfg_max, cc_max, cc_limit);

	return iin_max;
}

/* Compensate TA current for constant power mode */
/* hold mutex_lock(&wc68->lock), schedule on return 0 */
static int wc68_set_ta_current_comp2(struct wc68_charger *wc68)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			wc68->iin_cc - WC68_IIN_CC_COMP_OFFSET_CP,
			wc68->iin_cc,
			wc68->iin_cc + WC68_IIN_CC_COMP_OFFSET_CP,
			wc68->pdata->iin_cfg,
			icn, ibat, wc68->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (wc68->pdata->iin_cfg + wc68->pdata->iin_cc_comp_offset)) {
		/* TA current is higher than the target input current limit */
		wc68->ta_cur = wc68->ta_cur - PD_MSG_TA_CUR_STEP;

		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	} else if (iin < (wc68->iin_cc - WC68_IIN_CC_COMP_OFFSET_CP)) {

		/* TA current is lower than the target input current */
		/* IIN_ADC < IIN_CC -20mA */
		if (wc68->ta_vol == wc68->ta_max_vol) {
			const int iin_cc_lb = wc68->iin_cc -
				      wc68->pdata->iin_cc_comp_offset;

			/* Check IIN_ADC < IIN_CC - 50mA */
			if (iin < iin_cc_lb) {
				unsigned int iin_apdo;
				unsigned int val;

				/* Set new IIN_CC to IIN_CC - 50mA */
				wc68->iin_cc = wc68->iin_cc -
					  wc68->pdata->iin_cc_comp_offset;

				/* Set new TA_MAX_VOL to TA_MAX_PWR/IIN_CC */
				/* Adjust new IIN_CC with APDO resolution */
				iin_apdo = wc68->iin_cc / PD_MSG_TA_CUR_STEP;
				iin_apdo = iin_apdo * PD_MSG_TA_CUR_STEP;
				/* in mV */
				val = wc68->ta_max_pwr / (iin_apdo / wc68->chg_mode / 1000);
				/* Adjust values with APDO resolution(20mV) */
				val = val * 1000 / PD_MSG_TA_VOL_STEP;
				val = val * PD_MSG_TA_VOL_STEP; /* uV */

				/* Set new TA_MAX_VOL */
				wc68->ta_max_vol = min(val, (unsigned int)WC68_TA_MAX_VOL *
							  wc68->chg_mode);

				/* Increase TA voltage(40mV) */
				wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP * 2;

				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"Cont1: ta_vol=%u",
						wc68->ta_vol);

				/* Send PD Message */
				wc68->timer_id = TIMER_PDMSG_SEND;
				wc68->timer_period = 0;
			} else {
				/* Wait for next current step compensation */
				/* IIN_CC - 50mA < IIN ADC < IIN_CC - 20mA */
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"Comp.(wait): ta_vol=%u",
						wc68->ta_vol);

				/* Set timer */
				wc68->timer_id = TIMER_CHECK_CCMODE;
				wc68->timer_period = WC68_CCMODE_CHECK2_T;
			}
		} else {
			/* Increase TA voltage(40mV) */
			wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP * 2;
			if (wc68->ta_vol > wc68->ta_max_vol)
				wc68->ta_vol = wc68->ta_max_vol;

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
					wc68->ta_vol);

			/* Send PD Message */
			wc68->timer_id = TIMER_PDMSG_SEND;
			wc68->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CFG + 50mA */
		dev_dbg(wc68->dev, "End(valid): ta_vol=%u\n", wc68->ta_vol);

		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = WC68_CCMODE_CHECK2_T;

		/* b/186969924: reset increment state on valid */
		wc68->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	wc68->prev_iin = iin;
	return 0;
}

/* Compensate TA voltage for the target input current */
/* hold mutex_lock(&wc68->lock), schedule on return 0 */
static int wc68_set_ta_voltage_comp(struct wc68_charger *wc68)
{
	const int iin_high = wc68->iin_cc + wc68->pdata->iin_cc_comp_offset;
	const int iin_low = wc68->iin_cc - wc68->pdata->iin_cc_comp_offset;
	const int ibat_limit = (wc68->cc_max * FCC_POWER_INCREASE_THRESHOLD) / 100;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);
	dev_dbg(wc68->dev, "%s: = charging_state=%u == \n", __func__,
		 wc68->charging_state);

	/* IIN = IBAT+SYSLOAD */
	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, iin_low, wc68->iin_cc, iin_high,
			icn, ibat, wc68->cc_max, rc);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		wc68->ta_vol = wc68->ta_vol - PD_MSG_TA_VOL_STEP;
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				wc68->ta_vol);

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

	} else if (iin < wc68->iin_cc - wc68->pdata->iin_cc_comp_offset) {

		/* TA current is lower than the target input current */
		/* Compare TA max voltage */
		if (wc68->ta_vol == wc68->ta_max_vol) {
			/* TA is already at maximum voltage */
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,"End1(max TA vol): ta_vol=%u",
					wc68->ta_vol);

			/* Set timer */
			/* Check the current charging state */
			if (wc68->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				wc68->timer_id = TIMER_CHECK_CCMODE;
				wc68->timer_period = WC68_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				wc68->timer_id = TIMER_CHECK_CVMODE;
				wc68->timer_period = WC68_CVMODE_CHECK_T;
			}
		} else {
			const unsigned int ta_vol = wc68->ta_vol;

			/* Increase TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2: ta_vol:%u->%u",
					ta_vol, wc68->ta_vol);

			/* Send PD Message */
			wc68->timer_id = TIMER_PDMSG_SEND;
			wc68->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"End(valid): ta_vol=%u low_ibat=%d\n",
				wc68->ta_vol, ibat < ibat_limit);

		/* Check the current charging state */
		if (wc68->charging_state == DC_STATE_CC_MODE) {
			wc68->timer_id = TIMER_CHECK_CCMODE;
			wc68->timer_period = WC68_CCMODE_CHECK1_T;
		} else {
			wc68->timer_id = TIMER_CHECK_CVMODE;
			wc68->timer_period = WC68_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/* hold mutex_lock(&wc68->lock), schedule on return 0 */
static int wc68_set_rx_voltage_comp(struct wc68_charger *wc68)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);

	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			wc68->iin_cc - wc68->pdata->iin_cc_comp_offset,
			wc68->iin_cc,
			wc68->iin_cc + wc68->pdata->iin_cc_comp_offset,
			icn, ibat, wc68->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (wc68->iin_cc + wc68->pdata->iin_cc_comp_offset)) {

		/* RX current is higher than the target input current */
		wc68->ta_vol = wc68->ta_vol - WCRX_VOL_STEP;
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont1: rx_vol=%u",
				wc68->ta_vol);

		/* Set RX Voltage */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

	} else if (iin < (wc68->iin_cc - wc68->pdata->iin_cc_comp_offset)) {

		/* RX current is lower than the target input current */
		/* Compare RX max voltage */
		if (wc68->ta_vol == wc68->ta_max_vol) {

			/* TA current is already the maximum voltage */
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"End1(max RX vol): rx_vol=%u",
					wc68->ta_vol);

			/* Check the current charging state */
			if (wc68->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				wc68->timer_id = TIMER_CHECK_CCMODE;
				wc68->timer_period = WC68_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				wc68->timer_id = TIMER_CHECK_CVMODE;
				wc68->timer_period = WC68_CVMODE_CHECK_T;
			}
		} else {
			/* Increase RX voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol + WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2: rx_vol=%u",
					wc68->ta_vol);

			/* Set RX Voltage */
			wc68->timer_id = TIMER_PDMSG_SEND;
			wc68->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "End(valid): rx_vol=%u",
				wc68->ta_vol);

		if (wc68->charging_state == DC_STATE_CC_MODE) {
			wc68->timer_id = TIMER_CHECK_CCMODE;
			wc68->timer_period = WC68_CCMODE_CHECK1_T;
		} else {
			wc68->timer_id = TIMER_CHECK_CVMODE;
			wc68->timer_period = WC68_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/*
 * iin limit for 2:1 for the adapter and chg_mode
 * Minimum between the configuration, cc_max (scaled with offset) and the
 * adapter capabilities.
 */
static int wc68_get_iin_limit(const struct wc68_charger *wc68)
{
	int iin_cc;

	iin_cc = wc68_get_iin_max(wc68, wc68->cc_max);
	if (wc68->ta_max_cur * wc68->chg_mode < iin_cc)
		iin_cc = wc68->ta_max_cur * wc68->chg_mode;

	dev_dbg(wc68->dev, "%s: iin_cc=%d ta_max_cur=%u, chg_mode=%d\n", __func__,
		 iin_cc, wc68->ta_max_cur, wc68->chg_mode);

	return iin_cc;
}

/* recalculate ->ta_vol looking at demand (cc_max) */
static int wc68_set_wireless_dc(struct wc68_charger *wc68, int vbat)
{
	unsigned long val;

	wc68->iin_cc = wc68_get_iin_limit(wc68);

	/* RX_vol = MAX[(2*VBAT_ADC*CHG_mode + 500mV), 8.0V*CHG_mode] */
	wc68->ta_vol = max(WC68_TA_MIN_VOL_PRESET * wc68->chg_mode,
				(2 * vbat *  wc68->chg_mode +
				WC68_TA_VOL_PRE_OFFSET));

	/* RX voltage resolution is 100mV */
	val = wc68->ta_vol / WCRX_VOL_STEP;
	wc68->ta_vol = val * WCRX_VOL_STEP;
	/* Set RX voltage to MIN[RX voltage, RX_MAX_VOL*chg_mode] */
	wc68->ta_vol = min(wc68->ta_vol, wc68->ta_max_vol);

	/* ta_cur is ignored */
	logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_max_vol=%d", __func__,
			wc68->iin_cc, wc68->ta_vol, wc68->ta_max_vol);

	return 0;
}

/* recalculate ->ta_vol and ->ta_cur looking at demand (cc_max) */
static int wc68_set_wired_dc(struct wc68_charger *wc68, int vbat)
{
	unsigned long val;
	int iin_cc;

	wc68->iin_cc = wc68_get_iin_limit(wc68);

	/* Calculate new TA max voltage, current */
	val = wc68->iin_cc / PD_MSG_TA_CUR_STEP;
	iin_cc = val * PD_MSG_TA_CUR_STEP;

	val = wc68->ta_max_pwr / (iin_cc / wc68->chg_mode  / 1000); /* mV */

	/* Adjust values with APDO resolution(20mV) */
	val = val * 1000 / PD_MSG_TA_VOL_STEP;
	val = val * PD_MSG_TA_VOL_STEP; /* uV */
	wc68->ta_max_vol = min(val, (unsigned long)WC68_TA_MAX_VOL *
				  wc68->chg_mode);

	/* MAX[8000mV * chg_mode, 2 * VBAT_ADC * chg_mode + 500 mV] */
	wc68->ta_vol = max(WC68_TA_MIN_VOL_PRESET * wc68->chg_mode,
			      2 * vbat * wc68->chg_mode + WC68_TA_VOL_PRE_OFFSET);

	/* PPS voltage resolution is 20mV */
	val = wc68->ta_vol / PD_MSG_TA_VOL_STEP;
	wc68->ta_vol = val * PD_MSG_TA_VOL_STEP;
	wc68->ta_vol = min(wc68->ta_vol, wc68->ta_max_vol);
	/* Set TA current to IIN_CC */
	wc68->ta_cur = iin_cc / wc68->chg_mode;

	logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_cur=%d ta_max_vol=%d",
			__func__, wc68->iin_cc, wc68->ta_vol, wc68->ta_cur,
			wc68->ta_max_vol);

	return 0;
}

/*
 * like wc68_preset_dcmode() but will not query the TA.
 * Called from timer:
 * [wc68_charge_ccmode | wc68_charge_cvmode] ->
 * 	wc68_apply_new_iin() ->
 * 		wc68_adjust_ta_current() ->
 * 			wc68_reset_dcmode()
 * 	wc68_apply_new_vfloat() ->
 * 		wc68_reset_dcmode()
 *
 * NOTE: caller holds mutex_lock(&wc68->lock);
 */
static int wc68_reset_dcmode(struct wc68_charger *wc68)
{
	int ret = -EINVAL, vbat;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);
	dev_dbg(wc68->dev, "%s: = charging_state=%u == \n", __func__,
		 wc68->charging_state);

	if (wc68->cc_max < 0) {
		dev_err(wc68->dev, "%s: invalid cc_max=%d\n", __func__, wc68->cc_max);
		goto error;
	}

	/*
	 * VBAT is over threshold but it might be "bouncy" due to transitory
	 * used to determine ta_vout.
	 */
	vbat = wc68_read_adc(wc68, ADCCH_VBAT);
	if (vbat < 0)
		return vbat;

	/* Check the TA type and set the charging mode */
	if (wc68->ta_type == TA_TYPE_WIRELESS)
		ret = wc68_set_wireless_dc(wc68, vbat);
	else
		ret = wc68_set_wired_dc(wc68, vbat);

	/* Clear previous IIN ADC, TA increment flag */
	wc68->prev_inc = INC_NONE;
	wc68->prev_iin = 0;
error:
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The caller was triggered from wc68_apply_new_iin(), return to the
 * calling CC or CV loop.
 * call holding mutex_unlock(&wc68->lock);
 */
static void wc68_return_to_loop(struct wc68_charger *wc68)
{
	switch (wc68->ret_state) {
	case DC_STATE_CC_MODE:
		wc68->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_CV_MODE:
		wc68->timer_id = TIMER_CHECK_CVMODE;
		break;
	default:
		dev_err(wc68->dev, "%s: invalid ret_state=%u\n",
			__func__, wc68->ret_state);
		return;
	}

	dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
		 wc68->charging_state, wc68->ret_state);

	wc68->charging_state = wc68->ret_state;
	wc68->timer_period = 1000;
	wc68->ret_state = 0;
	wc68->new_iin = 0;
}

/*
 * Kicked from wc68_apply_new_iin() when wc68->new_iin!=0 and completed
 * off the timer. Never called on WLC_DC.
 * NOTE: Will return to the calling loop in ->ret_state
 */
static int wc68_adjust_ta_current(struct wc68_charger *wc68)
{
	const int ta_limit = wc68->iin_cc / wc68->chg_mode;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;
	int ret = 0;

	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=%d ta_limit=%d, iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, wc68->iin_cc, ta_limit, wc68->pdata->iin_cfg,
			icn, ibat, wc68->cc_max, rc);

	if (wc68->charging_state != DC_STATE_ADJUST_TACUR)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_ADJUST_TACUR);

	wc68->charging_state = DC_STATE_ADJUST_TACUR;

	if (wc68->ta_cur == ta_limit) {

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"adj. End, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				wc68->ta_cur, wc68->ta_vol,
				wc68->iin_cc, wc68->chg_mode);

		/* "Recover" IIN_CC to the original value (new_iin) */
		wc68->iin_cc = wc68->new_iin;
		wc68_return_to_loop(wc68);

	} else if (wc68->iin_cc > wc68->pdata->iin_cfg) {
		const int old_iin_cfg = wc68->pdata->iin_cfg;

		/* Raise iin_cfg to the new iin_cc value (why??!?!?) */
		wc68->pdata->iin_cfg = wc68->iin_cc;

		ret = wc68_set_input_current(wc68, wc68->iin_cc);
		if (ret == 0)
			ret = wc68_reset_dcmode(wc68);
		if (ret < 0)
			goto error;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"New IIN, ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, iin_cfg=%d->%d chg_mode=%u",
				wc68->ta_max_vol, wc68->ta_max_cur,
				wc68->ta_max_pwr, wc68->iin_cc,
				old_iin_cfg, wc68->iin_cc,
				wc68->chg_mode);

		wc68->new_iin = 0;

		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_ADJUST_CC);

		/* Send PD Message and go to Adjust CC mode */
		wc68->charging_state = DC_STATE_ADJUST_CC;
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	} else {
		unsigned int val;

		/*
		 * Adjust IIN_CC with APDO resolution(50mA)
		 * wc68->iin_cc will be reset to wc68->new_iin when
		 * ->ta_cur reaches the ta_limit at the beginning of the
		 * function
		 */
		val = wc68->iin_cc / PD_MSG_TA_CUR_STEP;
		wc68->iin_cc = val * PD_MSG_TA_CUR_STEP;
		wc68->ta_cur = wc68->iin_cc / wc68->chg_mode;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "adjust iin=%u ta_cur=%d chg_mode=%d",
				wc68->iin_cc, wc68->ta_cur, wc68->chg_mode);

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	}

	/* reschedule on ret == 0 */
error:
	return ret;
}

/* Kicked from apply_new_iin() then run off the timer
 * call holding mutex_lock(&wc68->lock);
 */
static int wc68_adjust_ta_voltage(struct wc68_charger *wc68)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (wc68->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_ADJUST_TAVOL);

	wc68->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, wc68->iin_cc - PD_MSG_TA_CUR_STEP,
			wc68->iin_cc, wc68->iin_cc + PD_MSG_TA_CUR_STEP,
			icn, ibat, wc68->cc_max, rc);

	if (iin < 0)
		return iin;


	/* Compare IIN ADC with targer input current */
	if (iin > (wc68->iin_cc + PD_MSG_TA_CUR_STEP)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		wc68->ta_vol = wc68->ta_vol - PD_MSG_TA_VOL_STEP;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont1, ta_vol=%u",
				wc68->ta_vol);

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	} else if (iin < (wc68->iin_cc - PD_MSG_TA_CUR_STEP)) {
		/* TA current is lower than the target input current */

		if (wc68_check_status(wc68) == STS_MODE_VFLT_LOOP) {
			/* IIN current may not able to increase in CV */

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					wc68->ta_cur, wc68->ta_vol,
					wc68->iin_cc, wc68->chg_mode);

			wc68_return_to_loop(wc68);
		} else if (wc68->ta_vol == wc68->ta_max_vol) {
			/* TA TA voltage is already at the maximum voltage */

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"End1, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					wc68->ta_cur, wc68->ta_vol,
					wc68->iin_cc, wc68->chg_mode);

			wc68_return_to_loop(wc68);
		} else {
			/* Increase TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol + PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2, ta_vol=%u",
					wc68->ta_vol);

			/* Send PD Message */
			wc68->timer_id = TIMER_PDMSG_SEND;
			wc68->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"End2, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				wc68->ta_cur, wc68->ta_vol,
			wc68->iin_cc, wc68->chg_mode);

		wc68_return_to_loop(wc68);
	}

	return 0;
}

/*
 * Kicked from apply_new_iin() then run off the timer
 * * NOTE: caller must hold mutex_lock(&wc68->lock)
 */
static int wc68_adjust_rx_voltage(struct wc68_charger *wc68)
{
	const int iin_high = wc68->iin_cc + wc68->pdata->iin_cc_comp_offset;
	const int iin_low = wc68->iin_cc - wc68->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (wc68->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_ADJUST_TAVOL);

	wc68->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = wc68_get_current_adcs(wc68, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > wc68->cc_max;
	if (ovc_flag)
		wc68_chg_stats_inc_ovcf(&wc68->chg_data, ibat, wc68->cc_max);

	logbuffer_prlog(wc68, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, iin_low, wc68->iin_cc, iin_high,
			icn, ibat, wc68->cc_max, rc);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with targer input current */
	if (iin > iin_high) {
		/* RX current is higher than the target input current */

		/* Decrease RX voltage (100mV) */
		wc68->ta_vol = wc68->ta_vol - WCRX_VOL_STEP;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont1, rx_vol=%u",
				wc68->ta_vol);

		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	} else if (iin < iin_low) {
		/* RX current is lower than the target input current */

		if (wc68_check_status(wc68) == STS_MODE_VFLT_LOOP) {
			/* RX current may not able to increase in CV */
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, rx_vol=%u, iin_cc=%u",
					wc68->ta_vol, wc68->iin_cc);

			wc68_return_to_loop(wc68);
		} else if (wc68->ta_vol == wc68->ta_max_vol) {
			/* RX current is already the maximum voltage */
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"End1, rx_vol=%u, iin_cc=%u, chg_mode=%u",
					wc68->ta_vol, wc68->iin_cc,
					wc68->chg_mode);

			/* Return charging state to the previous state */
			wc68_return_to_loop(wc68);
		} else {
			/* Increase RX voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol + WCRX_VOL_STEP;

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2, rx_vol=%u",
					wc68->ta_vol);

			/* Set RX voltage */
			wc68->timer_id = TIMER_PDMSG_SEND;
			wc68->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"End2, rx_vol=%u, iin_cc=%u, chg_mode=%u",
				wc68->ta_vol, wc68->iin_cc,
				wc68->chg_mode);

		/* Return charging state to the previous state */
		wc68_return_to_loop(wc68);
	}

	return 0;
}

/*
 * Called from CC and CV loops to set a new IIN (i.e. a new cc_max charging
 * current). Should also change the iin_cfg to avoid overcurrents.
 * NOTE: caller must hold mutex_lock(&wc68->lock)
 */
static int wc68_apply_new_iin(struct wc68_charger *wc68)
{
	int ret;

	logbuffer_prlog(wc68, LOGLEVEL_INFO,
			"new_iin=%d (cc_max=%d), ta_type=%d charging_state=%d",
			wc68->new_iin, wc68->cc_max,
			wc68->ta_type, wc68->charging_state);

	/* iin_cfg is adjusted UP in wc68_set_input_current() */
	ret = wc68_set_input_current(wc68, wc68->new_iin);
	if (ret < 0)
		return ret;
	wc68->pdata->iin_cfg = wc68->new_iin;

	 /*
	  * ->ret_state is used to go back to the loop (CC or CV) that called
	  * this function.
	  */
	wc68->ret_state = wc68->charging_state;

	/*
	 * new_iin is used to trigger the process which might span one or more
	 * timer ticks the new_iin . The flag will be cleared once the target
	 * is reached.
	 */
	wc68->iin_cc = wc68->new_iin;
	if (wc68->ta_type == TA_TYPE_WIRELESS) {
		ret = wc68_adjust_rx_voltage(wc68);
	} else if (wc68->iin_cc < (WC68_TA_MIN_CUR * wc68->chg_mode)) {
		/* TA current = WC68_TA_MIN_CUR(1.0A) */
		wc68->ta_cur = WC68_TA_MIN_CUR;
		ret = wc68_adjust_ta_voltage(wc68);
	} else {
		ret = wc68_adjust_ta_current(wc68);
	}

	/* need reschedule on ret != 0 */

	dev_dbg(wc68->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * also called from wc68_set_new_cc_max()
 * call holding mutex_unlock(&wc68->lock);
 */
static int wc68_set_new_iin(struct wc68_charger *wc68, int iin)
{
	int ret = 0;

	if (iin < 0) {
		dev_dbg(wc68->dev, "%s: ignore negative iin=%d\n", __func__, iin);
		return 0;
	}

	/* same as previous request nevermind */
	if (iin == wc68->new_iin)
		return 0;

	dev_dbg(wc68->dev, "%s: new_iin=%d->%d state=%d\n", __func__,
		 wc68->new_iin, iin, wc68->charging_state);

	/* apply iin_cc in wc68_preset_config() at start */
	if (wc68->charging_state == DC_STATE_NO_CHARGING ||
	    wc68->charging_state == DC_STATE_CHECK_VBAT) {

		/* used on start vs the ->iin_cfg one */
		wc68->pdata->iin_cfg = iin;
		wc68->iin_cc = iin;
	} else if (wc68->ret_state == 0) {
		/*
		 * wc68_apply_new_iin() has not picked out the value yet
		 * and the value can be changed safely.
		 */
		wc68->new_iin = iin;

		/* might want to tickle the loop now */
	} else {
		/* the caller must retry */
		ret = -EAGAIN;
	}

	dev_dbg(wc68->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The is no CC loop in this part: current must be controlled on TA side
 * adjusting output power. cc_max (the charging current) is scaled to iin
 *
 */
static int wc68_set_new_cc_max(struct wc68_charger *wc68, int cc_max)
{
	const int prev_cc_max = wc68->cc_max;
	int iin_max, ret = 0;

	if (cc_max < 0) {
		dev_dbg(wc68->dev, "%s: ignore negative cc_max=%d\n", __func__, cc_max);
		return 0;
	}

	mutex_lock(&wc68->lock);

	/* same as previous request nevermind */
	if (cc_max == wc68->cc_max)
		goto done;

	/* iin will be capped by the adapter capabilities in reset_dcmode() */
	iin_max = wc68_get_iin_max(wc68, cc_max);
	if (iin_max <= 0) {
		dev_dbg(wc68->dev, "%s: ignore negative iin_max=%d\n", __func__, iin_max);
		goto done;
	}

	ret = wc68_set_new_iin(wc68, iin_max);
	if (ret == 0)
		wc68->cc_max = cc_max;

	logbuffer_prlog(wc68, LOGLEVEL_INFO,
			"%s: charging_state=%d cc_max=%d->%d iin_max=%d, ret=%d",
			__func__, wc68->charging_state, prev_cc_max,
			cc_max, iin_max, ret);

done:
	dev_dbg(wc68->dev, "%s: ret=%d\n", __func__, ret);
	mutex_unlock(&wc68->lock);
	return ret;
}

/*
 * Apply wc68->new_vfloat to the charging voltage.
 * Called from CC and CV loops, needs mutex_lock(&wc68->lock)
 */
static int wc68_apply_new_vfloat(struct wc68_charger *wc68)
{
	int fv_uv, ret = 0;

	/* compensated float voltage, -EINVAL if under dc_vbat */
	fv_uv = wc68_apply_irdrop(wc68, wc68->new_vfloat);
	if (fv_uv < 0)
		return fv_uv;

	/* actually change the hardware */
	ret = wc68_set_vfloat(wc68, fv_uv);
	if (ret < 0)
		goto error_done;

	/* Restart the process (TODO: optimize this) */
	ret = wc68_reset_dcmode(wc68);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: cannot reset dcmode (%d)\n", __func__, ret);
	} else {
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_ADJUST_CC);

		wc68->charging_state = DC_STATE_ADJUST_CC;
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	}

error_done:
	logbuffer_prlog(wc68, LOGLEVEL_INFO,
			"%s: new_vfloat=%d, fv_uv=%d ret=%d", __func__,
			wc68->new_vfloat, fv_uv, ret);

	if (ret == 0)
		wc68->new_vfloat = 0;

	return ret;
}

static int wc68_set_new_vfloat(struct wc68_charger *wc68, int vfloat)
{
	int ret = 0;

	if (vfloat < 0) {
		dev_dbg(wc68->dev, "%s: ignore negative vfloat %d\n", __func__, vfloat);
		return 0;
	}

	mutex_lock(&wc68->lock);
	if (wc68->fv_uv == vfloat)
		goto done;

	/* this is what is requested */
	wc68->fv_uv = vfloat;

	/* use fv_uv at start in wc68_preset_config() */
	if (wc68->charging_state != DC_STATE_NO_CHARGING &&
	    wc68->charging_state != DC_STATE_CHECK_VBAT)
		/* applied in wc68_apply_new_vfloat() from CC or in CV loop */
		wc68->new_vfloat = vfloat;
		/* might want to tickle the cycle */

done:
	mutex_unlock(&wc68->lock);
	return ret;
}

/* called on loop inactive */
static int wc68_ajdust_ccmode_wireless(struct wc68_charger *wc68, int iin)
{
	/* IIN_ADC > IIN_CC -20mA ? */
	if (iin > (wc68->iin_cc - WC68_IIN_ADC_OFFSET)) {
		/* Input current is already over IIN_CC */
		/* End RX voltage adjustment */

		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		wc68->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "End1: IIN_ADC=%d, rx_vol=%u",
				iin, wc68->ta_vol);

		/* Clear TA increment flag */
		wc68->prev_inc = INC_NONE;
		/* Go to CC mode */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = 0;

	/* Check RX voltage */
	} else if (wc68->ta_vol == wc68->ta_max_vol) {
		/* RX voltage is already max value */
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,"End2: MAX value, rx_vol=%u max=%d",
				wc68->ta_vol, wc68->ta_max_vol);

		/* Clear TA increment flag */
		wc68->prev_inc = INC_NONE;
		/* Go to CC mode */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = 0;
	} else {
		/* Try to increase RX voltage(100mV) */
		wc68->ta_vol = wc68->ta_vol + WCRX_VOL_STEP;
		if (wc68->ta_vol > wc68->ta_max_vol)
			wc68->ta_vol = wc68->ta_max_vol;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont: rx_vol=%u",
				wc68->ta_vol);
		/* Set RX voltage */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	}

	return 0;
}

/* called on loop inactive */
static int wc68_ajdust_ccmode_wired(struct wc68_charger *wc68, int iin)
{

	/* USBPD TA is connected */
	if (iin > (wc68->iin_cc - WC68_IIN_ADC_OFFSET)) {
		/* IIN_ADC > IIN_CC -20mA ? */
		/* Input current is already over IIN_CC */
		/* End TA voltage and current adjustment */

		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		wc68->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"End1: IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, wc68->ta_vol, wc68->ta_cur);

		/* Clear TA increment flag */
		wc68->prev_inc = INC_NONE;
		/* Go to CC mode */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = 0;

	/* Check TA voltage */
	} else if (wc68->ta_vol == wc68->ta_max_vol) {
		/* TA voltage is already max value */
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"End2: MAX value, ta_vol=%u, ta_cur=%u",
				wc68->ta_vol, wc68->ta_cur);

		/* Clear TA increment flag */
		wc68->prev_inc = INC_NONE;
		/* Go to CC mode */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = 0;

		/* Check TA tolerance
		 * The current input current compares the final input
		 * current(IIN_CC) with 100mA offset PPS current tolerance
		 * has +/-150mA, so offset defined 100mA(tolerance +50mA)
		 */
	} else if (iin < (wc68->iin_cc - WC68_TA_IIN_OFFSET)) {
		/*
		 * TA voltage too low to enter TA CC mode, so we
		 * should increase TA voltage
		 */
		wc68->ta_vol = wc68->ta_vol + WC68_TA_VOL_STEP_ADJ_CC *
					wc68->chg_mode;

		if (wc68->ta_vol > wc68->ta_max_vol)
			wc68->ta_vol = wc68->ta_max_vol;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				wc68->ta_vol);

		/* Set TA increment flag */
		wc68->prev_inc = INC_TA_VOL;
		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

	/* compare IIN ADC with previous IIN ADC + 20mA */
	} else if (iin > (wc68->prev_iin + WC68_IIN_ADC_OFFSET)) {
		/* TA can supply more current if TA voltage is high */
		/* TA voltage too low for TA CC mode: increase it */
		wc68->ta_vol = wc68->ta_vol +
					WC68_TA_VOL_STEP_ADJ_CC *
					wc68->chg_mode;
		if (wc68->ta_vol > wc68->ta_max_vol)
			wc68->ta_vol = wc68->ta_max_vol;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
				wc68->ta_vol);
		/* Set TA increment flag */
		wc68->prev_inc = INC_TA_VOL;

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

	/* Check the previous increment */
	} else if (wc68->prev_inc == INC_TA_CUR) {
		/*
		 * The previous increment is TA current, but input
		 * current does not increase. Try with voltage.
		 */

		wc68->ta_vol = wc68->ta_vol +
					WC68_TA_VOL_STEP_ADJ_CC *
					wc68->chg_mode;
		if (wc68->ta_vol > wc68->ta_max_vol)
			wc68->ta_vol = wc68->ta_max_vol;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont3: ta_vol=%u",
				wc68->ta_vol);

		wc68->prev_inc = INC_TA_VOL;
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

		/*
		 * The previous increment is TA voltage, but input
		 * current does not increase
		 */

		/* Try to increase TA current */
		/* Check APDO max current */
	} else if (wc68->ta_cur == wc68->ta_max_cur) {
		/* TA current is maximum current */

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"End(MAX_CUR): IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, wc68->ta_vol, wc68->ta_cur);

		wc68->prev_inc = INC_NONE;

		/* Go to CC mode */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = 0;
	} else {
		/* TA has tolerance and compensate it as real current */
		/* Increase TA current(50mA) */
		wc68->ta_cur = wc68->ta_cur + PD_MSG_TA_CUR_STEP;
		if (wc68->ta_cur > wc68->ta_max_cur)
			wc68->ta_cur = wc68->ta_max_cur;

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "Cont4: ta_cur=%u",
				wc68->ta_cur);

		wc68->prev_inc = INC_TA_CUR;
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
	}

	return 0;
}


/* 2:1 Direct Charging Adjust CC MODE control
 * called at the beginnig of CC mode charging. Will be followed by
 * wc68_charge_ccmode with which share some of the adjustments.
 */
static int wc68_charge_adjust_ccmode(struct wc68_charger *wc68)
{
	int  iin, ccmode, vbatt, vin_vol;
	bool apply_ircomp = false;
	int ret = 0;

	mutex_lock(&wc68->lock);

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);
	wc68_prlog_state(wc68, __func__);

	if (wc68->charging_state != DC_STATE_ADJUST_CC)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_ADJUST_CC);

	wc68->charging_state = DC_STATE_ADJUST_CC;

	ret = wc68_check_error(wc68);
	if (ret != 0)
		goto error; /*This is not active mode. */

	ccmode = wc68_check_status(wc68);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}

	switch(ccmode) {
	case STS_MODE_IIN_LOOP:
	case STS_MODE_CHG_LOOP:	/* CHG_LOOP does't exist */
		apply_ircomp = true;

		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "End1: rx_vol=%u",
					 wc68->ta_vol);
		} else if (wc68->ta_cur > WC68_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			wc68->ta_cur = wc68->ta_cur - PD_MSG_TA_CUR_STEP;

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "End2: ta_cur=%u, ta_vol=%u",
					wc68->ta_cur, wc68->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol - PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "End3: ta_cur=%u, ta_vol=%u",
					wc68->ta_cur, wc68->ta_vol);
		}

		wc68->prev_inc = INC_NONE;

		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CC_MODE);

		/* Send PD Message and then go to CC mode */
		wc68->charging_state = DC_STATE_CC_MODE;
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		vbatt = wc68_read_adc(wc68, ADCCH_VBAT);

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "End4: vbatt=%d, ta_vol=%u",
				vbatt, wc68->ta_vol);

		/* Clear TA increment flag */
		wc68->prev_inc = INC_NONE;
		/* Go to Pre-CV mode */
		wc68->timer_id = TIMER_ENTER_CVMODE;
		wc68->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:

		iin = wc68_read_adc(wc68, ADCCH_IIN);
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"Inactive: iin=%d, iin_cc=%d, cc_max=%d",
				iin, wc68->iin_cc, wc68->cc_max);
		if (iin < 0)
			break;

		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			ret = wc68_ajdust_ccmode_wireless(wc68, iin);
		} else {
			ret = wc68_ajdust_ccmode_wired(wc68, iin);
		}

		if (ret < 0) {
			dev_err(wc68->dev, "%s: %d", __func__, ret);
		} else {
			wc68->prev_iin = iin;
			apply_ircomp = true;
		}

		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = wc68_read_adc(wc68, ADCCH_VIN);

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "VIN_UVLO: ta_vol=%u, vin_vol=%d",
				wc68->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		wc68->timer_id = TIMER_ADJUST_CCMODE;
		wc68->timer_period = 1000;
		break;

	default:
		goto error;
	}

	if (!wc68->irdrop_comp_ok && apply_ircomp) {
		int rc;

		rc = wc68_comp_irdrop(wc68);
		if (rc < 0)
			dev_err(wc68->dev, "%s: cannot apply ircomp (%d)\n", __func__, rc);
	}

	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			 msecs_to_jiffies(wc68->timer_period));
error:
	mutex_unlock(&wc68->lock);
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* <0 error, 0 no new limits, >0 new limits */
static int wc68_apply_new_limits(struct wc68_charger *wc68)
{
	int ret = 0;

	if (wc68->new_iin && wc68->new_iin < wc68->iin_cc) {
		ret = wc68_apply_new_iin(wc68);
		if (ret == 0)
			ret = 1;
	} else if (wc68->new_vfloat) {
		ret = wc68_apply_new_vfloat(wc68);
		if (ret == 0)
			ret = 1;
	} else if (wc68->new_iin) {
		ret = wc68_apply_new_iin(wc68);
		if (ret == 0)
			ret = 1;
	} else {
		return 0;
	}

	return ret;
}

/* 2:1 Direct Charging CC MODE control */
static int wc68_charge_ccmode(struct wc68_charger *wc68)
{
	int ccmode, vin_vol, iin, ret = 0;
	bool apply_ircomp = false;

	dev_dbg(wc68->dev, "%s: ======START======= \n", __func__);

	mutex_lock(&wc68->lock);

	if (wc68->charging_state != DC_STATE_CC_MODE)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CC_MODE);

	wc68->charging_state = DC_STATE_CC_MODE;

	wc68_prlog_state(wc68, __func__);

	ret = wc68_check_error(wc68);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in VFLOAT here means that we have busted the tier, a
	 * change in iin means that the thermal engine had changed cc_max.
	 * wc68_apply_new_limits() changes wc68->charging_state to
	 * DC_STATE_ADJUST_TAVOL or DC_STATE_ADJUST_TACUR when new limits
	 * need to be applied.
	 */
	ret = wc68_apply_new_limits(wc68);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	ccmode = wc68_check_status(wc68);
	if (ccmode < 0) {
		ret = ccmode;
		goto error_exit;
	}

	switch(ccmode) {
	case STS_MODE_LOOP_INACTIVE:

		/* Set input current compensation */
		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Need RX voltage compensation */
			ret = wc68_set_rx_voltage_comp(wc68);

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "INACTIVE1: rx_vol=%u",
					wc68->ta_vol);
		} else {
			const int ta_max_vol = wc68->ta_max_vol;

			/* Check TA current with TA_MIN_CUR */
			if (wc68->ta_cur <= WC68_TA_MIN_CUR) {
				wc68->ta_cur = WC68_TA_MIN_CUR;

				ret = wc68_set_ta_voltage_comp(wc68);
			} else if (ta_max_vol >= WC68_TA_MAX_VOL_CP) {
				ret = wc68_set_ta_current_comp(wc68);
			} else {
				/* constant power mode */
				ret = wc68_set_ta_current_comp2(wc68);
			}

			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"INACTIVE2: ta_cur=%u, ta_vol=%u",
					wc68->ta_cur,
					wc68->ta_vol);
		}

		if (ret == 0)
			apply_ircomp = true;
		break;

	case STS_MODE_VFLT_LOOP:
		/* TODO: adjust fv_uv here based on real vbatt */

		iin = wc68_read_adc(wc68, ADCCH_IIN);
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "CC VFLOAT: iin=%d", iin);

		/* go to Pre-CV mode */
		wc68->timer_id = TIMER_ENTER_CVMODE;
		wc68->timer_period = 0;
		break;

	case STS_MODE_IIN_LOOP:
	case STS_MODE_CHG_LOOP:
		iin = wc68_read_adc(wc68, ADCCH_IIN);
		if (iin < 0)
			break;

		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"IIN_LOOP1: iin=%d, next_rx_vol=%u",
					iin, wc68->ta_vol);
		} else if (wc68->ta_cur <= WC68_TA_MIN_CUR) {
			/* Decrease TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"IIN_LOOP2: iin=%d, next_ta_vol=%u",
					iin, wc68->ta_vol);
		} else {
			/* Decrease TA current (50mA) */
			wc68->ta_cur = wc68->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"IIN_LOOP3: iin=%d, next_ta_cur=%u",
					iin, wc68->ta_cur);
		}

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = wc68_read_adc(wc68, ADCCH_VIN);

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				wc68->ta_cur, wc68->ta_vol, vin_vol);

		/* Check VIN after 1sec */
		wc68->timer_id = TIMER_CHECK_CCMODE;
		wc68->timer_period = 1000;
		break;

	default:
		break;
	}

	if (!wc68->irdrop_comp_ok && apply_ircomp) {
		int rc;

		rc = wc68_comp_irdrop(wc68);
		if (rc < 0)
			dev_err(wc68->dev, "%s: cannot apply ircomp (%d)\n",
			       __func__, rc);
	}

done:
	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			 msecs_to_jiffies(wc68->timer_period));

error_exit:
	mutex_unlock(&wc68->lock);
	dev_dbg(wc68->dev, "%s: End, ccmode=%d timer_id=%d, timer_period=%lu ret=%d\n",
		 __func__, ccmode, wc68->timer_id, wc68->timer_period,
		 ret);
	return ret;
}


/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int wc68_charge_start_cvmode(struct wc68_charger *wc68)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&wc68->lock);

	if (wc68->charging_state != DC_STATE_START_CV)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_START_CV);

	wc68->charging_state = DC_STATE_START_CV;

	/* Check the charging type */
	ret = wc68_check_error(wc68);
	if (ret != 0)
		goto error_exit;

	/* Check the status */
	cvmode = wc68_check_status(wc68);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	switch(cvmode) {
	case STS_MODE_CHG_LOOP:
	case STS_MODE_IIN_LOOP:

		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: PreCV IIN_LOOP: rx_vol=%u",
				 __func__, wc68->ta_vol);
		} else {
			/* Check TA current */
			if (wc68->ta_cur > WC68_TA_MIN_CUR) {
				/* TA current is higher than 1.0A */

				/* Decrease TA current (50mA) */
				wc68->ta_cur = wc68->ta_cur - PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"%s: PreCV IIN_LOOP: ta_cur=%u",
						__func__, wc68->ta_cur);
			} else {
				/* TA current is less than 1.0A */
				/* Decrease TA voltage (20mV) */
				wc68->ta_vol = wc68->ta_vol - PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
						"%s: PreCV IIN_LOOP: ta_vol=%u",
						__func__, wc68->ta_vol);
			}
		}

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Check the TA type */
		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: PreCV VF Cont: rx_vol=%u",
					__func__, wc68->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol -
					  WC68_TA_VOL_STEP_PRE_CV *
					  wc68->chg_mode;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: PreCV VF Cont: ta_vol=%u",
					__func__, wc68->ta_vol);
		}

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: PreCV End: ta_vol=%u, ta_cur=%u",
				__func__, wc68->ta_vol, wc68->ta_cur);

		/* Need to implement notification to other driver */
		/* To do here */

		/* Go to CV mode */
		wc68->timer_id = TIMER_CHECK_CVMODE;
		wc68->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = wc68_read_adc(wc68, ADCCH_VIN);

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: PreCV VIN_UVLO: ta_vol=%u, vin_vol=%u",
				__func__, wc68->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		wc68->timer_id = TIMER_ENTER_CVMODE;
		wc68->timer_period = 1000;
		break;

	default:
		break;
	}

	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			 msecs_to_jiffies(wc68->timer_period));
error_exit:
	mutex_unlock(&wc68->lock);
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int wc68_check_eoc(struct wc68_charger *wc68)
{
	const int eoc_tolerance = 25000; /* 25mV under max float voltage */
	const int vlimit = WC68_COMP_VFLOAT_MAX - eoc_tolerance;
	int iin, vbat;

	iin = wc68_read_adc(wc68, ADCCH_IIN);
	if (iin < 0) {
		dev_err(wc68->dev, "%s: iin=%d\n", __func__, iin);
		return iin;
	}

	vbat = wc68_read_adc(wc68, ADCCH_VBAT);
	if (vbat < 0) {
		dev_err(wc68->dev, "%s: vbat=%d\n", __func__, vbat);
		return vbat;
	}

	dev_dbg(wc68->dev, "%s: iin=%d, topoff=%u, vbat=%d vlimit=%d\n", __func__,
		 iin, wc68->pdata->iin_topoff,
		 vbat, vlimit);

	return iin < wc68->pdata->iin_topoff && vbat >= vlimit;
}

/* 2:1 Direct Charging CV MODE control */
static int wc68_charge_cvmode(struct wc68_charger *wc68)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&wc68->lock);

	if (wc68->charging_state != DC_STATE_CV_MODE)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CV_MODE);

	wc68->charging_state = DC_STATE_CV_MODE;

	ret = wc68_check_error(wc68);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in vfloat and cc_max here is a normal tier transition, a
	 * change in iin  means that the thermal engine has changed cc_max.
	 */
	ret = wc68_apply_new_limits(wc68);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	cvmode = wc68_check_status(wc68);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	if (cvmode == STS_MODE_LOOP_INACTIVE) {
		ret = wc68_check_eoc(wc68);
		if (ret < 0)
			goto error_exit;
		if (ret)
			cvmode = STS_MODE_CHG_DONE;
	}

	switch(cvmode) {
	case STS_MODE_CHG_DONE: {
		const bool done_already = wc68->charging_state ==
					  DC_STATE_CHARGING_DONE;

		if (!done_already)
			dev_info(wc68->dev, "%s: charging_state=%u->%u\n",
				 __func__, wc68->charging_state,
				 DC_STATE_CHARGING_DONE);


		/* Keep CV mode until driver send stop charging */
		wc68->charging_state = DC_STATE_CHARGING_DONE;
		power_supply_changed(wc68->mains);

		/* _cpm already came in */
		if (wc68->charging_state == DC_STATE_NO_CHARGING) {
			dev_dbg(wc68->dev, "%s: Already stop DC\n", __func__);
			break;
		}

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: done_already=%d charge Done\n", __func__,
				done_already);

		wc68->timer_id = TIMER_CHECK_CVMODE;
		wc68->timer_period = WC68_CVMODE_CHECK_T;
	} break;

	case STS_MODE_CHG_LOOP:
	case STS_MODE_IIN_LOOP:
		/* Check the TA type */
		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX Voltage (100mV) */
			wc68->ta_vol = wc68->ta_vol -
						WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: rx_vol=%u",
					__func__, wc68->ta_vol);

		/* Check TA current */
		} else if (wc68->ta_cur > WC68_TA_MIN_CUR) {
			/* TA current is higher than (1.0A*chg_mode) */
			/* Decrease TA current (50mA) */
			wc68->ta_cur = wc68->ta_cur -
						PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_cur=%u",
					__func__, wc68->ta_cur);
		} else {
			/* TA current is less than (1.0A*chg_mode) */
			/* Decrease TA Voltage (20mV) */
			wc68->ta_vol = wc68->ta_vol -
						PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_vol=%u",
					__func__, wc68->ta_vol);
		}

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Check the TA type */
		if (wc68->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage */
			wc68->ta_vol = wc68->ta_vol -
						WCRX_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: CV VFLOAT, Cont: rx_vol=%u",
					__func__, wc68->ta_vol);
		} else {
			/* Decrease TA voltage */
			wc68->ta_vol = wc68->ta_vol -
						PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
					"%s: CV VFLOAT, Cont: ta_vol=%u",
					__func__, wc68->ta_vol);
		}

		/* Send PD Message */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		wc68->timer_id = TIMER_CHECK_CVMODE;
		wc68->timer_period = WC68_CVMODE_CHECK_T;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = wc68_read_adc(wc68, ADCCH_VIN);
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: CC VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				__func__, wc68->ta_cur, wc68->ta_vol,
				vin_vol);

		/* Check VIN after 1sec */
		wc68->timer_id = TIMER_CHECK_CVMODE;
		wc68->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	dev_dbg(wc68->dev, "%s: reschedule next id=%d period=%ld chg_state=%d\n",
		 __func__, wc68->timer_id, wc68->timer_period,
		wc68->charging_state);

	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			 msecs_to_jiffies(wc68->timer_period));
error_exit:
	mutex_unlock(&wc68->lock);
	dev_dbg(wc68->dev, "%s: End, ret=%d next\n", __func__, ret);
	return ret;
}

/*
 * Preset TA voltage and current for Direct Charging Mode using
 * the configured cc_max and fv_uv limits. Used only on start
 */
static int wc68_preset_dcmode(struct wc68_charger *wc68)
{
	int vbat;
	int ret = 0;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);
	dev_dbg(wc68->dev, "%s: = charging_state=%u == \n", __func__,
		 wc68->charging_state);

	/* gcpm set ->cc_max and ->fv_uv before starting */
	if (wc68->cc_max < 0 || wc68->fv_uv < 0) {
		dev_err(wc68->dev, "%s: cc_max=%d fv_uv=%d invalid\n", __func__,
		       wc68->cc_max, wc68->fv_uv);
		return -EINVAL;
	}

	if (wc68->charging_state != DC_STATE_PRESET_DC)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_PRESET_DC);

	wc68->charging_state = DC_STATE_PRESET_DC;

	/* VBAT is over threshold but it might be "bouncy" due to transitory */
	vbat = wc68_read_adc(wc68, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* v_float is set on start from GCPM */
	if (vbat > wc68->fv_uv) {
		dev_err(wc68->dev, "%s: vbat adc=%d is higher than VFLOAT=%d\n", __func__,
			vbat, wc68->fv_uv);
		ret = -EINVAL;
		goto error;
	}

	/* determined by ->cfg_iin and cc_max */
	wc68->ta_max_cur = wc68_get_iin_max(wc68, wc68->cc_max);
	dev_dbg(wc68->dev, "%s: ta_max_cur=%u, iin_cfg=%u, wc68->ta_type=%d\n",
		 __func__, wc68->ta_max_cur, wc68->pdata->iin_cfg,
		 wc68->ta_type);

	/* Check the TA type and set the charging mode */
	if (wc68->ta_type == TA_TYPE_WIRELESS) {
		/*
		 * Set the RX max voltage to enough high value to find RX
		 * maximum voltage initially
		 */
		wc68->ta_max_vol = WC68_WCRX_MAX_VOL * wc68->chg_mode;

		/* Get the RX max current/voltage(RX_MAX_CUR/VOL) */
		ret = wc68_get_rx_max_power(wc68);
		if (ret < 0) {
			dev_err(wc68->dev, "%s: no RX voltage to support 4:1 (%d)\n",
				__func__, ret);
			wc68->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		ret = wc68_set_wireless_dc(wc68, vbat);
		if (ret < 0) {
			dev_err(wc68->dev, "%s: set wired failed (%d)\n", __func__, ret);
			wc68->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		logbuffer_prlog(wc68, LOGLEVEL_INFO,
				"Preset DC, rx_max_vol=%u, rx_max_cur=%u, rx_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
				wc68->ta_max_vol, wc68->ta_max_cur, wc68->ta_max_pwr,
				wc68->iin_cc, wc68->chg_mode);
	} else {
		const unsigned int ta_max_vol = WC68_TA_MAX_VOL * wc68->chg_mode;

		/*
		 * Get the APDO max for 2:1 mode.
		 * Returns ->ta_max_vol, ->ta_max_cur, ->ta_max_pwr and
		 * ->ta_objpos for the given ta_max_vol and ta_max_cur.
		 */
		ret = wc68_get_apdo_max_power(wc68, ta_max_vol, WC68_TA_MAX_CUR);
		if (ret < 0) {
			dev_warn(wc68->dev, "%s: No APDO to support 2:1 for %d\n", __func__,
				WC68_TA_MAX_CUR);
			ret = wc68_get_apdo_max_power(wc68, ta_max_vol, 0);
		}
		if (ret < 0) {
			dev_err(wc68->dev, "%s: No APDO to support 2:1\n", __func__);
			wc68->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		/*
		 * ->ta_max_cur is too high for startup, needs to target
		 * CC before hitting max current AND work to ta_max_cur
		 * from there.
		 */
		ret = wc68_set_wired_dc(wc68, vbat);
		if (ret < 0) {
			dev_err(wc68->dev, "%s: set wired failed (%d)\n", __func__, ret);
			wc68->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		logbuffer_prlog(wc68, LOGLEVEL_INFO,
				"Preset DC, objpos=%d ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
				wc68->ta_objpos, wc68->ta_max_vol, wc68->ta_max_cur,
				wc68->ta_max_pwr, wc68->iin_cc, wc68->chg_mode);

	}

error:
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Preset direct charging configuration and start charging */
static int wc68_preset_config(struct wc68_charger *wc68)
{
	int ret = 0;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&wc68->lock);

	dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			wc68->charging_state, DC_STATE_PRESET_DC);

	wc68->charging_state = DC_STATE_PRESET_DC;

	/* ->iin_cc and ->fv_uv are configured externally */
	ret = wc68_set_input_current(wc68, wc68->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	ret = wc68_set_vfloat(wc68, wc68->fv_uv);
	if (ret < 0)
		goto error;

	/* Enable WC68 unless aready enabled */
	ret = wc68_set_charging(wc68, true);
	if (ret < 0)
		goto error;

	/* Clear previous iin adc */
	wc68->prev_iin = 0;
	wc68->prev_inc = INC_NONE;

	/* Go to CHECK_ACTIVE state after 150ms, 300ms for wireless */
	wc68->timer_id = TIMER_CHECK_ACTIVE;
	if (wc68->ta_type == TA_TYPE_WIRELESS)
		wc68->timer_period = WC68_ENABLE_WLC_DELAY_T;
	else
		wc68->timer_period = WC68_ENABLE_DELAY_T;
	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			   msecs_to_jiffies(wc68->timer_period));
error:
	mutex_unlock(&wc68->lock);
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * Check the charging status at start before entering the adjust cc mode or
 * from wc68_send_message() after a failure.
 */
static int wc68_check_active_state(struct wc68_charger *wc68)
{
	int ret = 0;

	dev_dbg(wc68->dev, "%s: ======START=======\n", __func__);
	dev_dbg(wc68->dev, "%s: = charging_state=%u == \n", __func__,
		 wc68->charging_state);

	mutex_lock(&wc68->lock);

	if (wc68->charging_state != DC_STATE_CHECK_ACTIVE)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CHECK_ACTIVE);

	wc68->charging_state = DC_STATE_CHECK_ACTIVE;

	ret = wc68_check_error(wc68);
	if (ret == 0) {
		/* WC68 is active state */
		wc68->retry_cnt = 0;
		wc68->timer_id = TIMER_ADJUST_CCMODE;
		wc68->timer_period = 0;
	} else if (ret == -EAGAIN) {

		/* try restarting only */
		if (wc68->retry_cnt >= WC68_MAX_RETRY_CNT) {
			dev_err(wc68->dev, "%s: retry failed\n", __func__);
			ret = -EINVAL;
			goto exit_done;
		}

		/*
		 * Disable charging to retry enabling it later, return 0 here
		 * and the timer loop will figure out that there is something
		 * wrong and will retry.
		 */
		ret = wc68_set_charging(wc68, false);
		dev_err(wc68->dev, "%s: retry cnt=%d, (%d)\n", __func__,
		       wc68->retry_cnt, ret);
		if (ret == 0) {
			wc68->timer_id = TIMER_PRESET_DC;
			wc68->timer_period = 0;
			wc68->retry_cnt++;
		}
	}

exit_done:

	/* Implement error handler function if it is needed */
	if (ret < 0) {
		logbuffer_prlog(wc68, LOGLEVEL_ERR,
				"%s: charging_state=%d, not active or error (%d)",
				__func__, wc68->charging_state, ret);
		wc68->timer_id = TIMER_ID_NONE;
		wc68->timer_period = 0;
	}

	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			 msecs_to_jiffies(wc68->timer_period));
	mutex_unlock(&wc68->lock);
	return ret;
}

/* Enter direct charging algorithm */
static int wc68_start_direct_charging(struct wc68_charger *wc68)
{
	struct wc68_chg_stats *chg_data = &wc68->chg_data;
	int ret;
	u16 cmd[2];

	dev_dbg(wc68->dev, "%s: =========START=========\n", __func__);
	mutex_lock(&wc68->lock);

	/* Enable CBUS_UCP prot for unplug detection */
	cmd[0] = cpu_to_be16(PROT_EN_0);
	cmd[1] = PROT_EN_0_CBUS_UCP_EN;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "%s: Error enabling UCP protection: %d\n",
			__func__, ret);
		goto error_done;
	}

	/* configure DC charging type for the requested index */
	ret = wc68_set_ta_type(wc68, wc68->pps_index);
	dev_info(wc68->dev, "%s: Current ta_type=%d, chg_mode=%d\n", __func__,
		wc68->ta_type, wc68->chg_mode);
	if (ret < 0)
		goto error_done;

	/* wake lock */
	__pm_stay_awake(wc68->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = wc68_preset_dcmode(wc68);
	if (ret == 0) {
		/* Configure the TA  and start charging */
		wc68->timer_id = TIMER_PDMSG_SEND;
		wc68->timer_period = 0;

		mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
				 msecs_to_jiffies(wc68->timer_period));
	}

error_done:
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);

	wc68_chg_stats_update(chg_data, wc68);
	mutex_unlock(&wc68->lock);
	return ret;
}

/* Check Vbat minimum level to start direct charging */
static int wc68_check_vbatmin(struct wc68_charger *wc68)
{
	int ret = 0, vbat;

	dev_dbg(wc68->dev, "%s: =========START=========\n", __func__);

	mutex_lock(&wc68->lock);

	if (wc68->charging_state != DC_STATE_CHECK_VBAT)
		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CHECK_VBAT);

	wc68->charging_state = DC_STATE_CHECK_VBAT;

	vbat = wc68_read_adc(wc68, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* wait for hw init and CPM to send in the params */
	if (wc68->cc_max < 0 || wc68->fv_uv < 0 || !wc68->hw_init_done) {
		dev_info(wc68->dev, "%s: not yet fv_uv=%d, cc_max=%d vbat=%d, hw_init_done=%d\n",
			 __func__, wc68->fv_uv, wc68->cc_max, vbat, wc68->hw_init_done);

		/* retry again after 1sec */
		wc68->timer_id = TIMER_VBATMIN_CHECK;
		wc68->timer_period = WC68_VBATMIN_CHECK_T;
		wc68->retry_cnt += 1;
	} else {
		logbuffer_prlog(wc68, LOGLEVEL_INFO,
				"%s: starts at fv_uv=%d, cc_max=%d vbat=%d (min=%d)",
				__func__, wc68->fv_uv, wc68->cc_max, vbat,
				WC68_DC_VBAT_MIN);

		wc68->timer_id = TIMER_PRESET_DC;
		wc68->timer_period = 0;
		wc68->retry_cnt = 0; /* start charging */
	}

	/* timeout for VBATMIN or charging parameters */
	if (wc68->retry_cnt > WC68_MAX_RETRY_CNT) {
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: TIMEOUT fv_uv=%d, cc_max=%d vbat=%d limit=%d",
				__func__, wc68->fv_uv, wc68->cc_max, vbat,
				WC68_DC_VBAT_MIN);
		ret = -ETIMEDOUT;
	} else {
		mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
				 msecs_to_jiffies(wc68->timer_period));
	}


error:
	mutex_unlock(&wc68->lock);
	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int wc68_send_message(struct wc68_charger *wc68)
{
	int val, ret;
	const int timer_id = wc68->timer_id;

	/* Go to the next state */
	mutex_lock(&wc68->lock);

	dev_dbg(wc68->dev, "%s: ====== START ======= \n", __func__);

	/* Adjust TA current and voltage step */
	if (wc68->ta_type == TA_TYPE_WIRELESS) {
		/* RX voltage resolution is 100mV */
		val = wc68->ta_vol / WCRX_VOL_STEP;
		wc68->ta_vol = val * WCRX_VOL_STEP;

		/* Set RX voltage */
		dev_dbg(wc68->dev, "%s: ta_type=%d, ta_vol=%d\n", __func__,
			 wc68->ta_type, wc68->ta_vol);
		ret = wc68_send_rx_voltage(wc68, WCRX_REQUEST_VOLTAGE);
	} else {
		/* PPS voltage resolution is 20mV */
		val = wc68->ta_vol / PD_MSG_TA_VOL_STEP;
		wc68->ta_vol = val * PD_MSG_TA_VOL_STEP;
		/* PPS current resolution is 50mA */
		val = wc68->ta_cur / PD_MSG_TA_CUR_STEP;
		wc68->ta_cur = val * PD_MSG_TA_CUR_STEP;
		/* PPS minimum current is 1000mA */
		if (wc68->ta_cur < WC68_TA_MIN_CUR)
			wc68->ta_cur = WC68_TA_MIN_CUR;

		dev_dbg(wc68->dev, "%s: ta_type=%d, ta_vol=%d ta_cur=%d\n", __func__,
			 wc68->ta_type, wc68->ta_vol, wc68->ta_cur);

		/* Send PD Message */
		ret = wc68_send_pd_message(wc68, PD_MSG_REQUEST_APDO);
	}

	switch (wc68->charging_state) {
	case DC_STATE_PRESET_DC:
		wc68->timer_id = TIMER_PRESET_CONFIG;
		break;
	case DC_STATE_ADJUST_CC:
		wc68->timer_id = TIMER_ADJUST_CCMODE;
		break;
	case DC_STATE_CC_MODE:
		wc68->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_START_CV:
		wc68->timer_id = TIMER_ENTER_CVMODE;
		break;
	case DC_STATE_CV_MODE:
		wc68->timer_id = TIMER_CHECK_CVMODE;
		break;
	case DC_STATE_ADJUST_TAVOL:
		wc68->timer_id = TIMER_ADJUST_TAVOL;
		break;
	case DC_STATE_ADJUST_TACUR:
		wc68->timer_id = TIMER_ADJUST_TACUR;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error-send_pd_message to %d (%d)\n",
			__func__, wc68->ta_type, ret);
		wc68->timer_id = TIMER_CHECK_ACTIVE;
	}

	if (wc68->ta_type == TA_TYPE_WIRELESS)
		wc68->timer_period = WC68_PDMSG_WLC_WAIT_T;
	else
		wc68->timer_period = WC68_PDMSG_WAIT_T;

	logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
			"%s: charging_state=%u timer_id:%d->%d ret=%d",
			__func__, wc68->charging_state,
			timer_id, wc68->timer_id, ret);

	mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
			 msecs_to_jiffies(wc68->timer_period));

	dev_dbg(wc68->dev, "%s: End: timer_id=%d timer_period=%lu\n", __func__,
		 wc68->timer_id, wc68->timer_period);

	mutex_unlock(&wc68->lock);
	return ret;
}

/* delayed work function for charging timer */
static void wc68_timer_work(struct work_struct *work)
{
	struct wc68_charger *wc68 =
		container_of(work, struct wc68_charger, timer_work.work);
	unsigned int charging_state;
	int timer_id;
	int ret = 0;

	dev_dbg(wc68->dev, "%s: ========= START =========\n", __func__);

	/* TODO: remove locks from the calls and run all of this locked */
	mutex_lock(&wc68->lock);

	wc68_chg_stats_update(&wc68->chg_data, wc68);
	charging_state = wc68->charging_state;
	timer_id = wc68->timer_id;

	dev_dbg(wc68->dev, "%s: timer id=%d, charging_state=%u\n", __func__,
		 wc68->timer_id, charging_state);

	mutex_unlock(&wc68->lock);

	switch (timer_id) {

	/* charging_state <- DC_STATE_CHECK_VBAT */
	case TIMER_VBATMIN_CHECK:
		ret = wc68_check_vbatmin(wc68);
		if (ret < 0)
			goto error;
		break;

	/* charging_state <- DC_STATE_PRESET_DC */
	case TIMER_PRESET_DC:
		ret = wc68_start_direct_charging(wc68);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	preset configuration, start charging
	 */
	case TIMER_PRESET_CONFIG:
		ret = wc68_preset_config(wc68);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	150 ms after preset_config
	 */
	case TIMER_CHECK_ACTIVE:
		ret = wc68_check_active_state(wc68);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ADJUST_CCMODE:
		ret = wc68_charge_adjust_ccmode(wc68);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CCMODE:
		ret = wc68_charge_ccmode(wc68);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = wc68_charge_start_cvmode(wc68);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CVMODE:
		ret = wc68_charge_cvmode(wc68);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		ret = wc68_send_message(wc68);
		if (ret < 0)
			goto error;
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TAVOL:
		mutex_lock(&wc68->lock);

		if (wc68->ta_type == TA_TYPE_WIRELESS)
			ret = wc68_adjust_rx_voltage(wc68);
		else
			ret = wc68_adjust_ta_voltage(wc68);
		if (ret < 0) {
			mutex_unlock(&wc68->lock);
			goto error;
		}

		mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
				 msecs_to_jiffies(wc68->timer_period));
		mutex_unlock(&wc68->lock);
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TACUR:
		mutex_lock(&wc68->lock);
		ret = wc68_adjust_ta_current(wc68);
		if (ret < 0) {
			mutex_unlock(&wc68->lock);
			goto error;
		}

		mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
				 msecs_to_jiffies(wc68->timer_period));
		mutex_unlock(&wc68->lock);
		break;

	case TIMER_ID_NONE:
		ret = wc68_stop_charging(wc68);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	/* Check the charging state again */
	if (wc68->charging_state == DC_STATE_NO_CHARGING) {
		cancel_delayed_work(&wc68->timer_work);
		cancel_delayed_work(&wc68->pps_work);
	}

	dev_dbg(wc68->dev, "%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld\n",
		 __func__, timer_id, wc68->timer_id, charging_state,
		 wc68->charging_state, wc68->timer_period);

	return;

error:
	dev_dbg(wc68->dev, "%s: ========= ERROR =========\n", __func__);
	logbuffer_prlog(wc68, LOGLEVEL_ERR,
			"%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld ret=%d",
			__func__, timer_id, wc68->timer_id, charging_state,
			wc68->charging_state, wc68->timer_period, ret);

	if (!wc68->dc_avail)
		wc68->dc_avail = gvotable_election_get_handle(VOTABLE_DC_CHG_AVAIL);

	if (wc68->dc_avail) {
		ret = gvotable_cast_int_vote(wc68->dc_avail, REASON_DC_DRV, 0, 1);
		if (ret < 0)
			dev_err(wc68->dev, "Unable to cast vote for DC Chg avail (%d)\n", ret);
	}

	wc68_stop_charging(wc68);
}

/* delayed work function for resetting DC chip */
static void wc68_init_hw_work(struct work_struct *work)
{
	struct wc68_charger *wc68 = container_of(work,
					struct wc68_charger, init_hw_work.work);
	int ret;

	ret = wc68_hw_init(wc68);
	if (ret) {
		dev_err(wc68->dev, "Error initializing hw %d\n", ret);
		goto error;
	}

	wc68->hw_init_done = true;

error:
	return;
}

/* delayed work function for pps periodic timer */
static void wc68_pps_request_work(struct work_struct *work)
{
	struct wc68_charger *wc68 = container_of(work,
					struct wc68_charger, pps_work.work);
	int ret;

	dev_dbg(wc68->dev, "%s: =========START=========\n", __func__);
	dev_dbg(wc68->dev, "%s: = charging_state=%u == \n", __func__,
		 wc68->charging_state);

	ret = wc68_send_pd_message(wc68, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		dev_err(wc68->dev, "%s: Error-send_pd_message\n", __func__);

	/* TODO: do other background stuff */

	dev_dbg(wc68->dev, "%s: ret=%d\n", __func__, ret);
}

int wc68_hw_ping(struct wc68_charger *wc68)
{
	int ret;
	u16 read_buff[1];
	u16 cmd = cpu_to_be16(FWREG_CHIP_ID_REG);
	u16 val;

	ret = wc68_i2c_read(wc68->client, &cmd, 2, read_buff, 2);
	if (ret) {
		dev_err(wc68->dev, "STWC68 Error reading version\n");
		return -EIO;
	}
	val = read_buff[0];
	dev_err(wc68->dev, "STWC68 version: %#04X\n", val);

	return 0;
}

static int wc68_hw_init(struct wc68_charger *wc68)
{
	int ret;
	u16 cmd[3];
	u8 val;
	int retries = 20;

	/* Reset the chip */
	cmd[0] = cpu_to_be16(SYS_CMD);
	cmd[1] = SYS_CMD_SYS_RESET;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "STWC68 Error resetting chip: %d\n", ret);
		goto error_done;
	}

	/* Wait for boot up after reset */
	do {
		u16 cmd2 = cpu_to_be16(INTR_FLG_3);
		msleep(100);
		ret = wc68_i2c_read(wc68->client, &cmd2, 2, &val, 1);
	} while (retries-- && !ret && !(val & INTR_FLG_3_BOOTUP_RDY));
	if (ret) {
		dev_err(wc68->dev, "STWC68 Error reading boot up flag %d\n", ret);
		goto error_done;
	}

	if(retries == 0) {
		dev_err(wc68->dev, "STWC68 retries exhausted waiting for chip boot up\n");
		ret = -EIO;
		goto error_done;
	}

	if (wc68->pdata->irq_gpio >= 0) {
		ret = wc68_irq_init(wc68, wc68->client);
		if (ret < 0)
			dev_warn(wc68->dev, "%s: failed to initialize IRQ: %d\n", __func__, ret);
		else
			disable_irq(wc68->client->irq);
	}

	/* Set switching frequency and deadtime */
	cmd[0] = cpu_to_be16(SYS_CFG_1);
	cmd[1] = 0xB9;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "%s: Error writing Switching frequency: %d\n",
			__func__, ret);
		goto error_done;
	}

	/* Set safety switch drive voltage 5V */
	cmd[0] = cpu_to_be16(SYS_CFG_3);
	cmd[1] = 0xFD;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "%s: Error setting Safety SW drv voltage: %d\n",
			__func__, ret);
		goto error_done;
	}

	/* Disable all protections */
	cmd[0] = cpu_to_be16(PROT_EN_0);
	cmd[1] = cmd[2] = 0;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 6);
	if (ret) {
		dev_err(wc68->dev, "%s: Error disabling protections: %d\n",
			__func__, ret);
		goto error_done;
	}

	/* Enable the protections we need */
	cmd[0] = cpu_to_be16(PROT_EN_1);
	cmd[1] = PROT_EN_1_VBAT_OVP_WARN_EN | PROT_EN_1_CBUS_OCP_WARN_EN;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "%s: Error enabling protections: %d\n",
			__func__, ret);
		goto error_done;
	}

	/* Configure UCP threshold for unplug detection */
	cmd[0] = cpu_to_be16(CBUS_UCP_THRES);
	cmd[1] = CBUS_UCP_DFT / CBUS_UCP_STEP;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 4);
	if (ret) {
		dev_err(wc68->dev, "%s: Error setting CBUS_UCP_THRES: %d\n",
			__func__, ret);
		goto error_done;
	}

	/* Enable ADC */
	cmd[0] = cpu_to_be16(ADC_CTRL);
	cmd[1] = ADC_CTRL_CONT_EN;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "STWC68 Error enabling ADC: %d\n", ret);
		goto error_done;
	}
	msleep(200);

	/* Set work mode 2:1 */
	cmd[0] = cpu_to_be16(SYS_CMD);
	cmd[1] = (SYS_CMD_SW_CTRL_USB | SYS_CMD_MODE_CTRL_2_1);
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret < 0) {
		dev_err(wc68->dev, "Error enabling 2:1 err: %d\n", ret);
		goto error_done;
	}

error_done:
	return ret;
}

static irqreturn_t wc68_interrupt_handler(int irq, void *data)
{
	struct wc68_charger *wc68 = data;
	bool handled = true;
	int ret;
	u16 cmd[2];
	u8 val[5];

	cmd[0] = cpu_to_be16(INTR_FLG_0);
	ret = wc68_i2c_read(wc68->client, &cmd[0], 2, &val[0], 5);
	if (ret) {
		dev_err(wc68->dev, "%s: Error reading INTR_FLG_0: %d\n",
			__func__, ret);
		goto exit_done;
	}
	dev_dbg(wc68->dev, "%s: FLG %d, %d, %d, %d, %d\n", __func__,
		val[0], val[1], val[2], val[3], val[4]);

	if (val[0] & VBAT_OVP_INTR_MSK) {
		dev_info(wc68->dev, "%s: In VFLT LOOP\n", __func__);
	}

	if (val[1] & CBUS_OCP_INTR_MSK) {
		dev_info(wc68->dev, "%s: In IIN LOOP\n", __func__);
	}

	cmd[0] = cpu_to_be16(INTR_EN_0);
	ret = wc68_i2c_read(wc68->client, &cmd[0], 2, &val[0], 5);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error reading interrupts enable: %d\n",
			__func__, ret);
		goto exit_done;
	}
	dev_dbg(wc68->dev, "%s: Interrupt enables: %d, %d, %d, %d, %d\n",
		__func__, val[0], val[1], val[2], val[3], val[4]);

	/* Clear the interrupts */
	cmd[0] = cpu_to_be16(INTR_CLR_0);
	cmd[1] = 0;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 3);
	if (ret) {
		dev_err(wc68->dev, "%s: Error clearing INTR_FLG_0: %d\n",
			__func__, ret);
		goto exit_done;
	}

exit_done:
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int wc68_irq_init(struct wc68_charger *wc68,
			    struct i2c_client *client)
{
	const struct wc68_platform_data *pdata = wc68->pdata;
	int ret, irq;
	u16 cmd[4];
	u8 val[5];

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, wc68_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, wc68);
	if (ret < 0)
		goto fail_gpio;


	cmd[0] = cpu_to_be16(INTR_EN_0);
	cmd[1] = VBAT_OVP_INTR_MSK | CBUS_OCP_INTR_MSK;
	cmd[2] = 0;
	cmd[3] = 0;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 7);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error enabling interrupts: %d\n",
			__func__, ret);
		goto fail_write;
	}

	cmd[0] = cpu_to_be16(INTR_EN_0);
	ret = wc68_i2c_read(wc68->client, &cmd[0], 2, &val[0], 5);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error reading interrupts enable: %d\n",
			__func__, ret);
		goto fail_write;
	}
	dev_info(wc68->dev, "%s: Interrupt enables: %d, %d, %d, %d, %d\n",
		__func__, val[0], val[1], val[2], val[3], val[4]);

	cmd[0] = cpu_to_be16(INTR_CLR_0);
	cmd[1] = cmd[2] = cmd[3] = 0;
	ret = wc68_i2c_write(wc68->client, &cmd[0], 7);
	if (ret < 0) {
		dev_err(wc68->dev, "%s: Error clearing interrupts: %d\n",
			__func__, ret);
		goto fail_write;
	}

	client->irq = irq;
	return 0;

fail_write:
	free_irq(irq, wc68);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;
	return ret;
}

/* Returns the input current limit programmed into the charger in uA. */
int wc68_input_current_limit(struct wc68_charger *wc68)
{
	int ret;
	u16 val;
	u16 cmd = cpu_to_be16(CBUS_OCP_WARN_THRES);

	if (!wc68->mains_online)
		return -ENODATA;

	ret = wc68_i2c_read(wc68->client, &cmd, 2, &val, 2);
	if (ret < 0) {
		dev_err(wc68->dev, "Error reading CBUS_OCP_WARN_THRES: %d\n", ret);
		return ret;
	}

	val = val * WC68_IIN_CFG_STEP;
	return val;
}

/* Returns the constant charge current requested from GCPM */
static int get_const_charge_current(struct wc68_charger *wc68)
{
	/* Charging current cannot be controlled directly */
	return wc68->cc_max;
}

/* Return the constant charge voltage programmed into the charger in uV. */
static int wc68_const_charge_voltage(struct wc68_charger *wc68)
{
	u16 val;
	int conv;
	int ret;
	u16 cmd = cpu_to_be16(VBAT_OVP_WARN_THRES);

	if (!wc68->mains_online)
		return -ENODATA;

	ret = wc68_i2c_read(wc68->client, &cmd, 2, &val, 2);
	if (ret < 0) {
		dev_err(wc68->dev, "Error reading VBAT_OVP_WARN_THRES: %d\n", ret);
		return ret;
	}

	conv = val * VFLOAT_STEP;
	return conv;
}

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

/* index is the PPS source to use */
static int wc68_set_charging_enabled(struct wc68_charger *wc68, int index)
{
	if (index < 0 || index >= PPS_INDEX_MAX)
		return -EINVAL;

	mutex_lock(&wc68->lock);

	/* Done is detected in CV when iin goes UNDER topoff. */
	if (wc68->charging_state == DC_STATE_CHARGING_DONE)
		index = 0;

	if (index == 0) {

		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: stop pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, wc68->pps_index, index,
				wc68->charging_state,
				wc68->timer_id);

		/* this is the same as stop charging */
		wc68->pps_index = 0;

		cancel_delayed_work(&wc68->timer_work);
		cancel_delayed_work(&wc68->pps_work);

		/* will call wc68_stop_charging() in timer_work() */
		wc68->timer_id = TIMER_ID_NONE;
		wc68->timer_period = 0;
		mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
				 msecs_to_jiffies(wc68->timer_period));
	} else if (wc68->charging_state == DC_STATE_NO_CHARGING) {
		logbuffer_prlog(wc68, LOGLEVEL_DEBUG,
				"%s: start pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, wc68->pps_index, index,
				wc68->charging_state,
				wc68->timer_id);

		/* Start Direct Charging on Index */
		wc68->dc_start_time = get_boot_sec();
		wc68_chg_stats_init(&wc68->chg_data);
		wc68->irdrop_comp_ok = false;
		wc68->pps_index = index;

		dev_info(wc68->dev, "%s: charging_state=%u->%u\n", __func__,
			 wc68->charging_state, DC_STATE_CHECK_VBAT);

		/* PD is already in PE_SNK_STATE */
		wc68->charging_state = DC_STATE_CHECK_VBAT;
		wc68->timer_id = TIMER_VBATMIN_CHECK;
		wc68->timer_period = 0;
		mod_delayed_work(wc68->dc_wq, &wc68->timer_work,
				 msecs_to_jiffies(wc68->timer_period));

		/* Set the initial charging step */
		power_supply_changed(wc68->mains);
	}

	mutex_unlock(&wc68->lock);

	return 0;
}

static int wc68_mains_set_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      const union power_supply_propval *val)
{
	struct wc68_charger *wc68 = power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(wc68->dev, "%s: =========START=========\n", __func__);
	dev_dbg(wc68->dev, "%s: prop=%d, val=%d\n", __func__, prop, val->intval);
	if (!wc68->init_done)
		return -EAGAIN;

	switch (prop) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval == 0) {
			ret = wc68_stop_charging(wc68);
			if (ret < 0)
				dev_err(wc68->dev, "%s: cannot stop charging (%d)\n",
				       __func__, ret);

			wc68->mains_online = false;
		} else if (wc68->mains_online == false) {
			wc68->mains_online = true;
		}

		break;

	/* TODO: locking is wrong */
	case GBMS_PROP_CHARGING_ENABLED:
		ret = wc68_set_charging_enabled(wc68, val->intval);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = wc68_set_new_vfloat(wc68, val->intval);
		break;

	/*
	 * dc charger cannot control charging current directly so need to control
	 * current on TA side resolving cc_max for TA_VOL*TA_CUT on vbat.
	 * NOTE: iin should be equivalent to iin = cc_max /2
	 */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = wc68_set_new_cc_max(wc68, val->intval);
		break;

	/* CURRENT MAX, same as IIN is really only set by the algo */
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		dev_dbg(wc68->dev, "%s: set iin %d, ignore\n", __func__, val->intval);
		break;

	/* allow direct setting, not used */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&wc68->lock);
		ret = wc68_set_new_iin(wc68, val->intval);
		mutex_unlock(&wc68->lock);
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		break;

	default:
		ret = -EINVAL;
		break;
	}

	dev_dbg(wc68->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int wc68_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct wc68_charger *wc68 = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int intval, rc, ret = 0;

	if (!wc68->init_done)
		return -EAGAIN;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = wc68->mains_online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = wc68_is_present(wc68);
		if (val->intval < 0)
			val->intval = 0;
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		ret = wc68_get_charging_enabled(wc68);
		if (ret < 0)
			return ret;
		val->intval = !ret;
		break;

	case GBMS_PROP_CHARGING_ENABLED:
		ret = wc68_get_charging_enabled(wc68);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = wc68_const_charge_voltage(wc68);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_const_charge_current(wc68);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = wc68_input_current_limit(wc68);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		rc = wc68_get_iin(wc68, &val->intval);
		if (rc < 0)
			dev_err(wc68->dev, "Invalid IIN ADC (%d)\n", rc);
		break;

	case GBMS_PROP_CHARGE_CHARGER_STATE:
		ret = wc68_get_chg_chgr_state(wc68, &chg_state);
		if (ret < 0)
			return ret;
		if (wc68->irdrop_comp_ok)
			chg_state.f.flags &= ~GBMS_CS_FLAG_NOCOMP;
		gbms_propval_int64val(val) = chg_state.v;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		intval = wc68_read_adc(wc68, ADCCH_VOUT);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		intval = wc68_read_adc(wc68, ADCCH_VBAT);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	/* TODO: read NTC temperature? */
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = wc68_read_adc(wc68, ADCCH_DIETEMP);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = wc68_get_charge_type(wc68);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = wc68_get_status(wc68);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = wc68_input_current_limit(wc68);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * GBMS not visible
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
 */
static enum power_supply_property wc68_mains_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
	/* same as POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT */
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static int wc68_mains_is_writeable(struct power_supply *psy,
				      enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc wc68_mains_desc = {
	.name		= "dc-mains",
	/* b/179246019 will not look online to Android */
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= wc68_mains_get_property,
	.set_property 	= wc68_mains_set_property,
	.properties	= wc68_mains_properties,
	.property_is_writeable = wc68_mains_is_writeable,
	.num_properties	= ARRAY_SIZE(wc68_mains_properties),
};

#if IS_ENABLED(CONFIG_OF)
static int of_wc68_dt(struct device *dev,
			 struct wc68_platform_data *pdata)
{
	struct device_node *np_wc68 = dev->of_node;
	int ret;

	if(!np_wc68)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_wc68, "wc68,irq-gpio", 0);
	dev_info(dev, "irq-gpio: %d \n", pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_wc68, "wc68,input-current-limit",
				   &pdata->iin_cfg_max);
	if (ret) {
		dev_warn(dev, "wc68,input-current-limit is Empty\n");
		pdata->iin_cfg_max = WC68_IIN_CFG_DFT;
	}
	pdata->iin_cfg = pdata->iin_cfg_max;
	dev_info(dev, "wc68,iin_cfg is %u\n", pdata->iin_cfg);

	/* charging float voltage */
	ret = of_property_read_u32(np_wc68, "wc68,float-voltage",
				   &pdata->v_float_dt);
	if (ret) {
		dev_warn(dev, "wc68,float-voltage is Empty\n");
		pdata->v_float_dt = WC68_VFLOAT_DFT;
	}
	pdata->v_float = pdata->v_float_dt;
	dev_info(dev, "wc68,v_float is %u\n", pdata->v_float);

	/* input topoff current */
	ret = of_property_read_u32(np_wc68, "wc68,input-itopoff",
				   &pdata->iin_topoff);
	if (ret) {
		dev_warn(dev, "wc68,input-itopoff is Empty\n");
		pdata->iin_topoff = WC68_IIN_DONE_DFT;
	}
	dev_info(dev, "wc68,iin_topoff is %u\n", pdata->iin_topoff);

	/* iin offsets */
	ret = of_property_read_u32(np_wc68, "wc68,iin-max-offset",
				   &pdata->iin_max_offset);
	if (ret)
		pdata->iin_max_offset = WC68_IIN_MAX_OFFSET;
	dev_info(dev, "wc68,iin_max_offset is %u\n", pdata->iin_max_offset);

	ret = of_property_read_u32(np_wc68, "wc68,iin-cc_comp-offset",
				   &pdata->iin_cc_comp_offset);
	if (ret)
		pdata->iin_cc_comp_offset = WC68_IIN_CC_COMP_OFFSET;
	dev_info(dev, "wc68,iin_cc_comp_offset is %u\n", pdata->iin_cc_comp_offset);

	/* irdrop limits */
	pdata->irdrop_limit_cnt =
	    of_property_count_elems_of_size(np_wc68, "google,irdrop-limits", sizeof(u32));
	if (pdata->irdrop_limit_cnt < WC68_IRDROP_LIMIT_CNT) {
		dev_info(dev, "google,irdrop-limits size get failed, use default irdrop limits %d\n",
			 pdata->irdrop_limit_cnt);
		ret = -EINVAL;
	} else {
		ret = of_property_read_u32_array(np_wc68, "google,irdrop-limits",
						 (u32 *)pdata->irdrop_limits,
						 WC68_IRDROP_LIMIT_CNT);
		if (ret)
			dev_info(dev, "google,irdrop-limits get failed, use default irdrop limits");
	}
	if (ret) {
		pdata->irdrop_limits[0] = WC68_IRDROP_LIMIT_TIER1;
		pdata->irdrop_limits[1] = WC68_IRDROP_LIMIT_TIER2;
		pdata->irdrop_limits[2] = WC68_IRDROP_LIMIT_TIER3;
	}

#if IS_ENABLED(CONFIG_THERMAL)
	/* USBC thermal zone */
	ret = of_property_read_string(np_wc68, "google,usb-port-tz-name",
				      &pdata->usb_tz_name);
	if (ret) {
		dev_info(dev, "google,usb-port-tz-name is Empty\n");
		pdata->usb_tz_name = NULL;
	} else {
		dev_info(dev, "google,usb-port-tz-name is %s\n", pdata->usb_tz_name);
	}
#endif

	return 0;
}
#else
static int of_wc68_dt(struct device *dev,
			 struct wc68_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_THERMAL)
static int wc68_usb_tz_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct wc68_charger *wc68 = tzd->devdata;

	if (!wc68)
		return -ENODEV;
	*temp = wc68_read_adc(wc68, ADCCH_NTC);

	return 0;
}

static struct thermal_zone_device_ops wc68_usb_tzd_ops = {
	.get_temp = wc68_usb_tz_read_temp,
};
#endif

static int debug_apply_offsets(void *data, u64 val)
{
	struct wc68_charger *chip = data;
	int ret;

	ret = wc68_set_new_cc_max(chip, chip->cc_max);
	dev_info(chip->dev, "Apply offsets iin_max_o=%d iin_cc_comp_o=%d ret=%d\n",
		chip->pdata->iin_max_offset, chip->pdata->iin_cc_comp_offset,
		ret);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(apply_offsets_debug_ops, NULL, debug_apply_offsets, "%#02llx\n");


static int debug_adc_chan_get(void *data, u64 *val)
{
	struct wc68_charger *wc68 = data;

	*val = wc68_read_adc(data, wc68->debug_adc_channel);
	return 0;
}

static int debug_adc_chan_set(void *data, u64 val)
{
	struct wc68_charger *wc68 = data;

	if (val < ADCCH_VOUT || val >= ADCCH_MAX)
		return -EINVAL;
	wc68->debug_adc_channel = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_adc_chan_ops, debug_adc_chan_get,
			debug_adc_chan_set, "%llu\n");


static int debug_pps_index_get(void *data, u64 *val)
{
	struct wc68_charger *wc68 = data;

	*val = wc68->pps_index;
	return 0;
}

static int debug_pps_index_set(void *data, u64 val)
{
	struct wc68_charger *wc68 = data;

	return wc68_set_charging_enabled(wc68, (int)val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_index_ops, debug_pps_index_get,
			debug_pps_index_set, "%llu\n");

static ssize_t chg_stats_show(struct device *dev, struct device_attribute *attr,
				    char *buff)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wc68_charger *wc68 = i2c_get_clientdata(client);
	struct wc68_chg_stats *chg_data = &wc68->chg_data;
	const int max_size = PAGE_SIZE;
	int len = -ENODATA;

	mutex_lock(&wc68->lock);

	if (!wc68_chg_stats_valid(chg_data))
		goto exit_done;

	len = scnprintf(buff, max_size,
			"D:%#x,%#x %#x,%#x,%#x,%#x,%#x\n",
			chg_data->adapter_capabilities[0],
			chg_data->adapter_capabilities[1],
			chg_data->receiver_state[0],
			chg_data->receiver_state[1],
			chg_data->receiver_state[2],
			chg_data->receiver_state[3],
			chg_data->receiver_state[4]);
	len += scnprintf(&buff[len], max_size - len,
			"N: ovc=%d,ovc_ibatt=%d,ovc_delta=%d rcp=%d,stby=%d\n",
			chg_data->ovc_count, chg_data->ovc_max_ibatt, chg_data->ovc_max_delta,
			chg_data->rcp_count,
			chg_data->stby_count);
	len += scnprintf(&buff[len], max_size - len,
			"C: nc=%d,pre=%d,ca=%d,cc=%d,cv=%d,adj=%d\n",
			chg_data->nc_count,
			chg_data->pre_count,
			chg_data->ca_count,
			chg_data->cc_count,
			chg_data->cv_count,
			chg_data->adj_count);

exit_done:
	mutex_unlock(&wc68->lock);
	return len;
}

static ssize_t chg_stats_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wc68_charger *wc68 = i2c_get_clientdata(client);

	mutex_lock(&wc68->lock);
	wc68_chg_stats_init(&wc68->chg_data);
	mutex_unlock(&wc68->lock);

	return count;
}

static DEVICE_ATTR_RW(chg_stats);

static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct wc68_charger *wc68 = dev_get_drvdata(dev);
	u8 tmp[FWREG_MAX_REGISTER - FWREG_CHIP_ID_REG + 1];
	int ret = 0, i;
	int len = 0;
	u16 cmd = cpu_to_be16(FWREG_CHIP_ID_REG);

	ret = wc68_i2c_read(wc68->client, &cmd, 2, &tmp,
			sizeof(tmp));
	if (ret < 0)
		return ret;

	for (i = 0; i < sizeof(tmp); i++)
		len += scnprintf(&buf[len], PAGE_SIZE - len, "%02x: %02x\n", i, tmp[i]);

	return len;
}

static DEVICE_ATTR_RO(registers_dump);

static int wc68_create_fs_entries(struct wc68_charger *chip)
{

	device_create_file(chip->dev, &dev_attr_chg_stats);
	device_create_file(chip->dev, &dev_attr_registers_dump);

	chip->debug_root = debugfs_create_dir("charger-wc68", NULL);
	if (IS_ERR_OR_NULL(chip->debug_root)) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -ENOENT;
	}

	debugfs_create_bool("wlc_rampout_iin", 0644, chip->debug_root,
			     &chip->wlc_ramp_out_iin);
	debugfs_create_u32("wlc_rampout_delay", 0644, chip->debug_root,
			   &chip->wlc_ramp_out_delay);
	debugfs_create_u32("wlc_rampout_vout_target", 0644, chip->debug_root,
			   &chip->wlc_ramp_out_vout_target);


	debugfs_create_u32("debug_level", 0644, chip->debug_root,
			   &debug_printk_prlog);
	debugfs_create_u32("no_logbuffer", 0644, chip->debug_root,
			   &debug_no_logbuffer);

	chip->debug_count = 1;  /* Read 1 byte by default */
	debugfs_create_file("data", 0644, chip->debug_root, chip, &register_debug_ops_wc68);
	debugfs_create_x32("address", 0644, chip->debug_root, &chip->debug_address);
	debugfs_create_x32("count", 0644, chip->debug_root, &chip->debug_count);

	debugfs_create_u32("iin_max_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_max_offset);
	debugfs_create_u32("iin_cc_comp_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_cc_comp_offset);
	debugfs_create_file("apply_offsets", 0644, chip->debug_root, chip,
			    &apply_offsets_debug_ops);

	chip->debug_adc_channel = ADCCH_VOUT;
	debugfs_create_file("adc_chan", 0644, chip->debug_root, chip,
			    &debug_adc_chan_ops);
	debugfs_create_file("pps_index", 0644, chip->debug_root, chip,
			    &debug_pps_index_ops);
	debugfs_create_bool("irdrop_comp", 0644, chip->debug_root,
			    &chip->irdrop_comp_ok);

	return 0;
}


static int wc68_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	static char *battery[] = { "wc68-battery" };
	struct power_supply_config mains_cfg = {};
	struct wc68_platform_data *pdata;
	struct wc68_charger *wc68_chg;
	struct device *dev = &client->dev;
	const char *psy_name = NULL;
	int ret;

	dev_dbg(dev, "%s: =========START=========\n", __func__);

	wc68_chg = devm_kzalloc(dev, sizeof(*wc68_chg), GFP_KERNEL);
	if (!wc68_chg)
		return -ENOMEM;

#if IS_ENABLED(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct wc68_platform_data),
				     GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		ret = of_wc68_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, wc68_chg);
	wc68_chg->client = client;

	mutex_init(&wc68_chg->lock);
	wc68_chg->dev = &client->dev;
	wc68_chg->pdata = pdata;
	wc68_chg->charging_state = DC_STATE_NO_CHARGING;
	wc68_chg->wlc_ramp_out_iin = true;
	wc68_chg->wlc_ramp_out_vout_target = 15300000; /* 15.3V as default */
	wc68_chg->wlc_ramp_out_delay = 250; /* 250 ms default */
	wc68_chg->hw_init_done = false;

	/* Create a work queue for the direct charger */
	wc68_chg->dc_wq = alloc_ordered_workqueue("wc68_dc_wq", WQ_MEM_RECLAIM);
	if (wc68_chg->dc_wq == NULL) {
		dev_err(wc68_chg->dev, "failed to create work queue\n");
		return -ENOMEM;
	}

	wc68_chg->monitor_wake_lock =
		wakeup_source_register(NULL, "wc68-charger-monitor");
	if (!wc68_chg->monitor_wake_lock) {
		dev_err(dev, "Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* initialize work */
	INIT_DELAYED_WORK(&wc68_chg->timer_work, wc68_timer_work);
	wc68_chg->timer_id = TIMER_ID_NONE;
	wc68_chg->timer_period = 0;

	INIT_DELAYED_WORK(&wc68_chg->pps_work, wc68_pps_request_work);
	INIT_DELAYED_WORK(&wc68_chg->init_hw_work, wc68_init_hw_work);
	ret = of_property_read_string(dev->of_node,
				      "wc68,psy_name", &psy_name);

	ret = wc68_probe_pps(wc68_chg);
	if (ret < 0) {
		dev_warn(dev, "wc68: PPS not available (%d)\n", ret);
	} else {
		const char *logname = "wc68";

		wc68_chg->log = logbuffer_register(logname);
		if (IS_ERR(wc68_chg->log)) {
			dev_err(dev, "no logbuffer (%ld)\n", PTR_ERR(wc68_chg->log));
			wc68_chg->log = NULL;
		}
	}

	ret = wc68_hw_ping(wc68_chg);
	if (ret < 0)
		goto error;

	schedule_delayed_work(&wc68_chg->init_hw_work, 0);
	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = wc68_chg;
	wc68_chg->mains = devm_power_supply_register(dev,
							&wc68_mains_desc,
							&mains_cfg);
	if (IS_ERR(wc68_chg->mains)) {
		ret = -ENODEV;
		goto error;
	}

	wc68_chg->attrs.attrs = wc_attr_group;
	ret = wc68_create_fs_entries(wc68_chg);
	if (ret < 0)
		dev_err(dev, "error while registering debugfs %d\n", ret);

#if IS_ENABLED(CONFIG_THERMAL)
	if (pdata->usb_tz_name) {
		wc68_chg->usb_tzd =
			thermal_zone_device_register(pdata->usb_tz_name, 0, 0,
						     wc68_chg,
						     &wc68_usb_tzd_ops,
						     NULL, 0, 0);
		if (IS_ERR(wc68_chg->usb_tzd)) {
			wc68_chg->usb_tzd = NULL;
			ret = PTR_ERR(wc68_chg->usb_tzd);
			dev_err(dev, "Couldn't register usb connector thermal zone ret=%d\n",
				ret);
		}
	}
#endif

	wc68_chg->dc_avail = NULL;
	wc68_chg->init_done = true;
	dev_info(dev, "wc68: probe_done\n");
	return 0;

error:
	destroy_workqueue(wc68_chg->dc_wq);
	mutex_destroy(&wc68_chg->lock);
	wakeup_source_unregister(wc68_chg->monitor_wake_lock);
	return ret;
}

static int wc68_remove(struct i2c_client *client)
{
	struct wc68_charger *wc68_chg = i2c_get_clientdata(client);

	/* stop charging if it is active */
	wc68_stop_charging(wc68_chg);

	if (client->irq) {
		free_irq(client->irq, wc68_chg);
		gpio_free(wc68_chg->pdata->irq_gpio);
	}

	destroy_workqueue(wc68_chg->dc_wq);

	wakeup_source_unregister(wc68_chg->monitor_wake_lock);

#if IS_ENABLED(CONFIG_THERMAL)
	if (wc68_chg->usb_tzd)
		thermal_zone_device_unregister(wc68_chg->usb_tzd);
#endif
	if (wc68_chg->log)
		logbuffer_unregister(wc68_chg->log);
	pps_free(&wc68_chg->pps_data);

	return 0;
}

static const struct i2c_device_id wc68_id[] = {
	{ "STM_WC", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wc68_id);

#if IS_ENABLED(CONFIG_OF)
static struct of_device_id wc68_i2c_dt_ids[] = {
	{ .compatible = "st_wc68",},
	{ },
};

MODULE_DEVICE_TABLE(of, wc68_i2c_dt_ids);
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
#if IS_ENABLED(CONFIG_RTC_HCTOSYS)
static int get_current_time(struct wc68_charger *wc68, unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		dev_err(wc68->dev, "%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		dev_err(wc68->dev, "Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		dev_err(wc68->dev, "Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	*now_tm_sec = rtc_tm_to_time64(&tm);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static void
wc68_check_and_update_charging_timer(struct wc68_charger *wc68)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(wc68, &current_time);

	if (wc68->timer_id != TIMER_ID_NONE)	{
		next_update_time = wc68->last_update_time +
				(wc68->timer_period / 1000); /* seconds */

		dev_dbg(wc68->dev, "%s: current_time=%ld, next_update_time=%ld\n",
			__func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&wc68->lock);
		wc68->timer_period = time_left * 1000; /* ms unit */
		mutex_unlock(&wc68->lock);
		schedule_delayed_work(&wc68->timer_work,
				msecs_to_jiffies(wc68->timer_period));

		dev_dbg(wc68->dev, "%s: timer_id=%d, time_period=%ld\n", __func__,
			 wc68->timer_id, wc68->timer_period);
	}
	wc68->last_update_time = current_time;
}
#endif

static int wc68_suspend(struct device *dev)
{
	struct wc68_charger *wc68 = dev_get_drvdata(dev);

	dev_dbg(wc68->dev, "%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&wc68->timer_work);
	return 0;
}

static int wc68_resume(struct device *dev)
{
	struct wc68_charger *wc68 = dev_get_drvdata(dev);

	dev_dbg(wc68->dev, "%s: update_timer\n", __func__);

	/* Update the current timer */
#if IS_ENABLED(CONFIG_RTC_HCTOSYS)
	wc68_check_and_update_charging_timer(wc68);
#else
	if (wc68->timer_id != TIMER_ID_NONE) {
		mutex_lock(&wc68->lock);
		wc68->timer_period = 0;	/* ms unit */
		mutex_unlock(&wc68->lock);
		schedule_delayed_work(&wc68->timer_work,
				      msecs_to_jiffies(wc68->timer_period));
	}
#endif
	return 0;
}
#else
#define wc68_suspend		NULL
#define wc68_resume		NULL
#endif

static const struct dev_pm_ops wc68_pm_ops = {
	.suspend = wc68_suspend,
	.resume = wc68_resume,
};

static struct i2c_driver wc68_driver = {
	.driver = {
		.name = "STM_WC",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = wc68_i2c_dt_ids,
#endif /* CONFIG_OF */
#if IS_ENABLED(CONFIG_PM)
		.pm = &wc68_pm_ops,
#endif
	},
	.probe        = wc68_probe,
	.remove       = wc68_remove,
	.id_table     = wc68_id,
};

module_i2c_driver(wc68_driver);

MODULE_AUTHOR("will.zhou@st.com");
MODULE_AUTHOR("prapancham@google.com");
MODULE_DESCRIPTION("ST WC68 Direct Charger Driver");
MODULE_LICENSE("GPL");
