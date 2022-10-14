/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for ST WC68 Direct charger
 * Based on sample linux driver for ST WLC98 from ST
 */

#ifndef _STWC68_H_
#define _STWC68_H_

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>

#define WC_DRV_VERSION			"1.0" /* driver version string format */
/* FW register address */
#define FWREG_CHIP_ID_REG		0xA400

#define FWREG_MAX_REGISTER		0xA479

/* error code */
#define E_BUS_R				0x80000001
#define E_BUS_W				0x80000002
#define E_BUS_WR			0x80000003

#define MAX_CMD_SIZE			200

#define SYS_CMD				0xA408
#define 	SYS_CMD_SYS_RESET		BIT(4)
#define		SYS_CMD_SW_CTRL_USB		(0x2 << 6)
#define 	SYS_CMD_MODE_CTRL_2_1		(0X2 << 0)
#define SYS_CFG_1			0xA40A
#define SYS_CFG_2			0xA40B
#define 	SYS_CFG_2_SC_EN			BIT(4)
#define SYS_CFG_3			0xA40C
#define PROT_EN_0			0xA40E
#define 	PROT_EN_0_CBUS_UCP_EN		BIT(6)
#define PROT_EN_1			0xA40F
#define 	PROT_EN_1_VBAT_OVP_WARN_EN	BIT(2)
#define 	PROT_EN_1_CBUS_OCP_WARN_EN	BIT(4)
#define PROT_STS_0			0xA412
#define PROT_STS_1			0xA413
#define 	PROT_STS_1_VBAT_OVP_WARN_STS	BIT(2)
#define 	PROT_STS_1_CBUS_OCP_WARN_STS	BIT(4)
#define CBUS_UCP_THRES			0xA41B
#define VBAT_OVP_WARN_THRES		0xA41F
#define CBUS_OCP_WARN_THRES		0xA42F
#define ADC_CTRL			0xA437
#define 	ADC_CTRL_CONT_EN		0x1
#define CHARGE_STS			0xA44D
#define 	CHARGE_STS_CHARGING		(0x2 << 0)
#define INTR_FLG_0			0xA44F
#define 	INTR_FLG_0_TDIE_OVTP		BIT(7)
#define INTR_FLG_3			0xA452
#define 	INTR_FLG_3_BOOTUP_RDY		BIT(0)
#define INTR_EN_0			0xA457
#define 	VBAT_OVP_INTR_MSK		BIT(0)
#define 	CBUS_OCP_INTR_MSK		BIT(5)
#define 	TDIE_OVTP_INTR_MSK		BIT(7)
#define INTR_CLR_0			0xA45F
#define VBAT1_ADC			0xA472
#define IBUS_ADC			0xA470
#define TDIE_ADC			0xA478

#define WC68_IIN_CFG_MIN		0
/* input current step, unit - uA */
#define WC68_IIN_CFG_STEP		3662
/* input current, unit - uA */
#define WC68_IIN_CFG(input_curr)	((input_curr) / WC68_IIN_CFG_STEP)
/* charging current, uint - uA  */
#define WC68_ICHG_CFG(_chg_current)	((_chg_current) / 100000)
/* v_float voltage, unit - uV */
#define WC68_V_FLOAT(_v_float)	(((_v_float) / 1000 - 3725) / 5)

/* ADC Channel */
enum {
	ADCCH_VOUT = 1,
	ADCCH_VIN,
	ADCCH_VBAT,	/* 3 */
	ADCCH_ICHG,
	ADCCH_IIN,	/* 5 */
	ADCCH_DIETEMP,	/* 6 */
	ADCCH_NTC,
	ADCCH_MAX
};

/* ADC step */
#define VIN_STEP	16000	/* 16mV(16000uV) LSB, Range(0V ~ 16.368V) */
#define VBAT_STEP	3076	/* (3076uV) LSB, Range(0V ~ 6.297V) */
#define IIN_STEP	3662 	/* (3662uA) LSB, Range(-7.5A ~ 7.496A) */
#define DIETEMP_STEP  	-116	/* 0.116C LSB, Range(-40 ~ 150C) */
#define DIETEMP_MIN 	-40  	/* -40C */
#define DIETEMP_MAX	150	/* 150C */
#define ADC_IIN_OFFSET	900000	/* 900mA */
#define VFLOAT_STEP	3076
#define CBUS_UCP_STEP	3662

#endif /* STWC68_H */

