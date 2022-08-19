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

#define FWREG_MAX_REGISTER		0xFFFF

/* error code */
#define E_BUS_R				(0x80000001)
#define E_BUS_W				(0x80000002)
#define E_BUS_WR			(0x80000003)

#define MAX_CMD_SIZE			200

struct wc_ts_info {
	struct device *dev;
	struct i2c_client *client;
	struct attribute_group attrs;    /* SysFS attributes */
	int irq_gpio;    /* number of the gpio associated to the interrupt pin */
	int irq;
	struct regmap *regmap;

	/* debug */
	struct dentry *debug_root;
	u32 debug_address;
	u32 debug_count;
};

#endif /* STWC68_H */

