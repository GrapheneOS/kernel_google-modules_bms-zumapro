/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for LN8411
 */

#ifndef _LN8411_H_
#define _LN8411_H_

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/regmap.h>

#define LN8411_DRV_VERSION		"1.0" /* driver version string format */
/* FW register address */
#define FWREG_DEVICE_ID_REG		0x00

#define FWREG_MAX_REGISTER		0x9c

struct ln8411_chip_info {
	u16 device_id;
};

struct ln8411_info {
	struct device *dev;
	struct i2c_client *client;
	struct ln8411_chip_info chip_info;
	struct attribute_group attrs; /* SysFS attributes */
	struct regmap *regmap;

	/* debug */
	struct dentry *debug_root;
	u32 debug_address;
};

#endif /* _STWLC98_H_ */

