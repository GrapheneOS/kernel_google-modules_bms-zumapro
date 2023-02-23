// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for LN8411
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

#include "ln8411_driver.h"

static int get_chip_info(struct ln8411_info *info)
{
	unsigned int val;

	int err = regmap_read(info->regmap, FWREG_DEVICE_ID_REG, &val);

	info->chip_info.device_id = val;

	dev_info(info->dev, "DeviceID: %02X\n",	val);

	return err;
}

static ssize_t chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct ln8411_info *info = dev_get_drvdata(dev);

	ret = get_chip_info(info);
	if (ret) {
		dev_err(dev, "Error while getting chip info\n");
		return ret;
	}

	ret = scnprintf(buf, PAGE_SIZE, "Chip Id : %#02X\n", info->chip_info.device_id);
	return ret;
}

static DEVICE_ATTR_RO(chip_info);

static struct attribute *ln8411_attr_group[] = {
	&dev_attr_chip_info.attr,
	NULL
};

static bool ln8411_is_reg(struct device *dev, unsigned int reg)
{
	switch(reg) {
	case 0x0 ... FWREG_MAX_REGISTER:
		return true;
	default:
		return false;
	}
}

static struct regmap_config ln8411_regmap = {
	.name		= "dc-mains",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= FWREG_MAX_REGISTER,
	.readable_reg = ln8411_is_reg,
	.volatile_reg = ln8411_is_reg,
};

static int read_reg(void *data, u64 *val)
{
	struct ln8411_info *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;

	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct ln8411_info *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;

	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't write %#02x to %#02x rc = %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "%#02llx\n");

static int ln8411_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0;
	struct ln8411_info *info = NULL;

	info = devm_kzalloc(&client->dev, sizeof(struct ln8411_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	info->client = client;
	info->dev = &client->dev;
	dev_set_drvdata(info->dev, info);

	info->regmap = devm_regmap_init_i2c(client, &ln8411_regmap);
	if (IS_ERR(info->regmap)) {
		error = -EINVAL;
		goto probe_exit_error1;
	}

	error = get_chip_info(info);
	if (error) {
		dev_err(info->dev, "ERROR: Cannot read chip info!\n");
		error = -ENODEV;
		goto probe_exit_error0;
	}

	/* sysfs stuff */
	info->attrs.attrs = ln8411_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		dev_err(info->dev, "ERROR: Cannot create sysfs structure!\n");
		error = -ENODEV;
		goto probe_exit_error0;
	}

	info->debug_root = debugfs_create_dir("charger-ln8411", NULL);
	if (IS_ERR_OR_NULL(info->debug_root)) {
		dev_err(info->dev, "Couldn't create debug dir\n");
		error = -ENOENT;
		goto probe_exit_error1;
	}

	debugfs_create_file("data", 0644, info->debug_root, info, &register_debug_ops);
	debugfs_create_x32("address", 0644, info->debug_root, &info->debug_address);

	dev_info(info->dev, "Driver Version %s\n", LN8411_DRV_VERSION);
	return 0;

probe_exit_error1:
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
probe_exit_error0:
	devm_kfree(info->dev, info);
	dev_err(&client->dev, "probe failed error: %d\n", error);
	return error;
}

static int ln8411_remove(struct i2c_client *client)
{
	struct ln8411_info *info = dev_get_drvdata(&(client->dev));
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
	devm_kfree(info->dev, info);
	return 0;
}

static const struct i2c_device_id id[] = {
	{ "LN8411", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, id);

#ifdef CONFIG_OF
static struct of_device_id ln8411_match_table[] = {
	{ .compatible = "ln8411",},
	{ },
};
MODULE_DEVICE_TABLE(of, ln8411_match_table);
#endif

static struct i2c_driver ln8411_driver = {
	.probe		= ln8411_probe,
	.remove		= ln8411_remove,
	.id_table	= id,
	.driver = {
		.name   = "LN8411",
#ifdef CONFIG_OF
		.of_match_table = ln8411_match_table,
#endif
	},
};

module_i2c_driver(ln8411_driver);

MODULE_AUTHOR("Prasanna Prapancham <prapancham@google.com>");
MODULE_DESCRIPTION("LN8411 Charger Pump Driver");
MODULE_LICENSE("GPL");
