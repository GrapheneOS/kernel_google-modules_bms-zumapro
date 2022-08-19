// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ST WC68 Direct charger
 * Based on sample linux driver for ST WLC98 from ST
 */

#include <linux/debugfs.h>
#include "wc68_driver.h"

static int wc68_i2c_read(struct i2c_client *client, u8 *cmd, u8 cmd_length,
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

	struct wc_ts_info *info = dev_get_drvdata(dev);

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

static int read_reg(void *data, u64 *val)
{
	struct wc_ts_info *chip = data;
	int rc;
	u16 cmd;
	u64 read_val = 0;
	u16 debug_address = (u16)chip->debug_address;

	if (chip->debug_count < 1 || chip->debug_count > 8) {
		dev_err(chip->dev, "Invalid count: %d. Valid: 1-8\n", chip->debug_count);
		return -EINVAL;
	}

	if (!wc68_is_reg(debug_address)) {
		dev_err(chip->dev, "Invalid register address: %04X\n", debug_address);
		return -EINVAL;
	}

	cmd = cpu_to_be16(debug_address);

	rc = wc68_i2c_read(chip->client, (u8 *)&cmd, 2, &read_val, chip->debug_count);
	if (rc) {
		dev_err(chip->dev, "Couldn't read reg %#x rc = %d\n",
			debug_address, rc);
		return -EAGAIN;
	}

	*val = be64_to_cpu(read_val) >> ((8 - chip->debug_count) * 8);
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct wc_ts_info *chip = data;
	u8 cmd[10];
	u16 debug_address = (u16)chip->debug_address;

	if (chip->debug_count < 1 || chip->debug_count > 8) {
		dev_err(chip->dev, "Invalid count: %d. Valid: 1-8\n", chip->debug_count);
		return -EINVAL;
	}

	if (!wc68_is_reg(debug_address)) {
		dev_err(chip->dev, "Invalid register address: %04X\n", debug_address);
		return -EINVAL;
	}

	cmd[0] = debug_address >> 8;
	cmd[1] = debug_address & 0xFF;

	val = cpu_to_be64(val) >> ((8 - chip->debug_count) * 8);

	memcpy(&cmd[2], &val, chip->debug_count);

	return wc68_i2c_write(chip->client, cmd, 2 + chip->debug_count);
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "%#llx\n");

static int wc68_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error = 0;
	struct wc_ts_info *info;

	info = devm_kzalloc(&client->dev, sizeof(struct wc_ts_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	info->client = client;
	info->dev = &client->dev;
	dev_set_drvdata(info->dev, info);

	/* sysfs stuff */
	info->attrs.attrs = wc_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		dev_err(info->dev, "ERROR: Cannot create sysfs structure!\n");
		error = -ENODEV;
		goto probe_exit_error0;
	}

	info->debug_root = debugfs_create_dir("charger-wc68", NULL);
	if (IS_ERR_OR_NULL(info->debug_root)) {
		dev_err(info->dev, "Couldn't create debug dir\n");
		error = -ENOENT;
		goto probe_exit_error1;
	}

	info->debug_count = 1;  /* Read 1 byte by default */
	debugfs_create_file("data", 0644, info->debug_root, info, &register_debug_ops);
	debugfs_create_x32("address", 0644, info->debug_root, &info->debug_address);
	debugfs_create_x32("count", 0644, info->debug_root, &info->debug_count);
	dev_info(info->dev, "Driver Version %s\n", WC_DRV_VERSION);

	return 0;

probe_exit_error1:
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
probe_exit_error0:
	devm_kfree(&client->dev, info);
	dev_err(info->dev, "probe failed error: %d\n", error);
	return error;
}

static int wc68_remove(struct i2c_client *client)
{
	struct wc_ts_info *info = dev_get_drvdata(&(client->dev));
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
	devm_kfree(&client->dev, info);
	return 0;
}

static const struct i2c_device_id id[] = {
	{ "STM_WC", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, id);

#ifdef CONFIG_OF
static struct of_device_id stwc68_match_table[] = {
	{ .compatible = "st_wc68",},
	{ },
};
#endif

static struct i2c_driver driver = {
	.probe		= wc68_probe,
	.remove		= wc68_remove,
	.id_table	= id,
	.driver = {
		.name	= "STM_WC",
#ifdef CONFIG_OF
		.of_match_table = stwc68_match_table,
#endif
	},
};

static int __init stwc68_wc_init(void)
{
	return i2c_add_driver(&driver);
}

static void __exit stwc68_wc_exit(void)
{
	i2c_del_driver(&driver);
}

late_initcall(stwc68_wc_init);
module_exit(stwc68_wc_exit);

MODULE_AUTHOR("will.zhou@st.com");
MODULE_AUTHOR("prapancham@google.com");
MODULE_DESCRIPTION("ST WC68 Direct Charger Driver");
MODULE_LICENSE("GPL");
