/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023, Google Inc
 *
 * MAX77779 BATTVIMON management
 */
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/regmap.h>

#include <linux/device.h>
#include "max77779_regs.h"
#include "max77779.h"
#include <linux/debugfs.h>

#define MAX77779_VIMON_SIZE 0xFF
#define MAX77779_VIMON_DEFAULT_MAX_CNT 256
#define MAX77779_VIMON_DEFAULT_MAX_TRIGGERS 1

#define MAX77779_VIMON_BUFFER_SIZE 0x100
#define MAX77779_VIMON_OFFSET_BASE 0x80

enum max77779_vimon_state {
	MAX77779_VIMON_ERROR = -1,
	MAX77779_VIMON_IDLE = 0,
	MAX77779_VIMON_RUNNING,
	MAX77779_VIMON_DATA_AVAILABLE,
};

struct max77779_vimon_data {
	struct device *dev;
	struct regmap *regmap;
	struct dentry *de;
	struct mutex vimon_lock;
	unsigned max_cnt;
	unsigned max_triggers;
	enum max77779_vimon_state state;
	char *buf;
};


static int max77779_vimon_reg_read(struct device *dev,
		unsigned int reg, unsigned int *val)
{
	int err;
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	if (!data) {
		dev_err(dev, "error to max77779_vimon_reg_read data is null\n");
		return -EINVAL;
	}

	err = regmap_read(data->regmap, reg, val);
	if (err)
		dev_err(dev, "error reading %#02x err = %d\n", reg, err);

	return err;
}

static int max77779_vimon_reg_write(struct device *dev,
		unsigned int reg, unsigned int val)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	int err;

	err = regmap_write(data->regmap, reg, val);
	if (err)
		dev_err(dev, "error writing %#02x err = %d\n", reg, err);
	return err;
}

static int max77779_vimon_reg_update(struct device *dev, unsigned int reg,
		unsigned int mask, unsigned int val)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	int err;

	err = regmap_update_bits(data->regmap, reg, mask, val);
	if (err)
		dev_err(dev, "error updating %#02x err = %d\n",
				reg, err);
	return err;
}

int max77779_external_vimon_reg_read(struct i2c_client *client,
				     unsigned int reg, void *val, int len)
{
	struct max77779_vimon_data *data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -EAGAIN;

	return regmap_raw_read(data->regmap, reg, val, len);
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_reg_read);

int max77779_external_vimon_reg_write(struct i2c_client *client,
				      unsigned int reg, const void *val, int len)
{
	struct max77779_vimon_data *data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -EAGAIN;

	return regmap_raw_write(data->regmap, reg, val, len);
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_reg_write);

static int max77779_vimon_start(struct device *dev)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->vimon_lock);

	ret = max77779_vimon_reg_update(dev, MAX77779_BVIM_bvim_cfg,
					MAX77779_BVIM_bvim_cfg_cnt_run_MASK,
					MAX77779_BVIM_bvim_cfg_cnt_run_MASK);
	if (ret)
		goto vimon_start_exit;
	ret = max77779_vimon_reg_write(dev, MAX77779_BVIM_CTRL,
					MAX77779_BVIM_CTRL_BVIMON_TRIG_MASK);
	if (ret == 0)
		data->state = MAX77779_VIMON_RUNNING;

vimon_start_exit:
	mutex_unlock(&data->vimon_lock);

	return ret;
}



static int max77779_vimon_stop(struct device *dev)
{
	return max77779_vimon_reg_write(dev, MAX77779_BVIM_CTRL, 0);
}

static int max77779_vimon_set_config(struct device *dev, uint16_t mask)
{
	return max77779_vimon_reg_write(dev, MAX77779_BVIM_bvim_cfg, mask);
}

static int max77779_vimon_clear_config(struct device *dev, uint16_t mask)
{
	return max77779_vimon_reg_write(dev, MAX77779_BVIM_bvim_cfg, 0);
}

/* TODO: b/299357412 applying math operation */
static int max77779_vimon_set_math_operation(struct device *dev, bool enable)
{
	return -ENOSYS;
}

/* TODO: b/299357412 hold lock on &data->vimon_lock */
static int max77779_vimon_rd(uint8_t *buff, size_t addr, size_t count,
			struct regmap *regmap)
{
	return -ENOSYS;
}

static int max7779_vimon_handle_data(struct max77779_vimon_data *data)
{
	unsigned bvim_rfap, rsc, bvim_osc, smpl_start_add;
	int ret;

	/* TODO: b/299357412 reduce the scope of critical section */

	mutex_lock(&data->vimon_lock);

	/* read Ready First Pointer */
	ret = max77779_vimon_reg_read(data->dev, MAX77779_BVIM_bvim_rfap, &bvim_rfap);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM rfap (%d).\n", ret);
		goto vimon_handle_data_exit;
	}
	/* read Ready Sample Count */
	ret = max77779_vimon_reg_read(data->dev, MAX77779_BVIM_bvim_rs, &rsc);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM rsc (%d).\n", ret);
		goto vimon_handle_data_exit;
	}
	rsc = _max77779_bvim_bvim_rs_rsc_get(rsc);

	/* Read data */

	/* TODO: b/299161645 Handle wrapped case*/
	ret = max77779_vimon_rd(data->buf, bvim_rfap, rsc, data->regmap);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM rsc (%d).\n", ret);
		goto vimon_handle_data_exit;
	}

	/* Read Ongoing Sample Count */
	ret = max77779_vimon_reg_read(data->dev, MAX77779_BVIM_bvim_sts, &bvim_osc);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM sts (%d).\n", ret);
		goto vimon_handle_data_exit;
	}
	bvim_osc = _max77779_bvim_bvim_sts_bvim_osc_get(bvim_osc);

	/* TODO: b/299161568 check for overflow */

	/* Read Sample Start Address Pointer */
	ret = max77779_vimon_reg_read(data->dev, MAX77779_BVIM_smpl_math, &smpl_start_add);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM smpl_math (%d).\n", ret);
		goto vimon_handle_data_exit;
	}
	smpl_start_add = _max77779_bvim_smpl_math_smpl_start_add_get(smpl_start_add);

	/* TODO: b/299161568 check for overflow */


vimon_handle_data_exit:
	mutex_unlock(&data->vimon_lock);

	return ret;
}

/*
 * BattVIMon's Buffer: (1024-32) bytes
 * -page[0:2] 256byts, page[3]:224(256-32)
 * -ranges
 *   page0: [0x000:0x07F]
 *   page1: [0x080:0x0FF]  ---> 0x80:0xFF
 *   page2: [0x100:0x17F]
 *   page3: [0x180:0x1EF]
 */
static ssize_t max77779_vimon_access_buffer(struct max77779_vimon_data *data,
				size_t offset, size_t len, uint8_t *buffer, bool toread)
{
	unsigned int target_addr;
	int ret = -1;
	size_t sz;
	unsigned int page;
	size_t start = offset;
	const char* type = toread?"read":"write";

	/* valid range: 0 - (1024-32)*/
	if (offset+len > 992) {
		dev_err(data->dev, "Failed to %s BVIM's buffer: out of range\n", type);
		return -EINVAL;
	}

	mutex_lock(&data->vimon_lock);
	while (len > 0) {
		/*
		 * page = offset / 256
		 * sz   = 256 - (offset % 256)
		 * target_addr = 0x80 + (offset % 256)
		 */
		page = offset>>8;
		sz = MAX77779_VIMON_BUFFER_SIZE - (offset & 0xFF);
		if (sz > len) sz = len;
		target_addr = MAX77779_VIMON_OFFSET_BASE + (offset & 0xFF);


		ret = regmap_write(data->regmap, MAX77779_SP_PAGE_CTRL, page);
		if (ret < 0)
			break;
		/*
		 * using async read?
		 * * TODO: profiling async version
		 */
		if (toread)
			ret = regmap_raw_read(data->regmap, target_addr, buffer, sz);
		else
			ret = regmap_raw_write(data->regmap, target_addr, buffer, sz);
		if (ret < 0)
			break;

		offset += sz;
		buffer += sz;
		len -= sz;
	}
	mutex_unlock(&data->vimon_lock);

	if (ret < 0)
		return ret;

	return offset - start;
}

static ssize_t bvim_cfg_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ret = -1;
	unsigned int val=-1;
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	ret = max77779_vimon_reg_read(data->dev,
					MAX77779_BVIM_bvim_cfg,
					&val);

	if (ret <0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t bvim_cfg_store(struct device *dev,
				struct device_attribute *attr,
				const char* buf, size_t count)
{
	int ret = -1;
	unsigned int val = -1;
        struct max77779_vimon_data *data = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 0, &val);
	if (ret <0)
		return ret;

        ret = max77779_vimon_reg_write(data->dev,
                                        MAX77779_BVIM_bvim_cfg,
                                        val);
        return (ret < 0) ? ret : count;
}

DEVICE_ATTR(bvim_cfg, 0660, bvim_cfg_show, bvim_cfg_store);

static struct attribute *max77779_vimon_attrs[] = {
	&dev_attr_bvim_cfg.attr,
	NULL,
};

static const struct attribute_group max77779_vimon_attr_grp = {
	.attrs = max77779_vimon_attrs,
};

/* -- debug --------------------------------------------------------------- */
static int max77779_vimon_debug_start(void *d, u64 *val)
{
	struct max77779_vimon_data *data = d;
	int ret;

	ret = max77779_vimon_start(data->dev);
	if (ret)
		return ret;

	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(debug_start_fops, max77779_vimon_debug_start,
			NULL, "%02llx\n");


static bool max77779_vimon_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0 && reg < MAX77779_VIMON_SIZE);
}

static const struct regmap_config max77779_vimon_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77779_VIMON_SIZE,
	.readable_reg = max77779_vimon_is_reg,
	.volatile_reg = max77779_vimon_is_reg,
};

static int max77779_vimon_init_fs(struct max77779_vimon_data *data)
{
	int ret = -1;

	ret = sysfs_create_group(&data->dev->kobj, &max77779_vimon_attr_grp);
	if (ret < 0) {
		dev_err(data->dev, "Failed to create sysfs group ret:%d\n", ret);
		return ret;
	}

	data->de = debugfs_create_dir("max77779_vimon", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_file("start", 0600, data->de, data, &debug_start_fops);

	return 0;
}

/* IRQ */
static irqreturn_t max77779_vimon_irq(int irq, void *ptr)
{
	struct max77779_vimon_data *data = ptr;
	unsigned int mask, val;
	int ret;
	enum max77779_vimon_state state;

	mutex_lock(&data->vimon_lock);
	state = data->state;
	mutex_unlock(&data->vimon_lock);

	if ((int)state < MAX77779_VIMON_RUNNING)
		return IRQ_NONE;

	ret = regmap_read(data->regmap, MAX77779_BVIM_MASK, &mask);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM Mask (%d).\n",
				ret);
		return IRQ_NONE;
	}

	ret = regmap_read(data->regmap, MAX77779_BVIM_INT_STS, &val);
	if (ret) {
		dev_err(data->dev, "Failed to read BVIM Val (%d).\n",
				ret);
		return IRQ_NONE;
	}

	if (!(val & mask))
		return IRQ_NONE;

	mutex_lock(&data->vimon_lock);
	data->state = MAX77779_VIMON_DATA_AVAILABLE;
	mutex_unlock(&data->vimon_lock);

	/* clear interrupt */
	ret = regmap_write(data->regmap, MAX77779_BVIM_INT_STS, val);
	if (ret)
		dev_err(data->dev, "Failed to clear INT_STS (%d).\n",
				ret);

	return IRQ_HANDLED;
}

static int max77779_vimon_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77779_vimon_data *data;
	struct regmap *regmap;
	uint16_t cfg_mask = 0;
	int ret;


	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	i2c_set_clientdata(client, data);

	regmap = devm_regmap_init_i2c(client, &max77779_vimon_regmap_cfg);
        if (IS_ERR(regmap)) {
                dev_err(dev, "Failed to initialize regmap\n");
                return -EINVAL;
        }
	data->regmap = regmap;

	mutex_init(&data->vimon_lock);

	cfg_mask = MAX77779_BVIM_bvim_cfg_vioaok_stop_MASK |
		   MAX77779_BVIM_bvim_cfg_top_fault_stop_MASK |
		   MAX77779_BVIM_bvim_cfg_batoiolo2_stop_MASK |
		   MAX77779_BVIM_bvim_cfg_batoiolo1_stop_MASK;

	ret = max77779_vimon_set_config(dev, cfg_mask);
	if (ret) {
		dev_err(dev, "Failed to configure vimon\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "max77779,max_cnt", &data->max_cnt);
	if (ret)
		data->max_cnt = MAX77779_VIMON_DEFAULT_MAX_CNT;

	ret = of_property_read_u32(dev->of_node, "max77779,max_triggers", &data->max_cnt);
	if (ret)
		data->max_triggers = MAX77779_VIMON_DEFAULT_MAX_TRIGGERS;

	data->buf = devm_kzalloc(dev, sizeof(*data->buf * data->max_cnt * data->max_triggers * 2),
				GFP_KERNEL);

	/* pmic-irq driver needs to setup the irq */
	if (client->irq < 0)
		return -EPROBE_DEFER;

	ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
			max77779_vimon_irq,
			IRQF_TRIGGER_LOW | IRQF_SHARED | IRQF_ONESHOT,
			"max77779_vimon", data);
	if (ret < 0)
		dev_err(dev, "Failed to get irq thread.\n");

	ret = max77779_vimon_init_fs(data);
	if (ret < 0)
		dev_err(dev, "Failed to initialize debug fs\n");

	return 0;
}

static void max77779_vimon_remove(struct i2c_client *client)
{
	struct max77779_vimon_data *data = i2c_get_clientdata(client);

	if (data->de)
		debugfs_remove(data->de);

	if (client->irq)
		free_irq(client->irq, data);
}

static const struct of_device_id max77779_vimon_of_match_table[] = {
	{ .compatible = "maxim,max77779vimon"},
	{},
};
MODULE_DEVICE_TABLE(of, max77779_vimon_of_match_table);

static const struct i2c_device_id max77779_vimon_id[] = {
	{"max77779_vimon", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_vimon_id);

static struct i2c_driver max77779_vimon_i2c_driver = {
	.driver = {
		.name = "max77779-vimon",
		.owner = THIS_MODULE,
		.of_match_table = max77779_vimon_of_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = max77779_vimon_id,
	.probe	= max77779_vimon_probe,
	.remove   = max77779_vimon_remove,
};


module_i2c_driver(max77779_vimon_i2c_driver);
MODULE_DESCRIPTION("max77779 VIMON Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_AUTHOR("Chungro Lee <chungro@google.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
