/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023, Google Inc
 *
 * MAX77779 BATTVIMON management
 */
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#include "google_bms.h"
#include "max77779.h"
#include "max77779_vimon.h"

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

int max77779_external_vimon_reg_read(struct device *dev, uint16_t reg, void *val, int len)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

	if (!data || !data->regmap)
		return -EAGAIN;

	return regmap_raw_read(data->regmap, reg, val, len);
}
EXPORT_SYMBOL_GPL(max77779_external_vimon_reg_read);

int max77779_external_vimon_reg_write(struct device *dev, uint16_t reg, const void *val, int len)
{
	struct max77779_vimon_data *data = dev_get_drvdata(dev);

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

static int max77779_vimon_debug_reg_read(void *d, u64 *val)
{
	struct max77779_vimon_data *data = d;
	int ret, reg;

	ret = regmap_read(data->regmap, data->debug_reg_address, &reg);

	*val = reg & 0xffff;

	return ret;
}

static int max77779_vimon_debug_reg_write(void *d, u64 val)
{
	struct max77779_vimon_data *data = d;

	return regmap_write(data->regmap, data->debug_reg_address, val & 0xffff);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77779_vimon_debug_reg_read,
			max77779_vimon_debug_reg_write, "%04llx\n");

static ssize_t max77779_vimon_show_reg_all(struct file *filp, char __user *buf,
						size_t count, loff_t *ppos)
{
	struct max77779_vimon_data *data = filp->private_data;
	u32 reg_address;
	char *tmp;
	int ret = 0, len = 0;
	int regread;

	if (!data->regmap)
		return -EIO;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (reg_address = 0; reg_address <= 0x7F; reg_address++) {
		ret = regmap_read(data->regmap, reg_address, &regread);
		if (ret < 0)
			continue;

		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %04x\n", reg_address,
				regread & 0xffff);
	}

	if (len > 0)
		len = simple_read_from_buffer(buf, count, ppos, tmp, len);

	kfree(tmp);

	return len;
}

BATTERY_DEBUG_ATTRIBUTE(debug_vimon_all_reg_fops, max77779_vimon_show_reg_all, NULL);

static ssize_t max77779_vimon_show_buff_all(struct file *filp, char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct max77779_vimon_data *data = filp->private_data;
	char *tmp;
	uint16_t *vals;
	int ret;
	int len = 0;
	int i;
	const size_t last_readback_size = MAX77779_VIMON_LAST_PAGE_SIZE *
					  MAX77779_VIMON_BYTES_PER_ENTRY;
	const size_t readback_size = MAX77779_VIMON_PAGE_SIZE * MAX77779_VIMON_BYTES_PER_ENTRY;
	int readback_cnt;

	if (!data->regmap)
		return -EIO;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	vals = kcalloc(MAX77779_VIMON_PAGE_SIZE, sizeof(uint16_t), GFP_KERNEL);

	mutex_lock(&data->vimon_lock);
	ret = regmap_write(data->regmap, MAX77779_BVIM_PAGE_CTRL, data->debug_buffer_page);
	if (ret < 0)
		goto vimon_show_buff_exit;

	if (data->debug_buffer_page < MAX77779_VIMON_PAGE_CNT - 1) {
		ret = regmap_raw_read(data->regmap, MAX77779_VIMON_OFFSET_BASE, vals,
				      readback_size);
		readback_cnt = MAX77779_VIMON_PAGE_SIZE;
	} else {
		ret = regmap_raw_read(data->regmap, MAX77779_VIMON_OFFSET_BASE, vals,
				      last_readback_size);
		readback_cnt = MAX77779_VIMON_LAST_PAGE_SIZE;
	}

	if (ret < 0)
		goto vimon_show_buff_exit;

	for (i = 0; i < readback_cnt; i++)
		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %04x\n",
				 data->debug_buffer_page * MAX77779_VIMON_PAGE_SIZE + i, vals[i]);

	if (len > 0)
		len = simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));

	ret = len;

vimon_show_buff_exit:
	mutex_unlock(&data->vimon_lock);

	kfree(tmp);
	kfree(vals);

	return ret;
}

BATTERY_DEBUG_ATTRIBUTE(debug_vimon_all_buff_fops, max77779_vimon_show_buff_all, NULL);

static int max77779_vimon_debug_buff_page_read(void *d, u64 *val)
{
	struct max77779_vimon_data *data = d;

	*val = data->debug_buffer_page;

	return 0;
}

static int max77779_vimon_debug_buff_page_write(void *d, u64 val)
{
	struct max77779_vimon_data *data = d;

	if (val >= MAX77779_VIMON_PAGE_CNT)
		return -EINVAL;

	data->debug_buffer_page = (u8)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_buff_page_rw_fops, max77779_vimon_debug_buff_page_read,
			max77779_vimon_debug_buff_page_write, "%llu\n");

bool max77779_vimon_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= 0 && reg <= MAX77779_VIMON_SIZE);
}
EXPORT_SYMBOL_GPL(max77779_vimon_is_reg);

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

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);
	debugfs_create_file("registers", 0444, data->de, data, &debug_vimon_all_reg_fops);
	debugfs_create_file("buffer", 0444, data->de, data, &debug_vimon_all_buff_fops);
	debugfs_create_file("buffer_page", 0600, data->de, data, &debug_buff_page_rw_fops);

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

/*
 * Initialization requirements
 * struct max77779_vimon_data *data
 * - dev
 * - regmap
 * - irq
 */
int max77779_vimon_init(struct max77779_vimon_data *data)
{
	struct device *dev = data->dev;
	uint16_t cfg_mask = 0;
	int ret;

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

	if (data->irq){
		ret = devm_request_threaded_irq(data->dev, data->irq, NULL,
				max77779_vimon_irq,
				IRQF_TRIGGER_LOW | IRQF_SHARED | IRQF_ONESHOT,
				"max77779_vimon", data);
		if (ret < 0)
			dev_warn(dev, "Failed to get irq thread.\n");
	} else {
		dev_warn(dev, "irq not setup\n");
	}

	ret = max77779_vimon_init_fs(data);
	if (ret < 0)
		dev_warn(dev, "Failed to initialize debug fs\n");

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_vimon_init);

void max77779_vimon_remove(struct max77779_vimon_data *data)
{
	if (data->de)
		debugfs_remove(data->de);

	if (data->irq)
		free_irq(data->irq, data);
}
EXPORT_SYMBOL_GPL(max77779_vimon_remove);

MODULE_DESCRIPTION("max77779 VIMON Driver");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_AUTHOR("Chungro Lee <chungro@google.com>");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
