/*
 * Fuel gauge driver for common
 *
 * Copyright (C) 2023 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include "maxfg_common.h"

/* dump FG model data */
void dump_model(struct device *dev, u16 model_start, u16 *data, int count)
{
	int i, j, len;
	char buff[16 * 5 + 1] = {};

	for (i = 0; i < count; i += 16) {

		for (len = 0, j = 0; j < 16; j++)
			len += scnprintf(&buff[len], sizeof(buff) - len,
					 "%04x ", data[i + j]);

		dev_info(dev, "%x: %s\n", i + model_start, buff);
	}
}
EXPORT_SYMBOL_GPL(dump_model);

int maxfg_get_fade_rate(struct device *dev, int bhi_fcn_count, int *fade_rate)
{
	struct maxfg_eeprom_history hist = { 0 };
	int ret, ratio, i, fcn_sum = 0;
	u16 hist_idx;

	ret = gbms_storage_read(GBMS_TAG_HCNT, &hist_idx, sizeof(hist_idx));
	if (ret < 0) {
		dev_err(dev, "failed to get history index (%d)\n", ret);
		return -EIO;
	}

	dev_dbg(dev, "%s: hist_idx=%d\n", __func__, hist_idx);

	/* no fade for new battery (less than 30 cycles) */
	if (hist_idx < bhi_fcn_count)
		return 0;

	while (hist_idx >= BATT_MAX_HIST_CNT && bhi_fcn_count > 1) {
		hist_idx--;
		bhi_fcn_count--;
		if (bhi_fcn_count == 1) {
			hist_idx = BATT_MAX_HIST_CNT - 1;
			break;
		}
	}

	for (i = bhi_fcn_count; i ; i--, hist_idx--) {
		ret = gbms_storage_read_data(GBMS_TAG_HIST, &hist,
					     sizeof(hist), hist_idx);

		dev_dbg(dev, "%s: idx=%d hist.fc=%d (%x) ret=%d\n", __func__,
			hist_idx, hist.fullcapnom, hist.fullcapnom, ret);

		if (ret < 0)
			return -EINVAL;

		/* hist.fullcapnom = fullcapnom * 800 / designcap */
		fcn_sum += hist.fullcapnom;
	}

	/* convert from maxfg_eeprom_history to percent */
	ratio = fcn_sum / (bhi_fcn_count * 8);

	/* allow negative value when capacity larger than design */
	*fade_rate = 100 - ratio;

	return 0;
}
EXPORT_SYMBOL_GPL(maxfg_get_fade_rate);

static const struct maxfg_reg * maxfg_find_by_index(struct maxfg_regtags *tags, int index)
{
	if (index < 0 || !tags || index >= tags->max)
		return NULL;

	return &tags->map[index];
}

const struct maxfg_reg * maxfg_find_by_tag(struct maxfg_regmap *map, enum maxfg_reg_tags tag)
{
	return maxfg_find_by_index(&map->regtags, tag);
}
EXPORT_SYMBOL_GPL(maxfg_find_by_tag);

int maxfg_reg_read(struct maxfg_regmap *map, enum maxfg_reg_tags tag, u16 *val)
{
	const struct maxfg_reg *reg;
	unsigned int tmp;
	int rtn;

	reg = maxfg_find_by_tag(map, tag);
	if (!reg)
		return -EINVAL;

	rtn = regmap_read(map->regmap, reg->reg, &tmp);
	if (rtn)
		pr_err("Failed to read %x\n", reg->reg);
	else
		*val = tmp;

	return rtn;
}
EXPORT_SYMBOL_GPL(maxfg_reg_read);

#define REG_HALF_HIGH(reg)     ((reg >> 8) & 0x00FF)
#define REG_HALF_LOW(reg)      (reg & 0x00FF)
int maxfg_collect_history_data(void *buff, size_t size, bool is_por,
			       struct maxfg_regmap *regmap, struct maxfg_regmap *regmap_debug)
{
	struct maxfg_eeprom_history hist = { 0 };
	u16 data, designcap;
	int ret;

	if (is_por)
		return -EINVAL;

	ret = maxfg_reg_read(regmap_debug, MAXFG_TAG_tempco, &data);
	if (ret)
		return ret;

	hist.tempco = data;

	ret = maxfg_reg_read(regmap_debug, MAXFG_TAG_rcomp0, &data);
	if (ret)
		return ret;

	hist.rcomp0 = data;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_timerh, &data);
	if (ret)
		return ret;

	/* Convert LSB from 3.2hours(192min) to 5days(7200min) */
	hist.timerh = data * 192 / 7200;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_descap, &designcap);
	if (ret)
		return ret;

	/* multiply by 100 to convert from mAh to %, LSB 0.125% */
	ret = maxfg_reg_read(regmap, MAXFG_TAG_fcnom, &data);
	if (ret)
		return ret;

	data = data * 800 / designcap;
	hist.fullcapnom = data > MAX_HIST_FULLCAP ? MAX_HIST_FULLCAP : data;

	/* multiply by 100 to convert from mAh to %, LSB 0.125% */
	ret = maxfg_reg_read(regmap, MAXFG_TAG_fcrep, &data);
	if (ret)
		return ret;

	data = data * 800 / designcap;
	hist.fullcaprep = data > MAX_HIST_FULLCAP ? MAX_HIST_FULLCAP : data;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_msoc, &data);
	if (ret)
		return ret;

	/* Convert LSB from 1% to 2% */
	hist.mixsoc = REG_HALF_HIGH(data) / 2;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_vfsoc, &data);
	if (ret)
		return ret;

	/* Convert LSB from 1% to 2% */
	hist.vfsoc = REG_HALF_HIGH(data) / 2;


	ret = maxfg_reg_read(regmap, MAXFG_TAG_mmdv, &data);
	if (ret)
		return ret;

	/* LSB is 20mV, store values from 4.2V min */
	hist.maxvolt = (REG_HALF_HIGH(data) * 20 - 4200) / 20;
	/* Convert LSB from 20mV to 10mV, store values from 2.5V min */
	hist.minvolt = (REG_HALF_LOW(data) * 20 - 2500) / 10;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_mmdt, &data);
	if (ret)
		return ret;

	/* Convert LSB from 1degC to 3degC, store values from 25degC min */
	hist.maxtemp = (REG_HALF_HIGH(data) - 25) / 3;
	/* Convert LSB from 1degC to 3degC, store values from -20degC min */
	hist.mintemp = (REG_HALF_LOW(data) + 20) / 3;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_mmdc, &data);
	if (ret)
		return ret;

	/* Convert LSB from 0.08A to 0.5A */
	hist.maxchgcurr = REG_HALF_HIGH(data) * 8 / 50;
	hist.maxdischgcurr = REG_HALF_LOW(data) * 8 / 50;

	memcpy(buff, &hist, sizeof(hist));
	return (size_t)sizeof(hist);
}
EXPORT_SYMBOL_GPL(maxfg_collect_history_data);