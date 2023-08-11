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

int maxfg_get_fade_rate(struct device *dev, int bhi_fcn_count)
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
	if (ratio > 100)
		ratio = 100;

	return 100 - ratio;
}
EXPORT_SYMBOL_GPL(maxfg_get_fade_rate);