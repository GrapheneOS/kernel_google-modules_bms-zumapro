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

		if (ret < 0 || ret != sizeof(hist))
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

#define REG_HALF_HIGH(reg)     ((reg >> 8) & 0x00FF)
#define REG_HALF_LOW(reg)      (reg & 0x00FF)
int maxfg_collect_history_data(void *buff, size_t size, bool is_por, u16 designcap,
			       struct maxfg_regmap *regmap, struct maxfg_regmap *regmap_debug)
{
	struct maxfg_eeprom_history hist = { 0 };
	u16 data;
	int temp, ret;

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

	if (!designcap) {
		ret = maxfg_reg_read(regmap, MAXFG_TAG_descap, &designcap);
		if (ret)
			return ret;
	}

	/* multiply by 100 to convert from mAh to %, LSB 0.125% */
	ret = maxfg_reg_read(regmap, MAXFG_TAG_fcnom, &data);
	if (ret)
		return ret;

	temp = (int)data * 800 / (int)designcap;
	hist.fullcapnom = temp > MAX_HIST_FULLCAP ? MAX_HIST_FULLCAP : temp;

	/* multiply by 100 to convert from mAh to %, LSB 0.125% */
	ret = maxfg_reg_read(regmap, MAXFG_TAG_fcrep, &data);
	if (ret)
		return ret;

	temp = (int)data * 800 / (int)designcap;
	hist.fullcaprep = temp > MAX_HIST_FULLCAP ? MAX_HIST_FULLCAP : temp;

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
	hist.maxtemp = ((s8)REG_HALF_HIGH(data) - 25) / 3;
	/* Convert LSB from 1degC to 3degC, store values from -20degC min */
	hist.mintemp = ((s8)REG_HALF_LOW(data) + 20) / 3;

	ret = maxfg_reg_read(regmap, MAXFG_TAG_mmdc, &data);
	if (ret)
		return ret;

	/* Convert LSB from 0.08A to 0.5A */
	hist.maxchgcurr = (s8)REG_HALF_HIGH(data) * 8 / 50;
	hist.maxdischgcurr = (s8)REG_HALF_LOW(data) * 8 / 50;

	memcpy(buff, &hist, sizeof(hist));
	return (size_t)sizeof(hist);
}

/* resistance and impedance ------------------------------------------------ */

int maxfg_read_resistance_avg(u16 RSense)
{
	u16 ravg;
	int ret = 0;

	ret = gbms_storage_read(GBMS_TAG_RAVG, &ravg, sizeof(ravg));
	if (ret < 0)
		return ret;

	return reg_to_resistance_micro_ohms(ravg, RSense);
}

int maxfg_read_resistance_raw(struct maxfg_regmap *map)
{
	u16 data;
	int ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_rslow, &data);
	if (ret < 0)
		return ret;

	return data;
}

int maxfg_read_resistance(struct maxfg_regmap *map, u16 RSense)
{
	int rslow;

	rslow = maxfg_read_resistance_raw(map);
	if (rslow < 0)
		return rslow;

	return reg_to_resistance_micro_ohms(rslow, RSense);
}

/* ----------------------------------------------------------------------- */

/* will return error if the value is not valid  */
int maxfg_health_get_ai(struct device *dev, int bhi_acim, u16 RSense)
{
	u16 act_impedance, act_timerh;
	int ret;

	if (bhi_acim != 0)
		return bhi_acim;

	/* read both and recalculate for compatibility */
	ret = gbms_storage_read(GBMS_TAG_ACIM, &act_impedance, sizeof(act_impedance));
	if (ret < 0)
		return -EIO;

	ret = gbms_storage_read(GBMS_TAG_THAS, &act_timerh, sizeof(act_timerh));
	if (ret < 0)
		return -EIO;

	/* need to get starting impedance (if qualified) */
	if (act_impedance == 0xffff || act_timerh == 0xffff)
		return -EINVAL;

	/* not zero, not negative */
	bhi_acim = reg_to_resistance_micro_ohms(act_impedance, RSense);

	/* TODO: correct impedance with timerh */

	dev_info(dev, "%s: bhi_acim =%d act_impedance=%x act_timerh=%x\n",
		 __func__, bhi_acim, act_impedance, act_timerh);

	return bhi_acim;
}

/* Capacity Estimation functions*/
static int batt_ce_regmap_read(struct maxfg_regmap *map, const struct maxfg_reg *bcea, u32 reg, u16 *data)
{
	int err;
	u16 val;

	if (!bcea)
		return -EINVAL;

	err = REGMAP_READ(map, bcea->map[reg], &val);
	if (err)
		return err;

	switch(reg) {
	case CE_DELTA_CC_SUM_REG:
	case CE_DELTA_VFSOC_SUM_REG:
		*data = val;
		break;
	case CE_CAP_FILTER_COUNT:
		val = val & 0x0F00;
		*data = val >> 8;
		break;
	default:
		break;
	}

	return err;
}

int batt_ce_load_data(struct maxfg_regmap *map, struct gbatt_capacity_estimation *cap_esti)
{
	u16 data;
	const struct maxfg_reg *bcea = cap_esti->bcea;

	cap_esti->estimate_state = ESTIMATE_NONE;
	if (batt_ce_regmap_read(map, bcea, CE_DELTA_CC_SUM_REG, &data) == 0)
		cap_esti->delta_cc_sum = data;
	else
		cap_esti->delta_cc_sum = 0;

	if (batt_ce_regmap_read(map, bcea, CE_DELTA_VFSOC_SUM_REG, &data) == 0)
		cap_esti->delta_vfsoc_sum = data;
	else
		cap_esti->delta_vfsoc_sum = 0;

	if (batt_ce_regmap_read(map, bcea, CE_CAP_FILTER_COUNT, &data) == 0)
		cap_esti->cap_filter_count = data;
	else
		cap_esti->cap_filter_count = 0;
	return 0;
}

void batt_ce_dump_data(const struct gbatt_capacity_estimation *cap_esti, struct logbuffer *log)
{
	logbuffer_log(log, "cap_filter_count: %d"
			    " start_cc: %d"
			    " start_vfsoc: %d"
			    " delta_cc_sum: %d"
			    " delta_vfsoc_sum: %d"
			    " state: %d"
			    " cable: %d",
			    cap_esti->cap_filter_count,
			    cap_esti->start_cc,
			    cap_esti->start_vfsoc,
			    cap_esti->delta_cc_sum,
			    cap_esti->delta_vfsoc_sum,
			    cap_esti->estimate_state,
			    cap_esti->cable_in);
}

static int batt_ce_regmap_write(struct maxfg_regmap *map,
				const struct maxfg_reg *bcea,
				u32 reg, u16 data)
{
	int err = -EINVAL;
	u16 val;

	if (!bcea)
		return -EINVAL;

	switch(reg) {
	case CE_DELTA_CC_SUM_REG:
	case CE_DELTA_VFSOC_SUM_REG:
		err = REGMAP_WRITE(map, bcea->map[reg], data);
		break;
	case CE_CAP_FILTER_COUNT:
		err = REGMAP_READ(map, bcea->map[reg], &val);
		if (err)
			return err;
		val = val & 0xF0FF;
		if (data > CE_FILTER_COUNT_MAX)
			val = val | 0x0F00;
		else
			val = val | (data << 8);
		err = REGMAP_WRITE(map, bcea->map[reg], val);
		break;
	default:
		break;
	}

	return err;
}

/* call holding &cap_esti->batt_ce_lock */
void batt_ce_store_data(struct maxfg_regmap *map, struct gbatt_capacity_estimation *cap_esti)
{
	if (cap_esti->cap_filter_count <= CE_FILTER_COUNT_MAX) {
		batt_ce_regmap_write(map, cap_esti->bcea,
					  CE_CAP_FILTER_COUNT,
					  cap_esti->cap_filter_count);
	}

	batt_ce_regmap_write(map, cap_esti->bcea,
				  CE_DELTA_VFSOC_SUM_REG,
				  cap_esti->delta_vfsoc_sum);
	batt_ce_regmap_write(map, cap_esti->bcea,
				  CE_DELTA_CC_SUM_REG,
				  cap_esti->delta_cc_sum);
}

/* call holding &cap_esti->batt_ce_lock */
void batt_ce_stop_estimation(struct gbatt_capacity_estimation *cap_esti, int reason)
{
	cap_esti->estimate_state = reason;
	cap_esti->start_vfsoc = 0;
	cap_esti->start_cc = 0;
}

int maxfg_health_write_ai(u16 act_impedance, u16 act_timerh)
{
	int ret;

	ret = gbms_storage_write(GBMS_TAG_ACIM, &act_impedance, sizeof(act_impedance));
	if (ret < 0)
		return -EIO;

	ret = gbms_storage_write(GBMS_TAG_THAS, &act_timerh, sizeof(act_timerh));
	if (ret < 0)
		return -EIO;

	return 0;
}

int maxfg_reg_log_data(struct maxfg_regmap *map, struct maxfg_regmap *map_debug, char *buf)
{
	u16 vfsoc, avcap, repcap, fullcap, fullcaprep, fullcapnom, qh0, qh, dqacc, dpacc, fstat;
	u16 qresidual, rcomp0, cycles, learncfg, tempco, filtercfg, mixcap, vfremcap, vcell, ibat;
	int ret, len;

	ret = maxfg_reg_read(map, MAXFG_TAG_vfsoc, &vfsoc);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_avcap, &avcap);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_repcap, &repcap);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_fulcap, &fullcap);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_fcrep, &fullcaprep);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_fcnom, &fullcapnom);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_qh0, &qh0);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_qh, &qh);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_dqacc, &dqacc);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_dpacc, &dpacc);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_qresd, &qresidual);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_fstat, &fstat);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_learn, &learncfg);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map_debug, MAXFG_TAG_tempco, &tempco);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map_debug, MAXFG_TAG_filcfg, &filtercfg);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_mcap, &mixcap);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_vfcap, &vfremcap);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_vcel, &vcell);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_curr, &ibat);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map_debug, MAXFG_TAG_rcomp0, &rcomp0);
	if (ret < 0)
		return ret;

	ret = maxfg_reg_read(map, MAXFG_TAG_cycles, &cycles);
	if (ret < 0)
		return ret;

	len = scnprintf(&buf[0], PAGE_SIZE, "%02X:%04X %02X:%04X %02X:%04X %02X:%04X"
			" %02X:%04X %02X:%04X %02X:%04X %02X:%04X %02X:%04X %02X:%04X"
			" %02X:%04X %02X:%04X %02X:%04X %02X:%04X %02X:%04X %02X:%04X"
			" %02X:%04X %02X:%04X %02X:%04X %02X:%04X %02X:%04X",
			MAXFG_TAG_vfsoc, vfsoc, MAXFG_TAG_avcap, avcap,
			MAXFG_TAG_repcap, repcap, MAXFG_TAG_fulcap, fullcap,
			MAXFG_TAG_fcrep, fullcaprep, MAXFG_TAG_fcnom, fullcapnom,
			MAXFG_TAG_qh0, qh0, MAXFG_TAG_qh, qh, MAXFG_TAG_dqacc, dqacc,
			MAXFG_TAG_dpacc, dpacc, MAXFG_TAG_qresd, qresidual,
			MAXFG_TAG_fstat, fstat, MAXFG_TAG_learn, learncfg,
			MAXFG_TAG_tempco, tempco, MAXFG_TAG_filcfg, filtercfg,
			MAXFG_TAG_mcap, mixcap, MAXFG_TAG_vfcap, vfremcap,
			MAXFG_TAG_vcel, vcell, MAXFG_TAG_curr, ibat,
			MAXFG_TAG_rcomp0, rcomp0, MAXFG_TAG_cycles, cycles);

	return len;
}
