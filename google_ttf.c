/*
 * Copyright 2018 Google, LLC
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

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <misc/logbuffer.h>
#include <linux/slab.h>
#include "google_bms.h"
#include "google_psy.h"
#include "qmath.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define ELAP_LIMIT_S 60


void ttf_log(const struct batt_ttf_stats *stats, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	logbuffer_vlog(stats->ttf_log, fmt, args);
	va_end(args);
}

/* actual adapter current capability for this charging event
 * NOTE: peformance for a tier are known only after entering the tier
 */
static int ttf_pwr_icl(const struct gbms_charging_event *ce_data,
		       int temp_idx, int vbatt_idx)
{
	const struct gbms_ce_tier_stats *ts = &ce_data->tier_stats[vbatt_idx];
	int elap, amperage;

	elap = ts->time_fast + ts->time_taper;
	if (elap <= ELAP_LIMIT_S)
		amperage = ce_data->adapter_details.ad_amperage * 100;
	else
		amperage = ts->icl_sum / (elap + ts->time_other);

	return amperage;
}

/* NOTE: the current in taper might need to be accounted in a different way */
static int ttf_pwr_ibatt(const struct gbms_charging_event *ce_data,
		         int temp_idx, int vbatt_idx)
{
	const struct gbms_ce_tier_stats *ts = &ce_data->tier_stats[vbatt_idx];
	int avg_ibatt, elap, sign = 1;

	elap = ts->time_fast + ts->time_taper;
	if (elap <= ELAP_LIMIT_S) {
		pr_debug("%d,%d: fast=%d taper=%d other=%d limit=%d\n",
			vbatt_idx, temp_idx,
			ts->time_fast, ts->time_taper, ts->time_other,
			ELAP_LIMIT_S);
		return 0;
	}

	/* actual */
	avg_ibatt = ts->ibatt_sum / (elap + ts->time_other);
	if (avg_ibatt < 0)
		sign = -1;

	pr_debug("%d,%d: fast=%d taper=%d other=%d avg_ibatt=%d\n",
		vbatt_idx, temp_idx,
		ts->time_fast, ts->time_taper, ts->time_other,
		avg_ibatt * sign);

	return avg_ibatt * sign;
}

/* nominal voltage tier index for this soc */
static int ttf_pwr_tier(const struct batt_ttf_stats *stats, int soc)
{
	int i;

	for (i = 1; i < GBMS_STATS_TIER_COUNT; i++)
		if (soc < stats->tier_stats[i].soc_in >> 8)
			break;

	return i - 1;
}

/* nominal average current demand for this tier at max rate
 * NOTE: tier and soc stats keep track of aging (might not need)
 */
static int ttf_pwr_avg_cc(const struct batt_ttf_stats *stats, int soc)
{
	const struct ttf_soc_stats *sstat = NULL;
	int delta_cc;

	/* soc average current demand */
	if (stats->soc_stats.cc[soc] && stats->soc_stats.elap[soc])
		sstat = &stats->soc_stats;
	else if (stats->soc_ref.cc[soc] && stats->soc_ref.elap[soc])
		sstat = &stats->soc_ref;
	else
		return 0;

	delta_cc = (sstat->cc[soc + 1] - sstat->cc[soc]);

	return (delta_cc * 3600) / sstat->elap[soc];
}

/* time scaling factor for available power and SOC demand.
 * NOTE: usually called when soc < ssoc_in && soc > ce_data->last_soc
 * TODO: this is very inefficient
 */
static int ttf_pwr_ratio(const struct batt_ttf_stats *stats,
			 const struct gbms_charging_event *ce_data,
			 int soc)
{
	int ratio;
	int avg_cc, pwr_demand;
	int act_icl, pwr_avail;
	int act_ibatt, cc_max;
	int vbatt_idx, temp_idx;
	const struct gbms_chg_profile *profile = ce_data->chg_profile;

	vbatt_idx = ttf_pwr_tier(stats, soc);
	if (vbatt_idx < 0)
		return -EINVAL;

	/* TODO: compensate with average increase/decrease of temperature? */
	temp_idx = ce_data->tier_stats[vbatt_idx].temp_idx;
	if (temp_idx == -1) {
		int64_t t_avg = 0;
		const int elap = ce_data->tier_stats[vbatt_idx].time_fast +
		 		 ce_data->tier_stats[vbatt_idx].time_taper +
		 		 ce_data->tier_stats[vbatt_idx].time_other;

		if (ce_data->tier_stats[vbatt_idx].temp_sum != 0 || elap == 0)
			t_avg = ce_data->tier_stats[vbatt_idx].temp_in;
		if (t_avg == 0)
			t_avg = 250;

		temp_idx = gbms_msc_temp_idx(profile, t_avg);
		pr_debug("%d: temp_idx=%d t_avg=%ld sum=%ld elap=%d\n",
			soc, temp_idx, t_avg,
			ce_data->tier_stats[vbatt_idx].temp_sum,
			elap);

		if (temp_idx < 0)
			return -EINVAL;
	}

	/* max tier demand at this temperature index */
	cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx) / 1000;
	/* statistical demand for soc, account for taper */
	avg_cc = ttf_pwr_avg_cc(stats, soc);
	if (avg_cc <= 0 || avg_cc > cc_max) {
		pr_debug("%d: demand use default avg_cc=%d->%d\n",
			soc, avg_cc, cc_max);
		avg_cc = cc_max;
	}

	/* actual battery power demand */
	pwr_demand = (profile->volt_limits[vbatt_idx] / 10000) * avg_cc;
	pr_debug("%d:%d,%d: pwr_demand=%d avg_cc=%d cc_max=%d\n",
		soc, temp_idx, vbatt_idx, pwr_demand,
		avg_cc, cc_max);

	/* actual adapter current capabilities for this tier */
	act_icl = ttf_pwr_icl(ce_data, temp_idx, vbatt_idx);
	if (act_icl <= 0) {
		pr_debug("%d: negative, null act_icl=%d\n", soc, act_icl);
		return -EINVAL;
	}

	/* compensate for temperature (might not need) */
	if (temp_idx != stats->ref_temp_idx && cc_max < act_icl) {
		pr_debug("%d: temp_idx=%d, reduce icl %d->%d\n",
			soc, temp_idx, act_icl, cc_max);
		act_icl = cc_max;
	}

	/* compensate for system load
	 * NOTE: act_ibatt = 0 means no system load, need to fix later
	 */
	act_ibatt = ttf_pwr_ibatt(ce_data, temp_idx, vbatt_idx);
	if (act_ibatt < 0) {
		pr_debug("%d: ibatt=%d, discharging\n", soc, act_ibatt);
		return -EINVAL;
	} else if (act_ibatt > 0 && act_ibatt < act_icl) {
		pr_debug("%d: sysload ibatt=%d, avg_cc=%d, reduce icl %d->%d\n",
			soc, act_ibatt, avg_cc, act_icl, act_ibatt);
		act_icl = act_ibatt;
	}

	pwr_avail = (ce_data->adapter_details.ad_voltage * 10) * act_icl;
	if (!pwr_avail)
		return -EINVAL;

	/* TODO: scale for efficiency? */

	if (pwr_avail < pwr_demand)
		ratio = (stats->ref_watts * 100000) / pwr_avail;
	else
		ratio = 100;

	pr_debug("%d: pwr_avail=%d, pwr_demand=%d ratio=%d\n",
		soc, pwr_avail, pwr_demand, ratio);

	return ratio;
}

/* SOC estimates ---------------------------------------------------------  */

static int ttf_elap(ktime_t *estimate, int i, const struct batt_ttf_stats *stats,
		    const struct gbms_charging_event *ce_data)
{
	int ratio;
	ktime_t elap;

	if (i < 0 || i >= 100) {
		*estimate = 0;
		return 0;
	}

	elap = stats->soc_stats.elap[i];
	if (elap == 0)
		elap = stats->soc_ref.elap[i];

	ratio = ttf_pwr_ratio(stats, ce_data, i);
	if (ratio < 0) {
		pr_debug("%d: negative ratio=%d\n", i, ratio);
		return -EINVAL;
	}

	pr_debug("i=%d elap=%ld ratio=%d\n", i, elap, ratio);
	*estimate = elap * ratio;

	return 0;
}

/* time to full from SOC%
 * NOTE: prediction is based stats and corrected with the ce_data
 * NOTE: usually called with soc > ce_data->last_soc
 */
int ttf_soc_estimate(ktime_t *res,
		     const struct batt_ttf_stats *stats,
		     const struct gbms_charging_event *ce_data,
		     qnum_t soc, qnum_t last)
{
	const int ssoc_in = ce_data->charging_stats.ssoc_in;
	const int end = qnum_toint(last);
	const int frac = qnum_fracdgt(soc);
	int i = qnum_toint(soc);
	ktime_t estimate = 0;
	int ret;

	if (end > 100)
		return -EINVAL;

	ret = ttf_elap(&estimate, i, stats, ce_data);
	if (ret < 0)
		return ret;

	/* add ttf_elap starting from i + 1 */
	estimate = (estimate * (100 - frac)) / 100;
	for (i += 1; i < end; i++) {
		ktime_t elap;

		if (i >= ssoc_in && i < ce_data->last_soc) {
			/* use real data if within charging event */
			elap = ce_data->soc_stats.elap[i] * 100;
		} else {
			/* future (and soc before ssoc_in) */
			ret = ttf_elap(&elap, i, stats, ce_data);
			if (ret < 0)
				return ret;
		}

		estimate += elap;
	}

	*res = estimate / 100;
	return 0;
}

int ttf_soc_cstr(char *buff, int size,
		 const struct ttf_soc_stats *soc_stats,
		 int start, int end)
{
	int i, len = 0, split = 100;

	if (start < 0 || start >= GBMS_SOC_STATS_LEN ||
	    end < 0 || end >= GBMS_SOC_STATS_LEN ||
	    start > end)
		return 0;

	/* only one way to print data @ 100 */
	if (end == 100 && start != 100)
		end = 99;
	/* std newline every 10 entries */
	if (start == 0 && end == 99)
		split = 10;

	/* dump elap time as T: */
	for (i = start; i <= end; i++) {
		if (i % split == 0 || i == start) {
			len += scnprintf(&buff[len], size - len, "T");
			if (split == 10)
				len += scnprintf(&buff[len], size - len,
						"%d", i / 10);
			len += scnprintf(&buff[len], size - len, ":");
		}

		len += scnprintf(&buff[len], size - len, " %4ld",
				soc_stats->elap[i]);
		if (i != end && (i + 1) % split == 0)
			len += scnprintf(&buff[len], size - len, "\n");
	}

	len += scnprintf(&buff[len], size - len, "\n");

	/* dump coulumb count as C: */
	for (i = start; i <= end; i++) {
		if (i % split == 0 || i == start) {
			len += scnprintf(&buff[len], size - len, "C");
			if (split == 10)
				len += scnprintf(&buff[len], size - len,
						 "%d", i / 10);
			len += scnprintf(&buff[len], size - len, ":");
		}

		len += scnprintf(&buff[len], size - len, " %4d",
				soc_stats->cc[i]);
		if (i != end && (i + 1) % split == 0)
			len += scnprintf(&buff[len], size - len, "\n");
	}

	len += scnprintf(&buff[len], size - len, "\n");

	return len;
}

/* TODO: tune these values */

/* discard updates for adapters that have less than 80% of nominal */
#define TTF_SOC_QUAL_ELAP_RATIO_MAX	200
/* cap updates of cc to no more of +-*_CUR_ABS_MAX from previous */
#define TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX	60
/* cap udpdates to cc max to no more of +-20% of reference */
#define TTF_SOC_QUAL_ELAP_DELTA_REF_PCT_MAX	20

/* return the weight to apply to this change */
static ktime_t ttf_soc_qual_elap(const struct batt_ttf_stats *stats,
				 const struct gbms_charging_event *ce_data,
				 int i)
{
	const struct ttf_soc_stats *src = &ce_data->soc_stats;
	const struct ttf_soc_stats *dst = &stats->soc_stats;
	const int limit = TTF_SOC_QUAL_ELAP_RATIO_MAX;
	const int max_elap = ((100 + TTF_SOC_QUAL_ELAP_DELTA_REF_PCT_MAX) *
			     stats->soc_ref.elap[i]) / 100;
	const int min_elap = ((100 - TTF_SOC_QUAL_ELAP_DELTA_REF_PCT_MAX) *
			     stats->soc_ref.elap[i]) / 100;
	ktime_t elap, elap_new, elap_cur;
	int ratio;

	if (!src->elap[i])
		return 0;

	/* weight the adapter, discard if ratio is too high (poor adapter) */
	ratio = ttf_pwr_ratio(stats, ce_data, i);
	if (ratio <= 0 || ratio > limit) {
		pr_debug("%d: ratio=%d limit=%d\n", i, ratio, limit);
		return 0;
	}

	elap_new = (src->elap[i] * 100) / ratio;
	elap_cur = dst->elap[i];
	if (!elap_cur)
		elap_cur = stats->soc_ref.elap[i];
	elap = (elap_cur + elap_new) / 2;

	/* bounds check to previous */
	if (elap > (elap_cur + TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX))
		elap = elap_cur + TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX;
	else if (elap < (elap_cur - TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX))
		elap = elap_cur - TTF_SOC_QUAL_ELAP_DELTA_CUR_ABS_MAX;

	/* bounds check to reference */
	if (elap > max_elap)
		elap = max_elap;
	else if (elap < min_elap)
		elap = min_elap;

	pr_debug("%d: dst->elap=%ld, ref_elap=%ld, elap=%ld, src_elap=%ld ratio=%d, min=%d max=%d\n",
		i, dst->elap[i], stats->soc_ref.elap[i], elap, src->elap[i],
		ratio, min_elap, max_elap);

	return elap;
}

/* cap updates of cc to no more of +-*_CUR_ABS_MAX from previous */
#define TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX	40
/* cap udpdates to cc max to no more of +-20% of reference */
#define TTF_SOC_QUAL_CC_DELTA_REF_PCT_MAX	20

static int ttf_soc_qual_cc(const struct batt_ttf_stats *stats,
			   const struct gbms_charging_event *ce_data,
			   int i)
{
	const struct ttf_soc_stats *src = &ce_data->soc_stats;
	const struct ttf_soc_stats *dst = &stats->soc_stats;
	const int max_cc = ((100 + TTF_SOC_QUAL_CC_DELTA_REF_PCT_MAX) *
			   stats->soc_ref.cc[i]) / 100;
	const int min_cc = ((100 - TTF_SOC_QUAL_CC_DELTA_REF_PCT_MAX) *
			   stats->soc_ref.cc[i]) / 100;
	int cc, cc_cur;

	if (!src->cc[i])
		return 0;

	cc_cur = dst->cc[i];
	if (cc_cur <= 0)
		cc_cur = stats->soc_ref.cc[i];

	cc = (cc_cur + src->cc[i]) / 2;

	/* bounds check to previous */
	if (cc > cc_cur + TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX)
		cc = cc_cur + TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX;
	else if (cc < cc_cur  -TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX)
		cc = cc_cur - TTF_SOC_QUAL_CC_DELTA_CUR_ABS_MAX;

	/* bounds check to reference */
	if (cc > max_cc)
		cc = max_cc;
	else if (cc < min_cc)
		cc = min_cc;

	pr_info("%d: cc_cur=%d, ref_cc=%d src->cc=%d, cc=%d\n",
		i, cc_cur, stats->soc_ref.cc[i], src->cc[i], cc);

	return cc;
}

/* update soc_stats using the charging event
 * NOTE: first_soc and last_soc are inclusive, will skip socs that have no
 * elap and no cc.
 */
static void ttf_soc_update(struct batt_ttf_stats *stats,
			   const struct gbms_charging_event *ce_data,
			   int first_soc, int last_soc)
{
	const struct ttf_soc_stats *src = &ce_data->soc_stats;
	int i;

	for (i = first_soc; i <= last_soc; i++) {
		ktime_t elap;
		int cc;

		/* need to have data on both */
		if (!src->elap[i] || !src->cc[i])
			continue;

		/* average the elap time at soc */
		elap = ttf_soc_qual_elap(stats, ce_data, i);
		if (elap)
			stats->soc_stats.elap[i] = elap;

		/* average the coulumb count at soc */
		cc = ttf_soc_qual_cc(stats, ce_data, i);
		if (cc)
			stats->soc_stats.cc[i] = cc;
	}
}

void ttf_soc_init(struct ttf_soc_stats *dst)
{
	memset(dst, 0, sizeof(*dst));
}

/* Tier estimates ---------------------------------------------------------  */

#define TTF_STATS_FMT "[%d,%d %d %ld]"

#define BATT_TTF_TS_VALID(ts) \
	(ts->cc_total != 0 && ts->avg_time != 0)

/* TODO: adjust for adapter capability */
static ktime_t ttf_tier_accumulate(const struct ttf_tier_stat *ts,
				  int vbatt_idx,
				  const struct batt_ttf_stats *stats)
{
	ktime_t estimate = 0;

	if (vbatt_idx >= GBMS_STATS_TIER_COUNT)
		return 0;

	for (; vbatt_idx < GBMS_STATS_TIER_COUNT; vbatt_idx++) {

		/* no data in this tier, sorry */
		if (!BATT_TTF_TS_VALID(ts))
			return -ENODATA;

		estimate += ts[vbatt_idx].avg_time;
	}

	return estimate;
}

/* */
static int ttf_tier_sscan(struct batt_ttf_stats *stats,
			  const char *buff,
			  size_t size)
{
	int j, len = 0;

	memset(&stats->tier_stats, 0, sizeof(*stats));

	while (buff[len] != '[' && len < size)
		len++;

	for (j = 0; j < GBMS_STATS_TIER_COUNT; j++) {
		sscanf(&buff[len], TTF_STATS_FMT, &stats->tier_stats[j].soc_in,
		       &stats->tier_stats[j].cc_in,
		       &stats->tier_stats[j].cc_total,
		       &stats->tier_stats[j].avg_time);

		len += sizeof(TTF_STATS_FMT) - 1;
	}

	return 0;
}

int ttf_tier_cstr(char *buff, int size, const struct ttf_tier_stat *tier_stats)
{
	int len = 0;

	len += scnprintf(&buff[len], size - len,
			 TTF_STATS_FMT,
			 tier_stats->soc_in >> 8,
			 tier_stats->cc_in,
			 tier_stats->cc_total,
			 tier_stats->avg_time);
	return len;
}

/* average soc_in, cc_in, cc_total an and avg time for charge tier */
static void ttf_tier_update_stats(struct ttf_tier_stat *ttf_ts,
			    	  const struct gbms_ce_tier_stats *chg_ts,
				  bool force)
{
	int elap;

	if (!force) {
		if (chg_ts->cc_total == 0)
			return;

		/* TODO: check dsg, qualify with adapter? */
	}

	/* TODO: check with -1 */
	if (ttf_ts->soc_in == 0)
		ttf_ts->soc_in = chg_ts->soc_in;
	 ttf_ts->soc_in = (ttf_ts->soc_in + chg_ts->soc_in) / 2;

	/* TODO: check with -1 */
	if (ttf_ts->cc_in == 0)
		ttf_ts->cc_in = chg_ts->cc_in;
	 ttf_ts->cc_in = (ttf_ts->cc_in + chg_ts->cc_in) / 2;

	if (ttf_ts->cc_total == 0)
		 ttf_ts->cc_total = chg_ts->cc_total;
	 ttf_ts->cc_total = (ttf_ts->cc_total + chg_ts->cc_total) / 2;

	/* */
	elap = chg_ts->time_fast + chg_ts->time_taper + chg_ts->time_other;
	if (ttf_ts->avg_time == 0)
		ttf_ts->avg_time = elap;

	/* qualify time with ratio */
	ttf_ts->avg_time =(ttf_ts->avg_time + elap) / 2;
}

/* updated tier stats using the charging event
 * NOTE: the ce has data from 1+ charging voltage and temperature tiers */
static void ttf_tier_update(struct batt_ttf_stats *stats,
			    const struct gbms_charging_event *data,
			    bool force)
{
	int i;

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const bool last_tier = i == (GBMS_STATS_TIER_COUNT - 1);
		const struct gbms_ce_tier_stats *chg_ts = &data->tier_stats[i];
		const struct gbms_ce_stats *chg_s = &data->charging_stats;
		long elap;

		/* skip data that has a temperature switch */
		if (chg_ts->temp_idx == -1)
			continue;
		/* or entries that have no actual charging */
		elap = chg_ts->time_fast + chg_ts->time_taper;
		if (!elap)
			continue;
		/* update first tier stats only at low soc_in */
		if (!force && i == 0 && (chg_ts->soc_in >> 8) > 1)
			continue;
		/* update last tier stats only at full */
		if (!force && last_tier && ((chg_s->ssoc_out >> 8) != 100))
			continue;

		/*  */
		ttf_tier_update_stats(&stats->tier_stats[i], chg_ts, false);
	}

}

/* tier estimates only, */
int ttf_tier_estimate(ktime_t *res, const struct batt_ttf_stats *stats,
		      int temp_idx, int vbatt_idx,
		      int capacity, int full_capacity)
{
	ktime_t estimate = 0;
	const struct ttf_tier_stat *ts;

	/* tier estimates, only when in tier */
	if (vbatt_idx == -1 && temp_idx == -1)
		return -EINVAL;

	ts = &stats->tier_stats[vbatt_idx];
	if (!ts || !BATT_TTF_TS_VALID(ts))
		return -ENODATA;

	/* accumulate next tier */
	estimate = ttf_tier_accumulate(ts, vbatt_idx + 1, stats);
	if (estimate < 0)
		return -ENODATA;

	/* eyeball current tier
	 * estimate =
	 * 	(ts->cc_in + ts->cc_total - capacity) *
	 * 	rs->avg_time) / ts->cc_total
	 */

	/* TODO: adjust for crossing thermals? */

	*res = estimate;
	return 0;
}

/* ----------------------------------------------------------------------- */

/* QUAL DELTA >= 3 */
#define TTF_STATS_QUAL_DELTA_MIN	3
#define TTF_STATS_QUAL_DELTA		TTF_STATS_QUAL_DELTA_MIN

static int ttf_soc_cstr_elap(char *buff, int size,
			     const struct ttf_soc_stats *soc_stats,
			     int start, int end)
{
	int i, len = 0;

	len += scnprintf(&buff[len], size - len, "T%d:", start);
	for (i = start; i < end; i++)
		len += scnprintf(&buff[len], size - len, " %4ld",
				 soc_stats->elap[i]);

	return len;
}

static int ttf_soc_cstr_cc(char *buff, int size,
			   const struct ttf_soc_stats *soc_stats,
			   int start, int end)
{
	int i, len = 0;

	len += scnprintf(&buff[len], size - len, "C%d:", start);
	for (i = start; i < end; i++)
		len += scnprintf(&buff[len], size - len, " %4d",
				 soc_stats->cc[i]);

	return len;
}

/* update ttf tier and soc stats using the charging event.
 * call holding stats->lock
 */
void ttf_stats_update(struct batt_ttf_stats *stats,
		      struct gbms_charging_event *ce_data,
		      bool force)
{
	int first_soc = ce_data->charging_stats.ssoc_in;
	const int last_soc = ce_data->last_soc;
	const int delta_soc = last_soc - first_soc;
	const int limit = force ? TTF_STATS_QUAL_DELTA_MIN :
			  TTF_STATS_QUAL_DELTA;
	const int tmp_size = PAGE_SIZE;
	char *tmp;

	/* skip data short periods */
	if (delta_soc < limit) {
		ttf_log(stats, "no updates delta_soc=%d, limit=%d, force=%d",
			delta_soc, limit, force);
		return;
	}

	/* ignore fist nozero and last entry because they are partial */
	for ( ; first_soc <= last_soc; first_soc++)
		if (ce_data->soc_stats.elap[first_soc] != 0)
			break;

	ttf_soc_update(stats, ce_data, first_soc + 1, last_soc - 1);
	ttf_tier_update(stats, ce_data, force);

	/* dump update stats to logbuffer */
	tmp = kzalloc(tmp_size, GFP_KERNEL);
	if (tmp) {
		const int split = 10;
		int i;

		for (i = first_soc + 1;i < last_soc - 1; i += split) {
			int end_soc = i + split;

			if (end_soc > last_soc - 1)
				end_soc = last_soc - 1;

			ttf_soc_cstr_elap(tmp, tmp_size, &stats->soc_stats,
					  i, end_soc);
			ttf_log(stats, "%s", tmp);
			ttf_soc_cstr_cc(tmp, tmp_size, &stats->soc_stats,
					  i,  end_soc);
			ttf_log(stats, "%s", tmp);
		}

		kfree(tmp);
	}
}

static int ttf_init_soc_parse_dt(struct ttf_adapter_stats *as,
				 struct device *device)
{
	int table_count;
	int ret;

	table_count = of_property_count_elems_of_size(device->of_node,
						      "google,ttf-soc-table",
						      sizeof(u32));
	if (table_count <= 0)
		return -EINVAL;
	if (table_count % 2)
		return -EINVAL;

	as->soc_table = devm_kzalloc(device, table_count * 2 * sizeof(u32),
				     GFP_KERNEL);
	if (!as->soc_table)
		return -ENOMEM;

	ret = of_property_read_u32_array(device->of_node,
					"google,ttf-soc-table",
					as->soc_table, table_count);
	if (ret < 0) {
		pr_err("cannot read google,ttf-soc-table %d\n", ret);
		return ret;
	}

	as->elap_table = &as->soc_table[table_count];
	ret = of_property_read_u32_array(device->of_node,
					"google,ttf-elap-table",
					as->elap_table, table_count);
	if (ret < 0) {
		pr_err("cannot read google,ttf-elap-table %d\n", ret);
		return ret;
	}

	as->table_count = table_count;
	return 0;
}

int ttf_stats_sscan(struct batt_ttf_stats *stats,
		    const char *buff,
		    size_t size)
{
	/* TODO: scan ttf_soc_* data as well */

	return ttf_tier_sscan(stats, buff, size);
}

static int ttf_as_default(struct ttf_adapter_stats *as, int i, int table_i)
{
	while (i > as->soc_table[table_i] && table_i < as->table_count)
		table_i++;

	return table_i;
}

static int ttf_init_tier_parse_dt(struct batt_ttf_stats *stats,
				  struct device *device)
{
	int i, count, ret;
	u32 tier_table[GBMS_STATS_TIER_COUNT];

	count = of_property_count_elems_of_size(device->of_node,
						"google,ttf-tier-table",
						sizeof(u32));
	if (count != GBMS_STATS_TIER_COUNT)
		return -EINVAL;

	ret = of_property_read_u32_array(device->of_node,
					"google,ttf-tier-table",
					tier_table, count);
	if (ret < 0) {
		pr_err("cannot read google,ttf-tier-table %d\n", ret);
		return ret;
	}

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++)
		stats->tier_stats[i].soc_in = tier_table[i] << 8;

	return 0;
}

/* clone and clear the stats */
struct batt_ttf_stats *ttf_stats_dup(struct batt_ttf_stats *dst,
				     const struct batt_ttf_stats *src)
{
	memcpy(dst, src, sizeof(*dst));
	memset(&dst->soc_stats, 0, sizeof(dst->soc_stats));
	memset(&dst->tier_stats, 0, sizeof(dst->tier_stats));
	return dst;
}

static void ttf_init_ref_table(struct batt_ttf_stats *stats,
			       struct ttf_adapter_stats *as,
			       int capacity_ma)
{
	int i, table_i = 0;
	const int cc = (capacity_ma * 100) / GBMS_SOC_STATS_LEN;

	for (i = 0; i < GBMS_SOC_STATS_LEN; i++) {
		table_i = ttf_as_default(as, i, table_i);

		stats->soc_ref.elap[i] = as->elap_table[table_i];

		/* assume same cc for each soc */
		stats->soc_ref.cc[i] = (cc * i) / 100;
	}

	/* TODO: allocate as->soc_table witk kzalloc, free here */
}

/* must come after charge profile */
int ttf_stats_init(struct batt_ttf_stats *stats, struct device *device,
		   int capacity_ma)
{
	struct ttf_adapter_stats as;
	u32 value;
	int ret;

	memset(stats, 0, sizeof(*stats));
	stats->ttf_fake = -1;

	/* reference adapter */
	ret = of_property_read_u32(device->of_node, "google,ttf-adapter",
				   &value);
	if (ret < 0)
		return ret;

	stats->ref_watts = value;

	/* reference temperature  */
	ret = of_property_read_u32(device->of_node, "google,ttf-temp-idx",
				   &value);
	if (ret < 0)
		return ret;

	stats->ref_temp_idx = value;

	/* reference soc estimates */
	ret = ttf_init_soc_parse_dt(&as, device);
	if (ret < 0)
		return ret;

	/* reference tier-based statistics */
	ret = ttf_init_tier_parse_dt(stats, device);
	if (ret < 0)
		return ret;

	/* initialize the reference stats for the reference soc estimates */
	ttf_init_ref_table(stats, &as, capacity_ma);


	/* TODO: use the soc stats to calculate cc_in */
	stats->tier_stats[0].cc_in = 0;
	stats->tier_stats[1].cc_in = (capacity_ma *
					stats->tier_stats[1].soc_in) /
					100;
	stats->tier_stats[2].cc_in = (capacity_ma *
					stats->tier_stats[2].soc_in) /
					100;

	/* TODO: use the soc stats to calculate cc_total */
	stats->tier_stats[0].cc_total = 0;
	stats->tier_stats[1].cc_total = (capacity_ma *
					(stats->tier_stats[2].soc_in -
					stats->tier_stats[1].soc_in)) /
					100;
	stats->tier_stats[2].cc_total = capacity_ma -
					stats->tier_stats[2].cc_in;


	return 0;
}


/* tier and soc details */
ssize_t ttf_dump_details(char *buf, int max_size,
			 const struct batt_ttf_stats *ttf_stats,
			 int last_soc)
{
	int i, len = 0;

	/* interleave tier with SOC data */
	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		int next_soc_in;

		len += scnprintf(&buf[len], max_size - len, "%d: ", i);
		len += ttf_tier_cstr(&buf[len], max_size - len,
				     &ttf_stats->tier_stats[i]);
		len += scnprintf(&buf[len], max_size - len, "\n");

		/* continue only first */
		if (ttf_stats->tier_stats[i].avg_time == 0)
			continue;

		if (i == GBMS_STATS_TIER_COUNT - 1) {
			next_soc_in = -1;
		} else {
			next_soc_in = ttf_stats->tier_stats[i + 1].soc_in >> 8;
			if (next_soc_in == 0)
				next_soc_in = -1;
		}

		if (next_soc_in == -1)
			next_soc_in = last_soc - 1;

		len += ttf_soc_cstr(&buf[len], max_size - len,
				    &ttf_stats->soc_stats,
				    ttf_stats->tier_stats[i].soc_in >> 8,
				    next_soc_in);
	}

	return len;
}
