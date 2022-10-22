// SPDX-License-Identifier: GPL-2.0
/*
 * WC68 Direct Charger PPS Integration
 * Based on PPS integration for PCA9468
 *
 * Copyright (C) 2022 Google, LLC
 *
 */


#include <linux/err.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/of_device.h>

#include "wc68_regs.h"
#include "wc68_charger.h"

/* Logging ----------------------------------------------------------------- */

int debug_printk_prlog = LOGLEVEL_INFO;
int debug_no_logbuffer;

/* DC PPS integration ------------------------------------------------------ */

static void wc68_chg_stats_set_apdo(struct wc68_chg_stats *chg_data, u32 apdo);

static struct device_node *wc68_find_config(struct device_node *node)
{
	struct device_node *temp;

	if (!node)
		return node;
	temp = of_parse_phandle(node, "wc68,google_cpm", 0);
	if (temp)
		node = temp;
	return node;
}

int wc68_probe_pps(struct wc68_charger *wc68_chg)
{
	const char *tmp_name = NULL;
	bool pps_available = false;
	struct device_node *node;
	int ret;

	node = wc68_find_config(wc68_chg->dev->of_node);
	if (!node)
		return -ENODEV;

	ret = of_property_read_u32(node, "google,tcpm-power-supply",
				   &wc68_chg->tcpm_phandle);
	if (ret < 0)
		dev_warn(wc68_chg->dev, "wc68: pca,tcpm-power-supply not defined\n");
	else
		pps_available |= true;

	ret = of_property_read_string(node, "google,wlc_dc-power-supply",
				      &tmp_name);
	if (ret < 0)
		dev_warn(wc68_chg->dev, "wc68: google,wlc_dc-power-supply not defined\n");
	if (ret == 0) {
		wc68_chg->wlc_psy_name =
			devm_kstrdup(wc68_chg->dev, tmp_name, GFP_KERNEL);
		if (!wc68_chg->wlc_psy_name)
			return -ENOMEM;

		pps_available |= true;
	}

	return pps_available ? 0 : -ENODEV;
}

/* ------------------------------------------------------------------------ */

/* switch PDO if needed */
int wc68_request_pdo(struct wc68_charger *wc68)
{
	int ret = 0;

	dev_dbg(wc68->dev, "%s: ta_objpos=%u, ta_vol=%u, ta_cur=%u\n", __func__,
		  wc68->ta_objpos, wc68->ta_vol, wc68->ta_cur);

	/*
	 * the reference implementation call pps_request_pdo() twice with a
	 * 100 ms delay between the calls when the function returns -EBUSY:
	 *
	 * 	ret = pps_request_pdo(&wc68->pps_data, wc68->ta_objpos,
	 *				wc68->ta_vol, wc68->ta_cur,
	 * 				wc68->pd);
	 *
	 * The wrapper in google_dc_pps route the calls to the tcpm engine
	 * via tcpm_update_sink_capabilities(). The sync capabilities are
	 * in pps_data, ->ta_objpos select the (A)PDO index, ->ta_vol and
	 * ->ta_cur are the desired TA voltage and current.
	 *
	 * this is now handled by pps_update_adapter()
	 *
	 * TODO: verify the timing and make sure that there are no races that
	 * cause the targets
	 */

	return ret;
}

int wc68_usbpd_setup(struct wc68_charger *wc68)
{
	struct power_supply *tcpm_psy;
	bool online;
	int ret = 0;

	if (wc68->pd != NULL)
		goto check_online;

	if (wc68->tcpm_psy_name) {
		tcpm_psy = power_supply_get_by_name(wc68->tcpm_psy_name);
		if (!tcpm_psy)
			return -ENODEV;

		wc68->pd = tcpm_psy;
	} else if (wc68->tcpm_phandle) {
		struct device_node *node;

		node = wc68_find_config(wc68->dev->of_node);
		if (!node)
			return -ENODEV;
		tcpm_psy = pps_get_tcpm_psy(node, 2);
		if (IS_ERR(tcpm_psy))
			return PTR_ERR(tcpm_psy);
		if (!tcpm_psy) {
			wc68->tcpm_phandle = 0;
			return -ENODEV;
		}

		dev_err(wc68->dev, "%s: TCPM name is %s\n", __func__, pps_name(tcpm_psy));
		wc68->tcpm_psy_name = tcpm_psy->desc->name;
		wc68->pd = tcpm_psy;
	} else {
		dev_err(wc68->dev, "%s: TCPM DC not defined\n", __func__);
		return -ENODEV;
	}

	/* not needed if tcpm-power-supply is not there */
	ret = pps_init(&wc68->pps_data, wc68->dev, tcpm_psy);
	if (ret == 0) {
		pps_set_logbuffer(&wc68->pps_data, wc68->log);
		pps_init_state(&wc68->pps_data);
	}

check_online:
	online = pps_prog_check_online(&wc68->pps_data, wc68->pd);
	if (!online)
		return -ENODEV;

	return ret;
}

/* call holding mutex_unlock(&wc68->lock); */
int wc68_send_pd_message(struct wc68_charger *wc68,
				   unsigned int msg_type)
{
	struct pd_pps_data *pps_data = &wc68->pps_data;
	struct power_supply *tcpm_psy = wc68->pd;
	bool online;
	int pps_ui;
	int ret;

	if (!tcpm_psy || (wc68->charging_state == DC_STATE_NO_CHARGING &&
	    msg_type == PD_MSG_REQUEST_APDO) || !wc68->mains_online) {
		dev_dbg(wc68->dev, "%s: failure tcpm_psy_ok=%d charging_state=%u online=%d",
			__func__,  tcpm_psy != 0, wc68->charging_state,
			wc68->mains_online);
		return -EINVAL;
	}

	/* false when offline (0) or not in prog (1) mode */
	online = pps_prog_check_online(&wc68->pps_data, tcpm_psy);
	if (!online) {
		dev_dbg(wc68->dev, "%s: not online", __func__);
		return -EINVAL;
	}

	/* turn off PPS/PROG, revert to PD */
	if (msg_type == MSG_REQUEST_FIXED_PDO) {
		ret = pps_prog_offline(&wc68->pps_data, tcpm_psy);
		dev_dbg(wc68->dev, "%s: requesting offline ret=%d\n", __func__, ret);
		/* TODO: reset state? */
		return ret;
	}

	dev_dbg(wc68->dev, "%s: tcpm_psy_ok=%d pd_online=%d pps_stage=%d charging_state=%u",
		__func__,  tcpm_psy != 0,  pps_data->pd_online,
		pps_data->stage, wc68->charging_state);

	if (wc68->pps_data.stage == PPS_ACTIVE) {

		/* not sure I need to do this */
		ret = wc68_request_pdo(wc68);
		if (ret == 0) {
			const int pre_out_uv = pps_data->out_uv;
			const int pre_out_ua = pps_data->op_ua;

			dev_dbg(wc68->dev, "%s: ta_vol=%u, ta_cur=%u, ta_objpos=%u\n",
				__func__, wc68->ta_vol, wc68->ta_cur,
				wc68->ta_objpos);

			pps_ui = pps_update_adapter(&wc68->pps_data,
						    wc68->ta_vol,
						    wc68->ta_cur,
						    tcpm_psy);
			dev_dbg(wc68->dev, "%s: out_uv=%d %d->%d, out_ua=%d %d->%d (%d)\n",
				 __func__,
				 pps_data->out_uv, pre_out_uv, wc68->ta_vol,
				 pps_data->op_ua, pre_out_ua, wc68->ta_cur,
				 pps_ui);

			if (pps_ui == 0)
				pps_ui = WC68_PDMSG_WAIT_T;
			if (pps_ui < 0)
				pps_ui = WC68_PDMSG_RETRY_T;
		} else {
			dev_dbg(wc68->dev, "%s: request_pdo failed ret=%d\n",
				 __func__, ret);
			pps_ui = WC68_PDMSG_RETRY_T;
		}

	} else {
		ret = pps_keep_alive(pps_data, tcpm_psy);
		if (ret == 0)
			pps_ui = PD_T_PPS_TIMEOUT;

		dev_dbg(wc68->dev, "%s: keep alive ret=%d\n", __func__, ret);
	}

	if (((wc68->charging_state == DC_STATE_NO_CHARGING) &&
		(msg_type == PD_MSG_REQUEST_APDO)) ||
		(wc68->mains_online == false)) {

		/*
		 *  Vbus reset might occour even when PD comms is successful.
		 * Check again.
		 */
		pps_ui = -EINVAL;
	}

	/* PPS_Work: will reschedule */
	dev_dbg(wc68->dev, "%s: pps_ui = %d\n", __func__, pps_ui);
	if (pps_ui > 0)
		mod_delayed_work(system_wq, &wc68->pps_work,
				 msecs_to_jiffies(pps_ui));

	return pps_ui;
}

/*
 * Get the max current/voltage/power of APDO from the CC/PD driver.
 *
 * Initialize &wc68->ta_max_vol, &wc68->ta_max_cur, &wc68->ta_max_pwr
 * initialize wc68->pps_data and &wc68->ta_objpos also
 *
 * call holding mutex_unlock(&wc68->lock);
 */
int wc68_get_apdo_max_power(struct wc68_charger *wc68,
			       unsigned int ta_max_vol,
			       unsigned int ta_max_cur)
{
	int ret;

	/* limits */
	wc68->ta_objpos = 0; /* if !=0 will return the ca */
	wc68->ta_max_vol = ta_max_vol;
	wc68->ta_max_cur = ta_max_cur;

	/* check the phandle */
	ret = wc68_usbpd_setup(wc68);
	if (ret != 0) {
		dev_err(wc68->dev, "cannot find TCPM %d\n", ret);
		wc68->pd = NULL;
		return ret;
	}

	/* technically already in pda_data since check online does it */
	ret = pps_get_src_cap(&wc68->pps_data, wc68->pd);
	if (ret < 0)
		return ret;

	ret = pps_get_apdo_max_power(&wc68->pps_data, &wc68->ta_objpos,
				     &wc68->ta_max_vol, &wc68->ta_max_cur,
				     &wc68->ta_max_pwr);
	if (ret < 0) {
		dev_err(wc68->dev, "cannot determine the apdo max power ret = %d\n", ret);
		return ret;
	}

	dev_dbg(wc68->dev, "%s: APDO pos=%u max_v=%u max_c=%u max_pwr=%lu\n", __func__,
		 wc68->ta_objpos, wc68->ta_max_vol, wc68->ta_max_cur,
		 wc68->ta_max_pwr);

	wc68_chg_stats_set_apdo(&wc68->chg_data,
				 wc68->pps_data.src_caps[wc68->ta_objpos - 1]);

	return 0;
}

/* WLC_DC ---------------------------------------------------------------- */
/* call holding mutex_unlock(&wc68->lock); */
struct power_supply *wc68_get_rx_psy(struct wc68_charger *wc68)
{
	if (!wc68->wlc_psy) {
		const char *wlc_psy_name = wc68->wlc_psy_name ?  : "wireless";

		wc68->wlc_psy = power_supply_get_by_name(wlc_psy_name);
		if (!wc68->wlc_psy) {
			dev_err(wc68->dev, "%s Cannot find %s power supply\n",
				__func__, wlc_psy_name);
		}
	}

	return wc68->wlc_psy;
}

/* call holding mutex_unlock(&wc68->lock); */
int wc68_send_rx_voltage(struct wc68_charger *wc68,
				   unsigned int msg_type)
{
	union power_supply_propval pro_val;
	struct power_supply *wlc_psy;
	int ret = -EINVAL;

	/* Vbus reset happened in the previous PD communication */
	if (wc68->mains_online == false)
		goto out;

	wlc_psy = wc68_get_rx_psy(wc68);
	if (!wlc_psy) {
		ret = -ENODEV;
		goto out;
	}

	/* turn off PPS/PROG, revert to PD */
	if (msg_type == MSG_REQUEST_FIXED_PDO) {
		union power_supply_propval online;
		int ret;

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE, &online);
		if (ret == 0 && online.intval == PPS_PSY_PROG_ONLINE) {
			/*
			 * Make sure the pca is in stby mode before setting
			 * online back to PPS_PSY_FIXED_ONLINE.
			 */
			if (!wc68_check_standby(wc68)) {
				dev_err(wc68->dev,
					"Device not in stby ret=(%d)\n",
					ret);

				return -EINVAL;
			}

			pro_val.intval = PPS_PSY_FIXED_ONLINE;
			ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_ONLINE,
							&pro_val);
			dev_dbg(wc68->dev, "%s: online=%d->%d ret=%d\n", __func__,
				 online.intval, pro_val.intval, ret);
		} else if (ret < 0) {
			dev_err(wc68->dev, "%s: online=%d ret=%d\n", __func__,
			       online.intval, ret);
		}

		/* TODO: reset state? */

		/* done if don't have alternate voltage */
		if (!wc68->ta_vol)
			return ret;
	}

	pro_val.intval = wc68->ta_vol;
	ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&pro_val);
	if (ret < 0)
		dev_err(wc68->dev, "Cannot set RX voltage to %d (%d)\n",
			pro_val.intval, ret);

	/* Vbus reset might happen, check the charging state again */
	if (wc68->mains_online == false) {
		dev_warn(wc68->dev, "%s: mains offline\n", __func__);
		ret = -EINVAL;
	}

	logbuffer_prlog(wc68, LOGLEVEL_DEBUG, "WLCDC: online=%d ta_vol=%d (%d)",
			wc68->mains_online, wc68->ta_vol, ret);

out:
	return ret;
}

/*
 * Get the max current/voltage/power of RXIC from the WCRX driver
 * Initialize &wc68->ta_max_vol, &wc68->ta_max_cur, &wc68->ta_max_pwr
 * call holding mutex_unlock(&wc68->lock);
 */
int wc68_get_rx_max_power(struct wc68_charger *wc68)
{
	union power_supply_propval pro_val;
	struct power_supply *wlc_psy;
	int ret = 0;

	wlc_psy = wc68_get_rx_psy(wc68);
	if (!wlc_psy) {
		dev_err(wc68->dev, "Cannot find wireless power supply\n");
		return -ENODEV;
	}

	/* Get the maximum voltage */
	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX,
					&pro_val);
	if (ret < 0) {
		dev_err(wc68->dev, "%s Cannot get the maximum RX voltage (%d)\n",
			__func__, ret);
		return ret;
	}

	/* RX IC cannot support the request maximum voltage */
	if (wc68->ta_max_vol > pro_val.intval) {
		dev_err(wc68->dev, "%s max %d cannot support ta_max %d voltage\n",
			__func__, pro_val.intval, wc68->ta_max_vol);
		return -EINVAL;
	}

	wc68->ta_max_vol = pro_val.intval;

	/* Get the maximum current */
	ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_CURRENT_MAX,
					&pro_val);
	if (ret < 0) {
		dev_err(wc68->dev, "%s Cannot get the maximum RX current (%d)\n",
			__func__, ret);
		return ret;
	}

	wc68->ta_max_cur = pro_val.intval;
	wc68->ta_max_pwr = (wc68->ta_max_vol / 1000) *
			      (wc68->ta_max_cur / 1000);

	logbuffer_prlog(wc68, LOGLEVEL_INFO, "WLCDC: max_cur=%d max_pwr=%ld",
			wc68->ta_max_cur, wc68->ta_max_pwr);
	return 0;
}

/* called from start_direct_charging(), negative will abort */
int wc68_set_ta_type(struct wc68_charger *wc68, int pps_index)
{
	if (pps_index == PPS_INDEX_TCPM) {
		int ret;

		ret = wc68_usbpd_setup(wc68);
		if (ret != 0) {
			dev_err(wc68->dev, "Cannot find the TA %d\n", ret);
			return ret;
		}

		wc68->ta_type = TA_TYPE_USBPD;
		wc68->chg_mode = CHG_2TO1_DC_MODE;
	} else if (pps_index == PPS_INDEX_WLC) {
		struct power_supply *wlc_psy;

		wlc_psy = wc68_get_rx_psy(wc68);
		if (!wlc_psy) {
			dev_err(wc68->dev, "Cannot find wireless power supply\n");
			return -ENODEV;
		}

		wc68->ta_type = TA_TYPE_WIRELESS;
		wc68->chg_mode = CHG_4TO1_DC_MODE;
	} else {
		wc68->ta_type = TA_TYPE_UNKNOWN;
		wc68->chg_mode = 0;
		return -EINVAL;
	}

	return 0;
}


/* GBMS integration ------------------------------------------------------ */

int wc68_get_charge_type(struct wc68_charger *wc68)
{
	if (!wc68->mains_online)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;


	/* Use SW state for now */
	switch (wc68->charging_state) {
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
	case DC_STATE_ADJUST_TAVOL:
	case DC_STATE_ADJUST_TACUR:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;
	case DC_STATE_CHECK_ACTIVE: /* in preset */
	case DC_STATE_CHARGING_DONE:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

#define WC68_NOT_CHARGING \
	(WC68_BIT_SHUTDOWN_STATE_STS | WC68_BIT_STANDBY_STATE_STS)
#define WC68_ANY_CHARGING_LOOP \
	(WC68_BIT_CHG_LOOP_STS | WC68_BIT_IIN_LOOP_STS | \
	WC68_BIT_VFLT_LOOP_STS)

int wc68_get_status(struct wc68_charger *wc68)
{
	if (wc68_check_standby(wc68)) {
		const bool online = wc68->mains_online;

		/* no disconnect during charger transition */
		return online ? POWER_SUPPLY_STATUS_NOT_CHARGING :
		       POWER_SUPPLY_STATUS_DISCHARGING;
	}

	/* Use SW state (for now) */
	switch (wc68->charging_state) {
	case DC_STATE_NO_CHARGING:
	case DC_STATE_CHECK_VBAT:
	case DC_STATE_PRESET_DC:
	case DC_STATE_CHECK_ACTIVE:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		return POWER_SUPPLY_STATUS_CHARGING;
	/* cpm will need to stop it */
	case DC_STATE_CHARGING_DONE:
		return POWER_SUPPLY_STATUS_CHARGING;
	default:
		break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

#define WC68_PRESENT_MASK \
	(WC68_BIT_ACTIVE_STATE_STS | WC68_BIT_STANDBY_STATE_STS)

int wc68_is_present(struct wc68_charger *wc68)
{
	return wc68_hw_ping(wc68);
}

int wc68_get_chg_chgr_state(struct wc68_charger *wc68,
				      union gbms_charger_state *chg_state)
{
	int vchrg;

	chg_state->v = 0;
	chg_state->f.chg_status = wc68_get_status(wc68);
	chg_state->f.chg_type = wc68_get_charge_type(wc68);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);
	chg_state->f.flags |= GBMS_CS_FLAG_DIRECT_CHG;

	vchrg = wc68_read_adc(wc68, ADCCH_VBAT);
	if (vchrg > 0)
		chg_state->f.vchrg = vchrg / 1000;

	if (chg_state->f.chg_status != POWER_SUPPLY_STATUS_DISCHARGING) {
		int rc;

		rc = wc68_input_current_limit(wc68);
		if (rc > 0)
			chg_state->f.icl = rc / 1000;
	}

	return 0;
}

/* ------------------------------------------------------------------------ */

/* call holding (&wc68->lock); */
void wc68_chg_stats_init(struct wc68_chg_stats *chg_data)
{
	memset(chg_data, 0, sizeof(*chg_data));
	chg_data->adapter_capabilities[0] |= WC68_CHGS_VER;
}

static void wc68_chg_stats_set_apdo(struct wc68_chg_stats *chg_data, u32 apdo)
{
	chg_data->adapter_capabilities[1] = apdo;
}

/* call holding (&wc68->lock); */
int wc68_chg_stats_update(struct wc68_chg_stats *chg_data,
			   const struct wc68_charger *wc68)
{
	switch (wc68->charging_state) {
	case DC_STATE_NO_CHARGING:
		chg_data->nc_count++;
		break;
	case DC_STATE_CHECK_VBAT:
	case DC_STATE_PRESET_DC:
		chg_data->pre_count++;
		break;
	case DC_STATE_CHECK_ACTIVE:
		chg_data->ca_count++;
		break;
	case DC_STATE_ADJUST_CC:
	case DC_STATE_CC_MODE:
		chg_data->cc_count++;
		break;
	case DC_STATE_START_CV:
	case DC_STATE_CV_MODE:
		chg_data->cv_count++;
		break;
	case DC_STATE_ADJUST_TAVOL:
	case DC_STATE_ADJUST_TACUR:
		chg_data->adj_count++;
		break;
	case DC_STATE_CHARGING_DONE:
		chg_data->receiver_state[0] |= WC68_CHGS_F_DONE;
		break;
	default:
		break;
	}

	return 0;
}

void wc68_chg_stats_dump(const struct wc68_charger *wc68)
{
	const struct wc68_chg_stats *chg_data = &wc68->chg_data;

	logbuffer_prlog(wc68, LOGLEVEL_INFO,
			"N: ovc=%d,ovc_ibatt=%d,ovc_delta=%d rcp=%d,stby=%d",
			chg_data->ovc_count,
			chg_data->ovc_max_ibatt, chg_data->ovc_max_delta,
			chg_data->rcp_count, chg_data->stby_count);
	logbuffer_prlog(wc68, LOGLEVEL_INFO,
			"C: nc=%d,pre=%d,ca=%d,cc=%d,cv=%d,adj=%d\n",
			chg_data->nc_count, chg_data->pre_count,
			chg_data->ca_count, chg_data->cc_count,
			chg_data->cv_count, chg_data->adj_count);
}

int wc68_chg_stats_done(struct wc68_chg_stats *chg_data,
			 const struct wc68_charger *wc68)
{
	/* AC[0] version */
	/* AC[1] is APDO */
	/* RS[0][0:8] flags */
	if (chg_data->stby_count)
		wc68_chg_stats_update_flags(chg_data, WC68_CHGS_F_STBY);
	chg_data->receiver_state[0] = (chg_data->pre_count & 0xff) <<
				      WC68_CHGS_PRE_SHIFT;
	chg_data->receiver_state[0] |= (chg_data->rcp_count & 0xff) <<
				       WC68_CHGS_RCPC_SHIFT;
	chg_data->receiver_state[0] |= (chg_data->nc_count & 0xff) <<
				       WC68_CHGS_NC_SHIFT;
	/* RS[1] counters */
	chg_data->receiver_state[1] = (chg_data->ovc_count & 0xffff) <<
				      WC68_CHGS_OVCC_SHIFT;
	chg_data->receiver_state[1] |= (chg_data->adj_count & 0xffff) <<
				       WC68_CHGS_ADJ_SHIFT;
	/* RS[2] counters */
	chg_data->receiver_state[2] = (chg_data->adj_count & 0xffff) <<
				      WC68_CHGS_ADJ_SHIFT;
	chg_data->receiver_state[2] |= (chg_data->adj_count & 0xffff) <<
				       WC68_CHGS_ADJ_SHIFT;
	/* RS[3] counters */
	chg_data->receiver_state[3] = (chg_data->cc_count & 0xffff) <<
				      WC68_CHGS_CC_SHIFT;
	chg_data->receiver_state[3] |= (chg_data->cv_count & 0xffff) <<
				       WC68_CHGS_CV_SHIFT;
	/* RS[4] counters */
	chg_data->receiver_state[1] = (chg_data->ca_count & 0xff) <<
				      WC68_CHGS_CA_SHIFT;

	chg_data->valid = true;

	return 0;
}
