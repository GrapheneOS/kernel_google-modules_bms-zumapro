/*
 * Fuel gauge driver for Maxim 77779
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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s " fmt, __func__

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include "max77779_fg.h"

#define MAX77779_FG_TPOR_MS 800

#define MAX77779_FG_TICLR_MS 500
#define MAX77779_FG_I2C_DRIVER_NAME "max77779_fg_irq"
#define MAX77779_FG_DELAY_INIT_MS 1000
#define FULLCAPNOM_STABILIZE_CYCLES 5

#define BHI_IMPEDANCE_SOC_LO		50
#define BHI_IMPEDANCE_SOC_HI		55
#define BHI_IMPEDANCE_TEMP_LO		250
#define BHI_IMPEDANCE_TEMP_HI		300
#define BHI_IMPEDANCE_CYCLE_CNT		5
#define BHI_IMPEDANCE_TIMERH		50 /* 7*24 / 3.2hr */

#define MAX77779_FG_FWUPDATE_SOC       95
#define MAX77779_FG_FWUPDATE_SOC_RAW   0x5F00 /* soc 95% */

enum max77779_fg_command_bits {
	MAX77779_FG_COMMAND_HARDWARE_RESET = 0x000F,
};

#define BHI_CAP_FCN_COUNT	3

#define DEFAULT_STATUS_CHARGE_MA	100

/* No longer used in 79, used for taskperiod re-scaling in 59 */
#define MAX77779_LSB 1

#define MAX77779_FG_EVENT_FULLCAPNOM_LOW     BIT(0)
#define MAX77779_FG_EVENT_FULLCAPNOM_HIGH    BIT(1)
#define MAX77779_FG_EVENT_REPSOC_EDET        BIT(2)
#define MAX77779_FG_EVENT_REPSOC_FDET        BIT(3)
#define MAX77779_FG_EVENT_REPSOC             BIT(4)
#define MAX77779_FG_EVENT_VFOCV              BIT(5)

static irqreturn_t max77779_fg_irq_thread_fn(int irq, void *obj);
static int max77779_fg_set_next_update(struct max77779_fg_chip *chip);
static int max77779_fg_update_cycle_count(struct max77779_fg_chip *chip);

static struct mutex section_lock;

static bool max77779_fg_reglog_init(struct max77779_fg_chip *chip)
{
	chip->regmap.reglog = devm_kzalloc(chip->dev, sizeof(*chip->regmap.reglog), GFP_KERNEL);

	return chip->regmap.reglog;
}

/* TODO: b/285191823 - Validate all conversion helper functions */
/* ------------------------------------------------------------------------- */

static inline int reg_to_percentage(u16 val)
{
	/* LSB: 1/256% */
	return val >> 8;
}

static inline int reg_to_twos_comp_int(u16 val)
{
	/* Convert u16 to twos complement  */
	return -(val & 0x8000) + (val & 0x7FFF);
}

static inline int reg_to_micro_amp(s16 val, u16 rsense)
{
	/* LSB: 1.5625μV/RSENSE ; Rsense LSB is 2μΩ */
	return div_s64((s64) val * 156250, rsense);
}

static inline int reg_to_deci_deg_cel(s16 val)
{
	/* LSB: 1/256°C */
	return div_s64((s64) val * 10, 256);
}

static inline int reg_to_cycles(u32 val)
{
	/* LSB: 25% of one cycle */
	return DIV_ROUND_CLOSEST(val * 25, 100);
}

static inline int reg_to_seconds(s16 val)
{
	/* LSB: 5.625 seconds */
	return DIV_ROUND_CLOSEST((int) val * 5625, 1000);
}

static inline int reg_to_vempty(u16 val)
{
	return ((val >> 7) & 0x1FF) * 10;
}

static inline int reg_to_vrecovery(u16 val)
{
	return (val & 0x7F) * 40;
}

static inline int reg_to_capacity_uah(u16 val, struct max77779_fg_chip *chip)
{
	return reg_to_micro_amp_h(val, chip->RSense, MAX77779_LSB);
}

static inline int reg_to_time_hr(u16 val, struct max77779_fg_chip *chip)
{
	return (val * 32) / 10;
}

/* log ----------------------------------------------------------------- */

static int format_battery_history_entry(char *temp, int size, int page_size, u16 *line)
{
	int length = 0, i;

	for (i = 0; i < page_size; i++) {
		length += scnprintf(temp + length,
			size - length, "%04x ",
			line[i]);
	}

	if (length > 0)
		temp[--length] = 0;
	return length;
}

/*
 * Removed the following properties:
 *   POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG
 *   POWER_SUPPLY_PROP_TIME_TO_FULL_AVG
 *   POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
 *   POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
 * Need to keep the number of properies under UEVENT_NUM_ENVP (minus # of
 * standard uevent variables).
 */
static enum power_supply_property max77779_fg_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,		/* replace with _RAW */
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,	/* used from gbattery */
	POWER_SUPPLY_PROP_CURRENT_AVG,		/* candidate for tier switch */
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

/* ------------------------------------------------------------------------- */

static ssize_t offmode_charger_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%hhd\n", chip->offmode_charger);
}

static ssize_t offmode_charger_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);

	if (kstrtobool(buf, &chip->offmode_charger))
		return -EINVAL;

	return count;
}

static DEVICE_ATTR_RW(offmode_charger);

int max77779_fg_usr_lock_section(const struct maxfg_regmap *map, enum max77779_fg_reg_sections section, bool enabled)
{
	int ret, i;
	u16 data;

	mutex_lock(&section_lock);
	ret = REGMAP_READ(map, MAX77779_FG_USR, &data);
	if (ret)
		goto unlock_exit;

	switch (section) {
	case MAX77779_FG_RAM_SECTION: /* addr: 0x36, reg: 0x00 ... 0xDF */
		data = _max77779_fg_usr_vlock_set(data, enabled);
		break;
	case MAX77779_FG_FUNC_SECTION: /* addr: 0x36, reg: 0xE0 ... 0xEE */
		data = _max77779_fg_usr_rlock_set(data, enabled);
		break;
	case MAX77779_FG_NVM_SECTION: /* addr: 0x37 */
		data = _max77779_fg_usr_nlock_set(data, enabled);
		break;
	case MAX77779_FG_ALL_SECTION:
		data = _max77779_fg_usr_vlock_set(data, enabled);
		data = _max77779_fg_usr_rlock_set(data, enabled);
		data = _max77779_fg_usr_nlock_set(data, enabled);
		break;
	default:
		pr_err("Failed to lock section %d\n", section);
		goto unlock_exit;
	}

	/* Requires write twice */
	for (i = 0; i < 2; i++) {
		ret = REGMAP_WRITE(map, MAX77779_FG_USR, data);
		if (ret)
			goto unlock_exit;
	}

unlock_exit:
	mutex_unlock(&section_lock);
	return ret;
}

/* NOTE: it might not be static inline depending on how it's used */
static inline int max77779_fg_usr_lock(const struct maxfg_regmap *map, unsigned int reg, bool enabled) {
	switch (reg) {
	case 0x00 ... 0xDF:
		return max77779_fg_usr_lock_section(map, MAX77779_FG_RAM_SECTION, enabled);
	case 0xE0 ... 0xEE:
		return max77779_fg_usr_lock_section(map, MAX77779_FG_FUNC_SECTION, enabled);
	default:
		pr_err("Failed to translate reg 0x%X to section\n", reg);
		return -EINVAL;
	}
}

int max77779_fg_register_write(const struct maxfg_regmap *map,
			       unsigned int reg, u16 value, bool verify)
{
	int ret, rc;

	ret = max77779_fg_usr_lock(map, reg, false);
	if (ret) {
		pr_err("Failed to unlock ret=%d\n", ret);
		return ret;
	}

	if (verify)
		ret = REGMAP_WRITE_VERIFY(map, reg, value);
	else
		ret = REGMAP_WRITE(map, reg, value);
	if (ret)
		pr_err("Failed to write reg verify=%d ret=%d\n", verify, ret);

	rc = max77779_fg_usr_lock(map, reg, true);
	if (rc)
		pr_err("Failed to lock ret=%d\n", rc);

	return ret;
}

int max77779_fg_nregister_write(const struct maxfg_regmap *map,
				const struct maxfg_regmap *debug_map,
				unsigned int reg, u16 value, bool verify)
{
	int ret, rc;

	ret = max77779_fg_usr_lock_section(map, MAX77779_FG_NVM_SECTION, false);
	if (ret) {
		pr_err("Failed to unlock ret=%d\n", ret);
		return ret;
	}

	if (verify)
		ret = REGMAP_WRITE_VERIFY(debug_map, reg, value);
	else
		ret = REGMAP_WRITE(debug_map, reg, value);
	if (ret)
		pr_err("Failed to write reg verify=%d ret=%d\n", verify, ret);

	rc = max77779_fg_usr_lock_section(map, MAX77779_FG_NVM_SECTION, true);
	if (rc)
		pr_err("Failed to lock ret=%d\n", rc);

	return ret;
}

/*
 * special reg_read for firmware update
 * - it will not change the lock status
 */
int max77779_external_fg_reg_read(struct device *dev, uint16_t reg, uint16_t *val)
{
	struct max77779_fg_chip *chip = dev_get_drvdata(dev);
	unsigned int tmp = 0;
	int ret;

	if (!chip || !chip->regmap.regmap)
		return -EAGAIN;

	tmp = *val;

	ret = regmap_read(chip->regmap.regmap, reg, &tmp);
	if (ret < 0)
		return ret;

	*val = tmp & 0xFFFF;

	return ret;
}
EXPORT_SYMBOL_GPL(max77779_external_fg_reg_read);

/*
 * special reg_write for firmware update
 * - it will not change the lock status
 */
int max77779_external_fg_reg_write(struct device *dev, uint16_t reg, uint16_t val)
{
	struct max77779_fg_chip *chip = dev_get_drvdata(dev);

	if (!chip || !chip->regmap.regmap)
		return -EAGAIN;

	return regmap_write(chip->regmap.regmap, reg, val);
}
EXPORT_SYMBOL_GPL(max77779_external_fg_reg_write);

/*
 * force is true when changing the model via debug props.
 * NOTE: call holding model_lock
 */
static int max77779_fg_model_reload(struct max77779_fg_chip *chip, bool force)
{
	const bool disabled = chip->model_reload == MAX77779_FG_LOAD_MODEL_DISABLED;
	const bool pending = chip->model_reload != MAX77779_FG_LOAD_MODEL_IDLE;
	int version_now, version_load;

	dev_info(chip->dev, "model_reload=%d force=%d pending=%d disabled=%d\n",
		 chip->model_reload, force, pending, disabled);

	if (!force && (pending || disabled))
		return -EEXIST;

	version_now = max77779_model_read_version(chip->model_data);
	version_load = max77779_fg_model_version(chip->model_data);

	if (!force && version_now == version_load)
		return -EEXIST;

	/* REQUEST -> IDLE or set to the number of retries */
	gbms_logbuffer_devlog(chip->monitor_log, chip->dev, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			      "Schedule Load FG Model, ID=%d, ver:%d->%d",
			      chip->batt_id, version_now, version_load);

	chip->model_reload = MAX77779_FG_LOAD_MODEL_REQUEST;
	chip->model_ok = false;
	mod_delayed_work(system_wq, &chip->model_work, 0);

	return 0;
}

/* ----------------------------------------------------------------------- */

static ssize_t model_state_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);
	ssize_t len = 0;

	if (!chip->model_data)
		return -EINVAL;

	mutex_lock(&chip->model_lock);
	len += scnprintf(&buf[len], PAGE_SIZE, "ModelNextUpdate: %d\n",
			 chip->model_next_update);
	len += max77779_model_state_cstr(&buf[len], PAGE_SIZE - len,
				       chip->model_data);
	mutex_unlock(&chip->model_lock);

	return len;
}

static DEVICE_ATTR_RO(model_state);

static ssize_t gmsr_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);
	ssize_t len = 0;

	mutex_lock(&chip->model_lock);
	len = max77779_gmsr_state_cstr(&buff[len], PAGE_SIZE);
	mutex_unlock(&chip->model_lock);

	return len;
}

static DEVICE_ATTR_RO(gmsr);

/* Was POWER_SUPPLY_PROP_RESISTANCE_ID */
static ssize_t resistance_id_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buff, PAGE_SIZE, "%d\n", chip->batt_id);
}

static DEVICE_ATTR_RO(resistance_id);

/* Was POWER_SUPPLY_PROP_RESISTANCE */
static ssize_t resistance_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buff, PAGE_SIZE, "%d\n",
			 maxfg_read_resistance(&chip->regmap, chip->RSense));
}

static DEVICE_ATTR_RO(resistance);

/* lsb 1/256, race with max77779_fg_model_work()  */
static int max77779_fg_get_capacity_raw(struct max77779_fg_chip *chip, u16 *data)
{
	if (chip->fw_update_mode) {
		*data = MAX77779_FG_FWUPDATE_SOC_RAW;
		return 0;
	}

	return REGMAP_READ(&chip->regmap, chip->reg_prop_capacity_raw, data);
}

static int max77779_fg_get_battery_soc(struct max77779_fg_chip *chip)
{
	u16 data;
	int capacity, err;

	if (chip->fake_capacity >= 0 && chip->fake_capacity <= 100)
		return chip->fake_capacity;

	if (chip->fw_update_mode)
		return MAX77779_FG_FWUPDATE_SOC;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_RepSOC, &data);
	if (err)
		return err;

	capacity = reg_to_percentage(data);

	if (capacity == 100 && chip->offmode_charger)
		chip->fake_capacity = 100;

	return capacity;
}

static int max77779_fg_get_battery_vfsoc(struct max77779_fg_chip *chip)
{
	u16 data;
	int capacity, err;

	if (chip->fw_update_mode)
		return MAX77779_FG_FWUPDATE_SOC;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_VFSOC, &data);
	if (err)
		return err;
	capacity = reg_to_percentage(data);

	return capacity;
}

static void max77779_fg_prime_battery_qh_capacity(struct max77779_fg_chip *chip)
{
	u16  mcap = 0, data = 0;

	(void)REGMAP_READ(&chip->regmap, MAX77779_FG_MixCap, &mcap);
	chip->current_capacity = mcap;

	(void)REGMAP_READ(&chip->regmap, MAX77779_FG_QH, &data);
	chip->previous_qh = reg_to_twos_comp_int(data);
}

/* NOTE: the gauge doesn't know if we are current limited to */
static int max77779_fg_get_battery_status(struct max77779_fg_chip *chip)
{
	u16 data = 0;
	int current_now, current_avg, ichgterm, vfsoc, soc, fullsocthr;
	int status = POWER_SUPPLY_STATUS_UNKNOWN, err;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_Current, &data);
	if (err)
		return -EIO;
	current_now = -reg_to_micro_amp(data, chip->RSense);

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_AvgCurrent, &data);
	if (err)
		return -EIO;
	current_avg = -reg_to_micro_amp(data, chip->RSense);

	if (chip->status_charge_threshold_ma) {
		ichgterm = chip->status_charge_threshold_ma * 1000;
	} else {
		err = REGMAP_READ(&chip->regmap, MAX77779_FG_IChgTerm, &data);
		if (err)
			return -EIO;
		ichgterm = reg_to_micro_amp(data, chip->RSense);
	}

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_FullSocThr, &data);
	if (err)
		return -EIO;
	fullsocthr = reg_to_percentage(data);

	soc = max77779_fg_get_battery_soc(chip);
	if (soc < 0)
		return -EIO;

	vfsoc = max77779_fg_get_battery_vfsoc(chip);
	if (vfsoc < 0)
		return -EIO;

	if (current_avg > -ichgterm && current_avg <= 0) {

		if (soc >= fullsocthr) {
			const bool needs_prime = (chip->prev_charge_status ==
						  POWER_SUPPLY_STATUS_CHARGING);

			status = POWER_SUPPLY_STATUS_FULL;
			if (needs_prime)
				max77779_fg_prime_battery_qh_capacity(chip);
		} else {
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

	} else if (current_now >= -ichgterm)  {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_CHARGING;
		if (chip->prev_charge_status == POWER_SUPPLY_STATUS_DISCHARGING
		    && current_avg  < -ichgterm)
			max77779_fg_prime_battery_qh_capacity(chip);
	}

	if (status != chip->prev_charge_status)
		dev_dbg(chip->dev,
			"s=%d->%d c=%d avg_c=%d ichgt=%d vfsoc=%d soc=%d fullsocthr=%d\n",
			chip->prev_charge_status, status, current_now, current_avg,
			ichgterm, vfsoc, soc, fullsocthr);

	chip->prev_charge_status = status;

	return status;
}

static int max77779_fg_update_battery_qh_based_capacity(struct max77779_fg_chip *chip)
{
	u16 data;
	int current_qh, err = 0;

	if (chip->por)
		return -EINVAL;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_QH, &data);
	if (err)
		return err;

	current_qh = reg_to_twos_comp_int(data);

	/* QH value accumulates as battery charges */
	chip->current_capacity -= (chip->previous_qh - current_qh);
	chip->previous_qh = current_qh;

	return 0;
}

static void max77779_fg_restore_battery_cycle(struct max77779_fg_chip *chip)
{
	int ret = 0;
	u16 reg_cycle;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_Cycles, &reg_cycle);
	if (ret < 0) {
		dev_info(chip->dev, "Fail to read reg %#x (%d)",
				MAX77779_FG_Cycles, ret);
		return;
	}

	ret = gbms_storage_read(GBMS_TAG_CNHS, &chip->eeprom_cycle,
				sizeof(chip->eeprom_cycle));
	if (ret < 0) {
		dev_info(chip->dev, "Fail to read eeprom cycle count (%d)", ret);
		return;
	}

	if (chip->eeprom_cycle == 0xFFFF) { /* empty storage */
		ret = gbms_storage_write(GBMS_TAG_CNHS, &reg_cycle, sizeof(reg_cycle));
		if (ret < 0)
			dev_info(chip->dev, "Fail to write eeprom cycle (%d)", ret);
		else
			chip->eeprom_cycle = reg_cycle;
		return;
	}

	dev_info(chip->dev, "reg_cycle:%d, eeprom_cycle:%d, update:%c",
		 reg_cycle, chip->eeprom_cycle, chip->eeprom_cycle > reg_cycle ? 'Y' : 'N');
	if (chip->eeprom_cycle > reg_cycle) {
		ret = MAX77779_FG_REGMAP_WRITE_VERIFY(&chip->regmap, MAX77779_FG_Cycles,
						      chip->eeprom_cycle);
		if (ret < 0)
			dev_warn(chip->dev, "fail to update cycles (%d)", ret);
		else
			max77779_fg_update_cycle_count(chip);
	}
}

static u16 max77779_fg_save_battery_cycle(const struct max77779_fg_chip *chip, u16 reg_cycle)
{
	int ret = 0;
	u16 eeprom_cycle = chip->eeprom_cycle;

	if (chip->por || reg_cycle == 0)
		return eeprom_cycle;

	if (reg_cycle <= eeprom_cycle)
		return eeprom_cycle;

	ret = gbms_storage_write(GBMS_TAG_CNHS, &reg_cycle, sizeof(reg_cycle));
	if (ret < 0) {
		dev_info(chip->dev, "Fail to write %d eeprom cycle count (%d)", reg_cycle, ret);
	} else {
		dev_info(chip->dev, "update saved cycle:%d -> %d\n", eeprom_cycle, reg_cycle);
		eeprom_cycle = reg_cycle;
	}

	return eeprom_cycle;
}

#define MAX17201_HIST_CYCLE_COUNT_OFFSET	0x4
#define MAX17201_HIST_TIME_OFFSET		0xf

static int max77779_fg_get_cycle_count(struct max77779_fg_chip *chip)
{
	return chip->cycle_count;
}

static int max77779_fg_update_cycle_count(struct max77779_fg_chip *chip)
{
	int err;
	u16 reg_cycle;

	/*
	 * Corner case: battery under 3V hit POR without irq.
	 * cycles reset in this situation, incorrect data
	 */
	if (chip->por)
		return -ECANCELED;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_Cycles, &reg_cycle);
	if (err < 0)
		return err;

	chip->cycle_count = reg_to_cycles((u32)reg_cycle);

	chip->eeprom_cycle = max77779_fg_save_battery_cycle(chip, reg_cycle);

	if (chip->model_ok && reg_cycle >= chip->model_next_update) {
		err = max77779_fg_set_next_update(chip);
		if (err < 0)
			dev_err(chip->dev, "%s cannot set next update (%d)\n",
				 __func__, err);
	}

	return chip->cycle_count;
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
		err = MAX77779_FG_REGMAP_WRITE(map, bcea->map[reg], data);
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
		err = MAX77779_FG_REGMAP_WRITE(map, bcea->map[reg], val);
		break;
	default:
		break;
	}

	return err;
}

static int batt_ce_full_estimate(struct gbatt_capacity_estimation *ce)
{
	return (ce->cap_filter_count > 0) && (ce->delta_vfsoc_sum > 0) ?
		ce->delta_cc_sum / ce->delta_vfsoc_sum : -1;
}

/* Measure the deltaCC, deltaVFSOC and CapacityFiltered */
static void batt_ce_capacityfiltered_work(struct work_struct *work)
{
	struct max77779_fg_chip *chip = container_of(work, struct max77779_fg_chip,
					    cap_estimate.settle_timer.work);
	struct gbatt_capacity_estimation *cap_esti = &chip->cap_estimate;
	int settle_cc = 0, settle_vfsoc = 0;
	int delta_cc = 0, delta_vfsoc = 0;
	int cc_sum = 0, vfsoc_sum = 0;
	bool valid_estimate = false;
	int rc = 0;
	int data;

	mutex_lock(&cap_esti->batt_ce_lock);

	/* race with disconnect */
	if (!cap_esti->cable_in ||
	    cap_esti->estimate_state != ESTIMATE_PENDING) {
		goto exit;
	}

	rc = max77779_fg_update_battery_qh_based_capacity(chip);
	if (rc < 0)
		goto ioerr;

	settle_cc = reg_to_micro_amp_h(chip->current_capacity, chip->RSense, MAX77779_LSB);

	data = max77779_fg_get_battery_vfsoc(chip);
	if (data < 0)
		goto ioerr;

	settle_vfsoc = data;
	settle_cc = settle_cc / 1000;
	delta_cc = settle_cc - cap_esti->start_cc;
	delta_vfsoc = settle_vfsoc - cap_esti->start_vfsoc;

	if ((delta_cc > 0) && (delta_vfsoc > 0)) {

		cc_sum = delta_cc + cap_esti->delta_cc_sum;
		vfsoc_sum = delta_vfsoc + cap_esti->delta_vfsoc_sum;

		if (cap_esti->cap_filter_count >= cap_esti->cap_filt_length) {
			const int filter_divisor = cap_esti->cap_filt_length;

			cc_sum -= cap_esti->delta_cc_sum/filter_divisor;
			vfsoc_sum -= cap_esti->delta_vfsoc_sum/filter_divisor;
		}

		cap_esti->cap_filter_count++;
		cap_esti->delta_cc_sum = cc_sum;
		cap_esti->delta_vfsoc_sum = vfsoc_sum;

		valid_estimate = true;
	}

ioerr:
	batt_ce_stop_estimation(cap_esti, ESTIMATE_DONE);

exit:
	logbuffer_log(chip->ce_log,
		      "valid=%d settle[cc=%d, vfsoc=%d], delta[cc=%d,vfsoc=%d] ce[%d]=%d",
		      valid_estimate, settle_cc, settle_vfsoc, delta_cc, delta_vfsoc,
		      cap_esti->cap_filter_count, batt_ce_full_estimate(cap_esti));

	mutex_unlock(&cap_esti->batt_ce_lock);

	/* force to update uevent to framework side. */
	if (valid_estimate)
		power_supply_changed(chip->psy);
}

/*
 * batt_ce_init(): estimate_state = ESTIMATE_NONE
 * batt_ce_start(): estimate_state = ESTIMATE_NONE -> ESTIMATE_PENDING
 * batt_ce_capacityfiltered_work(): ESTIMATE_PENDING->ESTIMATE_DONE
 */
static int batt_ce_start(struct gbatt_capacity_estimation *cap_esti,
			 int cap_tsettle_ms)
{
	mutex_lock(&cap_esti->batt_ce_lock);

	/* Still has cable and estimate is not pending or cancelled */
	if (!cap_esti->cable_in || cap_esti->estimate_state != ESTIMATE_NONE)
		goto done;

	pr_info("EOC: Start the settle timer\n");
	cap_esti->estimate_state = ESTIMATE_PENDING;
	schedule_delayed_work(&cap_esti->settle_timer,
		msecs_to_jiffies(cap_tsettle_ms));

done:
	mutex_unlock(&cap_esti->batt_ce_lock);
	return 0;
}

static int batt_ce_init(struct gbatt_capacity_estimation *cap_esti,
			struct max77779_fg_chip *chip)
{
	int rc, vfsoc;

	rc = max77779_fg_update_battery_qh_based_capacity(chip);
	if (rc < 0)
		return -EIO;

	vfsoc = max77779_fg_get_battery_vfsoc(chip);
	if (vfsoc < 0)
		return -EIO;

	cap_esti->start_vfsoc = vfsoc;
	cap_esti->start_cc = reg_to_micro_amp_h(chip->current_capacity,
						chip->RSense, MAX77779_LSB) / 1000;
	/* Capacity Estimation starts only when the state is NONE */
	cap_esti->estimate_state = ESTIMATE_NONE;
	return 0;
}

static int max77779_fg_resume_check(struct max77779_fg_chip *chip)
{
	int ret = 0;

	pm_runtime_get_sync(chip->dev);
	if (!chip->init_complete || !chip->resume_complete)
		ret = -EAGAIN;
	pm_runtime_put_sync(chip->dev);

	return ret;
}

/* call holding chip->model_lock */
static int max77779_fg_check_impedance(struct max77779_fg_chip *chip, u16 *th)
{
	struct maxfg_regmap *map = &chip->regmap;
	int soc, temp, cycle_count, ret;
	u16 data, timerh;

	if (!chip->model_state_valid)
		return -EAGAIN;

	soc = max77779_fg_get_battery_soc(chip);
	if (soc < BHI_IMPEDANCE_SOC_LO || soc > BHI_IMPEDANCE_SOC_HI)
		return -EAGAIN;

	ret = REGMAP_READ(map, MAX77779_FG_Temp, &data);
	if (ret < 0)
		return -EIO;

	temp = reg_to_deci_deg_cel(data);
	if (temp < BHI_IMPEDANCE_TEMP_LO || temp > BHI_IMPEDANCE_TEMP_HI)
		return -EAGAIN;

	cycle_count = max77779_fg_get_cycle_count(chip);
	if (cycle_count < 0)
		return -EINVAL;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_TimerH, &timerh);
	if (ret < 0 || timerh == 0)
		return -EINVAL;

	/* wait for a few cyles and time in field before validating the value */
	if (cycle_count < BHI_IMPEDANCE_CYCLE_CNT || timerh < BHI_IMPEDANCE_TIMERH)
		return -ENODATA;

	*th = timerh;
	return 0;
}

/* will return negative if the value is not qualified */
static int max77779_fg_health_read_impedance(struct max77779_fg_chip *chip)
{
	u16 timerh;
	int ret;

	ret = max77779_fg_check_impedance(chip, &timerh);
	if (ret < 0)
		return -EINVAL;

	return maxfg_read_resistance(&chip->regmap, chip->RSense);
}

/* in hours */
static int max77779_fg_get_age(struct max77779_fg_chip *chip)
{
	u16 timerh;
	int ret;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_TimerH, &timerh);
	if (ret < 0 || timerh == 0)
		return -ENODATA;

	return reg_to_time_hr(timerh, chip);
}

static int max77779_fg_find_pmic(struct max77779_fg_chip *chip)
{
	if (chip->pmic_dev)
		return 0;

	chip->pmic_dev = max77779_get_dev(chip->dev, MAX77779_PMIC_OF_NAME);

	return chip->pmic_dev == NULL ? -ENXIO : 0;
}

static int max77779_fg_get_fw_ver(struct max77779_fg_chip *chip)
{
	uint8_t fw_rev, fw_sub_rev;
	int ret;

	ret = max77779_fg_find_pmic(chip);
	if (ret) {
		dev_err(chip->dev, "Error finding pmic\n");
		return ret;
	}

	ret = max77779_external_pmic_reg_read(chip->pmic_dev, MAX77779_PMIC_RISCV_FW_REV, &fw_rev);
	if (ret < 0)
		return ret;

	ret = max77779_external_pmic_reg_read(chip->pmic_dev, MAX77779_PMIC_RISCV_FW_SUB_REV, &fw_sub_rev);
	if (ret < 0)
		return ret;

	chip->fw_rev = fw_rev;
	chip->fw_sub_rev = fw_sub_rev;

	gbms_logbuffer_devlog(chip->monitor_log, chip->dev, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			      "FW_REV=%d, FW_SUB_REV=%d", chip->fw_rev, chip->fw_sub_rev);

	return 0;
}

/* Report fake temp 22 degree if firmware < 1.15 */
#define MAX77779_FG_FAKE_TEMP_FW_REV     1
#define MAX77779_FG_FAKE_TEMP_FW_SUBREV  15
#define MAX77779_FG_FAKE_TEMP            220
static int max77779_fg_get_temp(struct max77779_fg_chip *chip)
{
	u16 data = 0;
	int err = 0;

	if (!chip->fw_rev && !chip->fw_sub_rev)
		max77779_fg_get_fw_ver(chip);

	if (chip->fw_rev == MAX77779_FG_FAKE_TEMP_FW_REV &&
	    chip->fw_sub_rev < MAX77779_FG_FAKE_TEMP_FW_SUBREV)
		return MAX77779_FG_FAKE_TEMP;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_Temp, &data);
	if (err < 0)
		return MAX77779_FG_FAKE_TEMP;

	return reg_to_deci_deg_cel(data);
}

static int max77779_adjust_cgain(struct max77779_fg_chip *chip, unsigned int otp_revision)
{
	u16 i_gtrim, i_otrim, ro_cgain, v_cgain;
	int i_otrim_real;
	int err;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_TrimIbattGain, &i_gtrim);
	if (err < 0)
		return err;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_TrimBattOffset, &i_otrim);
	if (err < 0)
		return err;

	/* i_gtrim_real = ((-1) * (i_gtrim & 0x0800)) | (i_gtrim & 0x07FF); */
	i_otrim_real = ((-1) * (i_otrim & 0x0080)) | (i_otrim & 0x007F);

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_CGain, &ro_cgain);
	if (err < 0)
		return err;

	v_cgain = ro_cgain & 0xFFC0;
	if (i_otrim_real > 32)
		v_cgain = v_cgain | 0x20; /* -32 & 0x3F */
	else if (i_otrim_real < -31)
		v_cgain = v_cgain | 0x1F; /* 31 & 0x3F */
	else
		v_cgain = v_cgain | (((-1) * i_otrim_real) & 0x3F);

	gbms_logbuffer_devlog(chip->monitor_log, chip->dev, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			      "OTP_VER:%d,%02X:%04X,%02X:%04X,%02X:%04X,trim:%d,new Cgain:%04X",
			      otp_revision, MAX77779_FG_TrimIbattGain, i_gtrim,
			      MAX77779_FG_TrimBattOffset, i_otrim, MAX77779_FG_CGain, ro_cgain,
			      i_otrim_real, v_cgain);

	if (v_cgain == ro_cgain)
		return 0;

	err = MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_CGain, v_cgain);
	if (err < 0)
		return err;

	return 0;
}

#define CHECK_CURRENT_OFFSET_OTP_REVISION	2
static void max77779_current_offset_check(struct max77779_fg_chip *chip)
{
	uint8_t otp_revision;
	int ret;

	if (chip->current_offset_check_done)
		return;

	ret = max77779_fg_find_pmic(chip);
	if (ret) {
		dev_err(chip->dev, "Error finding pmic\n");
		return;
	}

	ret = max77779_external_pmic_reg_read(chip->pmic_dev, MAX77779_PMIC_OTP_REVISION,
					      &otp_revision);
	if (ret < 0) {
		dev_err(chip->dev, "failed to read PMIC_OTP_REVISION\n");
		return;
	}

	if (otp_revision > CHECK_CURRENT_OFFSET_OTP_REVISION)
		goto done;

	ret = max77779_adjust_cgain(chip, otp_revision);
	if (ret < 0)
		return;
done:
	chip->current_offset_check_done = true;
}

static int max77779_fg_monitor_log_data(struct max77779_fg_chip *chip, bool force_log)
{
	int ret, charge_counter = -1;
	u16 repsoc, data;
	char buf[256] = { 0 };

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_RepSOC, &data);
	if (ret < 0)
		return ret;

	repsoc = (data >> 8) & 0x00FF;
	if (repsoc == chip->pre_repsoc && !force_log)
		return ret;

	ret = maxfg_reg_log_data(&chip->regmap, &chip->regmap_debug, buf);
	if (ret < 0)
		return ret;

	ret = max77779_fg_update_battery_qh_based_capacity(chip);
	if (ret == 0)
		charge_counter = reg_to_capacity_uah(chip->current_capacity, chip);

	gbms_logbuffer_devlog(chip->monitor_log, chip->dev, LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
			      "%02X:%04X %s CC:%d", MAX77779_FG_RepSOC, data, buf, charge_counter);

	chip->pre_repsoc = repsoc;

	return ret;
}

static int max77779_fg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)
					power_supply_get_drvdata(psy);
	struct maxfg_regmap *map = &chip->regmap;
	int rc, err = 0;
	u16 data = 0;
	int idata;

	mutex_lock(&chip->model_lock);

	if (max77779_fg_resume_check(chip)) {
		mutex_unlock(&chip->model_lock);
		return -EAGAIN;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		err = max77779_fg_get_battery_status(chip);
		if (err < 0)
			break;

		/*
		 * Capacity estimation must run only once.
		 * NOTE: this is a getter with a side effect
		 */
		val->intval = err;
		if (err == POWER_SUPPLY_STATUS_FULL)
			batt_ce_start(&chip->cap_estimate,
				      chip->cap_estimate.cap_tsettle);
		/* return data ok */
		err = 0;
		break;
	case GBMS_PROP_CAPACITY_RAW:
		err = max77779_fg_get_capacity_raw(chip, &data);
		if (err == 0)
			val->intval = (int)data;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		idata = max77779_fg_get_battery_soc(chip);
		if (idata < 0) {
			err = idata;
			break;
		}

		val->intval = idata;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		err = max77779_fg_update_battery_qh_based_capacity(chip);
		if (err < 0)
			break;

		val->intval = reg_to_capacity_uah(chip->current_capacity, chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/*
		 * Snap charge_full to DESIGNCAP during early charge cycles to
		 * prevent large fluctuations in FULLCAPNOM. MAX77779_FG_Cycles LSB
		 * is 25%
		 */
		err = max77779_fg_get_cycle_count(chip);
		if (err < 0)
			break;

		/* err is cycle_count */
		if (err <= FULLCAPNOM_STABILIZE_CYCLES)
			err = REGMAP_READ(map, MAX77779_FG_DesignCap, &data);
		else
			err = REGMAP_READ(map, MAX77779_FG_FullCapNom, &data);

		if (err == 0)
			val->intval = reg_to_capacity_uah(data, chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		err = REGMAP_READ(map, MAX77779_FG_DesignCap, &data);
		if (err == 0)
			val->intval = reg_to_capacity_uah(data, chip);
		break;
	/* current is positive value when flowing to device */
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		err = REGMAP_READ(map, MAX77779_FG_AvgCurrent, &data);
		if (err == 0)
			val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	/* current is positive value when flowing to device */
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		err = REGMAP_READ(map, MAX77779_FG_Current, &data);
		if (err == 0)
			val->intval = -reg_to_micro_amp(data, chip->RSense);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		err = max77779_fg_get_cycle_count(chip);
		if (err < 0)
			break;
		/* err is cycle_count */
		val->intval = err;
		/* return data ok */
		err = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:

		if (chip->fake_battery != -1) {
			val->intval = chip->fake_battery;
		} else {

			err = REGMAP_READ(map, MAX77779_FG_FG_INT_STS, &data);
			if (err < 0)
				break;

			/* BST is 0 when the battery is present */
			val->intval = !(data & MAX77779_FG_FG_INT_MASK_Bst_m_MASK);
			if (!val->intval)
				break;

			/* chip->por prevent garbage in cycle count */
			chip->por = (data & MAX77779_FG_FG_INT_MASK_POR_m_MASK) != 0;
			if (chip->por && chip->model_ok &&
			    chip->model_reload == MAX77779_FG_LOAD_MODEL_IDLE) {
				/* trigger reload model and clear of POR */
				mutex_unlock(&chip->model_lock);
				max77779_fg_irq_thread_fn(-1, chip);
				return err;
			}
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = max77779_fg_get_temp(chip);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		err = REGMAP_READ(map, MAX77779_FG_TTE, &data);
		if (err == 0)
			val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		err = REGMAP_READ(map, MAX77779_FG_TTF, &data);
		if (err == 0)
			val->intval = reg_to_seconds(data);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = -1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		err = REGMAP_READ(map, MAX77779_FG_AvgVCell, &data);
		if (err == 0)
			val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		/* LSB: 20mV */
		err = REGMAP_READ(map, MAX77779_FG_MaxMinVolt, &data);
		if (err == 0)
			val->intval = ((data >> 8) & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		/* LSB: 20mV */
		err = REGMAP_READ(map, MAX77779_FG_MaxMinVolt, &data);
		if (err == 0)
			val->intval = (data & 0xFF) * 20000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		err = REGMAP_READ(map, MAX77779_FG_VCell, &data);
		if (err == 0)
			val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		rc = REGMAP_READ(map, MAX77779_FG_VFOCV, &data);
		if (rc == -EINVAL) {
			val->intval = -1;
			break;
		}
		val->intval = reg_to_micro_volt(data);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = chip->serial_number;
		break;
	case GBMS_PROP_HEALTH_ACT_IMPEDANCE:
		val->intval = maxfg_health_get_ai(chip->dev, chip->bhi_acim, chip->RSense);
		break;
	case GBMS_PROP_HEALTH_IMPEDANCE:
		val->intval = max77779_fg_health_read_impedance(chip);
		break;
	case GBMS_PROP_RESISTANCE:
		val->intval = maxfg_read_resistance(map, chip->RSense);
		break;
	case GBMS_PROP_RESISTANCE_RAW:
		val->intval = maxfg_read_resistance_raw(map);
		break;
	case GBMS_PROP_RESISTANCE_AVG:
		val->intval = maxfg_read_resistance_avg(chip->RSense);
		break;
	case GBMS_PROP_BATTERY_AGE:
		val->intval = max77779_fg_get_age(chip);
		break;
	case GBMS_PROP_CHARGE_FULL_ESTIMATE:
		val->intval = batt_ce_full_estimate(&chip->cap_estimate);
		break;
	case GBMS_PROP_CAPACITY_FADE_RATE:
	case GBMS_PROP_CAPACITY_FADE_RATE_FCR:
		err = maxfg_get_fade_rate(chip->dev, chip->bhi_fcn_count, &val->intval, psp);
		break;
	case GBMS_PROP_BATT_ID:
		val->intval = chip->batt_id;
		break;
	case GBMS_PROP_RECAL_FG:
		/* TODO: under porting */
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err < 0)
		pr_debug("error %d reading prop %d\n", err, psp);

	mutex_unlock(&chip->model_lock);
	return err;
}

/* needs mutex_lock(&chip->model_lock); */
static int max77779_fg_health_update_ai(struct max77779_fg_chip *chip, int impedance)
{
	const u16 act_impedance = impedance / 100;
	unsigned int rcell = 0xffff;
	u16 timerh = 0xffff;
	int ret;

	if (impedance) {

		/* mOhms to reg */
		rcell = (impedance * 4096) / (1000 * chip->RSense);
		if (rcell > 0xffff) {
			pr_err("value=%d, rcell=%d out of bounds\n", impedance, rcell);
			return -ERANGE;
		}

		ret = REGMAP_READ(&chip->regmap, MAX77779_FG_TimerH, &timerh);
		if (ret < 0 || timerh == 0)
			return -EIO;
	}

	ret = maxfg_health_write_ai(act_impedance, timerh);
	if (ret == 0)
		chip->bhi_acim = 0;

	return ret;
}

static int max77779_fg_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)
					power_supply_get_drvdata(psy);
	struct gbatt_capacity_estimation *ce = &chip->cap_estimate;
	int rc = 0;

	mutex_lock(&chip->model_lock);
	if (max77779_fg_resume_check(chip)) {
		mutex_unlock(&chip->model_lock);
		return -EAGAIN;
	}
	mutex_unlock(&chip->model_lock);

	switch (psp) {
	case GBMS_PROP_BATT_CE_CTRL:

		mutex_lock(&ce->batt_ce_lock);

		if (!chip->model_state_valid) {
			mutex_unlock(&ce->batt_ce_lock);
			return -EAGAIN;
		}

		if (val->intval) {

			if (!ce->cable_in) {
				rc = batt_ce_init(ce, chip);
				ce->cable_in = (rc == 0);
			}

		} else if (ce->cable_in) {
			if (ce->estimate_state == ESTIMATE_PENDING)
				cancel_delayed_work_sync(&ce->settle_timer);

			/* race with batt_ce_capacityfiltered_work() */
			batt_ce_stop_estimation(ce, ESTIMATE_NONE);
			batt_ce_dump_data(ce, chip->ce_log);
			ce->cable_in = false;
		}
		mutex_unlock(&ce->batt_ce_lock);

		mod_delayed_work(system_wq, &chip->model_work, msecs_to_jiffies(351));

		break;
	case GBMS_PROP_HEALTH_ACT_IMPEDANCE:
		mutex_lock(&chip->model_lock);
		rc = max77779_fg_health_update_ai(chip, val->intval);
		mutex_unlock(&chip->model_lock);
		break;
	case GBMS_PROP_FG_REG_LOGGING:
		max77779_fg_monitor_log_data(chip, !!val->intval);
		break;
	case GBMS_PROP_RECAL_FG:
		/* TODO: under porting */
		break;
	default:
		return -EINVAL;
	}

	if (rc < 0)
		return rc;

	return 0;
}

static int max77779_fg_property_is_writeable(struct power_supply *psy,
					     enum power_supply_property psp)
{
	switch (psp) {
	case GBMS_PROP_BATT_CE_CTRL:
	case GBMS_PROP_HEALTH_ACT_IMPEDANCE:
		return 1;
	default:
		break;
	}

	return 0;
}

/* TODO: b/309384491 - FG register dump for max77779 */
static int max77779_fg_extensive_dump_work(struct max77779_fg_chip *chip) {
	return 0;
}

/* TODO: b/309384491 - FG register dump for max77779 */
static int max77779_fg_recurent_dump_work(struct max77779_fg_chip *chip) {
	return 0;
}

static int max77779_fg_check_logging_event(struct max77779_fg_chip *chip)
{
	int ret = 0, event = chip->fg_logging_events;
	u16 data, fullcapnom, designcap, repsoc, mixsoc, edet, fdet, vfocv, avgvcell, ibat;
	bool changed;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_FullCapNom, &fullcapnom);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_DesignCap, &designcap);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_RepSOC, &data);
	if (ret < 0)
		return ret;
	repsoc = (data >> 8) & 0x00FF;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_MixSOC, &data);
	if (ret < 0)
		return ret;
	mixsoc = (data >> 8) & 0x00FF;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_FStat, &data);
	if (ret < 0)
		return ret;
	edet = (data & MAX77779_FG_FStat_EDet_MASK);

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_Status2, &data);
	if (ret < 0)
		return ret;
	fdet = (data & MAX77779_FG_Status2_FullDet_MASK);

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_VFOCV, &vfocv);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_AvgVCell, &avgvcell);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_Current, &ibat);
	if (ret < 0)
		return ret;

	/* stop when FullCapNom updated */
	if ((event & MAX77779_FG_EVENT_FULLCAPNOM_LOW) &&
	    (fullcapnom != chip->pre_fullcapnom))
		event &= ~MAX77779_FG_EVENT_FULLCAPNOM_LOW;

	/* stop when FullCapNom updated */
	if ((event & MAX77779_FG_EVENT_FULLCAPNOM_HIGH) &&
	    (fullcapnom != chip->pre_fullcapnom))
		event &= ~MAX77779_FG_EVENT_FULLCAPNOM_HIGH;

	/* stop when RepSoc > 20% */
	if ((event & MAX77779_FG_EVENT_REPSOC_EDET) && (repsoc > 20))
		event &= ~MAX77779_FG_EVENT_REPSOC_EDET;

	/* stop when RepSoc < 80% */
	if ((event & MAX77779_FG_EVENT_REPSOC_FDET) && (repsoc < 80))
		event &= ~MAX77779_FG_EVENT_REPSOC_FDET;

	/* stop when abs(MixSoC - RepSoC) < 20% */
	if ((event & MAX77779_FG_EVENT_REPSOC) && (abs(mixsoc - repsoc) < 20))
		event &= ~MAX77779_FG_EVENT_REPSOC;

	/* stop when VFOCV < (AvgVCell - 200mV) || VFOCV > (AvgVCell + 200mV) */
	if ((event & MAX77779_FG_EVENT_VFOCV) &&
	    (reg_to_micro_volt(vfocv) < (reg_to_micro_volt(avgvcell) - 200000) ||
	     reg_to_micro_volt(vfocv) > (reg_to_micro_volt(avgvcell) + 200000)))
		event &= ~MAX77779_FG_EVENT_VFOCV;

	changed = event != chip->fg_logging_events;

	/* trigger when FullCapNom < DesignCap x 60% */
	if (fullcapnom < (designcap * 60 / 100)) {
		event |= MAX77779_FG_EVENT_FULLCAPNOM_LOW;
		chip->pre_fullcapnom = fullcapnom;
	}

	/* trigger when FullCapNom > DesignCap x 115% */
	if (fullcapnom > (designcap * 115 / 100)) {
		event |= MAX77779_FG_EVENT_FULLCAPNOM_HIGH;
		chip->pre_fullcapnom = fullcapnom;
	}

	/* trigger when RepSoC > 10% && Empty detection bit is set */
	if (repsoc > 10 && edet)
		event |= MAX77779_FG_EVENT_REPSOC_EDET;

	/* trigger when RepSoC < 90% && Full detection is enabled */
	if (repsoc < 90 && fdet)
		event |= MAX77779_FG_EVENT_REPSOC_FDET;

	/* trigger when abs(MixSoC - RepSoC) > 25% */
	if (abs(mixsoc - repsoc) > 25)
		event |= MAX77779_FG_EVENT_REPSOC;

	/*
	 * trigger when (VFOCV < (AvgVCell - 1V) || VFOCV > (AvgVCell + 1V))
	 *	         && abs(Current) < 5A
	 */
	if ((reg_to_micro_volt(vfocv) < (reg_to_micro_volt(avgvcell) - 1000000) ||
	     reg_to_micro_volt(vfocv) > (reg_to_micro_volt(avgvcell) + 1000000)) &&
	     abs(reg_to_micro_amp(ibat, chip->RSense)) < 5000000)
		event |= MAX77779_FG_EVENT_VFOCV;

	changed |= event != chip->fg_logging_events;

	if (changed) {
		gbms_logbuffer_devlog(chip->monitor_log, chip->dev,
				      LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				      "event changed 0x%02X->0x%02X: fullcapnom=%d designcap=%d "
				      "repsoc=%d edet=%d fdet=%d mixsoc=%d vfocv=%d avgvcell=%d "
				      "current=%d",
				      chip->fg_logging_events, event, fullcapnom, designcap,
				      repsoc, !!edet, !!fdet, mixsoc,
				      reg_to_micro_volt(vfocv), reg_to_micro_volt(avgvcell),
				      reg_to_micro_amp(ibat, chip->RSense));
	}
	chip->fg_logging_events = event;

	return event;
}

/*
 * A full reset restores the ICs to their power-up state the same as if power
 * had been cycled.
 */
#define CMD_HW_RESET 0x000F
static int max77779_fg_full_reset(struct max77779_fg_chip *chip)
{
	int ret = 0;

	ret = max77779_fg_find_pmic(chip);
	if (ret) {
		dev_err(chip->dev, "Error finding pmic\n");
		return ret;
	}

	ret = max77779_external_pmic_reg_write(chip->pmic_dev, MAX77779_PMIC_RISCV_COMMAND_HW,
					       CMD_HW_RESET);
	dev_warn(chip->dev, "%s, ret=%d\n", __func__, ret);
	if (ret == 0) {
		msleep(MAX77779_FG_TPOR_MS);
		/* check POR after reset */
		max77779_fg_irq_thread_fn(-1, chip);
	}

	return ret;
}

static int max77779_fg_mask_por(struct max77779_fg_chip *chip, bool mask)
{
	u16 fg_int_mask;
	int err;

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_FG_INT_MASK, &fg_int_mask);
	if (err)
		return err;

	if (mask)
		fg_int_mask |= MAX77779_FG_FG_INT_MASK_POR_m_MASK;
	else
		fg_int_mask &= ~MAX77779_FG_FG_INT_MASK_POR_m_MASK;

	err = MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_FG_INT_MASK, fg_int_mask);

	return err;
}

static irqreturn_t max77779_fg_irq_thread_fn(int irq, void *obj)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)obj;
	u16 fg_status, fg_int_sts, fg_int_sts_clr;
	int err = 0;

	if (!chip || (irq != -1 && irq != chip->irq)) {
		WARN_ON_ONCE(1);
		return IRQ_NONE;
	}

	if (irq != -1 && max77779_fg_resume_check(chip)) {
		dev_warn_ratelimited(chip->dev, "%s: irq skipped, irq%d\n", __func__, irq);
		return IRQ_HANDLED;
	}

	err = REGMAP_READ(&chip->regmap, MAX77779_FG_Status, &fg_status);
	if (err) {
		dev_err_ratelimited(chip->dev, "%s i2c error reading status, IRQ_NONE\n", __func__);
		return IRQ_NONE;
	}
	err = REGMAP_READ(&chip->regmap, MAX77779_FG_FG_INT_STS, &fg_int_sts);
	if (err) {
		dev_err_ratelimited(chip->dev, "%s i2c error reading INT status, IRQ_NONE\n", __func__);
		return IRQ_NONE;
	}
	if (fg_status == 0 && fg_int_sts == 0) {
		dev_err_ratelimited(chip->dev, "fg_status == 0 and fg_int_sts == 0\n");
		return IRQ_NONE;
	}

	dev_dbg(chip->dev, "FG_Status:%04x, FG_INT_STS:%04x\n", fg_status, fg_int_sts);

	/* only used to report health */
	chip->health_status |= fg_status;
	fg_int_sts_clr = fg_int_sts;

	if (fg_status & MAX77779_FG_Status_PONR_MASK) {
		mutex_lock(&chip->model_lock);
		chip->por = true;

		gbms_logbuffer_devlog(chip->monitor_log, chip->dev,
				      LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				      "POR is set (FG_INT_STS:%04x), irq:%d, model_reload:%d",
				      fg_int_sts, irq, chip->model_reload);

		/* trigger model load if not on-going */
		if (chip->model_reload == MAX77779_FG_LOAD_MODEL_IDLE) {
			err = max77779_fg_model_reload(chip, false);
			if (err < 0)
				dev_dbg(chip->dev, "unable to reload model, err=%d\n", err);
		}

		mutex_unlock(&chip->model_lock);
	}

	/* NOTE: should always clear everything even if we lose state */
	MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_FG_INT_STS, fg_int_sts_clr);

	/* SOC interrupts need to go through all the time */
	if (fg_status & MAX77779_FG_Status_dSOCi_MASK) {
		max77779_fg_monitor_log_data(chip, false);
		max77779_fg_update_cycle_count(chip);
		max77779_fg_check_logging_event(chip);
	}

	if (chip->psy)
		power_supply_changed(chip->psy);

	/*
	 * oneshot w/o filter will unmask on return but gauge will take up
	 * to 351 ms to clear ALRM1.
	 * NOTE: can do this masking on gauge side (Config, 0x1D) and using a
	 * workthread to re-enable.
	 */
	if (irq != -1)
		msleep(MAX77779_FG_TICLR_MS);


	return IRQ_HANDLED;
}

/* used to find batt_node and chemistry dependent FG overrides */
static int max77779_fg_read_batt_id(int *batt_id, const struct max77779_fg_chip *chip)
{
	bool defer;
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	u32 temp_id = 0;

	/* force the value in kohm */
	rc = of_property_read_u32(node, "max77779,force-batt-id", &temp_id);
	if (rc == 0) {
		dev_warn(chip->dev, "forcing battery RID %d\n", temp_id);
		*batt_id = temp_id;
		return 0;
	}

	/* return the value in kohm */
	rc = gbms_storage_read(GBMS_TAG_BRID, &temp_id, sizeof(temp_id));
	defer = (rc == -EPROBE_DEFER) ||
		(rc == -EINVAL) ||
		((rc == 0) && (temp_id == -EINVAL));
	if (defer)
		return -EPROBE_DEFER;

	if (rc < 0) {
		dev_err(chip->dev, "failed to get batt-id rc=%d\n", rc);
		*batt_id = -1;
		return -EPROBE_DEFER;
	}

	*batt_id = temp_id;
	return 0;
}

static struct device_node *max77779_fg_find_batt_node(struct max77779_fg_chip *chip)
{
	const int batt_id = chip->batt_id;
	const struct device *dev = chip->dev;
	struct device_node *config_node, *child_node;
	u32 batt_id_kohm;
	int ret;

	config_node = of_find_node_by_name(dev->of_node, "max77779,config");
	if (!config_node) {
		dev_warn(dev, "Failed to find max77779,config setting\n");
		return NULL;
	}

	for_each_child_of_node(config_node, child_node) {
		ret = of_property_read_u32(child_node, "max77779,batt-id-kohm", &batt_id_kohm);
		if (ret != 0)
			continue;

		if (batt_id == batt_id_kohm)
			return child_node;
	}

	return NULL;
}

static int get_irq_none_cnt(void *data, u64 *val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	*val = chip->debug_irq_none_cnt;
	return 0;
}

static int set_irq_none_cnt(void *data, u64 val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	if (val == 0)
		chip->debug_irq_none_cnt = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(irq_none_cnt_fops, get_irq_none_cnt,
			set_irq_none_cnt, "%llu\n");


static int debug_fg_reset(void *data, u64 val)
{
	struct max77779_fg_chip *chip = data;
	int ret = 0;

	mutex_lock(&chip->model_lock);
	/* irq_disabled set by firmware update */
	if (chip->irq_disabled)
		ret = -EBUSY;
	else if (val != 1)
		ret = -EINVAL;

	mutex_unlock(&chip->model_lock);

	if (ret == 0)
		ret = max77779_fg_full_reset(chip);
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fg_reset_fops, NULL, debug_fg_reset, "%llu\n");


int max77779_fg_enable_firmware_update(struct device *dev, bool enable) {
	struct max77779_fg_chip *chip = dev_get_drvdata(dev);
	int ret = -EAGAIN;

	if (!chip)
		return ret;

	mutex_lock(&chip->model_lock);

	if (max77779_fg_resume_check(chip))
		goto max77779_fg_enable_firmware_update_exit;

	/* enable/disable irq for firmware update */
	if (enable && !chip->irq_disabled) {
		chip->irq_disabled = true;
		disable_irq_wake(chip->irq);
		disable_irq(chip->irq);
	} else if (!enable && chip->irq_disabled) {
		chip->irq_disabled = false;
		enable_irq(chip->irq);
		enable_irq_wake(chip->irq);
	}

	chip->fw_update_mode = enable;
	ret = 0;

max77779_fg_enable_firmware_update_exit:
	mutex_unlock(&chip->model_lock);

	return ret;
};

EXPORT_SYMBOL_GPL(max77779_fg_enable_firmware_update);


static int debug_ce_start(void *data, u64 val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	batt_ce_start(&chip->cap_estimate, val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ce_start_fops, NULL, debug_ce_start, "%llu\n");

/* Model reload will be disabled if the node is not found */
static int max77779_fg_init_model(struct max77779_fg_chip *chip)
{
	const bool no_battery = chip->fake_battery == 0;
	void *model_data;

	if (no_battery)
		return 0;

	/* ->batt_id negative for no lookup */
	if (chip->batt_id >= 0) {
		chip->batt_node = max77779_fg_find_batt_node(chip);
		pr_debug("node found=%d for ID=%d\n",
			 !!chip->batt_node, chip->batt_id);
	}

	/* TODO: split allocation and initialization */
	model_data = max77779_init_data(chip->dev, chip->batt_node ?
					chip->batt_node : chip->dev->of_node,
					&chip->regmap, &chip->regmap_debug);
	if (IS_ERR(model_data))
		return PTR_ERR(model_data);

	chip->model_data = model_data;

	if (!chip->batt_node) {
		dev_warn(chip->dev, "No child node for ID=%d\n", chip->batt_id);
		chip->model_reload = MAX77779_FG_LOAD_MODEL_DISABLED;
	} else {
		dev_info(chip->dev, "model_data ok for ID=%d\n", chip->batt_id);
		chip->model_reload = MAX77779_FG_LOAD_MODEL_IDLE;
		chip->designcap = max77779_get_designcap(chip->model_data);
	}

	return 0;
}

/* change battery_id and cause reload of the FG model */
static int debug_batt_id_set(void *data, u64 val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;
	int ret;

	mutex_lock(&chip->model_lock);

	/* reset state (if needed) */
	if (chip->model_data)
		max77779_free_data(chip->model_data);
	chip->batt_id = val;

	/* re-init the model data (lookup in DT) */
	ret = max77779_fg_init_model(chip);
	if (ret == 0)
		max77779_fg_model_reload(chip, true);

	mutex_unlock(&chip->model_lock);

	dev_info(chip->dev, "Force model for batt_id=%llu (%d)\n", val, ret);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_batt_id_fops, NULL, debug_batt_id_set, "%llu\n");

static int debug_fake_battery_set(void *data, u64 val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	chip->fake_battery = (int)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fake_battery_fops, NULL,
			debug_fake_battery_set, "%llu\n");

static int debug_fw_revision_get(void *data, u64 *val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	*val = chip->fw_rev;
	return 0;
}

static int debug_fw_revision_set(void *data, u64 val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	chip->fw_rev = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fw_revision_fops, debug_fw_revision_get,
			debug_fw_revision_set, "%llu\n");

static int debug_fw_sub_revision_get(void *data, u64 *val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	*val = chip->fw_sub_rev;
	return 0;
}

static int debug_fw_sub_revision_set(void *data, u64 val)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)data;

	chip->fw_sub_rev = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fw_sub_revision_fops, debug_fw_sub_revision_get,
			debug_fw_sub_revision_set, "%llu\n");

static void max77779_fg_reglog_dump(struct maxfg_reglog *regs,
				    size_t size, char *buff)
{
	int i, len = 0;

	for (i = 0; i < NB_REGMAP_MAX; i++) {
		if (size <= len)
			break;
		if (test_bit(i, regs->valid))
			len += scnprintf(&buff[len], size - len, "%02X:%04X\n",
					 i, regs->data[i]);
	}

	if (len == 0)
		scnprintf(buff, size, "No record\n");
}

static ssize_t debug_get_reglog_writes(struct file *filp, char __user *buf,
				       size_t count, loff_t *ppos)
{
	char *buff;
	ssize_t rc = 0;
	struct maxfg_reglog *reglog = (struct maxfg_reglog *)filp->private_data;

	buff = kmalloc(count, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	max77779_fg_reglog_dump(reglog, count, buff);
	rc = simple_read_from_buffer(buf, count, ppos, buff, strlen(buff));

	kfree(buff);

	return rc;
}

BATTERY_DEBUG_ATTRIBUTE(debug_reglog_writes_fops,
			debug_get_reglog_writes, NULL);

static ssize_t max77779_fg_show_custom_model(struct file *filp, char __user *buf,
					     size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	char *tmp;
	int len;

	if (!chip->model_data)
		return -EINVAL;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	mutex_lock(&chip->model_lock);
	len = max77779_fg_model_cstr(tmp, PAGE_SIZE, chip->model_data);
	mutex_unlock(&chip->model_lock);

	if (len > 0)
		len = simple_read_from_buffer(buf, count,  ppos, tmp, len);

	kfree(tmp);

	return len;
}

static ssize_t max77779_fg_set_custom_model(struct file *filp, const char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	char *tmp;
	int ret;

	if (!chip->model_data)
		return -EINVAL;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = simple_write_to_buffer(tmp, PAGE_SIZE, ppos, user_buf, count);
	if (!ret) {
		kfree(tmp);
		return -EFAULT;
	}

	mutex_lock(&chip->model_lock);
	ret = max77779_fg_model_sscan(chip->model_data, tmp, count);
	if (ret < 0)
		count = ret;
	mutex_unlock(&chip->model_lock);

	kfree(tmp);

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_custom_model_fops, max77779_fg_show_custom_model,
			max77779_fg_set_custom_model);

static int debug_sync_model(void *data, u64 val)
{
	struct max77779_fg_chip *chip = data;
	int ret;

	if (!chip->model_data)
		return -EINVAL;

	/* re-read new state from Fuel gauge, save to storage  */
	ret = max77779_model_read_state(chip->model_data);
	if (ret == 0) {
		ret = max77779_model_check_state(chip->model_data);
		if (ret < 0)
			pr_warn("%s: warning invalid state %d\n", __func__, ret);

		ret = max77779_save_state_data(chip->model_data);
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_sync_model_fops, NULL, debug_sync_model, "%llu\n");


static ssize_t max77779_fg_show_debug_data(struct file *filp, char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	char msg[8];
	u16 data;
	int ret;

	ret = REGMAP_READ(&chip->regmap, chip->debug_reg_address, &data);
	if (ret < 0)
		return ret;

	ret = scnprintf(msg, sizeof(msg), "%x\n", data);

	return simple_read_from_buffer(buf, count, ppos, msg, ret);
}

static ssize_t max77779_fg_set_debug_data(struct file *filp,
					  const char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	char temp[8] = { };
	u16 data;
	int ret;

	ret = simple_write_to_buffer(temp, sizeof(temp) - 1, ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	ret = kstrtou16(temp, 16, &data);
	if (ret < 0)
		return ret;

	ret =  MAX77779_FG_REGMAP_WRITE(&chip->regmap, chip->debug_reg_address, data);
	if (ret < 0)
		return ret;

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_reg_data_fops, max77779_fg_show_debug_data,
			max77779_fg_set_debug_data);

static ssize_t max77779_fg_show_dbg_debug_data(struct file *filp, char __user *buf,
					       size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	char msg[8];
	u16 data;
	int ret;

	ret = REGMAP_READ(&chip->regmap_debug, chip->debug_dbg_reg_address, &data);
	if (ret < 0)
		return ret;

	ret = scnprintf(msg, sizeof(msg), "%x\n", data);

	return simple_read_from_buffer(buf, count, ppos, msg, ret);
}

static ssize_t max77779_fg_set_dbg_debug_data(struct file *filp,
					      const char __user *user_buf,
					      size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	char temp[8] = { };
	u16 data;
	int ret;

	ret = simple_write_to_buffer(temp, sizeof(temp) - 1, ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	ret = kstrtou16(temp, 16, &data);
	if (ret < 0)
		return ret;

	ret = MAX77779_FG_N_REGMAP_WRITE(&chip->regmap, &chip->regmap_debug,
					 chip->debug_dbg_reg_address, data);
	if (ret < 0)
		return ret;

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_reg_dbg_data_fops, max77779_fg_show_dbg_debug_data,
			max77779_fg_set_dbg_debug_data);

static ssize_t max77779_fg_show_reg_all(struct file *filp, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	const struct maxfg_regmap *map = &chip->regmap;
	u32 reg_address;
	unsigned int data;
	char *tmp;
	int ret = 0, len = 0;

	if (!map->regmap) {
		pr_err("Failed to read, no regmap\n");
		return -EIO;
	}

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (reg_address = 0; reg_address <= 0xFF; reg_address++) {
		ret = regmap_read(map->regmap, reg_address, &data);
		if (ret < 0)
			continue;

		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %04x\n", reg_address, data);
	}

	if (len > 0)
		len = simple_read_from_buffer(buf, count,  ppos, tmp, strlen(tmp));

	kfree(tmp);

	return len;
}

BATTERY_DEBUG_ATTRIBUTE(debug_reg_all_fops, max77779_fg_show_reg_all, NULL);

static ssize_t max77779_fg_show_dbg_reg_all(struct file *filp, char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;
	const struct maxfg_regmap *map = &chip->regmap_debug;
	u32 reg_address;
	unsigned int data;
	char *tmp;
	int ret = 0, len = 0;

	if (!map->regmap) {
		pr_err("Failed to read, no regmap\n");
		return -EIO;
	}

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (reg_address = 0; reg_address <= 0xFF; reg_address++) {
		ret = regmap_read(map->regmap, reg_address, &data);
		if (ret < 0)
			continue;

		len += scnprintf(tmp + len, PAGE_SIZE - len, "%02x: %04x\n", reg_address, data);
	}

	if (len > 0)
		len = simple_read_from_buffer(buf, count,  ppos, tmp, strlen(tmp));

	kfree(tmp);

	return len;
}

BATTERY_DEBUG_ATTRIBUTE(debug_reg_all_dbg_fops, max77779_fg_show_dbg_reg_all, NULL);

static ssize_t max77779_fg_force_psy_update(struct file *filp,
					    const char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)filp->private_data;

	if (chip->psy)
		power_supply_changed(chip->psy);

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_force_psy_update_fops, NULL,
			max77779_fg_force_psy_update);

static int debug_cnhs_reset(void *data, u64 val)
{
	struct max77779_fg_chip *chip = data;
	u16 reset_val;
	int ret;

	reset_val = (u16)val;

	ret = gbms_storage_write(GBMS_TAG_CNHS, &reset_val,
				sizeof(reset_val));
	dev_info(chip->dev, "reset CNHS to %d, (ret=%d)\n", reset_val, ret);

	return ret == sizeof(reset_val) ? 0 : ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_reset_cnhs_fops, NULL, debug_cnhs_reset, "%llu\n");

static int debug_gmsr_reset(void *data, u64 val)
{
	struct max77779_fg_chip *chip = data;
	int ret;

	ret = max77779_reset_state_data(chip->model_data);
	dev_info(chip->dev, "reset GMSR (ret=%d)\n", ret);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_reset_gmsr_fops, NULL, debug_gmsr_reset, "%llu\n");

static int debug_ini_reload(void *data, u64 val)
{
	struct max77779_fg_chip *chip = data;
	int ret;

	if (chip->model_data)
		max77779_free_data(chip->model_data);
	/* re-init the model data (lookup in DT) */
	ret = max77779_fg_init_model(chip);
	dev_info(chip->dev, "ini_model (ret=%d)\n", ret);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ini_reload_fops, NULL, debug_ini_reload, "%llu\n");
/*
 * TODO: add the building blocks of google capacity
 *
 * case POWER_SUPPLY_PROP_DELTA_CC_SUM:
 *	val->intval = chip->cap_estimate.delta_cc_sum;
 *	break;
 * case POWER_SUPPLY_PROP_DELTA_VFSOC_SUM:
 *	val->intval = chip->cap_estimate.delta_vfsoc_sum;
 *	break;
 */

static int fg_fw_update_set(void* data, u64 val) {
	int ret = -EINVAL;
	uint8_t op_st = (uint8_t)val;
	struct max77779_fg_chip *chip = data;

	if (chip)
		ret = gbms_storage_write(GBMS_TAG_FGST, &op_st, 1);

	dev_info(chip->dev, "set FG operation status: %02x, (ret=%d)\n", op_st, ret);
	return 0;
}

static int fg_fw_update_get(void* data, u64* val) {
	int ret = -EINVAL;
	uint8_t op_st = 0xff;
	struct max77779_fg_chip *chip = data;

	if (chip)
		ret = gbms_storage_read(GBMS_TAG_FGST, &op_st, 1);
	*val = op_st;

	dev_info(chip->dev, "get FG operation status: %02x, (ret=%d)\n", op_st, ret);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_fw_update_fops, fg_fw_update_get, fg_fw_update_set, "%llu\n");

static ssize_t act_impedance_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count) {
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);
	int value, ret = 0;

	ret = kstrtoint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&chip->model_lock);

	ret = max77779_fg_health_update_ai(chip, value);
	if (ret == 0)
		chip->bhi_acim = 0;

	dev_info(chip->dev, "value=%d  (%d)\n", value, ret);

	mutex_unlock(&chip->model_lock);
	return count;
}

static ssize_t act_impedance_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 maxfg_health_get_ai(chip->dev, chip->bhi_acim, chip->RSense));
}

static DEVICE_ATTR_RW(act_impedance);

static ssize_t fg_logging_events_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct max77779_fg_chip *chip = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "0x%02X\n", chip->fg_logging_events);
}

static DEVICE_ATTR_RO(fg_logging_events);

static int max77779_fg_init_sysfs(struct max77779_fg_chip *chip)
{
	struct dentry *de;

	de = debugfs_create_dir(chip->max77779_fg_psy_desc.name, 0);
	if (IS_ERR_OR_NULL(de))
		return -ENOENT;

	debugfs_create_file("irq_none_cnt", 0644, de, chip, &irq_none_cnt_fops);
	debugfs_create_file("fg_reset", 0400, de, chip, &debug_fg_reset_fops);
	debugfs_create_file("ce_start", 0400, de, chip, &debug_ce_start_fops);
	debugfs_create_file("fake_battery", 0400, de, chip, &debug_fake_battery_fops);
	debugfs_create_file("batt_id", 0600, de, chip, &debug_batt_id_fops);
	debugfs_create_file("force_psy_update", 0600, de, chip, &debug_force_psy_update_fops);

	if (chip->regmap.reglog)
		debugfs_create_file("regmap_writes", 0440, de,
					chip->regmap.reglog,
					&debug_reglog_writes_fops);

	debugfs_create_file("fg_model", 0444, de, chip, &debug_custom_model_fops);
	debugfs_create_bool("model_ok", 0444, de, &chip->model_ok);
	debugfs_create_file("sync_model", 0400, de, chip, &debug_sync_model_fops);

	/* new debug interface */
	debugfs_create_u32("address", 0600, de, &chip->debug_reg_address);
	debugfs_create_u32("debug_address", 0600, de, &chip->debug_dbg_reg_address);
	debugfs_create_file("data", 0600, de, chip, &debug_reg_data_fops);
	debugfs_create_file("debug_data", 0600, de, chip, &debug_reg_dbg_data_fops);

	/* dump all registers */
	debugfs_create_file("registers", 0444, de, chip, &debug_reg_all_fops);
	debugfs_create_file("debug_registers", 0444, de, chip, &debug_reg_all_dbg_fops);

	/* reset fg eeprom data for debugging */
	debugfs_create_file("cnhs_reset", 0400, de, chip, &debug_reset_cnhs_fops);
	debugfs_create_file("gmsr_reset", 0400, de, chip, &debug_reset_gmsr_fops);

	/* reloaded INI */
	debugfs_create_file("ini_reload", 0400, de, chip, &debug_ini_reload_fops);

	/* capacity fade */
	debugfs_create_u32("bhi_fcn_count", 0644, de, &chip->bhi_fcn_count);

	/* fuel gauge operation status */
	debugfs_create_file("fw_update", 0600, de, chip, &debug_fw_update_fops);

	debugfs_create_file("fw_revision", 0600, de, chip, &debug_fw_revision_fops);
	debugfs_create_file("fw_sub_revision", 0600, de, chip, &debug_fw_sub_revision_fops);

	return 0;
}

static u16 max77779_fg_read_rsense(const struct max77779_fg_chip *chip)
{
	u32 rsense_default = 0;
	u16 rsense = 200;
	int ret;

	ret = of_property_read_u32(chip->dev->of_node, "max77779,rsense-default",
				   &rsense_default);
	if (ret == 0)
		rsense = rsense_default;

	return rsense;
}

static int max77779_fg_dump_param(struct max77779_fg_chip *chip)
{
	int ret;
	u16 data;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_Config, &chip->RConfig);
	if (ret < 0)
		return ret;

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_IChgTerm, &data);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "Config: 0x%04x, IChgTerm: %d\n",
		 chip->RConfig, reg_to_micro_amp(data, chip->RSense));

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_VEmpty, &data);
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "VEmpty: VE=%dmV VR=%dmV\n",
		 reg_to_vempty(data), reg_to_vrecovery(data));

	return 0;
}

/* read state from fg (if needed) and set the next update field */
static int max77779_fg_set_next_update(struct max77779_fg_chip *chip)
{
	int rc;
	u16 reg_cycle;

	/* do not save data when battery ID not clearly */
	if (chip->batt_id == DEFAULT_BATTERY_ID)
		return 0;

	rc = REGMAP_READ(&chip->regmap, MAX77779_FG_Cycles, &reg_cycle);
	if (rc < 0)
		return rc;

	if (chip->model_next_update && reg_cycle < chip->model_next_update)
		return 0;

	/* read new state from Fuel gauge, save to storage if needed */
	rc = max77779_model_read_state(chip->model_data);
	if (rc == 0) {
		rc = max77779_model_check_state(chip->model_data);
		if (rc < 0) {
			pr_debug("%s: fg model state is corrupt rc=%d\n",
				 __func__, rc);
			return -EINVAL;
		}
	}

	if (rc == 0 && chip->model_next_update)
		rc = max77779_save_state_data(chip->model_data);
	/*
	 * cycle register LSB is 25% of one cycle
	 * schedule next update at multiples of 4
	 */
	if (rc == 0)
		chip->model_next_update = (reg_cycle + (1 << 2)) & ~((1 << 2) - 1);

	pr_debug("%s: reg_cycle=%d next_update=%d rc=%d\n", __func__,
		 reg_cycle, chip->model_next_update, rc);

	return 0;
}

static int max77779_fg_model_load(struct max77779_fg_chip *chip)
{
	int ret;

	/* retrieve model state from permanent storage only on boot */
	if (!chip->model_state_valid) {

		/*
		 * retrieve state from storage: retry on -EAGAIN as long as
		 * model_reload > _IDLE
		 */
		ret = max77779_load_state_data(chip->model_data);
		if (ret == -EAGAIN)
			return -EAGAIN;
		if (ret < 0)
			dev_warn(chip->dev, "Load Model Using Default State (%d)\n", ret);

		/* use the state from the DT when GMSR is invalid */
	}

	/* get fw version from pmic if it's not ready during init */
	if (!chip->fw_rev && !chip->fw_sub_rev)
		max77779_fg_get_fw_ver(chip);

	/*
	 * failure on the gauge: retry as long as model_reload > IDLE
	 * pass current firmware revision to model load procedure
	 */
	ret = max77779_load_gauge_model(chip->model_data, chip->fw_rev, chip->fw_sub_rev);
	if (ret < 0) {
		dev_err(chip->dev, "Load Model Failed ret=%d\n", ret);
		return -EAGAIN;
	}

	/* mark model state as "safe" */
	chip->reg_prop_capacity_raw = MAX77779_FG_RepSOC;
	chip->model_state_valid = true;
	return 0;
}

static void max77779_fg_model_work(struct work_struct *work)
{
	struct max77779_fg_chip *chip = container_of(work, struct max77779_fg_chip,
						     model_work.work);
	bool new_model = false;
	u16 reg_cycle;
	int rc;

	if (!chip->model_data)
		return;

	mutex_lock(&chip->model_lock);

	/* set model_reload to the #attempts, might change cycle count */
	if (chip->model_reload > MAX77779_FG_LOAD_MODEL_IDLE) {
		rc = max77779_fg_model_load(chip);
		gbms_logbuffer_devlog(chip->monitor_log, chip->dev,
				      LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				      "Model loading complete, rc=%d, reload=%d", rc,
				      chip->model_reload);
		if (rc == 0) {
			max77779_fg_restore_battery_cycle(chip);
			rc = REGMAP_READ(&chip->regmap, MAX77779_FG_Cycles, &reg_cycle);
			if (rc == 0 && reg_cycle >= 0) {
				chip->model_reload = MAX77779_FG_LOAD_MODEL_IDLE;
				chip->model_ok = true;
				new_model = true;
				/* saved new value in max77779_fg_set_next_update */
				chip->model_next_update = reg_cycle > 0 ? reg_cycle - 1 : 0;
			}
		} else if (rc != -EAGAIN) {
			chip->model_reload = MAX77779_FG_LOAD_MODEL_DISABLED;
			chip->model_ok = false;
		} else {
			chip->model_reload += 1;
			mod_delayed_work(system_wq, &chip->model_work, msecs_to_jiffies(1000));
		}
	}

	if (new_model) {
		dev_info(chip->dev, "FG Model OK, ver=%d next_update=%d\n",
			 max77779_fg_model_version(chip->model_data),
			 chip->model_next_update);
		/* force check again after model loading */
		chip->current_offset_check_done = false;
		max77779_current_offset_check(chip);
		max77779_fg_prime_battery_qh_capacity(chip);
		power_supply_changed(chip->psy);
	}

	mutex_unlock(&chip->model_lock);
}

static int read_chip_property_u32(const struct max77779_fg_chip *chip,
				  char *property, u32 *data32)
{
	int ret;

	if (chip->batt_node) {
		ret = of_property_read_u32(chip->batt_node, property, data32);
		if (ret == 0)
			return ret;
	}

	return of_property_read_u32(chip->dev->of_node, property, data32);
}

static int max77779_fg_log_event(struct max77779_fg_chip *chip, gbms_tag_t tag)
{
	u8 event_count;
	int ret = 0;

	ret = gbms_storage_read(tag, &event_count, sizeof(event_count));
	if (ret < 0)
		return ret;

	/* max count */
	if (event_count == 0xFE)
		return 0;

	/* initial value */
	if (event_count == 0xFF)
		event_count = 1;
	else
		event_count++;

	ret = gbms_storage_write(tag, &event_count, sizeof(event_count));
	if (ret < 0)
		return ret;

	dev_info(chip->dev, "tag:0x%X, event_count:%d\n", tag, event_count);

	return 0;
}

/* handle recovery of FG state */
static int max77779_fg_init_model_data(struct max77779_fg_chip *chip)
{
	int ret;

	if (!chip->model_data)
		return 0;

	if (!max77779_fg_model_check_version(chip->model_data)) {
		ret = max77779_reset_state_data(chip->model_data);
		if (ret < 0)
			dev_err(chip->dev, "GMSR: model data didn't erase ret=%d\n", ret);
		else
			dev_warn(chip->dev, "GMSR: model data erased\n");

		gbms_logbuffer_devlog(chip->monitor_log, chip->dev,
				      LOGLEVEL_INFO, 0, LOGLEVEL_INFO,
				      "FG Version Changed, Reload");

		ret = max77779_fg_full_reset(chip);
		if (ret < 0)
			dev_warn(chip->dev, "Reset unsuccessful, ret=%d\n", ret);

		return 0;
	}

	/* TODO add retries */
	ret = max77779_model_read_state(chip->model_data);
	if (ret < 0) {
		dev_err(chip->dev, "FG Model Error (%d)\n", ret);
		return -EPROBE_DEFER;
	}

	ret = max77779_fg_set_next_update(chip);
	if (ret < 0)
		dev_warn(chip->dev, "Error on Next Update, Will retry\n");

	dev_info(chip->dev, "FG Model OK, ver=%d next_update=%d\n",
		 max77779_model_read_version(chip->model_data),
		 chip->model_next_update);

	chip->reg_prop_capacity_raw = MAX77779_FG_RepSOC;
	chip->model_state_valid = true;
	chip->model_ok = true;
	return 0;
}

static int max77779_fg_init_chip(struct max77779_fg_chip *chip)
{
	int ret;
	u16 data = 0;

	if (of_property_read_bool(chip->dev->of_node, "max77779,force-hard-reset"))
		max77779_fg_full_reset(chip);

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_Status, &data);
	if (ret < 0)
		return -EPROBE_DEFER;
	chip->por = (data & MAX77779_FG_Status_PONR_MASK) != 0;

	/* TODO: handle RSense 0 */
	chip->RSense = max77779_fg_read_rsense(chip);
	if (chip->RSense == 0)
		dev_err(chip->dev, "no default RSense value\n");

	/* set maxim,force-batt-id in DT to not delay the probe */
	ret = max77779_fg_read_batt_id(&chip->batt_id, chip);
	if (ret == -EPROBE_DEFER) {
		if (chip->batt_id_defer_cnt) {
			chip->batt_id_defer_cnt -= 1;
			return -EPROBE_DEFER;
		}

		chip->batt_id = DEFAULT_BATTERY_ID;
		dev_info(chip->dev, "default device battery ID = %d\n",
			 chip->batt_id);
	} else {
		dev_info(chip->dev, "device battery RID: %d kohm\n",
			 chip->batt_id);
	}

	/* TODO: b/283489811 - fix this */
	/* do not request the interrupt if can't read battery or not present */
	if (chip->batt_id == DEFAULT_BATTERY_ID || chip->batt_id == DUMMY_BATTERY_ID) {
		ret = MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_Config2, 0x0);
		if (ret < 0)
			dev_warn(chip->dev, "Cannot write 0x0 to Config(%d)\n", ret);
	}

	/*
	 * FG model is ony used for integrated FG (MW). Loading a model might
	 * change the capacity drift WAR algo_ver and design_capacity.
	 * NOTE: design_capacity used for drift might be updated after loading
	 * a FG model.
	 */
	ret = max77779_fg_init_model(chip);
	if (ret < 0)
		dev_err(chip->dev, "Cannot init FG model (%d)\n", ret);

	ret = max77779_fg_dump_param(chip);
	if (ret < 0)
		return -EPROBE_DEFER;
	dev_info(chip->dev, "RSense value %d micro Ohm\n", chip->RSense * 10);

	ret = REGMAP_READ(&chip->regmap, MAX77779_FG_FG_INT_STS, &data);
	if (!ret && data & MAX77779_FG_FG_INT_STS_Br_MASK) {
		dev_info(chip->dev, "Clearing Battery Removal bit\n");
		MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_FG_INT_STS,
					 MAX77779_FG_FG_INT_STS_Br_MASK);
	}
	if (!ret && data & MAX77779_FG_FG_INT_STS_Bi_MASK) {
		dev_info(chip->dev, "Clearing Battery Insertion bit\n");
		MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_FG_INT_STS,
					 MAX77779_FG_FG_INT_STS_Bi_MASK);
	}

	MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_FG_INT_MASK,
				 MAX77779_FG_FG_INT_MASK_dSOCi_m_CLEAR);

	max77779_fg_restore_battery_cycle(chip);

	/* triggers loading of the model in the irq handler on POR */
	if (!chip->por) {
		max77779_fg_update_cycle_count(chip);
		ret = max77779_fg_init_model_data(chip);
		if (ret < 0)
			return ret;

		if (chip->model_ok)
			max77779_fg_prime_battery_qh_capacity(chip);
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
static int max77779_fg_prop_iter(int index, gbms_tag_t *tag, void *ptr)
{
	static gbms_tag_t keys[] = {GBMS_TAG_CLHI};
	const int count = ARRAY_SIZE(keys);

	if (index >= 0 && index < count) {
		*tag = keys[index];
		return 0;
	}

	return -ENOENT;
}

static int max77779_fg_prop_read(gbms_tag_t tag, void *buff, size_t size,
				 void *ptr)
{
	struct max77779_fg_chip *chip = (struct max77779_fg_chip *)ptr;
	int ret = -ENOENT;

	switch (tag) {
	case GBMS_TAG_CLHI:
		ret = maxfg_collect_history_data(buff, size, chip->por, chip->designcap,
						 &chip->regmap, &chip->regmap_debug);
		break;

	default:
		break;
	}

	return ret;
}

static struct gbms_storage_desc max77779_fg_prop_dsc = {
	.iter = max77779_fg_prop_iter,
	.read = max77779_fg_prop_read,
};

/* ------------------------------------------------------------------------- */

/* this must be not blocking */
static void max77779_fg_read_serial_number(struct max77779_fg_chip *chip)
{
	char buff[32] = {0};
	int ret = gbms_storage_read(GBMS_TAG_MINF, buff, GBMS_MINF_LEN);

	if (ret >= 0)
		strncpy(chip->serial_number, buff, ret);
	else
		chip->serial_number[0] = '\0';
}

static void max77779_fg_init_work(struct work_struct *work)
{
	struct max77779_fg_chip *chip = container_of(work, struct max77779_fg_chip,
						     init_work.work);
	int ret = 0;

	/* these don't require nvm storage */
	ret = gbms_storage_register(&max77779_fg_prop_dsc, "max77779fg", chip);
	if (ret == -EBUSY)
		ret = 0;

	if (ret == 0)
		ret = max77779_fg_init_chip(chip);
	if (ret == -EPROBE_DEFER) {
		schedule_delayed_work(&chip->init_work,
			msecs_to_jiffies(MAX77779_FG_DELAY_INIT_MS));
		return;
	}

	/* serial number might not be stored in the FG */
	max77779_fg_read_serial_number(chip);

	mutex_init(&chip->cap_estimate.batt_ce_lock);
	chip->prev_charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->fake_capacity = -EINVAL;
	chip->resume_complete = true;
	chip->init_complete = true;
	chip->bhi_acim = 0;

	ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
					max77779_fg_irq_thread_fn,
					IRQF_TRIGGER_LOW |
					IRQF_SHARED |
					IRQF_ONESHOT,
					MAX77779_FG_I2C_DRIVER_NAME,
					chip);
	dev_info(chip->dev, "FG irq handler registered at %d (%d)\n",
			chip->irq, ret);

	if (ret == 0) {
		device_init_wakeup(chip->dev, true);
		ret = enable_irq_wake(chip->irq);
		if (ret)
			dev_err(chip->dev, "Error enabling irq wake ret:%d\n", ret);
	}

	max77779_fg_init_sysfs(chip);

	/*
	 * Handle any IRQ that might have been set before init
	 * NOTE: will clear the POR bit and trigger model load if needed
	 */
	max77779_fg_irq_thread_fn(-1, chip);

	max77779_fg_monitor_log_data(chip, true);

	max77779_current_offset_check(chip);

	dev_info(chip->dev, "init_work done\n");
}

bool max77779_fg_dbg_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case 0x8C ... 0x8F:
		case 0x9C ... 0x9F:
		case 0xA0 ... 0xA7:
		case 0xA9:
		case 0xAF:
		case 0xB1 ... 0xB3:
		case 0xB6 ... 0xB7:
		case 0xBB ... 0xBC:
		case 0xC0:
		case 0xC6:
		case 0xC8 ... 0xCA:
		case 0xD6: /* nProtMiscTh */
			return true;
	}
	return false;
}
EXPORT_SYMBOL_GPL(max77779_fg_dbg_is_reg);

bool max77779_fg_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0x14:
	case 0x16 ... 0x1D:
	case 0x1F ... 0x27:
	case 0x29: /* ICHGTERM */
	case 0x2B: /* FullCapFltr */
	case 0x2E ... 0x35:
	case 0x37: /* VFSOC */
		return true;
	case 0x39 ... 0x3A:
	case 0x3D ... 0x3F:
	case 0x40: /* Can be used for boot completion check (0x82) */
	case 0x42:
	case 0x45 ... 0x48:
	case 0x4C ... 0x4E:
	case 0x52 ... 0x54:
	case 0x62 ... 0x63:
	case 0x6C: /* CurrentOffsetCal */
	case 0x6F: /* secure update result */
	case 0x80 ... 0x9F: /* Model */
	case 0xA0: /* CGain */
	case 0xA3: /* Model cfg */
	case 0xAB:
	case 0xB0:
	case 0xB2:
	case 0xB4:
	case 0xBA:
	case 0xBE ... 0xBF:
	case 0xD0 ... 0xDB:
	case 0xE0 ... 0xE1: /* FG_Func*/
	case 0xE9 ... 0xEA:
	case 0xFF:
		return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(max77779_fg_is_reg);

void *max77779_get_model_data(struct device *dev)
{
	struct max77779_fg_chip *chip = dev_get_drvdata(dev);

	return chip ? chip->model_data : NULL;
}

static struct attribute *max77779_fg_attrs[] = {
	&dev_attr_act_impedance.attr,
	&dev_attr_offmode_charger.attr,
	&dev_attr_resistance_id.attr,
	&dev_attr_resistance.attr,
	&dev_attr_gmsr.attr,
	&dev_attr_model_state.attr,
	&dev_attr_fg_logging_events.attr,
	NULL,
};

static const struct attribute_group max77779_fg_attr_grp = {
	.attrs = max77779_fg_attrs,
};

/*
 * Initialization requirements
 * struct max77779_fg_chip *chip
 *  - dev
 *  - irq
 *  - regmap
 *  - regmap_debug
 */
int max77779_fg_init(struct max77779_fg_chip *chip)
{
	struct device *dev = chip->dev;
	struct power_supply_config psy_cfg = { };
	const char *psy_name = NULL;
	char monitor_name[32];
	int ret = 0;
	u32 data32;

	if (!chip->irq) {
		dev_err(dev, "cannot allocate irq\n");
		return -1;
	}

	chip->fake_battery = of_property_read_bool(dev->of_node, "max77779,no-battery") ? 0 : -1;
	chip->batt_id_defer_cnt = DEFAULT_BATTERY_ID_RETRIES;

	mutex_init(&section_lock);

	ret = of_property_read_u32(dev->of_node, "max77779,status-charge-threshold-ma",
				   &data32);
	if (ret == 0)
		chip->status_charge_threshold_ma = data32;
	else
		chip->status_charge_threshold_ma = DEFAULT_STATUS_CHARGE_MA;

	if (of_property_read_bool(dev->of_node, "max77779,log_writes")) {
		bool debug_reglog;

		debug_reglog = max77779_fg_reglog_init(chip);
		dev_info(dev, "write log %savailable\n",
			 debug_reglog ? "" : "not ");
	}

	/*
	 * mask all interrupts before request irq
	 * unmask in init_work
	 */
	ret = MAX77779_FG_REGMAP_WRITE(&chip->regmap, MAX77779_FG_FG_INT_MASK, 0xFFFF);
	if (ret < 0)
		dev_warn(chip->dev, "Unable to mask all interrupts (%d)\n", ret);

	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;

	ret = of_property_read_string(dev->of_node, "max77779,dual-battery", &psy_name);
	if (ret == 0)
		chip->max77779_fg_psy_desc.name = devm_kstrdup(dev, psy_name, GFP_KERNEL);
	else
		chip->max77779_fg_psy_desc.name = "max77779fg";

	dev_info(dev, "max77779_fg_psy_desc.name=%s\n", chip->max77779_fg_psy_desc.name);

	chip->ce_log = logbuffer_register(chip->max77779_fg_psy_desc.name);
	if (IS_ERR(chip->ce_log)) {
		ret = PTR_ERR(chip->ce_log);
		dev_err(dev, "failed to obtain logbuffer, ret=%d\n", ret);
		chip->ce_log = NULL;
		goto irq_unregister;
	}

	scnprintf(monitor_name, sizeof(monitor_name), "%s_%s",
		  chip->max77779_fg_psy_desc.name, "monitor");
	chip->monitor_log = logbuffer_register(monitor_name);
	if (IS_ERR(chip->monitor_log)) {
		ret = PTR_ERR(chip->monitor_log);
		dev_err(dev, "failed to obtain logbuffer, ret=%d\n", ret);
		chip->monitor_log = NULL;
		goto irq_unregister;
	}

	/* POWER_SUPPLY_PROP_TEMP and model load need the version info */
	max77779_fg_get_fw_ver(chip);

	/* fuel gauge model needs to know the batt_id */
	mutex_init(&chip->model_lock);

	chip->max77779_fg_psy_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->max77779_fg_psy_desc.get_property = max77779_fg_get_property;
	chip->max77779_fg_psy_desc.set_property = max77779_fg_set_property;
	chip->max77779_fg_psy_desc.property_is_writeable = max77779_fg_property_is_writeable;
	chip->max77779_fg_psy_desc.properties = max77779_fg_battery_props;
	chip->max77779_fg_psy_desc.num_properties = ARRAY_SIZE(max77779_fg_battery_props);

	if (of_property_read_bool(dev->of_node, "max77779,psy-type-unknown"))
		chip->max77779_fg_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	chip->psy = devm_power_supply_register(dev, &chip->max77779_fg_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy)) {
		dev_err(dev, "Couldn't register as power supply\n");
		ret = PTR_ERR(chip->psy);
		goto power_supply_unregister;
	}

	ret = sysfs_create_group(&chip->psy->dev.kobj, &max77779_fg_attr_grp);
	if (ret)
		dev_warn(dev, "Failed to create sysfs group\n");

	/*
	 * TODO:
	 *	POWER_SUPPLY_PROP_CHARGE_FULL_ESTIMATE -> GBMS_TAG_GCFE
	 *	POWER_SUPPLY_PROP_RES_FILTER_COUNT -> GBMS_TAG_RFCN
	 */

	/* M5 battery model needs batt_id and is setup during init() */
	chip->model_reload = MAX77779_FG_LOAD_MODEL_DISABLED;

	ret = of_property_read_u32(dev->of_node, "google,bhi-fcn-count",
				   &chip->bhi_fcn_count);
	if (ret < 0)
		chip->bhi_fcn_count = BHI_CAP_FCN_COUNT;

	/* use VFSOC until it can confirm that FG Model is running */
	chip->reg_prop_capacity_raw = MAX77779_FG_VFSOC;

	INIT_DELAYED_WORK(&chip->cap_estimate.settle_timer,
			  batt_ce_capacityfiltered_work);
	INIT_DELAYED_WORK(&chip->init_work, max77779_fg_init_work);
	INIT_DELAYED_WORK(&chip->model_work, max77779_fg_model_work);

	schedule_delayed_work(&chip->init_work, 0);

	return 0;

power_supply_unregister:
	power_supply_unregister(chip->psy);
irq_unregister:
	free_irq(chip->irq, chip);

	return ret;
}
EXPORT_SYMBOL_GPL(max77779_fg_init);

void max77779_fg_remove(struct max77779_fg_chip *chip)
{
	if (chip->ce_log) {
		logbuffer_unregister(chip->ce_log);
		chip->ce_log = NULL;
	}

	if (chip->model_data)
		max77779_free_data(chip->model_data);
	cancel_delayed_work(&chip->init_work);
	cancel_delayed_work(&chip->model_work);

	disable_irq_wake(chip->irq);
	device_init_wakeup(chip->dev, false);
	if (chip->irq)
		free_irq(chip->irq, chip);

	if (chip->psy)
		power_supply_unregister(chip->psy);
}
EXPORT_SYMBOL_GPL(max77779_fg_remove);

#if IS_ENABLED(CONFIG_PM)
int max77779_fg_pm_suspend(struct device *dev)
{
	struct max77779_fg_chip *chip = dev_get_drvdata(dev);

	pm_runtime_get_sync(chip->dev);
	dev_dbg(chip->dev, "%s\n", __func__);
	chip->resume_complete = false;

	pm_runtime_put_sync(chip->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_fg_pm_suspend);

int max77779_fg_pm_resume(struct device *dev)
{
	struct max77779_fg_chip *chip = dev_get_drvdata(dev);

	pm_runtime_get_sync(chip->dev);
	dev_dbg(chip->dev, "%s\n", __func__);
	chip->resume_complete = true;

	pm_runtime_put_sync(chip->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(max77779_fg_pm_resume);
#endif

MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_AUTHOR("Keewan Jung <keewanjung@google.com>");
MODULE_AUTHOR("Jenny Ho <hsiufangho@google.com>");
MODULE_AUTHOR("Daniel Okazaki <dtokazaki@google.com>");
MODULE_DESCRIPTION("MAX77779 Fuel Gauge");
MODULE_LICENSE("GPL");