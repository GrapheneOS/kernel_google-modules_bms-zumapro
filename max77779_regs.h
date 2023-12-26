/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 Google, LLC
 *
 * SW Support for MAX77779 IF-PMIC
 */

#ifndef MAX77779_REG_H_
#define MAX77779_REG_H_

#include "max77779_regs_1211.h"

/* TODO: b/257309885 do we still need this? */
#define MAX77779_CHG_CNFG_11_OTG_VBYP_5000MV	0x0
#define MAX77779_CHG_CNFG_11_OTG_VBYP_5100MV	0x2

/* TODO: b/257309885 do we still need this? */
#define MAX77779_CHG_CNFG_05_OTG_ILIM_DISABLE	0x00
#define MAX77779_CHG_CNFG_05_OTG_ILIM_500MA		0x01
#define MAX77779_CHG_CNFG_05_OTG_ILIM_1500MA	0x0b

#define MAX77779_CHG_CNFG_12_CHG_EN	(0x1 << 7)
#define MAX77779_CHG_CNFG_12_WCINSEL	(0x1 << 6)
#define MAX77779_CHG_CNFG_12_CHGINSEL	(0x1 << 5)
#define MAX77779_CHG_CNFG_12_DISKIP	(0x1 << 0)
/* TODO: b/257309885 do we still need this? */
#define MAX77779_CHG_CNFG_12_WCIN_REG_4_5  (0x0 << MAX77779_CHG_CNFG_12_WCIN_REG_SHIFT)
#define MAX77779_CHG_CNFG_12_WCIN_REG_4_85 (0x3 << MAX77779_CHG_CNFG_12_WCIN_REG_SHIFT)

#define MAX77779_CHG_REVERSE_BOOST_VOUT_7V	0x28

#define MAX77779_FG_CMD_HW		0xbf

/* ----------------------------------------------------------------------------
 * Mode Register
 */

enum max77779_charger_modes {
	MAX77779_CHGR_MODE_ALL_OFF			= 0x00,
	MAX77779_CHGR_MODE_ALLOW_BYP			= 0x01,
	MAX77779_CHGR_MODE_BUCK_ON			= 0x04,
	MAX77779_CHGR_MODE_CHGR_BUCK_ON			= 0x05,
	MAX77779_CHGR_MODE_BOOST_UNO_ON			= 0x08,
	MAX77779_CHGR_MODE_BOOST_ON			= 0x09,
	MAX77779_CHGR_MODE_OTG_BOOST_ON			= 0x0a,
	MAX77779_CHGR_MODE_OTG_BOOST_UNO_ON		= 0x0b,
	MAX77779_CHGR_MODE_BUCK_BOOST_UNO_ON		= 0x0c,
	MAX77779_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON	= 0x0d,
	MAX77779_CHGR_MODE_OTG_BUCK_BOOST_ON		= 0x0e,
	MAX77779_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON	= 0x0f,
};


#endif /* MAX77779_REG_H_ */
