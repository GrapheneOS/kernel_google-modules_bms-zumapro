/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#ifndef MAX77779_VIMON_H_
#define MAX77779_VIMON_H_

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
	int irq;
	struct regmap *regmap;
	struct dentry *de;
	struct mutex vimon_lock;
	unsigned max_cnt;
	unsigned max_triggers;
	enum max77779_vimon_state state;
	char *buf;

	/* debug interface, register to read or write */
	u32 debug_reg_address;
};

int max77779_vimon_init(struct max77779_vimon_data *data);
void max77779_vimon_remove(struct max77779_vimon_data *data);
bool max77779_vimon_is_reg(struct device *dev, unsigned int reg);
#endif
