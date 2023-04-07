// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77779 sgpio driver
 *
 * Copyright (C) 2023 Google, LLC.
 */

#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include "max77779_pmic.h"

#define MAX77779_SGPIO_CNFGx_MODE_INPUT  0b01
#define MAX77779_SGPIO_CNFGx_MODE_OUTPUT 0b10

struct max77779_pmic_sgpio_info {
	struct device		*dev;
	struct device		*core;
	struct gpio_chip	gpio_chip;
	struct irq_chip		irq_chip;
	struct mutex		lock;

	unsigned int		mask;
	unsigned int		mask_u;

	unsigned int		trig_type;
	unsigned int		trig_type_u;
};

static int max77779_pmic_sgpio_get_direction(struct gpio_chip *gc,
		unsigned int offset)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	unsigned int reg = MAX77779_SGPIO_CNFG0 + offset;
	unsigned int mode;
	int err;

	if (offset >= gc->ngpio)
		return -EINVAL;

	err = max77779_pmic_reg_read(core, reg, &mode);
	if (err)
		return err;

	mode &= MAX77779_SGPIO_CNFG0_MODE_MASK;
	mode >>= MAX77779_SGPIO_CNFG0_MODE_SHIFT;

	switch (mode) {
	case MAX77779_SGPIO_CNFGx_MODE_INPUT:
		return GPIO_LINE_DIRECTION_IN;
	case MAX77779_SGPIO_CNFGx_MODE_OUTPUT:
		return GPIO_LINE_DIRECTION_OUT;
	case 0b00:
	case 0b11:
	default:
		return -ENODEV;
	}
}

static int max77779_pmic_sgpio_direction_input(struct gpio_chip *gc,
		unsigned int offset)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	unsigned int reg = MAX77779_SGPIO_CNFG0 + offset;
	unsigned int mask = MAX77779_SGPIO_CNFG0_MODE_MASK;
	unsigned int val;

	if (offset >= gc->ngpio)
		return -EINVAL;

	val = MAX77779_SGPIO_CNFGx_MODE_INPUT
			<< MAX77779_SGPIO_CNFG0_MODE_SHIFT;
	return max77779_pmic_reg_update(core, reg, mask, val);
}

static int max77779_pmic_sgpio_direction_output(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	unsigned int reg = MAX77779_SGPIO_CNFG0 + offset;
	unsigned int mask = MAX77779_SGPIO_CNFG0_MODE_MASK |
			MAX77779_SGPIO_CNFG0_DATA_MASK;
	unsigned int val;

	if (offset >= gc->ngpio)
		return -EINVAL;

	val = (!!value) << MAX77779_SGPIO_CNFG0_DATA_SHIFT;
	val |= MAX77779_SGPIO_CNFGx_MODE_OUTPUT
			<< MAX77779_SGPIO_CNFG0_MODE_SHIFT;

	return max77779_pmic_reg_update(core, reg, mask, val);
}

static int max77779_pmic_sgpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	unsigned int reg = MAX77779_SGPIO_CNFG0 + offset;
	unsigned int val;
	int err;

	if (offset >= gc->ngpio)
		return -EINVAL;

	err = max77779_pmic_reg_read(core, reg, &val);
	if (err)
		return err;

	val = !!(val & MAX77779_SGPIO_CNFG0_DATA_MASK);
	return val;
}

static void max77779_pmic_sgpio_set(struct gpio_chip *gc,
		unsigned int offset, int value)
{
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);
	struct device *core = info->core;
	unsigned int reg = MAX77779_SGPIO_CNFG0 + offset;
	unsigned int mask = MAX77779_SGPIO_CNFG0_DATA_MASK;
	unsigned int val;

	if (offset >= gc->ngpio)
		return;

	val = !!value << MAX77779_SGPIO_CNFG0_DATA_SHIFT;
	max77779_pmic_reg_update(core, reg, mask, val);
}

static __maybe_unused
void max77779_pmic_sgpio_set_irq_valid_mask(struct gpio_chip *gc,
	unsigned long *valid_mask, unsigned int ngpios)
{
	bitmap_clear(valid_mask, 0, ngpios);
	bitmap_set(valid_mask, 0, ngpios);
}

static __maybe_unused
int max77779_pmic_sgpio_irq_init_hw(struct gpio_chip *gc)
{
	return 0;
}

static __maybe_unused
void max77779_pmic_sgpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	info->mask |= BIT(d->hwirq);
	info->mask_u |= BIT(d->hwirq);
}

static __maybe_unused
void max77779_pmic_sgpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	info->mask &= ~BIT(d->hwirq);
	info->mask_u |= BIT(d->hwirq);
}

static __maybe_unused
void max77779_pmic_sgpio_irq_disable(struct irq_data *d)
{
	max77779_pmic_sgpio_irq_mask(d);
}

static __maybe_unused
void max77779_pmic_sgpio_irq_enable(struct irq_data *d)
{
	max77779_pmic_sgpio_irq_unmask(d);
}

static __maybe_unused
int max77779_pmic_sgpio_set_irq_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	switch (type) {
	case IRQF_TRIGGER_NONE:
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_LOW:
		info->trig_type &= (0xf << (d->hwirq * 4));
		info->trig_type |= (type << (d->hwirq * 4));
		info->trig_type_u |= (1 << d->hwirq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void max77779_pmic_sgpio_bus_lock(struct irq_data *d)
{
	/* called in atomic context */
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	mutex_lock(&info->lock);
}

static void max77779_pmic_sgpio_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_pmic_sgpio_info *info = gpiochip_get_data(gc);

	/* TODO(jwylder): Once the Datasheet defines the meaning of the IRQ_SEL
	 * field.  map the trig and mask settings to appropriate field values
	 */

	mutex_unlock(&info->lock);
}

static struct irq_chip max77779_pmic_sgpio_irq_chip = {
	.name = "max77779_sgpio_irq",
	.irq_enable = max77779_pmic_sgpio_irq_enable,
	.irq_disable = max77779_pmic_sgpio_irq_disable,
	.irq_mask = max77779_pmic_sgpio_irq_mask,
	.irq_unmask = max77779_pmic_sgpio_irq_unmask,
	.irq_set_type = max77779_pmic_sgpio_set_irq_type,
	.irq_bus_lock = max77779_pmic_sgpio_bus_lock,
	.irq_bus_sync_unlock = max77779_pmic_sgpio_bus_sync_unlock
};

static int max77779_pmic_sgpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77779_pmic_sgpio_info *info;
	struct gpio_chip *gpio_chip;
	int err;
	u32 ngpios;

	if (!dev->of_node)
		return -ENODEV;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = dev;
	info->core = dev->parent;
	mutex_init(&info->lock);

	/* Setup GPIO controller */
	gpio_chip = &info->gpio_chip;

	gpio_chip->label = "max77779_sgpio";
	gpio_chip->parent = info->dev;
	gpio_chip->owner = THIS_MODULE;
	gpio_chip->get_direction = max77779_pmic_sgpio_get_direction;
	gpio_chip->direction_input = max77779_pmic_sgpio_direction_input;
	gpio_chip->direction_output = max77779_pmic_sgpio_direction_output;
	gpio_chip->get = max77779_pmic_sgpio_get;
	gpio_chip->set = max77779_pmic_sgpio_set;
	gpio_chip->request = gpiochip_generic_request;
	gpio_chip->set_config = gpiochip_generic_config;
	gpio_chip->base = -1;
	gpio_chip->can_sleep = true;
	gpio_chip->of_node = dev->of_node;
	err = of_property_read_u32(dev->of_node, "ngpios", &ngpios);
	if (err) {
		dev_err(dev, "Failed to get ngpios %d\n", err);
		return err;
	}
	gpio_chip->ngpio = ngpios;

	gpio_chip->irq.chip = &max77779_pmic_sgpio_irq_chip;
	gpio_chip->irq.default_type = IRQ_TYPE_NONE;
	gpio_chip->irq.handler = handle_simple_irq;
	gpio_chip->irq.parent_handler = NULL;
	gpio_chip->irq.num_parents = 0;
	gpio_chip->irq.parents = NULL;
	gpio_chip->irq.threaded = true;
	gpio_chip->irq.init_hw = max77779_pmic_sgpio_irq_init_hw;
	gpio_chip->irq.init_valid_mask = max77779_pmic_sgpio_set_irq_valid_mask;
	gpio_chip->irq.first = 0;

	err = devm_gpiochip_add_data(dev, gpio_chip, info);
	if (err) {
		dev_err(dev, "Failed to initialize gpio chip err = %d\n", err);
		return err;
	}

	return 0;
}

static int max77779_pmic_sgpio_remove(struct platform_device *pdev)
{
	return 0;
}
static const struct platform_device_id max77779_pmic_sgpio_id[] = {
	{ "max77779-pmic-sgpio", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77779_pmic_sgpio_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id max77779_pmic_sgpio_match_table[] = {
	{ .compatible = "max77779-pmic-sgpio",},
	{ },
};
#endif

static struct platform_driver max77779_pmic_sgpio_driver = {
	.probe = max77779_pmic_sgpio_probe,
	.remove = max77779_pmic_sgpio_remove,
	.id_table = max77779_pmic_sgpio_id,
	.driver = {
		.name = "max77779-pmic-sgpio",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = max77779_pmic_sgpio_match_table,
#endif
	},
};

module_platform_driver(max77779_pmic_sgpio_driver);

MODULE_DESCRIPTION("Maxim 77779 SGPIO driver");
MODULE_AUTHOR("James Wylder <jwylder@google.com>");
MODULE_LICENSE("GPL");
