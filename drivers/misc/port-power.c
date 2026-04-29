// SPDX-License-Identifier: GPL-2.0+
/*
 * Port power control driver for 74HC595 shift register GPIO expander
 *
 * Controls 16 port power outputs via a GPIO expander (typically two
 * daisy-chained 74HC595 shift registers on a bit-banged SPI bus).
 *
 * Provides:
 *   - sysfs "enable_mask" for userspace control
 *   - Exported kernel API (port_power_trip, etc.)
 *     for use by the fast-fuse overcurrent protection driver
 *
 * DTS node provides:
 *   - 16 port-power GPIOs (active-high, from 74HC595 expander)
 *
 * Copyright (C) 2025 TechnipFMC Schilling Robotics
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/bitmap.h>
#include <linux/port_power.h>

struct port_power {
	struct device		*dev;
	struct gpio_descs	*gpios;

	spinlock_t		lock;
	u16			power_mask;
};

/* Global instance — only one port-power controller per system */
static struct port_power *global_pp;

/* ---------- Exported kernel API ---------- */

bool port_power_available(void)
{
	return READ_ONCE(global_pp) != NULL;
}
EXPORT_SYMBOL_GPL(port_power_available);

void port_power_trip(int port)
{
	struct port_power *pp = READ_ONCE(global_pp);

	if (!pp || port < 0 || port >= PORT_POWER_MAX_PORTS)
		return;

	spin_lock(&pp->lock);
	pp->power_mask &= ~BIT(port);
	spin_unlock(&pp->lock);

	gpiod_set_value_cansleep(pp->gpios->desc[port], 0);
}
EXPORT_SYMBOL_GPL(port_power_trip);

/* ---------- sysfs interface ---------- */

static ssize_t enable_mask_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct port_power *pp = dev_get_drvdata(dev);
	u16 mask;
	unsigned long flags;

	spin_lock_irqsave(&pp->lock, flags);
	mask = pp->power_mask;
	spin_unlock_irqrestore(&pp->lock, flags);

	return sysfs_emit(buf, "0x%04x\n", mask);
}

static ssize_t enable_mask_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct port_power *pp = dev_get_drvdata(dev);
	DECLARE_BITMAP(values, PORT_POWER_MAX_PORTS);
	unsigned long flags;
	u16 mask;
	int ret, i;

	ret = kstrtou16(buf, 0, &mask);
	if (ret)
		return ret;

	spin_lock_irqsave(&pp->lock, flags);
	pp->power_mask = mask;
	spin_unlock_irqrestore(&pp->lock, flags);

	values[0] = mask;

	if (pp->gpios->ndescs == PORT_POWER_MAX_PORTS) {
		gpiod_set_array_value_cansleep(pp->gpios->ndescs,
					       pp->gpios->desc,
					       pp->gpios->info,
					       values);
	} else {
		for (i = 0; i < pp->gpios->ndescs; i++)
			gpiod_set_value_cansleep(pp->gpios->desc[i],
						 !!(mask & BIT(i)));
	}

	return count;
}
static DEVICE_ATTR_RW(enable_mask);

static struct attribute *port_power_attrs[] = {
	&dev_attr_enable_mask.attr,
	NULL,
};
ATTRIBUTE_GROUPS(port_power);

/* ---------- platform driver ---------- */

static int port_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct port_power *pp;

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (!pp)
		return -ENOMEM;

	pp->dev = dev;
	spin_lock_init(&pp->lock);

	pp->gpios = devm_gpiod_get_array(dev, "port-power", GPIOD_OUT_LOW);
	if (IS_ERR(pp->gpios))
		return dev_err_probe(dev, PTR_ERR(pp->gpios),
				     "failed to get port-power GPIOs\n");

	if (pp->gpios->ndescs != PORT_POWER_MAX_PORTS)
		return dev_err_probe(dev, -EINVAL,
				     "expected %d port-power GPIOs, got %d\n",
				     PORT_POWER_MAX_PORTS,
				     pp->gpios->ndescs);

	platform_set_drvdata(pdev, pp);
	WRITE_ONCE(global_pp, pp);

	dev_info(dev, "port power control ready (%d ports)\n",
		 PORT_POWER_MAX_PORTS);

	return 0;
}

static void port_power_remove(struct platform_device *pdev)
{
	WRITE_ONCE(global_pp, NULL);
}

static const struct of_device_id port_power_of_match[] = {
	{ .compatible = "schilling,port-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, port_power_of_match);

static struct platform_driver port_power_driver = {
	.probe = port_power_probe,
	.remove = port_power_remove,
	.driver = {
		.name = "port-power",
		.of_match_table = port_power_of_match,
		.dev_groups = port_power_groups,
	},
};
module_platform_driver(port_power_driver);

MODULE_AUTHOR("TechnipFMC Schilling Robotics");
MODULE_DESCRIPTION("Port power control driver for 74HC595 GPIO expander");
MODULE_LICENSE("GPL");  // TODO(Trevor): Update the license everywhere if needed
