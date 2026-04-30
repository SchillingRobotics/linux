// SPDX-License-Identifier: GPL-2.0+
/*
 * Fast fuse hardware overcurrent protection driver
 *
 * Uses external comparators + priority encoder to detect overcurrent
 * and cut port power via the port-power driver.
 *
 * DTS node provides:
 *   - IRQ from priority encoder output (active-low, level-triggered)
 *   - 4 address GPIOs encoding the faulted port (MSB-first)
 *   - settle-time-ms: encoder settle delay after cutting power
 *   - holdoff-time-us: IRQ holdoff when no valid port trips
 *
 * Copyright (C) 2025 TechnipFMC Schilling Robotics
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/irqdesc.h>
#include <linux/port_power.h>

#define FAST_FUSE_MAX_PORTS	16
#define FAST_FUSE_ADDR_BITS	4

struct fast_fuse {
	struct device		*dev;
	int			irq;

	/* GPIO descriptors */
	struct gpio_descs	*addr_gpios;	/* 4-bit address from encoder */

	/* Configuration */
	u32			settle_time_ms;
	u32			holdoff_time_us;

	/* State (protected by lock) */
	spinlock_t		lock;
	u16			enable_mask;	/* ports enabled for fuse */
	u16			fault_mask;	/* ports that have faulted */

	/* Port address read in hardirq, consumed by thread */
	int			pending_port;
};

/*
 * Read the 4-bit port address from the priority encoder.
 * Address GPIOs are SoC MMIO pins, safe in hardirq context.
 */
static int fast_fuse_read_addr(struct fast_fuse *ff)
{
	int addr = 0;
	int i, val;

	for (i = 0; i < ff->addr_gpios->ndescs; i++) {
		val = gpiod_get_value(ff->addr_gpios->desc[i]);
		if (val < 0)
			return val;
		addr = (addr << 1) | val;
	}

	return addr;
}

static irqreturn_t fast_fuse_hardirq(int irq, void *data)
{
	struct fast_fuse *ff = data;
	int addr;

	addr = fast_fuse_read_addr(ff);
	WRITE_ONCE(ff->pending_port, addr);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t fast_fuse_thread(int irq, void *data)
{
	struct fast_fuse *ff = data;
	int port;
	unsigned long flags;
	bool tripped = false;

	port = READ_ONCE(ff->pending_port);
	if (port < 0 || port >= FAST_FUSE_MAX_PORTS)
		goto holdoff;

	spin_lock_irqsave(&ff->lock, flags);
	if (ff->enable_mask & BIT(port)) {
		/*
		 * Always record fault when hardware signals overcurrent,
		 * regardless of fault_mask state. Defense in depth:
		 * don't let a stale fault bit prevent a real trip.
		 */
		if (!(ff->fault_mask & BIT(port))) {
			ff->fault_mask |= BIT(port);
			tripped = true;
		}
	}
	spin_unlock_irqrestore(&ff->lock, flags);

	if (tripped) {
		/* Cut port power via port-power driver */
		port_power_trip(port);
		dev_warn(ff->dev, "fast fuse tripped on port %d\n", port + 1);

		// TODO(Trevor): Verify that this is the actual behavior we want
		//               The current implementation 
		/* Wait for priority encoder to settle after removing load */
		msleep(ff->settle_time_ms);

		/* IRQF_ONESHOT unmasks on return; re-enters if line still low */
		return IRQ_HANDLED;
	}

holdoff:
	/* No valid/enabled port — brief holdoff to prevent IRQ storm */
	usleep_range(ff->holdoff_time_us, ff->holdoff_time_us + 100);
	return IRQ_HANDLED;
}

/* ---------- sysfs interface ---------- */

static ssize_t enable_mask_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fast_fuse *ff = dev_get_drvdata(dev);
	u16 mask;
	unsigned long flags;

	spin_lock_irqsave(&ff->lock, flags);
	mask = ff->enable_mask;
	spin_unlock_irqrestore(&ff->lock, flags);

	return sysfs_emit(buf, "0x%04x\n", mask);
}

static ssize_t enable_mask_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fast_fuse *ff = dev_get_drvdata(dev);
	u16 mask;
	unsigned long flags;
	int ret;

	ret = kstrtou16(buf, 0, &mask);
	if (ret)
		return ret;

	spin_lock_irqsave(&ff->lock, flags);
	ff->enable_mask = mask;
	spin_unlock_irqrestore(&ff->lock, flags);

	return count;
}
static DEVICE_ATTR_RW(enable_mask);

static ssize_t fault_mask_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fast_fuse *ff = dev_get_drvdata(dev);
	u16 mask;
	unsigned long flags;

	spin_lock_irqsave(&ff->lock, flags);
	mask = ff->fault_mask;
	spin_unlock_irqrestore(&ff->lock, flags);

	return sysfs_emit(buf, "0x%04x\n", mask);
}

static ssize_t fault_mask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fast_fuse *ff = dev_get_drvdata(dev);
	u16 mask;
	unsigned long flags;
	int ret;

	ret = kstrtou16(buf, 0, &mask);
	if (ret)
		return ret;

	spin_lock_irqsave(&ff->lock, flags);
	ff->fault_mask &= ~mask;
	spin_unlock_irqrestore(&ff->lock, flags);

	return count;
}
static DEVICE_ATTR_RW(fault_mask);

static struct attribute *fast_fuse_attrs[] = {
	&dev_attr_enable_mask.attr,
	&dev_attr_fault_mask.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fast_fuse);

/* ---------- platform driver ---------- */

static int fast_fuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fast_fuse *ff;
	int ret;

	ff = devm_kzalloc(dev, sizeof(*ff), GFP_KERNEL);
	if (!ff)
		return -ENOMEM;

	ff->dev = dev;
	spin_lock_init(&ff->lock);
	ff->pending_port = -1;

	/* 4-bit port address from priority encoder */
	ff->addr_gpios = devm_gpiod_get_array(dev, "addr", GPIOD_IN);
	if (IS_ERR(ff->addr_gpios))
		return dev_err_probe(dev, PTR_ERR(ff->addr_gpios),
				     "failed to get addr GPIOs\n");

	if (ff->addr_gpios->ndescs != FAST_FUSE_ADDR_BITS)
		return dev_err_probe(dev, -EINVAL,
				     "expected %d addr GPIOs, got %d\n",
				     FAST_FUSE_ADDR_BITS,
				     ff->addr_gpios->ndescs);

	/* Verify port-power driver is available */
	if (!port_power_available())
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "port-power driver not ready\n");

	/* Timing from DT (with defaults matching old firmware) */
	of_property_read_u32(dev->of_node, "settle-time-ms",
			     &ff->settle_time_ms);
	of_property_read_u32(dev->of_node, "holdoff-time-us",
			     &ff->holdoff_time_us);
	if (!ff->settle_time_ms)
		ff->settle_time_ms = 1;
	if (!ff->holdoff_time_us)
		ff->holdoff_time_us = 300;

	/* IRQ from GPIO interrupt controller */
	ff->irq = platform_get_irq(pdev, 0);
	if (ff->irq < 0)
		return ff->irq;

	ret = devm_request_threaded_irq(dev, ff->irq,
					fast_fuse_hardirq,
					fast_fuse_thread,
					IRQF_ONESHOT,
					"fast-fuse", ff);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/* Elevate IRQ thread to SCHED_FIFO so port power is cut with
	 * minimal latency — the hardware comparators only signal the
	 * fault, port_power_trip() is what actually removes power.
	 */
	{
		struct irq_desc *desc = irq_to_desc(ff->irq);

		if (desc && desc->action && desc->action->thread)
			sched_set_fifo(desc->action->thread);
		else
			dev_warn(dev, "could not elevate IRQ thread priority\n");
	}

	platform_set_drvdata(pdev, ff);

	dev_info(dev, "fast fuse ready (settle=%ums holdoff=%uus)\n",
		 ff->settle_time_ms, ff->holdoff_time_us);

	return 0;
}

static const struct of_device_id fast_fuse_of_match[] = {
	{ .compatible = "schilling,fast-fuse" },
	{ }
};
MODULE_DEVICE_TABLE(of, fast_fuse_of_match);

static struct platform_driver fast_fuse_driver = {
	.probe = fast_fuse_probe,
	.driver = {
		.name = "fast-fuse",
		.of_match_table = fast_fuse_of_match,
		.dev_groups = fast_fuse_groups,
	},
};
module_platform_driver(fast_fuse_driver);

MODULE_AUTHOR("TechnipFMC Schilling Robotics");
MODULE_DESCRIPTION("Hardware fast fuse overcurrent protection driver");
MODULE_LICENSE("GPL");
