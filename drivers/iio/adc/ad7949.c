// SPDX-License-Identifier: GPL-2.0
/* ad7949.c - Analog Devices ADC driver 14/16 bits 4/8 channels
 *
 * Copyright (C) 2018 CMC NV
 *
 * https://www.analog.com/media/en/technical-documentation/data-sheets/AD7949.pdf
 *
 * Software fuse extension: optional fast kernel-side polling + GPIO trip.
 * Configured via DTS properties: trip-gpios, fuse-threshold, fuse-poll-hz.
 * Sysfs attrs: fuse_enable, fuse_threshold, fuse_poll_hz, fuse_fault_mask,
 *              fuse_fault_clear, fuse_consec_count.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/bitfield.h>

#define AD7949_CFG_MASK_TOTAL		GENMASK(13, 0)

/* CFG: Configuration Update */
#define AD7949_CFG_MASK_OVERWRITE	BIT(13)

/* INCC: Input Channel Configuration */
#define AD7949_CFG_MASK_INCC		GENMASK(12, 10)
#define AD7949_CFG_VAL_INCC_UNIPOLAR_GND	7
#define AD7949_CFG_VAL_INCC_UNIPOLAR_COMM	6
#define AD7949_CFG_VAL_INCC_UNIPOLAR_DIFF	4
#define AD7949_CFG_VAL_INCC_TEMP		3
#define AD7949_CFG_VAL_INCC_BIPOLAR		2
#define AD7949_CFG_VAL_INCC_BIPOLAR_DIFF	0

/* INX: Input channel Selection in a binary fashion */
#define AD7949_CFG_MASK_INX		GENMASK(9, 7)

/* BW: select bandwidth for low-pass filter. Full or Quarter */
#define AD7949_CFG_MASK_BW_FULL		BIT(6)

/* REF: reference/buffer selection */
#define AD7949_CFG_MASK_REF		GENMASK(5, 3)
#define AD7949_CFG_VAL_REF_EXT_TEMP_BUF		3
#define AD7949_CFG_VAL_REF_EXT_TEMP		2
#define AD7949_CFG_VAL_REF_INT_4096		1
#define AD7949_CFG_VAL_REF_INT_2500		0
#define AD7949_CFG_VAL_REF_EXTERNAL		BIT(1)

/* SEQ: channel sequencer. Allows for scanning channels */
#define AD7949_CFG_MASK_SEQ		GENMASK(2, 1)

/* RB: Read back the CFG register */
#define AD7949_CFG_MASK_RBN		BIT(0)

enum {
	ID_AD7949 = 0,
	ID_AD7682,
	ID_AD7689,
};

struct ad7949_adc_spec {
	u8 num_channels;
	u8 resolution;
};

static const struct ad7949_adc_spec ad7949_adc_spec[] = {
	[ID_AD7949] = { .num_channels = 8, .resolution = 14 },
	[ID_AD7682] = { .num_channels = 4, .resolution = 16 },
	[ID_AD7689] = { .num_channels = 8, .resolution = 16 },
};

/**
 * struct ad7949_adc_chip - AD ADC chip
 * @lock: protects write sequences
 * @vref: regulator generating Vref
 * @indio_dev: reference to iio structure
 * @spi: reference to spi structure
 * @refsel: reference selection
 * @resolution: resolution of the chip
 * @cfg: copy of the configuration register
 * @current_channel: current channel in use
 * @buffer: buffer to send / receive data to / from device
 * @buf8b: be16 buffer to exchange data with the device in 8-bit transfers
 * @num_channels: number of ADC channels
 * @fuse_gpios: optional trip GPIOs (one per channel)
 * @fuse_threshold: ADC raw count above which a channel trips
 * @fuse_poll_hz: polling rate in Hz for the fuse kthread
 * @fuse_consec_count: consecutive over-threshold readings required to trip
 * @fuse_enable: whether the fuse polling thread is active
 * @fuse_fault_mask: bitmask of tripped channels
 * @fuse_thread: kthread for polling
 * @fuse_last_raw: last raw ADC reading per channel (for sysfs)
 */
struct ad7949_adc_chip {
	struct mutex lock;
	struct regulator *vref;
	struct iio_dev *indio_dev;
	struct spi_device *spi;
	u32 refsel;
	u8 resolution;
	u16 cfg;
	unsigned int current_channel;
	u16 buffer __aligned(IIO_DMA_MINALIGN);
	__be16 buf8b;
	u8 num_channels;

	/* Software fuse fields */
	struct gpio_descs *fuse_gpios;
	u32 fuse_threshold;
	u32 fuse_poll_hz;
	u32 fuse_consec_count;
	bool fuse_enable;
	u32 fuse_fault_mask;
	u32 port_power_mask;  /* bitmask: 1=ON, 0=OFF. Written by userspace + fuse */
	struct task_struct *fuse_thread;
	u16 fuse_last_raw[8];
};

static int ad7949_spi_write_cfg(struct ad7949_adc_chip *ad7949_adc, u16 val,
				u16 mask)
{
	int ret;

	ad7949_adc->cfg = (val & mask) | (ad7949_adc->cfg & ~mask);

	switch (ad7949_adc->spi->bits_per_word) {
	case 16:
		ad7949_adc->buffer = ad7949_adc->cfg << 2;
		ret = spi_write(ad7949_adc->spi, &ad7949_adc->buffer, 2);
		break;
	case 14:
		ad7949_adc->buffer = ad7949_adc->cfg;
		ret = spi_write(ad7949_adc->spi, &ad7949_adc->buffer, 2);
		break;
	case 8:
		/* Here, type is big endian as it must be sent in two transfers */
		ad7949_adc->buf8b = cpu_to_be16(ad7949_adc->cfg << 2);
		ret = spi_write(ad7949_adc->spi, &ad7949_adc->buf8b, 2);
		break;
	default:
		dev_err(&ad7949_adc->indio_dev->dev, "unsupported BPW\n");
		return -EINVAL;
	}

	/*
	 * This delay is to avoid a new request before the required time to
	 * send a new command to the device
	 */
	udelay(2);
	return ret;
}

static int ad7949_spi_read_channel(struct ad7949_adc_chip *ad7949_adc, int *val,
				   unsigned int channel)
{
	int ret;
	int i;

	/*
	 * 1: write CFG for sample N and read old data (sample N-2)
	 * 2: if CFG was not changed since sample N-1 then we'll get good data
	 *    at the next xfer, so we bail out now, otherwise we write something
	 *    and we read garbage (sample N-1 configuration).
	 */
	for (i = 0; i < 2; i++) {
		ret = ad7949_spi_write_cfg(ad7949_adc,
					   FIELD_PREP(AD7949_CFG_MASK_INX, channel),
					   AD7949_CFG_MASK_INX);
		if (ret)
			return ret;
		if (channel == ad7949_adc->current_channel)
			break;
	}

	/* 3: write something and read actual data */
	if (ad7949_adc->spi->bits_per_word == 8)
		ret = spi_read(ad7949_adc->spi, &ad7949_adc->buf8b, 2);
	else
		ret = spi_read(ad7949_adc->spi, &ad7949_adc->buffer, 2);

	if (ret)
		return ret;

	/*
	 * This delay is to avoid a new request before the required time to
	 * send a new command to the device
	 */
	udelay(2);

	ad7949_adc->current_channel = channel;

	switch (ad7949_adc->spi->bits_per_word) {
	case 16:
		*val = ad7949_adc->buffer;
		/* Shift-out padding bits */
		*val >>= 16 - ad7949_adc->resolution;
		break;
	case 14:
		*val = ad7949_adc->buffer & GENMASK(13, 0);
		break;
	case 8:
		/* Here, type is big endian as data was sent in two transfers */
		*val = be16_to_cpu(ad7949_adc->buf8b);
		/* Shift-out padding bits */
		*val >>= 16 - ad7949_adc->resolution;
		break;
	default:
		dev_err(&ad7949_adc->indio_dev->dev, "unsupported BPW\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * Scan all channels using a batched spi_message with cs_change between
 * transfers. Each transfer sends the config for the next channel and reads
 * the result of the previous conversion (AD7689 pipeline). This minimises
 * SPI framework overhead: one mutex lock, one message, N+2 transfers.
 *
 * results[] must have room for ad7949_adc->num_channels entries.
 * Caller must hold ad7949_adc->lock.
 */
static int ad7949_scan_all_channels(struct ad7949_adc_chip *ad7949_adc,
				    u16 *results)
{
	int nch = ad7949_adc->num_channels;
	/* N+2 transfers: 2 primes + N reads (AD7689 pipeline is 2 deep) */
	int nxfers = nch + 2;
	struct spi_transfer *xfers;
	u16 *tx_bufs, *rx_bufs;
	struct spi_message msg;
	int shift = 16 - ad7949_adc->resolution;
	u16 base_cfg;
	int i, ret;

	xfers = kcalloc(nxfers, sizeof(*xfers), GFP_KERNEL);

	xfers = kcalloc(nxfers, sizeof(*xfers), GFP_KERNEL);
	if (!xfers)
		return -ENOMEM;

	tx_bufs = kcalloc(nxfers, sizeof(u16), GFP_KERNEL);
	rx_bufs = kcalloc(nxfers, sizeof(u16), GFP_KERNEL);
	if (!tx_bufs || !rx_bufs) {
		ret = -ENOMEM;
		goto out;
	}

	/* Build base config matching current settings, with CFG overwrite set */
	base_cfg = ad7949_adc->cfg | AD7949_CFG_MASK_OVERWRITE;

	spi_message_init(&msg);

	for (i = 0; i < nxfers; i++) {
		int target_ch;

		/*
		 * AD7689 pipeline is 2 deep: the result read during xfer K
		 * is from the CFG written during xfer K-2.  We need N+2
		 * transfers total: N transfers that advance through channels
		 * 0..N-1 plus 2 trailing repeats to flush the pipeline.
		 *
		 *   xfer[0]   → CFG ch0,     rx garbage
		 *   xfer[1]   → CFG ch1,     rx garbage
		 *   xfer[2]   → CFG ch2,     rx ch0
		 *   xfer[k]   → CFG ch(k),   rx ch(k-2)    for k < N
		 *   xfer[N]   → CFG ch(N-1), rx ch(N-2)
		 *   xfer[N+1] → CFG ch(N-1), rx ch(N-1)
		 */
		if (i < nch)
			target_ch = i;
		else
			target_ch = nch - 1;

		tx_bufs[i] = (base_cfg & ~AD7949_CFG_MASK_INX) |
			     FIELD_PREP(AD7949_CFG_MASK_INX, target_ch);

		switch (ad7949_adc->spi->bits_per_word) {
		case 16:
			tx_bufs[i] <<= 2;
			break;
		case 14:
			break;
		case 8:
			tx_bufs[i] = cpu_to_be16(tx_bufs[i] << 2);
			break;
		}

		xfers[i].tx_buf = &tx_bufs[i];
		xfers[i].rx_buf = &rx_bufs[i];
		xfers[i].len = 2;
		xfers[i].cs_change = 1; /* deassert CS between transfers */
		xfers[i].cs_change_delay.value = 4;
		xfers[i].cs_change_delay.unit = SPI_DELAY_UNIT_USECS;

		spi_message_add_tail(&xfers[i], &msg);
	}

	/* Last transfer: don't deassert CS needlessly */
	xfers[nxfers - 1].cs_change = 0;

	ret = spi_sync(ad7949_adc->spi, &msg);
	if (ret)
		goto out;

	/*
	 * Pipeline: xfers[0..1] are prime (discard), xfers[2..N+1] have ch0..ch(N-1)
	 */
	for (i = 0; i < nch; i++) {
		u16 raw = rx_bufs[i + 2];

		switch (ad7949_adc->spi->bits_per_word) {
		case 16:
			raw >>= shift;
			break;
		case 14:
			raw &= GENMASK(13, 0);
			break;
		case 8:
			raw = be16_to_cpu(raw);
			raw >>= shift;
			break;
		}
		results[i] = raw;
	}

	/* Update current_channel so single-channel reads stay efficient */
	ad7949_adc->current_channel = nch - 1;

out:
	kfree(rx_bufs);
	kfree(tx_bufs);
	kfree(xfers);
	return ret;
}

/*
 * Software fuse kthread: polls all channels, trips GPIOs on overcurrent.
 */
static int ad7949_fuse_thread_fn(void *data)
{
	struct ad7949_adc_chip *ad7949_adc = data;
	u16 results[8];
	u8 consec[8] = {};
	int nch = ad7949_adc->num_channels;
	struct gpio_descs *gpios = ad7949_adc->fuse_gpios;
	unsigned long sleep_us;
	bool tripped = false;
	int i, ret;

	dev_info(&ad7949_adc->spi->dev,
		 "fuse thread started: threshold=%u poll_hz=%u consec=%u\n",
		 ad7949_adc->fuse_threshold,
		 ad7949_adc->fuse_poll_hz,
		 ad7949_adc->fuse_consec_count);

	while (!kthread_should_stop()) {
		if (!ad7949_adc->fuse_enable) {
			msleep(100);
			continue;
		}

		sleep_us = ad7949_adc->fuse_poll_hz ?
			   1000000UL / ad7949_adc->fuse_poll_hz : 100000;

		mutex_lock(&ad7949_adc->lock);
		ret = ad7949_scan_all_channels(ad7949_adc, results);
		mutex_unlock(&ad7949_adc->lock);

		if (ret) {
			usleep_range(sleep_us, sleep_us + sleep_us / 10);
			continue;
		}

		for (i = 0; i < nch && i < 8; i++) {
			ad7949_adc->fuse_last_raw[i] = results[i];

			/* Skip already-tripped channels */
			if (ad7949_adc->fuse_fault_mask & BIT(i))
				continue;

			if (results[i] > ad7949_adc->fuse_threshold) {
				consec[i]++;
				if (consec[i] >= ad7949_adc->fuse_consec_count) {
					/* Trip: turn off port power */
					if (gpios && i < gpios->ndescs) {
						ad7949_adc->fuse_fault_mask |= BIT(i);
						ad7949_adc->port_power_mask &= ~BIT(i);
						tripped = true;
						dev_warn(&ad7949_adc->spi->dev,
							 "fuse trip ch%d raw=%u thresh=%u\n",
							 i, results[i],
							 ad7949_adc->fuse_threshold);
					}
					consec[i] = 0;
				}
			} else {
				consec[i] = 0;
			}
		}

		/* Apply all GPIO changes at once after processing all channels */
		if (tripped && gpios) {
			DECLARE_BITMAP(values, 8);
			values[0] = ad7949_adc->port_power_mask;
			gpiod_set_array_value_cansleep(gpios->ndescs,
						       gpios->desc,
						       gpios->info,
						       values);
			tripped = false;
		}

		usleep_range(sleep_us, sleep_us + sleep_us / 10);
	}

	return 0;
}

#define AD7949_ADC_CHANNEL(chan) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
}

static const struct iio_chan_spec ad7949_adc_channels[] = {
	AD7949_ADC_CHANNEL(0),
	AD7949_ADC_CHANNEL(1),
	AD7949_ADC_CHANNEL(2),
	AD7949_ADC_CHANNEL(3),
	AD7949_ADC_CHANNEL(4),
	AD7949_ADC_CHANNEL(5),
	AD7949_ADC_CHANNEL(6),
	AD7949_ADC_CHANNEL(7),
};

static int ad7949_spi_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	int ret;

	if (!val)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&ad7949_adc->lock);
		ret = ad7949_spi_read_channel(ad7949_adc, val, chan->channel);
		mutex_unlock(&ad7949_adc->lock);

		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (ad7949_adc->refsel) {
		case AD7949_CFG_VAL_REF_INT_2500:
			*val = 2500;
			break;
		case AD7949_CFG_VAL_REF_INT_4096:
			*val = 4096;
			break;
		case AD7949_CFG_VAL_REF_EXT_TEMP:
		case AD7949_CFG_VAL_REF_EXT_TEMP_BUF:
			ret = regulator_get_voltage(ad7949_adc->vref);
			if (ret < 0)
				return ret;

			/* convert value back to mV */
			*val = ret / 1000;
			break;
		}

		*val2 = (1 << ad7949_adc->resolution) - 1;
		return IIO_VAL_FRACTIONAL;
	}

	return -EINVAL;
}

static int ad7949_spi_reg_access(struct iio_dev *indio_dev,
			unsigned int reg, unsigned int writeval,
			unsigned int *readval)
{
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	int ret = 0;

	if (readval)
		*readval = ad7949_adc->cfg;
	else
		ret = ad7949_spi_write_cfg(ad7949_adc, writeval,
					   AD7949_CFG_MASK_TOTAL);

	return ret;
}

static const struct attribute_group ad7949_fuse_attr_group;

static const struct iio_info ad7949_spi_info = {
	.read_raw = ad7949_spi_read_raw,
	.debugfs_reg_access = ad7949_spi_reg_access,
	.attrs = &ad7949_fuse_attr_group,
};

/* ---- Software fuse sysfs attributes ---- */

static ssize_t fuse_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", ad7949_adc->fuse_enable ? 1 : 0);
}

static ssize_t fuse_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	ad7949_adc->fuse_enable = val;
	return len;
}
static DEVICE_ATTR_RW(fuse_enable);

static ssize_t fuse_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", ad7949_adc->fuse_threshold);
}

static ssize_t fuse_threshold_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	u32 val;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	ad7949_adc->fuse_threshold = val;
	return len;
}
static DEVICE_ATTR_RW(fuse_threshold);

static ssize_t fuse_poll_hz_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", ad7949_adc->fuse_poll_hz);
}

static ssize_t fuse_poll_hz_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	u32 val;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	if (val > 20000)
		val = 20000; /* cap at 20kHz */

	ad7949_adc->fuse_poll_hz = val;
	return len;
}
static DEVICE_ATTR_RW(fuse_poll_hz);

static ssize_t fuse_consec_count_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", ad7949_adc->fuse_consec_count);
}

static ssize_t fuse_consec_count_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	u32 val;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;
	if (val < 1)
		val = 1;

	ad7949_adc->fuse_consec_count = val;
	return len;
}
static DEVICE_ATTR_RW(fuse_consec_count);

static ssize_t fuse_fault_mask_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "0x%02x\n", ad7949_adc->fuse_fault_mask);
}
static DEVICE_ATTR_RO(fuse_fault_mask);

static ssize_t fuse_fault_clear_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	u32 mask;

	if (kstrtou32(buf, 0, &mask))
		return -EINVAL;

	ad7949_adc->fuse_fault_mask &= ~mask;
	return len;
}
static DEVICE_ATTR_WO(fuse_fault_clear);

static ssize_t fuse_last_raw_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	int i, n = 0;

	for (i = 0; i < ad7949_adc->num_channels && i < 8; i++)
		n += sysfs_emit_at(buf, n, "%u%s", ad7949_adc->fuse_last_raw[i],
				   i < ad7949_adc->num_channels - 1 ? " " : "\n");
	return n;
}
static DEVICE_ATTR_RO(fuse_last_raw);

static ssize_t port_power_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "0x%02x\n", ad7949_adc->port_power_mask);
}

static ssize_t port_power_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);
	struct gpio_descs *gpios = ad7949_adc->fuse_gpios;
	u32 new_mask;
	int i;

	if (kstrtou32(buf, 0, &new_mask))
		return -EINVAL;

	if (!gpios)
		return -ENODEV;

	for (i = 0; i < gpios->ndescs && i < 8; i++) {
		int want = (new_mask >> i) & 1;
		int had  = (ad7949_adc->port_power_mask >> i) & 1;

		if (want != had)
			gpiod_set_value_cansleep(gpios->desc[i], want);
	}

	ad7949_adc->port_power_mask = new_mask;
	return len;
}
static DEVICE_ATTR_RW(port_power);

static struct attribute *ad7949_fuse_attrs[] = {
	&dev_attr_fuse_enable.attr,
	&dev_attr_fuse_threshold.attr,
	&dev_attr_fuse_poll_hz.attr,
	&dev_attr_fuse_consec_count.attr,
	&dev_attr_fuse_fault_mask.attr,
	&dev_attr_fuse_fault_clear.attr,
	&dev_attr_fuse_last_raw.attr,
	&dev_attr_port_power.attr,
	NULL,
};

static umode_t ad7949_fuse_attrs_visible(struct kobject *kobj,
					  struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	/* Only show fuse attrs if trip-gpios were specified in DTS */
	if (!ad7949_adc->fuse_gpios)
		return 0;
	return attr->mode;
}

static const struct attribute_group ad7949_fuse_attr_group = {
	.attrs = ad7949_fuse_attrs,
	.is_visible = ad7949_fuse_attrs_visible,
};

static int ad7949_spi_init(struct ad7949_adc_chip *ad7949_adc)
{
	int ret;
	int val;
	u16 cfg;

	ad7949_adc->current_channel = 0;

	cfg = FIELD_PREP(AD7949_CFG_MASK_OVERWRITE, 1) |
		FIELD_PREP(AD7949_CFG_MASK_INCC, AD7949_CFG_VAL_INCC_UNIPOLAR_GND) |
		FIELD_PREP(AD7949_CFG_MASK_INX, ad7949_adc->current_channel) |
		FIELD_PREP(AD7949_CFG_MASK_BW_FULL, 1) |
		FIELD_PREP(AD7949_CFG_MASK_REF, ad7949_adc->refsel) |
		FIELD_PREP(AD7949_CFG_MASK_SEQ, 0x0) |
		FIELD_PREP(AD7949_CFG_MASK_RBN, 1);

	ret = ad7949_spi_write_cfg(ad7949_adc, cfg, AD7949_CFG_MASK_TOTAL);

	/*
	 * Do two dummy conversions to apply the first configuration setting.
	 * Required only after the start up of the device.
	 */
	ad7949_spi_read_channel(ad7949_adc, &val, ad7949_adc->current_channel);
	ad7949_spi_read_channel(ad7949_adc, &val, ad7949_adc->current_channel);

	return ret;
}

static void ad7949_disable_reg(void *reg)
{
	regulator_disable(reg);
}

static int ad7949_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct ad7949_adc_spec *spec;
	struct ad7949_adc_chip *ad7949_adc;
	struct iio_dev *indio_dev;
	u32 tmp;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*ad7949_adc));
	if (!indio_dev) {
		dev_err(dev, "can not allocate iio device\n");
		return -ENOMEM;
	}

	indio_dev->info = &ad7949_spi_info;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad7949_adc_channels;
	spi_set_drvdata(spi, indio_dev);

	ad7949_adc = iio_priv(indio_dev);
	ad7949_adc->indio_dev = indio_dev;
	ad7949_adc->spi = spi;

	spec = &ad7949_adc_spec[spi_get_device_id(spi)->driver_data];
	indio_dev->num_channels = spec->num_channels;
	ad7949_adc->num_channels = spec->num_channels;
	ad7949_adc->resolution = spec->resolution;

	/* Set SPI bits per word */
	if (spi_is_bpw_supported(spi, ad7949_adc->resolution)) {
		spi->bits_per_word = ad7949_adc->resolution;
	} else if (spi_is_bpw_supported(spi, 16)) {
		spi->bits_per_word = 16;
	} else if (spi_is_bpw_supported(spi, 8)) {
		spi->bits_per_word = 8;
	} else {
		dev_err(dev, "unable to find common BPW with spi controller\n");
		return -EINVAL;
	}

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "Error in SPI setup\n");
		return ret;
	}

	/* Setup internal voltage reference */
	tmp = 4096000;
	device_property_read_u32(dev, "adi,internal-ref-microvolt", &tmp);

	switch (tmp) {
	case 2500000:
		ad7949_adc->refsel = AD7949_CFG_VAL_REF_INT_2500;
		break;
	case 4096000:
		ad7949_adc->refsel = AD7949_CFG_VAL_REF_INT_4096;
		break;
	default:
		dev_err(dev, "unsupported internal voltage reference\n");
		return -EINVAL;
	}

	/* Setup external voltage reference, buffered? */
	ad7949_adc->vref = devm_regulator_get_optional(dev, "vrefin");
	if (IS_ERR(ad7949_adc->vref)) {
		ret = PTR_ERR(ad7949_adc->vref);
		if (ret != -ENODEV)
			return ret;
		/* unbuffered? */
		ad7949_adc->vref = devm_regulator_get_optional(dev, "vref");
		if (IS_ERR(ad7949_adc->vref)) {
			ret = PTR_ERR(ad7949_adc->vref);
			if (ret != -ENODEV)
				return ret;
		} else {
			ad7949_adc->refsel = AD7949_CFG_VAL_REF_EXT_TEMP;
		}
	} else {
		ad7949_adc->refsel = AD7949_CFG_VAL_REF_EXT_TEMP_BUF;
	}

	if (ad7949_adc->refsel & AD7949_CFG_VAL_REF_EXTERNAL) {
		ret = regulator_enable(ad7949_adc->vref);
		if (ret < 0) {
			dev_err(dev, "fail to enable regulator\n");
			return ret;
		}

		ret = devm_add_action_or_reset(dev, ad7949_disable_reg,
					       ad7949_adc->vref);
		if (ret)
			return ret;
	}

	mutex_init(&ad7949_adc->lock);

	ret = ad7949_spi_init(ad7949_adc);
	if (ret) {
		dev_err(dev, "fail to init this device: %d\n", ret);
		return ret;
	}

	/* Software fuse: optional DTS-driven fast polling + GPIO trip */
	ad7949_adc->fuse_gpios = devm_gpiod_get_array_optional(dev, "trip",
								GPIOD_OUT_LOW);
	if (IS_ERR(ad7949_adc->fuse_gpios)) {
		ret = PTR_ERR(ad7949_adc->fuse_gpios);
		dev_info(dev, "trip-gpios error: %d\n", ret);
		ad7949_adc->fuse_gpios = NULL;
		if (ret != -ENOENT) {
			dev_err(dev, "failed to get trip-gpios: %d\n", ret);
			return ret;
		}
	} else if (ad7949_adc->fuse_gpios) {
		dev_info(dev, "trip-gpios: got %d descriptors\n",
			 ad7949_adc->fuse_gpios->ndescs);
	} else {
		dev_info(dev, "trip-gpios: property not found (optional, skipping)\n");
	}

	// TODO(Trevor): Update the default fuse parameters (threshold, poll rate, consecutive count) based on measured values and desired trip behavior
	//               For now, these are just placeholders that can be overridden via DTS or sysfs.
	if (ad7949_adc->fuse_gpios) {
		ad7949_adc->fuse_threshold = 2000;
		device_property_read_u32(dev, "fuse-threshold",
					 &ad7949_adc->fuse_threshold);

		ad7949_adc->fuse_poll_hz = 2000;
		device_property_read_u32(dev, "fuse-poll-hz",
					 &ad7949_adc->fuse_poll_hz);

		ad7949_adc->fuse_consec_count = 3;
		device_property_read_u32(dev, "fuse-consec-count",
					 &ad7949_adc->fuse_consec_count);
		if (ad7949_adc->fuse_consec_count < 1)
			ad7949_adc->fuse_consec_count = 1;

		ad7949_adc->fuse_enable = true;
		ad7949_adc->fuse_fault_mask = 0;
		ad7949_adc->port_power_mask = 0;

		ad7949_adc->fuse_thread = kthread_run(
			ad7949_fuse_thread_fn, ad7949_adc,
			"ad7949-fuse-%s", dev_name(dev));
		if (IS_ERR(ad7949_adc->fuse_thread)) {
			ret = PTR_ERR(ad7949_adc->fuse_thread);
			ad7949_adc->fuse_thread = NULL;
			dev_err(dev, "failed to start fuse thread: %d\n", ret);
			return ret;
		}

		dev_info(dev, "software fuse: %d gpios, threshold=%u, poll=%uHz, consec=%u\n",
			 ad7949_adc->fuse_gpios->ndescs,
			 ad7949_adc->fuse_threshold,
			 ad7949_adc->fuse_poll_hz,
			 ad7949_adc->fuse_consec_count);
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		dev_err(dev, "fail to register iio device: %d\n", ret);

	return ret;
}

static void ad7949_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7949_adc_chip *ad7949_adc = iio_priv(indio_dev);

	if (ad7949_adc->fuse_thread)
		kthread_stop(ad7949_adc->fuse_thread);
}

static const struct of_device_id ad7949_spi_of_id[] = {
	{ .compatible = "adi,ad7949" },
	{ .compatible = "adi,ad7682" },
	{ .compatible = "adi,ad7689" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad7949_spi_of_id);

static const struct spi_device_id ad7949_spi_id[] = {
	{ "ad7949", ID_AD7949  },
	{ "ad7682", ID_AD7682 },
	{ "ad7689", ID_AD7689 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad7949_spi_id);

static struct spi_driver ad7949_spi_driver = {
	.driver = {
		.name		= "ad7949",
		.of_match_table	= ad7949_spi_of_id,
	},
	.probe	  = ad7949_spi_probe,
	.remove	  = ad7949_spi_remove,
	.id_table = ad7949_spi_id,
};
module_spi_driver(ad7949_spi_driver);

MODULE_AUTHOR("Charles-Antoine Couret <charles-antoine.couret@essensium.com>");
MODULE_DESCRIPTION("Analog Devices 14/16-bit 8-channel ADC driver");
MODULE_LICENSE("GPL v2");
