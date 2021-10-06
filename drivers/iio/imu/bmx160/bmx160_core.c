// SPDX-License-Identifier: GPL-2.0
/*
 * BMX160 - Bosch IMU (accel, gyro plus external magnetometer)
 *
 * Copyright (c) 2016, Intel Corporation.
 * Copyright (c) 2019, Martin Kelly.
 *
 * IIO core driver for BMX160, with support for I2C/SPI busses
 *
 * TODO: magnetometer, hardware FIFO
 */
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>

#include "bmx160.h"

#define BMX160_REG_CHIP_ID	0x00
#define BMX160_CHIP_ID_VAL	0xD8

#define BMX160_REG_PMU_STATUS	0x03

/* X axis data low byte address, the rest can be obtained using axis offset */
#define BMX160_REG_DATA_MAGN_XOUT_L	0x04
#define BMX160_REG_DATA_GYRO_XOUT_L	0x0C
#define BMX160_REG_DATA_ACCEL_XOUT_L	0x12

#define BMX160_REG_ACCEL_CONFIG		0x40
#define BMX160_ACCEL_CONFIG_ODR_MASK	GENMASK(3, 0)
#define BMX160_ACCEL_CONFIG_BWP_MASK	GENMASK(6, 4)

#define BMX160_REG_ACCEL_RANGE		0x41
#define BMX160_ACCEL_RANGE_2G		0x03
#define BMX160_ACCEL_RANGE_4G		0x05
#define BMX160_ACCEL_RANGE_8G		0x08
#define BMX160_ACCEL_RANGE_16G		0x0C

#define BMX160_REG_GYRO_CONFIG		0x42
#define BMX160_GYRO_CONFIG_ODR_MASK	GENMASK(3, 0)
#define BMX160_GYRO_CONFIG_BWP_MASK	GENMASK(5, 4)

#define BMX160_REG_GYRO_RANGE		0x43
#define BMX160_GYRO_RANGE_2000DPS	0x00
#define BMX160_GYRO_RANGE_1000DPS	0x01
#define BMX160_GYRO_RANGE_500DPS	0x02
#define BMX160_GYRO_RANGE_250DPS	0x03
#define BMX160_GYRO_RANGE_125DPS	0x04

#define BMX160_REG_CMD			0x7E
#define BMX160_CMD_ACCEL_PM_SUSPEND	0x10
#define BMX160_CMD_ACCEL_PM_NORMAL	0x11
#define BMX160_CMD_ACCEL_PM_LOW_POWER	0x12
#define BMX160_CMD_GYRO_PM_SUSPEND	0x14
#define BMX160_CMD_GYRO_PM_NORMAL	0x15
#define BMX160_CMD_GYRO_PM_FAST_STARTUP	0x17
#define BMX160_CMD_SOFTRESET		0xB6

#define BMX160_REG_INT_EN		0x51
#define BMX160_DRDY_INT_EN		BIT(4)

#define BMX160_REG_INT_OUT_CTRL		0x53
#define BMX160_INT_OUT_CTRL_MASK	0x0f
#define BMX160_INT1_OUT_CTRL_SHIFT	0
#define BMX160_INT2_OUT_CTRL_SHIFT	4
#define BMX160_EDGE_TRIGGERED		BIT(0)
#define BMX160_ACTIVE_HIGH		BIT(1)
#define BMX160_OPEN_DRAIN		BIT(2)
#define BMX160_OUTPUT_EN		BIT(3)

#define BMX160_REG_INT_LATCH		0x54
#define BMX160_INT1_LATCH_MASK		BIT(4)
#define BMX160_INT2_LATCH_MASK		BIT(5)

/* INT1 and INT2 are in the opposite order as in INT_OUT_CTRL! */
#define BMX160_REG_INT_MAP		0x56
#define BMX160_INT1_MAP_DRDY_EN		0x80
#define BMX160_INT2_MAP_DRDY_EN		0x08

#define BMX160_REG_DUMMY		0x7F

#define BMX160_NORMAL_WRITE_USLEEP	2
#define BMX160_SUSPENDED_WRITE_USLEEP	450

#define BMX160_ACCEL_PMU_MIN_USLEEP	3800
#define BMX160_GYRO_PMU_MIN_USLEEP	80000
#define BMX160_SOFTRESET_USLEEP		1000

#define BMX160_CHANNEL(_type, _axis, _index) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
	.ext_info = bmx160_ext_info,				\
}

/* scan indexes follow DATA register order */
enum bmx160_scan_axis {
	BMX160_SCAN_EXT_MAGN_X = 0,
	BMX160_SCAN_EXT_MAGN_Y,
	BMX160_SCAN_EXT_MAGN_Z,
	BMX160_SCAN_RHALL,
	BMX160_SCAN_GYRO_X,
	BMX160_SCAN_GYRO_Y,
	BMX160_SCAN_GYRO_Z,
	BMX160_SCAN_ACCEL_X,
	BMX160_SCAN_ACCEL_Y,
	BMX160_SCAN_ACCEL_Z,
	BMX160_SCAN_TIMESTAMP,
};

enum bmx160_sensor_type {
	BMX160_ACCEL	= 0,
	BMX160_GYRO,
	BMX160_EXT_MAGN,
	BMX160_NUM_SENSORS /* must be last */
};

enum bmx160_int_pin {
	BMX160_PIN_INT1,
	BMX160_PIN_INT2
};

const struct regmap_config bmx160_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};
EXPORT_SYMBOL(bmx160_regmap_config);

struct bmx160_regs {
	u8 data; /* LSB byte register for X-axis */
	u8 config;
	u8 config_odr_mask;
	u8 config_bwp_mask;
	u8 range;
	u8 pmu_cmd_normal;
	u8 pmu_cmd_suspend;
};

static struct bmx160_regs bmx160_regs[] = {
	[BMX160_ACCEL] = {
		.data	= BMX160_REG_DATA_ACCEL_XOUT_L,
		.config	= BMX160_REG_ACCEL_CONFIG,
		.config_odr_mask = BMX160_ACCEL_CONFIG_ODR_MASK,
		.config_bwp_mask = BMX160_ACCEL_CONFIG_BWP_MASK,
		.range	= BMX160_REG_ACCEL_RANGE,
		.pmu_cmd_normal = BMX160_CMD_ACCEL_PM_NORMAL,
		.pmu_cmd_suspend = BMX160_CMD_ACCEL_PM_SUSPEND,
	},
	[BMX160_GYRO] = {
		.data	= BMX160_REG_DATA_GYRO_XOUT_L,
		.config	= BMX160_REG_GYRO_CONFIG,
		.config_odr_mask = BMX160_GYRO_CONFIG_ODR_MASK,
		.config_bwp_mask = BMX160_GYRO_CONFIG_BWP_MASK,
		.range	= BMX160_REG_GYRO_RANGE,
		.pmu_cmd_normal = BMX160_CMD_GYRO_PM_NORMAL,
		.pmu_cmd_suspend = BMX160_CMD_GYRO_PM_SUSPEND,
	},
};

static unsigned long bmx160_pmu_time[] = {
	[BMX160_ACCEL] = BMX160_ACCEL_PMU_MIN_USLEEP,
	[BMX160_GYRO] = BMX160_GYRO_PMU_MIN_USLEEP,
};

struct bmx160_scale {
	u8 bits;
	int uscale;
};

struct bmx160_odr {
	u8 bits;
	int odr;
	int uodr;
};

static const struct bmx160_scale bmx160_accel_scale[] = {
	{ BMX160_ACCEL_RANGE_2G, 598},
	{ BMX160_ACCEL_RANGE_4G, 1197},
	{ BMX160_ACCEL_RANGE_8G, 2394},
	{ BMX160_ACCEL_RANGE_16G, 4788},
};

static const struct bmx160_scale bmx160_gyro_scale[] = {
	{ BMX160_GYRO_RANGE_2000DPS, 1065},
	{ BMX160_GYRO_RANGE_1000DPS, 532},
	{ BMX160_GYRO_RANGE_500DPS, 266},
	{ BMX160_GYRO_RANGE_250DPS, 133},
	{ BMX160_GYRO_RANGE_125DPS, 66},
};

struct bmx160_scale_item {
	const struct bmx160_scale *tbl;
	int num;
};

static const struct  bmx160_scale_item bmx160_scale_table[] = {
	[BMX160_ACCEL] = {
		.tbl	= bmx160_accel_scale,
		.num	= ARRAY_SIZE(bmx160_accel_scale),
	},
	[BMX160_GYRO] = {
		.tbl	= bmx160_gyro_scale,
		.num	= ARRAY_SIZE(bmx160_gyro_scale),
	},
};

static const struct bmx160_odr bmx160_accel_odr[] = {
	{0x01, 0, 781250},
	{0x02, 1, 562500},
	{0x03, 3, 125000},
	{0x04, 6, 250000},
	{0x05, 12, 500000},
	{0x06, 25, 0},
	{0x07, 50, 0},
	{0x08, 100, 0},
	{0x09, 200, 0},
	{0x0A, 400, 0},
	{0x0B, 800, 0},
	{0x0C, 1600, 0},
};

static const struct bmx160_odr bmx160_gyro_odr[] = {
	{0x06, 25, 0},
	{0x07, 50, 0},
	{0x08, 100, 0},
	{0x09, 200, 0},
	{0x0A, 400, 0},
	{0x0B, 800, 0},
	{0x0C, 1600, 0},
	{0x0D, 3200, 0},
};

struct bmx160_odr_item {
	const struct bmx160_odr *tbl;
	int num;
};

static const struct  bmx160_odr_item bmx160_odr_table[] = {
	[BMX160_ACCEL] = {
		.tbl	= bmx160_accel_odr,
		.num	= ARRAY_SIZE(bmx160_accel_odr),
	},
	[BMX160_GYRO] = {
		.tbl	= bmx160_gyro_odr,
		.num	= ARRAY_SIZE(bmx160_gyro_odr),
	},
};

static const struct iio_mount_matrix *
bmx160_get_mount_matrix(const struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan)
{
	struct bmx160_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info bmx160_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, bmx160_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec bmx160_channels[] = {
	BMX160_CHANNEL(IIO_ACCEL, X, BMX160_SCAN_ACCEL_X),
	BMX160_CHANNEL(IIO_ACCEL, Y, BMX160_SCAN_ACCEL_Y),
	BMX160_CHANNEL(IIO_ACCEL, Z, BMX160_SCAN_ACCEL_Z),
	BMX160_CHANNEL(IIO_ANGL_VEL, X, BMX160_SCAN_GYRO_X),
	BMX160_CHANNEL(IIO_ANGL_VEL, Y, BMX160_SCAN_GYRO_Y),
	BMX160_CHANNEL(IIO_ANGL_VEL, Z, BMX160_SCAN_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(BMX160_SCAN_TIMESTAMP),
};

static enum bmx160_sensor_type bmx160_to_sensor(enum iio_chan_type iio_type)
{
	switch (iio_type) {
	case IIO_ACCEL:
		return BMX160_ACCEL;
	case IIO_ANGL_VEL:
		return BMX160_GYRO;
	default:
		return -EINVAL;
	}
}

static
int bmx160_set_mode(struct bmx160_data *data, enum bmx160_sensor_type t,
		    bool mode)
{
	int ret;
	u8 cmd;

	if (mode)
		cmd = bmx160_regs[t].pmu_cmd_normal;
	else
		cmd = bmx160_regs[t].pmu_cmd_suspend;

	ret = regmap_write(data->regmap, BMX160_REG_CMD, cmd);
	if (ret)
		return ret;

	usleep_range(bmx160_pmu_time[t], bmx160_pmu_time[t] + 1000);

	return 0;
}

static
int bmx160_set_scale(struct bmx160_data *data, enum bmx160_sensor_type t,
		     int uscale)
{
	int i;

	for (i = 0; i < bmx160_scale_table[t].num; i++)
		if (bmx160_scale_table[t].tbl[i].uscale == uscale)
			break;

	if (i == bmx160_scale_table[t].num)
		return -EINVAL;

	return regmap_write(data->regmap, bmx160_regs[t].range,
			    bmx160_scale_table[t].tbl[i].bits);
}

static
int bmx160_get_scale(struct bmx160_data *data, enum bmx160_sensor_type t,
		     int *uscale)
{
	int i, ret, val;

	ret = regmap_read(data->regmap, bmx160_regs[t].range, &val);
	if (ret)
		return ret;

	for (i = 0; i < bmx160_scale_table[t].num; i++)
		if (bmx160_scale_table[t].tbl[i].bits == val) {
			*uscale = bmx160_scale_table[t].tbl[i].uscale;
			return 0;
		}

	return -EINVAL;
}

static int bmx160_get_data(struct bmx160_data *data, int chan_type,
			   int axis, int *val)
{
	u8 reg;
	int ret;
	__le16 sample;
	enum bmx160_sensor_type t = bmx160_to_sensor(chan_type);

	reg = bmx160_regs[t].data + (axis - IIO_MOD_X) * sizeof(sample);

	ret = regmap_bulk_read(data->regmap, reg, &sample, sizeof(sample));
	if (ret)
		return ret;

	*val = sign_extend32(le16_to_cpu(sample), 15);

	return 0;
}

static
int bmx160_set_odr(struct bmx160_data *data, enum bmx160_sensor_type t,
		   int odr, int uodr)
{
	int i;

	for (i = 0; i < bmx160_odr_table[t].num; i++)
		if (bmx160_odr_table[t].tbl[i].odr == odr &&
		    bmx160_odr_table[t].tbl[i].uodr == uodr)
			break;

	if (i >= bmx160_odr_table[t].num)
		return -EINVAL;

	return regmap_update_bits(data->regmap,
				  bmx160_regs[t].config,
				  bmx160_regs[t].config_odr_mask,
				  bmx160_odr_table[t].tbl[i].bits);
}

static int bmx160_get_odr(struct bmx160_data *data, enum bmx160_sensor_type t,
			  int *odr, int *uodr)
{
	int i, val, ret;

	ret = regmap_read(data->regmap, bmx160_regs[t].config, &val);
	if (ret)
		return ret;

	val &= bmx160_regs[t].config_odr_mask;

	for (i = 0; i < bmx160_odr_table[t].num; i++)
		if (val == bmx160_odr_table[t].tbl[i].bits)
			break;

	if (i >= bmx160_odr_table[t].num)
		return -EINVAL;

	*odr = bmx160_odr_table[t].tbl[i].odr;
	*uodr = bmx160_odr_table[t].tbl[i].uodr;

	return 0;
}

static irqreturn_t bmx160_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct bmx160_data *data = iio_priv(indio_dev);
	int i, ret, j = 0, base = BMX160_REG_DATA_MAGN_XOUT_L;
	__le16 sample;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap, base + i * sizeof(sample),
				       &sample, sizeof(sample));
		if (ret)
			goto done;
		data->buf[j++] = sample;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data->buf, pf->timestamp);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int bmx160_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret;
	struct bmx160_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = bmx160_get_data(data, chan->type, chan->channel2, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		ret = bmx160_get_scale(data,
				       bmx160_to_sensor(chan->type), val2);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = bmx160_get_odr(data, bmx160_to_sensor(chan->type),
				     val, val2);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bmx160_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct bmx160_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return bmx160_set_scale(data,
					bmx160_to_sensor(chan->type), val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return bmx160_set_odr(data, bmx160_to_sensor(chan->type),
				      val, val2);
	default:
		return -EINVAL;
	}

	return 0;
}

static
IIO_CONST_ATTR(in_accel_sampling_frequency_available,
	       "0.78125 1.5625 3.125 6.25 12.5 25 50 100 200 400 800 1600");
static
IIO_CONST_ATTR(in_anglvel_sampling_frequency_available,
	       "25 50 100 200 400 800 1600 3200");
static
IIO_CONST_ATTR(in_accel_scale_available,
	       "0.000598 0.001197 0.002394 0.004788");
static
IIO_CONST_ATTR(in_anglvel_scale_available,
	       "0.001065 0.000532 0.000266 0.000133 0.000066");

static struct attribute *bmx160_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group bmx160_attrs_group = {
	.attrs = bmx160_attrs,
};

static const struct iio_info bmx160_info = {
	.read_raw = bmx160_read_raw,
	.write_raw = bmx160_write_raw,
	.attrs = &bmx160_attrs_group,
};

static const char *bmx160_match_acpi_device(struct device *dev)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return NULL;

	return dev_name(dev);
}

static int bmx160_write_conf_reg(struct regmap *regmap, unsigned int reg,
				 unsigned int mask, unsigned int bits,
				 unsigned int write_usleep)
{
	int ret;
	unsigned int val;

	ret = regmap_read(regmap, reg, &val);
	if (ret)
		return ret;

	val = (val & ~mask) | bits;

	ret = regmap_write(regmap, reg, val);
	if (ret)
		return ret;

	/*
	 * We need to wait after writing before we can write again. See the
	 * datasheet, page 93.
	 */
	usleep_range(write_usleep, write_usleep + 1000);

	return 0;
}

static int bmx160_config_pin(struct regmap *regmap, enum bmx160_int_pin pin,
			     bool open_drain, u8 irq_mask,
			     unsigned long write_usleep)
{
	int ret;
	struct device *dev = regmap_get_device(regmap);
	u8 int_out_ctrl_shift;
	u8 int_latch_mask;
	u8 int_map_mask;
	u8 int_out_ctrl_mask;
	u8 int_out_ctrl_bits;
	const char *pin_name;

	switch (pin) {
	case BMX160_PIN_INT1:
		int_out_ctrl_shift = BMX160_INT1_OUT_CTRL_SHIFT;
		int_latch_mask = BMX160_INT1_LATCH_MASK;
		int_map_mask = BMX160_INT1_MAP_DRDY_EN;
		break;
	case BMX160_PIN_INT2:
		int_out_ctrl_shift = BMX160_INT2_OUT_CTRL_SHIFT;
		int_latch_mask = BMX160_INT2_LATCH_MASK;
		int_map_mask = BMX160_INT2_MAP_DRDY_EN;
		break;
	}
	int_out_ctrl_mask = BMX160_INT_OUT_CTRL_MASK << int_out_ctrl_shift;

	/*
	 * Enable the requested pin with the right settings:
	 * - Push-pull/open-drain
	 * - Active low/high
	 * - Edge/level triggered
	 */
	int_out_ctrl_bits = BMX160_OUTPUT_EN;
	if (open_drain)
		/* Default is push-pull. */
		int_out_ctrl_bits |= BMX160_OPEN_DRAIN;
	int_out_ctrl_bits |= irq_mask;
	int_out_ctrl_bits <<= int_out_ctrl_shift;

	ret = bmx160_write_conf_reg(regmap, BMX160_REG_INT_OUT_CTRL,
				    int_out_ctrl_mask, int_out_ctrl_bits,
				    write_usleep);
	if (ret)
		return ret;

	/* Set the pin to input mode with no latching. */
	ret = bmx160_write_conf_reg(regmap, BMX160_REG_INT_LATCH,
				    int_latch_mask, int_latch_mask,
				    write_usleep);
	if (ret)
		return ret;

	/* Map interrupts to the requested pin. */
	ret = bmx160_write_conf_reg(regmap, BMX160_REG_INT_MAP,
				    int_map_mask, int_map_mask,
				    write_usleep);
	if (ret) {
		switch (pin) {
		case BMX160_PIN_INT1:
			pin_name = "INT1";
			break;
		case BMX160_PIN_INT2:
			pin_name = "INT2";
			break;
		}
		dev_err(dev, "Failed to configure %s IRQ pin", pin_name);
	}

	return ret;
}

int bmx160_enable_irq(struct regmap *regmap, bool enable)
{
	unsigned int enable_bit = 0;

	if (enable)
		enable_bit = BMX160_DRDY_INT_EN;

	return bmx160_write_conf_reg(regmap, BMX160_REG_INT_EN,
				     BMX160_DRDY_INT_EN, enable_bit,
				     BMX160_NORMAL_WRITE_USLEEP);
}
EXPORT_SYMBOL(bmx160_enable_irq);

static int bmx160_get_irq(struct device_node *of_node, enum bmx160_int_pin *pin)
{
	int irq;

	/* Use INT1 if possible, otherwise fall back to INT2. */
	irq = of_irq_get_byname(of_node, "INT1");
	if (irq > 0) {
		*pin = BMX160_PIN_INT1;
		return irq;
	}

	irq = of_irq_get_byname(of_node, "INT2");
	if (irq > 0)
		*pin = BMX160_PIN_INT2;

	return irq;
}

static int bmx160_config_device_irq(struct iio_dev *indio_dev, int irq_type,
				    enum bmx160_int_pin pin)
{
	bool open_drain;
	u8 irq_mask;
	struct bmx160_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);

	/* Level-triggered, active-low is the default if we set all zeroes. */
	if (irq_type == IRQF_TRIGGER_RISING)
		irq_mask = BMX160_ACTIVE_HIGH | BMX160_EDGE_TRIGGERED;
	else if (irq_type == IRQF_TRIGGER_FALLING)
		irq_mask = BMX160_EDGE_TRIGGERED;
	else if (irq_type == IRQF_TRIGGER_HIGH)
		irq_mask = BMX160_ACTIVE_HIGH;
	else if (irq_type == IRQF_TRIGGER_LOW)
		irq_mask = 0;
	else {
		dev_err(&indio_dev->dev,
			"Invalid interrupt type 0x%x specified\n", irq_type);
		return -EINVAL;
	}

	open_drain = of_property_read_bool(dev->of_node, "drive-open-drain");

	return bmx160_config_pin(data->regmap, pin, open_drain, irq_mask,
				 BMX160_NORMAL_WRITE_USLEEP);
}

static int bmx160_setup_irq(struct iio_dev *indio_dev, int irq,
			    enum bmx160_int_pin pin)
{
	struct irq_data *desc;
	u32 irq_type;
	int ret;

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(&indio_dev->dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);

	ret = bmx160_config_device_irq(indio_dev, irq_type, pin);
	if (ret)
		return ret;

	return bmx160_probe_trigger(indio_dev, irq, irq_type);
}

static int bmx160_chip_init(struct bmx160_data *data, bool use_spi)
{
	int ret;
	unsigned int val;
	struct device *dev = regmap_get_device(data->regmap);

	ret = regulator_bulk_enable(ARRAY_SIZE(data->supplies), data->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = regmap_write(data->regmap, BMX160_REG_CMD, BMX160_CMD_SOFTRESET);
	if (ret)
		return ret;

	usleep_range(BMX160_SOFTRESET_USLEEP, BMX160_SOFTRESET_USLEEP + 1);

	/*
	 * CS rising edge is needed before starting SPI, so do a dummy read
	 * See Section 3.2.1, page 86 of the datasheet
	 */
	if (use_spi) {
		ret = regmap_read(data->regmap, BMX160_REG_DUMMY, &val);
		if (ret)
			return ret;
	}

	ret = regmap_read(data->regmap, BMX160_REG_CHIP_ID, &val);
	if (ret) {
		dev_err(dev, "Error reading chip id\n");
		return ret;
	}
	if (val != BMX160_CHIP_ID_VAL) {
		dev_err(dev, "Wrong chip id, got %x expected %x\n",
			val, BMX160_CHIP_ID_VAL);
		return -ENODEV;
	}

	ret = bmx160_set_mode(data, BMX160_ACCEL, true);
	if (ret)
		return ret;

	ret = bmx160_set_mode(data, BMX160_GYRO, true);
	if (ret)
		return ret;

	return 0;
}

static int bmx160_data_rdy_trigger_set_state(struct iio_trigger *trig,
					     bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct bmx160_data *data = iio_priv(indio_dev);

	return bmx160_enable_irq(data->regmap, enable);
}

static const struct iio_trigger_ops bmx160_trigger_ops = {
	.set_trigger_state = &bmx160_data_rdy_trigger_set_state,
};

int bmx160_probe_trigger(struct iio_dev *indio_dev, int irq, u32 irq_type)
{
	struct bmx160_data *data = iio_priv(indio_dev);
	int ret;

	data->trig = devm_iio_trigger_alloc(&indio_dev->dev, "%s-dev%d",
					    indio_dev->name, indio_dev->id);

	if (data->trig == NULL)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, irq,
			       &iio_trigger_generic_data_rdy_poll,
			       irq_type, "bmx160", data->trig);
	if (ret)
		return ret;

	data->trig->dev.parent = regmap_get_device(data->regmap);
	data->trig->ops = &bmx160_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);

	return 0;
}

static void bmx160_chip_uninit(void *data)
{
	struct bmx160_data *bmx_data = data;
	struct device *dev = regmap_get_device(bmx_data->regmap);
	int ret;

	bmx160_set_mode(bmx_data, BMX160_GYRO, false);
	bmx160_set_mode(bmx_data, BMX160_ACCEL, false);

	ret = regulator_bulk_disable(ARRAY_SIZE(bmx_data->supplies),
				     bmx_data->supplies);
	if (ret)
		dev_err(dev, "Failed to disable regulators: %d\n", ret);
}

int bmx160_core_probe(struct device *dev, struct regmap *regmap,
		      const char *name, bool use_spi)
{
	struct iio_dev *indio_dev;
	struct bmx160_data *data;
	int irq;
	enum bmx160_int_pin int_pin;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;

	data->supplies[0].supply = "vdd";
	data->supplies[1].supply = "vddio";
	ret = devm_regulator_bulk_get(dev,
				      ARRAY_SIZE(data->supplies),
				      data->supplies);
	if (ret) {
		dev_err(dev, "Failed to get regulators: %d\n", ret);
		return ret;
	}

	ret = iio_read_mount_matrix(dev, "mount-matrix",
				    &data->orientation);
	if (ret)
		return ret;

	ret = bmx160_chip_init(data, use_spi);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, bmx160_chip_uninit, data);
	if (ret)
		return ret;

	if (!name && ACPI_HANDLE(dev))
		name = bmx160_match_acpi_device(dev);

	indio_dev->channels = bmx160_channels;
	indio_dev->num_channels = ARRAY_SIZE(bmx160_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bmx160_info;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      bmx160_trigger_handler, NULL);
	if (ret)
		return ret;

	irq = bmx160_get_irq(dev->of_node, &int_pin);
	if (irq > 0) {
		ret = bmx160_setup_irq(indio_dev, irq, int_pin);
		if (ret)
			dev_err(&indio_dev->dev, "Failed to setup IRQ %d\n",
				irq);
	} else {
		dev_info(&indio_dev->dev, "Not setting up IRQ trigger\n");
	}

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(bmx160_core_probe);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_DESCRIPTION("Bosch BMX160 driver");
MODULE_LICENSE("GPL v2");
