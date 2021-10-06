// SPDX-License-Identifier: GPL-2.0
/*
 * BMX160 - Bosch IMU, I2C bits
 *
 * Copyright (c) 2016, Intel Corporation.
 *
 * 7-bit I2C slave address is:
 *      - 0x68 if SDO is pulled to GND
 *      - 0x69 if SDO is pulled to VDDIO
 */
#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "bmx160.h"

static int bmx160_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const char *name = NULL;

	regmap = devm_regmap_init_i2c(client, &bmx160_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap: %pe\n",
			regmap);
		return PTR_ERR(regmap);
	}

	if (id)
		name = id->name;

	return bmx160_core_probe(&client->dev, regmap, name, false);
}

static const struct i2c_device_id bmx160_i2c_id[] = {
	{"bmx160", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bmx160_i2c_id);

static const struct acpi_device_id bmx160_acpi_match[] = {
	{"BMX0160", 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, bmx160_acpi_match);

#ifdef CONFIG_OF
static const struct of_device_id bmx160_of_match[] = {
	{ .compatible = "bosch,bmx160" },
	{ },
};
MODULE_DEVICE_TABLE(of, bmx160_of_match);
#endif

static struct i2c_driver bmx160_i2c_driver = {
	.driver = {
		.name			= "bmx160_i2c",
		.acpi_match_table	= ACPI_PTR(bmx160_acpi_match),
		.of_match_table		= of_match_ptr(bmx160_of_match),
	},
	.probe		= bmx160_i2c_probe,
	.id_table	= bmx160_i2c_id,
};
module_i2c_driver(bmx160_i2c_driver);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_DESCRIPTION("BMX160 I2C driver");
MODULE_LICENSE("GPL v2");
