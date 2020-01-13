// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2020 Loys Ollivier <loys.ollivier@gmail.com>
*/

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include "inv_icm_iio.h"

static const struct regmap_config inv_icm_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

/**
 *  inv_icm_probe() - probe function.
 *  @client:          i2c client.
 *  @id:              i2c device id.
 *
 *  Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int result;
	enum inv_devices chip_type;
	struct regmap *regmap;
	const char *name;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EOPNOTSUPP;

	if (client->dev.of_node) {
		chip_type = (enum inv_devices)
			of_device_get_match_data(&client->dev);
		name = client->name;
	} else if (id) {
		chip_type = (enum inv_devices)
			id->driver_data;
		name = id->name;
	} else {
		return -ENOSYS;
	}

	regmap = devm_regmap_init_i2c(client, &inv_icm_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	result = inv_icm_core_probe(regmap, client->irq, name,
				    chip_type);
	if (result < 0)
		return result;

	return 0;
}

static int inv_icm_remove(struct i2c_client *client)
{
	return 0;
}

/*
 * device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_icm_id[] = {
	{"icm20948", INV_ICM20948},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_icm_id);

static const struct of_device_id inv_of_match[] = {
	{
		.compatible = "invensense,icm20948",
		.data = (void *)INV_ICM20948
	},
	{ }
};
MODULE_DEVICE_TABLE(of, inv_of_match);

static struct i2c_driver inv_icm_driver = {
	.probe		=	inv_icm_probe,
	.remove		=	inv_icm_remove,
	.id_table	=	inv_icm_id,
	.driver = {
		.of_match_table = inv_of_match,
		.name	=	"inv-icm-i2c",
	},
};

module_i2c_driver(inv_icm_driver);

// TODO fix this
MODULE_AUTHOR("Loys Ollivier");
MODULE_DESCRIPTION("Invensense device ICM20948 driver");
MODULE_LICENSE("GPL");
