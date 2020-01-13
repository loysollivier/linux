// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2012 Invensense, Inc.
*/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include "inv_icm_iio.h"

static const struct inv_icm_reg_map reg_set_20948 = {
	.sample_rate_div	= INV_ICM20948_REG_SAMPLE_RATE_DIV,
	.lpf                    = INV_ICM20948_REG_CONFIG,
	.user_ctrl              = INV_ICM20948_REG_USER_CTRL,
	.fifo_en                = INV_ICM20948_REG_FIFO_EN_1, // TODO edited
	.gyro_config            = INV_ICM20948_REG_GYRO_CONFIG,
	.accl_config            = INV_ICM20948_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_ICM20948_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_ICM20948_REG_FIFO_R_W,
	.raw_gyro               = INV_ICM20948_REG_RAW_GYRO,
	.raw_accl               = INV_ICM20948_REG_RAW_ACCEL,
	.temperature            = INV_ICM20948_REG_TEMPERATURE,
	.int_enable             = INV_ICM20948_REG_INT_ENABLE_1, // TODO edited
	.pwr_mgmt_1             = INV_ICM20948_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_ICM20948_REG_PWR_MGMT_2,
	.int_pin_cfg		= INV_ICM20948_REG_INT_PIN_CFG,
	.accl_offset		= INV_ICM20948_REG_ACCEL_OFFSET,
	.gyro_offset		= INV_ICM20948_REG_GYRO_OFFSET,
	.i2c_if                 = 0,
};

static const struct inv_icm_chip_config chip_config_20948 = {
	.fsr = INV_ICM20948_FSR_2000DPS,
	.lpf = INV_ICM20948_FILTER_20HZ,
	.divider = INV_ICM20948_FIFO_RATE_TO_DIVIDER(INV_ICM20948_INIT_FIFO_RATE),
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.magn_fifo_enable = false,
	.accl_fs = INV_ICM20948_FS_02G,
	.user_ctrl = 0,
};

/* Indexed by enum inv_devices */
static const struct inv_icm_hw hw_info[] = {
	{
		.whoami = INV_ICM20948_WHOAMI_VALUE,
		.name = "ICM20948",
		.reg = &reg_set_20948,
		.config = &chip_config_20948,
		.fifo_size = 512,
	},
};

/**
 *  inv_check_and_setup_chip() - check and setup chip.
 */
static int inv_check_and_setup_chip(struct inv_icm_state *st)
{
	int result;
	unsigned int regval;
	int i;

	st->hw  = &hw_info[st->chip_type];
	st->reg = hw_info[st->chip_type].reg;

	/* check chip self-identification */
	result = regmap_read(st->map, INV_ICM20948_REG_WHOAMI, &regval);
	if (result)
		return result;
	if (regval != st->hw->whoami) {
		/* check whoami against all possible values */
		for (i = 0; i < INV_NUM_PARTS; ++i) {
			if (regval == hw_info[i].whoami) {
				dev_warn(regmap_get_device(st->map),
					"whoami mismatch got %#02x (%s)"
					"expected %#02hhx (%s)\n",
					regval, hw_info[i].name,
					st->hw->whoami, st->hw->name);
				break;
			}
		}
		if (i >= INV_NUM_PARTS) {
			dev_err(regmap_get_device(st->map),
				"invalid whoami %#02x expected %#02hhx (%s)\n",
				regval, st->hw->whoami, st->hw->name);
			return -ENODEV;
		}
	}
	dev_warn(regmap_get_device(st->map), "ICM Found whoami %#02x", regval);

	/* reset to make sure previous state are not there */
	result = regmap_write(st->map, st->reg->pwr_mgmt_1,
			      INV_ICM20948_BIT_H_RESET);
	if (result)
		return result;
	msleep(INV_ICM20948_POWER_UP_TIME);
	// TODO function has been truncated
	return 0;
}

int inv_icm_core_probe(struct regmap *regmap, int irq, const char *name,
		int chip_type)
{
	struct inv_icm_state *st;
	struct iio_dev *indio_dev;
	struct inv_icm_platform_data *pdata;
	struct device *dev = regmap_get_device(regmap);
	int result;
	struct irq_data *desc;
	int irq_type;

	dev_warn(dev, "ICM probed\n");
	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	BUILD_BUG_ON(ARRAY_SIZE(hw_info) != INV_NUM_PARTS);
	if (chip_type < 0 || chip_type >= INV_NUM_PARTS) {
		dev_err(dev, "Bad invensense chip_type=%d name=%s\n",
				chip_type, name);
		return -ENODEV;
	}
	st = iio_priv(indio_dev);
	mutex_init(&st->lock);
	st->chip_type = chip_type;
	st->powerup_count = 0;
	st->irq = irq;
	st->map = regmap;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		result = iio_read_mount_matrix(dev, "mount-matrix",
					       &st->orientation);
		if (result) {
			dev_err(dev, "Failed to retrieve mounting matrix %d\n",
				result);
			return result;
		}
	}

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_RISING;
	if (irq_type == IRQF_TRIGGER_RISING)
		st->irq_mask = INV_ICM20948_ACTIVE_HIGH;
	else if (irq_type == IRQF_TRIGGER_FALLING)
		st->irq_mask = INV_ICM20948_ACTIVE_LOW;
	else if (irq_type == IRQF_TRIGGER_HIGH)
		st->irq_mask = INV_ICM20948_ACTIVE_HIGH |
			INV_ICM20948_LATCH_INT_EN;
	else if (irq_type == IRQF_TRIGGER_LOW)
		st->irq_mask = INV_ICM20948_ACTIVE_LOW |
			INV_ICM20948_LATCH_INT_EN;
	else {
		dev_err(dev, "Invalid interrupt type 0x%x specified\n",
			irq_type);
		return -EINVAL;
	}

	result = inv_check_and_setup_chip(st);
	if (result)
		return result;

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm_core_probe);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device ICM20948 driver");
MODULE_LICENSE("GPL");
