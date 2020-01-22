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
#include "inv_icm_channels.h"

// TODO verify these two tables
/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
static const int gyro_scale[] = {133090, 266181, 532362, 1064724};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
static const int accel_scale[] = {598, 1196, 2392, 4785};

static const struct inv_icm_reg_map reg_set_20948 = {
	.gyro_smplrt_div	= INV_ICM20948_REG_GYRO_SMPLRT_DIV,
	.accel_smplrt_div	= INV_ICM20948_REG_ACCEL_SMPLRT_DIV,
	.gyro_lpf               = INV_ICM20948_REG_CONFIG,
	.accel_lpf              = INV_ICM20948_REG_ACCEL_CONFIG,
	.user_ctrl              = INV_ICM20948_REG_USER_CTRL,
	.fifo_en                = INV_ICM20948_REG_FIFO_EN_2,
	.fifo_rst		= INV_ICM20948_REG_FIFO_RST,
	.gyro_config            = INV_ICM20948_REG_GYRO_CONFIG,
	.accl_config            = INV_ICM20948_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_ICM20948_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_ICM20948_REG_FIFO_R_W,
	.raw_gyro               = INV_ICM20948_REG_RAW_GYRO,
	.raw_accl               = INV_ICM20948_REG_RAW_ACCEL,
	.temperature            = INV_ICM20948_REG_TEMPERATURE,
	.int_enable             = INV_ICM20948_REG_INT_ENABLE_1,
	.int_status             = INV_ICM20948_REG_INT_STATUS_1,
	.pwr_mgmt_1             = INV_ICM20948_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_ICM20948_REG_PWR_MGMT_2,
	.int_pin_cfg		= INV_ICM20948_REG_INT_PIN_CFG,
	.accl_offset		= INV_ICM20948_REG_ACCEL_OFFSET,
	.gyro_offset		= INV_ICM20948_REG_GYRO_OFFSET,
	.i2c_if                 = 0,
};

static const struct inv_icm_chip_config chip_config_20948 = {
	.fsr = INV_ICM20948_FSR_2000DPS,
	.gyro_lpf = INV_ICM20948_GYRO_FILTER_18HZ,
	.accel_lpf = INV_ICM20948_ACCEL_FILTER_17HZ,
	.gyro_div = INV_ICM20948_FIFO_RATE_TO_DIVIDER(INV_ICM20948_GYRO_FREQ_HZ,
						      INV_ICM20948_INIT_FIFO_RATE),
	.accel_div = INV_ICM20948_FIFO_RATE_TO_DIVIDER(INV_ICM20948_ACCEL_FREQ_HZ,
						       INV_ICM20948_INIT_FIFO_RATE),
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

static int inv_icm_sensor_set(struct inv_icm_state  *st, int reg,
			      int axis, int val)
{
	int ind, result;
	__be16 d = cpu_to_be16(val);

	ind = (axis - IIO_MOD_X) * 2;
	result = regmap_bulk_write(st->map, reg + ind, (u8 *)&d, 2);
	if (result)
		return -EINVAL;

	return 0;
}

static int inv_icm_sensor_show(struct inv_icm_state  *st, int reg,
			       int axis, int *val)
{
	int ind, result;
	__be16 d;

	ind = (axis - IIO_MOD_X) * 2;
	result = regmap_bulk_read(st->map, reg + ind, (u8 *)&d, 2);
	if (result)
		return -EINVAL;
	*val = (short)be16_to_cpup(&d);

	return IIO_VAL_INT;
}

static int inv_icm_read_channel_data(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     int *val)
{
	struct inv_icm_state *st = iio_priv(indio_dev);
	int result;
	int ret;

	result = inv_icm_set_power_itg(st, true);
	if (result)
		return result;

	switch (chan->type) {
	case IIO_ANGL_VEL:
		result = inv_icm_switch_engine(st, true,
					       INV_ICM20948_BIT_PWR_GYRO_STBY);
		if (result)
			goto error_power_off;
		ret = inv_icm_sensor_show(st, st->reg->raw_gyro,
					  chan->channel2, val);
		result = inv_icm_switch_engine(st, false,
					       INV_ICM20948_BIT_PWR_GYRO_STBY);
		if (result)
			goto error_power_off;
		break;
	case IIO_ACCEL:
		result = inv_icm_switch_engine(st, true,
					       INV_ICM20948_BIT_PWR_ACCL_STBY);
		if (result)
			goto error_power_off;
		ret = inv_icm_sensor_show(st, st->reg->raw_accl,
					  chan->channel2, val);
		result = inv_icm_switch_engine(st, false,
					       INV_ICM20948_BIT_PWR_ACCL_STBY);
		if (result)
			goto error_power_off;
		break;
	case IIO_TEMP:
		/* wait for stablization */
		msleep(INV_ICM20948_SENSOR_UP_TIME);
		ret = inv_icm_sensor_show(st, st->reg->temperature,
					  IIO_MOD_X, val);
		break;
	// TODO removed magnetometer here
	default:
		ret = -EINVAL;
		break;
	}

	result = inv_icm_set_power_itg(st, false);
	if (result)
		goto error_power_off;

	return ret;

error_power_off:
	inv_icm_set_power_itg(st, false);
	return result;
}

static int inv_icm_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct inv_icm_state  *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		mutex_lock(&st->lock);
		ret = inv_icm_read_channel_data(indio_dev, chan, val);
		mutex_unlock(&st->lock);
		iio_device_release_direct_mode(indio_dev);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			mutex_lock(&st->lock);
			*val  = 0;
			*val2 = gyro_scale[st->chip_config.fsr];
			mutex_unlock(&st->lock);

			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ACCEL:
			mutex_lock(&st->lock);
			*val = 0;
			*val2 = accel_scale[st->chip_config.accl_fs];
			mutex_unlock(&st->lock);

			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = INV_ICM20948_TEMP_SCALE;

			return IIO_VAL_INT_PLUS_MICRO;
		// TODO Removed magnetometer
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = INV_ICM20948_TEMP_OFFSET;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			mutex_lock(&st->lock);
			ret = inv_icm_sensor_show(st, st->reg->gyro_offset,
						chan->channel2, val);
			mutex_unlock(&st->lock);
			return IIO_VAL_INT;
		case IIO_ACCEL:
			mutex_lock(&st->lock);
			ret = inv_icm_sensor_show(st, st->reg->accl_offset,
						chan->channel2, val);
			mutex_unlock(&st->lock);
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}


static int inv_icm_write_gyro_scale(struct inv_icm_state *st, int val)
{
	int result, i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(gyro_scale); ++i) {
		if (gyro_scale[i] == val) {
			d = (i << INV_ICM20948_GYRO_CONFIG_FSR_SHIFT);
			result = regmap_write(st->map, st->reg->gyro_config, d);
			if (result)
				return result;

			st->chip_config.fsr = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_write_raw_get_fmt(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int inv_icm_write_accel_scale(struct inv_icm_state *st, int val)
{
	int result, i;
	u8 d;

	for (i = 0; i < ARRAY_SIZE(accel_scale); ++i) {
		if (accel_scale[i] == val) {
			d = (i << INV_ICM20948_ACCL_CONFIG_FSR_SHIFT);
			result = regmap_write(st->map, st->reg->accl_config, d);
			if (result)
				return result;

			st->chip_config.accl_fs = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_icm_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct inv_icm_state  *st = iio_priv(indio_dev);
	int result;

	/*
	 * we should only update scale when the chip is disabled, i.e.
	 * not running
	 */
	result = iio_device_claim_direct_mode(indio_dev);
	if (result)
		return result;

	mutex_lock(&st->lock);
	result = inv_icm_set_power_itg(st, true);
	if (result)
		goto error_write_raw_unlock;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = inv_icm_write_gyro_scale(st, val2);
			break;
		case IIO_ACCEL:
			result = inv_icm_write_accel_scale(st, val2);
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = inv_icm_sensor_set(st,
						    st->reg->gyro_offset,
						    chan->channel2, val);
			break;
		case IIO_ACCEL:
			result = inv_icm_sensor_set(st,
						    st->reg->accl_offset,
						    chan->channel2, val);
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	default:
		result = -EINVAL;
		break;
	}

	result |= inv_icm_set_power_itg(st, false);
error_write_raw_unlock:
	mutex_unlock(&st->lock);
	iio_device_release_direct_mode(indio_dev);

	return result;
}

/**
 * inv_icm_validate_trigger() - validate_trigger callback for invensense
 *                              ICM devices.
 * @indio_dev: The IIO device
 * @trig: The new trigger
 *
 * Returns: 0 if the 'trig' matches the trigger registered by the ICM
 * device, -EINVAL otherwise.
 */
static int inv_icm_validate_trigger(struct iio_dev *indio_dev,
					 struct iio_trigger *trig)
{
	struct inv_icm_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_info icm_info = {
	.read_raw = &inv_icm_read_raw,
	.write_raw = &inv_icm_write_raw,
	.write_raw_get_fmt = &inv_write_raw_get_fmt,
	// TODO attributes were removed
	.validate_trigger = inv_icm_validate_trigger,
};

int inv_icm_switch_engine(struct inv_icm_state *st, bool en, u32 mask)
{
	unsigned int d, mgmt_1;
	int result;
	/*
	 * switch clock needs to be careful. Only when gyro is on, can
	 * clock source be switched to gyro. Otherwise, it must be set to
	 * internal clock
	 */
	if (mask == INV_ICM20948_BIT_PWR_GYRO_STBY) {
		result = regmap_read(st->map, st->reg->pwr_mgmt_1, &mgmt_1);
		if (result)
			return result;

		mgmt_1 &= ~INV_ICM20948_BIT_CLK_MASK;
	}

	if ((mask == INV_ICM20948_BIT_PWR_GYRO_STBY) && (!en)) {
		/*
		 * turning off gyro requires switch to internal clock first.
		 * Then turn off gyro engine
		 */
		mgmt_1 |= INV_CLK_INTERNAL;
		result = regmap_write(st->map, st->reg->pwr_mgmt_1, mgmt_1);
		if (result)
			return result;
	}

	result = regmap_read(st->map, st->reg->pwr_mgmt_2, &d);
	if (result)
		return result;
	if (en)
		d &= ~mask;
	else
		d |= mask;
	result = regmap_write(st->map, st->reg->pwr_mgmt_2, d);
	if (result)
		return result;

	if (en) {
		/* Wait for output to stabilize */
		msleep(INV_ICM20948_TEMP_UP_TIME);
		if (mask == INV_ICM20948_BIT_PWR_GYRO_STBY) {
			/* switch internal clock to PLL */
			mgmt_1 |= INV_CLK_PLL;
			result = regmap_write(st->map,
					      st->reg->pwr_mgmt_1, mgmt_1);
			if (result)
				return result;
		}
	}

	return 0;
}

int inv_icm_set_power_itg(struct inv_icm_state *st, bool power_on)
{
	int result;

	if (power_on) {
		if (!st->powerup_count) {
			result = regmap_write(st->map, st->reg->pwr_mgmt_1, 0);
			if (result)
				return result;
			usleep_range(INV_ICM20948_REG_UP_TIME_MIN,
				     INV_ICM20948_REG_UP_TIME_MAX);
		}
		st->powerup_count++;
	} else {
		// TODO fix this in the upstream driver ?
		// Could be stuck if power_itg is called sev times.
		if (st->powerup_count >= 1) {
			result = regmap_write(st->map, st->reg->pwr_mgmt_1,
					      INV_ICM20948_BIT_SLEEP);
			if (result)
				return result;
		}
		st->powerup_count--;
	}

	dev_dbg(regmap_get_device(st->map), "set power %d, count=%u\n",
		power_on, st->powerup_count);

	return 0;
}

/**
 * inv_icm_userbank_write() - write a value to a single user bank register
 *
 * @st: pointer to the driver state
 * @bank_reg: Register to write to, MSB u8 is user bank, LSB u8 is register
 * @val: Value to be written
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
int inv_icm_userbank_write(struct inv_icm_state *st,
				  u16 bank_reg,
				  u8 val)
{
	int result;
	u8 bank;
	u8 reg;

	bank = (u8) bank_reg >> 8;
	reg = (u8) bank_reg & 0xFF;

	result = regmap_write(st->map, st->reg->bank_sel, bank);
	if (result)
		return result;

	result = regmap_write(st->map, reg, val);
	if (result)
		return result;

	/* Switch back to user bank 0 (default) */
	result = regmap_write(st->map, st->reg->bank_sel, 0x00);

	return result;
}

/**
 *  inv_icm_set_lpf_regs() - set low pass filter registers for gyro and accel
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
static int inv_icm_set_lpf_regs(struct inv_icm_state *st,
				enum inv_icm_accel_filter_e acc_val,
				enum inv_icm_gyro_filter_e gyr_val)
{
	int result;

	/* set accel lpf */
	result = inv_icm_userbank_write(st, st->reg->accel_lpf, acc_val);
	if (result)
		return result;

	/* set gyro lpf */
	result = inv_icm_userbank_write(st, st->reg->accel_lpf, gyr_val);
	if (result)
		return result;

	/* set the user bank back to default (0) */
	result = regmap_write(st->map, st->reg->bank_sel, 0x00);

	return result;
}

/**
 *  inv_icm_init_config() - Initialize hardware, disable FIFO.
 *
 *  Initial configuration:
 *  FSR: ± 2000DPS
 *  DLPF acc:  17Hz
 *  DLPF gyro: 18Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_icm_init_config(struct iio_dev *indio_dev)
{
	int result;
	u8 d;
	struct inv_icm_state *st = iio_priv(indio_dev);

	result = inv_icm_set_power_itg(st, true);
	if (result)
		return result;

	d = (INV_ICM20948_FS_02G << INV_ICM20948_ACCL_CONFIG_FSR_SHIFT);
	result = inv_icm_userbank_write(st, st->reg->accl_config, d);
	if (result)
		goto error_power_off;

	d = (INV_ICM20948_FSR_2000DPS << INV_ICM20948_GYRO_CONFIG_FSR_SHIFT);
	result = inv_icm_userbank_write(st, st->reg->gyro_config, d);
	if (result)
		goto error_power_off;

	result = inv_icm_set_lpf_regs(st, INV_ICM20948_ACCEL_FILTER_17HZ,
				          INV_ICM20948_GYRO_FILTER_18HZ);
	if (result)
		goto error_power_off;

	d = INV_ICM20948_FIFO_RATE_TO_DIVIDER(INV_ICM20948_ACCEL_FREQ_HZ,
					      INV_ICM20948_INIT_FIFO_RATE);
	result = inv_icm_userbank_write(st, st->reg->accel_smplrt_div, d);
	if (result)
		goto error_power_off;

	d = INV_ICM20948_FIFO_RATE_TO_DIVIDER(INV_ICM20948_GYRO_FREQ_HZ,
					      INV_ICM20948_INIT_FIFO_RATE);
	result = inv_icm_userbank_write(st, st->reg->gyro_smplrt_div, d);
	if (result)
		goto error_power_off;

	result = regmap_write(st->map, st->reg->int_pin_cfg, st->irq_mask);
	if (result)
		return result;

	memcpy(&st->chip_config, hw_info[st->chip_type].config,
	       sizeof(struct inv_icm_chip_config));

	/*
	 * Internal chip period is 1ms (1kHz).
	 * Let's use at the beginning the theorical value before measuring
	 * with interrupt timestamps.
	 */
	st->chip_period = NSEC_PER_MSEC;

	// TODO removed magnetometer here

	return inv_icm_set_power_itg(st, false);

error_power_off:
	inv_icm_set_power_itg(st, false);
	return result;
}

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

	/*
	 * Turn power on. After reset, the sleep bit could be on
	 * or off depending on the OTP settings. Turning power on
	 * make it in a definite state as well as making the hardware
	 * state align with the software state
	 */
	dev_warn(regmap_get_device(st->map), "ICM power\n");
	result = inv_icm_set_power_itg(st, true);
	if (result)
		return result;

	result = inv_icm_switch_engine(st, false,
					   INV_ICM20948_BIT_PWR_ACCL_STBY);
	if (result)
		goto error_power_off;
	result = inv_icm_switch_engine(st, false,
					   INV_ICM20948_BIT_PWR_GYRO_STBY);
	if (result)
		goto error_power_off;

	return inv_icm_set_power_itg(st, false);

error_power_off:
	inv_icm_set_power_itg(st, false);
	return result;
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

	result = inv_icm_init_config(indio_dev);
	if (result) {
		dev_err(dev, "Could not initialize device.\n");
		return result;
	}

	dev_set_drvdata(dev, indio_dev);
	indio_dev->dev.parent = dev;
	indio_dev->name = name;

	// TODO what is this ?
	indio_dev->channels = inv_icm_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm_channels);
	indio_dev->available_scan_masks = inv_icm_scan_masks;

	indio_dev->info = &icm_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	result = devm_iio_triggered_buffer_setup(dev, indio_dev,
						 iio_pollfunc_store_time,
						 inv_icm_read_fifo,
						 NULL);
	if (result) {
		dev_err(dev, "configure buffer fail %d\n", result);
		return result;
	}
	dev_err(dev, "configure buffer success %d\n", result);
	result = inv_icm_probe_trigger(indio_dev, irq_type);
	if (result) {
		dev_err(dev, "trigger probe fail %d\n", result);
		return result;
	}

	result = devm_iio_device_register(dev, indio_dev);
	if (result) {
		dev_err(dev, "IIO register fail %d\n", result);
		return result;
	}

	return 0;
}

MODULE_AUTHOR("Loys Ollivier <loys.ollivier@gmail.com>");
MODULE_DESCRIPTION("Invensense ICM20948 driver");
MODULE_LICENSE("GPL");
