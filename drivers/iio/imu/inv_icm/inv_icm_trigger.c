// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (C) 2012 Invensense, Inc.
* Copyright (C) 2020 Loys Ollivier <loys.ollivier@gmail.com>
*/

#include "inv_icm_iio.h"

static void inv_scan_query(struct iio_dev *indio_dev)
{
	struct inv_icm_state  *st = iio_priv(indio_dev);

	st->chip_config.gyro_fifo_enable =
		test_bit(INV_ICM_SCAN_GYRO_X,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_ICM_SCAN_GYRO_Y,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_ICM_SCAN_GYRO_Z,
			 indio_dev->active_scan_mask);

	st->chip_config.accl_fifo_enable =
		test_bit(INV_ICM_SCAN_ACCL_X,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_ICM_SCAN_ACCL_Y,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_ICM_SCAN_ACCL_Z,
			 indio_dev->active_scan_mask);
}

static unsigned int inv_compute_skip_samples(const struct inv_icm_state *st)
{
	unsigned int gyro_skip = 0;
	unsigned int magn_skip = 0;
	unsigned int skip_samples;

	/* gyro first sample is out of specs, skip it */
	if (st->chip_config.gyro_fifo_enable)
		gyro_skip = 1;

	/* mag first sample is always not ready, skip it */
	if (st->chip_config.magn_fifo_enable)
		magn_skip = 1;

	/* compute first samples to skip */
	skip_samples = gyro_skip;
	if (magn_skip > skip_samples)
		skip_samples = magn_skip;

	return skip_samples;
}

/**
 *  inv_icm_set_enable() - enable chip functions.
 *  @indio_dev:	Device driver instance.
 *  @enable: enable/disable
 */
static int inv_icm_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct inv_icm_state *st = iio_priv(indio_dev);
	uint8_t d;
	int result;

	if (enable) {
		result = inv_icm_set_power_itg(st, true);
		if (result)
			return result;
		inv_scan_query(indio_dev);
		if (st->chip_config.gyro_fifo_enable) {
			result = inv_icm_switch_engine(st, true,
					INV_ICM20948_BIT_PWR_GYRO_STBY);
			if (result)
				goto error_power_off;
		}
		if (st->chip_config.accl_fifo_enable) {
			result = inv_icm_switch_engine(st, true,
					INV_ICM20948_BIT_PWR_ACCL_STBY);
			if (result)
				goto error_gyro_off;
		}
		if (st->chip_config.magn_fifo_enable) {
			d = st->chip_config.user_ctrl |
					INV_ICM20948_BIT_I2C_MST_EN;
			result = regmap_write(st->map, st->reg->user_ctrl, d);
			if (result)
				goto error_accl_off;
			st->chip_config.user_ctrl = d;
		}
		st->skip_samples = inv_compute_skip_samples(st);
		result = inv_reset_fifo(indio_dev);
		if (result)
			goto error_magn_off;
	} else {
		result = regmap_write(st->map, st->reg->fifo_en, 0);
		if (result)
			goto error_magn_off;

		result = regmap_write(st->map, st->reg->int_enable, 0);
		if (result)
			goto error_magn_off;

		d = st->chip_config.user_ctrl & ~INV_ICM20948_BIT_I2C_MST_EN;
		result = regmap_write(st->map, st->reg->user_ctrl, d);
		if (result)
			goto error_magn_off;
		st->chip_config.user_ctrl = d;

		result = inv_icm_switch_engine(st, false,
					INV_ICM20948_BIT_PWR_ACCL_STBY);
		if (result)
			goto error_accl_off;

		result = inv_icm_switch_engine(st, false,
					INV_ICM20948_BIT_PWR_GYRO_STBY);
		if (result)
			goto error_gyro_off;

		result = inv_icm_set_power_itg(st, false);
		if (result)
			goto error_power_off;
	}

	return 0;

error_magn_off:
	/* always restore user_ctrl to disable fifo properly */
	st->chip_config.user_ctrl &= ~INV_ICM20948_BIT_I2C_MST_EN;
	regmap_write(st->map, st->reg->user_ctrl, st->chip_config.user_ctrl);
error_accl_off:
	if (st->chip_config.accl_fifo_enable)
		inv_icm_switch_engine(st, false,
					  INV_ICM20948_BIT_PWR_ACCL_STBY);
error_gyro_off:
	if (st->chip_config.gyro_fifo_enable)
		inv_icm_switch_engine(st, false,
					  INV_ICM20948_BIT_PWR_GYRO_STBY);
error_power_off:
	inv_icm_set_power_itg(st, false);
	return result;
}

/**
 * inv_icm_data_rdy_trigger_set_state() - set data ready interrupt state
 * @trig: Trigger instance
 * @state: Desired trigger state
 */
static int inv_icm_data_rdy_trigger_set_state(struct iio_trigger *trig,
					      bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct inv_icm_state *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&st->lock);
	result = inv_icm_set_enable(indio_dev, state);
	mutex_unlock(&st->lock);

	return result;
}

static const struct iio_trigger_ops inv_icm_trigger_ops = {
	.set_trigger_state = &inv_icm_data_rdy_trigger_set_state,
};

int inv_icm_probe_trigger(struct iio_dev *indio_dev, int irq_type)
{
	int ret;
	struct inv_icm_state *st = iio_priv(indio_dev);

	st->trig = devm_iio_trigger_alloc(&indio_dev->dev,
					  "%s-dev%d",
					  indio_dev->name,
					  indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, st->irq,
			       &iio_trigger_generic_data_rdy_poll,
			       irq_type,
			       "inv_icm",
			       st->trig);
	if (ret)
		return ret;

	st->trig->dev.parent = regmap_get_device(st->map);
	st->trig->ops = &inv_icm_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	return 0;
}
