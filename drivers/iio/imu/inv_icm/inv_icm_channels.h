/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Loys Ollivier <loys.ollivier@gmail.com>
 */

#ifndef INV_ICM_CHANNELS_H_
#define INV_ICM_CHANNELS_H_

// TODO move that to a C file

static const struct iio_mount_matrix *
inv_get_mount_matrix(const struct iio_dev *indio_dev,
		     const struct iio_chan_spec *chan)
{
	struct inv_icm_state *data = iio_priv(indio_dev);
	const struct iio_mount_matrix *matrix;

	if (chan->type == IIO_MAGN)
		matrix = &data->magn_orient;
	else
		matrix = &data->orientation;

	return matrix;
}

static const struct iio_chan_spec_ext_info inv_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, inv_get_mount_matrix),
	{ }
};

#define INV_ICM_CHAN(_type, _channel2, _index)                    \
	{                                                             \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	      \
				      BIT(IIO_CHAN_INFO_CALIBBIAS),   \
		.scan_index = _index,                                 \
		.scan_type = {                                        \
				.sign = 's',                          \
				.realbits = 16,                       \
				.storagebits = 16,                    \
				.shift = 0,                           \
				.endianness = IIO_BE,                 \
			     },                                       \
		.ext_info = inv_ext_info,                             \
	}

static const struct iio_chan_spec inv_icm_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(INV_ICM_SCAN_TIMESTAMP),
        // TODO handle temp
	/*
	 * Note that temperature should only be via polled reading only,
	 * not the final scan elements output.
	 */
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
				| BIT(IIO_CHAN_INFO_OFFSET)
				| BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1,
	},
	INV_ICM_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_ICM_SCAN_GYRO_X),
	INV_ICM_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_ICM_SCAN_GYRO_Y),
	INV_ICM_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_ICM_SCAN_GYRO_Z),

	INV_ICM_CHAN(IIO_ACCEL, IIO_MOD_X, INV_ICM_SCAN_ACCL_X),
	INV_ICM_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_ICM_SCAN_ACCL_Y),
	INV_ICM_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_ICM_SCAN_ACCL_Z),
};

static const unsigned long inv_icm_scan_masks[] = {
	/* 3-axis accel */
	BIT(INV_ICM_SCAN_ACCL_X)
        | BIT(INV_ICM_SCAN_ACCL_Y)
        | BIT(INV_ICM_SCAN_ACCL_Z),
	/* 3-axis gyro */
	BIT(INV_ICM_SCAN_GYRO_X)
        | BIT(INV_ICM_SCAN_GYRO_Y)
        | BIT(INV_ICM_SCAN_GYRO_Z),
	/* 6-axis accel + gyro */
	BIT(INV_ICM_SCAN_ACCL_X)
        | BIT(INV_ICM_SCAN_ACCL_Y)
        | BIT(INV_ICM_SCAN_ACCL_Z)
        | BIT(INV_ICM_SCAN_GYRO_X)
        | BIT(INV_ICM_SCAN_GYRO_Y)
        | BIT(INV_ICM_SCAN_GYRO_Z),
	0,
};

#endif
