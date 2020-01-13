/* SPDX-License-Identifier: GPL-2.0-only */
/*
* Copyright (C) 2020 Loys Ollivier <loys.ollivier@gmail.com>
*/

#ifndef INV_ICM_IIO_H_
#define INV_ICM_IIO_H_

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/regmap.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

/**
 *  struct inv_icm_reg_map - Notable registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal low pass filter.
 *  @accel_lpf:		Configures accelerometer low pass filter.
 *  @user_ctrl:		Enables/resets the FIFO.
 *  @fifo_en:		Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accl_config:	accel config register
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:		FIFO register.
 *  @raw_gyro:		Address of first gyro register.
 *  @raw_accl:		Address of first accel register.
 *  @temperature:	temperature register
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt status register.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 *  @int_pin_cfg;	Controls interrupt pin configuration.
 *  @accl_offset:	Controls the accelerometer calibration offset.
 *  @gyro_offset:	Controls the gyroscope calibration offset.
 *  @i2c_if:		Controls the i2c interface
 */
struct inv_icm_reg_map {
	u8 sample_rate_div;
	u8 lpf;
	u8 accel_lpf;
	u8 user_ctrl;
	u8 fifo_en;
	u8 gyro_config;
	u8 accl_config;
	u8 fifo_count_h;
	u8 fifo_r_w;
	u8 raw_gyro;
	u8 raw_accl;
	u8 temperature;
	u8 int_enable;
	u8 int_status;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
	u8 int_pin_cfg;
	u8 accl_offset;
	u8 gyro_offset;
	u8 i2c_if;
};

/*device enum */
enum inv_devices {
	INV_ICM20948,
	INV_NUM_PARTS
};

/**
 *  struct inv_icm_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @accl_fifo_enable:	enable accel data output
 *  @gyro_fifo_enable:	enable gyro data output
 *  @magn_fifo_enable:	enable magn data output
 *  @divider:		chip sample rate divider (sample rate divider - 1)
 */
struct inv_icm_chip_config {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int accl_fs:2;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_fifo_enable:1;
	unsigned int magn_fifo_enable:1;
	u8 divider;
	u8 user_ctrl;
};

/**
 *  struct inv_icm_hw - Other important hardware information.
 *  @whoami:	Self identification byte from WHO_AM_I register
 *  @name:      name of the chip.
 *  @reg:   register map of the chip.
 *  @config:    configuration of the chip.
 *  @fifo_size:	size of the FIFO in bytes.
 */
struct inv_icm_hw {
	u8 whoami;
	u8 *name;
	const struct inv_icm_reg_map *reg;
	const struct inv_icm_chip_config *config;
	size_t fifo_size;
};

/*
 *  struct inv_icm_state - Driver state variables.
 *  @lock:              Chip access lock.
 *  @trig:              IIO trigger.
 *  @chip_config:	Cached attribute information.
 *  @reg:		Map of important registers.
 *  @hw:		Other hardware-specific information.
 *  @chip_type:		chip type.
 *  @orientation:	sensor chip orientation relative to main hardware.
 *  @map		regmap pointer.
 *  @irq		interrupt number.
 *  @irq_mask		the int_pin_cfg mask to configure interrupt type.
 *  @chip_period:	chip internal period estimation (~1kHz).
 *  @it_timestamp:	timestamp from previous interrupt.
 *  @data_timestamp:	timestamp for next data sample.
 *  @vdd_supply:	VDD voltage regulator for the chip.
 *  @vddio_supply	I/O voltage regulator for the chip.
 *  @magn_disabled:     magnetometer disabled for backward compatibility reason.
 *  @magn_raw_to_gauss:	coefficient to convert mag raw value to Gauss.
 *  @magn_orient:       magnetometer sensor chip orientation if available.
 */
struct inv_icm_state {
	struct mutex lock;
	struct iio_trigger  *trig;
	struct inv_icm_chip_config chip_config;
	const struct inv_icm_reg_map *reg;
	const struct inv_icm_hw *hw;
	enum   inv_devices chip_type;
	struct i2c_mux_core *muxc;
	struct i2c_client *mux_client;
	unsigned int powerup_count;
	struct iio_mount_matrix orientation;
	struct regmap *map;
	int irq;
	u8 irq_mask;
	unsigned skip_samples;
	s64 chip_period;
	s64 it_timestamp;
	s64 data_timestamp;
	struct regulator *vdd_supply;
	struct regulator *vddio_supply;
	bool magn_disabled;
	s32 magn_raw_to_gauss[3];
	struct iio_mount_matrix magn_orient;
};

/*register and associated bit definition*/
#define INV_ICM20948_REG_ACCEL_OFFSET        0x0120
#define INV_ICM20948_REG_GYRO_OFFSET         0x0203

#define INV_ICM20948_REG_SAMPLE_RATE_DIV     0x0200
#define INV_ICM20948_REG_CONFIG              0x0201
#define INV_ICM20948_REG_GYRO_CONFIG         0x0202
#define INV_ICM20948_REG_ACCEL_CONFIG        0x0214

#define INV_ICM20948_REG_FIFO_EN_1           0x0066
#define INV_ICM20948_REG_FIFO_EN_2           0x0067
#define INV_ICM20948_BIT_SLAVE_0             0x01
#define INV_ICM20948_BIT_SLAVE_1             0x02
#define INV_ICM20948_BIT_SLAVE_2             0x04
#define INV_ICM20948_BIT_SLAVE_3             0x08
#define INV_ICM20948_BIT_ACCEL_OUT           0x10
#define INV_ICM20948_BITS_GYRO_OUT           0x0E

#define INV_ICM20948_REG_I2C_MST_CTRL        0x0301
#define INV_ICM20948_BITS_I2C_MST_CLK_400KHZ 0x07
#define INV_ICM20948_BIT_I2C_MST_P_NSR       0x10
#define INV_ICM20948_BIT_MULT_MST_EN         0x80

/* control I2C slaves from 0 to 3 */
#define INV_ICM20948_REG_I2C_SLV_ADDR(_x)    (0x0303 + 4 * (_x))UL
#define INV_ICM20948_BIT_I2C_SLV_RNW         0x80

#define INV_ICM20948_REG_I2C_SLV_REG(_x)     (0x0304 + 4 * (_x))

#define INV_ICM20948_REG_I2C_SLV_CTRL(_x)    (0x0305 + 4 * (_x))
#define INV_ICM20948_BIT_SLV_GRP             0x10
#define INV_ICM20948_BIT_SLV_REG_DIS         0x20
#define INV_ICM20948_BIT_SLV_BYTE_SW         0x40
#define INV_ICM20948_BIT_SLV_EN              0x80

/* I2C master delay register */
#define INV_ICM20948_REG_I2C_SLV4_CTRL       0x0315
#define INV_ICM20948_BITS_I2C_MST_DLY(_x)    ((_x) & 0x1F)

#define INV_ICM20948_REG_I2C_MST_STATUS      0x0017
#define INV_ICM20948_BIT_I2C_SLV0_NACK       0x01
#define INV_ICM20948_BIT_I2C_SLV1_NACK       0x02
#define INV_ICM20948_BIT_I2C_SLV2_NACK       0x04
#define INV_ICM20948_BIT_I2C_SLV3_NACK       0x08

// Todo replace by a dynamic define
#define INV_ICM20948_REG_INT_ENABLE_0        0x0010
#define INV_ICM20948_REG_INT_ENABLE_1        0x0011
#define INV_ICM20948_REG_INT_ENABLE_2        0x0012
#define INV_ICM20948_REG_INT_ENABLE_3        0x0013
#define INV_ICM20948_BIT_DATA_RDY_EN         0x01
#define INV_ICM20948_BIT_DMP_INT_EN          0x02

#define INV_ICM20948_REG_RAW_ACCEL           0x002D
#define INV_ICM20948_REG_TEMPERATURE         0x0039
#define INV_ICM20948_REG_RAW_GYRO            0x0033

// Todo replace by a dynamic define
#define INV_ICM20948_REG_INT_STATUS_0        0x0019
#define INV_ICM20948_REG_INT_STATUS_1        0x001A
#define INV_ICM20948_REG_INT_STATUS_2        0x001B
#define INV_ICM20948_REG_INT_STATUS_3        0x001C
#define INV_ICM20948_BIT_FIFO_OVERFLOW_INT   0x1F // TODO verify this
#define INV_ICM20948_BIT_RAW_DATA_RDY_INT    0x01

#define INV_ICM20948_REG_EXT_SENS_DATA       0x003B

/* I2C slaves data output from 0 to 3 */
#define INV_ICM20948_REG_I2C_SLV_DO(_x)      (0x0306 + 4 * (_x))

#define INV_ICM20948_REG_I2C_MST_DELAY_CTRL  0x0302
#define INV_ICM20948_BIT_I2C_SLV0_DLY_EN     0x01
#define INV_ICM20948_BIT_I2C_SLV1_DLY_EN     0x02
#define INV_ICM20948_BIT_I2C_SLV2_DLY_EN     0x04
#define INV_ICM20948_BIT_I2C_SLV3_DLY_EN     0x08
#define INV_ICM20948_BIT_DELAY_ES_SHADOW     0x80

#define INV_ICM20948_REG_USER_CTRL           0x0003
#define INV_ICM20948_BIT_DMP_RST             0x04
#define INV_ICM20948_BIT_I2C_IF_DIS          0x10
#define INV_ICM20948_BIT_I2C_MST_EN          0x20
#define INV_ICM20948_BIT_FIFO_EN             0x40
#define INV_ICM20948_BIT_DMP_EN              0x80

// TODO behavior has changed
#define INV_ICM20948_REG_FIFO_RST            0x0068
#define INV_ICM20948_BIT_FIFO_RST            0x1F

#define INV_ICM20948_REG_PWR_MGMT_1          0x0006
#define INV_ICM20948_BIT_H_RESET             0x80
#define INV_ICM20948_BIT_SLEEP               0x40
#define INV_ICM20948_BIT_CLK_MASK            0x7

#define INV_ICM20948_REG_PWR_MGMT_2          0x0007
#define INV_ICM20948_BIT_PWR_ACCL_STBY       0x38
#define INV_ICM20948_BIT_PWR_GYRO_STBY       0x07

#define INV_ICM20948_REG_FIFO_COUNT_H        0x0070
#define INV_ICM20948_REG_FIFO_R_W            0x0072

#define INV_ICM20948_BYTES_PER_3AXIS_SENSOR   6
#define INV_ICM20948_FIFO_COUNT_BYTE          2

/* ICM20602 FIFO samples include temperature readings */
#define INV_ICM20602_BYTES_PER_TEMP_SENSOR   2

/* delay time in milliseconds */
#define INV_ICM20948_POWER_UP_TIME            100
#define INV_ICM20948_TEMP_UP_TIME             100
#define INV_ICM20948_SENSOR_UP_TIME           30

/* delay time in microseconds */
#define INV_ICM20948_REG_UP_TIME_MIN          5000
#define INV_ICM20948_REG_UP_TIME_MAX          10000

#define INV_ICM20948_TEMP_OFFSET              12421
#define INV_ICM20948_TEMP_SCALE               2941
#define INV_ICM20948_MAX_GYRO_FS_PARAM        3
#define INV_ICM20948_MAX_ACCL_FS_PARAM        3
#define INV_ICM20948_THREE_AXIS               3
#define INV_ICM20948_GYRO_CONFIG_FSR_SHIFT    3
#define INV_ICM20948_ACCL_CONFIG_FSR_SHIFT    3

// TODO Fix whitespace
#define INV_ICM20948_REG_INT_PIN_CFG	0x37
#define INV_ICM20948_ACTIVE_HIGH		0x00
#define INV_ICM20948_ACTIVE_LOW		0x80
/* enable level triggering */
#define INV_ICM20948_LATCH_INT_EN	0x20
#define INV_ICM20948_BIT_BYPASS_EN	0x2

/* Allowed timestamp period jitter in percent */
#define INV_ICM20948_TS_PERIOD_JITTER	4

/* init parameters */
#define INV_ICM20948_INIT_FIFO_RATE           50
#define INV_ICM20948_MAX_FIFO_RATE            1000
#define INV_ICM20948_MIN_FIFO_RATE            4

/* chip internal frequency: 1KHz */
// TODO validate chip freq, looks like it's 1125 (see ACCEL_SMPLRT_DIV)
#define INV_ICM20948_INTERNAL_FREQ_HZ		1000
/* return the frequency divider (chip sample rate divider + 1) */
#define INV_ICM20948_FREQ_DIVIDER(st)					\
	((st)->chip_config.divider + 1)
/* chip sample rate divider to fifo rate */
#define INV_ICM20948_FIFO_RATE_TO_DIVIDER(fifo_rate)			\
	((INV_ICM20948_INTERNAL_FREQ_HZ / (fifo_rate)) - 1)
#define INV_ICM20948_DIVIDER_TO_FIFO_RATE(divider)			\
	(INV_ICM20948_INTERNAL_FREQ_HZ / ((divider) + 1))

#define INV_ICM20948_REG_WHOAMI			0x0000

#define INV_ICM20948_WHOAMI_VALUE		0xEA

enum inv_icm_filter_e {
	INV_ICM20948_FILTER_256HZ_NOLPF2 = 0,
	INV_ICM20948_FILTER_188HZ,
	INV_ICM20948_FILTER_98HZ,
	INV_ICM20948_FILTER_42HZ,
	INV_ICM20948_FILTER_20HZ,
	INV_ICM20948_FILTER_10HZ,
	INV_ICM20948_FILTER_5HZ,
	INV_ICM20948_FILTER_2100HZ_NOLPF,
	NUM_ICM20948_FILTER
};

/* IIO attribute address */
enum INV_ICM_IIO_ATTR_ADDR {
	ATTR_GYRO_MATRIX,
	ATTR_ACCL_MATRIX,
};

enum inv_icm_accl_fs_e {
	INV_ICM20948_FS_02G = 0,
	INV_ICM20948_FS_04G,
	INV_ICM20948_FS_08G,
	INV_ICM20948_FS_16G,
	NUM_ACCL_FSR
};

enum inv_icm_fsr_e {
	INV_ICM20948_FSR_250DPS = 0,
	INV_ICM20948_FSR_500DPS,
	INV_ICM20948_FSR_1000DPS,
	INV_ICM20948_FSR_2000DPS,
	NUM_ICM20948_FSR
};


int inv_icm_core_probe(struct regmap *regmap, int irq, const char *name,
		       int chip_type);

#endif
