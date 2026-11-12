/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define DT_DRV_COMPAT mysentech_t117b

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define T117B_REG_TEMP_LSB 0x00U
#define T117B_REG_TEMP_CMD 0x04U
#define T117B_REG_ROMCODE  0x18U

struct t117b_config {
	struct i2c_dt_spec bus;
};

struct t117b_data {
	int16_t sample;
};

static int t117b_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct t117b_config *config = dev->config;
	struct t117b_data *data = dev->data;
	uint8_t raw[2];
	int ret;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}
	ret = i2c_burst_read_dt(&config->bus, T117B_REG_TEMP_LSB,
				raw, sizeof(raw));
	if (ret == 0) {
		/* T117B sends the low byte first. The code is relative to 25 C. */
		data->sample = (int16_t)((uint16_t)raw[0] |
					 ((uint16_t)raw[1] << 8));
	}
	return ret;
}

static int t117b_channel_get(const struct device *dev,
			     enum sensor_channel chan, struct sensor_value *val)
{
	const struct t117b_data *data = dev->data;
	int32_t microcelsius;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP || val == NULL) {
		return -EINVAL;
	}
	microcelsius = 25000000L + (int32_t)data->sample * 1000000L / 256L;
	val->val1 = microcelsius / 1000000L;
	val->val2 = microcelsius % 1000000L;
	return 0;
}

static const struct sensor_driver_api t117b_api = {
	.sample_fetch = t117b_sample_fetch,
	.channel_get = t117b_channel_get,
};

static int t117b_init(const struct device *dev)
{
	const struct t117b_config *config = dev->config;
	uint8_t romcode[2];
	uint8_t start_continuous[2] = {T117B_REG_TEMP_CMD, 0x00U};
	int ret;

	if (!i2c_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}
	ret = i2c_burst_read_dt(&config->bus, T117B_REG_ROMCODE,
				romcode, sizeof(romcode));
	if (ret != 0) {
		return ret;
	}
	if (romcode[0] != 0x01U || romcode[1] != 0x16U) {
		return -ENODEV;
	}
	/* The T117B starts in stop mode; enable temperature conversion. */
	return i2c_write_dt(&config->bus, start_continuous,
			    sizeof(start_continuous));
}

#define T117B_DEFINE(inst)                                               \
	static const struct t117b_config t117b_config_##inst = {           \
		.bus = I2C_DT_SPEC_INST_GET(inst),                        \
	};                                                                 \
	static struct t117b_data t117b_data_##inst;                        \
	DEVICE_DT_INST_DEFINE(inst, t117b_init, NULL,                     \
			      &t117b_data_##inst, &t117b_config_##inst,   \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,      \
			      &t117b_api);

DT_INST_FOREACH_STATUS_OKAY(T117B_DEFINE)
