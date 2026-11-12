/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define DT_DRV_COMPAT owntech_hb_board

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "hb.h"

struct hb_config {
	const struct device *sensors[2];
	const struct device *eeprom;
	struct gpio_dt_spec alerts[2];
};

struct hb_data {
	struct k_mutex lock;
};

BUILD_ASSERT(CONFIG_OWNTECH_HB_DRIVER_INIT_PRIORITY >
	     CONFIG_SENSOR_INIT_PRIORITY,
	     "HB driver must initialize after T117B sensors");
BUILD_ASSERT(CONFIG_OWNTECH_HB_DRIVER_INIT_PRIORITY >
	     CONFIG_EEPROM_INIT_PRIORITY,
	     "HB driver must initialize after the 24C02S EEPROM");
#ifdef CONFIG_GPIO_PCA_SERIES
BUILD_ASSERT(CONFIG_OWNTECH_HB_DRIVER_INIT_PRIORITY >
	     CONFIG_GPIO_PCA_SERIES_INIT_PRIORITY,
	     "HB driver must initialize after the alert GPIO expander");
#endif

static bool hb_side_is_valid(enum hb_side side)
{
	return side == HB_HIGH_SIDE || side == HB_LOW_SIDE;
}

static int hb_driver_init(const struct device *dev)
{
	const struct hb_config *config = dev->config;
	struct hb_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);
	if (!device_is_ready(config->eeprom)) {
		return -ENODEV;
	}

	for (size_t side = 0; side < ARRAY_SIZE(config->sensors); ++side) {
		if (!device_is_ready(config->sensors[side]) ||
		    !gpio_is_ready_dt(&config->alerts[side])) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->alerts[side], GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

const struct device *hb_sensor_device(const struct device *dev,
				      enum hb_side side)
{
	const struct hb_config *config = dev->config;

	return hb_side_is_valid(side) ? config->sensors[side] : NULL;
}

const struct device *hb_eeprom_device(const struct device *dev)
{
	const struct hb_config *config = dev->config;
	return config->eeprom;
}

static int hb_text_read(const struct device *dev, off_t offset,
			char *out, size_t length)
{
	const struct hb_config *config = dev->config;
	struct hb_data *data = dev->data;
	bool blank = true;
	int ret;

	if (out == NULL) {
		return -EINVAL;
	}
	out[0] = '\0';
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = eeprom_read(config->eeprom, offset, out, length);
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		out[0] = '\0';
		return ret;
	}
	for (size_t i = 0; i < length; ++i) {
		blank &= (uint8_t)out[i] == 0xffU;
	}
	if (blank) {
		out[0] = '\0';
		return -ENODATA;
	}
	for (size_t i = 0; i < length; ++i) {
		if (out[i] < '!' || out[i] > '~') {
			out[0] = '\0';
			return -EILSEQ;
		}
	}
	out[length] = '\0';
	return 0;
}

static int hb_text_write(const struct device *dev, off_t offset,
			 const char *value, size_t length)
{
	const struct hb_config *config = dev->config;
	struct hb_data *data = dev->data;
	char readback[HB_SERIAL_NUMBER_LENGTH];
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}
	for (size_t i = 0; i < length; ++i) {
		if (value[i] < '!' || value[i] > '~') {
			return -EINVAL;
		}
	}
	if (value[length] != '\0') {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = eeprom_read(config->eeprom, offset, readback, length);
	if (ret == 0 && memcmp(value, readback, length) != 0) {
		ret = eeprom_write(config->eeprom, offset, value, length);
		if (ret == 0) {
			ret = eeprom_read(config->eeprom, offset, readback, length);
		}
	}
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}
	return memcmp(value, readback, length) == 0 ? 0 : -EIO;
}

int hb_serial_number_read(const struct device *dev,
			  char out[HB_SERIAL_NUMBER_LENGTH + 1U])
{
	return hb_text_read(dev, HB_SERIAL_NUMBER_OFFSET, out,
			    HB_SERIAL_NUMBER_LENGTH);
}

int hb_serial_number_write(const struct device *dev, const char *serial)
{
	return hb_text_write(dev, HB_SERIAL_NUMBER_OFFSET, serial,
			     HB_SERIAL_NUMBER_LENGTH);
}

int hb_panel_key_read(const struct device *dev,
		      char out[HB_PANEL_KEY_LENGTH + 1U])
{
	return hb_text_read(dev, HB_PANEL_KEY_OFFSET, out,
			    HB_PANEL_KEY_LENGTH);
}

int hb_panel_key_write(const struct device *dev, const char *key)
{
	return hb_text_write(dev, HB_PANEL_KEY_OFFSET, key,
			     HB_PANEL_KEY_LENGTH);
}

int hb_temperature_get(const struct device *dev, enum hb_side side,
		       struct sensor_value *temperature)
{
	const struct hb_config *config = dev->config;
	struct hb_data *data = dev->data;
	int ret;

	if (!hb_side_is_valid(side) || temperature == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sensor_sample_fetch_chan(config->sensors[side],
				       SENSOR_CHAN_AMBIENT_TEMP);
	if (ret == 0) {
		ret = sensor_channel_get(config->sensors[side],
					 SENSOR_CHAN_AMBIENT_TEMP, temperature);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

int hb_temperature_get_millicelsius(const struct device *dev,
				     enum hb_side side,
				     int32_t *temperature_mc)
{
	struct sensor_value temperature;
	int64_t value;
	int ret;

	if (temperature_mc == NULL) {
		return -EINVAL;
	}

	ret = hb_temperature_get(dev, side, &temperature);
	if (ret != 0) {
		return ret;
	}

	value = sensor_value_to_milli(&temperature);
	if (value < INT32_MIN || value > INT32_MAX) {
		return -ERANGE;
	}
	*temperature_mc = (int32_t)value;
	return 0;
}

int hb_alert_is_active(const struct device *dev, enum hb_side side)
{
	const struct hb_config *config = dev->config;

	if (!hb_side_is_valid(side)) {
		return -EINVAL;
	}

	return gpio_pin_get_dt(&config->alerts[side]);
}

int hb_module_data_read(const struct device *dev,
			struct hb_module_data *module_data)
{
	struct hb_data *data = dev->data;
	const struct hb_config *config = dev->config;
	uint8_t bytes[HB_MODULE_DATA_SIZE];
	int ret;

	if (module_data == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = eeprom_read(config->eeprom, 0, bytes, sizeof(bytes));
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}
	if (bytes[0] != HB_MODULE_DATA_VERSION) {
		return -EPROTONOSUPPORT;
	}

	module_data->version = bytes[0];
	module_data->operating_hours = (uint32_t)bytes[1] |
				       ((uint32_t)bytes[2] << 8) |
				       ((uint32_t)bytes[3] << 16);
	module_data->max_voltage_code = (uint16_t)bytes[4] |
					((uint16_t)(bytes[5] & 0x0FU) << 8);
	module_data->max_current_code = (uint16_t)(bytes[5] >> 4) |
					((uint16_t)bytes[6] << 4);
	module_data->flags = bytes[7];
	return 0;
}

int hb_module_data_write(const struct device *dev,
			 const struct hb_module_data *module_data)
{
	struct hb_data *data = dev->data;
	const struct hb_config *config = dev->config;
	uint8_t bytes[HB_MODULE_DATA_SIZE];
	uint8_t readback[HB_MODULE_DATA_SIZE];
	int ret;

	if (module_data == NULL ||
	    module_data->version != HB_MODULE_DATA_VERSION ||
	    module_data->operating_hours > HB_MODULE_RUNTIME_HOURS_MAX ||
	    module_data->max_voltage_code > HB_MODULE_MEASUREMENT_MAX ||
	    module_data->max_current_code > HB_MODULE_MEASUREMENT_MAX) {
		return -EINVAL;
	}

	bytes[0] = module_data->version;
	bytes[1] = (uint8_t)module_data->operating_hours;
	bytes[2] = (uint8_t)(module_data->operating_hours >> 8);
	bytes[3] = (uint8_t)(module_data->operating_hours >> 16);
	bytes[4] = (uint8_t)module_data->max_voltage_code;
	bytes[5] = (uint8_t)((module_data->max_voltage_code >> 8) |
			     ((module_data->max_current_code & 0x0FU) << 4));
	bytes[6] = (uint8_t)(module_data->max_current_code >> 4);
	bytes[7] = module_data->flags;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = eeprom_read(config->eeprom, 0, readback, sizeof(readback));
	if (ret == 0 && memcmp(bytes, readback, sizeof(bytes)) != 0) {
		ret = eeprom_write(config->eeprom, 0, bytes, sizeof(bytes));
		if (ret == 0) {
			ret = eeprom_read(config->eeprom, 0, readback,
					  sizeof(readback));
		}
	}
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}
	return memcmp(bytes, readback, sizeof(bytes)) == 0 ? 0 : -EIO;
}

#define HB_DEFINE(inst)                                                       \
	static const struct hb_config hb_config_##inst = {                       \
		.sensors = {                                                       \
			DEVICE_DT_GET(DT_INST_PHANDLE(inst, high_side_sensor)),       \
			DEVICE_DT_GET(DT_INST_PHANDLE(inst, low_side_sensor)),        \
		},                                                                  \
		.eeprom = DEVICE_DT_GET(DT_INST_PHANDLE(inst, eeprom)),            \
		.alerts = {                                                        \
			GPIO_DT_SPEC_INST_GET_BY_IDX(inst, alert_gpios, 0),            \
			GPIO_DT_SPEC_INST_GET_BY_IDX(inst, alert_gpios, 1),            \
		},                                                                  \
	};                                                                         \
	static struct hb_data hb_data_##inst;                                      \
	DEVICE_DT_INST_DEFINE(inst, hb_driver_init, NULL, &hb_data_##inst,          \
			      &hb_config_##inst, POST_KERNEL,                       \
			      CONFIG_OWNTECH_HB_DRIVER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(HB_DEFINE)
