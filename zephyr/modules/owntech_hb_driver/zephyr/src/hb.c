/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define DT_DRV_COMPAT owntech_hb_board

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
	const struct device *eeproms[2];
	struct gpio_dt_spec alerts[2];
};

struct hb_data {
	struct k_mutex lock;
};

BUILD_ASSERT(CONFIG_OWNTECH_HB_DRIVER_INIT_PRIORITY >
	     CONFIG_SENSOR_INIT_PRIORITY,
	     "HB driver must initialize after TMP117 sensors");
BUILD_ASSERT(CONFIG_OWNTECH_HB_DRIVER_INIT_PRIORITY >
	     CONFIG_EEPROM_INIT_PRIORITY,
	     "HB driver must initialize after TMP117 EEPROMs");
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

	for (size_t side = 0; side < ARRAY_SIZE(config->sensors); ++side) {
		if (!device_is_ready(config->sensors[side]) ||
		    !device_is_ready(config->eeproms[side]) ||
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

const struct device *hb_eeprom_device(const struct device *dev,
				      enum hb_side side)
{
	const struct hb_config *config = dev->config;

	return hb_side_is_valid(side) ? config->eeproms[side] : NULL;
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

int hb_eeprom_read_word(const struct device *dev, enum hb_side side,
			uint8_t word, uint16_t *value)
{
	const struct hb_config *config = dev->config;
	struct hb_data *data = dev->data;
	int ret;

	if (!hb_side_is_valid(side) || value == NULL ||
	    word >= HB_TMP117_EEPROM_WORD_COUNT) {
		return -EINVAL;
	}
	if (word == HB_TMP117_EEPROM_OFFSET_WORD) {
		return -EACCES;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = eeprom_read(config->eeproms[side],
			  (off_t)word * sizeof(*value), value, sizeof(*value));
	k_mutex_unlock(&data->lock);
	return ret;
}

int hb_eeprom_write_word(const struct device *dev, enum hb_side side,
			 uint8_t word, uint16_t value)
{
	const struct hb_config *config = dev->config;
	struct hb_data *data = dev->data;
	uint16_t readback;
	int ret;

	if (!hb_side_is_valid(side) || word >= HB_TMP117_EEPROM_WORD_COUNT) {
		return -EINVAL;
	}
	if (word == 0U || word == HB_TMP117_EEPROM_OFFSET_WORD) {
		return -EACCES;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = eeprom_write(config->eeproms[side],
			   (off_t)word * sizeof(value), &value, sizeof(value));
	if (ret == 0) {
		ret = eeprom_read(config->eeproms[side],
				  (off_t)word * sizeof(value), &readback,
				  sizeof(readback));
	}
	k_mutex_unlock(&data->lock);

	return ret == 0 && readback != value ? -EIO : ret;
}

static int hb_module_data_read_locked(const struct device *dev,
				      uint8_t bytes[HB_MODULE_DATA_SIZE])
{
	uint16_t word;
	int ret;

	for (size_t side = 0; side < 2U; ++side) {
		for (size_t slot = 0; slot < 2U; ++slot) {
			uint8_t word_index = slot == 0U ? 1U : 3U;
			size_t offset = side * 4U + slot * 2U;

			ret = eeprom_read(hb_eeprom_device(dev, (enum hb_side)side),
					  (off_t)word_index * sizeof(word), &word,
					  sizeof(word));
			if (ret != 0) {
				return ret;
			}
			bytes[offset] = (uint8_t)word;
			bytes[offset + 1U] = (uint8_t)(word >> 8);
		}
	}
	return 0;
}

int hb_module_data_read(const struct device *dev,
			struct hb_module_data *module_data)
{
	struct hb_data *data = dev->data;
	uint8_t bytes[HB_MODULE_DATA_SIZE];
	int ret;

	if (module_data == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = hb_module_data_read_locked(dev, bytes);
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
	uint8_t bytes[HB_MODULE_DATA_SIZE];
	uint16_t word;
	int ret = 0;

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
	/* Write low-side data first and the high-side version word last. */
	for (size_t side_index = 0; side_index < 2U && ret == 0;
	     ++side_index) {
		size_t side = side_index == 0U ? HB_LOW_SIDE : HB_HIGH_SIDE;

		for (size_t slot_index = 0; slot_index < 2U && ret == 0;
		     ++slot_index) {
			size_t slot = slot_index == 0U ? 1U : 0U;
			uint8_t word_index = slot == 0U ? 1U : 3U;
			size_t offset = side * 4U + slot * 2U;

			word = (uint16_t)bytes[offset] |
			       ((uint16_t)bytes[offset + 1U] << 8);
			ret = eeprom_write(hb_eeprom_device(dev, (enum hb_side)side),
				    (off_t)word_index * sizeof(word), &word,
				    sizeof(word));
			if (ret == 0) {
				uint16_t readback;

				ret = eeprom_read(
					hb_eeprom_device(dev, (enum hb_side)side),
					(off_t)word_index * sizeof(word), &readback,
					sizeof(readback));
				if (ret == 0 && readback != word) {
					ret = -EIO;
				}
			}
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

#define HB_DEFINE(inst)                                                       \
	static const struct hb_config hb_config_##inst = {                       \
		.sensors = {                                                       \
			DEVICE_DT_GET(DT_INST_PHANDLE(inst, high_side_sensor)),       \
			DEVICE_DT_GET(DT_INST_PHANDLE(inst, low_side_sensor)),        \
		},                                                                  \
		.eeproms = {                                                       \
			DEVICE_DT_GET(DT_INST_PHANDLE(inst, high_side_eeprom)),       \
			DEVICE_DT_GET(DT_INST_PHANDLE(inst, low_side_eeprom)),        \
		},                                                                  \
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
