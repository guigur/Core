/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define DT_DRV_COMPAT owntech_afe_v3

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "afe.h"

LOG_MODULE_REGISTER(owntech_afe, CONFIG_OWNTECH_AFE_DRIVER_LOG_LEVEL);

#define AFE_GPIO_REG_INPUT       0x00U
#define AFE_GPIO_REG_OUTPUT      0x02U
#define AFE_GPIO_REG_POLARITY    0x04U
#define AFE_GPIO_REG_DIRECTION   0x06U

#define AFE_GPIO_INPUT_MASK      0x0C90U
#define AFE_GPIO_LDAC            BIT(6)
#define AFE_GPIO_LED             BIT(5)
#define AFE_GPIO_BANDWIDTH_MASK  0x0007U
#define AFE_GPIO_BANDWIDTH_OFF   BIT(3)
#define AFE_GPIO_INPUT_MINUS     BIT(8)
#define AFE_GPIO_INPUT_PLUS      BIT(9)
#define AFE_GPIO_GAIN_MASK       0x7000U
#define AFE_GPIO_GAIN_OFF        BIT(15)

#define AFE_DAC_READ_SIZE        24U
#define AFE_DAC_MAX_MV           4095U
#define AFE_DAC_READY            BIT(7)
#define AFE_DAC_INTERNAL_REF     BIT(7)
#define AFE_DAC_GAIN_X2          BIT(4)
#define AFE_IDENTIFY_DEFAULT_HZ  2U
#define AFE_IDENTIFY_DEFAULT_MS  3000U
#define AFE_IDENTIFY_MAX_HZ      20U

struct afe_config {
	struct i2c_dt_spec gpio_i2c;
	uint16_t dac_address;
	struct gpio_dt_spec interrupt_gpio;
	struct gpio_dt_spec address_gpios[3];
	bool has_address_gpios;
	uint8_t startup_gain;
	uint8_t startup_bandwidth;
	uint16_t startup_dac_mv[AFE_DAC_CHANNEL_COUNT];
};

struct afe_data {
	struct k_mutex lock;
	struct k_work_delayable identify_work;
	const struct device *dev;
	int64_t identify_deadline_ms;
	uint32_t identify_half_period_ms;
	uint16_t outputs;
	bool configured;
	bool identifying;
};

static const float afe_gains[AFE_GAIN_COUNT] = {
	20.0F, 10.0F, 4.99F, 2.0F, 1.0F, 0.499F, 0.249F, 0.124F,
};

static int afe_gpio_read_pair(const struct afe_config *config, uint8_t reg,
			      uint16_t *value)
{
	uint8_t bytes[2];
	int ret;

	ret = i2c_write_read_dt(&config->gpio_i2c, &reg, sizeof(reg), bytes,
				sizeof(bytes));
	if (ret == 0) {
		*value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
	}

	return ret;
}

static int afe_gpio_write_pair(const struct afe_config *config, uint8_t reg,
			       uint16_t value)
{
	const uint8_t bytes[] = { reg, (uint8_t)value, (uint8_t)(value >> 8) };
	uint16_t readback;
	int ret;

	ret = i2c_write_dt(&config->gpio_i2c, bytes, sizeof(bytes));
	if (ret != 0) {
		return ret;
	}

	ret = afe_gpio_read_pair(config, reg, &readback);
	if (ret != 0) {
		return ret;
	}

	return readback == value ? 0 : -EIO;
}

static int afe_dac_read_raw(const struct afe_config *config,
			    uint8_t raw[AFE_DAC_READ_SIZE])
{
	return i2c_read(config->gpio_i2c.bus, raw, AFE_DAC_READ_SIZE,
			config->dac_address);
}

static int afe_dac_write_raw(const struct afe_config *config, uint8_t channel,
			     uint16_t millivolts, bool deferred)
{
	const uint8_t bytes[] = {
		(uint8_t)(0x40U | (channel << 1) | (deferred ? 1U : 0U)),
		(uint8_t)(0x90U | (millivolts >> 8)),
		(uint8_t)millivolts,
	};
	uint8_t raw[AFE_DAC_READ_SIZE];
	uint8_t offset = channel * 6U;
	int ret;

	ret = i2c_write(config->gpio_i2c.bus, bytes, sizeof(bytes),
			config->dac_address);
	if (ret != 0) {
		return ret;
	}

	ret = afe_dac_read_raw(config, raw);
	if (ret != 0) {
		return ret;
	}

	if ((raw[offset] & AFE_DAC_READY) == 0U ||
	    ((raw[offset] >> 4) & 0x03U) != channel) {
		return -EIO;
	}

	/* Deferred writes may keep reporting the active value until LDAC. */
	if (!deferred &&
	    (raw[offset + 1U] != bytes[1] || raw[offset + 2U] != bytes[2])) {
		return -EIO;
	}

	return 0;
}

static int afe_dac_verify(const struct afe_config *config,
			  const uint16_t millivolts[AFE_DAC_CHANNEL_COUNT])
{
	uint8_t raw[AFE_DAC_READ_SIZE];
	int ret = afe_dac_read_raw(config, raw);

	if (ret != 0) {
		return ret;
	}

	for (uint8_t channel = 0; channel < AFE_DAC_CHANNEL_COUNT; ++channel) {
		uint8_t offset = channel * 6U;
		uint8_t high = 0x90U | (millivolts[channel] >> 8);

		if ((raw[offset] & AFE_DAC_READY) == 0U ||
		    ((raw[offset] >> 4) & 0x03U) != channel ||
		    raw[offset + 1U] != high ||
		    raw[offset + 2U] != (uint8_t)millivolts[channel]) {
			return -EIO;
		}
	}

	return 0;
}

static int afe_set_outputs_locked(const struct device *dev, uint16_t outputs)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT, outputs);

	if (ret == 0) {
		data->outputs = outputs;
	} else {
		data->configured = false;
	}

	return ret;
}

static int afe_update_outputs_locked(const struct device *dev, uint16_t mask,
				     uint16_t value)
{
	struct afe_data *data = dev->data;

	if (!data->configured) {
		return -EACCES;
	}

	return afe_set_outputs_locked(dev,
			(data->outputs & (uint16_t)~mask) | (value & mask));
}

static void afe_identify_work_handler(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct afe_data *data = CONTAINER_OF(delayable, struct afe_data,
						 identify_work);
	int64_t remaining;
	uint32_t delay_ms;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->identifying) {
		goto out;
	}
	remaining = data->identify_deadline_ms == 0 ? INT64_MAX :
		data->identify_deadline_ms - k_uptime_get();
	if (remaining <= 0) {
		data->identifying = false;
		ret = afe_update_outputs_locked(data->dev, AFE_GPIO_LED, 0U);
		if (ret != 0) {
			LOG_ERR("%s identify LED stop failed (%d)", data->dev->name,
				ret);
		}
		goto out;
	}
	if (data->identify_half_period_ms != 0U) {
		uint16_t next = (data->outputs & AFE_GPIO_LED) == 0U ?
				AFE_GPIO_LED : 0U;

		ret = afe_update_outputs_locked(data->dev, AFE_GPIO_LED, next);
		if (ret != 0) {
			data->identifying = false;
			LOG_ERR("%s identify LED update failed (%d)",
				data->dev->name, ret);
			goto out;
		}
	}
	if (data->identify_half_period_ms == 0U &&
	    data->identify_deadline_ms == 0) {
		goto out;
	}
	delay_ms = data->identify_half_period_ms == 0U ?
		(uint32_t)MIN(remaining, (int64_t)UINT32_MAX) :
		(uint32_t)MIN(remaining,
			      (int64_t)data->identify_half_period_ms);
	(void)k_work_reschedule(&data->identify_work, K_MSEC(delay_ms));
out:
	k_mutex_unlock(&data->lock);
}

static int afe_latch_locked(const struct device *dev)
{
	struct afe_data *data = dev->data;
	int ret;

	ret = afe_set_outputs_locked(dev, data->outputs & (uint16_t)~AFE_GPIO_LDAC);
	if (ret != 0) {
		return ret;
	}

	k_busy_wait(10U);
	return afe_set_outputs_locked(dev, data->outputs | AFE_GPIO_LDAC);
}

static int afe_reset_locked(const struct device *dev)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint8_t raw[AFE_DAC_READ_SIZE];
	uint16_t outputs;
	int ret;

	data->configured = false;

	outputs = AFE_GPIO_LDAC | AFE_GPIO_GAIN_OFF | AFE_GPIO_BANDWIDTH_OFF |
		  ((uint16_t)config->startup_gain << 12) |
		  config->startup_bandwidth;

	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT, outputs);
	if (ret == 0) {
		ret = afe_gpio_write_pair(config, AFE_GPIO_REG_POLARITY, 0U);
	}
	if (ret == 0) {
		ret = afe_gpio_write_pair(config, AFE_GPIO_REG_DIRECTION,
					  AFE_GPIO_INPUT_MASK);
	}
	if (ret != 0) {
		return ret;
	}

	data->outputs = outputs;

	/* Initialize the expander even when the DAC has not been addressed yet. */
	ret = afe_dac_read_raw(config, raw);
	if (ret != 0 || (raw[0] & AFE_DAC_READY) == 0U) {
		return ret != 0 ? ret : -EBUSY;
	}

	for (uint8_t channel = 0; channel < AFE_DAC_CHANNEL_COUNT; ++channel) {
		ret = afe_dac_write_raw(config, channel,
					config->startup_dac_mv[channel], true);
		if (ret != 0) {
			return ret;
		}
	}

	ret = afe_latch_locked(dev);
	if (ret == 0) {
		ret = afe_dac_verify(config, config->startup_dac_mv);
	}
	if (ret == 0) {
		data->configured = true;
	}

	return ret;
}

static int afe_driver_init(const struct device *dev)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->identify_work, afe_identify_work_handler);
	data->dev = dev;

	if (!device_is_ready(config->gpio_i2c.bus)) {
		return -ENODEV;
	}

	if (config->has_address_gpios) {
		uint8_t address_bits = (uint8_t)(config->gpio_i2c.addr - 0x20U);

		for (uint8_t index = 0; index < ARRAY_SIZE(config->address_gpios);
		     ++index) {
			if (!gpio_is_ready_dt(&config->address_gpios[index])) {
				return -ENODEV;
			}
			ret = gpio_pin_configure_dt(&config->address_gpios[index],
						GPIO_OUTPUT_INACTIVE);
			if (ret == 0) {
				ret = gpio_pin_set_dt(&config->address_gpios[index],
						      (address_bits >> index) & 1U);
			}
			if (ret != 0) {
				return ret;
			}
		}
	}

	if (config->interrupt_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->interrupt_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
	}

	ret = afe_reset_locked(dev);
	if (ret != 0) {
		LOG_ERR("%s initialization failed (%d)", dev->name, ret);
	}
	return ret;
}

int afe_reset(const struct device *dev)
{
	struct afe_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->identifying = false;
	(void)k_work_cancel_delayable(&data->identify_work);
	ret = afe_reset_locked(dev);
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_dac_provision_address_experimental(const struct device *dev,
					   uint8_t current_address)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint8_t raw[AFE_DAC_READ_SIZE];
	uint16_t outputs;
	uint8_t old_bits;
	uint8_t new_bits;
	int ret;
	int restore_ret;
	int command_ret = -EIO;

	if (current_address < 0x60U || current_address > 0x67U ||
	    config->dac_address < 0x60U || config->dac_address > 0x67U ||
	    current_address == config->dac_address) {
		return -EINVAL;
	}
	if (!device_is_ready(config->gpio_i2c.bus)) {
		return -ENODEV;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = afe_gpio_read_pair(config, AFE_GPIO_REG_OUTPUT, &outputs);
	if (ret != 0) {
		goto out;
	}
	ret = i2c_read(config->gpio_i2c.bus, raw, sizeof(raw),
		       current_address);
	if (ret != 0) {
		goto out;
	}
	old_bits = current_address & 7U;
	new_bits = config->dac_address & 7U;
	for (uint8_t channel = 0; channel < AFE_DAC_CHANNEL_COUNT; ++channel) {
		uint8_t header = raw[channel * 6U];

		if ((header & AFE_DAC_READY) == 0U ||
		    ((header >> 4) & 3U) != channel ||
		    (header & 7U) != old_bits) {
			ret = -EIO;
			goto out;
		}
	}
	if (i2c_read(config->gpio_i2c.bus, raw, sizeof(raw),
		     config->dac_address) == 0) {
		ret = -EADDRINUSE;
		goto out;
	}

	/* Closest possible attempt: LDAC falls before, not during, byte two. */
	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
				  outputs | AFE_GPIO_LDAC);
	if (ret != 0) {
		goto out;
	}
	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
				  outputs & (uint16_t)~AFE_GPIO_LDAC);
	if (ret != 0) {
		goto restore;
	}
	const uint8_t command[] = {
		(uint8_t)(0x61U | (old_bits << 2)),
		(uint8_t)(0x62U | (new_bits << 2)),
		(uint8_t)(0x63U | (new_bits << 2)),
	};
	command_ret = i2c_write(config->gpio_i2c.bus, command,
				sizeof(command), current_address);
restore:
	restore_ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
					  outputs | AFE_GPIO_LDAC);
	if (restore_ret != 0) {
		ret = restore_ret;
		goto out;
	}
	k_msleep(100);
	ret = i2c_read(config->gpio_i2c.bus, raw, sizeof(raw),
		       config->dac_address);
	if (ret != 0 || (raw[0] & 7U) != new_bits ||
	    (raw[0] & AFE_DAC_READY) == 0U) {
		ret = command_ret != 0 ? command_ret : -EIO;
		goto out;
	}
	ret = i2c_read(config->gpio_i2c.bus, raw, sizeof(raw),
		       current_address) == 0 ? -EADDRINUSE : 0;
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int afe_update_outputs(const struct device *dev, uint16_t mask,
			      uint16_t value)
{
	struct afe_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = afe_update_outputs_locked(dev, mask, value);
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_set_gain(const struct device *dev, uint8_t index)
{
	if (index >= AFE_GAIN_COUNT) {
		return -EINVAL;
	}
	return afe_update_outputs(dev, AFE_GPIO_GAIN_MASK, (uint16_t)index << 12);
}

float afe_gain_value(uint8_t index)
{
	return index < AFE_GAIN_COUNT ? afe_gains[index] : 0.0F;
}

int afe_set_bandwidth(const struct device *dev, uint8_t index)
{
	if (index >= AFE_BANDWIDTH_COUNT) {
		return -EINVAL;
	}
	return afe_update_outputs(dev, AFE_GPIO_BANDWIDTH_MASK, index);
}

int afe_set_inputs(const struct device *dev, bool plus_self, bool minus_self)
{
	uint16_t value = (plus_self ? AFE_GPIO_INPUT_PLUS : 0U) |
			 (minus_self ? AFE_GPIO_INPUT_MINUS : 0U);

	return afe_update_outputs(dev,
				  AFE_GPIO_INPUT_PLUS | AFE_GPIO_INPUT_MINUS, value);
}

int afe_set_enabled(const struct device *dev, bool gain_enabled,
		    bool bandwidth_enabled)
{
	uint16_t value = (gain_enabled ? 0U : AFE_GPIO_GAIN_OFF) |
			 (bandwidth_enabled ? 0U : AFE_GPIO_BANDWIDTH_OFF);

	return afe_update_outputs(dev,
				  AFE_GPIO_GAIN_OFF | AFE_GPIO_BANDWIDTH_OFF, value);
}

int afe_set_led(const struct device *dev, bool enabled)
{
	struct afe_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->identifying = false;
	(void)k_work_cancel_delayable(&data->identify_work);
	ret = afe_update_outputs_locked(dev, AFE_GPIO_LED,
					  enabled ? AFE_GPIO_LED : 0U);
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_identify_start(const struct device *dev, uint8_t frequency_hz,
		       uint32_t duration_ms)
{
	struct afe_data *data = dev->data;
	uint32_t delay_ms;
	int ret;

	if (frequency_hz > AFE_IDENTIFY_MAX_HZ) {
		return -EINVAL;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	data->identifying = false;
	(void)k_work_cancel_delayable(&data->identify_work);
	ret = afe_update_outputs_locked(dev, AFE_GPIO_LED, AFE_GPIO_LED);
	if (ret == 0) {
		data->identify_half_period_ms = frequency_hz == 0U ? 0U :
			1000U / (2U * frequency_hz);
		data->identify_deadline_ms = duration_ms == 0U ? 0 :
			k_uptime_get() + duration_ms;
		data->identifying = true;
		if (frequency_hz != 0U || duration_ms != 0U) {
			delay_ms = data->identify_half_period_ms == 0U ?
				duration_ms : data->identify_half_period_ms;
			if (duration_ms != 0U) {
				delay_ms = MIN(delay_ms, duration_ms);
			}
			(void)k_work_reschedule(&data->identify_work,
						K_MSEC(delay_ms));
		}
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_identify(const struct device *dev)
{
	return afe_identify_start(dev, AFE_IDENTIFY_DEFAULT_HZ,
				  AFE_IDENTIFY_DEFAULT_MS);
}

int afe_identify_stop(const struct device *dev)
{
	struct afe_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->identifying = false;
	(void)k_work_cancel_delayable(&data->identify_work);
	ret = afe_update_outputs_locked(dev, AFE_GPIO_LED, 0U);
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_dac_set_mv(const struct device *dev, enum afe_dac_channel channel,
		   uint16_t millivolts, bool defer_update)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret;

	if ((unsigned int)channel >= AFE_DAC_CHANNEL_COUNT ||
	    millivolts > AFE_DAC_MAX_MV) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = data->configured ?
		afe_dac_write_raw(config, channel, millivolts, defer_update) :
		-EACCES;
	if (ret != 0 && ret != -EACCES) {
		data->configured = false;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_dac_set_all_mv(const struct device *dev,
		       const uint16_t millivolts[AFE_DAC_CHANNEL_COUNT])
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret = 0;

	if (millivolts == NULL) {
		return -EINVAL;
	}
	for (uint8_t channel = 0; channel < AFE_DAC_CHANNEL_COUNT; ++channel) {
		if (millivolts[channel] > AFE_DAC_MAX_MV) {
			return -EINVAL;
		}
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->configured) {
		ret = -EACCES;
		goto out;
	}
	for (uint8_t channel = 0; channel < AFE_DAC_CHANNEL_COUNT; ++channel) {
		ret = afe_dac_write_raw(config, channel, millivolts[channel], true);
		if (ret != 0) {
			goto fail;
		}
	}
	ret = afe_latch_locked(dev);
	if (ret == 0) {
		ret = afe_dac_verify(config, millivolts);
	}
	if (ret != 0) {
	fail:
		data->configured = false;
	}
out:
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_latch(const struct device *dev)
{
	struct afe_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = data->configured ? afe_latch_locked(dev) : -EACCES;
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_dac_read(const struct device *dev,
		 struct afe_dac_state state[AFE_DAC_CHANNEL_COUNT])
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint8_t raw[AFE_DAC_READ_SIZE];
	int ret;

	if (state == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = afe_dac_read_raw(config, raw);
	k_mutex_unlock(&data->lock);
	if (ret != 0) {
		return ret;
	}

	for (uint8_t channel = 0; channel < AFE_DAC_CHANNEL_COUNT; ++channel) {
		uint8_t offset = channel * 6U;
		state[channel].value_mv =
			((uint16_t)(raw[offset + 1U] & 0x0FU) << 8) |
			raw[offset + 2U];
		state[channel].eeprom_value_mv =
			((uint16_t)(raw[offset + 4U] & 0x0FU) << 8) |
			raw[offset + 5U];
		state[channel].ready = (raw[offset] & AFE_DAC_READY) != 0U;
		state[channel].internal_reference =
			(raw[offset + 1U] & AFE_DAC_INTERNAL_REF) != 0U;
		state[channel].gain_x2 =
			(raw[offset + 1U] & AFE_DAC_GAIN_X2) != 0U;
		state[channel].power_down = (raw[offset + 1U] >> 5) & 0x03U;
	}

	return 0;
}

int afe_gpio_read(const struct device *dev, struct afe_gpio_state *state)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret;

	if (state == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = afe_gpio_read_pair(config, AFE_GPIO_REG_INPUT, &state->inputs);
	if (ret == 0) {
		ret = afe_gpio_read_pair(config, AFE_GPIO_REG_OUTPUT, &state->outputs);
	}
	if (ret == 0) {
		ret = afe_gpio_read_pair(config, AFE_GPIO_REG_POLARITY,
					 &state->polarity);
	}
	if (ret == 0) {
		ret = afe_gpio_read_pair(config, AFE_GPIO_REG_DIRECTION,
					 &state->directions);
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_interrupt_acknowledge(const struct device *dev, uint16_t *inputs)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret;

	if (inputs == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = afe_gpio_read_pair(config, AFE_GPIO_REG_INPUT, inputs);
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_interrupt_is_pending(const struct device *dev)
{
	const struct afe_config *config = dev->config;

	if (config->interrupt_gpio.port == NULL) {
		return -ENOTSUP;
	}

	return gpio_pin_get_dt(&config->interrupt_gpio);
}

#define AFE_ADDRESS_GPIO(inst, index) \
	GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, address_gpios, index, { 0 })

#define AFE_DEFINE(inst)                                                       \
	BUILD_ASSERT(DT_INST_REG_ADDR(inst) >= 0x20 &&                          \
		     DT_INST_REG_ADDR(inst) <= 0x27,                              \
		     "AFE TCA9555 address must be in range 0x20..0x27");          \
	BUILD_ASSERT(DT_INST_PROP(inst, dac_address) <= 0x7f,                    \
		     "AFE DAC address must be a 7-bit I2C address");               \
	BUILD_ASSERT(DT_INST_PROP(inst, dac_address) >= 0x08,                    \
		     "AFE DAC address is in the reserved I2C address range");       \
	BUILD_ASSERT(DT_INST_PROP(inst, startup_gain) < AFE_GAIN_COUNT,          \
		     "AFE startup-gain must be in range 0..7");                    \
	BUILD_ASSERT(DT_INST_PROP(inst, startup_bandwidth) <                     \
		     AFE_BANDWIDTH_COUNT,                                          \
		     "AFE startup-bandwidth must be in range 0..7");               \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, startup_dac_millivolts) == 4,        \
		     "AFE startup-dac-millivolts requires four values");           \
	BUILD_ASSERT(DT_INST_PROP_BY_IDX(inst, startup_dac_millivolts, 0) <=     \
		     AFE_DAC_MAX_MV &&                                              \
		     DT_INST_PROP_BY_IDX(inst, startup_dac_millivolts, 1) <=        \
		     AFE_DAC_MAX_MV &&                                              \
		     DT_INST_PROP_BY_IDX(inst, startup_dac_millivolts, 2) <=        \
		     AFE_DAC_MAX_MV &&                                              \
		     DT_INST_PROP_BY_IDX(inst, startup_dac_millivolts, 3) <=        \
		     AFE_DAC_MAX_MV,                                                \
		     "AFE startup DAC values must be in range 0..4095 mV");        \
	BUILD_ASSERT(DT_INST_PROP_LEN_OR(inst, address_gpios, 0) == 0 ||         \
		     DT_INST_PROP_LEN_OR(inst, address_gpios, 0) == 3,              \
		     "AFE address-gpios requires A0, A1 and A2");                  \
	static const struct afe_config afe_config_##inst = {                     \
		.gpio_i2c = I2C_DT_SPEC_INST_GET(inst),                           \
		.dac_address = DT_INST_PROP(inst, dac_address),                    \
		.interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),\
		.address_gpios = {                                                 \
			AFE_ADDRESS_GPIO(inst, 0), AFE_ADDRESS_GPIO(inst, 1),         \
			AFE_ADDRESS_GPIO(inst, 2),                                    \
		},                                                                 \
		.has_address_gpios = DT_INST_NODE_HAS_PROP(inst, address_gpios),   \
		.startup_gain = DT_INST_PROP(inst, startup_gain),                  \
		.startup_bandwidth = DT_INST_PROP(inst, startup_bandwidth),        \
		.startup_dac_mv = DT_INST_PROP(inst, startup_dac_millivolts),      \
	};                                                                         \
	static struct afe_data afe_data_##inst;                                    \
	DEVICE_DT_INST_DEFINE(inst, afe_driver_init, NULL, &afe_data_##inst,        \
			      &afe_config_##inst, POST_KERNEL,                      \
			      CONFIG_OWNTECH_AFE_DRIVER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AFE_DEFINE)
