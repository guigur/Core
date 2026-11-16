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
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "afe.h"

LOG_MODULE_REGISTER(owntech_afe, CONFIG_OWNTECH_AFE_DRIVER_LOG_LEVEL);

#define AFE_GPIO_REG_INPUT       0x00U
#define AFE_GPIO_REG_OUTPUT      0x02U
#define AFE_GPIO_REG_POLARITY    0x04U
#define AFE_GPIO_REG_DIRECTION   0x06U

#define AFE_GPIO_LED             BIT(5)
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
#define AFE_PROVISION_HALF_US    5U

#if DT_NODE_EXISTS(DT_NODELABEL(i2c1))
PINCTRL_DT_DEFINE(DT_NODELABEL(i2c1));
#endif

/* PB15 is shared by all AFEs; only the selected expander output enables it. */
K_MUTEX_DEFINE(afe_ldac_bus_lock);

struct afe_config {
	struct i2c_dt_spec gpio_i2c;
	uint16_t dac_address;
	struct gpio_dt_spec interrupt_gpio;
	struct gpio_dt_spec address_gpios[3];
	struct gpio_dt_spec ldac_pulse_gpio;
	struct gpio_dt_spec provision_scl_gpio;
	struct gpio_dt_spec provision_sda_gpio;
	bool has_address_gpios;
	bool auto_provision;
	uint8_t gpio_layout;
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
	bool prepared;
};

static const float afe_gains[AFE_GAIN_COUNT] = {
	20.0F, 10.0F, 4.99F, 2.0F, 1.0F, 0.499F, 0.249F, 0.124F,
};

static uint16_t afe_ldac_mask(const struct afe_config *config)
{
	return config->gpio_layout == 1U ? BIT(0) : BIT(6);
}

static uint16_t afe_bandwidth_mask(const struct afe_config *config)
{
	return config->gpio_layout == 1U ? 0x000EU : 0x0007U;
}

static uint16_t afe_bandwidth_off(const struct afe_config *config)
{
	return config->gpio_layout == 1U ? BIT(4) : BIT(3);
}

static uint16_t afe_input_mask(const struct afe_config *config)
{
	return config->gpio_layout == 1U ? 0x0CC0U : 0x0C90U;
}

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
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret;
	int restore_ret;

	if (config->ldac_pulse_gpio.port != NULL) {
		k_mutex_lock(&afe_ldac_bus_lock, K_FOREVER);
		ret = afe_set_outputs_locked(dev,
				data->outputs & (uint16_t)~afe_ldac_mask(config));
		if (ret == 0) {
			ret = gpio_pin_set_dt(&config->ldac_pulse_gpio, 1);
			k_busy_wait(10U);
		}
		(void)gpio_pin_set_dt(&config->ldac_pulse_gpio, 0);
		restore_ret = afe_set_outputs_locked(dev,
				data->outputs | afe_ldac_mask(config));
		k_mutex_unlock(&afe_ldac_bus_lock);
		return ret != 0 ? ret : restore_ret;
	}

	ret = afe_set_outputs_locked(dev,
			data->outputs & (uint16_t)~afe_ldac_mask(config));
	if (ret != 0) {
		return ret;
	}

	k_busy_wait(10U);
	return afe_set_outputs_locked(dev, data->outputs | afe_ldac_mask(config));
}

static int afe_prepare_gpio_locked(const struct device *dev)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint16_t outputs;
	int ret;

	data->configured = false;

	outputs = afe_ldac_mask(config) | AFE_GPIO_GAIN_OFF |
		  afe_bandwidth_off(config) |
		  ((uint16_t)config->startup_gain << 12) |
		  ((uint16_t)config->startup_bandwidth << config->gpio_layout);

	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT, outputs);
	if (ret == 0) {
		ret = afe_gpio_write_pair(config, AFE_GPIO_REG_POLARITY, 0U);
	}
	if (ret == 0) {
		ret = afe_gpio_write_pair(config, AFE_GPIO_REG_DIRECTION,
					  afe_input_mask(config));
	}
	if (ret != 0) {
		return ret;
	}

	data->outputs = outputs;
	return 0;
}

static int afe_reset_locked(const struct device *dev)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint8_t raw[AFE_DAC_READ_SIZE];
	int ret = afe_prepare_gpio_locked(dev);

	if (ret != 0) {
		return ret;
	}

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

static int afe_prepare_platform(const struct device *dev)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	int ret;

	if (data->prepared) {
		return 0;
	}
	k_work_init_delayable(&data->identify_work, afe_identify_work_handler);
	data->dev = dev;

	if (!device_is_ready(config->gpio_i2c.bus)) {
		return -ENODEV;
	}
	if (config->ldac_pulse_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->ldac_pulse_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->ldac_pulse_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
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

	data->prepared = true;
	return 0;
}

static int afe_driver_init(const struct device *dev)
{
	int ret = afe_prepare_platform(dev);

	if (ret != 0) {
		return ret;
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

static int afe_bb_restore_i2c(void)
{
#if !DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	return -ENOTSUP;
#else
	int ret = pinctrl_apply_state(
		PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(i2c1)),
		PINCTRL_STATE_DEFAULT);

	if (ret != 0) {
		return ret;
	}
	/* GPIO bit-banging bypasses the STM32 I2C state machine. Reset it and
	 * release both lines before the next transaction through U805.
	 */
	return i2c_recover_bus(DEVICE_DT_GET(DT_NODELABEL(i2c1)));
#endif
}

static int afe_bb_set(const struct gpio_dt_spec *pin, int value)
{
	int ret = gpio_pin_set_dt(pin, value);

	k_busy_wait(AFE_PROVISION_HALF_US);
	return ret;
}

static int afe_bb_scl_high(const struct afe_config *config)
{
	int ret = afe_bb_set(&config->provision_scl_gpio, 1);

	if (ret != 0) {
		return ret;
	}
	for (unsigned int us = 0; us < 100U; us += 2U) {
		ret = gpio_pin_get_dt(&config->provision_scl_gpio);
		if (ret != 0) {
			return ret < 0 ? ret : 0;
		}
		k_busy_wait(2U);
	}
	return -ETIMEDOUT;
}

static int afe_bb_write_byte(const struct afe_config *config, uint8_t value,
			     bool pulse_after_eighth_clock)
{
	int ret;

	for (int bit = 7; bit >= 0; --bit) {
		ret = afe_bb_set(&config->provision_sda_gpio,
				 (value >> bit) & 1U);
		if (ret == 0) {
			ret = afe_bb_scl_high(config);
		}
		if (ret == 0) {
			ret = afe_bb_set(&config->provision_scl_gpio, 0);
		}
		if (ret != 0) {
			return ret;
		}
		if (pulse_after_eighth_clock && bit == 0) {
			/* SCL is low after bit 8 and before its ACK clock. */
			ret = afe_bb_set(&config->ldac_pulse_gpio, 1);
			if (ret != 0) {
				return ret;
			}
		}
	}
	ret = afe_bb_set(&config->provision_sda_gpio, 1);
	if (ret == 0) {
		ret = afe_bb_scl_high(config);
	}
	if (ret != 0) {
		return ret;
	}
	ret = gpio_pin_get_dt(&config->provision_sda_gpio);
	(void)afe_bb_set(&config->provision_scl_gpio, 0);
	return ret < 0 ? ret : (ret == 0 ? 0 : -ENXIO);
}

static int afe_bb_read_byte(const struct afe_config *config, uint8_t *value)
{
	uint8_t bits = 0U;
	int ret = afe_bb_set(&config->provision_sda_gpio, 1);

	if (ret != 0) {
		return ret;
	}
	for (unsigned int bit = 0; bit < 8U; ++bit) {
		ret = afe_bb_scl_high(config);
		if (ret != 0) {
			return ret;
		}
		ret = gpio_pin_get_dt(&config->provision_sda_gpio);
		if (ret < 0) {
			return ret;
		}
		bits = (uint8_t)((bits << 1) | (ret != 0));
		ret = afe_bb_set(&config->provision_scl_gpio, 0);
		if (ret != 0) {
			return ret;
		}
	}
	/* Figure 5-6 specifies a master ACK after the returned address byte. */
	ret = afe_bb_set(&config->provision_sda_gpio, 0);
	if (ret == 0) {
		ret = afe_bb_scl_high(config);
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_scl_gpio, 0);
	}
	(void)afe_bb_set(&config->provision_sda_gpio, 1);
	if (ret == 0) {
		*value = bits;
	}
	return ret;
}

static int afe_bb_address_read(const struct afe_config *config,
			       uint8_t *current_address)
{
#if !DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	ARG_UNUSED(config);
	ARG_UNUSED(current_address);
	return -ENOTSUP;
#else
	uint8_t reply = 0U;
	int ret;
	int restore_ret;
	bool started = false;

	ret = gpio_pin_configure_dt(&config->provision_scl_gpio,
				    GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
	if (ret != 0) {
		goto restore;
	}
	ret = gpio_pin_configure_dt(&config->provision_sda_gpio,
				    GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
	if (ret != 0) {
		goto restore;
	}
	k_sched_lock();
	ret = afe_bb_scl_high(config);
	if (ret == 0) {
		ret = gpio_pin_get_dt(&config->provision_sda_gpio);
		ret = ret < 0 ? ret : (ret == 1 ? 0 : -EBUSY);
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_sda_gpio, 0); /* START */
		started = ret == 0;
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_scl_gpio, 0);
	}
	if (ret == 0) {
		ret = afe_bb_write_byte(config, 0x00U, false); /* General call */
	}
	if (ret == 0) {
		ret = afe_bb_write_byte(config, 0x0CU, true);
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_sda_gpio, 1);
	}
	if (ret == 0) {
		ret = afe_bb_scl_high(config);
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_sda_gpio, 0); /* RESTART */
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_scl_gpio, 0);
	}
	if (ret == 0) {
		ret = afe_bb_write_byte(config, 0xC1U, false);
	}
	/* The selected DAC has seen the third byte and its ACK. */
	(void)afe_bb_set(&config->ldac_pulse_gpio, 0);
	if (ret == 0) {
		ret = afe_bb_read_byte(config, &reply);
	}
	if (started) {
		(void)afe_bb_set(&config->provision_sda_gpio, 0);
		(void)afe_bb_scl_high(config);
		(void)afe_bb_set(&config->provision_sda_gpio, 1); /* STOP */
	}
	k_sched_unlock();
restore:
	restore_ret = afe_bb_restore_i2c();
	if (restore_ret != 0) {
		return restore_ret;
	}
	if (ret != 0) {
		return ret;
	}
	/* Figure 5-6: EEPROM A2:A0, marker 1, input A2:A0, marker 0. */
	if ((reply & 0x11U) != 0x10U ||
	    ((reply >> 5) & 7U) != ((reply >> 1) & 7U)) {
		return -EIO;
	}
	*current_address = 0x60U | (reply >> 5);
	return 0;
#endif
}

static int afe_bb_address_write(const struct afe_config *config,
				uint8_t address, const uint8_t command[3])
{
#if !DT_NODE_EXISTS(DT_NODELABEL(i2c1))
	ARG_UNUSED(config);
	ARG_UNUSED(address);
	ARG_UNUSED(command);
	return -ENOTSUP;
#else
	int ret;
	int restore_ret;
	bool started = false;

	ret = gpio_pin_configure_dt(&config->provision_scl_gpio,
				    GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
	if (ret != 0) {
		goto restore;
	}
	ret = gpio_pin_configure_dt(&config->provision_sda_gpio,
				    GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
	if (ret != 0) {
		goto restore;
	}

	/* The mux was selected with normal I2C before taking over PA15/PB7. */
	k_sched_lock();
	ret = afe_bb_scl_high(config);
	if (ret == 0) {
		ret = gpio_pin_get_dt(&config->provision_sda_gpio);
		ret = ret < 0 ? ret : (ret == 1 ? 0 : -EBUSY);
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_sda_gpio, 0); /* START */
		started = ret == 0;
	}
	if (ret == 0) {
		ret = afe_bb_set(&config->provision_scl_gpio, 0);
	}
	if (ret == 0) {
		ret = afe_bb_write_byte(config, address << 1, false);
	}
	if (ret == 0) {
		ret = afe_bb_write_byte(config, command[0], true);
	}
	if (ret == 0) {
		ret = afe_bb_write_byte(config, command[1], false);
	}
	/* LDAC stays low through the 3rd byte's ACK, then returns high. */
	(void)afe_bb_set(&config->ldac_pulse_gpio, 0);
	if (ret == 0) {
		ret = afe_bb_write_byte(config, command[2], false);
	}
	/* Always release the bus and restore the hardware I2C pin function. */
	if (started) {
		(void)afe_bb_set(&config->provision_sda_gpio, 0);
		(void)afe_bb_scl_high(config);
		(void)afe_bb_set(&config->provision_sda_gpio, 1); /* STOP */
	}
	k_sched_unlock();
restore:
	restore_ret = afe_bb_restore_i2c();
	return restore_ret != 0 ? restore_ret : ret;
#endif
}

int afe_dac_provision_address_experimental(const struct device *dev,
					   uint8_t current_address)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint8_t raw[AFE_DAC_READ_SIZE];
	uint16_t outputs;
	uint8_t detected_address;
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
	if (config->ldac_pulse_gpio.port == NULL ||
	    config->provision_scl_gpio.port == NULL ||
	    config->provision_sda_gpio.port == NULL ||
	    !gpio_is_ready_dt(&config->ldac_pulse_gpio) ||
	    !gpio_is_ready_dt(&config->provision_scl_gpio) ||
	    !gpio_is_ready_dt(&config->provision_sda_gpio)) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	k_mutex_lock(&afe_ldac_bus_lock, K_FOREVER);
	ret = afe_gpio_read_pair(config, AFE_GPIO_REG_OUTPUT, &outputs);
	if (ret != 0) {
		goto out;
	}
	old_bits = current_address & 7U;
	new_bits = config->dac_address & 7U;

	/* The configured expander output selects this AFE's MOSFET. */
	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
				  outputs | afe_ldac_mask(config));
	if (ret != 0) {
		goto out;
	}
	/* No DAC may answer the address-read command before this slot is
	 * selected. Otherwise selection is stuck or another LDAC is active.
	 */
	ret = afe_bb_address_read(config, &detected_address);
	if (ret != -ENXIO) {
		ret = ret == 0 ? -EADDRINUSE : ret;
		goto restore;
	}
	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
				  outputs & (uint16_t)~afe_ldac_mask(config));
	if (ret != 0) {
		goto restore;
	}
	ret = afe_bb_address_read(config, &detected_address);
	if (ret != 0) {
		goto restore;
	}
	if (detected_address != current_address) {
		ret = -EADDRNOTAVAIL;
		goto restore;
	}
	if (i2c_read(config->gpio_i2c.bus, raw, sizeof(raw),
		     config->dac_address) == 0) {
		ret = -EADDRINUSE;
		goto restore;
	}
	const uint8_t command[] = {
		(uint8_t)(0x61U | (old_bits << 2)),
		(uint8_t)(0x62U | (new_bits << 2)),
		(uint8_t)(0x63U | (new_bits << 2)),
	};
	command_ret = afe_bb_address_write(config, current_address, command);
restore:
	(void)gpio_pin_set_dt(&config->ldac_pulse_gpio, 0);
	restore_ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
					  outputs | afe_ldac_mask(config));
	if (restore_ret != 0) {
		ret = restore_ret;
		goto out;
	}
	if (ret != 0 || command_ret != 0) {
		ret = ret != 0 ? ret : command_ret;
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
	/* Other unprovisioned DACs may still respond at the old address. */
	ret = 0;
out:
	k_mutex_unlock(&afe_ldac_bus_lock);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int afe_dac_detect_address(const struct device *dev, uint8_t *address)
{
	const struct afe_config *config = dev->config;
	struct afe_data *data = dev->data;
	uint16_t outputs;
	uint8_t unselected_address;
	int ret;
	int restore_ret;

	if (config->ldac_pulse_gpio.port == NULL ||
	    config->provision_scl_gpio.port == NULL ||
	    config->provision_sda_gpio.port == NULL ||
	    !gpio_is_ready_dt(&config->ldac_pulse_gpio) ||
	    !gpio_is_ready_dt(&config->provision_scl_gpio) ||
	    !gpio_is_ready_dt(&config->provision_sda_gpio)) {
		return -ENOTSUP;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	k_mutex_lock(&afe_ldac_bus_lock, K_FOREVER);
	ret = afe_gpio_read_pair(config, AFE_GPIO_REG_OUTPUT, &outputs);
	if (ret != 0) {
		goto out;
	}
	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
				  outputs | afe_ldac_mask(config));
	if (ret != 0) {
		goto out;
	}
	ret = afe_bb_address_read(config, &unselected_address);
	if (ret != -ENXIO) {
		ret = ret == 0 ? -EADDRINUSE : ret;
		goto restore;
	}
	ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
				  outputs & (uint16_t)~afe_ldac_mask(config));
	if (ret == 0) {
		ret = afe_bb_address_read(config, address);
	}
	restore:
	(void)gpio_pin_set_dt(&config->ldac_pulse_gpio, 0);
	restore_ret = afe_gpio_write_pair(config, AFE_GPIO_REG_OUTPUT,
					  outputs | afe_ldac_mask(config));
	if (ret == 0) {
		ret = restore_ret;
	}
out:
	k_mutex_unlock(&afe_ldac_bus_lock);
	k_mutex_unlock(&data->lock);
	return ret;
}

int afe_dac_provision_address_auto(const struct device *dev,
				   uint8_t *previous_address)
{
	const struct afe_config *config = dev->config;
	uint8_t current_address;
	int ret = afe_dac_detect_address(dev, &current_address);

	if (ret != 0) {
		return ret;
	}
	if (previous_address != NULL) {
		*previous_address = current_address;
	}
	if (current_address == config->dac_address) {
		return 0;
	}
	return afe_dac_provision_address_experimental(dev, current_address);
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
	const struct afe_config *config = dev->config;

	if (index >= AFE_BANDWIDTH_COUNT) {
		return -EINVAL;
	}
	return afe_update_outputs(dev, afe_bandwidth_mask(config),
				  (uint16_t)index << config->gpio_layout);
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
	const struct afe_config *config = dev->config;
	uint16_t value = (gain_enabled ? 0U : AFE_GPIO_GAIN_OFF) |
			 (bandwidth_enabled ? 0U : afe_bandwidth_off(config));

	return afe_update_outputs(dev,
				  AFE_GPIO_GAIN_OFF | afe_bandwidth_off(config), value);
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
	BUILD_ASSERT(!DT_INST_PROP(inst, auto_provision) ||                      \
		     DT_INST_NODE_HAS_PROP(inst, zephyr_deferred_init),            \
		     "AFE auto-provision requires zephyr,deferred-init");         \
	static const struct afe_config afe_config_##inst = {                     \
		.gpio_i2c = I2C_DT_SPEC_INST_GET(inst),                           \
		.dac_address = DT_INST_PROP(inst, dac_address),                    \
		.interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),\
		.address_gpios = {                                                 \
			AFE_ADDRESS_GPIO(inst, 0), AFE_ADDRESS_GPIO(inst, 1),         \
			AFE_ADDRESS_GPIO(inst, 2),                                    \
		},                                                                 \
		.ldac_pulse_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,                  \
						ldac_pulse_gpios, { 0 }),            \
		.provision_scl_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,               \
						provision_scl_gpios, { 0 }),          \
		.provision_sda_gpio = GPIO_DT_SPEC_INST_GET_OR(inst,               \
						provision_sda_gpios, { 0 }),          \
		.has_address_gpios = DT_INST_NODE_HAS_PROP(inst, address_gpios),   \
		.auto_provision = DT_INST_PROP(inst, auto_provision),             \
		.gpio_layout = DT_INST_ENUM_IDX(inst, gpio_layout),              \
		.startup_gain = DT_INST_PROP(inst, startup_gain),                  \
		.startup_bandwidth = DT_INST_PROP(inst, startup_bandwidth),        \
		.startup_dac_mv = DT_INST_PROP(inst, startup_dac_millivolts),      \
	};                                                                         \
	static struct afe_data afe_data_##inst = {                                \
		.lock = Z_MUTEX_INITIALIZER(afe_data_##inst.lock),              \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(inst, afe_driver_init, NULL, &afe_data_##inst,        \
			      &afe_config_##inst, POST_KERNEL,                      \
			      CONFIG_OWNTECH_AFE_DRIVER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AFE_DEFINE)

#define AFE_DEVICE_ENTRY(inst) DEVICE_DT_INST_GET(inst),
static const struct device *const afe_devices[] = {
	DT_INST_FOREACH_STATUS_OKAY(AFE_DEVICE_ENTRY)
};

static int afe_autoprovision_init(void)
{
	for (size_t index = 0; index < ARRAY_SIZE(afe_devices); ++index) {
		const struct device *dev = afe_devices[index];
		const struct afe_config *config = dev->config;
		uint8_t old_address = 0U;
		int ret;

		if (!config->auto_provision) {
			continue;
		}
		ret = afe_prepare_platform(dev);
		if (ret == 0) {
			/* A missing expander must never cause SDA/SCL takeover. */
			ret = afe_prepare_gpio_locked(dev);
		}
		if (ret == 0) {
			ret = afe_dac_provision_address_auto(dev, &old_address);
		}
		if (ret == 0) {
			ret = device_init(dev);
		}
		if (ret == 0) {
			LOG_INF("%s DAC 0x%02x -> 0x%02x; ready", dev->name,
				old_address, config->dac_address);
		} else {
			LOG_WRN("%s auto-provision skipped/failed (%d)",
				dev->name, ret);
		}
	}
	return 0;
}

SYS_INIT(afe_autoprovision_init, APPLICATION, 91);
