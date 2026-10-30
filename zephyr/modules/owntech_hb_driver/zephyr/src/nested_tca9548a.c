/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define DT_DRV_COMPAT owntech_tca9548a_nested

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(owntech_nested_tca9548a, CONFIG_I2C_LOG_LEVEL);

struct nested_tca9548a_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec reset_gpio;
};

struct nested_tca9548a_data {
	struct k_mutex lock;
	uint8_t selected_mask;
};

struct nested_tca9548a_channel_config {
	const struct device *root;
	uint8_t channel_mask;
};

static int nested_tca9548a_select(const struct device *root, uint8_t mask)
{
	const struct nested_tca9548a_config *config = root->config;
	struct nested_tca9548a_data *data = root->data;
	int ret;

	if (data->selected_mask == mask) {
		return 0;
	}

	ret = i2c_write_dt(&config->i2c, &mask, sizeof(mask));
	if (ret == 0) {
		data->selected_mask = mask;
	}
	return ret;
}

static int nested_tca9548a_configure(const struct device *dev,
				      uint32_t dev_config)
{
	const struct nested_tca9548a_channel_config *channel = dev->config;
	const struct nested_tca9548a_config *root_config = channel->root->config;

	return i2c_configure(root_config->i2c.bus, dev_config);
}

static int nested_tca9548a_transfer(const struct device *dev,
				     struct i2c_msg *messages,
				     uint8_t message_count, uint16_t address)
{
	const struct nested_tca9548a_channel_config *channel = dev->config;
	const struct nested_tca9548a_config *root_config = channel->root->config;
	struct nested_tca9548a_data *data = channel->root->data;
	int ret;

	ret = k_mutex_lock(&data->lock, K_MSEC(5000));
	if (ret != 0) {
		return ret;
	}

	ret = nested_tca9548a_select(channel->root, channel->channel_mask);
	if (ret == 0) {
		ret = i2c_transfer(root_config->i2c.bus, messages, message_count,
				   address);
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static const struct i2c_driver_api nested_tca9548a_channel_api = {
	.configure = nested_tca9548a_configure,
	.transfer = nested_tca9548a_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int nested_tca9548a_root_init(const struct device *dev)
{
	const struct nested_tca9548a_config *config = dev->config;
	struct nested_tca9548a_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->i2c.bus)) {
		return -ENODEV;
	}

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->reset_gpio,
					    GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			return ret;
		}
	}

	data->selected_mask = 0U;
	return 0;
}

static int nested_tca9548a_channel_init(const struct device *dev)
{
	const struct nested_tca9548a_channel_config *config = dev->config;

	return device_is_ready(config->root) ? 0 : -ENODEV;
}

BUILD_ASSERT(CONFIG_OWNTECH_NESTED_TCA9548A_CHANNEL_INIT_PRIORITY >
	     CONFIG_OWNTECH_NESTED_TCA9548A_ROOT_INIT_PRIORITY,
	     "nested TCA9548A channels must initialize after their root");

#define NESTED_TCA9548A_CHANNEL_DEFINE(node_id)                              \
	BUILD_ASSERT(DT_REG_ADDR(node_id) < 8,                                  \
		     "TCA9548A channel must be in range 0..7");                 \
	static const struct nested_tca9548a_channel_config                      \
		nested_tca9548a_channel_config_##node_id = {                     \
			.root = DEVICE_DT_GET(DT_PARENT(node_id)),                   \
			.channel_mask = BIT(DT_REG_ADDR(node_id)),                   \
	};                                                                        \
	DEVICE_DT_DEFINE(node_id, nested_tca9548a_channel_init, NULL, NULL,       \
			 &nested_tca9548a_channel_config_##node_id, POST_KERNEL,   \
			 CONFIG_OWNTECH_NESTED_TCA9548A_CHANNEL_INIT_PRIORITY,     \
			 &nested_tca9548a_channel_api);

#define NESTED_TCA9548A_DEFINE(inst)                                        \
	static const struct nested_tca9548a_config                              \
		nested_tca9548a_config_##inst = {                                \
			.i2c = I2C_DT_SPEC_INST_GET(inst),                           \
			.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios,    \
							  { 0 }),                \
	};                                                                        \
	static struct nested_tca9548a_data nested_tca9548a_data_##inst = {       \
		.lock = Z_MUTEX_INITIALIZER(nested_tca9548a_data_##inst.lock),    \
	};                                                                        \
	I2C_DEVICE_DT_INST_DEFINE(inst, nested_tca9548a_root_init, NULL,          \
				  &nested_tca9548a_data_##inst,                     \
				  &nested_tca9548a_config_##inst, POST_KERNEL,       \
				  CONFIG_OWNTECH_NESTED_TCA9548A_ROOT_INIT_PRIORITY, \
				  NULL);                                            \
	DT_FOREACH_CHILD(DT_DRV_INST(inst), NESTED_TCA9548A_CHANNEL_DEFINE)

DT_INST_FOREACH_STATUS_OKAY(NESTED_TCA9548A_DEFINE)
