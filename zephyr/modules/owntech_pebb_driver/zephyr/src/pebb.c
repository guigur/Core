#define DT_DRV_COMPAT owntech_pebb

#include <errno.h>
#include <stddef.h>

#include <pebb.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct pebb_config {
	const struct device *muxes[PEBB_MUX_COUNT];
	const struct device *buses[PEBB_I2C_BUS_COUNT];
	const struct device *alert_gpio;
	const struct device *control_gpio;
	const struct device *feeder;
	struct gpio_dt_spec fan_faults[PEBB_FAN_COUNT];
	struct gpio_dt_spec termination;
	struct gpio_dt_spec i2c_scl_select;
};

struct pebb_data {
	const struct device *dev;
	struct k_mutex lock;
	struct k_work_delayable fan_work;
	uint8_t critical_fans;
	uint8_t active_fan_faults;
	uint8_t latched_faults;
	enum pebb_state state;
	pebb_state_callback_t state_callback;
	void *callback_user_data;
};

static int pebb_fan_faults_poll_device(const struct device *dev)
{
	const struct pebb_config *config = dev->config;
	struct pebb_data *data = dev->data;
	pebb_state_callback_t callback = NULL;
	void *user_data = NULL;
	uint8_t faults = 0U;
	uint8_t latched = 0U;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	for (size_t i = 0; i < PEBB_FAN_COUNT; ++i) {
		ret = gpio_pin_get_dt(&config->fan_faults[i]);
		if (ret < 0) {
			if (data->critical_fans != 0U) {
				data->latched_faults |= PEBB_FAULT_FAN_IO;
			}
			break;
		}
		if (ret != 0) {
			faults |= BIT(i);
		}
	}
	if (ret >= 0) {
		data->active_fan_faults = faults;
		data->latched_faults |= faults & data->critical_fans;
	}
	if (data->state == PEBB_STATE_READY && data->latched_faults != 0U) {
		data->state = PEBB_STATE_FAULT;
		callback = data->state_callback;
		user_data = data->callback_user_data;
		latched = data->latched_faults;
	}
	k_mutex_unlock(&data->lock);

	if (callback != NULL) {
		callback(PEBB_STATE_FAULT, latched, user_data);
	}
	return ret < 0 ? ret : faults;
}

static void pebb_fan_monitor_work(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct pebb_data *data = CONTAINER_OF(delayable, struct pebb_data,
					      fan_work);

	(void)pebb_fan_faults_poll_device(data->dev);
	(void)k_work_reschedule(&data->fan_work,
				K_MSEC(CONFIG_OWNTECH_PEBB_FAN_POLL_MS));
}

static int pebb_init(const struct device *dev)
{
	const struct pebb_config *config = dev->config;
	struct pebb_data *data = dev->data;
	int ret;

	for (size_t i = 0; i < PEBB_MUX_COUNT; ++i) {
		if (!device_is_ready(config->muxes[i])) {
			return -ENODEV;
		}
	}
	for (size_t i = 0; i < PEBB_I2C_BUS_COUNT; ++i) {
		if (!device_is_ready(config->buses[i])) {
			return -ENODEV;
		}
	}
	if (!device_is_ready(config->alert_gpio) ||
	    !device_is_ready(config->control_gpio)) {
		return -ENODEV;
	}
	for (size_t i = 0; i < PEBB_FAN_COUNT; ++i) {
		if (!gpio_is_ready_dt(&config->fan_faults[i])) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->fan_faults[i], GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
	}
	if (!gpio_is_ready_dt(&config->termination) ||
	    !gpio_is_ready_dt(&config->i2c_scl_select)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&config->termination, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&config->i2c_scl_select,
				    GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}
	k_mutex_init(&data->lock);
	data->dev = dev;
	data->state = PEBB_STATE_READY;
	k_work_init_delayable(&data->fan_work, pebb_fan_monitor_work);
	(void)k_work_schedule(&data->fan_work,
			      K_MSEC(CONFIG_OWNTECH_PEBB_FAN_POLL_MS));
	return 0;
}

#define PEBB_BUS(inst, prop) DEVICE_DT_GET(DT_INST_PHANDLE(inst, prop))
#define PEBB_DEFINE(inst)                                                        \
	static struct pebb_data pebb_data_##inst;                                 \
	static const struct pebb_config pebb_config_##inst = {                    \
		.muxes = { PEBB_BUS(inst, root_i2c_mux),                         \
			   PEBB_BUS(inst, power_i2c_mux) },                         \
		.buses = { PEBB_BUS(inst, afe_main_bus),                         \
			   PEBB_BUS(inst, afe_sincos_bus),                         \
			   PEBB_BUS(inst, power_bus),                              \
			   PEBB_BUS(inst, feeder_bus),                             \
			   PEBB_BUS(inst, power_io_bus),                           \
			   PEBB_BUS(inst, hb1_bus),                                \
			   PEBB_BUS(inst, hb2_bus),                                \
			   PEBB_BUS(inst, hb3_bus) },                              \
		.alert_gpio = PEBB_BUS(inst, alert_gpio_expander),              \
		.control_gpio = PEBB_BUS(inst, control_gpio_expander),          \
		.feeder = PEBB_BUS(inst, feeder),                               \
		.fan_faults = { GPIO_DT_SPEC_INST_GET_BY_IDX(inst, fan_fault_gpios, 0), \
				GPIO_DT_SPEC_INST_GET_BY_IDX(inst, fan_fault_gpios, 1), \
				GPIO_DT_SPEC_INST_GET_BY_IDX(inst, fan_fault_gpios, 2) }, \
		.termination = GPIO_DT_SPEC_INST_GET(inst, termination_gpios),  \
		.i2c_scl_select = GPIO_DT_SPEC_INST_GET(inst, i2c_scl_select_gpios), \
	};                                                                       \
	DEVICE_DT_INST_DEFINE(inst, pebb_init, NULL, &pebb_data_##inst,          \
			      &pebb_config_##inst, POST_KERNEL,                     \
			      CONFIG_OWNTECH_PEBB_DRIVER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PEBB_DEFINE)

const struct device *pebb_get(void)
{
	return DEVICE_DT_GET(DT_INST(0, owntech_pebb));
}

bool pebb_is_ready(void)
{
	return device_is_ready(pebb_get());
}

const struct device *pebb_i2c_bus_get(enum pebb_i2c_bus_id bus)
{
	if (bus < 0 || bus >= PEBB_I2C_BUS_COUNT) {
		return NULL;
	}
	const struct pebb_config *config = pebb_get()->config;
	return config->buses[bus];
}

const struct device *pebb_mux_get(enum pebb_mux_id mux)
{
	if (mux < 0 || mux >= PEBB_MUX_COUNT) {
		return NULL;
	}
	const struct pebb_config *config = pebb_get()->config;
	return config->muxes[mux];
}

const struct device *pebb_alert_gpio_get(void)
{
	const struct pebb_config *config = pebb_get()->config;
	return config->alert_gpio;
}

const struct device *pebb_control_gpio_get(void)
{
	const struct pebb_config *config = pebb_get()->config;
	return config->control_gpio;
}

const struct device *pebb_feeder_get(void)
{
	const struct pebb_config *config = pebb_get()->config;
	return config->feeder;
}

int pebb_fan_fault_get(enum pebb_fan_id fan)
{
	if (fan < 0 || fan >= PEBB_FAN_COUNT) {
		return -EINVAL;
	}
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	const struct pebb_config *config = pebb_get()->config;
	return gpio_pin_get_dt(&config->fan_faults[fan]);
}

int pebb_fan_fault_policy_set(enum pebb_fan_id fan,
			      enum pebb_fan_fault_policy policy)
{
	if (fan < 0 || fan >= PEBB_FAN_COUNT ||
	    (policy != PEBB_FAN_FAULT_IGNORE &&
	     policy != PEBB_FAN_FAULT_CRITICAL)) {
		return -EINVAL;
	}
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	struct pebb_data *data = pebb_get()->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	if (policy == PEBB_FAN_FAULT_CRITICAL) {
		data->critical_fans |= BIT(fan);
	} else {
		data->critical_fans &= ~BIT(fan);
	}
	k_mutex_unlock(&data->lock);
	(void)k_work_reschedule(&data->fan_work, K_NO_WAIT);
	return 0;
}

int pebb_fan_fault_policy_get(enum pebb_fan_id fan)
{
	if (fan < 0 || fan >= PEBB_FAN_COUNT) {
		return -EINVAL;
	}
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	struct pebb_data *data = pebb_get()->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	int policy = (data->critical_fans & BIT(fan)) != 0U ?
		PEBB_FAN_FAULT_CRITICAL : PEBB_FAN_FAULT_IGNORE;
	k_mutex_unlock(&data->lock);
	return policy;
}

int pebb_fan_faults_poll(void)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	return pebb_fan_faults_poll_device(pebb_get());
}

int pebb_state_get(void)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	struct pebb_data *data = pebb_get()->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	int state = data->state;
	k_mutex_unlock(&data->lock);
	return state;
}

int pebb_fault_mask_get(void)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	struct pebb_data *data = pebb_get()->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	int faults = data->latched_faults;
	k_mutex_unlock(&data->lock);
	return faults;
}

int pebb_fault_acknowledge(void)
{
	int ret = pebb_fan_faults_poll();
	if (ret < 0) {
		return ret;
	}
	struct pebb_data *data = pebb_get()->data;
	pebb_state_callback_t callback = NULL;
	void *user_data = NULL;
	k_mutex_lock(&data->lock, K_FOREVER);
	if ((data->active_fan_faults & data->critical_fans) != 0U) {
		k_mutex_unlock(&data->lock);
		return -EBUSY;
	}
	if (data->state == PEBB_STATE_FAULT) {
		callback = data->state_callback;
		user_data = data->callback_user_data;
	}
	data->latched_faults = 0U;
	data->state = PEBB_STATE_READY;
	k_mutex_unlock(&data->lock);
	if (callback != NULL) {
		callback(PEBB_STATE_READY, 0U, user_data);
	}
	return 0;
}

int pebb_state_callback_set(pebb_state_callback_t callback, void *user_data)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	struct pebb_data *data = pebb_get()->data;
	k_mutex_lock(&data->lock, K_FOREVER);
	data->state_callback = callback;
	data->callback_user_data = user_data;
	k_mutex_unlock(&data->lock);
	return 0;
}

int pebb_bus_termination_set(bool enabled)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	const struct pebb_config *config = pebb_get()->config;
	return gpio_pin_set_dt(&config->termination, enabled ? 1 : 0);
}

int pebb_bus_termination_get(void)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	const struct pebb_config *config = pebb_get()->config;
	return gpio_pin_get_dt(&config->termination);
}

int pebb_i2c_scl_source_set(enum pebb_i2c_scl_source source)
{
	if (source != PEBB_I2C_SCL_PA15 && source != PEBB_I2C_SCL_PA13) {
		return -EINVAL;
	}
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	const struct pebb_config *config = pebb_get()->config;
	return gpio_pin_set_dt(&config->i2c_scl_select, source);
}

int pebb_i2c_scl_source_get(void)
{
	if (!pebb_is_ready()) {
		return -ENODEV;
	}
	const struct pebb_config *config = pebb_get()->config;
	return gpio_pin_get_dt(&config->i2c_scl_select);
}
