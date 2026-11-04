#ifndef OWNTECH_PEBB_H_
#define OWNTECH_PEBB_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Logical buses; the I2C mux drivers select the physical channels. */
enum pebb_i2c_bus_id {
	PEBB_I2C_AFE_MAIN,
	PEBB_I2C_AFE_SINCOS,
	PEBB_I2C_POWER,
	PEBB_I2C_FEEDER,
	PEBB_I2C_POWER_IO,
	PEBB_I2C_HB_1,
	PEBB_I2C_HB_2,
	PEBB_I2C_HB_3,
	PEBB_I2C_BUS_COUNT,
};

enum pebb_mux_id {
	PEBB_MUX_ROOT,
	PEBB_MUX_POWER,
	PEBB_MUX_COUNT,
};

enum pebb_fan_id {
	PEBB_FAN_1,
	PEBB_FAN_2,
	PEBB_FAN_3,
	PEBB_FAN_COUNT,
};

enum pebb_i2c_scl_source {
	PEBB_I2C_SCL_PA15 = 0,
	PEBB_I2C_SCL_PA13 = 1,
};

enum pebb_fan_fault_policy {
	PEBB_FAN_FAULT_IGNORE = 0,
	PEBB_FAN_FAULT_CRITICAL = 1,
};

enum pebb_state {
	PEBB_STATE_READY = 0,
	PEBB_STATE_FAULT = 1,
};

#define PEBB_FAULT_FAN_1 (1U << PEBB_FAN_1)
#define PEBB_FAULT_FAN_2 (1U << PEBB_FAN_2)
#define PEBB_FAULT_FAN_3 (1U << PEBB_FAN_3)
#define PEBB_FAULT_FAN_IO (1U << 3)

/* Called in the polling or acknowledgment caller's context. Periodic polls
 * run on the Zephyr system work queue; keep the callback short. */
typedef void (*pebb_state_callback_t)(enum pebb_state state,
				      uint8_t latched_faults, void *user_data);

const struct device *pebb_get(void);
bool pebb_is_ready(void);
const struct device *pebb_i2c_bus_get(enum pebb_i2c_bus_id bus);
const struct device *pebb_mux_get(enum pebb_mux_id mux);
const struct device *pebb_alert_gpio_get(void);
const struct device *pebb_control_gpio_get(void);
const struct device *pebb_feeder_get(void);

/* Returns 1 for a fan fault, 0 for healthy tach activity, or a negative errno. */
int pebb_fan_fault_get(enum pebb_fan_id fan);

/* Each fan is ignored until explicitly marked critical after spin-up. */
int pebb_fan_fault_policy_set(enum pebb_fan_id fan,
			      enum pebb_fan_fault_policy policy);
int pebb_fan_fault_policy_get(enum pebb_fan_id fan);

/* Poll now; returns the raw three-bit fan fault mask or a negative errno. */
int pebb_fan_faults_poll(void);

/* A critical fan fault latches PEBB_STATE_FAULT until acknowledged. */
int pebb_state_get(void);
int pebb_fault_mask_get(void);
int pebb_fault_acknowledge(void);
int pebb_state_callback_set(pebb_state_callback_t callback, void *user_data);

/* TERM120 drives the CAN and RS485 120-ohm switches together. */
int pebb_bus_termination_set(bool enabled);
int pebb_bus_termination_get(void);

/* U726 selects PA15 (default) or PA13 as SCL for the downstream I2C tree. */
int pebb_i2c_scl_source_set(enum pebb_i2c_scl_source source);
int pebb_i2c_scl_source_get(void);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_PEBB_H_ */
