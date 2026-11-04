/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef OWNTECH_AFE_H_
#define OWNTECH_AFE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AFE_GAIN_COUNT 8U
#define AFE_BANDWIDTH_COUNT 8U
#define AFE_DAC_CHANNEL_COUNT 4U

/** MCP4728 channels as connected on the AFE v3 board. */
enum afe_dac_channel {
	AFE_DAC_SELF_PLUS = 0,
	AFE_DAC_SELF_MINUS = 1,
	AFE_DAC_INPUT_OFFSET = 2,
	AFE_DAC_OUTPUT_OFFSET = 3,
};

/** Complete TCA9555 register snapshot. */
struct afe_gpio_state {
	uint16_t inputs;
	uint16_t outputs;
	uint16_t polarity;
	uint16_t directions;
};

/** MCP4728 volatile and EEPROM values for one channel. */
struct afe_dac_state {
	uint16_t value_mv;
	uint16_t eeprom_value_mv;
	bool ready;
	bool internal_reference;
	bool gain_x2;
	uint8_t power_down;
};

/**
 * Reinitialize an AFE with its devicetree startup values.
 *
 * Gain and bandwidth paths remain disconnected after this call. No MCP4728
 * EEPROM location is written.
 */
int afe_reset(const struct device *dev);

/** Select gain index 0..7. See afe_gain_value(). */
int afe_set_gain(const struct device *dev, uint8_t index);

/** Return the nominal gain associated with an index, or 0 for an invalid one. */
float afe_gain_value(uint8_t index);

/** Select bandwidth index 0..7. */
int afe_set_bandwidth(const struct device *dev, uint8_t index);

/** Select external (false) or onboard self-test DAC (true) for each input. */
int afe_set_inputs(const struct device *dev, bool plus_self, bool minus_self);

/** Independently connect or disconnect the gain and bandwidth stages. */
int afe_set_enabled(const struct device *dev, bool gain_enabled,
		    bool bandwidth_enabled);

/** Control the AFE status LED. */
int afe_set_led(const struct device *dev, bool enabled);

/** Identify this AFE by blinking its LED at 2 Hz for three seconds. */
int afe_identify(const struct device *dev);

/**
 * Start LED identification without blocking the caller.
 *
 * Frequency is in complete on/off cycles per second (0..20). At 0 Hz the LED
 * stays on. Duration is in milliseconds; 0 means continue until stopped.
 * Starting again replaces the current identification pattern.
 */
int afe_identify_start(const struct device *dev, uint8_t frequency_hz,
		       uint32_t duration_ms);

/** Stop identification and switch off the LED. */
int afe_identify_stop(const struct device *dev);

/**
 * Set a volatile MCP4728 value in millivolts (0..4095).
 *
 * When defer_update is true, call afe_latch() after all staged channels have
 * been written. This function never writes the MCP4728 EEPROM.
 */
int afe_dac_set_mv(const struct device *dev, enum afe_dac_channel channel,
		   uint16_t millivolts, bool defer_update);

/** Atomically stage four values and transfer them with the AFE LDAC signal. */
int afe_dac_set_all_mv(const struct device *dev,
		       const uint16_t millivolts[AFE_DAC_CHANNEL_COUNT]);

/** Pulse the MCP4728 LDAC signal through the TCA9555. */
int afe_latch(const struct device *dev);

/** Read all volatile and EEPROM MCP4728 channel states. */
int afe_dac_read(const struct device *dev,
		 struct afe_dac_state state[AFE_DAC_CHANNEL_COUNT]);

/** Read all TCA9555 register pairs. Reading inputs acknowledges its INT. */
int afe_gpio_read(const struct device *dev, struct afe_gpio_state *state);

/** Read both TCA9555 input ports, acknowledging a pending INT. */
int afe_interrupt_acknowledge(const struct device *dev, uint16_t *inputs);

/**
 * Return 1 when the optional active-low INT GPIO is asserted, 0 otherwise.
 * Returns -ENOTSUP when int-gpios is absent, or another negative errno.
 */
int afe_interrupt_is_pending(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_AFE_H_ */
