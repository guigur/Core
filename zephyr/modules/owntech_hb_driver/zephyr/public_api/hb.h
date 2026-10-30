/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef OWNTECH_HB_H_
#define OWNTECH_HB_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HB_TMP117_EEPROM_WORD_COUNT 4U
#define HB_TMP117_EEPROM_OFFSET_WORD 2U

enum hb_side {
	HB_HIGH_SIDE = 0,
	HB_LOW_SIDE = 1,
};

/** Return the native Zephyr TMP117 sensor for one side. */
const struct device *hb_sensor_device(const struct device *dev,
				      enum hb_side side);

/** Return the native Zephyr EEPROM device for one side. */
const struct device *hb_eeprom_device(const struct device *dev,
				      enum hb_side side);

/** Fetch and return the latest temperature through the Zephyr sensor API. */
int hb_temperature_get(const struct device *dev, enum hb_side side,
		       struct sensor_value *temperature);

/** Fetch temperature as signed millidegrees Celsius. */
int hb_temperature_get_millicelsius(const struct device *dev,
				     enum hb_side side,
				     int32_t *temperature_mc);

/** Return 1 when the active-low TMP117 ALERT line is asserted, else 0. */
int hb_alert_is_active(const struct device *dev, enum hb_side side);

/**
 * Read one of the four 16-bit TMP117 EEPROM words.
 *
 * Word 2 is the TMP117 temperature-offset register and returns -EACCES.
 */
int hb_eeprom_read_word(const struct device *dev, enum hb_side side,
			uint8_t word, uint16_t *value);

/**
 * Write and verify one user EEPROM word.
 *
 * Word 2 is reserved for the temperature offset and returns -EACCES. EEPROM
 * endurance is finite; this function must not be called periodically.
 */
int hb_eeprom_write_word(const struct device *dev, enum hb_side side,
			 uint8_t word, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_HB_H_ */
