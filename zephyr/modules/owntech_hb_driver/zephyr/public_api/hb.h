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

#define HB_MODULE_DATA_SIZE 8U
#define HB_MODULE_DATA_VERSION 1U
#define HB_SERIAL_NUMBER_OFFSET 0x10U
#define HB_SERIAL_NUMBER_LENGTH 13U
#define HB_PANEL_KEY_OFFSET 0x20U
#define HB_PANEL_KEY_LENGTH 10U
#define HB_MODULE_RUNTIME_HOURS_MAX 0xFFFFFFUL
#define HB_MODULE_MEASUREMENT_MAX 0x0FFFU

#define HB_MODULE_FLAG_FAULT   (1U << 0)
#define HB_MODULE_FLAG_WARNING (1U << 1)

/** Compact persistent limits and status for one complete half-bridge module. */
struct hb_module_data {
	uint8_t version;
	uint32_t operating_hours;
	uint16_t max_voltage_code;
	uint16_t max_current_code;
	uint8_t flags;
};

enum hb_side {
	HB_HIGH_SIDE = 0,
	HB_LOW_SIDE = 1,
};

/** Return the native Zephyr T117B sensor for one side. */
const struct device *hb_sensor_device(const struct device *dev,
				      enum hb_side side);

/** Return the shared 24C02S EEPROM device for this HB board. */
const struct device *hb_eeprom_device(const struct device *dev);

/** Read the 13-character serial number; output has room for a terminator. */
int hb_serial_number_read(const struct device *dev,
			  char out[HB_SERIAL_NUMBER_LENGTH + 1U]);

/** Store exactly 13 printable ASCII characters and verify the EEPROM write. */
int hb_serial_number_write(const struct device *dev, const char *serial);

/** Read the 10-character panel access key; output has room for a terminator. */
int hb_panel_key_read(const struct device *dev,
		      char out[HB_PANEL_KEY_LENGTH + 1U]);

/** Store exactly 10 printable ASCII characters and verify the EEPROM write. */
int hb_panel_key_write(const struct device *dev, const char *key);

/** Fetch and return the latest temperature through the Zephyr sensor API. */
int hb_temperature_get(const struct device *dev, enum hb_side side,
		       struct sensor_value *temperature);

/** Fetch temperature as signed millidegrees Celsius. */
int hb_temperature_get_millicelsius(const struct device *dev,
				     enum hb_side side,
				     int32_t *temperature_mc);

/** Return 1 when the active-low T117B ALERT line is asserted, else 0. */
int hb_alert_is_active(const struct device *dev, enum hb_side side);

/** Read version 1 of the shared 8-byte record; reject other versions. */
int hb_module_data_read(const struct device *dev,
			struct hb_module_data *module_data);

/**
 * Write the shared 8-byte operating data at offset 0 of the 24C02S.
 *
 * Set version to HB_MODULE_DATA_VERSION. Voltage/current are raw 12-bit codes;
 * their physical scaling is application specific. Other EEPROM bytes are
 * preserved. Writes are verified; callers must limit frequency for endurance.
 */
int hb_module_data_write(const struct device *dev,
			 const struct hb_module_data *module_data);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_HB_H_ */
