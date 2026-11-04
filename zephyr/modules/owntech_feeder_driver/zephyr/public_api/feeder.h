/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef OWNTECH_FEEDER_H_
#define OWNTECH_FEEDER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FEEDER_RUNTIME_HOURS_MAX 0xFFFFFFUL
#define FEEDER_VOLTAGE_CODE_MAX  0xFFFFU

/* EEPROM record header. This driver writes LOW records; HIGH is reserved for
 * a future high-voltage feeder using the same record layout. */
#define FEEDER_RECORD_TYPE_LOW  0x01U
#define FEEDER_RECORD_TYPE_HIGH 0x02U
#define FEEDER_RECORD_VERSION   0x01U

#define FEEDER_FLAG_FAULT   (1U << 0)
#define FEEDER_FLAG_WARNING (1U << 1)

struct feeder_operating_data {
	uint32_t operating_hours;
	uint16_t voltage_min_code;
	uint16_t voltage_max_code;
	uint8_t flags;
	bool voltage_valid;
};

/** Return the native Zephyr EEPROM device used by the feeder. */
const struct device *feeder_eeprom_device(const struct device *dev);

/**
 * Start or stop counting operating time.
 *
 * Runtime and observed voltage extrema are checkpointed periodically while
 * running. Call feeder_runtime_flush() before a controlled shutdown to force
 * a checkpoint sooner.
 */
int feeder_runtime_set_running(const struct device *dev, bool running);

/** Return whether operating time is currently being counted. */
bool feeder_runtime_is_running(const struct device *dev);

/** Return accumulated operating time, including the current session. */
int feeder_runtime_get_seconds(const struct device *dev, uint64_t *seconds);

/** Return completed operating hours, including the current session. */
int feeder_runtime_get_hours(const struct device *dev, uint64_t *hours);

/**
 * Observe an input voltage code and update the stored min/max candidates.
 *
 * The code must use the application's ADC or measurement scale. The API does
 * not convert it to volts.
 */
int feeder_voltage_observe(const struct device *dev, uint16_t voltage_code);

/** Set application flags. The voltage-valid bit is managed by the driver. */
int feeder_flags_set(const struct device *dev, uint8_t flags);

/** Read the current in-memory operating record. */
int feeder_data_get(const struct device *dev,
		    struct feeder_operating_data *operating_data);

/**
 * Force a wear-levelled checkpoint if the operating record has changed.
 *
 * Call before a controlled power-down; do not call from a fast periodic loop.
 */
int feeder_runtime_flush(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_FEEDER_H_ */
