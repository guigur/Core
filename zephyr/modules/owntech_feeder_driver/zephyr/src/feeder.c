/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define DT_DRV_COMPAT owntech_low_voltage_feeder

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "feeder.h"

LOG_MODULE_REGISTER(owntech_feeder);

#define FEEDER_RECORD_SIZE       16U
#define FEEDER_LEGACY_RECORD_MAGIC 0x3246U /* Old "F2" header */
#define FEEDER_RECORD_DATA_SIZE  12U
#define FEEDER_MINUTE_MS         60000ULL
#define FEEDER_HOUR_MS           3600000ULL
#define FEEDER_FLAG_VOLTAGE_VALID 0x80U

struct feeder_config {
	const struct device *eeprom;
	uint32_t persist_interval_minutes;
};

struct feeder_data {
	struct k_mutex lock;
	struct k_work_delayable persist_work;
	const struct device *dev;
	uint64_t accumulated_ms;
	int64_t running_since_ms;
	uint32_t persisted_minutes;
	uint32_t sequence;
	uint16_t voltage_min_code;
	uint16_t voltage_max_code;
	uint16_t persisted_voltage_min_code;
	uint16_t persisted_voltage_max_code;
	uint16_t next_slot;
	uint8_t flags;
	uint8_t persisted_flags;
	bool running;
};

BUILD_ASSERT(CONFIG_OWNTECH_FEEDER_DRIVER_INIT_PRIORITY >
	     CONFIG_EEPROM_INIT_PRIORITY,
	     "Feeder driver must initialize after its EEPROM");

static uint32_t feeder_record_crc(const uint8_t record[FEEDER_RECORD_SIZE])
{
	return crc32_ieee(record, FEEDER_RECORD_DATA_SIZE);
}

static bool feeder_record_decode(const uint8_t record[FEEDER_RECORD_SIZE],
				 uint16_t *sequence, uint32_t *hours,
				 uint16_t *voltage_min, uint16_t *voltage_max,
				 uint8_t *flags)
{
	bool current_format = record[0] == FEEDER_RECORD_TYPE_LOW &&
			      record[1] == FEEDER_RECORD_VERSION;
	bool legacy_format = sys_get_le16(&record[0]) ==
			     FEEDER_LEGACY_RECORD_MAGIC;

	if ((!current_format && !legacy_format) ||
	    sys_get_le32(&record[12]) != feeder_record_crc(record)) {
		return false;
	}

	*sequence = sys_get_le16(&record[2]);
	*hours = (uint32_t)record[4] | ((uint32_t)record[5] << 8) |
		 ((uint32_t)record[6] << 16);
	*voltage_min = sys_get_le16(&record[7]);
	*voltage_max = sys_get_le16(&record[9]);
	*flags = record[11];
	return true;
}

static void feeder_record_encode(uint8_t record[FEEDER_RECORD_SIZE],
				 uint16_t sequence, uint32_t hours,
				 uint16_t voltage_min, uint16_t voltage_max,
				 uint8_t flags)
{
	record[0] = FEEDER_RECORD_TYPE_LOW;
	record[1] = FEEDER_RECORD_VERSION;
	sys_put_le16(sequence, &record[2]);
	record[4] = (uint8_t)hours;
	record[5] = (uint8_t)(hours >> 8);
	record[6] = (uint8_t)(hours >> 16);
	sys_put_le16(voltage_min, &record[7]);
	sys_put_le16(voltage_max, &record[9]);
	record[11] = flags;
	sys_put_le32(feeder_record_crc(record), &record[12]);
}

static bool feeder_sequence_is_newer(uint16_t candidate, uint16_t current)
{
	return (int16_t)(candidate - current) > 0;
}

static void feeder_update_elapsed_locked(struct feeder_data *data)
{
	int64_t now;

	if (!data->running) {
		return;
	}

	now = k_uptime_get();
	if (now > data->running_since_ms) {
		data->accumulated_ms += (uint64_t)(now - data->running_since_ms);
		data->running_since_ms = now;
	}
}

static uint32_t feeder_runtime_minutes_locked(const struct feeder_data *data)
{
	uint64_t minutes = data->accumulated_ms / FEEDER_MINUTE_MS;

	return minutes > UINT32_MAX ? UINT32_MAX : (uint32_t)minutes;
}

static uint32_t feeder_runtime_hours_locked(const struct feeder_data *data)
{
	uint64_t hours = data->accumulated_ms / FEEDER_HOUR_MS;

	return hours > FEEDER_RUNTIME_HOURS_MAX ? FEEDER_RUNTIME_HOURS_MAX :
		(uint32_t)hours;
}

static bool feeder_state_changed_locked(const struct feeder_data *data)
{
	return data->flags != data->persisted_flags ||
	       data->voltage_min_code != data->persisted_voltage_min_code ||
	       data->voltage_max_code != data->persisted_voltage_max_code;
}

static int feeder_checkpoint_locked(const struct device *dev, bool force)
{
	const struct feeder_config *config = dev->config;
	struct feeder_data *data = dev->data;
	uint8_t record[FEEDER_RECORD_SIZE];
	uint8_t readback[FEEDER_RECORD_SIZE];
	uint32_t minutes = feeder_runtime_minutes_locked(data);
	uint32_t hours = feeder_runtime_hours_locked(data);
	uint16_t sequence;
	off_t offset;
	size_t slot_count = eeprom_get_size(config->eeprom) / FEEDER_RECORD_SIZE;
	bool runtime_due = minutes - data->persisted_minutes >=
			   config->persist_interval_minutes;
	int ret;

	if (!runtime_due && !feeder_state_changed_locked(data)) {
		return 0;
	}
	if (!force && !runtime_due && data->running) {
		return 0;
	}

	sequence = (uint16_t)(data->sequence + 1U);
	offset = (off_t)(data->next_slot * FEEDER_RECORD_SIZE);
	feeder_record_encode(record, sequence, hours, data->voltage_min_code,
			     data->voltage_max_code, data->flags);

	ret = eeprom_write(config->eeprom, offset, record, sizeof(record));
	if (ret == 0) {
		ret = eeprom_read(config->eeprom, offset, readback,
				  sizeof(readback));
	}
	if (ret != 0) {
		return ret;
	}
	if (memcmp(record, readback, sizeof(record)) != 0) {
		return -EIO;
	}

	data->sequence = sequence;
	data->persisted_minutes = minutes;
	data->persisted_voltage_min_code = data->voltage_min_code;
	data->persisted_voltage_max_code = data->voltage_max_code;
	data->persisted_flags = data->flags;
	data->next_slot = (uint16_t)((data->next_slot + 1U) % slot_count);
	return 0;
}

static void feeder_schedule_next_locked(const struct device *dev)
{
	const struct feeder_config *config = dev->config;
	struct feeder_data *data = dev->data;
	uint32_t minutes = feeder_runtime_minutes_locked(data);
	uint32_t elapsed = minutes - data->persisted_minutes;
	uint32_t remaining = elapsed < config->persist_interval_minutes ?
		config->persist_interval_minutes - elapsed : 1U;

	(void)k_work_reschedule(&data->persist_work, K_MINUTES(remaining));
}

static void feeder_persist_work_handler(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct feeder_data *data = CONTAINER_OF(delayable, struct feeder_data,
						 persist_work);
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	feeder_update_elapsed_locked(data);
	ret = feeder_checkpoint_locked(data->dev, false);
	if (ret != 0) {
		LOG_ERR("operating-data checkpoint failed: %d", ret);
	}
	if (data->running) {
		feeder_schedule_next_locked(data->dev);
	}
	k_mutex_unlock(&data->lock);
}

static int feeder_load_journal(const struct device *dev)
{
	const struct feeder_config *config = dev->config;
	struct feeder_data *data = dev->data;
	uint8_t record[FEEDER_RECORD_SIZE];
	uint16_t sequence;
	uint16_t best_sequence = 0U;
	uint16_t best_slot = 0U;
	uint32_t hours;
	uint32_t best_hours = 0U;
	uint16_t voltage_min;
	uint16_t voltage_max;
	uint16_t best_voltage_min = 0xFFFFU;
	uint16_t best_voltage_max = 0U;
	uint8_t flags;
	uint8_t best_flags = 0U;
	size_t slot_count = eeprom_get_size(config->eeprom) / FEEDER_RECORD_SIZE;
	bool found = false;
	int ret;

	for (size_t slot = 0; slot < slot_count; ++slot) {
		ret = eeprom_read(config->eeprom,
				  (off_t)(slot * FEEDER_RECORD_SIZE), record,
				  sizeof(record));
		if (ret != 0) {
			return ret;
		}
		if (feeder_record_decode(record, &sequence, &hours, &voltage_min,
					 &voltage_max, &flags) &&
		    (!found || feeder_sequence_is_newer(sequence, best_sequence))) {
			found = true;
			best_sequence = sequence;
			best_hours = hours;
			best_voltage_min = voltage_min;
			best_voltage_max = voltage_max;
			best_flags = flags;
			best_slot = (uint16_t)slot;
		}
	}

	data->sequence = found ? best_sequence : 0U;
	data->accumulated_ms = (uint64_t)best_hours * FEEDER_HOUR_MS;
	data->persisted_minutes = (uint32_t)(data->accumulated_ms /
					      FEEDER_MINUTE_MS);
	data->voltage_min_code = best_voltage_min;
	data->voltage_max_code = best_voltage_max;
	data->flags = best_flags;
	data->persisted_voltage_min_code = best_voltage_min;
	data->persisted_voltage_max_code = best_voltage_max;
	data->persisted_flags = best_flags;
	data->next_slot = found ? (uint16_t)((best_slot + 1U) % slot_count) : 0U;
	return 0;
}

static int feeder_driver_init(const struct device *dev)
{
	const struct feeder_config *config = dev->config;
	struct feeder_data *data = dev->data;
	size_t eeprom_size;

	if (!device_is_ready(config->eeprom)) {
		return -ENODEV;
	}

	eeprom_size = eeprom_get_size(config->eeprom);
	if (eeprom_size < 2U * FEEDER_RECORD_SIZE ||
	    eeprom_size % FEEDER_RECORD_SIZE != 0U) {
		return -EINVAL;
	}

	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->persist_work, feeder_persist_work_handler);
	data->dev = dev;
	data->voltage_min_code = 0xFFFFU;
	return feeder_load_journal(dev);
}

const struct device *feeder_eeprom_device(const struct device *dev)
{
	const struct feeder_config *config = dev->config;

	return config->eeprom;
}

int feeder_runtime_set_running(const struct device *dev, bool running)
{
	struct feeder_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->running == running) {
		k_mutex_unlock(&data->lock);
		return 0;
	}

	if (running) {
		data->running = true;
		data->running_since_ms = k_uptime_get();
		feeder_schedule_next_locked(dev);
	} else {
		feeder_update_elapsed_locked(data);
		data->running = false;
	}
	k_mutex_unlock(&data->lock);

	if (!running) {
		(void)k_work_cancel_delayable(&data->persist_work);
	}
	return ret;
}

bool feeder_runtime_is_running(const struct device *dev)
{
	struct feeder_data *data = dev->data;
	bool running;

	k_mutex_lock(&data->lock, K_FOREVER);
	running = data->running;
	k_mutex_unlock(&data->lock);
	return running;
}

int feeder_runtime_get_seconds(const struct device *dev, uint64_t *seconds)
{
	struct feeder_data *data = dev->data;

	if (seconds == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	feeder_update_elapsed_locked(data);
	*seconds = data->accumulated_ms / 1000ULL;
	k_mutex_unlock(&data->lock);
	return 0;
}

int feeder_runtime_get_hours(const struct device *dev, uint64_t *hours)
{
	uint64_t seconds;
	int ret;

	if (hours == NULL) {
		return -EINVAL;
	}
	ret = feeder_runtime_get_seconds(dev, &seconds);
	if (ret == 0) {
		*hours = seconds / 3600ULL;
	}
	return ret;
}

int feeder_voltage_observe(const struct device *dev, uint16_t voltage_code)
{
	struct feeder_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	if ((data->flags & FEEDER_FLAG_VOLTAGE_VALID) == 0U) {
		data->voltage_min_code = voltage_code;
		data->voltage_max_code = voltage_code;
		data->flags |= FEEDER_FLAG_VOLTAGE_VALID;
	} else {
		if (voltage_code < data->voltage_min_code) {
			data->voltage_min_code = voltage_code;
		}
		if (voltage_code > data->voltage_max_code) {
			data->voltage_max_code = voltage_code;
		}
	}
	k_mutex_unlock(&data->lock);
	return 0;
}

int feeder_flags_set(const struct device *dev, uint8_t flags)
{
	struct feeder_data *data = dev->data;

	if ((flags & FEEDER_FLAG_VOLTAGE_VALID) != 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->flags = (uint8_t)((data->flags & FEEDER_FLAG_VOLTAGE_VALID) | flags);
	k_mutex_unlock(&data->lock);
	return 0;
}

int feeder_data_get(const struct device *dev, struct feeder_operating_data *out)
{
	struct feeder_data *data = dev->data;

	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	feeder_update_elapsed_locked(data);
	out->operating_hours = feeder_runtime_hours_locked(data);
	out->voltage_min_code = data->voltage_min_code;
	out->voltage_max_code = data->voltage_max_code;
	out->flags = data->flags & (uint8_t)~FEEDER_FLAG_VOLTAGE_VALID;
	out->voltage_valid = (data->flags & FEEDER_FLAG_VOLTAGE_VALID) != 0U;
	k_mutex_unlock(&data->lock);
	return 0;
}

int feeder_runtime_flush(const struct device *dev)
{
	struct feeder_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	feeder_update_elapsed_locked(data);
	ret = feeder_checkpoint_locked(dev, true);
	k_mutex_unlock(&data->lock);
	return ret;
}

#define FEEDER_DEFINE(inst)                                                   \
	BUILD_ASSERT(DT_INST_PROP(inst, persist_interval_minutes) > 0U,          \
		     "Feeder persistence interval must be non-zero");           \
	static const struct feeder_config feeder_config_##inst = {               \
		.eeprom = DEVICE_DT_GET(DT_INST_PHANDLE(inst, eeprom)),           \
		.persist_interval_minutes =                                      \
			DT_INST_PROP(inst, persist_interval_minutes),                \
	};                                                                         \
	static struct feeder_data feeder_data_##inst;                              \
	DEVICE_DT_INST_DEFINE(inst, feeder_driver_init, NULL,                      \
			      &feeder_data_##inst, &feeder_config_##inst,          \
			      POST_KERNEL,                                        \
			      CONFIG_OWNTECH_FEEDER_DRIVER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(FEEDER_DEFINE)
