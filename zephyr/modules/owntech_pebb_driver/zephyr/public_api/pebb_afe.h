/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef OWNTECH_PEBB_AFE_H_
#define OWNTECH_PEBB_AFE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PEBB_AFE_COUNT 10U

/** Functional AFE identifiers from the PEBB schematic. */
enum pebb_afe_id {
	PEBB_AFE_V_HIGH = 0,
	PEBB_AFE_V1_LOW,
	PEBB_AFE_V2_LOW,
	PEBB_AFE_V3_LOW,
	PEBB_AFE_I_HIGH,
	PEBB_AFE_I1_LOW,
	PEBB_AFE_I2_LOW,
	PEBB_AFE_I3_LOW,
	PEBB_AFE_SIN,
	PEBB_AFE_COS,
};

const struct device *pebb_afe_get(enum pebb_afe_id id);
bool pebb_afe_all_ready(void);
int pebb_afe_reset_all(void);
int pebb_afe_set_gain_all(uint8_t index);
int pebb_afe_set_bandwidth_all(uint8_t index);
int pebb_afe_set_inputs_all(bool plus_self, bool minus_self);
int pebb_afe_set_enabled_all(bool gain_enabled, bool bandwidth_enabled);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_PEBB_AFE_H_ */
