/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include "afe.h"
#include "pebb_afe.h"

#define PEBB_AFE_DEVICE(label) DEVICE_DT_GET(DT_NODELABEL(label))

static const struct device *const pebb_afes[PEBB_AFE_COUNT] = {
	[PEBB_AFE_V_HIGH] = PEBB_AFE_DEVICE(afe_v_high),
	[PEBB_AFE_V1_LOW] = PEBB_AFE_DEVICE(afe_v1_low),
	[PEBB_AFE_V2_LOW] = PEBB_AFE_DEVICE(afe_v2_low),
	[PEBB_AFE_V3_LOW] = PEBB_AFE_DEVICE(afe_v3_low),
	[PEBB_AFE_I_HIGH] = PEBB_AFE_DEVICE(afe_i_high),
	[PEBB_AFE_I1_LOW] = PEBB_AFE_DEVICE(afe_i1_low),
	[PEBB_AFE_I2_LOW] = PEBB_AFE_DEVICE(afe_i2_low),
	[PEBB_AFE_I3_LOW] = PEBB_AFE_DEVICE(afe_i3_low),
	[PEBB_AFE_SIN] = PEBB_AFE_DEVICE(afe_sin),
	[PEBB_AFE_COS] = PEBB_AFE_DEVICE(afe_cos),
};

BUILD_ASSERT(ARRAY_SIZE(pebb_afes) == PEBB_AFE_COUNT);

const struct device *pebb_afe_get(enum pebb_afe_id id)
{
	return (unsigned int)id < PEBB_AFE_COUNT ? pebb_afes[id] : NULL;
}

bool pebb_afe_all_ready(void)
{
	for (size_t index = 0; index < ARRAY_SIZE(pebb_afes); ++index) {
		if (!device_is_ready(pebb_afes[index])) {
			return false;
		}
	}
	return true;
}

typedef int (*pebb_afe_operation_t)(const struct device *dev, const void *context);

static int pebb_afe_apply_all(pebb_afe_operation_t operation, const void *context)
{
	int first_error = 0;

	for (size_t index = 0; index < ARRAY_SIZE(pebb_afes); ++index) {
		int ret = device_is_ready(pebb_afes[index]) ?
			operation(pebb_afes[index], context) : -ENODEV;

		if (first_error == 0 && ret != 0) {
			first_error = ret;
		}
	}

	return first_error;
}

static int pebb_afe_reset_one(const struct device *dev, const void *context)
{
	ARG_UNUSED(context);
	return afe_reset(dev);
}

int pebb_afe_reset_all(void)
{
	return pebb_afe_apply_all(pebb_afe_reset_one, NULL);
}

static int pebb_afe_set_gain_one(const struct device *dev, const void *context)
{
	return afe_set_gain(dev, *(const uint8_t *)context);
}

int pebb_afe_set_gain_all(uint8_t index)
{
	return pebb_afe_apply_all(pebb_afe_set_gain_one, &index);
}

static int pebb_afe_set_bandwidth_one(const struct device *dev,
				      const void *context)
{
	return afe_set_bandwidth(dev, *(const uint8_t *)context);
}

int pebb_afe_set_bandwidth_all(uint8_t index)
{
	return pebb_afe_apply_all(pebb_afe_set_bandwidth_one, &index);
}

struct pebb_afe_inputs {
	bool plus_self;
	bool minus_self;
};

static int pebb_afe_set_inputs_one(const struct device *dev, const void *context)
{
	const struct pebb_afe_inputs *inputs = context;

	return afe_set_inputs(dev, inputs->plus_self, inputs->minus_self);
}

int pebb_afe_set_inputs_all(bool plus_self, bool minus_self)
{
	const struct pebb_afe_inputs inputs = {
		.plus_self = plus_self,
		.minus_self = minus_self,
	};

	return pebb_afe_apply_all(pebb_afe_set_inputs_one, &inputs);
}

struct pebb_afe_enabled {
	bool gain;
	bool bandwidth;
};

static int pebb_afe_set_enabled_one(const struct device *dev,
				     const void *context)
{
	const struct pebb_afe_enabled *enabled = context;

	return afe_set_enabled(dev, enabled->gain, enabled->bandwidth);
}

int pebb_afe_set_enabled_all(bool gain_enabled, bool bandwidth_enabled)
{
	const struct pebb_afe_enabled enabled = {
		.gain = gain_enabled,
		.bandwidth = bandwidth_enabled,
	};

	return pebb_afe_apply_all(pebb_afe_set_enabled_one, &enabled);
}
