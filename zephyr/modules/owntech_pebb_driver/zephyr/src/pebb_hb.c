/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include "pebb_hb.h"

static const struct device *const pebb_hbs[PEBB_HB_COUNT] = {
	[PEBB_HB_1] = DEVICE_DT_GET(DT_NODELABEL(hb1)),
	[PEBB_HB_2] = DEVICE_DT_GET(DT_NODELABEL(hb2)),
	[PEBB_HB_3] = DEVICE_DT_GET(DT_NODELABEL(hb3)),
};

BUILD_ASSERT(ARRAY_SIZE(pebb_hbs) == PEBB_HB_COUNT);

const struct device *pebb_hb_get(enum pebb_hb_id id)
{
	return (unsigned int)id < PEBB_HB_COUNT ? pebb_hbs[id] : NULL;
}

bool pebb_hb_all_ready(void)
{
	for (size_t index = 0; index < ARRAY_SIZE(pebb_hbs); ++index) {
		if (!device_is_ready(pebb_hbs[index])) {
			return false;
		}
	}
	return true;
}
