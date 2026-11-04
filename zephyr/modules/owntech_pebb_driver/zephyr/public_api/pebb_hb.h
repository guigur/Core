/*
 * Copyright (c) 2026 OwnTech
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef OWNTECH_PEBB_HB_H_
#define OWNTECH_PEBB_HB_H_

#include <stdbool.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PEBB_HB_COUNT 3U

enum pebb_hb_id {
	PEBB_HB_1 = 0,
	PEBB_HB_2,
	PEBB_HB_3,
};

const struct device *pebb_hb_get(enum pebb_hb_id id);
bool pebb_hb_all_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* OWNTECH_PEBB_HB_H_ */
