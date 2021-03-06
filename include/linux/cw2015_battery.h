/*
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CW2015_BATTERY_H_
#define __CW2015_BATTERY_H_

struct cw2015_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_full)(void);
};

#endif
