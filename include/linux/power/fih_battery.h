/*
 * Driver for fih battery feature.
 *
 * Copyright (C) 2014 FIH co.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author:  PinyCHWu <pinychwu@fih-foxconn.com>
 *	    November 2014
 */

#ifndef __fih_battery_h__
#define __fih_battery_h__

#define FIH_BATTERY_DEV_NAME "fih_battery"

struct fih_platform_data {
	int reseve;
};

extern int fih_battery_fake_capacity(int cap);
extern int fih_battery_fake_temp(int fake_temp_en);
extern int fih_battery_status(int status);
extern int fih_battery_health(int health);
extern int fih_battery_capacity(int capacity);

#endif /* end of __fih_battery_h__ */
