/*
 * Copyright (C) 2014, PinyCHWu <pinychwu@fih-foxconn.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_BATTERY_PROTECTION_H__
#define __LINUX_BATTERY_PROTECTION_H__

#define BATT_PROTECT_DEV_NAME "battery_protection"
#define PSY_NAME_LEN 16
#define TEMP_STATUS_COUNT 5

enum batt_protect_status {
	TEMP_NORMAL = 0,
	TEMP_HOT,
	TEMP_WARM,
	TEMP_COOL,
	TEMP_COLD,
};

struct temp_action {
	int enabled;
	int temp_c;
	int vol_mv;
	int cur_ma;
};

/* Note: if not use */
struct batt_protect_platform_data {
	struct temp_action setting[TEMP_STATUS_COUNT];
	char bat_psy_name[PSY_NAME_LEN];
	char chg_psy_name[PSY_NAME_LEN];
};

struct batt_protect_chip {
	struct device *dev;
	struct temp_action setting[TEMP_STATUS_COUNT];
	struct power_supply *bat_psy;
	struct power_supply *chg_psy;
	char bat_psy_name[PSY_NAME_LEN];
	char chg_psy_name[PSY_NAME_LEN];
	enum batt_protect_status status;
};

extern int batt_protection_detect(void);
#endif
