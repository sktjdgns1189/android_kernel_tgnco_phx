/*
 * Copyright (C) 2013, PinyCHWu <pinychwu@fih-foxconn.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_POWER_BQ5101X_CHARGER_H__
#define __LINUX_POWER_BQ5101X_CHARGER_H__

#define BQ5101X_WLC_DEV_NAME "bq5101x_wlc"

enum wlc_enable_mode {
	WLC_AD_WL_BOTH = 0,
	WLC_WL_ONLY,
	WLC_OTG_MODE,
	WLC_DISABLE,

	WLC_MODE_MAX,
};

enum wlc_terminal_state {
	WLC_NORMAL_STATE = 0,
	WLC_COMPLETE_STATE,
	WLC_FAULT_STATE,

	WLC_STATE_MAX,
};

/* Note, if gpio is not used, please set as -1 */
struct bq5101x_wlc_platform_data {
	int term_gpio;
	int fault_gpio;
	int en1_gpio;
	int en2_gpio;
	int ad_en_n_gpio;
	int chg_state_n_gpio;
	enum wlc_enable_mode mode;
};

int bq5101x_change_mode(enum wlc_enable_mode mode);
int bq5101x_change_state(enum wlc_terminal_state state);
int bq5101x_wlc_get_chg_state(void);
int bq5101x_wlc_is_use_chgstate(void);
void bq5101x_wlc_control(int on);
void bq5101x_set_power_supply_changed(void);
#endif
