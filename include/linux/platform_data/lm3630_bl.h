/*
* Simple driver for Texas Instruments LM3630 LED Flash driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#ifndef __LINUX_LM3630_H
#define __LINUX_LM3630_H

#define LM3630_NAME "lm3630_bl"

enum lm3630_pwm_ctrl {
	PWM_CTRL_DISABLE = 0,
	PWM_CTRL_BANK_A,
	PWM_CTRL_BANK_B,
	PWM_CTRL_BANK_ALL,
};

enum lm3630_pwm_active {
	PWM_ACTIVE_HIGH = 0,
	PWM_ACTIVE_LOW,
};

struct lm3630_platform_data {

	/* default values */
	int def_ctrl;
	int def_config;
	int def_boost_ctrl;
	int def_brt;
	int def_current;
	int def_filter_str;

	/* maximum brightness */
	int max_brt_led;

	/* initial on brightness */
	int init_brt_led;
	enum lm3630_pwm_ctrl pwm_ctrl;
	enum lm3630_pwm_active pwm_active;
	int hwen_gpio;
	u32 hwen_gpio_flags;
	int fs_current;
};

extern int lm3630_lcd_backlight_set_level(int level, int enable);
extern int lm3630_lcd_backlight_get_level(int *level);
#endif /* __LINUX_LM3630_H */
