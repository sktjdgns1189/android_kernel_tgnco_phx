/* include/linux/rt4501_backlight.c
 * RT4501 LED Backlight Driver
 *
 * Copyright (C) 2013 Richtek Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef LINUX_LEDS_RT4501_BACKLIGHT_H
#define LINUX_LEDS_RT4501_BACKLIGHT_H

#define RT4501_DRV_VER 		"1.0.0G"
#define RT4501_DEV_NAME 	"rt4501"

enum
{
	RT4501_DEV_ID = 0x00,
	RT4501_MANUF,
	RT4501_CONFIG,
	RT4501_TIMING,
	RT4501_BACKLIGHT_CTRL,
	RT4501_FLAG,
	RT4501_MAX_REG,
};

enum
{
	RT4501_OVP_16V,
	RT4501_OVP_23V,
};

enum
{
	RT4501_PWM_HA,
	RT4501_PWM_LA,
};

enum
{
	RT4501_MAXCURR_20MA,
	RT4501_MAXCURR_30MA,
};

enum
{
	RT4501_RAMP_32US,
	RT4501_RAMP_4MS,
	RT4501_RAMP_8MS,
	RT4501_RAMP_16MS,
	RT4501_RAMP_32MS,
	RT4501_RAMP_65MS,
	RT4501_RAMP_131MS,
	RT4501_RAMP_262MS,
};

// for non-device tree
struct rt4501_platform_data
{
	int en_pin;
	unsigned int ovp_sel:1;
	unsigned int pwm_en:1;
	unsigned int pwm_pol:1;
	unsigned int max_curr:1;
	unsigned int up_ramprate:3;
	unsigned int down_ramprate:3;
};

#define RT4501_OVPSEL_MASK	0x80
#define RT4501_PWMEN_MASK	0x40
#define RT4501_PWMPOL_MASK	0x20
#define RT4501_MAXCURR_MASK	0x08
#define RT4501_UPRAMP_MASK	0x38
#define RT4501_UPRAMP_SHFT	3
#define RT4501_DNRAMP_MASK	0x07
#define RT4501_DNRAMP_SHFT	0

#endif
