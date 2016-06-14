/*
 * Copyright (C) 2013, PinyCHWu <pinychwu@fih-foxconn.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#ifndef __LINUX_BQ24314C_OVP_H__
#define __LINUX_BQ24314C_OVP_H__

#define BQ24314C_OVP_DEV_NAME "bq24314c_ovp"

#define OVP_DEG

#ifdef OVP_DEG
#define OVP_DBG_INFO(fmt, args...) \
	pr_info("ovp: %s: " fmt, __func__, ##args)
#define OVP_DBG(fmt, args...) \
	pr_debug("ovp: %s: " fmt, __func__, ##args)
#else
#define OVP_DBG_INFO(fmt, args...)  do { } while(0)
#define OVP_DBG(fmt, arges...)      do { } while(0)
#endif

/* Note, if gpio is not used, please set as -1 */
struct bq24314c_ovp_platform_data {
	bool enabled;
	int ce_gpio;
	int fault_gpio;
};

int bq24314c_set_enable(int enable);
int bq24314c_is_enabled(void);
int bq24314c_is_fault(void);
#endif
