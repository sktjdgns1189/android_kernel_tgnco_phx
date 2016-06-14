/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SYNAPTICS_DSX_FIH_H_
#define _SYNAPTICS_DSX_FIH_H_

#define PLATFORM_DRIVER_NAME "synaptics_dsx"
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"
#define SPI_DRIVER_NAME "synaptics_dsx_spi"

/*
 * struct synaptics_dsx_cap_button_map - 0d button map
 * @nbuttons: number of 0d buttons
 * @map: pointer to array of button types
 */
struct synaptics_dsx_cap_button_map {
	unsigned char nbuttons;
	u32 *map;
};

/*
 * struct synaptics_dsx_board_data - dsx board data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @irq_gpio: attention interrupt gpio
 * @power_gpio: power switch gpio
 * @power_on_state: power switch active state
 * @reset_gpio: reset gpio
 * @reset_on_state: reset active state
 * @virtual_key: whether virtual exist or not flag
 * @irq_flags: irq flags
 * @panel_x: x-axis resolution of display panel
 * @panel_y: y-axis resolution of display panel
 * @power_delay_ms: delay time to wait after power-on
 * @reset_delay_ms: delay time to wait after reset
 * @reset_active_ms: reset active time
 * @byte_delay_us: delay time between two bytes of SPI data
 * @block_delay_us: delay time between two SPI transfers
 * @regulator_name: pointer to name of regulator
 * @gpio_config: pointer to gpio configuration function
 * @cap_button_map: pointer to 0d button map
 * @project_name: pointer to project name
 */
struct synaptics_dsx_board_data {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	bool reg_en;
	bool i2c_pull_up;
	bool led_en;
	int irq_gpio;
	int power_gpio;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	int virtual_key;
	unsigned long irq_flags;
	unsigned int panel_x;
	unsigned int panel_y;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	unsigned int byte_delay_us;
	unsigned int block_delay_us;
	unsigned char *regulator_name;
	struct regulator *vcc;
	struct regulator *vcc_i2c;
	struct regulator *v_led;
	int (*gpio_config)(int gpio, bool configure, int dir, int state);
	struct synaptics_dsx_cap_button_map *cap_button_map;
	const char *upper_bound;
	const char *lower_bound;
	unsigned int upper_bound_array_size;
	unsigned int lower_bound_array_size;
	int rx_of_key;
	const char *project_name;
	unsigned short config_id_addr;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
};

#endif
