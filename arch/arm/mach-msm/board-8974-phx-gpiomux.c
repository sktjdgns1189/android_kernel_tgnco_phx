/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include <fih/hwid.h>

/* gpio in */
static struct gpiomux_setting gpio_2ma_np_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_2ma_pu_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_2ma_pd_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

/* gpio out */
static struct gpiomux_setting gpio_2ma_np_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting gpio_6ma_np_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting gpio_2ma_np_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting gpio_2ma_pu_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

/* func1 */
static struct gpiomux_setting fun1_2ma_pd_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting fun1_2ma_np_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting fun1_8ma_np_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting fun1_8ma_pk_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting fun1_12ma_np_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* suspend */
static struct gpiomux_setting gpio_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

/* uart */
static struct gpiomux_setting gpio_uart_config = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

/* i2c */
static struct gpiomux_setting gpio_i2c_config = {
	.func = GPIOMUX_FUNC_3,
	/*
	 * Please keep I2C GPIOs drive-strength at minimum (2ma). It is a
	 * workaround for HW issue of glitches caused by rapid GPIO current-
	 * change.
	 */
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* mi2s */
static struct gpiomux_setting gpio_mi2s_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm8974_phx_evt_gpio_configs[] __initdata = {
	{
		.gpio = 0,			/* NC(TP)_UART_TX */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_uart_config,
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		}
	},
	{
		.gpio = 1,			/* NC(TP)_UART_RX */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_uart_config,
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		}
	},
	{
		.gpio = 2,			/* AUDIOAMP_I2C_DATA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		}
	},
	{
		.gpio = 3,			/* AUDIOAMP_I2C_DCLK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		}
	},
	{
		.gpio = 4,			/* DBG_UART_TX */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_uart_config,
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		}
	},
	{
		.gpio = 5,			/* DBG_UART_RX */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_uart_config,
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		}
	},
	{
		.gpio = 6,			/* TP_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		}
	},
	{
		.gpio = 7,			/* TP_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		}
	},
	{
		.gpio = 8,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 9,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 10,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 11,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 12,			/* LCD_TE */
		.settings = {
			[GPIOMUX_ACTIVE] = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		}
	},
	{
		.gpio = 13,			/* OVP_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		}
	},
	{
		.gpio = 14,			/* LCM_ID0 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		}
	},
	{
		.gpio = 15,			/* ISP_CLK_12M */
		.settings = {
			[GPIOMUX_ACTIVE] = &fun1_12ma_np_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		}
	},
	{
		.gpio = 16,			/* LCM_ID0 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		}
	},
	{
		.gpio = 17,			/* ISP_SUSPEND */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pu_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pu_in_cfg,
		}
	},
	{
		.gpio = 18,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 19,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 20,			/* HALL_OUT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		}
	},
	{
		.gpio = 21,			/* ISP_I2C_SDA0 */
		.settings = {
			[GPIOMUX_ACTIVE] = &fun1_2ma_np_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		}
	},
	{
		.gpio = 22,			/* ISP_I2C_SCL0 */
		.settings = {
			[GPIOMUX_ACTIVE] = &fun1_2ma_np_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		}
	},
	{
		.gpio = 23,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 24,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 25,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
#if 0
	{
		.gpio = 26,			/* ISP_CLK_12M_1 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pu_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pu_out_high_cfg,
		}
	},
#endif
	{
		.gpio = 27,			/* OVP_FLAG */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		}
	},
	{
		.gpio = 28,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 29,			/* NFC_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		}
	},
	{
		.gpio = 30,			/* NFC_I2C_SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
		}
	},
	{
		.gpio = 31,			/* BL_INT_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
		}
	},
	{
		.gpio = 32,			/* NC */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
		}
	},
	{
		.gpio = 33,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 34,			/* ANT_CUTBACK (MDM) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 35,			/* BT_SSBI */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 36,			/* WL_CMD_DATA2 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 37,			/* WL_CMD_DATA1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pu_in_cfg,
		},
	},
	{
		.gpio = 38,			/* WL_CMD_DATA0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 39,			/* WL_CMD_SET */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 40,			/* WL_CMD_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 41,			/* FM_SSBI */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 42,			/* FM_DATA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 43,			/* BT_CTL */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 44,			/* BT_DATA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 45,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 46,			/* NFC_IRQ */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 47,			/* BL_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 48,			/* BL_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 49,			/* UIM2_DATA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 50,			/* UIM2_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 51,			/* UIM2_RST */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 52,			/* UIM2_DETECT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_np_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_np_cfg,
		},
	},
	{
		.gpio = 53,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 54,			/* CHG_INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 55,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 56,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 57,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 58,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 59,			/* MS_INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 60,			/* TP_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pu_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pu_out_high_cfg,
		},
	},
	{
		.gpio = 61,			/* TP_INT_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 62,			/* SD_CARD_DET_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 63,			/* CODEC_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_6ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 64,			/* LCM_HSYNC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 65,			/* GS_INT2 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 66,			/* GYRO_INT1_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 67,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 68,			/* AUDIOAMP_INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 69,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 70,			/* SLIMBUS_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &fun1_8ma_pk_cfg,
		},
	},
	{
		.gpio = 71,			/* SLIMBUS_DATA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &fun1_8ma_pk_cfg,
		},
	},
	{
		.gpio = 72,			/* CODEC_INT1_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 73,			/* GS_INT1 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 74,			/* PROXIMITY_INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 75,			/* NFC_EXT_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 76,			/* CHG_/PG */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 77,			/* SOC_INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 78,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 79,			/* AUDIOAMP_I2S_BCLK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_mi2s_config,
			[GPIOMUX_SUSPENDED] = &gpio_mi2s_config,
		},
	},
	{
		.gpio = 80,			/* AUDIOAMP_I2S_WS */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_mi2s_config,
			[GPIOMUX_SUSPENDED] = &gpio_mi2s_config,
		},
	},
	{
		.gpio = 81,			/* AUDIOAMP_I2S_D1 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_mi2s_config,
			[GPIOMUX_SUSPENDED] = &gpio_mi2s_config,
		},
	},
	{
		.gpio = 82,			/* AUDIOAMP_I2S_D0 */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_mi2s_config,
			[GPIOMUX_SUSPENDED] = &gpio_mi2s_config,
		},
	},
	{
		.gpio = 83,			/* CHARGER_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 84,			/* CHARGER_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 85,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 86,			/* HAPTICS_LEN */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 87,			/* SENSORS1_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 88,			/* SENSORS1_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 89,			/* CHG_STAT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_in_cfg,
		},
	},
	{
		.gpio = 90,			/* ISP_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pu_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 91,			/* FLASH_TX2_MASK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 92,			/* FLASH_TX1_MASK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 93,			/* CODEC_INT2_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pu_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 94,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 95,			/* AUDIOAMP_RST */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 96,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 97,			/* UIM1_DATA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 98,			/* UIM1_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 99,			/* UIM1_RST */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 100,			/* UIM1_DETECT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 101,			/* BATT_REM_ALARM */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 102,			/* ISP_INTR */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pu_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
#if 0
	{
		.gpio = 103,			/* FORCE_USB_BOOT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pu_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
#endif
	{
		.gpio = 104,			/* D_LB_ANT_M3 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 105,			/* D_HB_ANT_M3 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 106,			/* D_LB_ANT_M4 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 107,			/* D_HB_ANT_M4 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 108,			/* P_ANT_SEC_RFFE3_CLK (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 109,			/* P_ANT_SEC_RFFE3_DATA (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 110,			/* P_ANT_M2 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 111,			/* HBMPA_VCONT1 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 112,			/* HBMPA_VCONT2 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 113,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 114,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 115,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
#if 0
	{
		.gpio = 116,			/* TX_GTR_THRES */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
#endif
	{
		.gpio = 117,			/* SW_WB_CPL (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 118,			/* HBMPA_VMODE (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 119,			/* HBMPA_VEN (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 120,			/* NFC_DWL_REQ (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 121,			/* B28_A_B_SEL (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 122,			/* SPDT_2ND_EN (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 123,			/* SP4T_CTRL_2 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 124,			/* TM8_SEL0 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 125,			/* TM8_SEL1 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 126,			/* P_ANT_M1 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 127,			/* SP4T_CTRL_1 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 128,			/* GPS_EXT_LNA_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 129,			/* LCD BIAS_P_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pu_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pu_out_high_cfg,
		},
	},
	{
		.gpio = 130,			/* LCD BIAS_N_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pu_out_high_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pu_out_high_cfg,
		},
	},
	{
		.gpio = 131,			/* CHG_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 132,			/* OTG */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_np_out_low_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_np_out_low_cfg,
		},
	},
	{
		.gpio = 133,			/* WTR0_SSBI_TX_GPS */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_8ma_np_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 134,			/* WTR0_SSBI_PRX_DRX */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_8ma_np_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 135,			/* P_ANT_M3 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 136,			/* P_ANT_M4 (GRFC) */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &fun1_2ma_pd_cfg,
		},
	},
	{
		.gpio = 137,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 138,			/* GSM_TX_PHASE_D1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 139,			/* GSM_TX_PHASE_D0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 140,			/* RFFE1_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 141,			/* RFFE1_DATA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 142,			/* RFFE2_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 143,			/* RFFE2_DATA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &fun1_2ma_pd_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_cfg,
		},
	},
	{
		.gpio = 144,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
	{
		.gpio = 145,			/* NC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_2ma_pd_in_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pd_in_cfg,
		},
	},
};

void __init msm_8974_phx_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

	pr_info("%s, default gpio setting\n", __func__);
	msm_gpiomux_install(msm8974_phx_evt_gpio_configs, ARRAY_SIZE(msm8974_phx_evt_gpio_configs));
}
