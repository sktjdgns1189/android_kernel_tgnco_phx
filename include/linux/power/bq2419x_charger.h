/*
 * Copyright (C) 2013 FIH CO.
 * based on work by:
 *      Author: Balaji T K
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_BQ2419X_I2C_H
#define _LINUX_BQ2419X_I2C_H

#include <linux/power_supply.h>

#ifdef CONFIG_PM8921_COINCELL
#include <linux/mfd/pm8xxx/misc.h>
#endif

#define BQ2419x_DEV_NAME	"bq2419x"

/* TODO: BEGIN -- This are the previous contents of the file - DELETE */

#define BQ2419x_CHARGE_DONE             0x20
#define BQ2419x_FAULT_VBUS_OVP          0x31
#define BQ2419x_FAULT_SLEEP             0x32
#define BQ2419x_FAULT_BAD_ADAPTOR       0x33
#define BQ2419x_FAULT_BAT_OVP           0x34
#define BQ2419x_FAULT_THERMAL_SHUTDOWN  0x35
#define BQ2419x_FAULT_TIMER             0x36
#define BQ2419x_FAULT_NO_BATTERY        0x37

/* not a bq generated event,we use this to reset the
 * the timer from the twl driver.
 */
#define BQ2419x_RESET_TIMER             0x38

/* BQ24193 / BQ24195 / BQ24198 */
/* Status/Control Register */
#define REG_STATUS_CONTROL              0x08
#define TIMER_RST                       (1 << 7)
#define ENABLE_STAT_PIN                 (1 << 6)

/* Control Register */
#define REG_CONTROL_REGISTER		0x01
#define	INPUT_CURRENT_LIMIT_SHIFT	6
#define	ENABLE_ITERM_SHIFT		3

/* Control/Battery Voltage Register */
#define REG_BATTERY_VOLTAGE		0x02
#define	VOLTAGE_SHIFT			2

/* Vender/Part/Revision Register */
#define REG_PART_REVISION		0x0a //0x03

/* Battery Termination/Fast Charge Current Register */
#define REG_BATTERY_CURRENT		0x04
#define	BQ24195_CURRENT_SHIFT		3
#define	BQ24193_CURRENT_SHIFT		4

/* Special Charger Voltage/Enable Pin Status Register */
#define REG_SPECIAL_CHARGER_VOLTAGE	0x05

/* Safety Limit Register */
#define REG_SAFETY_LIMIT		0x06
#define	MAX_CURRENT_SHIFT		4

#define BQ2419x_WATCHDOG_TIMEOUT	20000


/* TODO: END -- This are the previous contents of the file - DELETE */

#define BQ2419x 			BIT(5)
#define BQ24195 			BQ2419x
#define BQ24196				(BQ2419x|BIT(3))
#define BQ24196_REV_1_2			(BQ24196|BIT(0))
#define BQ24196_REV_1_3			(BQ24196|BIT(1))
#define BQ24196_REV_1_4			(BQ24196|BIT(1)|BIT(2))
#define BQ2419x_PN_REV_BIT_MASK		0x3F

/* register addresses */
#define Reg00Address	0x00 // Input Source Control Register
#define Reg01Address	0x01 // Power On Configuration Reg
#define Reg02Address	0x02 // Charge Current Control Reg
#define Reg03Address	0x03 // Pre-Charge/Termination Current Control Reg
#define Reg04Address	0x04 // Charge Voltage Control Reg
#define Reg05Address	0x05 // Charge Termination/Timer Control Register
#define Reg06Address	0x06 // Threm Reg Control
#define Reg07Address	0x07 // Misc Operation Control
#define Reg08Address	0x08 // System Status Reg
#define Reg09Address	0x09 // Fault Reg
#define Reg10Address	0x0A // Vendor/Part/Rev Status reg

#define DevID 0x6B //bq24190/192/192I

#define DISABLE 0
#define ENABLE 1
#define RESET 1

/* Variables used on bqSetVINDPM Function */
#define VINDPM_MIN	3880	//value in mV
#define VINDPM_MAX	5080	//value in mV
#define VINDPM_STEP	80	//value in mV
#define VINDPM_OFFSET	3880	//value in mV
#define VINDPM_LSHIFT	3
#define VINDPM_MASK	0x87	//Bits Set to 1 on mask will remain unchanged

/* Variables used on bqSetIINDPM Function */
#define IINLIM_100MA	0	//000
#define IINLIM_150MA	1	//001
#define IINLIM_500MA	2	//010
#define IINLIM_900MA	3	//011
#define IINLIM_1200MA	4	//100
#define IINLIM_1500MA	5	//101
#define IINLIM_2000MA	6	//110
#define IINLIM_3000MA	7	//111
#define IINDPM_LSHIFT	0
#define IINDPM_MASK	0xF8 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetCHGCONFIG Function
//#define CHGCONFIG_DISABLE		0
#define CHARGE_BATTERY	1
#define OTG 2
//#define OTG 3
#define CHGCONFIG_LSHIFT 4
#define CHGCONFIG_MASK 0xCF //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetSYSMIN Function
#define SYSMIN_MIN 3000		//value in mV
#define SYSMIN_MAX 3700		//value in mV
#define SYSMIN_STEP 100		//value in mV
#define SYSMIN_OFFSET 3000	//value in mV
#define SYSMIN_LSHIFT 1
#define SYSMIN_MASK 0xF1 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetOTGILIM Function
#define BOOSTLIM_500mA 0
#define BOOSTLIM_1300mA 1
#define BOOSTLIM_LSHIFT 0
#define BOOSTLIM_MASK 0xFE //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetFASTCHRG Function
#define ICHG_MIN 512		//value in mA
#define ICHG_MAX 4544		//value in mA
#define ICHG_DEFAULT 2048	//value in mA
#define ICHG_STEP 64		//value in mA
#define ICHG_LSHIFT 2
#define ICHG_MASK 0x03 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetPRECHRG Function
#define PRECHG_MIN 128		//value in mA
#define PRECHG_MAX 2048		//value in mA
#define PRECHG_STEP 128		//value in mA
#define PRECHG_LSHIFT 4
#define PRECHG_MASK 0x0F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetTERMCHRG Function
#define ITERM_MIN 128		//value in mA
#define ITERM_MAX 2048		//value in mA
#define ITERM_STEP 128		//value in mA
#define ITERM_LSHIFT 0
#define ITERM_MASK 0xF0 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetChgVoltage Function
#define VREG_MIN 3504		//value in mV
#define VREG_MAX 4512		//value in mV
#define VREG_STEP 16		//value in mV
#define VREG_LSHIFT 2
#define VREG_MASK 0x03 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetBATLOWV Function
#define BATLOWV_2800mV 0
#define BATLOWV_3000mV 1		//default value
#define BATLOWV_LSHIFT 1
#define BATLOWV_MASK 0xFD //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetRECHRG Function
#define VRECHG_100mV 0		//default value
#define VRECHG_300mV 1
#define VRECHG_LSHIFT 0
#define VRECHG_MASK 0xFE //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetWatchDog Function
//#define DISABLE 0
#define WatchDog_40s 1		//default value
#define WatchDog_80s 2
#define WatchDog_160s 3
#define WatchDog_LSHIFT 4
#define WatchDog_MASK 0xCF //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetFastChgTimer Function
#define CHGTIMER_5h  0
#define CHGTIMER_8h  1		//default value
#define CHGTIMER_12h 2
#define CHGTIMER_20h 3
#define CHGTIMER_LSHIFT 1
#define CHGTIMER_MASK 0xF9 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetJEITAISET Function
#define JEITA_50p 0
#define JEITA_20p 1
#define JEITAISET_LSHIFT 0
#define JEITAISET_MASK 0xFE //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetBATCOMP Function
#define BATCOMP_MIN 0		//value in mOhm
#define BATCOMP_MAX 70		//value in mOhm
#define BATCOMP_STEP 10		//value in mOhm
#define BATCOMP_LSHIFT 5
#define BATCOMP_MASK 0x1F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetVCLAMP Function
#define VCLAMP_MIN 0		//value in mV
#define VCLAMP_MAX 112		//value in mV
#define VCLAMP_STEP 16		//value in mV
#define VCLAMP_LSHIFT 2
#define VCLAMP_MASK 0xE3 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqSetTREG Function
#define TREG_60C 0
#define TREG_80C 1
#define TREG_100C 2
#define TREG_120C 3		//default value
#define TREG_LSHIFT 0
#define TREG_MASK 0xFC //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnHIZ Function
#define ENHIZ_LSHIFT 7
#define ENHIZ_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqRstREG Function
#define RESETREG_LSHIFT 7
#define RESETREG_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqRstWatchDog Function
#define RESETWATCHDOG_LSHIFT 6
#define RESETWATCHDOG_MASK 0xBF //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnTERM Function
#define ENTERM_LSHIFT 7
#define ENTERM_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqTERMSTAT Function
#define TERMSTAT_ITERM 0		//default value
#define TERMSTAT_EARLY 1
#define TERMSTAT_LSHIFT 6
#define TERMSTAT_MASK 0xBF //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnTIMER Function
#define ENTIMER_LSHIFT 3
#define ENTIMER_MASK 0xF7 //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnDPDM Function
#define ENDPDM_LSHIFT 7
#define ENDPDM_MASK 0x7F //Bits Set to 1 on mask will remain unchanged

// Variables used on bqEnTMR2X Function
#define EN2XTIMER_LSHIFT 6
#define EN2XTIMER_MASK 0xBF //Bits Set to 1 on mask will remain unchanged

// Variables used on bqOffBATFET Function
#define OFFBATFET_LSHIFT 5
#define OFFBATFET_MASK 0xDF //Bits Set to 1 on mask will remain unchanged

//Variables used on bqJEITA_VSET Function
#define JEITA_4P05V	0
#define JEITA_4P20V	1
#define JEITAVSET_LSHIFT 4
#define JEITAVSET_MASK 0xEF //Bits Set to 1 on mask will remain unchanged

#define USB_CURRENT_LIMIT_LOW		100000   /* microAmp */
#define USB_CURRENT_LIMIT_HIGH		500000   /* microAmp */
#define AC_CURRENT_LIMIT		2000000  /* microAmp */

enum bq2419x_usb_type {
	BQ2419x_TYPE_NONE = 0,
	BQ2419x_TYPE_USB,
	BQ2419x_TYPE_AC,
	BQ2419x_TYPE_OTG
};

enum bq2419x_gpio_map {
	BQ2419x_GPIO_PSEL = 0,
	BQ2419x_GPIO_INT,
	BQ2419x_GPIO_CE,
	BQ2419x_GPIO_CHGPG,
	BQ2419x_GPIO_CHGOTG,
	BQ2419x_GPIO_CHGSTAT,
	BQ2419x_IRQ_CHGSTAT,
	BQ2419x_IRQ_CHGPG,
	BQ2419x_IRQ_NINT,
	BQ2419x_WAKE_NINT,
};

struct bq2419x_platform_data {
	//gpio settings
	int		gpio_int;
	int		gpio_ce;
	int		gpio_psel;
	int		gpio_chgpg;
	int		gpio_chgotg;
	int		gpio_chgstat;
	int		psel_invert;
	int		chgotg_invert;
	//charge settings
	unsigned int	input_voltage_mV;
	int		input_current_mA_sdp;
	int		input_current_mA_dcp;
	unsigned int	min_system_mV;
	unsigned int	prechg_current_mA;
	unsigned int	max_charge_current_mA;
	unsigned int	term_current_mA;
	unsigned int	max_charge_voltage_mV;
	int		pre_to_fast_mV;
	int		otg_current_mA;
	int		recharge_delta_mV;
	//other settings
	int		stimer_hr_sdp;
	int		stimer_hr_dcp;
	int		ir_resistor_mohm;
	int		ir_voltage_mV;
	int		thermal_threshold;
	int		jeita_current;
	int		jeita_voltage;
	int		dynamic_vindpm;
#ifdef CONFIG_PM8921_COINCELL
	struct pm8xxx_coincell_chg coin_setting;
#endif
};

struct bq2419x_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct delayed_work	iwork;
	struct work_struct	ework;
	struct delayed_work	vindpm_work;
	struct power_supply	*usb;
	struct power_supply	wall;
	struct mutex		lock;
	struct wake_lock	timer_wakelock;
	struct wake_lock	usb_wake_lock;
	struct wake_lock	eoc_wake_lock;

	int			bq2419x_usb_type;
	int			msm_usb_type;
	int			host_mode;
	u8			charge_state;

	unsigned long		event;
	unsigned short		bqchip_version;

	unsigned int		gpio_map;
	int 			gpio_int;
	int 			gpio_ce;
	int 			gpio_psel;
	int 			gpio_chgpg;
	int 			gpio_chgotg;
	int 			gpio_chgstat;
	int			psel_invert;
	int			chgotg_invert;

	//charge settings
	int			input_voltage_mV;
	int			input_current_mA_sdp;
	int			input_current_mA_dcp;
	int			stimer_hr_sdp;
	int			stimer_hr_dcp;
	int			recharge_delta_mV;
	int			dynamic_vindpm;

	char			reg08;
	char			reg09;
#ifdef CONFIG_PM8921_COINCELL
	struct pm8xxx_coincell_chg coin_setting;
#endif
};

#endif
