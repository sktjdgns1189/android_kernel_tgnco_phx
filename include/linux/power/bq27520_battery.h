/*
 * BQ27520 I2C Gas Gauge IC
 *
 * Copyright (C) 2010, PinyCHWu <pinychwu@fihtdc.com>
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 */

#ifndef __bq27520_battery_h__
#define __bq27520_battery_h__

#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
#include <linux/qpnp/qpnp-adc.h>
#endif

#define BQ27520_DEV_NAME	"bq27520_battery"

//Firmware version
#define BQ27520_G1		0x302
#define BQ27520_G2		0x311
#define BQ27520_G3		0x324
#define BQ27520_G4		0x329

/* Standard data commands 	ADDR	UNITS */
#define BQ27520_CMD_CNTL	(0x00)	// N/A
#define BQ27520_CMD_AR		(0x02)	// mA
#define BQ27520_CMD_ARTTE	(0x04)	// Min.
#define BQ27520_CMD_TEMP	(0x06)	// 0.1K
#define BQ27520_CMD_VOLT	(0x08)	// mV
#define BQ27520_CMD_FLAGS	(0x0A)	// N/A
#define BQ27520_CMD_NAC		(0x0C)	// mAh
#define BQ27520_CMD_FAC		(0x0E)	// mAh
#define BQ27520_CMD_RM		(0x10)	// mAh
#define BQ27520_CMD_FCC		(0x12)	// mAh
#define BQ27520_CMD_AI		(0x14)	// mA
#define BQ27520_CMD_TTE		(0x16)	// Min.
#define BQ27520_CMD_TTF		(0x18)	// Min.
#define BQ27520_CMD_SI		(0x1A)	// mA
#define BQ27520_CMD_STTE	(0x1C)	// Min.
#define BQ27520_CMD_MLI		(0x1E)	// mA
#define BQ27520_CMD_MLTTE	(0x20)	// Min.
#define BQ27520_CMD_AE		(0x22)	// mWh
#define BQ27520_CMD_AP		(0x24)	// mW
#define BQ27520_CMD_TTECP	(0x26)	// Min.
#define BQ27520_CMD_SOH		(0x28)	// %/num
#define BQ27520_CMD_CC		(0x2A)	// num
#define BQ27520_CMD_SOC		(0x2C)	// %
#define BQ27520_CMD_NIC		(0x2E)	// mohm
#define BQ27520_CMD_ICR		(0x30)	// mA
#define BQ27520_CMD_DLI		(0x32)	// N/A
#define BQ27520_CMD_DLB		(0x34)	// N/A
#define BQ27520_CMD_ITEMP	(0x36)	// 0.1K
#define BQ27520_CMD_OPCFG	(0x3A)	// N/A

/* Extended data commands */
#define BQ27520_EXT_DCAP	(0x3C)	// mAh
#define BQ27520_EXT_DFCLS	(0x3E)	// N/A
#define BQ27520_EXT_DFBLK	(0x3F)	// N/A
#define BQ27520_EXT_DFD		(0x40)	// N/A
#define BQ27520_EXT_DFDCKS	(0x60)	// N/A
#define BQ27520_EXT_DFDCNTL	(0x61)	// N/A
#define BQ27520_EXT_DNAMELEN	(0x62)	// N/A
#define BQ27520_EXT_DNAME	(0x63)	// N/A
#define BQ27520_EXT_APPSTAT	(0x6A)	// N/A

//Only for G4 used
#define BQ27520_CMD_RES_G4	(0x01)	//virtual address for G3
#define BQ27520_CMD_DCAP_G4	(0x03)	//virtual address for G3

#define BQ27520_CMD_LAST	(0x6B)	//for bq27520 command table last

/* Control Subcommands */
#define CNTL_CONTROL_STATUS	(0x0000)
#define CNTL_DEVICE_TYPE	(0x0001)
#define CNTL_FW_VERSION		(0x0002)
#define CNTL_HW_VERSION		(0x0003)
#define CNTL_DF_CHECKSUM	(0x0004)
#define CNTL_PREV_MACWRITE	(0x0007)
#define CNTL_CHEM_ID		(0x0008)
#define CNTL_BOARD_OFFSET	(0x0009)
#define CNTL_CC_INT_OFFSET	(0x000A)
#define CNTL_WRITE_CC_OFFSET	(0x000B)
#define CNTL_OCV_CMD		(0x000C)
#define CNTL_BAT_INSERT		(0x000D)
#define CNTL_BAT_REMOVE		(0x000E)
#define CNTL_SET_HIBERNATE	(0x0011)
#define CNTL_CLEAR_HIBERNATE	(0x0012)
#define CNTL_SET_SLEEP		(0x0013)
#define CNTL_CLEAR_SLEEP	(0x0014)
#define CNTL_FACTORY_RESTORE	(0x0015)
#define CNTL_ENABLE_DLOG	(0x0018)
#define CNTL_DISABLE_DLOG	(0x0019)
#define CNTL_DF_VERSION		(0x001F)
#define CNTL_SEALED		(0x0020)
#define CNTL_IT_ENABLE		(0x0021)
#define CNTL_IT_DISABLE		(0x0023)
#define CNTL_CAL_MODE		(0x0040)
#define CNTL_RESET		(0x0041)

#define	CNTL_LAST		(0x0042) //for table last

/* Status bits define */
/* High Byte */
#define STATUS_DLOGEN		(1 << 7)
#define STATUS_FAS		(1 << 6)
#define STATUS_SS		(1 << 5)
#define STATUS_CSV		(1 << 4)
#define STATUS_CCA		(1 << 3)
#define STATUS_BCA		(1 << 2)
#define STATUS_OCVCMDCOMP	(1 << 1)
#define STATUS_OCVFAIL		(1 << 0)
/* Low Byte */
#define STATUS_INITCOMP		(1 << 7)
#define STATUS_HIBERNATE	(1 << 6)
#define STATUS_SNOOZE		(1 << 5)
#define STATUS_SLEEP		(1 << 4)
#define STATUS_LDMD		(1 << 3)
#define STATUS_RUP_DIS		(1 << 2)
#define STATUS_VOK		(1 << 1)
#define STATUS_QEN		(1 << 0)

/* Flags bits define */
/* High Byte */
#define FLAGS_OTC		(1 << 7)
#define FLAGS_OTD		(1 << 6)
#define FLAGS_CHG_INH		(1 << 3)
#define FLAGS_XCHG		(1 << 2)
#define FLAGS_FC		(1 << 1)
#define FLAGS_CHG		(1 << 0)
/* Low Byte */
#define FLAGS_OCV_GD		(1 << 5)
#define FLAGS_WAIT_ID		(1 << 4)
#define FLAGS_BAT_DET		(1 << 3)
#define FLAGS_SOC1		(1 << 2)
#define FLAGS_SYSDOWN		(1 << 1)
#define FLAGS_DSG		(1 << 0)

/* Application Status bits define */
#define APPSTAT_LU_PROF		(1 << 0)

/* Operation Configuration A bits define */
/* High byte */
#define OPA_RESCAP	(1 << 7)
#define OPA_BATG_OVR	(1 << 6)
#define OPA_INT_BREM	(1 << 5)
#define OPA_PFC_CFG1	(1 << 4)
#define OPA_PFC_CFG0	(1 << 3)
#define OPA_IWAKE	(1 << 2)
#define OPA_RSNS1	(1 << 1)
#define OPA_RSNS0	(1 << 0)
/* Low byte */
#define OPA_INT_FOCV	(1 << 7)
#define OPA_IDSELEN	(1 << 6)
#define OPA_SLEEP	(1 << 5)
#define OPA_RMFCC	(1 << 4)
#define OPA_SOCI_POL	(1 << 3)
#define OPA_BATG_POL	(1 << 2)
#define OPA_BATL_POL	(1 << 1)
#define OPA_TEMPS	(1 << 0)

/* Operation Configuration B bits define */
#define OPB_WRTEMP	(1 << 7)
#define OPB_BIE		(1 << 6)
#define OPB_BL_INT	(1 << 5)
#define OPB_GNDSEL	(1 << 4)
#define OPB_RSVD	(1 << 3)
#define OPB_DFWRINDBL	(1 << 2)
#define OPB_RFACTSTEP	(1 << 1)
#define OPB_INDFACRES	(1 << 0)

/* Operation Configuration C bits define */
#define OPC_BATGSPUEN	(1 << 7)
#define OPC_BATGWPUEN	(1 << 6)
#define OPC_BATLSPUEN	(1 << 5)
#define OPC_BATLWPUEN	(1 << 4)
#define OPC_RSVD	(1 << 3)
#define OPC_SLPWKCHG	(1 << 2)
#define OPC_DELTAVOPT1	(1 << 1)
#define OPC_DELTAVOPT0	(1 << 0)

/* Operation Configuration D bits define */
#define OPD_RCJUMPOK	(1 << 7)
#define OPD_SMTHEN	(1 << 6)
#define OPD_SOC_STATE	(1 << 5)
#define OPD_SOC_OCV	(1 << 4)
#define OPD_SOC_DFW	(1 << 3)
#define OPD_SOC_OT	(1 << 2)
#define OPD_CHGDODEOC	(1 << 1)
#define OPD_RSVD	(1 << 0)

/* Operation Configuration E bits define */
#define OPE_DSGFASTQM	(1 << 3)
#define OPE_CHGFASTQM	(1 << 2)

/* data flash access class in sealed mode */
#define SUBCLASS_SAFETY		(2)
#define SUBCLASS_CHG_INH	(32)
#define SUBCLASS_CHARGE		(34)
#define SUBCLASS_CHG_TERM	(36)
#define SUBCLASS_DATA		(48)
#define SUBCLASS_DISCHARGE	(49)
#define SUBCLASS_MANUFACTURE	(57)
#define SUBCLASS_REGISTER	(64)
#define SUBCLASS_POWER		(68)
#define SUBCLASS_ITCFG		(80)
#define SUBCLASS_CUR_THR	(81)
#define SUBCLASS_STATE		(82)
#define SUBCLASS_OCV0		(83)
#define SUBCLASS_OCV1		(84)
#define SUBCLASS_DEF0_RA	(87)
#define SUBCLASS_DEF1_RA	(88)
#define SUBCLASS_PACK0_RA	(91)
#define SUBCLASS_PACK1_RA	(92)
#define SUBCLASS_PACK0_RAX	(93)
#define SUBCLASS_PACK1_RAX	(94)
#define SUBCLASS_DATA2		(104)
#define SUBCLASS_DATA3		(106)
#define SUBCLASS_CURRENT	(107)
#define SUBCLASS_CODES		(112)

enum dffs_proj_version {
	DFFS_VER_BP5 = 0,
	DFFS_VER_BP8,
	DFFS_VER_VNA,
	DFFS_VER_MO7,
	DFFS_VER_BPC,
	DFFS_VER_VN2,
};

struct bq27520_platform_data {
	unsigned int	rom_addr;
	int		soc_gpio;
	int		blo_gpio;
	int		bgd_gpio;
	int		dffs_ver;
	int		design_max_vol_mV;
	int		design_fcc_mAh;
	int		use_batt_id;
	bool		wrtemp;
};

/*
   This struct is based on powersupply properties
   You can add new item if needed
 */
struct bq27520_property {
	//gas gauge status
	//control
	unsigned int	cntrl_status;
	unsigned int	device_type;
	unsigned int	fw_version;
	unsigned int	hw_version;
	unsigned int	df_checksum;
	unsigned int	chem_id;

	//external
	unsigned int	design_capacity;//unit of mAh
	int		dev_name_len;
	char		dev_name[8];
	unsigned int	app_status;

	//power supply variables
	unsigned int	flags;
	int		status;
	int		health;
	int		present;
	int		online;
	u16		capacity;	//unit of %
	u16		soh;		//unit of %/num
	u16		voltage_mV;	//unit of mV
	s16		current_mA;	//unit of mA
	s16		temp_C;		//unit of 0.1C
	u16		rm_mAh;		//unit of mAh
	u16		im_mohm;	//unit of mohm
	u16		finetune_capacity;	//unit of %
	u16		fcc_mAh;	//unit of mAh
	u16		cc;		//unit of num
	int		design_max_vol_mV;
	int		design_fcc_mAh;
	const char 	*manufacture;
};

struct bq27520_manufacture_info {
	u8	dfi_version;
	char	prj_name[7];
	u8	date[7];
	u8	reserve[17];
};

struct bq27520_opcfg {
	u8 op_a[2];
	u8 reserve[5];
	u8 soc_delta;
	u8 i2c_timeout;
	u8 dfwrindwaittime[2];
	u8 op_b;
	u8 op_c;
	u8 op_d;
	u8 op_e;
	u8 reserve3[17];
};

struct bq27520_chip {
	struct device 		*dev;
	struct mutex		io_lock;
	struct mutex		read_lock;
	struct mutex		update_lock;
	struct i2c_client	*client;
	struct i2c_client	*rom_client;
	struct wake_lock	fw_update_wake_lock;
	unsigned int		rom_addr;
	int			soc_irq;
	int			blo_irq;
	int			bgd_irq;
	int			soc_gpio;
	int			blo_gpio;
	int			bgd_gpio;
	int			irq_reg_num;
	bool			wrtemp;

	struct power_supply	bat;
	struct delayed_work	monitor_work;
	struct delayed_work	interrupt_work;
	struct bq27520_property	prop;
	struct bq27520_manufacture_info m_info;
	struct bq27520_opcfg	op_cfg;
	unsigned long		update_time;
	int			id;
	unsigned int		sealed_status;
	int			int_flag;

	int			chgr_status;
	int			chgr_online;
	int			chgr_present;
	int			invalid_batt;
	int			smooth_flag;
#ifdef CONFIG_BATTERY_PROTECTION
	int			batt_protect_state;
	int			batt_protect_change;
#endif
	int			bbs_fcc_thresh;

	//batt id
	int			use_batt_id;
	int64_t			batt_id_uV; //uV
#ifdef CONFIG_SENSORS_QPNP_ADC_VOLTAGE
	struct qpnp_vadc_chip	*vadc_dev;
#endif
};

#define DEFAULT_MAX_VOL		4200
#define DEFAULT_MAX_FCC		1000

//soc int flag definition
#define NONE_FLAG		0
#define INT_FLAG		1
#define NOTIFY_FLAG		2
#define SHUTDOWN_FLAG		3

#define PARTIAL_OVERTEMP	450	//45C for MUL project
#define OVERTEMP_LOW		50	//5C for MUL project
#define OVERTEMP_HIGH		550	//55C for MUL project
#define OVERTEMP_TOLE		30	//3C for MUL project
#define RECHARGE_TH		95	//95% for recharge when discharging
#define FULL_LIMIT_CURRENT	50	//50mA for MUL charging full condition

#define VBAT_TS_PARTIAL_NORMAL_DPOINT	4000	//4V

extern int bq27520_battery_temperature(void);
extern int bq27520_battery_current(void);
extern int bq27520_battery_ar_current(void);
extern unsigned int bq27520_battery_online(void);
extern unsigned int bq27520_battery_flags(void);
extern unsigned int bq27520_battery_voltage(void);
extern unsigned int bq27520_battery_capacity(void);
extern unsigned int bq27520_battery_soh(void);
extern unsigned int bq27520_battery_health(void);
extern unsigned int bq27520_battery_control_status(void);
extern unsigned int bq27520_battery_rm(void);
extern unsigned int bq27520_battery_fullcharge(void);
extern unsigned int bq27520_battery_cyclecount(void);
extern unsigned int bq27520_design_capacity(void);
extern void bq27520_set_temperature(int temperature);
extern bool bq27520_present(void);
extern void bq27520_dump_register(void);
#endif /* !__bq27520_battery_h__ */
