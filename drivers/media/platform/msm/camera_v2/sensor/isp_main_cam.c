/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "msm.h"
#include <linux/irq.h>  /*for ISP interrupt*/
#include <linux/i2c.h>  /*for ISP firmware upgrade*/
#include "isp_main_cam.h"
#include <linux/workqueue.h>
#include "icatch7002a_img.h"
#include <asm/bug.h>
#include <linux/proc_fs.h>

#include <mach/socinfo.h>  //SW4-Rocky-EVT-Camera-PortingCamera_20131016

#include <mach/gpiomux.h>//SW4-Rocky-Camera-PortingCamera_20130719_00
#include <media/msm_cam_sensor.h>//SW4-Rocky-Camera-PortingCamera_20130719_00
#include<linux/kthread.h>

#define SENSOR_NAME "isp_main_cam"
#define PLATFORM_DRIVER_NAME "msm_camera_isp_main_cam"
#define isp_main_cam_obj isp_main_cam_##obj

#define ENABLE_ISP_INTERRUPT 1
#define ENABLE_ISP_FIRMWARE_UPGRADE 1
//#define ENABLE_GPIO_DEBUG 1
//#define DEBUG_GPIO 36
//#define ENABLE_NORMAL_DEBUG 1
//#define ENABLE_SENSOR_REGULATOR 1
#define ENABLE_SKIP_FRAMES 1//SW4-RK-Camera-FlashTiming-00+20140313  //SW4-RK-Camera-QCTPatchToFixCaptureHangWhenSkipFrameIs0-MCS-4514-00*_20140415

#define CONFIG_MSMB_CAMERA_DEBUG//SW4-Rocky-Camera-PortingCamera_20130719_00

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

struct device *dev_uevent;

static int g_cur_bestshot = -1;
static int g_cur_effect = -1;
static int g_cur_ev = -1;
static int g_cur_iso = -1;
static int g_cur_aec = -1;
static int g_cur_whitebalance = -1;
static int g_cur_saturation = -1;
static int g_cur_sharpness = -1;
static int g_cur_contrast = -1;
static int g_cur_antibanding = -1;
static int g_cur_ledflash = -1;
static int g_cur_hdr = -1;
static int g_cur_aec_lock = -1;
static int g_cur_awb_lock = -1;
static int g_cur_wdr = -1; //RL*_20140117

static unsigned int preview_resolution=MAIN_RES_1040x780;//fihtdc,derekcwwu, add for tx3
static unsigned int snapshot_resolution=MAIN_RES_3264x2448;	//RL*_20140710
static unsigned int hd_resolution=MAIN_RES_1280x720; //ChunJeng Add 20141007
static int isp_main_cam_mode;
static int restore_isp_main_cam_mode;  //SW5-Webber-20150513-Add for restore camera resolution for recover mode.
int isp_is_power_on = 0;
//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
static int main_cam_is_power_on = 0;
//SW4-HL-Camera-FixCameraSwitchFailIssue-00+{_20130926
int front_cam_is_power_on;
extern int isp_imx132_mode;
//SW4-HL-Camera-FixCameraSwitchFailIssue-00+}_20130926
//HL+_20130927
extern bool g_isp_imx132_first_open;
//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+_20131030
int giCatchStreamOff = 0;
int interrup_error=0;//fihtdc,derekcww,add for get 0x72f8 error

//static int g_hdr_done = 0;		//RL*_20140125
bool iCatch_first_open = true;  //for iCatch ISP used
bool enableFaceTracking = false;	//SW4-RL-implementFaceTracking-00+_20140919
int Main_Camera_ISP_Ready=0;
int Front_Camera_ISP_Ready=0;
struct completion g_iCatch_comp;//fihtdc,derekcww,add for front cam when front cam can focus
static int g_pre_res = MSM_SENSOR_INVALID_RES;
static int g_time_start = 0;
static int g_caf_sensor_enable =0;
//struct delayed_work CAF_work;

//Rocky_{_20131012
static struct focus_cfg focusROI;
bool isROIset = false;
kernel_isp3a_af_mode_t afMode = -1;
//Rocky_}_20131012

int isp_ois_on = 1;	//SW4-RL-Camera-implementOIS-00+_20140430

int delayed_work_status=0;
#if ENABLE_ISP_INTERRUPT
//SW4-HL-Camera-EnhanceKernelLog-MCS-3055-00*_20140219
static unsigned int ISP_INTR_GPIO;
static unsigned int g_isp_irq;
#endif
DEFINE_MUTEX(isp_main_cam_mut);
static struct msm_sensor_ctrl_t isp_main_cam_s_ctrl;
static int HDR_mode = 0;
static struct kobject *example_kobj = NULL;

static bool g_isAFCancel = false;//SW4-Rocky-Camera-PortingCamera_20130719_00
static bool g_afmode=0; //0: Auto AF, 1:Full search AF  //SW4-Rocky-Camera-PortingCamera_20130719_00
//static int g_flash_mode = 0;	//"Flash mode for EXIF"
static bool g_enable_roi_debug = false; //add 13M camera TAF support  //SW4-Rocky-Camera-PortingCamera_20130719_00
static bool g_TAEenable = false;	//0: Disable, 1:Enable  //SW4-Rocky-Camera-PortingCamera_20130719_00
static unsigned int isp_firmware_version = 0x0;
static unsigned int bin_file_version = 0x0;
static bool caf_mode = false;

#if ENABLE_ISP_FIRMWARE_UPGRADE
#define SPI_CMD_BYTE_READ 			0x03
#define SPI_CMD_RD_ID 				0x9F
#define SPI_CMD_WRT_EN				0x06
#define SPI_CMD_BYTE_PROG 			0x02
#define SPI_CMD_RD_STS 				0x05
#define SPI_CMD_BYTE_PROG_AAI 		0xAD
#define SPI_CMD_WRT_STS_EN  		0x50
#define SPI_CMD_WRT_STS 			0x01
#define SPI_CMD_WRT_DIS 			0x04
#define SPI_CMD_ERASE_ALL			0xC7
#define SPI_CMD_SECTOR_ERASE		0x20
#define SPI_CMD_32KB_BLOCK_ERASE	0x52
#define SPI_CMD_64KB_BLOCK_ERASE	0xD8

//Selwyn 2013/10/07 modified for FTM
//#define FILE_PATH_BOOTCODE "/data/BOOT.BIN"
#define FILE_PATH_BOOTCODE "/system/etc/ISP_Firmware.bin"
//~Selwyn modified
#define SPI_CMD_RD_ID 				0x9F
#define WAIT_COUNT 0
//#define printf printk
#define SUCCESS 0
unsigned char g_memory_512K[1024*1024];	//Rocky_20131024

int isp_main_cam_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int isp_main_cam_power_down(struct msm_sensor_ctrl_t *s_ctrl);
int32_t isp_main_cam_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl, uint16_t res);
int32_t isp_main_cam_match_id(struct msm_sensor_ctrl_t *s_ctrl);

static int recover_isp(struct msm_sensor_ctrl_t *s_ctrl);
static int isp_enable_recovery = 1;
int isp_enable_i2c_dbg = 0;
static struct proc_dir_entry *isp_proc_dir;
static struct proc_dir_entry *isp_proc_entry;
static struct proc_dir_entry *isp_proc_entry_i2c;
#define ISP_DBG_PROC_DIR "isp_dbg"
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

int af_not_ready=0;

unsigned int firmwaresize=0;
unsigned int lastpage=0;
char camera_firmware_name[30];
static char *firmware_path=NULL;
typedef enum
{
	FW_STATUS_READY,
	FW_STATUS_UPGRADING,
	FW_STATUS_FAIL,
}FW_STATUS;

typedef enum
{
	FW_UPGRADE_MSG_READY,
	FW_UPGRADE_MSG_UPGRADING,
	FW_UPGRADE_MSG_DO_NOT_NEED_UPGRADE,
	FW_UPGRADE_MSG_UPGRADE_DONE,
	FW_UPGRADE_MSG_NOT_FOUND,
	FW_UPGRADE_MSG_CANT_UPGRADE
}FW_UPGRADE_MSG;

//SW4-RL-FTM-implementEXIF-00+{_20140827
typedef struct
{
	uint16_t iso_value_L;
	uint16_t iso_value_H;
	uint32_t iso_value;
	uint16_t exposure_numerator;
	uint16_t exposure_denominator_L;
	uint16_t exposure_denominator_M;
	uint16_t exposure_denominator_H;
	uint32_t exposure_denominator;
	uint16_t capExpIdx;
	uint16_t capAgcIdx;
	uint16_t pvExpIdx;
	uint16_t pvAgcIdx;
	uint16_t aeEntry;
	uint16_t aeLuma;
	uint16_t aeStep;
	uint16_t aetarget;
	uint16_t awbRGain_LSB;
	uint16_t awbRGain_MSB;
	uint16_t awbBGain_LSB;
	uint16_t awbBGain_MSB;
	uint16_t afVertical_step_LSB;
	uint16_t afVertical_step_MSB;
	uint16_t afMulti_Step_LSB;
	uint16_t afMulti_Step_MSB;
	uint16_t afFinal_Step_LSB;
	uint16_t afFinal_Step_MSB;
	uint16_t afFinal_Result_LSB;
	uint16_t afFinal_Result_MSB;
	uint16_t flash_state;
	uint16_t metering_mode;
	uint16_t awbiCandidateIdx;
	uint16_t awbDynamicIQCIdx;
	uint16_t threeA_Version;
	uint16_t threeA_ID;
}EXIF_info_t;
//SW4-RL-FTM-implementEXIF-00+}_20140827

static FW_STATUS fw_status = FW_STATUS_READY;
static FW_UPGRADE_MSG fw_upgrade_msg = FW_UPGRADE_MSG_READY;
static void set_firmware_upgrade_status(FW_STATUS status, FW_UPGRADE_MSG msg);
static void do_firmware_upgrade(char *filepath);
static int isp_get_firmware_version(void)
{
    unsigned char temp_1[4] = {0};
    uint16_t status_1=0;
    int rc=0;

    pr_err("%s:Get ISP Firmware Version \n",__func__);
    msleep(200);
    rc = msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client,0x72cc, &status_1,MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: Get ISP Firmware Version failed 1\n", __func__);
        return rc;
    }
    temp_1[2]=(char)status_1;
    rc = msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client,0x72cb, &status_1,MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: Get ISP Firmware Version failed 2\n", __func__);
        return rc;
    }
    temp_1[1]=(char)status_1;
    rc = msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client,0x72ca, &status_1,MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s: Get ISP Firmware Version failed 3\n", __func__);
        return rc;
    }
    temp_1[0]=(char)status_1;
    isp_firmware_version = ((temp_1[3] << 24) | (temp_1[2] << 16) | (temp_1[1] << 8) | (temp_1[0]));

    return 0;
}
static char *isp_get_firmware_path(void)
{
    struct file *file_filp = NULL;

    if(!firmware_path){
        firmware_path=kzalloc(sizeof(char)*100,GFP_KERNEL);
        if (!firmware_path) {
            pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
            goto END;
        }
    }else{
        pr_err("return firmware path=%s\n",firmware_path);
        goto END;
    }
    strcat(firmware_path,"/system/etc/");

    strcat(firmware_path,camera_firmware_name);
    strcat(firmware_path,"_ISP_Firmware.bin");

    pr_err("firmware path=%s",firmware_path);

    file_filp = filp_open(firmware_path, O_RDONLY, 0);
    if (IS_ERR(file_filp)){
		    pr_err("%s: Fail to open %s\n", __func__,firmware_path);
		    kfree(firmware_path);
		    firmware_path=NULL;
        goto END;
    }
END:
    return firmware_path;
}
static ssize_t isp_proc_write( struct file *filp, const char __user *buff, unsigned long len, void *data )
{
    sscanf(buff, "%d", &isp_enable_recovery) ;
	pr_debug("%s: %d\n", __func__, isp_enable_recovery);
    return len;
}

static int isp_proc_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
    return sprintf(page, "%d\n", isp_enable_recovery);

}

static ssize_t isp_proc_i2c_write( struct file *filp, const char __user *buff, unsigned long len, void *data )
{
    sscanf(buff, "%d", &isp_enable_i2c_dbg) ;
	pr_debug("%s: %d\n", __func__, isp_enable_i2c_dbg);
    return len;
}

static int isp_proc_i2c_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
    return sprintf(page, "%d\n", isp_enable_i2c_dbg);

}
#if 1
static const UINT32 stSpiIdInfo[8][3] =
{
	/*Winbond*/
	{0x00EF3017,4096, 2048},
	{0x00EF3016,4096, 1024},
	{0x00EF3015,4096, 512},
	{0x00EF3014,4096, 256},
	{0x00EF5014,4096, 256},
	{0x00EF3013,4096, 128},
	{0x00EF5013,4096, 128},
	/*Fail*/
	{0x00000000,0,0},
};

static const UINT32 sstSpiIdInfo[6][3] =
{
	/*ESMT*/
	{0x008C4016,4096,512},
	/*SST*/
	{0x00BF254A,4096,1024},
	{0x00BF2541,4096,512},
	{0x00BF258E,4096,256},
	{0x00BF258D,4096,128},
	/*Fail*/
	{0x00000000,0,0},
};
#endif

void I2CDataWrite(UINT32 addr, UINT16 value)
{
	int rc = -1;
	rc = msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, addr, value, MSM_CAMERA_I2C_BYTE_DATA);
}

void hsI2CDataWrite(UINT32 addr, UINT16 value)
{
	I2CDataWrite(addr, value);
}

UINT32 I2CDataRead(UINT32 addr)
{
	UINT16 value = 0;
	int rc = -1;

	rc = msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, addr, &value, MSM_CAMERA_I2C_BYTE_DATA);
	return value;
}

UINT32 hsI2CDataRead(UINT32 addr)
{
	return I2CDataRead(addr);
}
#endif
#if 1
UINT32 BB_SerialFlashTypeCheck(UINT32 id, UINT32 *spiSize)
{
	UINT32 i=0;
	UINT32 fullID = 1;
	UINT32 shift = 0, tblId, type = 0;
	/*pr_err("id:0x%x spiSize:0x%x\n",id,spiSize);*/
	/* check whether SST type serial flash */
	while( 1 ){
		tblId = sstSpiIdInfo[i][0] >> shift;
		if( id == tblId ) {
			pr_err("SST type serial flash:%x %x %x\n",i,id,sstSpiIdInfo[i][0]);
			type = 2;
			*spiSize = sstSpiIdInfo[i][1]*sstSpiIdInfo[i][2];
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( sstSpiIdInfo[i][0] == 0x00000000 ) {
			#if 0
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			#endif
			type = 3;
			break;
		}
		i ++;
	}
	if( type == 2 )
		return type;

	i = 0;
	/* check whether ST type serial flash */
	while( 1 ){
		tblId = stSpiIdInfo[i][0] >> shift;
		if( id == tblId ) {
			pr_err("ST Type serial flash:%x %x %x\n",i,id,stSpiIdInfo[i][0]);
			type = 1;
			*spiSize = stSpiIdInfo[i][1]*stSpiIdInfo[i][2];
			/*pr_err("spiSize:0x%x\n",*spiSize);*/
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( stSpiIdInfo[i][0] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}

	return type;
}


/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIInit
 *  return SUCCESS: normal
		   FAIL: if wait spi flash time out
 *------------------------------------------------------------------------*/
UINT32 I2C_SPIFlashPortWait(void)
{
	//UINT32 cnt = WAIT_COUNT;
#if 0
	while(I2CDataRead(0x40e6) != 0x00){
		cnt--;
		if(cnt == 0x00)
		{
			pr_err("serial flash port wait time out!!\n");
			return FAIL;
		}
	}
#endif
	return SUCCESS;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashPortRead
 *  return SUCCESS: normal
		   FAIL:    if wait spi flash time out
 *------------------------------------------------------------------------*/
UINT32 I2C_SPIFlashPortRead(void)
{
	UINT32 ret;

	ret = hsI2CDataRead(0x40e4);
	/* polling SPI state machine ready */
	if (I2C_SPIFlashPortWait() != SUCCESS) {
		return 0;
	}
	ret = hsI2CDataRead(0x40e5);

	return ret;
}


/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashPortWrite
 *  return SUCCESS: normal
		   FAIL:    if wait spi flash time out
 *------------------------------------------------------------------------*/
UINT32 I2C_SPIFlashPortWrite(UINT32 wData)
{
	hsI2CDataWrite(0x40e3,(UINT8)wData);
	return I2C_SPIFlashPortWait();
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashWrEnable
 *  return none
 *------------------------------------------------------------------------*/
void I2C_SPIFlashWrEnable(void)
{
	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_EN);
	hsI2CDataWrite(0x40e7,0x01);
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIStsRegRead
 *  return ret
 *------------------------------------------------------------------------*/
UINT32 I2C_SPIStsRegRead(void)
{
	UINT32 ret;

	hsI2CDataWrite(0x40e7,0x00);

	I2C_SPIFlashPortWrite(SPI_CMD_RD_STS);
	ret = I2C_SPIFlashPortRead();

	hsI2CDataWrite(0x40e7,0x01);

	return ret;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPITimeOutWait
 *  return none
 *------------------------------------------------------------------------*/
void I2C_SPITimeOutWait(UINT32 poll, UINT32 *ptimeOut)
{
	/* MAX_TIME for SECTOR/BLOCK ERASE is 25ms */
	UINT32 sts;
	UINT32 time = 0;
	while (1) {
		sts = I2C_SPIStsRegRead();
		if (!(sts & poll))	/* sfStatusRead() > 4.8us */ {
			break;
		}
		time ++;
		if( *ptimeOut < time ) {
			pr_err("TimeOut %d, sts=0x%x, poll=0x%x\n",time,sts,poll);
			break;
		}
	}
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashReadId
 *  return SPI flash ID
		   0: if read ID failed
 *------------------------------------------------------------------------*/
UINT32 I2C_SPIFlashReadId(void)
{
	UINT8 id[3];
	UINT32 ID;
	UINT32 err;

	id[0] = 0;
	id[1] = 0;
	id[2] = 0;

	hsI2CDataWrite(0x40e7,0x00);

	err = I2C_SPIFlashPortWrite(SPI_CMD_RD_ID); /*read ID command*/
	if (err != SUCCESS) {
		pr_err("Get serial flash ID failed\n");
		return 0;
	}

	id[0] = I2C_SPIFlashPortRead();    /* Manufacturer's  ID */
	id[1] = I2C_SPIFlashPortRead();    /* Device ID          */
	id[2] = I2C_SPIFlashPortRead();    /* Manufacturer's  ID */

	hsI2CDataWrite(0x40e7,0x01);

	pr_err("ID %2x %2x %2x\n", id[0], id[1], id[2]);

	ID = ((UINT32)id[0] << 16) | ((UINT32)id[1] << 8) | \
	((UINT32)id[2] << 0);

	return ID;
}

UINT32 I2C_SPISectorErase(UINT32 address, UINT32 stFlag)
{
	UINT32 timeout;
	pr_err("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();

		hsI2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		hsI2CDataWrite(0x40e7,0x01);
	}

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	hsI2CDataWrite(0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_SECTOR_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
	hsI2CDataWrite(0x40e7,0x01);

	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);

	return SUCCESS;
}

UINT32 I2C_SPI32KBBlockErase(UINT32 address, UINT32 stFlag)
{
	UINT32 timeout;
	pr_err("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();

		hsI2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		hsI2CDataWrite(0x40e7,0x01);
	}

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	hsI2CDataWrite(0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_32KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
	hsI2CDataWrite(0x40e7,0x01);

	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);

	return SUCCESS;
}

UINT32 I2C_SPI64KBBlockErase(UINT32 address, UINT32 stFlag)
{
	UINT32 timeout;
	pr_err("addr:0x%x\n",address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();

		hsI2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
		hsI2CDataWrite(0x40e7,0x01);
	}

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	hsI2CDataWrite(0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_64KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
	hsI2CDataWrite(0x40e7,0x01);

	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);

	return SUCCESS;
}

void BB_EraseSPIFlash(UINT32 type, UINT32 spiSize)
{
	UINT32 typeFlag = 0;
	UINT32 i, temp1;
	if( type == 2 )/* SST */
	{
		typeFlag = 0;
	}
	else if( type == 1 || type == 3 )/* ST */
	{
		typeFlag = 1;
	}
	/*pr_err("spiSize:0x%x\n",spiSize);*/
	if(spiSize == (512*1024))
	{
		/* skip 0x7B000 ~ 0x7EFF, to keep calibration data */
		temp1 = (spiSize / 0x10000)-1;
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase(temp1*0x10000,typeFlag);
		temp1 = temp1*0x10000 + 0x8000;
		for(i=temp1;i<spiSize-0x5000;i+=0x1000)
		{
			I2C_SPISectorErase(i,typeFlag);
		}
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
	else if(spiSize == (1024*1024))
	{
		temp1 = (spiSize / 0x10000)-3; /* temp1 = 0x0D */
		for(i=0;i<temp1;i++) /* 0 ~ 0xCFFFF will be erased */
		{
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
				/* 0xD0000 ~ 0xFF000 is reserved for calibration */
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
}

void I2C_SPISstStatusWrite(UINT8 dat)
{
	UINT32 timeout, poll;

	I2C_SPIFlashWrEnable();

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);
	hsI2CDataWrite(0x40e7,0x01);

	hsI2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	I2C_SPIFlashPortWrite(dat);
	hsI2CDataWrite(0x40e7,0x01);

	poll = 0x01;

	timeout = 100000;
	I2C_SPITimeOutWait(poll, &timeout);
	return;
}

UINT32 I2C_SPISstFlashWrite(UINT32 addr, UINT32 pages, UINT8 *pbuf)
{
	UINT32 i, err = SUCCESS;
	UINT32 pageSize = 0x100;
	UINT32 timeout = 100000;

	addr = addr * pageSize;

	pr_err("SST type writing...\n");
	I2C_SPISstStatusWrite(0x40);
	while( pages ) {
		pr_err("page:0x%x\n",pages);
		if((addr>=0x7C000) && (addr <0x7F000))
		{
			addr += 0x1000;
			pages -= 0x10;
			continue;
		}
		I2C_SPIFlashWrEnable();
		hsI2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);               /* Write one byte command*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 16));               /* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
		I2C_SPIFlashPortWrite((UINT8)(addr));
		I2C_SPIFlashPortWrite(*pbuf);
		pbuf++;
		I2C_SPIFlashPortWrite(*pbuf);
		pbuf++;
		hsI2CDataWrite(0x40e7,0x01);
		timeout = 100000;
		I2C_SPITimeOutWait(0x01,&timeout);

		for (i = 2; i < pageSize ; i = i+2) {
				hsI2CDataWrite(0x40e7,0x00);
				I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);
				I2C_SPIFlashPortWrite(*pbuf);
				pbuf++;
				I2C_SPIFlashPortWrite(*pbuf);
				pbuf++;
				hsI2CDataWrite(0x40e7,0x01);
				timeout = 100000;
				I2C_SPITimeOutWait(0x01,&timeout);
		  }

		  hsI2CDataWrite(0x40e7,0x00);
		  I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		  hsI2CDataWrite(0x40e7,0x01);

		  addr += pageSize;
		  pages --;

		  hsI2CDataWrite(0x40e7,0x00);
		  I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		  hsI2CDataWrite(0x40e7,0x01);
	}

	return err;
}

void I2C_7002DmemWr(UINT32 bankNum, UINT32 byteNum, UINT8* pbuf)
{
	UINT32 i, j, bank;
	UINT32 size = 64;	//4	//Rocky_20131031
	unsigned char tmp[size];	//Rocky_20131031

	bank = 0x40+bankNum;
	I2CDataWrite(0x10A6,bank);

	for(i=0;i<byteNum;i+=size)	//Rocky_20131031
	{
		memset(tmp, 0, sizeof(tmp));
		for(j=0;j<size;j++)	//Rocky_20131031
		{
					tmp[j]=(*(pbuf + i + j));
		}
		msm_camera_cci_i2c_write_seq(isp_main_cam_s_ctrl.sensor_i2c_client,(0x1800+i), tmp,size);	//Rocky_20131031
	}

	bank = 0x40 + ((bankNum+1)%2);
	hsI2CDataWrite(0x10A6,bank);
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashWrite
 *  addr: SPI flash starting address
	pages: pages size for data -> datasize = pages * pagesize(0x100)
	pbuf: data buffer
 *  return SUCCESS: normal finish
		   FAIL:    write failed
 *------------------------------------------------------------------------*/
UINT32 I2C_SPIFlashWrite(UINT32 addr,UINT32 pages,UINT8 *pbuf)
{
	UINT32 i, err = SUCCESS;
	UINT32 pageSize = 0x100;
	//UINT32 timeout = 100000;
	//UINT32 rsvSec1, rsvSec2;

	#if 0
	rsvSec1 = pages*pageSize - 0x5000;
	rsvSec2 = pages*pageSize - 0x1000;
	#endif
	addr = addr * pageSize;

	pr_err("ST type writing...\n");
	while( pages ) {
		if((pages%0x40)==0)pr_err("page:0x%x\n",pages);
		#if 0
		if((addr>=rsvSec1) && (addr <rsvSec2))
		{
			addr += 0x1000;
			pbuf += 0x1000;
			pages -= 0x10;
			continue;
		}
		#endif
		if((pages==1))
		{
			for (i = 0; i < pageSize ; i++) {
				pr_err("%2x ",*(pbuf+i));
				if((i%0x10)==0x0f) pr_err("\n");
			}
		}
		I2C_SPIFlashWrEnable();
		hsI2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);               /* Write one byte command*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 16));               /* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
		I2C_SPIFlashPortWrite((UINT8)(addr));

		for (i = 0; i < pageSize ; i++) {
			I2C_SPIFlashPortWrite(*pbuf);
			pbuf++;
		}
		hsI2CDataWrite(0x40e7,0x01);
		addr += pageSize;
		pages --;
		usleep(400);
		//tmrUsWait(400);
	}

	return err;
}

/* usage: 1 will skip writing reserved block of calibration data
		  0 will write reserved block of calibration data */
#if 1
UINT32 I2C_SPIFlashWrite_DMA(	UINT32 addr,	UINT32 pages, UINT32 usage, UINT8 *pbuf)
{
	UINT32 i, temp, err = SUCCESS;
	UINT32 pageSize = 0x100, size;
	//UINT32 timeout = 100000;
	UINT32 rsvSec1, rsvSec2;
	UINT32 dmemBank = 0;
	UINT32 chk1=0, chk2=0;
	UINT32 count;

	rsvSec1 = pages*pageSize - 0x7000;
	rsvSec2 = pages*pageSize - 0x1000;
	addr = addr * pageSize;

	/* Set DMA bytecnt as 256-1 */
	I2CDataWrite(0x4170,0xff);
	I2CDataWrite(0x4171,0x00);
	I2CDataWrite(0x4172,0x00);

	/* Set DMA bank & DMA start address */
	I2CDataWrite(0x1084,0x01);
	I2CDataWrite(0x1080,0x00);
	I2CDataWrite(0x1081,0x00);
	I2CDataWrite(0x1082,0x00);

	/* enable DMA checksum and reset checksum */
	I2CDataWrite(0x4280,0x01);
	I2CDataWrite(0x4284,0x00);
	I2CDataWrite(0x4285,0x00);
	I2CDataWrite(0x4164,0x00);

	size = pages * pageSize;
	for(i=0;i<size;i++)
	{
		if((i>=rsvSec2) || (i <rsvSec1))
		{
			chk1 += *(pbuf+i);
		}
		if(chk1>=0x10000)
		{
			chk1 -= 0x10000;
		}
	}

	while( pages )
	{
		if((pages%0x40)==0)
		{
			pr_err("%s page:0x%x\n", __func__ , pages);
		}
		if((addr>=rsvSec1) && (addr <rsvSec2) && (usage!=0))
		{
			addr += 0x1000;
			pbuf += 0x1000;
			pages -= 0x10;
			continue;
		}
		if((pages==1))
		{
			for (i = 0; i < pageSize ; i++)
			{
				pr_err("%2x ",*(pbuf+i));
				if((i%0x10)==0x0f)
					pr_err("\n");
			}
		}

		dmemBank = pages % 2;
		I2CDataWrite(0x1081,dmemBank*0x20);
		I2CDataWrite(0x1084,(1<<dmemBank));
		I2C_7002DmemWr(dmemBank,pageSize,pbuf);

		I2C_SPIFlashWrEnable();
		I2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);               /* Write one byte command*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 16));               /* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
		I2C_SPIFlashPortWrite((UINT8)(addr));

		I2CDataWrite(0x4160,0x01);
		count = 30;
		pbuf += pageSize;
		addr += pageSize;
		pages --;
		while( hsI2CDataRead(0x4003) == 0 )
		{
			count--;
			pr_err("%s count:0x%x\n", __func__ , count);
			//tmrUsWait(200);/* wait for DMA done */
			usleep(200);
			if( count == 0 )
			{
				pr_err("DMA time out: %2x, 0x%2x%2x, %2x\n",pages,hsI2CDataRead(0x4179),hsI2CDataRead(0x4178),hsI2CDataRead(0x40E6));
				hsI2CDataWrite(0x4011,0x10);
				hsI2CDataWrite(0x1010,0x02);
				pbuf -= pageSize;
				addr -= pageSize;
				pages ++;
				hsI2CDataWrite(0x1010,0x00);
				break;
			}
		}
		hsI2CDataWrite(0x4003, 0x02);
		I2CDataWrite(0x40e7,0x01);
	}

	//tmrUsWait(500);/* wait for DMA done */
	usleep(500);

	temp = hsI2CDataRead(0x4285);
	chk2 = hsI2CDataRead(0x4284);
	chk2 = chk2 | (temp<<8);
	pr_err("checksum: 0x%x 0x%x\n",chk1,chk2);

	return err;
}
#endif

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIInit
 *  return none
 *------------------------------------------------------------------------*/
void I2C_SPIInit(void)
{
	//UINT32 temp;

	/*temp = I2CDataRead(0x0026);*/
	I2CDataWrite(0x0026,0xc0);
	I2CDataWrite(0x4051,0x01); /* spien */
	I2CDataWrite(0x40e1,0x00); /* spi mode */
	I2CDataWrite(0x40e0,0x11); /* spi freq */
}

UINT32 I2C_SPIFlashRead(UINT32 addr, UINT32 pages, UINT8 *pbuf)
{
	UINT32 err = 0;
	UINT32 i, size=0;
	UINT32 pageSize = 0x100;

	addr = addr * pageSize;
	size = pages*pageSize;

	I2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);               /* Write one byte command*/
	I2C_SPIFlashPortWrite((UINT8)(addr >> 16));               /* Send 3 bytes address*/
	I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
	I2C_SPIFlashPortWrite((UINT8)(addr));

	for (i = 0; i < size ; i++) {
		*pbuf = I2C_SPIFlashPortRead();
		if((i%256)==0)
			pr_err("page:0x%x\n",(i/256));
		pbuf ++;
	}

	I2CDataWrite(0x40e7,0x01);

	return err;
}

/* BB_WrSPIFlash is main function for burning SPI */
void BB_WrSPIFlash(char * filepath)
{
	UINT32 id, type, size;
	UINT32 pages;
	UINT32 spiSize;
	int fileSize;
	struct file *file_filp = NULL;
	mm_segment_t old_fs;

	if(fw_status == FW_STATUS_UPGRADING)
		return;
	//Selwyn modified for FTM
	set_firmware_upgrade_status(FW_STATUS_UPGRADING, FW_UPGRADE_MSG_UPGRADING);
	//~Selwyn modified

	pr_err("loadcode from file\n");

	I2CDataWrite(0x70c4,0x00);
	I2CDataWrite(0x70c5,0x00);

	I2CDataWrite(0x1011,0x01); /* CPU Reset */
	I2CDataWrite(0x001C,0x08);/* FM reset */
	I2CDataWrite(0x001C,0x00);
	I2CDataWrite(0x108C,0x00);/* DMA select */
	I2CDataWrite(0x009a,0x00);/*CPU normal operation */

	//fd = sp5kFsFileOpen( FILE_PATH_BOOTCODE, SP5K_FS_OPEN_RDONLY );

	if(filepath==NULL){
		pr_err("update firmware fail\n");
		set_firmware_upgrade_status(FW_STATUS_FAIL, FW_UPGRADE_MSG_NOT_FOUND);
		return;
	}
	file_filp = filp_open(filepath, O_RDONLY, 0);
	size=firmwaresize;

	if (IS_ERR(file_filp)){
		pr_err("%s: Fail to open iCatch ISP BOOTCODE\n", __func__);
		//Selwyn 2012/10/07 modified for FTM
		set_firmware_upgrade_status(FW_STATUS_FAIL, FW_UPGRADE_MSG_NOT_FOUND);
		//~Selwyn modified
		file_filp=NULL;
		return ;
	}

	//copy boot.bin to file_buf
	old_fs=get_fs();
	set_fs(KERNEL_DS);
	fileSize = file_filp->f_op->read(file_filp, (unsigned char __user*)&g_memory_512K, firmwaresize, &file_filp->f_pos);	//Rocky_20131024

	set_fs(old_fs);
	pr_err("%s fileSize: %d\n",__func__,  fileSize);
	filp_close(file_filp, NULL);

	#if 0
	for(offset = 0; offset < 256; offset+=8)
	{
		pr_err("0x%2X 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X 0x%2X\n", *file_buf, *(file_buf+1), *(file_buf+2),
			*(file_buf+3), *(file_buf+4), *(file_buf+5), *(file_buf+6), *(file_buf+7));
		file_buf += 8;
	}
	file_buf = (file_buf -(offset +8));
	#endif

	I2C_SPIInit();

	id = I2C_SPIFlashReadId();
	if(id==0)
	{
		//Selwyn 2012/10/07 modified for FTM
		set_firmware_upgrade_status(FW_STATUS_FAIL, FW_UPGRADE_MSG_CANT_UPGRADE);
		//~Selwyn modified
		pr_err("read id failed\n");
		return;
	}

	/*pr_err("spiSize:0x%x\n",&spiSize);*/
	type = BB_SerialFlashTypeCheck(id, &spiSize);
	pr_err("FlashType: %d, spiSize: %d\n", type, spiSize);
	if( type == 0 )
	{
		//Selwyn 2012/10/07 modified for FTM
		set_firmware_upgrade_status(FW_STATUS_FAIL, FW_UPGRADE_MSG_CANT_UPGRADE);
		//~Selwyn modified
		pr_err("read id failed\n");
		return;
	}

	set_firmware_upgrade_status(FW_STATUS_UPGRADING, FW_UPGRADE_MSG_UPGRADING);

	if( size > 0 && size < fileSize)
	{
		pages = size/0x100;
		if((size%0x100)!=0)
			pages += 1;
	}
	else
	{
		pages = fileSize/0x100;
	}
	/*pr_err("pages:0x%x\n",pages);*/
	BB_EraseSPIFlash(type,spiSize);

	if( type == 2 )
	{
		pr_err("SST operation\n");
		I2C_SPISstFlashWrite(0,pages,g_memory_512K);	//Rocky_20131024
	}
	else if( type == 1 || type == 3 )
	{
		pr_err("ST operation\n");
		//I2C_SPIFlashWrite(0, pages, file_buf);
		I2C_SPIFlashWrite_DMA(0,pages,1,g_memory_512K);	//Rocky_20131024
	}

	/* Check checksum */
	set_firmware_upgrade_status(FW_STATUS_READY, FW_UPGRADE_MSG_UPGRADE_DONE);
}
#endif

void I2C_7002DmemRd(UINT32 bankNum, UINT32 byteNum, UINT8* pbuf)
{
	UINT32 i, j, bank;
	unsigned char tmp[4];

	bank = 0x40+bankNum;
	hsI2CDataWrite(0x10A6,bank);

	/* I2C sequential read: here 4 bytes each time */
	for(i=0;i<byteNum;i+=4)
	{
		//seqI2CDataRead((0x1800+i),(pbuf+i));
		memset(tmp, 0, sizeof(tmp));
		msm_camera_cci_i2c_read_seq(isp_main_cam_s_ctrl.sensor_i2c_client,(0x1800+i), tmp,4);
		for(j=0;j<4;j++)
		{
			(*(pbuf + i + j)) = tmp[j];
		}
		//pr_err("%s: 0x%X 0x%X 0x%X 0x%X\n", __func__, tmp[0], tmp[1], tmp[2], tmp[3]);
	}

	bank = 0x40 + ((bankNum+1)%2);
	hsI2CDataWrite(0x10A6,bank);
}

UINT32 I2C_SPIFlashRead_DMA(UINT32 addr, UINT32 size, UINT8 *pbuf)
{
	UINT32 err = 0, dmemBank;
	UINT32 count=0, tempSize;
	UINT32 pageSize = 0x100;

	/* Set DMA bytecnt as 256-1 */
	I2CDataWrite(0x4170,0xff);
	I2CDataWrite(0x4171,0x00);
	I2CDataWrite(0x4172,0x00);

	/* Set DMA bank & DMA start address */
	I2CDataWrite(0x1084,0x01);
	I2CDataWrite(0x1080,0x00);
	I2CDataWrite(0x1081,0x00);
	I2CDataWrite(0x1082,0x00);

	/* enable DMA checksum and reset checksum */
	I2CDataWrite(0x4280,0x01);
	I2CDataWrite(0x4284,0x00);
	I2CDataWrite(0x4285,0x00);
	I2CDataWrite(0x4164,0x01);

	while(size)
	{
		if( size > pageSize )
		{
			tempSize = 0x100;
			size -= tempSize;
		}
		else
		{
			tempSize = size;
			size = 0;
		}

		I2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);               /* Write one byte command*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 16));               /* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
		I2C_SPIFlashPortWrite((UINT8)(addr));

		if( ((size/0x100)%0x40)==0x00 )
		{
			pr_err("RE:0x%x\n",((size/0x100)%0x40));
		}
		dmemBank = count % 2;
		I2CDataWrite(0x1081,dmemBank*0x20);
		I2CDataWrite(0x1084,(1<<dmemBank));
		I2CDataWrite(0x4160,0x01);
		usleep(100);
		//tmrUsWait(100);
		I2CDataWrite(0x40e7,0x01);
		I2C_7002DmemRd(dmemBank,tempSize,pbuf);

		pbuf += pageSize;
		addr += pageSize;
		count++;
	}

	return err;
}

void BB_WrSPIFlash_UpdateNative(void)
{
	UINT32 id, type;
	UINT32 pages;
	UINT32 spiSize;
	unsigned int fileSize;
	unsigned char * fileBuf;
	int err;

	if(fw_status == FW_STATUS_UPGRADING)
		return;
	//set_firmware_upgrade_status(FW_STATUS_UPGRADING, FW_UPGRADE_MSG_UPGRADING);

	if(isp_is_power_on == 0)
	{
		//SW4-HL-Camera-ErrorHandling-00*{_20140410
		if (isp_main_cam_power_up(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: power up failed!\n", __func__);
			return;
		}
		//SW4-HL-Camera-ErrorHandling-00*}_20140410
	}

	// Avoid laoding code to cause error
	msleep(400);

	I2CDataWrite(0x70c4,0x00);
	I2CDataWrite(0x70c5,0x00);
	I2CDataWrite(0x1011,0x01); /* CPU Reset */
	I2CDataWrite(0x001C,0x08);/* FM reset */
	I2CDataWrite(0x001C,0x00);
	I2CDataWrite(0x108C,0x00);/* DMA select */
	I2CDataWrite(0x009a,0x00);/*CPU normal operation */

	// Avoid laoding code to cause error
	msleep(400);

	I2C_SPIInit();

	id = I2C_SPIFlashReadId();
	if(id==0)
	{
		pr_err("read id failed\n");
		return;
	}

	set_firmware_upgrade_status(FW_STATUS_UPGRADING, FW_UPGRADE_MSG_UPGRADING);

	fileSize = firmwaresize;
	fileBuf = iCatch7002a_img;

	/*pr_err("spiSize:0x%x\n",&spiSize);*/
	type = BB_SerialFlashTypeCheck(id, &spiSize);
	pr_err("FlashType: %d, spiSize: %d\n", type, spiSize);
	if( type == 0 )
	{
		pr_err("read id failed\n");
		return;
	}

	if(fileSize > 0)
	{
		pages = fileSize/0x100;
		if((fileSize%0x100)!=0)
			pages += 1;
	}
	else
	{
		pages = fileSize/0x100;
	}
	/*pr_err("pages:0x%x\n",pages);*/
	BB_EraseSPIFlash(type,spiSize);

	if( type == 2 )
	{
		pr_err("SST operation\n");
		I2C_SPISstFlashWrite(0,pages,fileBuf);
	}
	else if( type == 1 || type == 3 )
	{
		pr_err("ST operation\n");
		//I2C_SPIFlashWrite(0, pages, file_buf);
		err = I2C_SPIFlashWrite_DMA(0, pages, 1, fileBuf);
	}

	set_firmware_upgrade_status(FW_STATUS_READY, FW_UPGRADE_MSG_UPGRADE_DONE);
	//SW4-HL-Camera-ErrorHandling-00*{_20140410
	if (isp_main_cam_power_down(&isp_main_cam_s_ctrl) < 0)
	{
		pr_err("%s: power down failed!\n", __func__);
		return;
	}
	//SW4-HL-Camera-ErrorHandling-00*}_20140410

	return;
}

static ssize_t isp_firmware_update_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	struct file *file_filp = NULL;
	loff_t offset = 0;
	UINT32 fileSize = 0;
	mm_segment_t old_fs;

	if(isp_is_power_on == 0)//If ISP already power on by other control,Ignore update command.
	{
		file_filp = filp_open("/data/dump_isp.bin", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
		if (IS_ERR(file_filp))
		{
			pr_err("%s: Fail to open /data/dump_isp.bin\n", __func__);
			file_filp=NULL;
			return 0;
		}

		//SW4-HL-Camera-ErrorHandling-00*{_20140410
		if (isp_main_cam_power_up(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: power up failed!\n", __func__);
			return 0;
		}
		//SW4-HL-Camera-ErrorHandling-00*}_20140410

		// Avoid laoding code to cause error
		msleep(400);

		I2CDataWrite(0x70c4,0x00);
		I2CDataWrite(0x70c5,0x00);
		I2CDataWrite(0x1011,0x01);
		I2CDataWrite(0x001C,0x08);
		I2CDataWrite(0x001C,0x00);
		I2CDataWrite(0x108C,0x00);
		I2CDataWrite(0x009a,0x00);

		I2C_SPIInit();
		I2C_SPIFlashReadId();

		/* pages = (1024*1024)/256 */
		//I2C_SPIFlashRead(0, 4096, file_buf);
		I2C_SPIFlashRead_DMA(0, firmwaresize, g_memory_512K);	//Rocky_20131024

		//copy boot.bin to file_buf
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(file_filp->f_op != NULL && file_filp->f_op->write != NULL)
		{
			fileSize = file_filp->f_op->write(file_filp, (unsigned char __user *)&g_memory_512K, firmwaresize, &offset);	//Rocky_20131024
		}
		else
		{
			pr_err("%s: operation error\n", __func__);
		}
		if(fileSize == 0)
		{
			pr_err("%s: write error\n", __func__);
		}
		else
		{
			pr_err("%s: write size: %d\n", __func__, fileSize);
		}
		set_fs(old_fs);

		filp_close(file_filp, NULL);
		pr_err("%s: dump completed\n", __func__);

		//SW4-HL-Camera-ErrorHandling-00*{_20140410
		if (isp_main_cam_power_down(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: power down failed!\n", __func__);
			return 0;
		}
		//SW4-HL-Camera-ErrorHandling-00*}_20140410
	}

	return 0;
}

static ssize_t isp_firmware_update_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	do_firmware_upgrade(isp_get_firmware_path());
	return size;
}

static ssize_t isp_read_register_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg = 0;
	uint16_t value = 0;

	if (sscanf(buf, "%x", &reg) <= 0) {
		pr_err("Could not tranform the register value\n");
		return -EINVAL;
	}

	msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, reg, &value, MSM_CAMERA_I2C_BYTE_DATA);
//	value = hsI2CDataRead(reg);
	pr_err("Register 0x%X: 0x%X\n", reg, value);
	return strlen("isp_read_register_store\n");
}

static ssize_t isp_write_register_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	unsigned int rc=0,reg = 0,value;
//	uint16_t write_value = 0;

	if (sscanf(buf, "%x,%x", &reg , &value) <= 0) {
		pr_err("Could not tranform the register value\n");
		return -EINVAL;
	}
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, (uint16_t)reg, (uint16_t) value, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("Write Register 0x%X: 0x%X\n", reg, value);
	if(rc){
		pr_err("Write Register fail\n");
	}

	return strlen("isp_read_register_store\n");
}

static ssize_t isp_roi_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	unsigned int rc=0,roi_size = 0,roi_x=0,roi_y=0;
//	uint16_t write_value = 0;

	if (sscanf(buf, "%x,%x,%x", &roi_size , &roi_x,&roi_y) < 0) {
		pr_err("Could not tranform the register value\n");
		return -EINVAL;
	}

	//AF ROI
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA); //ROI on

	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7140, ((uint16_t)roi_size >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7141, ((uint16_t)roi_size & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7142, ((uint16_t)roi_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7143, ((uint16_t)roi_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7144, ((uint16_t)roi_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7145, ((uint16_t)roi_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);

	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7148, ((uint16_t)roi_size >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7149, ((uint16_t)roi_size & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x714a, ((uint16_t)roi_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x714b, ((uint16_t)roi_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x714c, ((uint16_t)roi_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x714d, ((uint16_t)roi_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
    msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x714E, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA);

	msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x243F, 0x03, MSM_CAMERA_I2C_BYTE_DATA);

	pr_err("%s,Set ROI roi_size= 0x%X: roi_x=0x%X roi_y=0x%X \n",__func__, roi_size, roi_x, roi_y);
	if(rc){
		pr_err("Write isp_roi fail\n");
	}

	return strlen("isp_read_register_store\n");
}
static int to_string_firmware_upgrade_status(char *buf)
{
	switch(fw_status)
	{
		case FW_STATUS_READY:
			sprintf(buf, "ready\n");
			break;
		case FW_STATUS_UPGRADING:
			sprintf(buf, "upgrading\n");
			break;
		case FW_STATUS_FAIL:
			sprintf(buf, "fail\n");
			break;
		default:
			sprintf(buf, "unknown\n");
	}

	return strlen(buf);
}

static int to_string_firmware_upgrade_msg(char *buf)
{
	switch(fw_upgrade_msg)
	{
		case FW_UPGRADE_MSG_READY:
			sprintf(buf, "ready\n");
			break;
		case FW_UPGRADE_MSG_UPGRADING:
			sprintf(buf, "upgrading\n");
			break;
		case FW_UPGRADE_MSG_DO_NOT_NEED_UPGRADE:
			sprintf(buf, "no need\n");
			break;
		case FW_UPGRADE_MSG_UPGRADE_DONE:
			sprintf(buf, "done\n");
			break;
		//Selwyn 2012/10/07 modified for FTM
		case FW_UPGRADE_MSG_NOT_FOUND:
			sprintf(buf, "no file\n");
			break;
		case FW_UPGRADE_MSG_CANT_UPGRADE:
			sprintf(buf, "error\n");
			break;
		//Selwyn 2012/10/07
		default:
			sprintf(buf, "unknown\n");
	}

	return strlen(buf);
}

static void set_firmware_upgrade_status(FW_STATUS status, FW_UPGRADE_MSG msg)
{
	char status_buf[15]={0}, msg_buf[15]={0};

	fw_status = status;
	fw_upgrade_msg = msg;

	to_string_firmware_upgrade_status(status_buf);
	to_string_firmware_upgrade_msg(msg_buf);
	pr_debug("@@@@ %s @@@@ status = %s\n", __func__, status_buf);
	pr_debug("@@@@ %s @@@@ msg = %s\n", __func__,msg_buf);
}

static ssize_t firmware_upgrade_status_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	return to_string_firmware_upgrade_status(buf);
}

static void do_firmware_upgrade(char *filepath)
{
	pr_debug("%s: filepath = %s\n", __func__, filepath);
	if(isp_is_power_on == 0)//If ISP already power on by other control,Ignore update command.
	{
		if (isp_main_cam_power_up(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s power up failed\n", __func__);
			return;
		}
		msleep(400);// Avoid laoding code to cause error
		isp_firmware_version=0x0;
		BB_WrSPIFlash(filepath);
		//SW4-HL-Camera-ErrorHandling-00*{_20140410
		if (isp_main_cam_power_down(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: power down failed!\n", __func__);
			return;
		}
		//SW4-HL-Camera-ErrorHandling-00*}_20140410

	}
}

static ssize_t isp_platform_version_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	if(isp_firmware_version==0)
	{
		if(isp_is_power_on == 0)
		{
			if (isp_main_cam_power_up(&isp_main_cam_s_ctrl) < 0)	//isp_main_cam_power_up will call isp_get_firmware_version
			{
				pr_err("%s power up failed\n", __func__);
				return strlen(buf);
			}

			//SW4-HL-Camera-ErrorHandling-01*{_20140429
			if (isp_main_cam_power_down(&isp_main_cam_s_ctrl) < 0)
			{
				pr_err("%s: power down failed!\n", __func__);
				return strlen(buf);
			}
			//SW4-HL-Camera-ErrorHandling-01*}_20140429
		}
		else
		{
			isp_get_firmware_version();
		}
	}

	if(((isp_firmware_version & 0xFF000000) >> 24) != 0x00)
	{
		pr_err("[RK]illegal ISP Firmware=0x%08x!\n",isp_firmware_version);
		sprintf(buf, "V00.00.00.00\n");
	}
	else
	{
		sprintf(buf, "V%02X.%02X.%02X.%02X\n",(isp_firmware_version & 0xFF000000) >> 24
		                                    ,(isp_firmware_version & 0x00FF0000) >> 16
		                                    ,(isp_firmware_version & 0x0000FF00) >> 8
		                                    ,(isp_firmware_version & 0x000000FF));
	}

	return strlen(buf);
}

static ssize_t isp_firmware_update_by_name_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	char filename[256] = {0};
	/* Parse last one byte from input */
	if(*(buf + (strlen(buf) -1)) == 0x0A)
	{
		memcpy(filename, buf, strlen(buf)-1);
	}
	else
	{
		memcpy(filename, buf, strlen(buf));
	}
	if(filename != NULL)
	{
		if(strncmp(filename, FILE_PATH_BOOTCODE, strlen(FILE_PATH_BOOTCODE))==0)
			do_firmware_upgrade(isp_get_firmware_path());
		else
			do_firmware_upgrade(filename);
	}
	return size;
}

static ssize_t isp_firmware_update_by_name_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	return to_string_firmware_upgrade_msg(buf);
}

static ssize_t isp_firmware_update_native_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	return to_string_firmware_upgrade_msg(buf);
}

static ssize_t isp_firmware_update_native_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	if(fw_status != FW_STATUS_UPGRADING)
	{
		isp_firmware_version=0x0;
		BB_WrSPIFlash_UpdateNative();
	}
	return size;
}

static ssize_t isp_file_version_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	int fileSize;
	struct file *file_filp = NULL;
	mm_segment_t old_fs;
	//char *filepath=NULL;
	char filepath[256] = {0};

	/* Parse last one byte from input */
	if(*(buf + (strlen(buf) -1)) == 0x0A)
	{
		memcpy(filepath, buf, strlen(buf)-1);
	}
	else
	{
		memcpy(filepath, buf, strlen(buf));
	}
	if(filepath==NULL){
		pr_err("update firmware fail\n");
		return size;
	}
	if(strncmp(filepath, FILE_PATH_BOOTCODE, strlen(FILE_PATH_BOOTCODE))==0)
	{
		if(isp_get_firmware_path() == NULL)
			return size;
		else
			file_filp = filp_open(isp_get_firmware_path(), O_RDONLY, 0);
	}
	else
	{
		file_filp = filp_open(filepath, O_RDONLY, 0);
	}

	if (IS_ERR(file_filp)){
		pr_err("%s: Fail to open iCatch ISP BOOTCODE\n", __func__);
		file_filp=NULL;
		return size;
	}

	//copy boot.bin to file_buf
	old_fs=get_fs();
	set_fs(KERNEL_DS);
	fileSize = file_filp->f_op->read(file_filp, (unsigned char __user *)&g_memory_512K, firmwaresize, &file_filp->f_pos);	//Rocky_20131024
	set_fs(old_fs);
	pr_err("%s fileSize: %d\n",__func__,  fileSize);
	filp_close(file_filp, NULL);
	bin_file_version = ((g_memory_512K[fileSize-1] << 24) | (g_memory_512K[fileSize-2] << 16) | (g_memory_512K[fileSize-3] << 8) | (g_memory_512K[fileSize-4]));	//Rocky_20131024
	pr_err("file_version 0x%08X\n", bin_file_version);

	return size;
}
static ssize_t isp_file_version_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	if(isp_get_firmware_path() == NULL)
		sprintf(buf, "V00.00.00.00\n");
	else
		sprintf(buf, "V%02X.%02X.%02X.%02X\n", (unsigned char)(bin_file_version>>24), (unsigned char)(bin_file_version>>16), (unsigned char)(bin_file_version>>8), (unsigned char)bin_file_version);
	return strlen(buf);
}

static ssize_t isp_flash_on_off_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	if(isp_is_power_on == 0)
	{
		//SW4-HL-Camera-ErrorHandling-00*{_20140415
		if (isp_main_cam_power_up(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: power up failed!\n", __func__);
			return strlen("isp_flash_on_off_store\n");	//SW4-HL-Camera-ErrorHandling-01*_20140429
		}

		if (isp_main_cam_match_id(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: match id failed!\n", __func__);
			return strlen("isp_flash_on_off_store\n");	//SW4-HL-Camera-ErrorHandling-01*_20140429
		}
		//SW4-HL-Camera-ErrorHandling-00*}_20140415
		/* Need to enter preview mode the flash can work */
		//RL*-Camera-Fix flash led on without entering Preview mode-{_20131021
		//isp_main_cam_write_res_settings(&isp_main_cam_s_ctrl, 1);
		//isp_main_cam_start_stream(&isp_main_cam_s_ctrl);
		//RL*-Camera-Fix flash led on without entering Preview mode-}_20131021
	}

	if(strncmp(buf, "on", strlen("on")) == 0)
	{
		pr_debug("%s: on\n", __func__);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7104, 02, MSM_CAMERA_I2C_BYTE_DATA);
	}
	else if(strncmp(buf, "off", strlen("off")) == 0)
	{
		pr_debug("%s: off\n", __func__);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7104, 01, MSM_CAMERA_I2C_BYTE_DATA);
		msleep(100);
		//SW4-HL-Camera-ErrorHandling-00*{_20140415
		if (isp_main_cam_power_down(&isp_main_cam_s_ctrl) < 0)
		{
			pr_err("%s: power down failed!\n", __func__);
			return strlen("isp_flash_on_off_store\n");
		}
		//SW4-HL-Camera-ErrorHandling-00*}_20140415
	}
	//ChunJeng modify for VN2 FTM flash led 20141031 Start
	else if(strncmp(buf, "torch", strlen("torch")) == 0)
	{
		pr_debug("%s: torch\n", __func__);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7136, 40, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7104, 04, MSM_CAMERA_I2C_BYTE_DATA);
	}
	else if(strncmp(buf, "LEDtorch", strlen("LEDtorch")) == 0)
	{
		pr_debug("%s: LED1torch\n", __func__);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7136, 55, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7104, 04, MSM_CAMERA_I2C_BYTE_DATA);
	}
	else if(strncmp(buf, "LED2torch", strlen("LED2torch")) == 0)
	{
		pr_debug("%s: LED2torch\n", __func__);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7136, 24, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7104, 04, MSM_CAMERA_I2C_BYTE_DATA);
	}
	//ChunJeng modify for VN2 FTM flash led 20141031 End
	return strlen("isp_flash_on_off_store\n");
}

//fihtdc,derekcwwu modify start
//SW4-RL-Camera-implementOIS-00+{_20140430
static ssize_t isp_ois_on_off_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	pr_debug("%s:+++\n", __func__);
	if(isp_is_power_on == 0)
	{
		if(strncmp(buf, "on", strlen("on")) == 0)
		{
			isp_ois_on = 1;
			pr_debug("%s isp_ois_on = %d \n", __func__,isp_ois_on);
		}
		else if(strncmp(buf, "off", strlen("off")) == 0)
		{
			isp_ois_on = 0;
			pr_debug("%s isp_ois_on = %d \n", __func__,isp_ois_on);
		}
	}else
	{
		uint16_t ois_val=0;
		if(strncmp(buf, "on", strlen("on")) == 0)
		{
			msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x729C, &ois_val, MSM_CAMERA_I2C_BYTE_DATA);
			ois_val |= 0x01;
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x711C, ois_val, MSM_CAMERA_I2C_BYTE_DATA);
			pr_debug("%s power on and ois on \n", __func__);
		}
		else if(strncmp(buf, "off", strlen("off")) == 0)
		{
			msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x729C, &ois_val, MSM_CAMERA_I2C_BYTE_DATA);
			ois_val &= 0xFE;
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x711C, ois_val, MSM_CAMERA_I2C_BYTE_DATA);
			pr_debug("%s power on and ois off\n", __func__);
		}
	}
	pr_debug("%s:---\n", __func__);
	return strlen("isp_ois_on_off_store\n");
}
//SW4-RL-Camera-implementOIS-00+}_20140430
//fihtdc,derekcwwu modify end
//SW4-RL-FTM-implementEXIF-00+{_20140827
static ssize_t isp_EXIF_info_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	EXIF_info_t exif;
	//char buf1[20];
	char tmp[30], str1[4];

	memset(str1, 0, sizeof(str1));
	strncpy (str1, camera_firmware_name, 3);

	if(isp_is_power_on == 0)
	{
		pr_err("%s: isp power off, cannot use this feature now\n", __func__);
		return strlen(buf);
	}
	else
	{
		/*ISO Speed*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72b7, &exif.iso_value_L, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72b8, &exif.iso_value_H, MSM_CAMERA_I2C_BYTE_DATA);
		exif.iso_value = (exif.iso_value_H << 8) + exif.iso_value_L;
		sprintf(tmp, "%d", exif.iso_value);
		strcat(buf, tmp);

		/*Exposure Time*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72b0, &exif.exposure_numerator, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72b1, &exif.exposure_denominator_L, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72b2, &exif.exposure_denominator_M, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72b3, &exif.exposure_denominator_H, MSM_CAMERA_I2C_BYTE_DATA);
		exif.exposure_denominator = (exif.exposure_denominator_H << 16) + (exif.exposure_denominator_M << 8) + exif.exposure_denominator_L;
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.exposure_numerator);
		strcat(buf, tmp);
		sprintf(tmp, "%d", exif.exposure_denominator);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*Flash LED status*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72c3, &exif.flash_state, MSM_CAMERA_I2C_BYTE_DATA);
		exif.flash_state = exif.flash_state&0x04;
		sprintf(tmp, "%d", exif.flash_state);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*Metering mode*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x728e, &exif.metering_mode, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.metering_mode);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*capExpIdx*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72d8, &exif.capExpIdx, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.capExpIdx);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*capAgcIdx*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72d9, &exif.capAgcIdx, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.capAgcIdx);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*pvExpIdx*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72da, &exif.pvExpIdx, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.pvExpIdx);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*pvAgcIdx*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72db, &exif.pvAgcIdx, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.pvAgcIdx);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*AE Entry*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72dc, &exif.aeEntry, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.aeEntry);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*AE Luma*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72dd, &exif.aeLuma, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.aeLuma);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*AE Step*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72de, &exif.aeStep, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.aeStep);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*AE Target*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72df, &exif.aetarget, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.aetarget);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*AWB R_Gain*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e0, &exif.awbRGain_LSB, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e1, &exif.awbRGain_MSB, MSM_CAMERA_I2C_BYTE_DATA);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.awbRGain_LSB);
		strcat(buf, tmp);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.awbRGain_MSB);
		strcat(buf, tmp);
		strcat(buf, ",");

		/*AWB B_Gain*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e2, &exif.awbBGain_LSB, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e3, &exif.awbBGain_MSB, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.awbBGain_LSB);
		strcat(buf, tmp);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.awbBGain_MSB);
		strcat(buf, tmp);

		/*AWB Candidate Idx*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e4, &exif.awbiCandidateIdx, MSM_CAMERA_I2C_BYTE_DATA);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.awbiCandidateIdx);
		strcat(buf, tmp);

		/*AWB Dynamic IQC Idx*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e5, &exif.awbDynamicIQCIdx, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.awbDynamicIQCIdx);
		strcat(buf, ",");
		strcat(buf, tmp);
		strcat(buf, ",");

		/*AF Vertical step*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e6, &exif.afVertical_step_LSB, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e7, &exif.afVertical_step_MSB, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.afVertical_step_LSB);
		strcat(buf, tmp);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.afVertical_step_MSB);
		strcat(buf, tmp);
		strcat(buf, ",");

		/*AF Multi step*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e8, &exif.afMulti_Step_LSB, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72e9, &exif.afMulti_Step_MSB, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.afMulti_Step_LSB);
		strcat(buf, tmp);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.afMulti_Step_MSB);
		strcat(buf, tmp);
		strcat(buf, ",");

		/*AF Final step*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72ea, &exif.afFinal_Step_LSB, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72eb, &exif.afFinal_Step_MSB, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.afFinal_Step_LSB);
		strcat(buf, tmp);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.afFinal_Step_MSB);
		strcat(buf, tmp);

		/*AF Final Result*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72ec, &exif.afFinal_Result_LSB, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72ed, &exif.afFinal_Result_MSB, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.afFinal_Result_LSB);
		strcat(buf, ",");
		strcat(buf, tmp);
		strcat(buf, ",");
		sprintf(tmp, "%d", exif.afFinal_Result_MSB);
		strcat(buf, tmp);

		/*3A Version*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72ee, &exif.threeA_Version, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.threeA_Version);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*3A ID*/
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72ef, &exif.threeA_ID, MSM_CAMERA_I2C_BYTE_DATA);
		sprintf(tmp, "%d", exif.threeA_ID);
		strcat(buf, ",");
		strcat(buf, tmp);

		/*Project Name*/
		strcat(buf, ",");
		strcat(buf, str1);

		pr_err("%s: %s\n", __func__, buf);
	}

	return strlen(buf);
}
//SW4-RL-FTM-implementEXIF-00+}_20140827
static DEVICE_ATTR(firmware_update, 0644, isp_firmware_update_show, isp_firmware_update_store);
static DEVICE_ATTR(firmware_update_native, 0644, isp_firmware_update_native_show, isp_firmware_update_native_store);
static DEVICE_ATTR(register_read, 0644, NULL, isp_read_register_store);
static DEVICE_ATTR(register_write, 0644, NULL, isp_write_register_store);
static DEVICE_ATTR(platform_version, 0644, isp_platform_version_show, NULL);
static DEVICE_ATTR(firmware_update_by_name, 0644, isp_firmware_update_by_name_show, isp_firmware_update_by_name_store);
static DEVICE_ATTR(file_version, 0644, isp_file_version_show, isp_file_version_store);
static DEVICE_ATTR(roi, 0644, NULL, isp_roi_store);
static DEVICE_ATTR(flash_on_off, 0644, NULL, isp_flash_on_off_store);
static DEVICE_ATTR(ois_on_off, 0644, NULL, isp_ois_on_off_store);		//SW4-RL-Camera-implementOIS-00+_20140430
static DEVICE_ATTR(firmware_status, 0644, firmware_upgrade_status_show, NULL);
static DEVICE_ATTR(get_EXIF_data, 0644, isp_EXIF_info_show, NULL);//SW4-RL-FTM-implementEXIF-00+_20140827

static struct attribute *isp_attributes[] = {
		&dev_attr_firmware_update.attr,
		&dev_attr_firmware_update_native.attr,
		&dev_attr_register_read.attr,
		&dev_attr_register_write.attr,
		&dev_attr_platform_version.attr,
		&dev_attr_firmware_update_by_name.attr,
		&dev_attr_file_version.attr,
		&dev_attr_roi.attr,
		&dev_attr_flash_on_off.attr,
		&dev_attr_ois_on_off.attr,
		&dev_attr_firmware_status.attr,
		&dev_attr_get_EXIF_data.attr,
		NULL
};

static const struct attribute_group isp_attr_group = {
		.attrs = isp_attributes,
};

static struct v4l2_subdev_info isp_main_cam_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

//SW4-RL-Camera-FixFTMFocusNotWork-00+_20140305
void iCatch_set_AF_Mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);

#if ENABLE_ISP_INTERRUPT
static int thread_fn(void *arg)
{
	int value1=0, value2=0;
	UINT32 irq_check=0;
	interrup_error=-1;//-1 is error, 0 is normal ,fihtdc,derekcww,add for get 0x72f8 error

	value1 = gpio_get_value(ISP_INTR_GPIO);
	pr_debug("[CAF][ISP_INTR]=%d\n",value1);
	if(value1)
	{
		irq_check=I2CDataRead(0x72F9);
		if(irq_check&0x40)
		{
			pr_debug("%s AF start:%d\n", __func__,irq_check);
			irq_check&=(0x40UL);
			irq_check|=(0x40UL);
			I2CDataWrite(0x72F9,(UINT16)irq_check);
			value2=I2CDataRead(0x72a0);
			if(value2 == 0x01 || value2 == 0x00){
				//0x72a9 = 0x01(TAF busy) or 0x00(TAF idle) means Touch-AF case
				kobject_uevent(&dev_uevent->kobj,KOBJ_CHANGE);
			}
			else{
				//0x72a9 = 0x11(CAF busy) or 0x10(CAF idle) means C-AF case
				kobject_uevent(&dev_uevent->kobj,KOBJ_MOVE);
			}
			goto done;
		}
		irq_check=0;
		irq_check=I2CDataRead(0x72F8);
		if(irq_check&0x08UL)
		{
			pr_debug("%s AF done:%d\n", __func__,irq_check);
			value1=I2CDataRead(0x72a0);
			if(value1 == 0x01) //in busy
				pr_debug("[CAF] af busy\n");
			else if(value1 == 0x00)//in idle
				pr_debug("[CAF] af idle\n");
			else if(value1 == 0x10) //CAF idle
				pr_debug("[CAF] CAF idle\n");
			else if(value1 == 0x11) //CAF busy
				pr_debug("[CAF] CAF busy\n");
			if((value1 == 0x01)||(value1 == 0x00))
				af_not_ready=0;
			if(value1 == 0x10 || value1 == 0x11){
				//0x72a9 = 0x11(CAF busy) or 0x10(CAF idle) means C-AF case
				kobject_uevent(&dev_uevent->kobj,KOBJ_ONLINE);
			}
			else{
				//0x72a9 = 0x01(TAF busy) or 0x00(TAF idle) means Touch-AF case
				value1=I2CDataRead(0x72a1);
				pr_debug("%s,caf,af_status : %d \n", __func__,value1);
				if(value1 == 0x01) //focus failed
					value1=kobject_uevent(&dev_uevent->kobj,KOBJ_REMOVE);
				else//focus success
					value1=kobject_uevent(&dev_uevent->kobj,KOBJ_ADD);
			}
			irq_check&=(0x08UL);
			irq_check|=(0x08UL);
			I2CDataWrite(0x72F8,(UINT16)irq_check);
			goto done;
		}
		if(irq_check&0x04)
		{
			pr_debug("%s frame ready +:%d\n", __func__,irq_check);
			irq_check&=(0x04UL);
			irq_check|=(0x04UL);
			I2CDataWrite(0x72F8,(UINT16)irq_check);
			complete(&g_iCatch_comp);
			goto done;
		}
		pr_err("%s:WARNING,interrupt value maybe be error:0x72F8=%d\n", __func__,irq_check);//fihtdc,derekcww,add for get 0x72f8 error
		return 0;
	}
	else
	{
		pr_err("%s:not real INT, interrupt gpio value=%d\n",__func__,value1);//fihtdc,derekcww,add for get 0x72f8 error
	}
done:
	pr_debug("%s -\n", __func__);
	interrup_error=0;//0 is normal,fihtdc,derekcww,add for get 0x72f8 error
	return 0;
}
void isp_disable_interrupt(void)
{
	pr_debug("%s:\n", __func__);
	disable_irq(g_isp_irq);
}

void isp_enable_interrupt(void)
{
	 struct irq_desc *desc;

	pr_debug("%s\n", __func__);
	I2CDataWrite(0x72FC,0x0C);
	I2CDataWrite(0x72FD,0x40);
	init_completion(&g_iCatch_comp);
	pr_debug("\n\n********************* [HL] %s, check irq before enable it ******************************\n\n", __func__);
	desc = irq_to_desc(g_isp_irq);
	//fihtdc,derekcwwu, remove enable_irq cause request will auto enalbe irq
	//if (desc && desc->depth > 0)
	//enable_irq(g_isp_irq);
}

static irqreturn_t fih_detect_isp_irq_handler(int irq, void *dev_id)
{
#if 0
    struct task_struct *simple_tsk;

    pr_debug("%s\n", __func__);

    simple_tsk = kthread_create(thread_fn, NULL,"camera_irq_fn");
    if (IS_ERR(simple_tsk))
        pr_err("%s:can't create thread: %d\n", __func__, __LINE__);
    else
        wake_up_process(simple_tsk);
#else
    pr_debug("%s\n", __func__);
    thread_fn(NULL);
#endif
	return IRQ_HANDLED;
	}

void  isp_deinit_interrupt(void)
{
    pr_debug("%s\n", __func__);
    isp_disable_interrupt();
    free_irq(g_isp_irq,NULL);
}
int  isp_init_interrupt(unsigned int intr_gpio)//fihtdc,derekcww,add for front cam when front cam can focus
{
	int ret;

	pr_debug("%s\n", __func__);

	ISP_INTR_GPIO=intr_gpio;//fihtdc,derekcww,add for front cam when front cam can focus
	ret = gpio_direction_input(ISP_INTR_GPIO);
	if (ret < 0)
		goto err_set_isp_int_gpio;

	g_isp_irq = gpio_to_irq(ISP_INTR_GPIO);
	if (g_isp_irq < 0) {
		ret = g_isp_irq;
		goto err_get_isp_irq_num_failed;
	}

	isp_enable_interrupt();//fihtdc,derek,should init before request

	ret = request_threaded_irq(g_isp_irq,NULL, fih_detect_isp_irq_handler,
			 IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "fih_isp_ISR", NULL);

	pr_debug("g_isp_irq=%d\n",g_isp_irq);
	if (ret < 0)
		goto err_request_detect_isp_irq;

	return ret;

err_request_detect_isp_irq:
	pr_err("%s: err_request_detect_isp_irq\n", __func__);

err_get_isp_irq_num_failed:
	pr_err("%s: err_get_isp_irq_num_failed\n", __func__);

err_set_isp_int_gpio:
	pr_err("%s: err_set_isp_int_gpio\n", __func__);
	return ret;
}
#endif

void isp_main_cam_inti_parms(void)
{
	pr_debug("%s: +\n", __func__);
	g_cur_bestshot = -1;
	g_cur_effect = -1;
	g_cur_ev = -1;
	g_cur_iso = -1;
	g_cur_aec = -1;
	g_cur_whitebalance = -1;
	g_cur_saturation = -1;
	g_cur_sharpness = -1;
	g_cur_contrast = -1;
	g_cur_antibanding = -1;
	g_cur_ledflash = -1;
	g_cur_hdr = -1;
	g_cur_aec_lock = -1;
	g_cur_awb_lock = -1;
	g_cur_wdr = -1; 	//RL*_20140117
	//fihtdc,derekcww,add for front cam when front cam can focus
	caf_mode=false;
	isp_main_cam_mode=0;
	g_pre_res=MSM_SENSOR_INVALID_RES;
	af_not_ready=0;
	//fihtdc,derekcww,add for front cam when front cam can focus
	pr_debug("%s: -\n", __func__);
}

int32_t isp_main_cam_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc=0;

	pr_debug("%s: res=%d\n", __func__, res);
	//here need to implement snapshot mode, preview mode,...etc.

	switch(res)
	{
		case MSM_SENSOR_RES_FULL: /*snapshot mode, 4160*3120*/
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_FULL *************************\n\n", __func__);
			isp_main_cam_mode = MSM_SENSOR_RES_FULL;
		break;

		case MSM_SENSOR_RES_QTR: /*preview mode, 2080*1560*/
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_QTR *************************\n\n", __func__);
			isp_main_cam_mode = MSM_SENSOR_RES_QTR;
		break;

		case MSM_SENSOR_RES_HD: /*preview mode, 1280*720*/
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_HD *************************\n\n", __func__);
			isp_main_cam_mode = MSM_SENSOR_RES_HD;
		break;

		case MSM_SENSOR_RES_FULL_PREVIEW: /*preview mode, 4160*3120*/
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_FULL_PREVIEW *************************\n\n", __func__);
			isp_main_cam_mode=MSM_SENSOR_RES_FULL_PREVIEW;
		break;

		//SW4-RL-Camera-addFor_FHD_Recording-00+{_20140423
		case MSM_SENSOR_RES_FHD: /*preview mode, 1920*1080*/
			pr_debug("\n\n******************* [RL] %s, MSM_SENSOR_RES_FHD *************************\n\n", __func__);
			isp_main_cam_mode = MSM_SENSOR_RES_FHD;
		break;
		//SW4-RL-Camera-addFor_FHD_Recording-00+}_20140423

		////SW5-Webber-Camera-add_for_HighFrameRecording_90fps_20150423++
		case MSM_SENSOR_RES_HFR90: /*HFR mode, 1040*780 90fps*/
			pr_debug("\n\n******************* [RL] %s, MSM_SENSOR_RES_HFR90 *************************\n\n", __func__);
			isp_main_cam_mode = MSM_SENSOR_RES_HFR90;
		break;
		////SW5-Webber-Camera-add_for_HighFrameRecording_90fps_20150423--

		default:
			pr_err("%s: Do not support this res=%d\n", __func__, res);
		break;
	}

	return rc;
}

//HL+{_20130911
void polling_isp_status (void)
{
	uint16_t status = 0x0;
	int count = 0;

	do{
		msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		pr_debug("\n\n********** [HL] %s: status=%d, count=%d **********\n\n", __func__, status, count);
		count ++;
	}while((status != 0x04) && (count <= 400));

	/*status is not ready*/
	if(status != 0x04)
	{
		pr_err("\n\n********** [HL] %s: status = %d is not ready!!!!! **********\n\n", __func__, status);
	}
}
//HL+}_20130911

int wait_for_next_frame(struct msm_sensor_ctrl_t *s_ctrl)
{
	u32 timeout_count = 1;
	#ifdef ENABLE_NORMAL_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
	#endif

	pr_debug("%s: E\n",__func__);

	g_time_start = jiffies;
	//Confirmed with iCatch, the worst case is 4.6s
	//But we reserve 400ms for recovery mode, so timeout will be 4.2s
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 4.2*HZ);
	pr_debug("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));
	if (!timeout_count)
	{
		pr_err("%s: interrupt timedout or ISP error, [wait_time] %d\n", __func__, jiffies_to_msecs(jiffies-g_time_start));
		pr_err("*********dump isp register before recover isp, start*********\n");
		DumpICatchRegister(s_ctrl);//fihtdc,derekcwwu, add for debug
		pr_err("*********dump isp register before recover isp, end**********\n");
		return -1;
	}
	else
	{
		pr_debug("%s interrupt done\n",__func__);

		#ifdef ENABLE_GPIO_DEBUG
		//gpio_direction_output(DEBUG_GPIO,1);
		mdelay(5);
		gpio_direction_output(DEBUG_GPIO,0);
		pr_debug("%s: DEBUG_GPIO: %d\n", __func__, gpio_get_value(DEBUG_GPIO));
		#endif

		#ifdef ENABLE_NORMAL_DEBUG
		pr_debug("=======%s START ===== \n",__func__);
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);

		pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
		#endif

		#ifdef ENABLE_NORMAL_DEBUG
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);

		pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
		pr_debug("======%s  END ====== \n",__func__);
		#endif
	}

	return 0;
}

int wait_for_AE_ready(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t AE_ready=0;
	int count=0;

	iCatch_first_open = false;

	do{
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72C3, &AE_ready, MSM_CAMERA_I2C_BYTE_DATA) < 0)
		{
			pr_err("%s,%d\n",__func__,__LINE__);
			pr_err("BBox::UEC; 9::7\n");
			return -1;
		}

		msleep(5);

		pr_debug("%s: AE_ready=%d, count=%d\n", __func__, AE_ready, count);

		count ++;
	}while(((AE_ready&0x01) != 0x01) && (count <=200));

	/*AE is not ready*/
	if((AE_ready&0x01) != 0x01)
	{
		pr_err("%s: AE_ready=%d is not ready!!!!!\n", __func__, AE_ready);
		pr_err("BBox::UEC; 9::8\n");
		return -1;
	}

	return 0;
}

int32_t isp_main_cam_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	pr_debug("\n\n******************* [HL] %s +++ *************************\n\n", __func__);
	pr_debug("%s: update_type=%d, res=%d\n", __func__, update_type, res);

	if (update_type == MSM_SENSOR_REG_INIT)
	{
		iCatch_first_open=true;
		Main_Camera_ISP_Ready=1;
		//msm_sensor_write_init_settings(s_ctrl);
	}
	else if (update_type == MSM_SENSOR_UPDATE_PERIODIC)
	{
		isp_main_cam_write_res_settings(s_ctrl, res);
		//v4l2_subdev_notify(&s_ctrl->msm_sd, CFG_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->output_settings[res].op_pixel_clk);//for test
	}

	pr_debug("\n\n******************* [HL] %s ---, rc = %d *************************\n\n", __func__, rc);

	return rc;
}

int32_t isp_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

int32_t isp_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}
struct file_operations af_fops = {
        .owner = THIS_MODULE,
 };
static int fih_camera_focus_uevent_probe (void)
{
    int result=0,error=0, devno=0;
    struct class *my_class;
    struct cdev cdev;
    dev_t dev_u = 0;

    result=alloc_chrdev_region(&dev_u,0,1,"camera_caf");
    if (result<0) {
        printk (KERN_WARNING "hello: alloc_chrdev_region fail\n");
        return result;
    }

    devno = MKDEV (MAJOR(dev_u), MINOR(dev_u));
    cdev_init (&cdev, &af_fops);
    cdev.owner = THIS_MODULE;
    cdev.ops = &af_fops;
    error = cdev_add (&cdev, devno , 1);
    if (error){
        printk (KERN_WARNING "Error %d adding char_reg_setup_cdev", error);
        return error;
    }
    /* creating your own class */
    my_class =class_create(THIS_MODULE, "camera_uevent");
    if(IS_ERR(my_class)) {
        printk(KERN_WARNING "Err: failed in creating class.\n");
        return -1;
    }

    /* register your own device in sysfs, and this will cause udevd to create corresponding device node */
    dev_uevent=device_create(my_class,NULL, devno, NULL,"af_uevent");
    if(IS_ERR(dev_uevent)) {
        printk(KERN_WARNING "Err: failed in creating cahr device.\n");
        return -1;
    }
    pr_err ( "fih_camera_focus_uevent_probe done\n");
    return 0;
}
static const struct i2c_device_id isp_main_cam_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&isp_main_cam_s_ctrl},
	{ }
};

static int32_t isp_main_cam_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &isp_main_cam_s_ctrl);
}

static struct i2c_driver isp_main_cam_i2c_driver = {
	.id_table = isp_main_cam_i2c_id,
	.probe  = isp_main_cam_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client isp_main_cam_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id isp_main_cam_dt_match[] = {
	{.compatible = "qcom,isp_main_cam", .data = &isp_main_cam_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, isp_main_cam_dt_match);

static struct platform_driver isp_main_cam_platform_driver = {
	.driver = {
		.name = "qcom,isp_main_cam",
		.owner = THIS_MODULE,
		.of_match_table = isp_main_cam_dt_match,
	},
};

static int32_t isp_main_cam_platform_probe(struct platform_device *pdev)
{
	int32_t i,rc = 0;
	const struct of_device_id *match;
    const char *ret_data=NULL,*ret_data2=NULL;//derek

    rc = fih_camera_focus_uevent_probe();
	pr_debug("%s:%d++++of_match_device++++\n", __func__, __LINE__);
	match = of_match_device(isp_main_cam_dt_match, &pdev->dev);
	pr_debug("%s:%d----of_match_device----\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log

    pr_debug("%s:%d++++power setting++++\n", __func__, __LINE__);
    ret_data = of_get_property(pdev->dev.of_node, "qcom,sensor-name", NULL);
    pr_err("camera sensor name=%s",ret_data);
    ret_data = of_get_property(pdev->dev.of_node, "firmware_name", NULL);
    strcpy(camera_firmware_name,ret_data);
    pr_err("camera firmware name=%s",camera_firmware_name);
    ret_data2 = of_get_property(pdev->dev.of_node, "powersetting", NULL);
    if(ret_data2){
        ret_data=ret_data2;
        pr_err("camera powersetting=%s",ret_data);
    }
    for(i=0;fih_cam_all_setting[i].project_power_setting;i++)
    {
        pr_debug("check name=%s",fih_cam_all_setting[i].project_name);
        if(!strncmp(ret_data,fih_cam_all_setting[i].project_name,strlen(ret_data)))
        {
            ((struct msm_sensor_ctrl_t *)(match->data))->power_setting_array.power_setting = fih_cam_all_setting[i].project_power_setting;
            ((struct msm_sensor_ctrl_t *)(match->data))->power_setting_array.size = fih_cam_all_setting[i].power_setting_size;
            preview_resolution=fih_cam_all_setting[i].qtr_resolution;//fihtdc,derekcwwu, add for tx3
            snapshot_resolution = fih_cam_all_setting[i].full_resolution;
            hd_resolution = fih_cam_all_setting[i].hd_resolution;
            pr_err("set power setting:%s, preview_resolution =%d , snapshot_resolution = %d , hd_resolution = %d",fih_cam_all_setting[i].project_name,preview_resolution,snapshot_resolution,hd_resolution);
            firmwaresize=fih_cam_all_setting[i].isp_firmware_size;
            lastpage=(((firmwaresize)/(256))-(1));
            pr_err("set power setting:%s, firmwaresize=%d",fih_cam_all_setting[i].project_name,fih_cam_all_setting[i].isp_firmware_size);
            break;
        }
    }
    pr_debug("%s:%d----power setting----\n", __func__, __LINE__);

	pr_debug("%s:%d++++msm_sensor_platform_probe++++\n", __func__, __LINE__);
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_debug("%s:%d----msm_sensor_platform_probe----\n", __func__, __LINE__);

	return rc;
}

static int __init isp_main_cam_init_module(void)
{
	int32_t rc = 0;
	pr_debug("[RK]%s:%d++++\n", __func__, __LINE__);

	//INIT_DELAYED_WORK(&CAF_work, isp_af_work);
	isp_proc_dir = proc_mkdir(ISP_DBG_PROC_DIR, NULL);
	isp_proc_entry = create_proc_entry( "enable_recovery", 0644, isp_proc_dir);
        isp_proc_entry->read_proc = isp_proc_read;
        isp_proc_entry->write_proc = isp_proc_write;
        isp_proc_entry_i2c = create_proc_entry( "enable_i2c_dbg", 0644, isp_proc_dir);
        isp_proc_entry_i2c->read_proc = isp_proc_i2c_read;
        isp_proc_entry_i2c->write_proc = isp_proc_i2c_write;

	rc = platform_driver_probe(&isp_main_cam_platform_driver,
		isp_main_cam_platform_probe);
	pr_debug("[RK]%s:%d rc %d\n", __func__, __LINE__, rc);

	if (!rc)
		return rc;

	rc = i2c_add_driver(&isp_main_cam_i2c_driver);
	pr_debug("[RK]%s:%d(%d)----\n", __func__, __LINE__, rc);

	return rc;
}

//*************************************
//ISP Function implement
//Anvoi 20130212
void iCatch_wait_AF_done(struct msm_sensor_ctrl_t *s_ctrl)
{
	int retry=0;
	u16 status;

	//Wait AF interrupt
	do{
		mutex_unlock(s_ctrl->msm_sensor_mutex);
		if(retry==0){
			msleep(120); // LiJen: wait for ISP AF process
		}else{
			msleep(15);
		}
		mutex_lock(s_ctrl->msm_sensor_mutex);

		if(g_isAFCancel == true){
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//need del
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x714f, 0x01);
			pr_debug("%s g_isAFCancel = ture\n",__func__);
			break;
		}

		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72a0, &status, MSM_CAMERA_I2C_BYTE_DATA);
		//need del
		//sensor_read_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x72a0, &status);
		pr_debug("status=0x%X, retry=%d\n",status,retry);
		retry += 1;
	} while((status != 0x00) && (retry < 250));
}

void iCatch_set_touch_AF(struct msm_sensor_ctrl_t *s_ctrl, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w){
	u32 af_w=0xc0, af_h=0xc0, af_x=0x18b, af_y=0x19f;//fihtdc,derekcwwu,fix default value to center
	//const u32 roi_bytes = 11;
	//unsigned char data[roi_bytes];
	pr_debug("%s +++\n",__func__);
	pr_debug("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);

	// get preview resolution from ISP
	if(coordinate_x == -1){
		af_x = 0x18b;  // ISP default//fihtdc,derekcwwu,fix default value to center
	}else if(coordinate_x > 0x0400){
		af_x = 0x0400;
	}else if(coordinate_x < 0){
		af_x = 0x0;
	}else{
		af_x = coordinate_x;
	}

	if(coordinate_y == -1){
		af_y = 0x19f;  // ISP default//fihtdc,derekcwwu,fix default value to center
	}else if(coordinate_y > 0x0400){
		af_y = 0x0400;
	}else if(coordinate_y < 0){
		af_y = 0x0;
	}else{
		af_y = coordinate_y;
	}

	if(rectangle_w == -1){
		af_w = 0xc0;  // ISP default//fihtdc,derekcwwu,fix default value to center
	}else if(rectangle_w > 0x0400){
		af_w = 0x0400;
	}else if(rectangle_w < 0){
		af_w = 0x0;
	}else{
		af_w = rectangle_w;
	}

	if(rectangle_h == -1){
		af_h = 0xc0; // ISP default//fihtdc,derekcwwu,fix default value to center
	}else if(rectangle_h > 0x0400){
		af_h = 0x0400;
	}else if(rectangle_h < 0){
		af_h = 0x0;
	}else{
		af_h = rectangle_h;
	}
	//Make sure NULL area(0,0,0,0) and AP center(512,512,0,0) is forcs ISP ROI Center
	if( ((af_w ==0)&&(af_x==0)&&(af_y==0)) || ((af_x==512)&&(af_y==512)&&(af_w ==0)&&(af_h==0))){
		af_w=0x147;
		af_x=0x15d;
		af_y=0x15d;
	}
	pr_debug("%s: af_x:0x%x af_y:0x%x af_w:0x%x af_h:0x%x g_TAEenable:%d\n", __func__, af_x, af_y, af_w, af_h,g_TAEenable);

	// set focus coodinate and retangle
	if(mode == ISP_FOCUS_MODE_CONTINOUS_PICTURE  ||mode ==  ISP_FOCUS_MODE_CONTINOUS_VIDEO){
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA); // iCatch:ROI only vaild in focus mode = auto
		//need del
		//sensor_write_reg(s_ctrl->sensor_i2c_client->client, 0x7105, 0x00); // iCatch:ROI only vaild in focus mode = auto
	}

	//AF ROI
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA); //ROI on

	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7140, (af_w >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7141, (af_w & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7142, (af_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7143, (af_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7144, (af_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7145, (af_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);

	//need del
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7188, 0x01);//ROI on

	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7140, (af_w >> 8));
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7141, (af_w & 0xFF));
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7142, (af_x >> 8));
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7143, (af_x & 0xFF));
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7144, (af_y >> 8));
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7145, (af_y & 0xFF));

	//sensor_write_reg_bytes(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7140, data, roi_bytes);

	//AE trigger
	if(g_TAEenable == true){
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714e, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
		//need del
		//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x714e, 0x02);
	}

	// AF trigger
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72a0, 0x01, MSM_CAMERA_I2C_BYTE_DATA);	// Clean AF state bit ??
	//need del
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7146, 0x01);
	//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x72a0, 0x01);     // Clean AF state bit

	pr_debug("%s ---\n",__func__);

}

void iCatch_start_AF(struct msm_sensor_ctrl_t *s_ctrl, bool on, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	//int retry=0;
       //u16 status;
	pr_debug("%s +++ Param(%d,%d,%d,%d,%d,%d)\n",__func__,on,mode,coordinate_x,coordinate_y,rectangle_h,rectangle_w);
	g_isAFCancel = false;
	//"[A60K][8M][NA][Others]implement cancel autofocus in 8M camera with ISP"
	if(on){
		//SW4-RL-Camera-FixFTMFocusNotWork-00+{_20140305
		if (mode == -1){
			mode = ISP_FOCUS_MODE_AUTO;
			iCatch_set_AF_Mode(s_ctrl, mode);
		}
		pr_debug("%s, mode = %d\n",__func__, mode);
		//SW4-RL-Camera-FixFTMFocusNotWork-00+}_20140305

		//SW4-RL-implementFaceTracking-00+{_20140919
		if(enableFaceTracking){
			mode = ISP_FOCUS_MODE_FACE_TRACKING;
			//iCatch_set_AF_Mode(s_ctrl, mode);
			pr_debug("%s, enable Face tracking mode\n",__func__);
		}else{
			pr_debug("%s, disable Face tracking mode\n",__func__);
		}
		//SW4-RL-implementFaceTracking-00+}_20140919

		switch(mode) {
		case ISP_FOCUS_MODE_MACRO:
			caf_mode=false;
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//need del
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7105, 0x01);
			//Any point focus setting
			iCatch_set_touch_AF(s_ctrl, mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
			break;

		case ISP_FOCUS_MODE_CONTINOUS_PICTURE:
		case ISP_FOCUS_MODE_CONTINOUS_VIDEO:
			caf_mode=true;
			//Any point focus setting
			iCatch_set_touch_AF(s_ctrl, mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			//need del
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7105, 0x03);
			break;

		case ISP_FOCUS_MODE_AUTO:
			if(af_not_ready)
				return;
			af_not_ready=1;

			caf_mode=false;
			if(g_afmode == 0)
			{
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
				//need del
				//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // auto af
			}
			else
			{
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
				//need del
				//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7105, 0x04); // full search af
			}
			//Any point focus setting
			iCatch_set_touch_AF(s_ctrl, mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
			//Wait AF done
			//iCatch_wait_AF_done();
			break;
#if 0
		case ISP_AF_MODE_NORMAL: // normal focus instead of full search af
			caf_mode=false;
			//msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7105, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			//need del
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7105, 0x04);
			//Any point focus setting
			iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
			//Wait AF done
			//iCatch_wait_AF_done();
			break;
#endif
		case ISP_FOCUS_MODE_INFINITY:
		case ISP_FOCUS_MODE_FIXED:
			caf_mode=false;
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			//need del
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7105, 0x02);
			goto end;
			//case ISP_AF_MODE_UNCHANGED:
			//case ISP_AF_MODE_MAX:
			//SW4-RL-implementFaceTracking-00+_20140919
		case ISP_FOCUS_MODE_FACE_TRACKING:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			iCatch_set_touch_AF(s_ctrl, mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
			break;
		//SW4-RL-implementFaceTracking-00+}_20140919
		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			goto end;
            }

		//Enable ROI debug
		if(true == g_enable_roi_debug){
			if( mode != ISP_FOCUS_MODE_AUTO){
				//Wait AF done
				iCatch_wait_AF_done(s_ctrl);
			}
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x243f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x243f, 0x01);
            }

	}else{
		//Cancel AutoFocus
		//AF_START: AF release
		g_isAFCancel = true;
		//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x714f, 0x01);
		pr_debug("Cancel autofocus\n");
	}

end:
	pr_debug("%s ---\n",__func__);
}

#if 1
void iCatch_set_AF_Mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	g_isAFCancel = false;
	pr_debug("[RK]%s +++mode(%d) \n",__func__,mode);
	afMode = mode;

	switch(mode) {
		case ISP_FOCUS_MODE_CONTINOUS_PICTURE:
		case ISP_FOCUS_MODE_CONTINOUS_VIDEO:
			//restart CAF if is in streaming
			pr_debug("pre_res=%d, caf_mode=%d\n",g_pre_res, caf_mode);
			//if(g_pre_res!=MSM_SENSOR_INVALID_RES && !caf_mode)
			if((g_pre_res == MSM_SENSOR_RES_QTR || g_pre_res == MSM_SENSOR_RES_HD) && caf_mode == false)
			{
				pr_debug("%s: restart CAF\n",__func__);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7140, 0x00, MSM_CAMERA_I2C_BYTE_DATA);//fihtdc,derekcwwu,modify for optical RD
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7141, 0xc0, MSM_CAMERA_I2C_BYTE_DATA);//fihtdc,derekcwwu,modify for optical RD
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7142, 0x01, MSM_CAMERA_I2C_BYTE_DATA);//fihtdc,derekcwwu,modify for optical RD
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7143, 0x8b, MSM_CAMERA_I2C_BYTE_DATA);//fihtdc,derekcwwu,modify for optical RD
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7144, 0x01, MSM_CAMERA_I2C_BYTE_DATA);//fihtdc,derekcwwu,modify for optical RD
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7145, 0x9f, MSM_CAMERA_I2C_BYTE_DATA);//fihtdc,derekcwwu,modify for optical RD
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			}
			break;
		case ISP_FOCUS_MODE_INFINITY://fihtdc,derekcwwu,20140524,add for panorama mode.
			pr_debug("%s, ISP_FOCUS_MODE_INFINITY mode\n",__func__);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		//SW4-RL-Camera-addForFocusFullSearchMode-00+{_20140117
		case ISP_FOCUS_MODE_FULL_SEARCH:
			pr_debug("[RL]%s, Focus_FULL_SEARCH mode\n",__func__);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		//SW4-RL-Camera-addForFocusFullSearchMode-00+}_20140117

		//SW4-RL-implementFaceTracking-00+{_20140919
		case ISP_FOCUS_MODE_FACE_TRACKING:
			//pr_err("[RL]%s, set Focus_FACE_TRACKING mode\n",__func__);
			break;
		//SW4-RL-implementFaceTracking-00+}_20140919

		default:
			if(af_not_ready)
				return;
			if(g_pre_res!=MSM_SENSOR_INVALID_RES)
			{
				pr_err("%s: lock Focus\n",__func__);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
				//msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x714f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			}
			break;
	}
	if(mode==ISP_FOCUS_MODE_CONTINOUS_PICTURE||mode==ISP_FOCUS_MODE_CONTINOUS_VIDEO)
		caf_mode = true;
	else
		caf_mode = false;
	pr_debug("%s ---\n",__func__);
}
#endif

//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+{_20140328
void iCatch_set_touch_AE(struct msm_sensor_ctrl_t *s_ctrl, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	u32 af_w=0x80, af_h=0x80, af_x=0x33, af_y=0x33;
	//const u32 roi_bytes = 11;
	//unsigned char data[roi_bytes];
	pr_debug("%s +++\n",__func__);
	pr_debug("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);

	// get preview resolution from ISP
	if(coordinate_x == -1){
		af_x = 0x80;  // ISP default
	}else if(coordinate_x > 0x0400){
		af_x = 0x0400;
	}else if(coordinate_x < 0){
		af_x = 0x0;
	}else{
		af_x = coordinate_x;
	}

	if(coordinate_y == -1){
		af_y = 0x80;  // ISP default
	}else if(coordinate_y > 0x0400){
		af_y = 0x0400;
	}else if(coordinate_y < 0){
		af_y = 0x0;
	}else{
		af_y = coordinate_y;
	}

	if(rectangle_w == -1){
		af_w = 0x33;  // ISP default
	}else if(rectangle_w > 0x0400){
		af_w = 0x0400;
	}else if(rectangle_w < 0){
		af_w = 0x0;
	}else{
		af_w = rectangle_w;
	}

	if(rectangle_h == -1){
		af_h = 0x33;  // ISP default
	}else if(rectangle_h > 0x0400){
		af_h = 0x0400;
	}else if(rectangle_h < 0){
		af_h = 0x0;
	}else{
		af_h = rectangle_h;
	}
	//Make sure NULL area(0,0,0,0) and AP center(512,512,0,0) is forcs ISP ROI Center
	if( ((af_w ==0)&&(af_x==0)&&(af_y==0)) || ((af_x==512)&&(af_y==512)&&(af_w ==0)&&(af_h==0))){
		af_w=0x147;
		af_x=0x15d;
		af_y=0x15d;
	}
	pr_debug("%s: af_x:0x%x af_y:0x%x af_w:0x%x af_h:0x%x g_TAEenable:%d\n", __func__, af_x, af_y, af_w, af_h,g_TAEenable);

	//AE ROI
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA); //ROI on

	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7148, (af_w >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7149, (af_w & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714A, (af_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714B, (af_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714C, (af_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714D, (af_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);

	//AE trigger
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714E, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	pr_debug("%s ---\n",__func__);
}

void iCatch_set_AEC_ROI(struct msm_sensor_ctrl_t *s_ctrl, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	pr_debug("[RK] %s, E",__func__);

	pr_debug("[RK]%s +++ AEC_ROI(%d, %d,%d,%d,%d)\n",__func__, g_TAEenable, coordinate_x, coordinate_y, rectangle_h, rectangle_w);
	if(g_TAEenable)
	{
		iCatch_set_touch_AE(s_ctrl, coordinate_x, coordinate_y, rectangle_h, rectangle_w);
	}

	pr_debug("%s +++ AEC_ROI(%d,%d,%d,%d)\n",__func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);
}

//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+}_20140328

void iCatch_set_AF_ROI(struct msm_sensor_ctrl_t *s_ctrl, int16_t enable, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w, uint8_t isFaceDetectMode)	//SW4-RL-implementFaceTracking-00+_20140919
{
	pr_debug("[RK] %s, E",__func__);

	//SW4-RL-implementFaceTracking-00+{_20140919
	if(isFaceDetectMode)
		enableFaceTracking = true;
	else
		enableFaceTracking = false;
	//SW4-RL-implementFaceTracking-00+}_20140919

	if(enable > 0)
	{
		focusROI.coordinate_x = coordinate_x;
		focusROI.coordinate_y = coordinate_y;
		focusROI.rectangle_h = rectangle_h;
		focusROI.rectangle_w = rectangle_w;
		isROIset = true;
	}
	else
	{
		focusROI.coordinate_x= -1;
		focusROI.coordinate_y = -1;
		focusROI.rectangle_h = -1;
		focusROI.rectangle_w = -1;
		isROIset = false;
	}

	pr_debug("[RL]%s +++ AF_ROI(%d, x: %d, y:%d, h:%d,w:%d, enableFaceDetectMode = %d)\n",__func__, enable, focusROI.coordinate_x, focusROI.coordinate_y, focusROI.rectangle_h, focusROI.rectangle_w, enableFaceTracking);	//SW4-RL-implementFaceTracking-00*_20140919
	pr_debug("[RK] %s isROIset = %s\n",__func__,isROIset ? "true" : "false");
	pr_debug("[RK] %s, X", __func__);
}

void iCatch_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    int temp_value = -1;

	pr_debug("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_bestshot == mode)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_bestshot = mode;

	if(mode == MSM_CAMERA_SCENE_MODE_HDR)
	{
		HDR_mode = 1;
		pr_debug("HDR_mode : %d \n", mode);
	}
	else
	{
		HDR_mode = 0;
		pr_debug("HDR_mode : %d \n", mode);
	}

	switch(mode)
	{
        //case MSM_CAMERA_SCENE_MODE_NORMAL:
		case MSM_CAMERA_SCENE_MODE_AUTO:
		case MSM_CAMERA_SCENE_MODE_OFF:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x00);
			break;
		case MSM_CAMERA_SCENE_MODE_LANDSCAPE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x06);
			break;
		case MSM_CAMERA_SCENE_MODE_SNOW:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0B);
			break;
		case MSM_CAMERA_SCENE_MODE_SUNSET:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0E, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0E);
			break;
		case MSM_CAMERA_SCENE_MODE_NIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x07);
			break;
		case MSM_CAMERA_SCENE_MODE_PORTRAIT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0A);
			break;
		case MSM_CAMERA_SCENE_MODE_BACKLIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x16, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x16);
			break;
		case MSM_CAMERA_SCENE_MODE_SPORTS:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0C);
			break;
		case MSM_CAMERA_SCENE_MODE_FLOWERS:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x13, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_PARTY:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x09);
			break;
		case MSM_CAMERA_SCENE_MODE_BEACH:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x03);
			break;
		case MSM_CAMERA_SCENE_MODE_ANTISHAKE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0D, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0D);
			break;
		case MSM_CAMERA_SCENE_MODE_CANDLELIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x04);
			break;
		case MSM_CAMERA_SCENE_MODE_FIREWORKS:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x05);
			break;
		case MSM_CAMERA_SCENE_MODE_NIGHT_PORTRAIT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x08);
			break;
		case MSM_CAMERA_SCENE_MODE_ACTION:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7109, 0x01);
			break;
        case MSM_CAMERA_BESTSHOT_TEXT:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_HIGHSENSITIVITY:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_LANDSCAPEPORTRAIT:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_KID:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x11, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_PET:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x12, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_FLOWER:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x13, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_SOFTFLOWINGWATER:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x14, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_FOOD:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x15, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_BESTSHOT_INDOOR:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x17, MSM_CAMERA_I2C_BYTE_DATA);
            break;

	case MSM_CAMERA_SCENE_MODE_HDR://already set HDR_mode = 1 before switch case
		break;
	case MSM_CAMERA_SCENE_MODE_THEATRE:
	case MSM_CAMERA_SCENE_MODE_AR:
	case MSM_CAMERA_SCENE_MODE_FACE_PRIORITY:
	case MSM_CAMERA_SCENE_MODE_BARCODE:
	case MSM_CAMERA_SCENE_MODE_MAX:
		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}
    if(mode == MSM_CAMERA_SCENE_MODE_OFF || mode == MSM_CAMERA_SCENE_MODE_AUTO)/*mode == ISP_CAMERA_BESTSHOT_NORMAL || */
    {
        temp_value = g_cur_whitebalance;
        g_cur_whitebalance = -1;
        iCatch_set_wb(s_ctrl, temp_value);
        temp_value = g_cur_effect;
        g_cur_effect = -1;
        iCatch_set_effect_mode(s_ctrl, temp_value);
    }
	pr_debug("%s ---\n",__func__);
}

void iCatch_set_effect_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_debug("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_effect == mode)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_effect = mode;
	switch(mode)
	{
		case MSM_CAMERA_EFFECT_MODE_OFF:
		//case CAMERA_EFFECT_NORMAL:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x00);
			break;
		case MSM_CAMERA_EFFECT_MODE_MONO:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x04);
			break;
		case MSM_CAMERA_EFFECT_MODE_NEGATIVE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x02);
			break;
		case MSM_CAMERA_EFFECT_MODE_SEPIA:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x03);
			break;
		case MSM_CAMERA_EFFECT_MODE_AQUA:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x01);
			break;
		case MSM_CAMERA_EFFECT_AURA:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x06);
			break;
		case MSM_CAMERA_EFFECT_VINTAGE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x07);
			break;
		case MSM_CAMERA_EFFECT_VINTAGE2:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x08);
			break;
		case MSM_CAMERA_EFFECT_LOMO:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x09);
			break;
		case MSM_CAMERA_EFFECT_RED:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0A);
			break;
		case MSM_CAMERA_EFFECT_BLUE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0B);
			break;
		case MSM_CAMERA_EFFECT_GREEN:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0C);
			break;
        case MSM_CAMERA_EFFECT_VIVID:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
            //sensor_write_reg(isp_main_cam_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0C);
            break;
        case MSM_CAMERA_EFFECT_AURA_RED:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7119, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_EFFECT_AURA_ORANGE:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7119, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_EFFECT_AURA_YELLOW:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7119, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_EFFECT_AURA_GREEN:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7119, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_EFFECT_AURA_BLUE:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7119, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_EFFECT_AURA_VIOLET:
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7119, 0x20, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_CAMERA_EFFECT_YELLOW:
		case MSM_CAMERA_EFFECT_MODE_SOLARIZE:
		case MSM_CAMERA_EFFECT_MODE_POSTERIZE:
		case MSM_CAMERA_EFFECT_MODE_WHITEBOARD:
		case MSM_CAMERA_EFFECT_MODE_BLACKBOARD:
		case MSM_CAMERA_EFFECT_MODE_EMBOSS:
		case MSM_CAMERA_EFFECT_MODE_SKETCH:
		case MSM_CAMERA_EFFECT_MODE_NEON:
		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}

	pr_debug("%s ---\n",__func__);
}
//**********************************************

//SW4-L1-HL-Camera-ImplementExposureCompensation-00+{_20130227
void iCatch_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_debug("%s +++ value(%d)\n",__func__,value);
	if(g_cur_ev == value)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_ev = value;

#if 1  //fihtdc, add for EV tuning, 2015.04.28 {++
	if(value>=-6 && value <=6){
		//value=iCatch_EV :  [6=2], [5=1.7], ..., [1=0.3], [0=0], [-1=-0.3], [-2=-0,7], ..., [-6=-2]
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7103, (6-value), MSM_CAMERA_I2C_BYTE_DATA);
	}else
		pr_err("%s value(%d) is not support \n",__func__,value);
#else
	switch(value)
	{
		case CAMERA_EXPOSURE_COMPENSATION_LV0:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7103, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_EXPOSURE_COMPENSATION_LV1:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7103, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_EXPOSURE_COMPENSATION_LV2:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7103, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_EXPOSURE_COMPENSATION_LV3:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7103, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_EXPOSURE_COMPENSATION_LV4:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7103, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}
#endif  //fihtdc, add for EV tuning, 2015.04.28 --}
	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementExposureCompensation-00+}_20130227

//SW4-L1-HL-Camera-ImplementExposureMeter-00+{_20130227
void iCatch_set_aec_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_debug("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_aec == mode)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_aec = mode;
	switch(mode)
	{
		case MSM_CAMERA_AE_MODE_FRAME_AVERAGE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710E, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			g_TAEenable = false;
			break;

		case MSM_CAMERA_AE_MODE_CENTER_WEIGHTED:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710E, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			g_TAEenable = false;
			break;

		case MSM_CAMERA_AE_MODE_SPOT_METERING:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710E, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			g_TAEenable = true;
			break;

		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}

	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementExposureMeter-00+}_20130227

//SW4-L1-HL-Camera-ImplementISO-00+{_20130304
void iCatch_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_debug("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_iso == mode)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_iso = mode;
	switch(mode)
	{
		case MSM_CAMERA_ISO_MODE_AUTO:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_ISO_MODE_100:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_ISO_MODE_200:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_ISO_MODE_400:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_ISO_MODE_800:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		//SW4-L1-HL-Camera-ImplementISO-01+{_20130313
		case MSM_CAMERA_ISO_MODE_1600:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		//RL-Camera-ImplementISO50-00+_20140106
		case MSM_CAMERA_ISO_MODE_50:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		//SW4-L1-HL-Camera-ImplementISO-01+}_20130313

		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}

	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementISO-00+}_20130304

//SW4-L1-HL-Camera-ImplementWhiteBalance-00+{_20130304
void iCatch_set_wb(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_debug("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_whitebalance == mode)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_whitebalance = mode;
	switch(mode)
	{
		case MSM_CAMERA_WB_MODE_AUTO:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710A, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_WB_MODE_DAYLIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710A, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710A, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_WB_MODE_SHADE:
			 msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710A, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_WB_MODE_FLUORESCENT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710A, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case MSM_CAMERA_WB_MODE_INCANDESCENT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710A, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}

	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementWhiteBalance-00+}_20130304
//SW4-L1-HL-Camera-ImplementSaturation-00+{_20130305
void iCatch_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_debug("%s +++ value(%d)\n",__func__,value);
	if(g_cur_saturation == value)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_saturation = value;

#if 1  //May modify it for sharpness, contrast, and saturation tuning, 2015.05.21 {++
	if((value>= -100) && (value<=100))
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7117, value, MSM_CAMERA_I2C_BYTE_DATA);
	else
		pr_err("%s value(%d) is not support \n",__func__,value);
#else
	switch(value)
	{
	//Rocky_20131001_for_QC_CAMERA_AP_{
	#if 1//orig //SW4-RK-Camera-forFIHCamera-00+20140103
		case CAMERA_SATURATION_FIHLV0:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7117, 0xD3, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SATURATION_FIHLV1:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7117, 0xE9, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SATURATION_FIHLV2:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7117, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SATURATION_FIHLV3:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7117, 0x17, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SATURATION_FIHLV4:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7117, 0x2D, MSM_CAMERA_I2C_BYTE_DATA);
			break;
	#else//for QC camera AP
		case CAMERA_SATURATION_FIHLV0:
		case CAMERA_SATURATION_FIHLV1:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7117, 0xD3, MSM_CAMERA_I2C_BYTE_DATA);//-45
			break;

		case CAMERA_SATURATION_FIHLV2:
		case CAMERA_SATURATION_FIHLV3:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7117, 0xE9, MSM_CAMERA_I2C_BYTE_DATA);//-23
			break;

		case CAMERA_SATURATION_FIHLV4:
		case CAMERA_SATURATION_FIHLV5:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7117, 0x00, MSM_CAMERA_I2C_BYTE_DATA);//0
			break;

		case CAMERA_SATURATION_FIHLV6:
		case CAMERA_SATURATION_FIHLV7:

			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7117, 0x17, MSM_CAMERA_I2C_BYTE_DATA);//23
			break;

		case CAMERA_SATURATION_FIHLV8:
		case CAMERA_SATURATION_FIHLV9:
		case CAMERA_SATURATION_FIHLV10:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7117, 0x2D, MSM_CAMERA_I2C_BYTE_DATA);//45
			break;
	#endif
	//Rocky_20131001_for_QC_CAMERA_AP_{
		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}
#endif  //May modify it for sharpness, contrast, and saturation tuning, 2015.05.21 --}
	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementSaturation-00+}_20130305

//SW4-L1-HL-Camera-ImplementSharpness-00+{_20130305
void iCatch_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_debug("%s +++ value(%d)\n",__func__,value);
	if(g_cur_sharpness == value)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_sharpness = value;

#if 1  //May modify it for sharpness, contrast, and saturation tuning, 2015.05.21 {++
	if((value>= -100) && (value<=100))
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7116, value, MSM_CAMERA_I2C_BYTE_DATA);
	else
		pr_err("%s value(%d) is not support \n",__func__,value);
#else
	switch(value)
	{
	//Rocky_20131001_for_QC_CAMERA_AP_{
	#if 1//orig //SW4-RK-Camera-forFIHCamera-00+20140103
		case CAMERA_SHARPNESS_FIHLV0:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7116, 0xA6, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SHARPNESS_FIHLV1:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7116, 0xD3, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SHARPNESS_FIHLV2:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7116, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SHARPNESS_FIHLV3:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7116, 0x2D, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_SHARPNESS_FIHLV4:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7116, 0x5A, MSM_CAMERA_I2C_BYTE_DATA);
			break;
	#else//for QC camera AP
	case CAMERA_SHARPNESS_FIHLV0:
	case CAMERA_SHARPNESS_FIHLV1:
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7116, 0xA6, MSM_CAMERA_I2C_BYTE_DATA);//-90
		break;

	case CAMERA_SHARPNESS_FIHLV2:
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7116, 0xD3, MSM_CAMERA_I2C_BYTE_DATA);//-45
		break;

	case CAMERA_SHARPNESS_FIHLV3:
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7116, 0x00, MSM_CAMERA_I2C_BYTE_DATA);//0
		break;

	case CAMERA_SHARPNESS_FIHLV4:
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7116, 0x2D, MSM_CAMERA_I2C_BYTE_DATA);//45
		break;

	case CAMERA_SHARPNESS_FIHLV5:
	case CAMERA_SHARPNESS_FIHLV6:
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7116, 0x5A, MSM_CAMERA_I2C_BYTE_DATA);//90
		break;

	#endif
//Rocky_20131001_for_QC_CAMERA_AP_}

		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}
#endif  //May modify it for sharpness, contrast, and saturation tuning, 2015.05.21 --}
	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementSharpness-00+}_20130305

//SW4-L1-HL-Camera-ImplementContrast-00+{_20130305
void iCatch_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_debug("%s +++ value(%d)\n",__func__,value);
	if(g_cur_contrast == value)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_contrast = value;
#if 1  //May modify it for sharpness, contrast, and saturation tuning, 2015.05.21 {++
	if((value>= -100) && (value<=100))
		msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7115, value, MSM_CAMERA_I2C_BYTE_DATA);
	else
		pr_err("%s value(%d) is not support \n",__func__,value);
#else
	switch(value)
	{
	//Rocky_20131001_for_QC_CAMERA_AP_{
	#if 1//orig //SW4-RK-Camera-forFIHCamera-00+20140103
		case CAMERA_CONTRAST_FIHLV0:
			//SW4-L1-HL-Camera-ImplementContrast-FineTune-01*-_20130509
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7115, 0xF1, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_CONTRAST_FIHLV1:
			//SW4-L1-HL-Camera-ImplementContrast-FineTune-01*-_20130509
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7115, 0xF6, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_CONTRAST_FIHLV2:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7115, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_CONTRAST_FIHLV3:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7115, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_CONTRAST_FIHLV4:
			msm_camera_cci_i2c_write(isp_main_cam_s_ctrl.sensor_i2c_client, 0x7115, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);
			break;
	#else//for QC camera AP
		case CAMERA_CONTRAST_FIHLV0:
		case CAMERA_CONTRAST_FIHLV1:
			//SW4-L1-HL-Camera-ImplementContrast-FineTune-01*-_20130509
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7115, 0xF1, MSM_CAMERA_I2C_BYTE_DATA);//-15
			break;

		case CAMERA_CONTRAST_FIHLV2:
		case CAMERA_CONTRAST_FIHLV3:
			//SW4-L1-HL-Camera-ImplementContrast-FineTune-01*-_20130509
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7115, 0xF6, MSM_CAMERA_I2C_BYTE_DATA);//-10
			break;

		case CAMERA_CONTRAST_FIHLV4:
		case CAMERA_CONTRAST_FIHLV5:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7115, 0x00, MSM_CAMERA_I2C_BYTE_DATA);//0
			break;

		case CAMERA_CONTRAST_FIHLV6:
		case CAMERA_CONTRAST_FIHLV7:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7115, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);//10
			break;

		case CAMERA_CONTRAST_FIHLV8:
		case CAMERA_CONTRAST_FIHLV9:
		case CAMERA_CONTRAST_FIHLV10:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7115, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);//15
			break;
	#endif
	//Rocky_20131001_for_QC_CAMERA_AP_}

		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}
#endif  //May modify it for sharpness, contrast, and saturation tuning, 2015.05.21 --}
	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementContrast-00+}_20130305

//SW4-L1-HL-Camera-ImplementAntiBanding-00+{_20130307
void iCatch_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_debug("%s +++ value(%d)\n",__func__,value);
	if(g_cur_antibanding == value)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_antibanding = value;
	switch(value)
	{
		case CAMERA_ANTIBANDING_50HZ:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7101, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_ANTIBANDING_60HZ:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7101, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}

	pr_debug("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementAntiBanding-00+}_20130307

//SW5-Webber-Camera-ImplementLedFlash-20130313-start
void iCatch_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_debug("flash_mode : %d \n", mode);
	if(g_cur_ledflash == mode)
	{
		pr_debug("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_ledflash = mode;

    if(mode == ISP_FLASH_MODE_AUTO)//auto flash
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
    else if(mode == ISP_FLASH_MODE_ON)//flash on
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
    else if(mode == ISP_FLASH_MODE_TORCH)//torch
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
    else//0 & other value, flash off
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
}
//SW5-Webber-Camera-ImplementLedFlash-20130313-end

//SW5-Marx-Camera-ImplementHDR-20130318-start
void iCatch_set_HDR_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_debug("%s%d E \n", __func__, __LINE__);
	if(mode==1)
		HDR_mode=1;
	else
		HDR_mode=0;

	pr_debug("%s%d, X, HDR_mode : %d \n", __func__, __LINE__, mode);
}
//SW5-Marx-Camera-ImplementHDR-20130318-end

//SW4-RK-Camera-SetEXIFInformation-00+{_20131225
void iCatch_get_exposure_time(struct msm_sensor_ctrl_t *s_ctrl, exposure_value_cfg *value)
{
	uint32_t exposure_denominator;
	uint16_t exposure_numerator, exposure_denominator_L, exposure_denominator_M, exposure_denominator_H;

	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72b0, &exposure_numerator,	  MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72b1, &exposure_denominator_L, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72b2, &exposure_denominator_M, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72b3, &exposure_denominator_H, MSM_CAMERA_I2C_BYTE_DATA);

	exposure_denominator = (exposure_denominator_H << 16) + (exposure_denominator_M << 8) + exposure_denominator_L;
	value->num = exposure_numerator;
	value->denom = exposure_denominator;

	pr_debug("CFG_GET_EXPOSURE_TIME, Exposure N/(H-M-L): %d/(%d-%d-%d) \n", exposure_numerator, exposure_denominator_L, exposure_denominator_M, exposure_denominator_H);
}
//SW4-RK-Camera-SetEXIFInformation-00+}_20131225

//FihtdcCode@AlanHZChang, add for flashLED status from ISP, 2014/02/19, begin
void iCatch_isp_flash_status(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *value)
{
	uint16_t flash_state;

	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72c3, &flash_state, MSM_CAMERA_I2C_BYTE_DATA);

	pr_debug("isp_main_cam_config(), CFG_GET_FLASH_STATE, flash_state(before) = %d \n", flash_state);
	flash_state = flash_state&0x04;
	pr_debug("isp_main_cam_config(), CFG_GET_FLASH_STATE, flash_state(after) = %d \n", flash_state);

    *value = flash_state&0xFF;
}
//FihtdcCode@AlanHZChang, add for flashLED status from ISP, 2014/02/19, end

//SW4-RK-Camera-SetEXIFInformation-00+{_20131230
void iCatch_get_iso_value(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *value)
{
	uint16_t iso_value_low = 0, iso_value_high = 0;
	uint32_t result = 0;

	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72b7, &iso_value_low, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72b8, &iso_value_high, MSM_CAMERA_I2C_BYTE_DATA);
	result = (iso_value_high << 8) + iso_value_low;
	pr_debug("iso_value_low : %d, iso_value_high : %d, result : %d \n", iso_value_low, iso_value_high, result);
	*value = result;
}
//SW4-RK-Camera-SetEXIFInformation-00+}_20131230

//SW4-RK-Camera-SetEXIF3AInformation-00+{_20140218
void iCatch_get_3a_info(struct msm_sensor_ctrl_t *s_ctrl, threeA_info_get_cfg *value)
{
	uint16_t capExpIdx, capAgcIdx, pvExpIdx, pvAgcIdx, aeEntry, aeLuma, aeStep, aetarget, awbRGain_LSB, awbRGain_MSB, awbBGain_LSB,
	         awbBGain_MSB, awbiCandidateIdx, awbDynamicIQCtIdx, afVertical_step_LSB, afVertical_step_MSB, afMulti_Step_LSB,
	         afMulti_Step_MSB, afFinal_Step_LSB, afFinal_Step_MSB, afFinal_Result_LSB, afFinal_Result_MSB, threeA_Version, threeA_ID;

	pr_debug("%s:%d calling CFG_GET_3A_INFO\n", __func__, __LINE__);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72d8, &capExpIdx,			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72d9, &capAgcIdx,			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72da, &pvExpIdx,			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72db, &pvAgcIdx,			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72dc, &aeEntry,			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72dd, &aeLuma, 			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72de, &aeStep, 			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72df, &aetarget,			 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e0, &awbRGain_LSB,		 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e1, &awbRGain_MSB,		 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e2, &awbBGain_LSB,		 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e3, &awbBGain_MSB,		 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e4, &awbiCandidateIdx,	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e5, &awbDynamicIQCtIdx,	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e6, &afVertical_step_LSB, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e7, &afVertical_step_MSB, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e8, &afMulti_Step_LSB,	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72e9, &afMulti_Step_MSB,	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72ea, &afFinal_Step_LSB,	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72eb, &afFinal_Step_MSB,	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72ec, &afFinal_Result_LSB,  MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72ed, &afFinal_Result_MSB,  MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72ee, &threeA_Version, 	 MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72ef, &threeA_ID,			 MSM_CAMERA_I2C_BYTE_DATA);

	value->capExpIdx = capExpIdx;
	value->capAgcIdx = capAgcIdx;
	value->pvExpIdx = pvExpIdx;
	value->pvAgcIdx = pvAgcIdx;
	value->aeEntry = aeEntry;
	value->aeLuma = aeLuma;
	value->aeStep = aeStep;
	value->aetarget = aetarget;
	value->awbRGain_LSB = awbRGain_LSB;
	value->awbRGain_MSB = awbRGain_MSB;
	value->awbBGain_LSB = awbBGain_LSB;
	value->awbBGain_MSB = awbBGain_MSB;
	value->awbiCandidateIdx = awbiCandidateIdx;
	value->awbDynamicIQCtIdx = awbDynamicIQCtIdx;
	value->afVertical_step_LSB = afVertical_step_LSB;
	value->afVertical_step_MSB = afVertical_step_MSB;
	value->afMulti_Step_LSB = afMulti_Step_LSB;
	value->afMulti_Step_MSB = afMulti_Step_MSB;
	value->afFinal_Step_LSB = afFinal_Step_LSB;
	value->afFinal_Step_MSB = afFinal_Step_MSB;
	value->afFinal_Result_LSB = afFinal_Result_LSB;
	value->afFinal_Result_MSB = afFinal_Result_MSB;
	value->threeA_Version = threeA_Version;
	value->threeA_ID = threeA_ID;
}
//SW4-RK-Camera-SetEXIF3AInformation-00+}_20140218

//SW4-RK-Camera-GetGYRO_GsensorData-00+{_20140116
static int32_t SetCAF_gsensor(struct msm_sensor_ctrl_t *s_ctrl, long datax, long datay, long dataz)
{
	int32_t rc = 0;
        int temp;
        uint16_t rawx, rawy, rawz;

        //printk("\r\n");
        //pr_err("[RK] camera gsensor data x=%ld, y=%ld, z=%ld \n", datax, datay, dataz);
//(-65536 ~ 65535)*10
//(-32.768 ~ 32.767)*1000 >> 8
	temp = (datax/10) >> 9;
	if(temp > 127)
		temp = 127;
	else if(temp < -128)
		temp = -128;
	rawx = (uint16_t)temp;

	temp = (datay/10) >> 9;
	if(temp > 127)
		temp = 127;
	else if(temp < -128)
		temp = -128;
	rawy = (uint16_t)temp;

	temp = (dataz/10) >> 9;
	if(temp > 127)
		temp = 127;
	else if(temp < -128)
		temp = -128;
	rawz = (uint16_t)temp;

	//pr_err("[RK] transform gsensor data x=%u, y=%u, z=%u \r\n", rawx, rawy, rawz);
	pr_err("[RK] transform gsensor data x=%d, y=%d, z=%d", (int16_t)rawx, (int16_t)rawy, (int16_t)rawz);
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x00E0, rawx, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x00E1, rawy, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1300, rawz, MSM_CAMERA_I2C_BYTE_DATA);
        return rc;
}

static int32_t SetCAF_gyro(struct msm_sensor_ctrl_t *s_ctrl, long datax, long datay, long dataz)
{
	int32_t rc = 0;
	int temp;
	uint16_t rawx, rawy, rawz;

	//printk("\r\n");
	//pr_err("[RK] camera gyro data x=%ld, y=%ld, z=%ld \r\n", datax, datay, dataz);

//(0 ~ 2*3.14)*1000 >> 8

	if(datax < 0)
		datax = -datax;
	//temp = datax * (2000/32768) * (628/100)*10000;
	temp = (datax * 628) / 32768;
	rawx = temp >> 7;

	if(datay < 0)
                datay = -datay;
	temp = (datay * 628) / 32768;
	rawy = temp >> 7;

	if(dataz < 0)
                dataz = -dataz;
	temp = (dataz * 628) / 32768;
	rawz = temp >> 7;

	//pr_err("[RK] transform gyro data x=%u, y=%u, z=%u \r\n", rawx, rawy, rawz);
	pr_err("[RK] transform gyro data x=%d, y=%d, z=%d", (int16_t)rawx, (int16_t)rawy, (int16_t)rawz);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1301, rawx, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1302, rawy, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1303, rawz, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

void iCatch_set_sensor_data(struct msm_sensor_ctrl_t *s_ctrl, sensor_data_get_t *value)
{
	SetCAF_gsensor(s_ctrl, value->acce[0], value->acce[1], value->acce[2]);
	SetCAF_gyro(s_ctrl, value->gyro[0], value->gyro[1], value->gyro[2]);
}
//SW4-RK-Camera-GetGYRO_GsensorData-00+}_20140116

//SW4-RL-Camera-WDR-00*{_20140117
void iCatch_set_wdr_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    uint16_t wdr_val=0;

    pr_debug("[RL]%s: E, setting wdr mode to %d \n", __func__,mode);
    pr_debug("[RL]%s (begin) g_cur_wdr = %d \n", __func__,g_cur_wdr);

    if(g_cur_wdr == mode)
    {
        pr_err("%s: cur_wdr_mode = prev_wdr_mode, ignore setting....\n", __func__);
        return;
    }

    msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x729B, &wdr_val, MSM_CAMERA_I2C_BYTE_DATA);
    pr_debug("[RL]%s (read from ISP register) wdr_val = %d \n", __func__,wdr_val);
	if(mode==1)
	{
		wdr_val |= 0x01;
		pr_debug("[RL]%s set wdr on\n",__func__);
    	}
	else
	{
		wdr_val &= ~0x01;
		pr_debug("[RL]%s set wdr off\n",__func__);
	}
    pr_debug("[RL]%s (after setting) wdr_val = %d \n", __func__,wdr_val);

    msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x711B, wdr_val, MSM_CAMERA_I2C_BYTE_DATA);
    g_cur_wdr = mode;
    pr_debug("[RL]%s: X, g_cur_wdr = %d \n",__func__, g_cur_wdr);
}
//SW4-RL-Camera-WDR-00*}_20140117

//SW4-RL-Camera-implementAE/AWBLock-00+{_20140710
void iCatch_set_aec_lock(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    pr_debug("%s aec_lock mode = %d \n", __func__, mode);
    if(g_cur_aec_lock == mode)
    {
        pr_err("%s: ignore setting\n",__func__);
        return;
    }
    if(mode==1)
    {
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x04, MSM_CAMERA_I2C_BYTE_DATA);  //SW4-RK-Camera-CTS+_20141215
    }
    else
    {
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x06, MSM_CAMERA_I2C_BYTE_DATA);  //SW4-RK-Camera-CTS+_20141215
    }
    g_cur_aec_lock = mode;
    g_cur_awb_lock = mode;  //SW4-RK-Camera-CTS+_20141215
}

void iCatch_set_awb_lock(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    pr_debug("%s awb_lock mode = %d \n", __func__, mode);
    if(g_cur_awb_lock == mode)
    {
        pr_err("%s: ignore setting\n",__func__);
        return;
    }
    if(mode==1)
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
    else
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
    g_cur_awb_lock = mode;
}
//SW4-RL-Camera-implementAE/AWBLock-00+}_20140710

//SW4-HL-Camera-ErrorHandling-00*{_20140410
int DumpIspRegister(struct msm_sensor_ctrl_t *s_ctrl, UINT32 addr)
{
	UINT16 value = 0;
	int rc = -1;

	rc = msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, addr, &value, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
	{
		pr_err("\n\n*************** [HL] %s, Failed to read register 0x%x value ***********************\n \n", __func__, addr);
		return rc;
	}
	else
	{
		pr_err("[Dump_Isp_Register] Addr = 0x%x, Value = 0x%x", addr, value);
	}
	return rc;
}

void DumpICatchRegister(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i = 0;
	int rc=0;
	rc=DumpIspRegister(s_ctrl, (0x72f8));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x0024));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x002c));
	if(rc<0)
		goto END;
	for (i = 0; i <= 0x03; i ++)
	{
		rc=DumpIspRegister(s_ctrl, (0x2030 + i));
		if(rc<0)
			goto END;
	}

	for (i = 0; i <= 0x03; i ++)
	{
		rc=DumpIspRegister(s_ctrl, (0x2058 + i));
		if(rc<0)
			goto END;
	}

	rc=DumpIspRegister(s_ctrl, (0x4284));//Rocky_forPreview_20131023
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x4285));//Rocky_forPreview_20131023
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x7000));//Rocky_forPreview_20131023
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x7005));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x7006));
	if(rc<0)
		goto END;
	 rc=DumpIspRegister(s_ctrl, (0x7048));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x7067));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x7070));
	if(rc<0)
		goto END;

	for (i = 0; i <= 0x03; i ++)
	{
		rc=DumpIspRegister(s_ctrl, (0x7072 + i));
		if(rc<0)
			goto END;
	}

	//Rocky_forPreview_{_20131023
	for (i = 0; i <= 0x80; i ++)
	{
		rc=DumpIspRegister(s_ctrl, (0x7200 + i));
		if(rc<0)
			goto END;
	}

	for (i = 0; i <= 0x06; i ++)
	{
		rc=DumpIspRegister(s_ctrl, (0x72c6 + i));//Rocky_forPreview_20131023
		if(rc<0)
			goto END;
	}
	rc=DumpIspRegister(s_ctrl, (0x9080));
	if(rc<0)
		goto END;
	for (i = 0; i <= 0x03; i ++)
	{
		rc=DumpIspRegister(s_ctrl, (0x90cc + i));
		if(rc<0)
			goto END;
	}
	rc=DumpIspRegister(s_ctrl, (0x9400));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x9408));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x9409));
	if(rc<0)
		goto END;
	rc=DumpIspRegister(s_ctrl, (0x7286));	//SW4-RL-dumpForCheckPreviewSize-00+_20140410
	if(rc<0)
		goto END;
END:
	return;
}

int isp_main_cam_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	#ifdef ENABLE_NORMAL_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
	#endif
	uint16_t ois_val=0;		//SW4-RL-Camera-implementOIS-00+_20140430

        af_not_ready = 0;

	pr_debug("%s: pre_res=%d, cur_res=%d\n", __func__, g_pre_res,isp_main_cam_mode);

	if(g_pre_res == isp_main_cam_mode)
	{
		pr_debug("%s: ignore mode change\n",__func__);

		//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+{_20131030
		if (giCatchStreamOff == 1)
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
			giCatchStreamOff = 0;
		}
		//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+}_20131030

		return rc;
	}
	else
	{
		pr_debug("\n\n*************** %s, Mode Changed!!! ***********************\n \n", __func__);
	}

	#ifdef ENABLE_NORMAL_DEBUG
        pr_debug("=======%s [1] START ===== \n",__func__);
        if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)		return -1;
        if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
	#endif

	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	#ifdef ENABLE_NORMAL_DEBUG
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	#endif

	mdelay(10);

//SW4-RL-Camera-implementOIS-00+{_20140430
	pr_debug("%s camera_firmware_name = %s \n", __func__, camera_firmware_name);
	if(!strncmp (camera_firmware_name, "MO7",strlen("MO7"))){
		if (isp_ois_on == 1){
			msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x729C, &ois_val, MSM_CAMERA_I2C_BYTE_DATA);
			pr_debug("%s (read from ISP register) ois_val = %d \n", __func__, ois_val);
			ois_val |= 0x01;
			pr_debug("%s (after setting) ois_val = %d \n", __func__,ois_val);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x711C, ois_val, MSM_CAMERA_I2C_BYTE_DATA);
		}else if(isp_ois_on == 0){
			msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x729C, &ois_val, MSM_CAMERA_I2C_BYTE_DATA);
			pr_debug("%s (read from ISP register) ois_val = %d \n", __func__, ois_val);
			ois_val &= 0xFE;
			pr_debug("%s (after setting) ois_val = %d \n", __func__,ois_val);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x711C, ois_val, MSM_CAMERA_I2C_BYTE_DATA);
		}
	}
//SW4-RL-Camera-implementOIS-00+}_20140430
	restore_isp_main_cam_mode = isp_main_cam_mode;  //SW5-Webber-20150513-Add for restore camera resolution for recover mode.
	switch(isp_main_cam_mode)
	{
		case MSM_SENSOR_RES_FULL_PREVIEW:
		{
			//ChunJengKang modify for full size resolution 20141104
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710F, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x03, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
		}
		break;

		case MSM_SENSOR_RES_FULL:
		{
			//To capture mode
			if(HDR_mode == 1)
			{
				pr_debug("\n\n*************** [HL] %s, HDR_mode == 1 ***********************\n \n", __func__);
				if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7127, 0x17, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
				if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710F, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
				if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			}
			else
			{
				pr_debug("\n\n*************** [HL] %s, HDR_mode == 0 ***********************\n \n", __func__);
				//SW4-RK-Camera-FlashTiming-00+{_20140313
				if(ENABLE_SKIP_FRAMES)//if [sensorName]_lib.c set .sensor_num_frame_skip > 0 , using Burst Caputre (Non-ZSL)
				{
					/* if AE is locked (in iCatch solution means both AE and AWB are locked). We have to unlock it before capture event or it will cause iCatch timeout */
					if(g_cur_aec_lock == 1) {
						msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
						msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
					}
					if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710F, 0x03, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
				}
				else
				{
					if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710F, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
				}
				//SW4-RK-Camera-FlashTiming-00+}_20140313
				if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x03, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			}
			pr_debug("%s: waiting for MSM_SENSOR_RES_FULL Clean 0x72F8 \n", __func__);
			//make sure TAE off for next preview.
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714E, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
		break;

		case MSM_SENSOR_RES_QTR:
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, preview_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1; //fihtdc,derekcwwu, add for tx3
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1; //Set ICatch to preview mode
			pr_debug("%s: waiting for MSM_SENSOR_RES_QTR Clean 0x72F8 \n", __func__);
		}
		break;

		case MSM_SENSOR_RES_HD:
		{
			//SW4-RL-Camera-modify_HD_RecordingResolution-00*_20140423
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, hd_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1; //Set ICatch to preview mode
			pr_debug("%s: waiting for MSM_SENSOR_RES_HD Clean 0x72F8 \n", __func__);
		}
		break;

		//SW4-RL-Camera-addFor_FHD_Recording-00+{_20140423
		case MSM_SENSOR_RES_FHD:
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FHD Clean 0x72F8 \n", __func__);
		}
		break;
		//SW4-RL-Camera-addFor_FHD_Recording-00+}_20140423

		//SW5-Webber-Camera-add_for_HighFrameRecording_90fps_20150423++
		case MSM_SENSOR_RES_HFR90:
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1040x780, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_HFR90 Clean 0x72F8 \n", __func__);
		}
		break;
		//SW5-Webber-Camera-add_for_HighFrameRecording_90fps_20150423--

		default:
		{
			pr_err("%s: Do not support this res=%d\n", __func__, isp_main_cam_mode);
			goto isp_error;
		}
		break;
	}

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*{_20131031
	if (giCatchStreamOff == 1)
	{
		if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
		giCatchStreamOff = 0;
	}
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*}_20131031

	//SW5-Webber-20150528-set FHD and HD to min.24fps due to optical's requirement++++
	if(strncmp (camera_firmware_name, "VNA",strlen("VNA"))) {
		//For PHX and VN2, we set FHD and HD to minimum 24fps
		if ((isp_main_cam_mode==MSM_SENSOR_RES_HD) || (isp_main_cam_mode==MSM_SENSOR_RES_FHD)) {
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7125, 24, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
		else{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7125, 0, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
	}
	//SW5-Webber-20150528-set FHD and HD to min.24fps due to optical's requirement----

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*_20131031
	rc = wait_for_next_frame(s_ctrl);
	if(rc < 0)
	{
		pr_debug("%s: isp_enable_recovery=%d\n", __func__, isp_enable_recovery);
		if(isp_enable_recovery)
		{
			rc = recover_isp(s_ctrl);
			if (rc < 0)
			{
				goto isp_error;
			}
		}
		else
		{
			goto isp_error;
		}
	}

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*_20131031
	if(true == iCatch_first_open)
	{
		rc = wait_for_AE_ready(s_ctrl);
		if(rc < 0)
		{
			goto isp_error;
		}
	}

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*_20131031
	if ((isp_main_cam_mode==MSM_SENSOR_RES_HD) || (isp_main_cam_mode==MSM_SENSOR_RES_QTR))
	{
		//schedule_delayed_work(&CAF_work, 20);
		if(caf_mode)
		{ //CAF Mode can't trigger in isp idle mode.
			//set ROI
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7140, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu,modify for optical RD
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7141, 0xc0, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu,modify for optical RD
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7142, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu,modify for optical RD
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7143, 0x8b, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu,modify for optical RD
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7144, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu,modify for optical RD
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7145, 0x9f, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu,modify for optical RD

			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7105, 0x03, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
	}

	#ifdef ENABLE_NORMAL_DEBUG
	DumpICatchRegister(s_ctrl);
	#endif

	g_pre_res = isp_main_cam_mode;

	return rc;

isp_error:
	DumpICatchRegister(s_ctrl);

	return rc;
}

int isp_main_cam_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_debug("%s, E\n", __func__);

	kobject_uevent(&dev_uevent->kobj,KOBJ_OFFLINE);

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*{_20131031
	if (giCatchStreamOff == 0)
	{
		//SW4-RK-Camera-FlashTiming-00+{_20140313
		if(ENABLE_SKIP_FRAMES && (g_pre_res == MSM_SENSOR_RES_FULL) && (g_cur_ledflash == ISP_FLASH_MODE_AUTO || g_cur_ledflash == ISP_FLASH_MODE_ON))
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7122, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
		//SW4-RK-Camera-FlashTiming-00+}_20140313
		if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("%s, SEND STREAM OFF command to 7002A\n", __func__);
		giCatchStreamOff = 1;
	}
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*}_20131031

	pr_debug("%s, X\n", __func__);

	return 0;
}
//SW4-HL-Camera-ErrorHandling-00*}_20140410

//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00*{_20130311
//Orig -- static int isp_main_cam_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
int isp_main_cam_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00*}_20130311
{
	//struct sensor_cfg_data cdata;
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;//SW4-Rocky-Camera-PortingCamera_20130719_00
	long rc = 0;//SW4-Rocky-Camera-PortingCamera_20130719_00
	int32_t i = 0;
	int32_t setting_lev;
	uint16_t AF_done=0, AF_status=0;

	pr_debug("\n\n******************* [HL] %s +++ *************************\n\n", __func__);

	mutex_lock(s_ctrl->msm_sensor_mutex);

	pr_debug("%s:%d %s cfgtype = %d\n", __func__, __LINE__, s_ctrl->sensordata->sensor_name, cdata->cfgtype);

	switch (cdata->cfgtype)
	{//orig:cdata.cfgtype  //SW4-Rocky-Camera-PortingCamera_20130719_00
		case CFG_GET_SENSOR_INFO:
		{
			pr_debug("\n\n******************* [HL] %s, CFG_GET_SENSOR_INFO *************************\n\n", __func__);

			memcpy(cdata->cfg.sensor_info.sensor_name,
				s_ctrl->sensordata->sensor_name,
				sizeof(cdata->cfg.sensor_info.sensor_name));
			cdata->cfg.sensor_info.session_id =
				s_ctrl->sensordata->sensor_info->session_id;
			for (i = 0; i < SUB_MODULE_MAX; i++)
				cdata->cfg.sensor_info.subdev_id[i] =
					s_ctrl->sensordata->sensor_info->subdev_id[i];
			CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
				cdata->cfg.sensor_info.sensor_name);
			CDBG("%s:%d session id %d\n", __func__, __LINE__,
				cdata->cfg.sensor_info.session_id);
			for (i = 0; i < SUB_MODULE_MAX; i++)
				CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
					cdata->cfg.sensor_info.subdev_id[i]);
			break;
		}

		case CFG_GET_SENSOR_INIT_PARAMS:
		{
			pr_debug("\n\n******************* [HL] %s, CFG_GET_SENSOR_INIT_PARAMS *************************\n\n", __func__);

			cdata->cfg.sensor_init_params.modes_supported =
    				s_ctrl->sensordata->sensor_info->modes_supported;
			cdata->cfg.sensor_init_params.position =
    				s_ctrl->sensordata->sensor_info->position;
			cdata->cfg.sensor_init_params.sensor_mount_angle =
    				s_ctrl->sensordata->sensor_info->sensor_mount_angle;

			CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
				__LINE__,
				cdata->cfg.sensor_init_params.modes_supported,
				cdata->cfg.sensor_init_params.position,
				cdata->cfg.sensor_init_params.sensor_mount_angle);
			break;
		}

		case CFG_SET_SLAVE_INFO:
		{
			struct msm_camera_sensor_slave_info sensor_slave_info;
			struct msm_sensor_power_setting_array *power_setting_array;
			int slave_index = 0;

			pr_debug("\n\n******************* [HL] %s, CFG_SET_SLAVE_INFO *************************\n\n", __func__);

			if (copy_from_user(&sensor_slave_info,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_sensor_slave_info))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			/* Update sensor slave address */
			if (sensor_slave_info.slave_addr) {
				s_ctrl->sensor_i2c_client->cci_client->sid =
					sensor_slave_info.slave_addr >> 1;
			}

			/* Update sensor address type */
			s_ctrl->sensor_i2c_client->addr_type =
				sensor_slave_info.addr_type;

			/* Update power up / down sequence */
			s_ctrl->power_setting_array =
				sensor_slave_info.power_setting_array;
			power_setting_array = &s_ctrl->power_setting_array;
			power_setting_array->power_setting = kzalloc(
				power_setting_array->size *
				sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
			if (!power_setting_array->power_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(power_setting_array->power_setting,
				(void *)
				sensor_slave_info.power_setting_array.power_setting,
				power_setting_array->size *
				sizeof(struct msm_sensor_power_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(power_setting_array->power_setting);
				rc = -EFAULT;
				break;
			}
			s_ctrl->free_power_setting = true;
			CDBG("%s sensor id %x\n", __func__,
				sensor_slave_info.slave_addr);
			CDBG("%s sensor addr type %d\n", __func__,
				sensor_slave_info.addr_type);
			CDBG("%s sensor reg %x\n", __func__,
				sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
			CDBG("%s sensor id %x\n", __func__,
				sensor_slave_info.sensor_id_info.sensor_id);
			for (slave_index = 0; slave_index <
				power_setting_array->size; slave_index++)
			{
				CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
					slave_index,
					power_setting_array->power_setting[slave_index].
					seq_type,
					power_setting_array->power_setting[slave_index].
					seq_val,
					power_setting_array->power_setting[slave_index].
					config_val,
					power_setting_array->power_setting[slave_index].
					delay);
			}
			break;
		}

		case CFG_WRITE_I2C_ARRAY:
		{
			struct msm_camera_i2c_reg_setting conf_array;
			struct msm_camera_i2c_reg_array *reg_setting = NULL;

			pr_debug("\n\n******************* [HL] %s, CFG_WRITE_I2C_ARRAY *************************\n\n", __func__);

			if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
				pr_err("%s:%d failed: invalid state %d\n", __func__,
					__LINE__, s_ctrl->sensor_state);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&conf_array,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			reg_setting = kzalloc(conf_array.size *
				(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
			if (!reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
				conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}

			conf_array.reg_setting = reg_setting;
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &conf_array);
			kfree(reg_setting);
			break;
		}

		case CFG_SLAVE_READ_I2C:
		{
			struct msm_camera_i2c_read_config read_config;
			uint16_t local_data = 0;
			uint16_t orig_slave_addr = 0, read_slave_addr = 0;

			pr_debug("\n\n******************* [HL] %s, CFG_SLAVE_READ_I2C *************************\n\n", __func__);

			if (copy_from_user(&read_config,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_read_config))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			read_slave_addr = read_config.slave_addr;
			CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
			CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
				__func__, read_config.slave_addr,
				read_config.reg_addr, read_config.data_type);
			if (s_ctrl->sensor_i2c_client->cci_client) {
				orig_slave_addr =
					s_ctrl->sensor_i2c_client->cci_client->sid;
				s_ctrl->sensor_i2c_client->cci_client->sid =
					read_slave_addr >> 1;
			} else if (s_ctrl->sensor_i2c_client->client) {
				orig_slave_addr =
					s_ctrl->sensor_i2c_client->client->addr;
				s_ctrl->sensor_i2c_client->client->addr =
					read_slave_addr >> 1;
			} else {
				pr_err("%s: error: no i2c/cci client found.", __func__);
				rc = -EFAULT;
				break;
			}
			CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
					__func__, orig_slave_addr,
					read_slave_addr >> 1);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					read_config.reg_addr,
					&local_data, read_config.data_type);
			if (rc < 0) {
				pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
				break;
			}
			if (copy_to_user((void __user *)read_config.data,
				(void *)&local_data, sizeof(uint16_t))) {
				pr_err("%s:%d copy failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			break;
		}

		case CFG_SLAVE_WRITE_I2C_ARRAY:
		{
			struct msm_camera_i2c_array_write_config write_config;
			struct msm_camera_i2c_reg_array *reg_setting = NULL;
			uint16_t write_slave_addr = 0;
			uint16_t orig_slave_addr = 0;

			pr_debug("\n\n******************* [HL] %s, CFG_SLAVE_WRITE_I2C_ARRAY *************************\n\n", __func__);

			if (copy_from_user(&write_config,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_array_write_config))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			CDBG("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
			CDBG("%s:slave_addr=0x%x, array_size=%d\n", __func__,
				write_config.slave_addr,
				write_config.conf_array.size);
			reg_setting = kzalloc(write_config.conf_array.size *
				(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
			if (!reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(reg_setting,
					(void *)(write_config.conf_array.reg_setting),
					write_config.conf_array.size *
					sizeof(struct msm_camera_i2c_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}
			write_config.conf_array.reg_setting = reg_setting;
			write_slave_addr = write_config.slave_addr;
			if (s_ctrl->sensor_i2c_client->cci_client) {
				orig_slave_addr =
					s_ctrl->sensor_i2c_client->cci_client->sid;
				s_ctrl->sensor_i2c_client->cci_client->sid =
					write_slave_addr >> 1;
			} else if (s_ctrl->sensor_i2c_client->client) {
				orig_slave_addr =
					s_ctrl->sensor_i2c_client->client->addr;
				s_ctrl->sensor_i2c_client->client->addr =
					write_slave_addr >> 1;
			} else {
				pr_err("%s: error: no i2c/cci client found.", __func__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}
			CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
					__func__, orig_slave_addr,
					write_slave_addr >> 1);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
				s_ctrl->sensor_i2c_client, &(write_config.conf_array));
			if (s_ctrl->sensor_i2c_client->cci_client) {
				s_ctrl->sensor_i2c_client->cci_client->sid =
					orig_slave_addr;
			} else if (s_ctrl->sensor_i2c_client->client) {
				s_ctrl->sensor_i2c_client->client->addr =
					orig_slave_addr;
			} else {
				pr_err("%s: error: no i2c/cci client found.", __func__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}
			kfree(reg_setting);
			break;
		}

		case CFG_WRITE_I2C_SEQ_ARRAY:
		{
			struct msm_camera_i2c_seq_reg_setting conf_array;
			struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

			pr_debug("\n\n******************* [HL] %s, CFG_WRITE_I2C_SEQ_ARRAY *************************\n\n", __func__);

			if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
				pr_err("%s:%d failed: invalid state %d\n", __func__,
					__LINE__, s_ctrl->sensor_state);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&conf_array,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_seq_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			reg_setting = kzalloc(conf_array.size *
				(sizeof(struct msm_camera_i2c_seq_reg_array)),
				GFP_KERNEL);
			if (!reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
				conf_array.size *
				sizeof(struct msm_camera_i2c_seq_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}

			conf_array.reg_setting = reg_setting;
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_seq_table(s_ctrl->sensor_i2c_client,
				&conf_array);
			kfree(reg_setting);
			break;
		}

#if 1//orig  //SW4-Rocky-Camera-PortingCamera_20130719_00_{
		case CFG_POWER_UP:
		{
			pr_debug("\n\n******************* [HL] %s, CFG_POWER_UP *************************\n\n", __func__);

				if (s_ctrl->func_tbl->sensor_power_up)
					rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
				else
					rc = -EFAULT;
			break;
		}

		case CFG_POWER_DOWN:
		{
			pr_debug("\n\n******************* [HL] %s, CFG_POWER_DOWN *************************\n\n", __func__);

				if (s_ctrl->func_tbl->sensor_power_down)
					rc = s_ctrl->func_tbl->sensor_power_down(
						s_ctrl);
				else
					rc = -EFAULT;
			break;
		}
#else//msm_sensor.c   //SW4-Rocky-Camera-PortingCamera_20130719_00_}
		case CFG_POWER_UP:
			if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
				pr_err("%s:%d failed: invalid state %d\n", __func__,
					__LINE__, s_ctrl->sensor_state);
				rc = -EFAULT;
				break;
			}
			if (s_ctrl->func_tbl->sensor_power_up) {
				if (s_ctrl->sensordata->misc_regulator)
					msm_sensor_misc_regulator(s_ctrl, 1);

				rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
				if (rc < 0) {
					pr_err("%s:%d failed rc %ld\n", __func__,
						__LINE__, rc);
					break;
				}
				s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
				pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
					s_ctrl->sensor_state);
			} else {
				rc = -EFAULT;
			}
			break;

		case CFG_POWER_DOWN:
			kfree(s_ctrl->stop_setting.reg_setting);
			s_ctrl->stop_setting.reg_setting = NULL;
			if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
				pr_err("%s:%d failed: invalid state %d\n", __func__,
					__LINE__, s_ctrl->sensor_state);
				rc = -EFAULT;
				break;
			}
			if (s_ctrl->func_tbl->sensor_power_down) {
				if (s_ctrl->sensordata->misc_regulator)
					msm_sensor_misc_regulator(s_ctrl, 0);

				rc = s_ctrl->func_tbl->sensor_power_down(
					s_ctrl);
				if (rc < 0) {
					pr_err("%s:%d failed rc %ld\n", __func__,
						__LINE__, rc);
					break;
				}
				s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
				pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
					s_ctrl->sensor_state);
			} else {
				rc = -EFAULT;
			}
			break;
#endif

		case CFG_SET_INIT_SETTING:
		{
			pr_debug("\n\n******************* [HL] %s, CFG_SET_INIT_SETTING *************************\n\n", __func__);

			if (s_ctrl->func_tbl->sensor_setting == NULL)
			{
				//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_err("%s:%d calling sensor_setting is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_setting(s_ctrl, MSM_SENSOR_REG_INIT, 0);
			break;
		}

		case CFG_SET_STOP_STREAM_SETTING:
		{
			struct msm_camera_i2c_reg_setting *stop_setting =
				&s_ctrl->stop_setting;
			struct msm_camera_i2c_reg_array *reg_setting = NULL;

			pr_debug("\n\n******************* [HL] %s, CFG_SET_STOP_STREAM_SETTING *************************\n\n", __func__);

			if (copy_from_user(stop_setting,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->stop_setting_valid = 1;

			reg_setting = stop_setting->reg_setting;
			stop_setting->reg_setting = kzalloc(stop_setting->size *
				(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
			if (!stop_setting->reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(stop_setting->reg_setting,
				(void *)reg_setting,
				stop_setting->size *
				sizeof(struct msm_camera_i2c_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(stop_setting->reg_setting);
				stop_setting->reg_setting = NULL;
				stop_setting->size = 0;
				rc = -EFAULT;
				break;
			}
			break;
		}

		case CFG_SET_RESOLUTION:
		{
			int res = 0;

			pr_debug("\n\n******************* [HL] %s, CFG_SET_RESOLUTION *************************\n\n", __func__);

			if (copy_from_user(&res, (void *)cdata->cfg.setting, sizeof(int)))
			{
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (s_ctrl->func_tbl->sensor_setting == NULL)
			{
				//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_err("%s:%d calling sensor_setting is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_setting(s_ctrl, MSM_SENSOR_UPDATE_PERIODIC, res);
			break;
		}

		case CFG_SET_START_STREAM:
		{
			pr_debug("\n\n******************* [HL] %s, CFG_SET_START_STREAM *************************\n\n", __func__);

			if (s_ctrl->func_tbl->sensor_start_stream == NULL)
			{
				//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_err("%s:%d calling sensor_start_stream is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			pr_debug("[RK]%s : X!!\n", __func__);
			break;
		}

		case CFG_SET_STOP_STREAM:
		{
			pr_debug("[RK]%s : CFG_SET_STOP_STREAM!!\n", __func__);

			if (s_ctrl->func_tbl->sensor_stop_stream == NULL)
			{
				//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_err("%s:%d calling sensor_stop_stream is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			pr_debug("[RK]%s : X!!\n", __func__);
			break;
		}

		case CFG_SET_EFFECT:
			pr_debug("%s:%d calling CFG_SET_EFFECT\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_effect_mode == NULL) {
				//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_err("%s:%d calling sensor_set_isp_effect_mode is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting, sizeof(int32_t)))
			{
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_EFFECT mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_effect_mode(s_ctrl, setting_lev);
			break;

		case CFG_SET_BESTSHOT_MODE://Orig : CFG_SET_ISP_SCENE_MODE:
			pr_debug("%s:%d calling CFG_SET_BESTSHOT_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_scene_mode == NULL) {
				pr_err("%s:%d calling sensor_set_isp_scene_mode is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting, sizeof(int32_t)))
			{
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
			break;
			}
			pr_debug("CFG_SET_BESTSHOT_MODE mode(%d)\n",setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_scene_mode(s_ctrl, setting_lev);
			break;

		case CFG_SET_AUTOFOCUS://VKY:CFG_ISP_AF_START:
			pr_debug("%s:%d CFG_SET_AUTOFOCUS case E: \n", __func__, __LINE__);

			if (s_ctrl->func_tbl->sensor_isp_af_start == NULL) {
				//rc = -EFAULT;	//SW4-RK-Camera-NotReturnFailIfNotSupport-00+_20131119
				pr_debug("%s:%d sensor_isp_af_start is NULL\n", __func__, __LINE__);	//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_debug("%s:%d CFG_SET_AUTOFOCUS case X: \n", __func__, __LINE__);
				break;
			}
			pr_debug("CFG_SET_AUTOFOCUS afParam(%d,%d,%d,%d)\n", focusROI.coordinate_x, focusROI.coordinate_y, focusROI.rectangle_h, focusROI.rectangle_w);
			s_ctrl->func_tbl->sensor_isp_af_start(
				s_ctrl, 1,
				isROIset ? ISP_FOCUS_MODE_AUTO : afMode,
				focusROI.coordinate_x,
				focusROI.coordinate_y,
				focusROI.rectangle_h,
				focusROI.rectangle_w);

				isROIset = false;

				pr_debug("[RK] %s FOCUS_MODE %d : \n", __func__, isROIset ? ISP_FOCUS_MODE_AUTO : afMode);

				pr_debug("%s:%d CFG_ISP_AF_START case X: \n", __func__, __LINE__);

			break;
#if 1
			case CFG_SET_AUTOFOCUS_MODE:
				pr_debug("%s:%d CFG_SET_AUTOFOCUS_MODE case E: \n", __func__, __LINE__);

				if (s_ctrl->func_tbl->sensor_set_isp_af_mode == NULL) {
					//rc = -EFAULT;	//SW4-RK-Camera-NotReturnFailIfNotSupport-00+_20131119
					pr_debug("%s:%d sensor_set_isp_af_mode is NULL\n", __func__, __LINE__);
					break;
		}
				if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
					sizeof(int32_t))) {
					pr_err("%s:%d failed\n", __func__, __LINE__);
					rc = -EFAULT;
					break;
				}
				pr_debug("CFG_SET_AUTOFOCUS_MODE mode(%d)\n", setting_lev);
				//SW4-RL-implementFaceTracking-00+{_20140919
				if(setting_lev == ISP_FOCUS_MODE_AUTO && enableFaceTracking){
					pr_debug("%s:%d set mode as FACE_TRACKING \n", __func__, __LINE__);
					s_ctrl->func_tbl->sensor_set_isp_af_mode(s_ctrl, ISP_FOCUS_MODE_FACE_TRACKING);
				}else{
					pr_debug("%s:%d set mode as %d \n", __func__, __LINE__, setting_lev);
					s_ctrl->func_tbl->sensor_set_isp_af_mode(s_ctrl, setting_lev);
				}
				//SW4-RL-implementFaceTracking-00+}_20140919
				pr_debug("%s:%d CFG_SET_AUTOFOCUS_MODE case X: \n", __func__, __LINE__);
			break;
#else
		case CFG_SET_AUTOFOCUS_MODE:
			pr_err("%s:%d CFG_SET_AUTOFOCUS_MODE case E: \n", __func__, __LINE__);

			if (s_ctrl->func_tbl->sensor_set_isp_af_mode == NULL) {
				rc = -EFAULT;
				pr_err("%s:%d sensor_set_isp_af_mode is NULL\n", __func__, __LINE__);
				break;
	}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_AUTOFOCUS_MODE mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_af_mode(
				s_ctrl, 1,
				setting_lev,
				cdata->cfg.focus.coordinate_x,
				cdata->cfg.focus.coordinate_y,
				cdata->cfg.focus.rectangle_h,
				cdata->cfg.focus.rectangle_w);
			pr_err("%s:%d CFG_SET_AUTOFOCUS_MODE case X: \n", __func__, __LINE__);
		break;
#endif
			//Rocky_{_20131009
			#if 1
			case CFG_SET_FOCUS_ROI:
				pr_debug("%s:%d CFG_SET_FOCUS_ROI case E: \n", __func__, __LINE__);

				if (s_ctrl->func_tbl->sensor_set_isp_af_roi == NULL) {
					//rc = -EFAULT;	//SW4-RK-Camera-NotReturnFailIfNotSupport-00+_20131119
					pr_debug("%s:%d sensor_set_isp_af_roi is NULL\n", __func__, __LINE__);	//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
					break;
				}
				s_ctrl->func_tbl->sensor_set_isp_af_roi(s_ctrl, cdata->cfg.focus.af_enable, cdata->cfg.focus.coordinate_x, cdata->cfg.focus.coordinate_y, cdata->cfg.focus.rectangle_h, cdata->cfg.focus.rectangle_w, cdata->cfg.focus.isFaceDetectMode);	//SW4-RL-implementFaceTracking-00*_20140919
				pr_debug("%s:%d isFaceDetectMode = %d \n", __func__, __LINE__, cdata->cfg.focus.isFaceDetectMode);	//SW4-RL-implementFaceTracking-00+_20140919
				pr_debug("%s:%d CFG_SET_FOCUS_ROI case X: \n", __func__, __LINE__);

				break;
			#endif
			//Rocky_}-20131009

			//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+{_20140328
			case CFG_SET_METERING_ROI:
				//pr_debug("%s:%d CFG_SET_METERING_ROI case E: \n", __func__, __LINE__);
				pr_debug("%s:%d CFG_SET_METERING_ROI case E: \n", __func__, __LINE__);
				if (s_ctrl->func_tbl->sensor_set_isp_aec_roi == NULL) {
					//rc = -EFAULT;	//SW4-RK-Camera-NotReturnFailIfNotSupport-00+_20131119
					pr_debug("%s:%d sensor_set_isp_aec_roi is NULL\n", __func__, __LINE__);
					break;
				}
				s_ctrl->func_tbl->sensor_set_isp_aec_roi(s_ctrl, cdata->cfg.focus.coordinate_x, cdata->cfg.focus.coordinate_y, cdata->cfg.focus.rectangle_h, cdata->cfg.focus.rectangle_w);
					pr_debug("%s:%d CFG_SET_METERING_ROI case X: \n", __func__, __LINE__);

				break;
			//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+}_20140328

			//SW5-Webber-Camera-ImplementCancelAF-20130331-begin
			case CFG_CANCEL_AUTOFOCUS://vky:CFG_SET_CANCEL_AF:
				pr_debug("%s:%d calling CFG_SET_CANCEL_AF\n", __func__, __LINE__);
				if (s_ctrl->func_tbl->sensor_set_cancel_af ==  NULL)
				{
					//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
					pr_debug("%s:%d calling sensor_set_cancel_af is NULL\n", __func__, __LINE__);
					break;
				}
				s_ctrl->func_tbl->sensor_set_cancel_af(s_ctrl);
			break;
			//SW5-Webber-Camera-ImplementCancelAF-20130331-end

//****************************************

		//SW4-L1-HL-Camera-ImplementExposureCompensation-00+{_20130227
		case CFG_SET_EXPOSURE_COMPENSATION:
			pr_debug("%s:%d calling CFG_SET_EXPOSURE_COMPENSATION\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_exposure_compensation ==  NULL) {
				pr_err("%s:%d calling sensor_set_isp_exposure_compensation is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_EXPOSURE_COMPENSATION mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_exposure_compensation(s_ctrl, setting_lev);
			break;
		//SW4-L1-HL-Camera-ImplementExposureCompensation-00+}_20130227

		//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227
		case CFG_SET_EXPOSURE_MODE:
			pr_debug("%s:%d calling CFG_SET_EXPOSURE_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_aec_mode ==  NULL) {
				pr_err("%s:%d calling sensor_set_isp_aec_mode is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_EXPOSURE_MODE mode(%d)\n",setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_aec_mode(s_ctrl, setting_lev);
			break;
		//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227
		//20130227@Rocky add iso/wb function[START]
		case CFG_SET_ISO:
			pr_debug("%s:%d calling CFG_SET_ISO\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_iso == NULL) {
				pr_err("%s:%d calling sensor_set_iso is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_ISO mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_iso(s_ctrl, setting_lev);
			break;
		case CFG_SET_WHITE_BALANCE:
			pr_debug("%s:%d calling CFG_SET_WB\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_wb == NULL) {
				pr_err("%s:%d calling sensor_set_isp_wb is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			pr_debug("CFG_SET_WB mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_wb(s_ctrl, setting_lev);
			break;
		//20130227@Rocky add iso/wb function[END]

		//SW4-L1-HL-Camera-ImplementSaturation-00+{_20130305
		case CFG_SET_SATURATION:
			pr_debug("%s:%d calling CFG_SET_SATURATION\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_saturation ==  NULL)
			{
				pr_err("%s:%d calling sensor_set_isp_saturation is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
			break;
		}
			pr_debug("CFG_SET_SATURATION mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_saturation(s_ctrl, setting_lev);
			break;
		//SW4-L1-HL-Camera-ImplementSaturation-00+}_20130305
		//SW4-L1-HL-Camera-ImplementSharpness-00+{_20130305
		case CFG_SET_SHARPNESS:
			pr_debug("%s:%d calling CFG_SET_SHARPNESS\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_sharpness ==  NULL)
			{
				pr_err("%s:%d calling sensor_set_isp_sharpness is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
			break;
		}
			pr_debug("CFG_SET_SHARPNESS mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_sharpness(s_ctrl, setting_lev);
			break;
		//SW4-L1-HL-Camera-ImplementSharpness-00+}_20130305
		//SW4-L1-HL-Camera-ImplementContrast-00+{_20130305
		case CFG_SET_CONTRAST:
			pr_debug("%s:%d calling CFG_SET_CONTRAST\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_contrast ==  NULL)
			{
				pr_err("%s:%d calling sensor_set_isp_contrast is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
			break;
		}
			pr_debug("CFG_SET_CONTRAST mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_contrast(s_ctrl, setting_lev);
			break;
		//SW4-L1-HL-Camera-ImplementContrast-00+}_20130305

		//SW4-L1-HL-Camera-ImplementAntiBanding-00+{_20130307
		case CFG_SET_ANTIBANDING:
			pr_debug("%s:%d calling CFG_SET_ANTIBANDING\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_antibanding ==  NULL)
			{
				pr_err("%s:%d calling sensor_set_isp_antibanding is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_ANTIBANDING value(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_antibanding(s_ctrl, setting_lev);
			break;
		//SW4-L1-HL-Camera-ImplementAntiBanding-00+{_20130307
        case CFG_GET_FOCUS_STATUS:
            if(cdata->cfg.focus.af_get_choice==0)
            {
                msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72a0, &AF_done, MSM_CAMERA_I2C_BYTE_DATA);
                if(AF_done == 0x01) //in busy
                    cdata->cfg.focus.af_done=0;
                else if(AF_done == 0x00)//in idle
                    cdata->cfg.focus.af_done=1;
                else if(AF_done == 0x10) //CAF idle
                    cdata->cfg.focus.af_done=2;
                else if(AF_done == 0x11) //CAF busy
                    cdata->cfg.focus.af_done=3;
                if((AF_done == 0x01)||(AF_done == 0x00))
                    af_not_ready=0;
                pr_debug("%s, cdata.cfg.focus.af_done : %d \n", __func__, cdata->cfg.focus.af_done);
            }
            if(cdata->cfg.focus.af_get_choice==1)
            {
                msm_camera_cci_i2c_read(isp_main_cam_s_ctrl.sensor_i2c_client, 0x72a1, &AF_status, MSM_CAMERA_I2C_BYTE_DATA);
                if(AF_status == 0x01) //focus failed
                    cdata->cfg.focus.af_status=1;
                else//focus success
                    cdata->cfg.focus.af_status=0;
                pr_debug("%s, cdata.cfg.focus.af_status : %d \n", __func__, cdata->cfg.focus.af_status);
            }
            //if(copy_to_user((void *)argp, &cdata, sizeof(struct sensorb_cfg_data)))
            //    rc = -EFAULT;
            break;
        //SW5-Webber-Camera-ImplementLedFlash-20130313-start
		case CFG_SET_FLASH_MODE:
			pr_debug("%s:%d calling CFG_SET_FLASH_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_led_flash_mode == NULL) {
				//rc = -EFAULT;	//SW4-RK-Camera-NotReturnFailIfNotSupport-00+_20131119
				pr_debug("%s:%d calling sensor_set_isp_led_flash_mode is NULL\n", __func__, __LINE__);
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			pr_debug("CFG_SET_FLASH_MODE mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_led_flash_mode(s_ctrl, setting_lev);
			break;
		//SW5-Webber-Camera-ImplementLedFlash-20130313-start

        case CFG_GET_ISO_VALUE:
		 pr_debug("%s:%d calling CFG_GET_ISO_VALUE\n", __func__, __LINE__);

		  if (s_ctrl->func_tbl->sensor_get_isp_iso_value == NULL) {
			  pr_err("%s:%d calling sensor_get_isp_iso_value is NULL\n", __func__, __LINE__);
			  rc = -EFAULT;
			  break;
		  }
		  s_ctrl->func_tbl->sensor_get_isp_iso_value(s_ctrl, (uint32_t *)cdata->cfg.setting);
		  pr_debug("%s:%d num = %d \n", __func__, __LINE__, *(int *)(cdata->cfg.setting));
		break;

//SW4-RK-Camera-SetEXIFInformation-00+{_20131225
		case CFG_GET_EXPOSURE_TIME:
		  pr_debug("%s:%d calling CFG_GET_EXPOSURE_TIME\n", __func__, __LINE__);

		  if (s_ctrl->func_tbl->sensor_get_isp_exposure_time == NULL)
		  {
			  pr_err("%s:%d calling sensor_get_isp_exposure_time is NULL\n", __func__, __LINE__);
			  rc = -EFAULT;
			  break;
		  }
		  s_ctrl->func_tbl->sensor_get_isp_exposure_time(s_ctrl, (exposure_value_cfg *)cdata->cfg.setting);
		  pr_debug("%s:%d num = %d, denom = %d \n", __func__, __LINE__, ((exposure_value_cfg *)cdata->cfg.setting)->num, ((exposure_value_cfg *)cdata->cfg.setting)->denom);
		  break;
//SW4-RK-Camera-SetEXIFInformation-00+}_20131225

//SW4-RK-Camera-GetGYRO_GsensorData-00+{_20140116
		case CFG_SET_SENSOR_DATA:
		  pr_debug("%s:%d calling CFG_SET_SENSOR_DATA\n", __func__, __LINE__);

		  if (s_ctrl->func_tbl->sensor_set_isp_sensor_data == NULL)
		  {
			//SW4-HL-Camera-EnhanceKernelLog-01-_20140409
			pr_debug("%s:%d calling sensor_set_isp_sensor_data is NULL\n", __func__, __LINE__);
			break;
		  }
		  s_ctrl->func_tbl->sensor_set_isp_sensor_data(s_ctrl, (sensor_data_get_t *)cdata->cfg.setting);
		  break;
  //SW4-RK-Camera-GetGYRO_GsensorData-00+}_20140116

//SW4-RK-Camera-SetEXIF3AInformation-00+{_20140218
		case CFG_GET_3A_INFO:
			pr_debug("%s:%d calling CFG_GET_EXPOSURE_TIME\n", __func__, __LINE__);

			if (s_ctrl->func_tbl->sensor_get_isp_3a_info == NULL)
			{
				pr_err("%s:%d calling sensor_get_isp_3a_info is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_get_isp_3a_info(s_ctrl, (threeA_info_get_cfg *)cdata->cfg.setting);
			//pr_err("%s:%d num = %d, denom = %d \n", __func__, __LINE__, ((exposure_value_cfg *)cdata->cfg.setting)->num, ((exposure_value_cfg *)cdata->cfg.setting)->denom);
			break;
//SW4-RK-Camera-SetEXIF3AInformation-00+{_20140218

#if 1
//SW4-RL-Camera-implementHDR-00+{_20140125
		case CFG_SET_HDR:
			pr_debug("%s:%d calling CFG_SET_ISP_HDR_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_HDR_mode == NULL) {
				//SW4-HL-Camera-EnhanceKernelLog-01-_20140409
				//rc = -EFAULT;
				//SW4-HL-Camera-EnhanceKernelLog-01+_20140409
				pr_debug("%s:%d calling sensor_set_isp_HDR_mode is NULL\n", __func__, __LINE__);
				break;
			}
			pr_debug("%s:%d CFG_SET_ISP_HDR_MODE mode(%d)\n", __func__, __LINE__, cdata->cfg.hdr_mode);
			s_ctrl->func_tbl->sensor_set_isp_HDR_mode(s_ctrl, cdata->cfg.hdr_mode);
			break;
//SW4-RL-Camera-implementHDR-00+}_20140125

//SW4-RL-Camera-WDR-00*{_20140117
		case CFG_SET_WDR:
			pr_debug("%s:%d calling CFG_SET_ISP_WDR_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_wdr == NULL) {
				pr_err("%s:%d sensor_set_isp_wdr = NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_ISP_WDR_MODE = %d\n",cdata->cfg.wdr_mode);
			s_ctrl->func_tbl->sensor_set_isp_wdr(s_ctrl, cdata->cfg.wdr_mode);
		break;
//SW4-RL-Camera-WDR-00*}_20140117
#endif
//RL*}_20140110

		case CFG_GET_FLASH_STATE:
			pr_debug("%s:%d calling CFG_GET_FLASH_STATE\n", __func__, __LINE__);

			if (s_ctrl->func_tbl->sensor_get_isp_flash_status == NULL)
			{
				//SW4-HL-Camera-EnhanceKernelLog-01*_20140409
				pr_debug("%s:%d calling sensor_get_isp_flash_status is NULL\n", __func__, __LINE__);
				break;
			}
			s_ctrl->func_tbl->sensor_get_isp_flash_status(s_ctrl, (uint32_t *)cdata->cfg.setting);
			pr_debug("%s:%d num = %d \n", __func__, __LINE__, *(int *)(cdata->cfg.setting));
		break;

//SW4-RL-Camera-implementAELock-00+{_20140710
		case CFG_SET_AEC_LOCK:
			pr_debug("%s:%d calling CFG_SET_AEC_LOCK\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_aec_lock == NULL) {
				pr_err("%s:%d sensor_set_isp_aec_lock is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_AEC_LOCK mode(%d)\n",cdata->cfg.aec_mode);
			s_ctrl->func_tbl->sensor_set_isp_aec_lock(s_ctrl, cdata->cfg.aec_mode);
		break;

		case CFG_SET_AWB_LOCK:
			pr_debug("%s:%d calling CFG_SET_AWB_LOCK\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_awb_lock == NULL) {
				pr_err("%s:%d sensor_set_isp_awb_lock is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_debug("CFG_SET_AWB_LOCK mode(%d)\n",cdata->cfg.awb_mode);
			s_ctrl->func_tbl->sensor_set_isp_awb_lock(s_ctrl, cdata->cfg.awb_mode);
		break;
//SW4-RL-Camera-implementAELock-00+}_20140710

		default:
		{
			pr_debug("\n\n******************* [HL] %s, default *************************\n\n", __func__);
			break;
		}
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	pr_debug("\n\n******************* [HL] %s ---, rc = %ld *************************\n\n", __func__, rc);

	return rc;
}

static int32_t isp_main_cam_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t isp_main_cam_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

int isp_main_cam_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct msm_camera_power_ctrl_t *power_info = &data->power_info;
	struct msm_camera_gpio_conf *gpio_conf = power_info->gpio_conf;
	s_ctrl->stop_setting_valid = 0;

	pr_debug("%s: E\n", __func__);

	//fihtdc, derekcwwu, move to this for correct sequence
	#if ENABLE_ISP_INTERRUPT
	isp_deinit_interrupt();
	#endif
	//fihtdc, derekcwwu, move to this for correct sequence

	kobject_uevent(&dev_uevent->kobj,KOBJ_OFFLINE);
	//SW5-Anvoi-camera firmware sometime upgrade fail. 20130613,+++
	//Retrun error to avoid camera sequence mix if process firmware upgrade running . camera will back to normal when upgrade task done.
	if(fw_status == FW_STATUS_UPGRADING)
		return -1;
	//SW5-Anvoi-camera firmware sometime upgrade fail. 20130613,---

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (main_cam_is_power_on == 0)
		return 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402

	pr_debug("[RK]%s:%d i2c_util ++\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}
	pr_debug("[RK]%s:%d i2c_util --\n", __func__, __LINE__);

	pr_debug("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	for (index = (power_setting_array->size - 1); index >= 0; index--) {
		pr_debug("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		pr_debug("%s seq_type = %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(power_info->dev,
				&power_info->clk_info[0],
				(struct clk **)&power_setting->data[0],
				power_info->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= ISP_MAIN_SENSOR_GPIO_MAX ||//SW4-Rocky-Camera-PortingCamera_20130816_00
				!gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					ISP_MAIN_SENSOR_GPIO_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				continue;
			}
			gpio_set_value_cansleep(
				gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {//SW4-Rocky-Camera-PortingCamera_20130816_00
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				continue;
			}
			msm_camera_config_single_vreg(power_info->dev,
				&power_info->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (power_info->i2c_conf && power_info->i2c_conf->use_i2c_mux)
				isp_main_cam_disable_i2c_mux(power_info->i2c_conf);
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			pr_debug("[RK]%s mdelay(%d)\n", __func__, power_setting->delay);
			mdelay(power_setting->delay);
		}
	}

	if(g_caf_sensor_enable){
		//cancel_delayed_work_sync(&CAF_work);
		//isp_af_close();
		g_caf_sensor_enable=0;
	}

	msm_camera_request_gpio_table(
	gpio_conf->cam_gpio_req_tbl,
	gpio_conf->cam_gpio_req_tbl_size, 0);

	//kfree(s_ctrl->reg_ptr);
	//s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
	g_pre_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	main_cam_is_power_on = 0;

	pr_debug("%s: X\n", __func__);

	return 0;
}


int  isp_main_cam_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, index = 0;
	//SW4-HL-Camera-EnhanceCameraStability-MCS-1213-00-_20140217
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct msm_camera_power_ctrl_t *power_info = &data->power_info;
	struct msm_camera_gpio_conf *gpio_conf = power_info->gpio_conf;
	unsigned int intr_gpio=0;//fihtdc,derekcww,add for front cam when front cam can focus
	//struct device *dev = NULL;
	//int32_t rc = 0;

	int testINT = 0;//SW4-Rocky-Camera-PortingCamera_20130816_00

	s_ctrl->stop_setting_valid = 0;

	pr_debug("\n\n[RK]%s E %d\n\n", __func__, index);

	//SW5-Anvoi-camera firmware sometime upgrade fail. 20130613,+++
	//Retrun error to avoid camera sequence mix if process firmware upgrade running . camera will back to normal when upgrade task done.
	if(fw_status == FW_STATUS_UPGRADING)
	//HL+_20130910
	{
		pr_debug("\n\n******************* [HL] %s, fw_status == FW_STATUS_UPGRADING *************************\n\n", __func__);
		return -1;
	//HL+_20130910
	}
	//SW5-Anvoi-camera firmware sometime upgrade fail. 20130613,---

	if(isp_is_power_on == 1)
	//HL+_20130910
	{
		pr_debug("\n\n******************* [HL] %s, isp_is_power_on == 1 *************************\n\n", __func__);
		return 0;
	//HL+_20130910
	}

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (main_cam_is_power_on == 1)
	//HL+_20130910
	{
		pr_debug("\n\n******************* [HL] %s, main_cam_is_power_on == 1 *************************\n\n", __func__);
		return 0;
	//HL+_20130910
	}
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402
	power_setting_array = &s_ctrl->power_setting_array;

	if (gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			gpio_conf->cam_gpiomux_conf_tbl,
			gpio_conf->cam_gpiomux_conf_tbl_size);
	}
	//HL+_20130910
	else
	{
		pr_debug("\n\n******************* [HL] %s, data->gpio_conf->cam_gpiomux_conf_tbl == NULL, no msm_gpiomux_install() *************************\n\n", __func__);
	}
	//HL+_20130910

/*fihtdc,derekcwwu, fix gpio free
	//Fixme: for mo7, need to do this
if(!strncmp(camera_firmware_name,"MO7",strlen("MO7"))){//fihtdc,derekcwwu, add for fix vna warning log
	msm_camera_request_gpio_table(
	gpio_conf->cam_gpio_req_tbl,
	gpio_conf->cam_gpio_req_tbl_size, 0);
}//fihtdc,derekcwwu, add for fix vna warning log
*/
	rc = msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

/*
	if (s_ctrl->sensor_device_type == MSM_SENSOR_PLATFORM_DEVICE)
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;
	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	if (!s_ctrl->reg_ptr) {
		pr_err("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}
*/
	//pr_err("%s: platform device name: %s\n", __func__, s_ctrl->pdev->name);
	//pr_err("%s: i2c device name: %s\n", __func__, s_ctrl->msm_sensor_client->name);
	//pr_err("%s: platform device name: %s\n", __func__, s_ctrl->pdev->name);

	for (index = 0; index < power_setting_array->size; index++)
	{
		power_setting = &power_setting_array->power_setting[index];

//Rocky add for test{
#if 1
		pr_debug("[RK]%s index %d\n", __func__, index);
		if(power_setting->seq_type == SENSOR_CLK)
		{
			pr_debug("[RK]%s type %d(SENSOR_CLK)\n", __func__, power_setting->seq_type);
		}
		else if(power_setting->seq_type == SENSOR_GPIO)
		{
			pr_debug("[RK]%s type %d(SENSOR_GPIO)\n", __func__, power_setting->seq_type);
		}
		else if(power_setting->seq_type == SENSOR_VREG)
		{
			pr_debug("[RK]%s type %d(SENSOR_VREG)\n", __func__, power_setting->seq_type);
		}
		else if(power_setting->seq_type == SENSOR_I2C_MUX)
		{
			pr_debug("[RK]%s type %d(SENSOR_I2C_MUX)\n", __func__, power_setting->seq_type);
		}
		else
		{
			pr_debug("[RK]%s type %d\n", __func__, power_setting->seq_val);
		}
#endif
//Rocky add for test}

		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			if (power_setting->seq_val >= power_info->clk_info_size) {
				pr_err("%s clk index %d >= max %d\n", __func__,
					power_setting->seq_val,
					power_info->clk_info_size);
				goto power_up_failed;
			}
//Rocky add for test{
#if 1
			if(power_setting->seq_val == SENSOR_CAM_MCLK)
			{
				pr_debug("[RK]%s seq_val %d(SENSOR_CAM_MCLK)\n", __func__, power_setting->seq_val);
			}
			else if(power_setting->seq_val == SENSOR_CAM_CLK)
			{
				pr_debug("[RK]%s seq_val %d(SENSOR_CAM_CLK)\n", __func__, power_setting->seq_val);
			}
			else if(power_setting->seq_val == SENSOR_CAM_CLK_MAX)
			{
				pr_debug("[RK]%s seq_val %d(SENSOR_CAM_CLK_MAX)\n", __func__, power_setting->seq_val);
			}
			else
			{
				pr_debug("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);
			}

			pr_debug("[RK]%s config_val %ld\n", __func__, power_setting->config_val);
#endif
//Rocky add for test}

			if (power_setting->config_val)
				power_info->clk_info[power_setting->seq_val].clk_rate = power_setting->config_val;

			//SW4-Rocky-Camera-PortingCamera_20130816_00_{
			for(testINT = 0; testINT<power_info->clk_info_size; testINT++)
			{
				pr_debug("[RK]%s [%d] name:%s, rate:%ld\n", __func__, testINT, power_info->clk_info[testINT].clk_name, power_info->clk_info[testINT].clk_rate);
			}
			//SW4-Rocky-Camera-PortingCamera_20130816_00_}

			rc = msm_cam_clk_enable(power_info->dev,
				&power_info->clk_info[0],
				(struct clk **)&power_setting->data[0],
				power_info->clk_info_size,
				1);
			if (rc < 0) {
				pr_err("%s: clk enable failed\n",
					__func__);
				goto power_up_failed;
			}
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= ISP_MAIN_SENSOR_GPIO_MAX ||
				!gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					ISP_MAIN_SENSOR_GPIO_MAX);
				goto power_up_failed;
			}
//Rocky add for test{
#if 1
				if(power_setting->seq_val == ISP_SUSPEND)
				{
					pr_debug("[RK]%s seq_val %d(ISP_SUSPEND)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == ISP_RESET_N)
				{
					pr_debug("[RK]%s seq_val %d(ISP_RESET_N)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == ISP_INTR)
				{
					pr_debug("[RK]%s seq_val %d(ISP_INTR)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == ISP_AVDD_EN)
				{
					pr_debug("[RK]%s seq_val %d(ISP_AVDD_EN)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == ISP_DVDD_EN)
				{
					pr_debug("[RK]%s seq_val %d(ISP_DVDD_EN)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == ISP_VCORE_EN)
				{
					pr_debug("[RK]%s seq_val %d(ISP_VCORE_EN)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == CAM_8M_1P8_EN)
				{
					pr_debug("[RK]%s seq_val %d(CAM_8M_1P8_EN)\n", __func__, power_setting->seq_val);
				}
				else if(power_setting->seq_val == CAM_8M_2P8_EN)
				{
					pr_debug("[RK]%s seq_val %d(CAM_8M_2P8_EN)\n", __func__, power_setting->seq_val);
				}
				else
				{
					pr_debug("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);
				}

				pr_debug("[RK]%s:%d gpio set val %d\n", __func__, __LINE__, gpio_conf->gpio_num_info->gpio_num[power_setting->seq_val]);

				if(power_setting->config_val == GPIO_OUT_LOW)
				{
					pr_debug("[RK]%s config_val %ld(GPIO_OUT_LOW)\n", __func__, power_setting->config_val);
				}
				else if(power_setting->config_val == GPIO_OUT_HIGH)
				{
					pr_debug("[RK]%s config_val %ld(GPIO_OUT_HIGH)\n", __func__, power_setting->config_val);
				}
				else
				{
					pr_debug("[RK]%s config_val %ld\n", __func__, power_setting->config_val);
				}
#endif
//Rocky add for test}

				pr_debug("%s:%d gpio set val %d\n", __func__, __LINE__,
					gpio_conf->gpio_num_info->gpio_num
					[power_setting->seq_val]);
				gpio_set_value_cansleep(
					gpio_conf->gpio_num_info->gpio_num
					[power_setting->seq_val],
					power_setting->config_val);
				break;
			case SENSOR_VREG:
				if (power_setting->seq_val >= CAM_VREG_MAX) {
					pr_err("%s vreg index %d >= max %d\n", __func__,
						power_setting->seq_val,
						CAM_VREG_MAX);
					goto power_up_failed;
				}
	//Rocky add for test{
#if 1
			 if(power_setting->seq_val == MAIN_CAM_VAF)
			 {
				pr_debug("[RK]%s seq_val %d(MAIN_CAM_VAF)\n", __func__, power_setting->seq_val);
			 }
			 else if(power_setting->seq_val == MAIN_CAM_VDIG)
			 {
				pr_debug("[RK]%s seq_val %d(MAIN_CAM_VDIG)\n", __func__, power_setting->seq_val);
			 }
			else
			{
				pr_debug("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);
			}
#endif
//Rocky add for test}

			msm_camera_config_single_vreg(power_info->dev,
				&power_info->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				1);
			break;
		case SENSOR_I2C_MUX:
			pr_debug("[RK]%s:%d SENSOR_I2C_MUX ++++\n", __func__, __LINE__);
			if (power_info->i2c_conf && power_info->i2c_conf->use_i2c_mux)
			{
				pr_debug("[RK]%s:%d Do isp_main_cam_sensor_enable_i2c_mux!!\n", __func__, __LINE__);
				isp_main_cam_enable_i2c_mux(power_info->i2c_conf);
			}
			pr_debug("[RK]%s:%d SENSOR_I2C_MUX ----\n", __func__, __LINE__);

			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			pr_debug("[RK]%s mdelay(%d)\n", __func__, power_setting->delay);
			mdelay(power_setting->delay);
		}
	}

	pr_debug("[RK]%s:%d i2c_util ++++\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		pr_debug("[RK]%s:%d i2c_util [IN]\n", __func__, __LINE__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
			goto power_up_failed;
		}
	}

	pr_debug("[RK]%s:%d i2c_util ----\n", __func__, __LINE__);

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0) {
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
		goto power_up_failed;
	}
	//fihtdc,derekcww,add for front cam when front cam can focus
	/* 0: Main Cam, 1: Front Cam*/
	if(s_ctrl->func_tbl->sensor_match_id != isp_main_cam_match_id)
	{
		//ChunJeng add,if front have auto focus change resolution start 20141017
		pr_err("%s: Modify main resolution to front", __func__);
		preview_resolution=MAIN_RES_1040x780;//fihtdc,derekcwwu, add for tx3
		snapshot_resolution=MAIN_RES_3264x2448;	//RL*_20140710
		hd_resolution=MAIN_RES_1280x720;
		//ChunJeng add,if front have auto focus change resolution end 20141017
		if (ISPI2C_SensorSelectSet_EVT(s_ctrl, 1) < 0)	//after firmware 03.00.04 use "1"
		{
			pr_err("%s: ISPI2C_SensorSelectSet_EVT  failed!\n", __func__);
			return -1;
		}
	}
	else
	{
		preview_resolution=MAIN_RES_2080x1560;//fihtdc,derekcwwu, add for tx3
		snapshot_resolution=MAIN_RES_4160x3120;	//RL*_20140710
		hd_resolution=MAIN_RES_1280x720;
	}
	//fihtdc,derekcww,add for front cam when front cam can focus
	//SW4-HL-Camera-EnhanceKernelLog-MCS-3055-00*{_20140219
	intr_gpio = gpio_conf->cam_gpio_req_tbl[ISP_INTR].gpio;
	pr_debug("\n\n*** [HL] %s,  ISP_INTR_GPIO = %d ***\n\n", __func__, intr_gpio);
	//SW4-HL-Camera-EnhanceKernelLog-MCS-3055-00*}_20140219


#if ENABLE_ISP_INTERRUPT
    rc = isp_init_interrupt(intr_gpio);//fihtdc,derekcww,add for front cam when front cam can focus
    if(rc < 0)
    {
        pr_err("%s: isp_init_interrupt fail. \n", __func__);
        return rc;
    }
#endif
	if(isp_firmware_version == 0x0)
	{
		isp_get_firmware_version();
	}
	pr_err("*********************** ISP FIRMWARE VERSION : 0x%X *********************** \n", isp_firmware_version);

	//s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
	g_pre_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 1;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	main_cam_is_power_on = 1;
	isp_main_cam_inti_parms();

	return rc;

power_up_failed:
	pr_err("[RK]%s DO msm_sensor_power_down!!\n", __func__);
	pr_err("%s:%d failed\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index--; index >= 0; index--) {
		power_setting = &power_setting_array->power_setting[index];
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(power_info->dev,
				&power_info->clk_info[0],
				(struct clk **)&power_setting->data[0],
				power_info->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			gpio_set_value_cansleep(
				gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			msm_camera_config_single_vreg(power_info->dev,
				&power_info->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (power_info->i2c_conf && power_info->i2c_conf->use_i2c_mux)
				isp_main_cam_disable_i2c_mux(power_info->i2c_conf);
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 0);
	//kfree(s_ctrl->reg_ptr);

	return rc;
}

int32_t isp_main_cam_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	int retval = 0;
	 int count = 0;

	//test ++
	//while(1)
	//{
	rc = msm_camera_cci_i2c_write(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,0x88,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_write id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		pr_err("BBox::UEC; 9::1\n");
//		return rc;
	}

	pr_debug("[RK]%s: msm_camera_cci_i2c_write() done!\n", __func__);

	rc = msm_camera_cci_i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr, &chipid,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_read id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		pr_err("BBox::UEC; 9::1\n");
//		return rc;
	//}
	}

	pr_debug("[RK]%s: msm_camera_cci_i2c_read() done!\n", __func__);

	//test --
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}
		pr_debug("%s: expected id : %x, read id : %x\n", __func__, s_ctrl->sensordata->slave_info->sensor_id, chipid);

        #if 0
        if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
                pr_err("msm_sensor_match_id chip id doesnot match\n");
                pr_err("BBox::UEC; 9::2\n");
                return -ENODEV;
        }
        #else
        if (chipid != s_ctrl->sensordata->slave_info->sensor_id)
        {
		pr_debug("\n\n******************* [HL] %s, Match ID fail, START of Retry *************************\n\n", __func__);
                do
                {
                        mdelay(50);
                        rc = msm_camera_cci_i2c_read(
                                s_ctrl->sensor_i2c_client,
                                s_ctrl->sensordata->slave_info->sensor_id_reg_addr, &chipid,
                                MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
			{
				pr_err("%s: %s: msm_camera_cci_i2c_read id 0x88 failed\n", __func__,
				s_ctrl->sensordata->sensor_name);
				pr_err("BBox::UEC; 9::1\n");
				return -ENODEV;
			}
			pr_debug("[RL]%s: expected id : %x, read id : %x, count = %d\n", __func__, s_ctrl->sensordata->slave_info->sensor_id, chipid, count);

			count ++;
                } while((chipid != s_ctrl->sensordata->slave_info->sensor_id) && (count < 30));

		if (chipid != s_ctrl->sensordata->slave_info->sensor_id)
		{
			pr_err("\n\n******************* [HL] %s, Match ID fail, End of Retry *************************\n\n", __func__);
			chipid = 0x88;
			pr_debug("[RL]%s: hardcode_chipid = 0x88\n", __func__);
		}
        }
        #endif

	//create sysfs
	if(example_kobj == NULL)
	{
		example_kobj = kobject_create_and_add("isp_control", kernel_kobj);
		retval = sysfs_create_group(example_kobj, &isp_attr_group);
		if (retval)
			kobject_put(example_kobj);
		//sysfs_create_group(&isp_main_cam_sensor_i2c_client.client.dev->dev.kobj, &isp_attr_group);
	}
	return rc;
}

//SW4-HL-Camera-ErrorHandling-00*{_20140410
static int recover_isp(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct device *dev = NULL;
	#ifdef ENABLE_NORMAL_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;//SW4-Rocky-Camera-PortingCamera_20130719_00
	#endif
	int temp_value = -1;//SW4-Rocky-Camera-PortingCamera_20130719_00
	u32 timeout_count = 1;

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE)//MSM_SENSOR_PLATFORM_DEVICE
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;

	if (isp_main_cam_power_down(s_ctrl) < 0)
	{
		pr_err("%s: power down failed!\n", __func__);
		return -1;
	}

	if (isp_main_cam_power_up(s_ctrl) < 0)
	{
		pr_err("%s: power up failed!\n", __func__);
		return -1;
	}

	#ifdef ENABLE_NORMAL_DEBUG
	pr_debug("=======%s [1] START ===== \n",__func__);
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
	#endif

	pr_debug("%s: pre_res=%d, cur_res=%d\n", __func__, g_pre_res,isp_main_cam_mode);
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	#ifdef ENABLE_NORMAL_DEBUG
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_debug("======%s [1] END ====== \n",__func__);
	#endif

	mdelay(10);

	/*=========reload camera user setting start==========*/
	temp_value = g_cur_bestshot;
	g_cur_bestshot = -1;
	iCatch_set_scene_mode(s_ctrl, temp_value);
	temp_value = g_cur_saturation;
	g_cur_saturation = -1;
	iCatch_set_saturation(s_ctrl, temp_value);
	temp_value = g_cur_sharpness;
	g_cur_sharpness = -1;
	iCatch_set_sharpness(s_ctrl, temp_value);
	temp_value = g_cur_contrast;
	g_cur_contrast = -1;
	iCatch_set_contrast(s_ctrl, temp_value);
	temp_value = g_cur_ledflash;
	g_cur_ledflash = -1;
	iCatch_set_led_flash_mode(s_ctrl, temp_value);
	temp_value = g_cur_ev;
	g_cur_ev = -1;
	iCatch_set_exposure_compensation(s_ctrl, temp_value);
	temp_value = g_cur_iso;
	g_cur_iso = -1;
	iCatch_set_iso(s_ctrl, temp_value);
	/*=========reload camera user setting end============*/

	//switch mode**********************************************
	pr_debug("%s: switch mode****************\n", __func__);
	isp_main_cam_mode = restore_isp_main_cam_mode;  //SW5-Webber-20150513-Add for restore camera resolution for recover mode.
	switch(isp_main_cam_mode)
	{
		case MSM_SENSOR_RES_FULL_PREVIEW:
		case MSM_SENSOR_RES_FULL:
			//SW5-Webber-20150513-Add for restore camera resolution for recover mode++
			//if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			//if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710F, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			//if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x03, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			//if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			//pr_debug("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_4160x3120, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			//SW5-Webber-20150513-Add for restore camera resolution for recover mode--
		break;

        	case MSM_SENSOR_RES_QTR:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, preview_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, add for tx3
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_QTR Clean 0x72F8 \n", __func__);
        	break;
		case MSM_SENSOR_RES_HD:
			//SW4-RL-Camera-modify_HD_RecordingResolution-00*_20140423
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, hd_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_HD Clean 0x72F8 \n", __func__);
		break;
		//SW4-RL-Camera-addFor_FHD_Recording-00+{_20140423
		case MSM_SENSOR_RES_FHD:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		break;
		//SW4-RL-Camera-addFor_FHD_Recording-00+}_20140423
		//SW5-Webber-Camera-add_for_HighFrameRecording_90fps_20150423++
		case MSM_SENSOR_RES_HFR90:
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1040x780, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_HFR90 Clean 0x72F8 \n", __func__);
		}
		break;
		//SW5-Webber-Camera-add_for_HighFrameRecording_90fps_20150423--
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_main_cam_mode);
			return -1;
		break;
	}

	//Force to send stream on command no matter giCatchStreamOff is 1 or not since isp already is power down and power up
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
	giCatchStreamOff = 0;

	g_time_start = jiffies;
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 3*HZ);
	pr_debug("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));
	if (!timeout_count||interrup_error<0)//interrup_error=-1 is error,fihtdc,derekcww,add for get 0x72f8 error
	{
		pr_err("%s: interrupt timedout or ISP error, [wait_time] %d\n", __func__, jiffies_to_msecs(jiffies-g_time_start));
		pr_err("BBox::UEC; 9::4\n");
		interrup_error=0;//fihtdc,derekcww,add for get 0x72f8 error
		return -1;
	}
	else
	{
		pr_err("%s interrupt done\n",__func__);

		#ifdef ENABLE_GPIO_DEBUG
		//gpio_direction_output(DEBUG_GPIO,1);
		mdelay(5);
		gpio_direction_output(DEBUG_GPIO,0);
		pr_debug("%s: DEBUG_GPIO: %d\n", __func__, gpio_get_value(DEBUG_GPIO));
		#endif

		#ifdef ENABLE_NORMAL_DEBUG
		pr_debug("=======%s START ===== \n",__func__);
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
		#endif

		#ifdef ENABLE_NORMAL_DEBUG
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
		pr_debug("======%s  END ====== \n",__func__);
		#endif
	}

	return 0;
}
//SW4-HL-Camera-ErrorHandling-00*}_20140410

static struct msm_sensor_fn_t isp_main_cam_func_tbl = {
	.sensor_start_stream = isp_main_cam_start_stream,
	.sensor_stop_stream = isp_main_cam_stop_stream,
	.sensor_setting = isp_main_cam_setting,		//Orig -marked //HL*_20130927
	//.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	//.sensor_mode_init = msm_sensor_mode_init,
	//.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = isp_main_cam_config,
//	.sensor_config = msm_sensor_config,
	.sensor_power_up = isp_main_cam_power_up,
	.sensor_power_down = isp_main_cam_power_down,
	//.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id = isp_main_cam_match_id,
//Anvoi
	.sensor_isp_af_start = iCatch_start_AF,
	.sensor_set_isp_scene_mode =  iCatch_set_scene_mode,
	.sensor_set_isp_effect_mode = iCatch_set_effect_mode,
	//SW4-L1-HL-Camera-ImplementExposureCompensation-00+_20130227
	.sensor_set_isp_exposure_compensation = iCatch_set_exposure_compensation,
	//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227
	.sensor_set_isp_aec_mode = iCatch_set_aec_mode,
	.sensor_set_isp_af_mode = iCatch_set_AF_Mode,//Rocky_20131005_FORTEST
	.sensor_set_isp_af_roi = iCatch_set_AF_ROI,//Rocky_20131009_FORTEST
	//SW4-L1-HL-Camera-ImplementISO-00+_20130304
	.sensor_set_iso = iCatch_set_iso,
	//SW4-L1-HL-Camera-ImplementWhiteBalance-00+_20130304
	.sensor_set_isp_wb = iCatch_set_wb,

	 //SW4-L1-HL-Camera-ImplementSaturation-00+_20130305
	 .sensor_set_isp_saturation = iCatch_set_saturation,
	 //SW4-L1-HL-Camera-ImplementSharpness-00+_20130305
	 .sensor_set_isp_sharpness = iCatch_set_sharpness,
	 //SW4-L1-HL-Camera-ImplementContrast-00+_20130305
	 .sensor_set_isp_contrast = iCatch_set_contrast,
	 //SW4-L1-HL-Camera-ImplementAntiBanding-00+_20130307
 	 .sensor_set_isp_antibanding = iCatch_set_antibanding,
     //SW5-Webber-Camera-ImplementLedFlash-20130313-start
	 .sensor_set_isp_led_flash_mode=iCatch_set_led_flash_mode,
	 //SW5-Webber-Camera-ImplementLedFlash-20130313-end
	  //SW5-Marx-Camera-ImplementHDR-20130318-start
	 .sensor_set_isp_HDR_mode=iCatch_set_HDR_mode,
	 //SW5-Marx-Camera-ImplementHDR-20130318-end
	.sensor_get_isp_exposure_time = iCatch_get_exposure_time,//SW4-RK-Camera-SetEXIFInformation-00+_20131225
	.sensor_get_isp_iso_value = iCatch_get_iso_value,//SW4-RK-Camera-SetEXIFInformation-00+_20131230
	.sensor_set_isp_wdr = iCatch_set_wdr_mode, //SW4-RL-Camera-WDR-00*_20140117
	.sensor_set_isp_sensor_data = iCatch_set_sensor_data,//SW4-RK-Camera-GetGYRO_GsensorData-00+_20140116
	.sensor_get_isp_flash_status = iCatch_isp_flash_status,//FihtdcCode@AlanHZChang, add for flashLED status from ISP, 2014/02/19
	.sensor_get_isp_3a_info = iCatch_get_3a_info,//SW4-RK-Camera-SetEXIF3AInformation-00+_20140218
	.sensor_set_isp_aec_lock = iCatch_set_aec_lock,		//SW4-RL-Camera-implementAELock-00+_20140710
	.sensor_set_isp_awb_lock = iCatch_set_awb_lock,	//SW4-RL-Camera-implementAWBLock-00+_20140710
};

static struct msm_sensor_ctrl_t isp_main_cam_s_ctrl = {
	//.msm_sensor_reg = &isp_main_cam_regs,
	//.msm_sensor_v4l2_ctrl_info = isp_main_cam_v4l2_ctrl_info,
	//.num_v4l2_ctrl = ARRAY_SIZE(isp_main_cam_v4l2_ctrl_info),
	.sensor_i2c_client = &isp_main_cam_sensor_i2c_client,
	//.sensor_i2c_addr = 0x78,
	.power_setting_array.power_setting = NULL,
	.power_setting_array.size = 0,
//#if ENABLE_SENSOR_REGULATOR
//	.vreg_seq = isp_main_cam_veg_seq,
//	.num_vreg_seq = ARRAY_SIZE(isp_main_cam_veg_seq),
//#endif
//	.sensor_output_reg_addr = &isp_main_cam_reg_addr,
//	.sensor_id_info = &isp_main_cam_id_info,
//	.cam_mode = MSM_SENSOR_MODE_INVALID,
//	.min_delay = 30,
//	.power_seq_delay = 0,
	.msm_sensor_mutex = &isp_main_cam_mut,
	//.sensor_i2c_driver = &isp_main_cam_i2c_driver,
	.sensor_v4l2_subdev_info = isp_main_cam_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(isp_main_cam_subdev_info),
	//.sensor_v4l2_subdev_ops = &isp_main_cam_subdev_ops,
	.func_tbl = &isp_main_cam_func_tbl,
	//.clk_rate = MSM_SENSOR_MCLK_12HZ,  /*1041 port*/
};

static void __exit isp_main_cam_exit_module(void)
{
	pr_debug("%s:%d\n", __func__, __LINE__);
	if (isp_main_cam_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&isp_main_cam_s_ctrl);
		platform_driver_unregister(&isp_main_cam_platform_driver);
	} else
		i2c_del_driver(&isp_main_cam_i2c_driver);
	return;
}

module_init(isp_main_cam_init_module);
module_exit(isp_main_cam_exit_module);

MODULE_DESCRIPTION("isp_main_cam");
MODULE_LICENSE("GPL v2");
