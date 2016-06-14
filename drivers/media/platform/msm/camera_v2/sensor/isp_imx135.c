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
#define ISP_IMX135_C
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include "msm.h"
#include <linux/irq.h>  /*for ISP interrupt*/
#include <linux/i2c.h>  /*for ISP firmware upgrade*/
#include "isp_imx135.h"
#include <linux/workqueue.h>
#include <asm/bug.h>
#include <linux/proc_fs.h>
#include "icatch7002a_img.h"
#include <mach/gpiomux.h>//SW4-Rocky-Camera-PortingCamera_20130719_00
#include <media/msm_cam_sensor.h>
#include <mach/camera2.h>

#include "icatch7002a.h" //ICATCH function

#define SENSOR_NAME "isp_imx135"
#define PLATFORM_DRIVER_NAME "msm_camera_isp_imx135"
#define isp_imx135_obj isp_imx135_##obj

#define ENABLE_ISP_INTERRUPT 1
#define ENABLE_ISP_FIRMWARE_UPGRADE 1
//#define ENABLE_GPIO_DEBUG 1
//#define DEBUG_GPIO 36
#define ENABLE_REG_DEBUG 1
#define ENABLE_CMDQ_DEBUG 1
//#define ENABLE_SENSOR_REGULATOR 1

#define CONFIG_MSMB_CAMERA_DEBUG//SW4-Rocky-Camera-PortingCamera_20130719_00

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


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

static int isp_imx135_mode;
int isp_is_power_on = 0;
//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
static int main_cam_is_power_on = 0;
//SW4-HL-Camera-FixCameraSwitchFailIssue-00+{_20130926
extern int front_cam_is_power_on;
extern int isp_ov5648_mode;
//SW4-HL-Camera-FixCameraSwitchFailIssue-00+}_20130926
int giCatchStreamOff = 0;

static int g_hdr_done = 0;
static int g_irq_requested = 0;
bool iCatch_first_open = true;  //for iCatch ISP used
int Main_Camera_ISP_Ready=0;
int Front_Camera_ISP_Ready=0;
static struct completion g_iCatch_comp;
static int g_pre_res = MSM_SENSOR_INVALID_RES;
static int g_time_start = 0;
static int g_caf_sensor_enable =0;
struct delayed_work CAF_work;

int delayed_work_status=0;
#if ENABLE_ISP_INTERRUPT
static unsigned int ISP_INTR_GPIO = 102;
static unsigned int g_isp_irq;
#endif
DEFINE_MUTEX(isp_imx135_mut);
static struct msm_sensor_ctrl_t isp_imx135_s_ctrl;
static int HDR_mode = 0;
static struct kobject *example_kobj = NULL;

static bool g_isAFCancel = false;//SW4-Rocky-Camera-PortingCamera_20130719_00
static bool g_afmode=0; //0: Auto AF, 1:Full search AF  //SW4-Rocky-Camera-PortingCamera_20130719_00
//static int g_flash_mode = 0;	//"Flash mode for EXIF"
static bool g_enable_roi_debug = false; //add 13M camera TAF support  //SW4-Rocky-Camera-PortingCamera_20130719_00
static bool g_TAEenable = false;	//0: Disable, 1:Enable  //SW4-Rocky-Camera-PortingCamera_20130719_00
static unsigned int isp_firmware_version = 0x0;
static int isp_is_upgrading = 0;
static unsigned int bin_file_version = 0x0;
static bool caf_mode = false;

typedef enum
{
	ICATCH_FW_READY = 0,
	ICATCH_FW_UPGRADING = 1,
	ICATCH_FW_DO_NOT_NEED_UPGRADE = 2,
	ICATCH_FW_UPGRADE_DONE = 3,
	ICATCH_FW_NOT_FOUND = 4,
	ICATCH_FW_CANT_UPGRADE = 5,
}ICATCH_FW_STATUS;

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

#define SIZE_512K ((512)*(1024))	//Rocky_20131024
#define LASTPAGE (((SIZE_512K)/(256))-(1))	//Rocky_20131024
#define FILE_PATH_BOOTCODE "/system/etc/ISP_Firmware.bin"
#define SPI_CMD_RD_ID 				0x9F
#define WAIT_COUNT 0
//#define printf printk
#define SUCCESS 0
unsigned char g_memory_512K[SIZE_512K];	//Rocky_20131024

int32_t isp_imx135_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int32_t isp_imx135_power_down(struct msm_sensor_ctrl_t *s_ctrl);
int32_t isp_imx135_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl, uint16_t res);
static void isp_imx135_start_stream(struct msm_sensor_ctrl_t *s_ctrl);
int32_t isp_imx135_match_id(struct msm_sensor_ctrl_t *s_ctrl);
void iCatch_start_AF(bool on, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
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

static ssize_t isp_proc_write( struct file *filp, const char __user *buff, unsigned long len, void *data )
{
    sscanf(buff, "%d", &isp_enable_recovery) ;
    pr_err("%s: %d\n", __func__, isp_enable_recovery);
    return len;
}

static int isp_proc_read( char *page, char **start, off_t off, int count, int *eof, void *data )
{
    return sprintf(page, "%d\n", isp_enable_recovery);

}

static ssize_t isp_proc_i2c_write( struct file *filp, const char __user *buff, unsigned long len, void *data )
{
    sscanf(buff, "%d", &isp_enable_i2c_dbg) ;
    pr_err("%s: %d\n", __func__, isp_enable_i2c_dbg);
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
	rc = msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, addr, value, MSM_CAMERA_I2C_BYTE_DATA);
}

void hsI2CDataWrite(UINT32 addr, UINT16 value)
{
	I2CDataWrite(addr, value);
}

UINT32 I2CDataRead(UINT32 addr)
{
	UINT16 value = 0;
	int rc = -1;

	rc = msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, addr, &value, MSM_CAMERA_I2C_BYTE_DATA);
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
		pr_err("%s: 512*1024\n",__func__);
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
		pr_err("%s: 1024*1024\n",__func__);
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

#if 0
static int sensor_write_reg_bytes(struct i2c_client *client, u16 addr, unsigned char* val, u32 bytes)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[bytes+2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	   memcpy((data+2), val, bytes);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = bytes+2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err > 0)
			return 0;
		retry++;
//		pr_err("yuv_sensor : i2c transfer failed, retrying %x %llu\n", addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x\n", msg.addr);
	} while (retry <= 3);

	return err;
}

static int sensor_read_reg_bytes(struct i2c_client *client, u16 addr, unsigned char* val, u32 bytes)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[bytes+2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	   memcpy((data+2), val, bytes);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = bytes+2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err > 0)
			return 0;
		retry++;
//		pr_err("yuv_sensor : i2c transfer failed, retrying %x %llu\n", addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x\n", msg.addr);
	} while (retry <= 3);

	return err;
}
#endif

void I2C_7002DmemWr(UINT32 bankNum, UINT32 byteNum, UINT8* pbuf)
{
	UINT32 i, j, bank;
	UINT32 size = 64;
	unsigned char tmp[size];//4

	bank = 0x40+bankNum;
	I2CDataWrite(0x10A6,bank);

	for (i = 0; i < byteNum; i += size) {
		memset(tmp, 0, sizeof(tmp));
		for ( j = 0; j < size; j++)	{
			tmp[j]=(*(pbuf + i + j));
		}
		msm_camera_cci_i2c_write_seq(isp_imx135_s_ctrl.sensor_i2c_client, (0x1800+i), tmp, size);
	}

	bank = 0x40 + ((bankNum + 1) % 2);
	hsI2CDataWrite(0x10A6, bank);
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
//pr_err("[RK]%s +++++\n", __func__);

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
		//pr_err("[RK]%s page:0x%x\n", __func__ , pages);
		if((pages%0x40)==0)
		{
			pr_err("%s:%d page:0x%x\n", __func__ ,__LINE__, pages);
		}
		if((addr>=rsvSec1) && (addr <rsvSec2) && (usage!=0))
		{
			addr += 0x1000;
			pbuf += 0x1000;
			pages -= 0x10;
			continue;
		}
#if 0
		if((pages==1))
		{
			for (i = 0; i < pageSize ; i++)
			{
				pr_err("%2x ",*(pbuf+i));
				if((i%0x10)==0x0f)
					pr_err("\n");
			}
		}
#endif
		dmemBank = pages % 2;

		I2CDataWrite(0x1081,dmemBank*0x20);

		I2CDataWrite(0x1084,(1<<dmemBank));
		//pr_err("[RK]%s dmemBank %u, pageSize %u, pbuf %2x\n", __func__, dmemBank, pageSize, *pbuf);

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
	UINT32 i, size = 0;
	UINT32 pageSize = 0x100;

	addr = addr * pageSize;
	size = pages * pageSize;

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);	/* Write one byte command*/
	I2C_SPIFlashPortWrite((UINT8)(addr >> 16));	/* Send 3 bytes address*/
	I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
	I2C_SPIFlashPortWrite((UINT8)(addr));

	for (i = 0; i < size; i++) {
		*pbuf = I2C_SPIFlashPortRead();
		if((i % 256) == 0)
			pr_err("[%s:%d] page:0x%x\n",__func__ ,__LINE__ ,(i / 256));
		pbuf ++;
	}

	I2CDataWrite(0x40e7,0x01);
	return err;
}

/* BB_WrSPIFlash is main function for burning SPI */
void BB_WrSPIFlash(UINT32 size)
{
	UINT32 id, type;
	UINT32 pages;
	UINT32 spiSize;
	//UINT32 fd;
	int fileSize;
	//UINT8* pbootBuf = NULL;
	unsigned char temp[256] = {0};

	struct file *file_filp = NULL;
	//int output_size=0;
	//loff_t temp = 0;
	//int offset = 0;
	mm_segment_t old_fs;

	if(isp_is_upgrading == ICATCH_FW_UPGRADING)
		return;

	pr_err("loadcode from file\n");

	I2CDataWrite(0x70c4,0x00);
	I2CDataWrite(0x70c5,0x00);

	I2CDataWrite(0x1011,0x01); /* CPU Reset */
	I2CDataWrite(0x001C,0x08);/* FM reset */
	I2CDataWrite(0x001C,0x00);
	I2CDataWrite(0x108C,0x00);/* DMA select */
	I2CDataWrite(0x009a,0x00);/*CPU normal operation */

	//fd = sp5kFsFileOpen( FILE_PATH_BOOTCODE, SP5K_FS_OPEN_RDONLY );
	file_filp = filp_open(FILE_PATH_BOOTCODE, O_RDONLY, 0);
	if (IS_ERR(file_filp)){
		pr_err("%s: Fail to open iCatch ISP BOOTCODE\n", __func__);
		isp_is_upgrading = ICATCH_FW_NOT_FOUND;
		file_filp=NULL;
		return ;
	}

	//copy boot.bin to file_buf
	old_fs=get_fs();
	set_fs(KERNEL_DS);
	fileSize = file_filp->f_op->read(file_filp, (unsigned char __user*)&g_memory_512K, SIZE_512K, &file_filp->f_pos);	//Rocky_20131024

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
		isp_is_upgrading = ICATCH_FW_CANT_UPGRADE;
		pr_err("read id failed\n");
		return;
	}

	/*pr_err("spiSize:0x%x\n",&spiSize);*/
	type = BB_SerialFlashTypeCheck(id, &spiSize);
	pr_err("FlashType: %d, spiSize: %d\n", type, spiSize);
	if( type == 0 )
	{
		isp_is_upgrading = ICATCH_FW_CANT_UPGRADE;
		pr_err("read id failed\n");
		return;
	}

	isp_is_upgrading = ICATCH_FW_UPGRADING;

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
		I2C_SPIFlashWrite_DMA(0,pages,1,g_memory_512K);	//Rocky_20131024
	}

	/* Read last page */
	I2C_SPIFlashRead(LASTPAGE, 1, temp);

	isp_firmware_version = ((temp[255] << 24) | (temp[254] << 16) | (temp[253] << 8) | (temp[252]));
	pr_err("isp_firmware_version: 0x%X\n", isp_firmware_version);

	/* Check checksum */
	isp_is_upgrading = ICATCH_FW_UPGRADE_DONE;

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
		msm_camera_cci_i2c_read_seq(isp_imx135_s_ctrl.sensor_i2c_client,(0x1800+i), tmp,4);
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

void BB_WrSPIFlashByFileName(char *filename)
{
	UINT32 id, type;
	UINT32 pages;
	UINT32 spiSize;
	int fileSize;
	unsigned char temp[256] = {0};
	UINT32 size;

	struct file *file_filp = NULL;
	mm_segment_t old_fs;

	if(isp_is_upgrading == ICATCH_FW_UPGRADING)
		return;

	I2CDataWrite(0x70c4,0x00);
	I2CDataWrite(0x70c5,0x00);

	I2CDataWrite(0x1011,0x01); /* CPU Reset */
	I2CDataWrite(0x001C,0x08);/* FM reset */
	I2CDataWrite(0x001C,0x00);
	I2CDataWrite(0x108C,0x00);/* DMA select */
	I2CDataWrite(0x009a,0x00);/*CPU normal operation */

	file_filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(file_filp)){
		pr_err("%s: Fail to open iCatch ISP BOOTCODE\n", __func__);
		file_filp=NULL;
		return ;
	}

	//copy boot.bin to file_buf
	old_fs=get_fs();
	set_fs(KERNEL_DS);
	fileSize = file_filp->f_op->read(file_filp, (unsigned char __user *)&g_memory_512K, SIZE_512K, &file_filp->f_pos);	//Rocky_20131024

	set_fs(old_fs);
	pr_err("%s fileSize: %d\n",__func__,  fileSize);
	filp_close(file_filp, NULL);

	I2C_SPIInit();

	id = I2C_SPIFlashReadId();
	if(id==0)
	{
		pr_err("read id failed\n");
		return;
	}

	/* Read last page */
	//I2C_SPIFlashRead(4095, 1, temp);
	I2C_SPIFlashRead(LASTPAGE, 1, temp);

	isp_firmware_version = ((temp[255] << 24) | (temp[254] << 16) | (temp[253] << 8) | (temp[252]));
	//HL*_20130910
	//Orig - pr_err("isp_firmware_version: 0x%X\n", isp_firmware_version);
	pr_err("\n\n********************* [HL] %s, Current isp_firmware_version: 0x%X ******************************\n\n", __func__, isp_firmware_version);

	/*pr_err("spiSize:0x%x\n",&spiSize);*/
	type = BB_SerialFlashTypeCheck(id, &spiSize);
	pr_err("FlashType: %d, spiSize: %d\n", type, spiSize);
	if( type == 0 )
	{
		pr_err("read id failed\n");
		return;
	}

	isp_is_upgrading = ICATCH_FW_UPGRADING;

	size = fileSize;
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
	isp_is_upgrading = ICATCH_FW_UPGRADE_DONE;
}

void BB_WrSPIFlash_UpdateNative(void)
{
	UINT32 id;
	UINT32 type;
	UINT32 pages;
	UINT32 spiSize;

	unsigned int fileSize;
	unsigned char * fileBuf;
	unsigned char buf[256] = {0};
	unsigned int isp_firmware_version = 0x00;
	unsigned int platform_firmware_version = 0x00;
	int err;

	if(isp_is_upgrading == ICATCH_FW_UPGRADING)
		return;

	if(isp_is_power_on == 0)
	{
		isp_imx135_power_up(&isp_imx135_s_ctrl);
	}

	msleep(400);

	/*ISP CPU INIT*/
	I2CDataWrite(0x70c4,0x00);
	I2CDataWrite(0x70c5,0x00);
	I2CDataWrite(0x1011,0x01); /* CPU Reset */
	I2CDataWrite(0x001C,0x08);/* FM reset */
	I2CDataWrite(0x001C,0x00);
	I2CDataWrite(0x108C,0x00);/* DMA select */
	I2CDataWrite(0x009a,0x00);/*CPU normal operation */

	msleep(400);

	I2C_SPIInit();

	id = I2C_SPIFlashReadId();
	if(id==0)
	{
		pr_err("%s, read id failed\n", __func__);
		return;
	}

	/* Read last page */
	I2C_SPIFlashRead(LASTPAGE, 1, buf);

	isp_firmware_version = ((buf[255] << 24) | (buf[254] << 16) | (buf[253] << 8) | (buf[252]));
	platform_firmware_version = ((iCatch7002a_img[SIZE_512K-1] << 24) | (iCatch7002a_img[SIZE_512K-2] << 16) |
		(iCatch7002a_img[SIZE_512K-3] << 8) | (iCatch7002a_img[SIZE_512K-4] << 0));
	pr_err("%s: isp_firmware_version=0x%X, platform_firmware_version=0x%X\n", __func__,
		isp_firmware_version, platform_firmware_version);

	isp_is_upgrading = ICATCH_FW_UPGRADING;

	fileSize = sizeof(iCatch7002a_img);
	fileBuf = iCatch7002a_img;

	type = BB_SerialFlashTypeCheck(id, &spiSize);
	pr_err("%s, FlashType: %d, spiSize: %d\n", __func__, type, spiSize);
	if( type == 0 )
	{
		pr_err("%sread id failed\n", __func__);
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
	pr_err("%spages:0x%x\n", __func__, pages);
	BB_EraseSPIFlash(type,spiSize);

	if( type == 2 )
	{
		pr_err("%s, SST operation\n", __func__);
		I2C_SPISstFlashWrite(0,pages,fileBuf);
	}
	else if( type == 1 || type == 3 )
	{
		pr_err("%s, operation\n", __func__);
		err = I2C_SPIFlashWrite_DMA(0, pages, 1, fileBuf);
	}
	isp_is_upgrading = ICATCH_FW_UPGRADE_DONE;
	isp_imx135_power_down(&isp_imx135_s_ctrl);
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

		isp_imx135_power_up(&isp_imx135_s_ctrl);

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
		I2C_SPIFlashRead_DMA(0, SIZE_512K, g_memory_512K);	//Rocky_20131024

		//copy boot.bin to file_buf
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(file_filp->f_op != NULL && file_filp->f_op->write != NULL)
		{
			fileSize = file_filp->f_op->write(file_filp, (unsigned char __user *)&g_memory_512K, SIZE_512K, &offset);	//Rocky_20131024
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

		isp_imx135_power_down(&isp_imx135_s_ctrl);
	}

	return 0;
}

static ssize_t isp_firmware_update_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	int rc = 0;
	if(isp_is_power_on == 0)//If ISP already power on by other control,Ignore update command.
	{
		rc = isp_imx135_power_up(&isp_imx135_s_ctrl);
		if (rc < 0) {
			pr_err("%s power up failed\n", __func__);
		}

		// Avoid laoding code to cause error
		msleep(400);

		if(value > 0)
		{
			BB_WrSPIFlash(value);
		}
		isp_imx135_power_down(&isp_imx135_s_ctrl);
	}

	return size;
}

static ssize_t isp_firmware_native_update_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	pr_err("%s : E\n", __func__);

	if(isp_is_upgrading == ICATCH_FW_UPGRADING) {
		pr_err("%s : firmware is upgrading now, return!!\n", __func__);
	}
	else {
		pr_err("%s : start to upgrade firmware!!\n", __func__);
		BB_WrSPIFlash_UpdateNative();
	}

	pr_err("%s : X\n", __func__);
	return strlen("isp_firmware_native_update_store\n");
}

static ssize_t isp_read_register_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg = 0;
	uint16_t value = 0;

	if (sscanf(buf, "%x", &reg) <= 0) {
		pr_err("Could not tranform the register value\n");
		return -EINVAL;
	}

	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, reg, &value, MSM_CAMERA_I2C_BYTE_DATA);
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
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, (uint16_t)reg, (uint16_t) value, MSM_CAMERA_I2C_BYTE_DATA);
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
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA); //ROI on

	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7140, ((uint16_t)roi_size >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7141, ((uint16_t)roi_size & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7142, ((uint16_t)roi_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7143, ((uint16_t)roi_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7144, ((uint16_t)roi_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7145, ((uint16_t)roi_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);

	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7148, ((uint16_t)roi_size >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7149, ((uint16_t)roi_size & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714a, ((uint16_t)roi_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714b, ((uint16_t)roi_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714c, ((uint16_t)roi_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714d, ((uint16_t)roi_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
    msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714E, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
    msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA);

	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x243F, 0x03, MSM_CAMERA_I2C_BYTE_DATA);

	pr_err("%s,Set ROI roi_size= 0x%X: roi_x=0x%X roi_y=0x%X \n",__func__, roi_size, roi_x, roi_y);
	if(rc){
		pr_err("Write isp_roi fail\n");
	}

	return strlen("isp_read_register_store\n");
}


static ssize_t isp_platform_version_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	unsigned char temp[256] = {0};
	unsigned int need_power_down = 0;

	if(isp_is_power_on == 0)
	{
		isp_imx135_power_up(&isp_imx135_s_ctrl);

		I2CDataWrite(0x70c4,0x00);
		I2CDataWrite(0x70c5,0x00);
		I2CDataWrite(0x1011,0x01); /* CPU Reset */
		I2CDataWrite(0x001C,0x08);/* FM reset */
		I2CDataWrite(0x001C,0x00);
		I2CDataWrite(0x108C,0x00);/* DMA select */
		I2CDataWrite(0x009a,0x00);/*CPU normal operation */

		I2C_SPIInit();
		need_power_down = 1;
	}

	/* Read last page */
	//I2C_SPIFlashRead(4095, 1, temp);
	I2C_SPIFlashRead(LASTPAGE, 1, temp);

	isp_firmware_version = ((temp[255] << 24) | (temp[254] << 16) | (temp[253] << 8) | (temp[252]));
	sprintf(buf, "V%02X.%02X.%02X.%02X\n", temp[255], temp[254], temp[253], temp[252]);

	if(need_power_down == 1)
	{
		isp_imx135_power_down(&isp_imx135_s_ctrl);
		need_power_down = 0;
	}
	return strlen(buf);
}

static ssize_t isp_firmware_update_by_name_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	char filename[256] = {0};

	if(isp_is_power_on == 0)//If ISP already power on by other control,Ignore update command.
	{
		isp_imx135_power_up(&isp_imx135_s_ctrl);

		// Avoid loading code to cause error
		msleep(400);

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
			BB_WrSPIFlashByFileName(filename);
		}
		isp_imx135_power_down(&isp_imx135_s_ctrl);
	}

	return size;
}

static ssize_t isp_firmware_update_by_name_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	if(isp_is_upgrading == ICATCH_FW_READY)
	{
		sprintf(buf, "ready\n");
		return strlen(buf);
	}
	else if(isp_is_upgrading == ICATCH_FW_UPGRADING)
	{
		sprintf(buf, "upgrading\n");
		return strlen(buf);
	}
	else if(isp_is_upgrading == ICATCH_FW_DO_NOT_NEED_UPGRADE)
	{
		sprintf(buf, "no need\n");
		return strlen(buf);
	}
	else if(isp_is_upgrading == ICATCH_FW_UPGRADE_DONE)
	{
		sprintf(buf, "done\n");
		return strlen(buf);
	}
	sprintf(buf, "unknown\n");
	return strlen(buf);
}

static ssize_t isp_file_version_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	char filename[256] = {0};
	int fileSize;
	struct file *file_filp = NULL;
	mm_segment_t old_fs;
	//unsigned char temp[256] = {0};

	/* Parse last one byte from input */
	if(*(buf + (strlen(buf) -1)) == 0x0A)
	{
		memcpy(filename, buf, strlen(buf)-1);
	}
	else
	{
		memcpy(filename, buf, strlen(buf));
	}

	file_filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(file_filp)){
		pr_err("%s: Fail to open iCatch ISP BOOTCODE\n", __func__);
		file_filp=NULL;
		return size;
	}

	//copy boot.bin to file_buf
	old_fs=get_fs();
	set_fs(KERNEL_DS);
	fileSize = file_filp->f_op->read(file_filp, (unsigned char __user *)&g_memory_512K, SIZE_512K, &file_filp->f_pos);	//Rocky_20131024
	set_fs(old_fs);
	pr_err("%s fileSize: %d\n",__func__,  fileSize);
	filp_close(file_filp, NULL);
	bin_file_version = ((g_memory_512K[fileSize-1] << 24) | (g_memory_512K[fileSize-2] << 16) | (g_memory_512K[fileSize-3] << 8) | (g_memory_512K[fileSize-4]));	//Rocky_20131024
	pr_err("file_version 0x%8X\n", bin_file_version);

	return size;
}
static ssize_t isp_file_version_show(struct device *dev, struct  device_attribute *attr , char *buf)
{
	sprintf(buf, "V%02X.%02X.%02X.%02X\n", (unsigned char)(bin_file_version>>24), (unsigned char)(bin_file_version>>16), (unsigned char)(bin_file_version>>8), (unsigned char)bin_file_version);
	return strlen(buf);
}

static ssize_t isp_flash_on_off_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
		if(strncmp(buf, "on", strlen("on")) == 0)
		{
			pr_err("%s: on\n", __func__);
			if(isp_is_power_on == 0)
			{
				isp_imx135_power_up(&isp_imx135_s_ctrl);
				isp_imx135_match_id(&isp_imx135_s_ctrl);
			}
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7104, 02, MSM_CAMERA_I2C_BYTE_DATA);
		}
		else if(strncmp(buf, "off", strlen("off")) == 0)
		{
			pr_err("%s: off\n", __func__);
			if(isp_is_power_on == 1)
			{
				msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7104, 01, MSM_CAMERA_I2C_BYTE_DATA);
				msleep(100);
				isp_imx135_power_down(&isp_imx135_s_ctrl);
			}
		}
		else if(strncmp(buf, "torch", strlen("torch")) == 0)
		{
			pr_err("%s: torch\n", __func__);
			if(isp_is_power_on == 0)
			{
				isp_imx135_power_up(&isp_imx135_s_ctrl);
				isp_imx135_match_id(&isp_imx135_s_ctrl);
			}
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7104, 04, MSM_CAMERA_I2C_BYTE_DATA);
		}


	return strlen("isp_flash_on_off_store\n");
}

//Sys interface for FTM AF trigger
static ssize_t af_trigger_store(struct device *dev, struct  device_attribute *attr, const char *buf, size_t size)
{
	if (strncmp(buf, "on", strlen("on")) == 0) {
		if (isp_is_power_on == 1) {
			pr_err("%s: on\n", __func__);
			iCatch_start_AF(1,ISP_AF_MODE_AUTO,0,0,0,0);
		}
	}
	return strlen("af_trigger_store\n");
}

static DEVICE_ATTR(firmware_update_native, 0644, NULL, isp_firmware_native_update_store);
static DEVICE_ATTR(firmware_update, 0644, isp_firmware_update_show, isp_firmware_update_store);
static DEVICE_ATTR(register_read, 0644, NULL, isp_read_register_store);
static DEVICE_ATTR(register_write, 0644, NULL, isp_write_register_store);
static DEVICE_ATTR(platform_version, 0644, isp_platform_version_show, NULL);
static DEVICE_ATTR(firmware_update_by_name, 0644, isp_firmware_update_by_name_show, isp_firmware_update_by_name_store);
static DEVICE_ATTR(file_version, 0644, isp_file_version_show, isp_file_version_store);
static DEVICE_ATTR(roi, 0644, NULL, isp_roi_store);
static DEVICE_ATTR(flash_on_off, 0644, NULL, isp_flash_on_off_store);
static DEVICE_ATTR(af_trigger, 0644, NULL, af_trigger_store);

static struct attribute *isp_attributes[] = {
		&dev_attr_firmware_update_native.attr,
		&dev_attr_firmware_update.attr,
		&dev_attr_register_read.attr,
		&dev_attr_register_write.attr,
		&dev_attr_platform_version.attr,
		&dev_attr_firmware_update_by_name.attr,
		&dev_attr_file_version.attr,
		&dev_attr_roi.attr,
		&dev_attr_flash_on_off.attr,
		&dev_attr_af_trigger.attr,
		NULL
};

static const struct attribute_group isp_attr_group = {
		.attrs = isp_attributes,
};

#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_camera_i2c_reg_conf isp_imx135_720p_settings[] = {

};
#endif

//HL*_20130906
//Orig - 1
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_camera_i2c_reg_conf isp_sensor_recommend_settings[] = {

};
#endif

static struct v4l2_subdev_info isp_imx135_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

//HL*_20130906
//Orig - 1
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_camera_i2c_reg_conf isp_sensor_config_change_settings[] = {

};
#endif

#if ENABLE_ISP_INTERRUPT
void isp_disable_interrupt(void)
{
	pr_err("%s:\n", __func__);
	disable_irq(g_isp_irq);
}

void isp_enable_interrupt(void)
{
	//HL+_20130909
	 struct irq_desc *desc;

	pr_err("%s\n", __func__);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x72FC, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
	init_completion(&g_iCatch_comp);
	//HL*{_20130909
	#if 0
	enable_irq(g_isp_irq);
	#else
	pr_err("\n\n********************* [HL] %s, check irq before enable it ******************************\n\n", __func__);
	desc = irq_to_desc(g_isp_irq);
	if (desc && desc->depth > 0)
	enable_irq(g_isp_irq);
	#endif
	//HL*}_20130909
}

static irqreturn_t fih_detect_isp_irq_handler(int irq, void *dev_id)
{
	int value1;

	pr_err("%s +\n", __func__);
	//WARN_ON(1);
#ifdef ENABLE_GPIO_DEBUG
        gpio_direction_output(DEBUG_GPIO,1);
#endif
	value1 = gpio_get_value(ISP_INTR_GPIO);
	pr_err("[ISP_INTR]=%d\n",value1);
	if(value1)
	{
		pr_err("send complete\n");
		complete(&g_iCatch_comp);
	}
	else
	{
		pr_err("not real INT\n");
	}

	pr_err("%s -\n", __func__);
	return IRQ_HANDLED;
}

int  isp_init_interrupt(void)
{
	int ret;

	pr_err("%s\n", __func__);

	ret = gpio_request(ISP_INTR_GPIO, "ISP_INTR");
	//SW4-Rocky-Camera-PortingCamera_20130719_00_{
	if (ret < 0)
	{
		gpio_free(ISP_INTR_GPIO);
		pr_err("\n\n[RK]%s GPIO_FREE(%d)\n\n", __func__, ISP_INTR_GPIO);
	}
	ret = gpio_request(ISP_INTR_GPIO, "ISP_INTR");
	//SW4-Rocky-Camera-PortingCamera_20130719_00_}
	if (ret < 0)
		goto err_request_isp_int_gpio;

	ret = gpio_direction_input(ISP_INTR_GPIO);
	if (ret < 0)
		goto err_set_isp_int_gpio;

	g_isp_irq = gpio_to_irq(ISP_INTR_GPIO);
	if (g_isp_irq < 0) {
		ret = g_isp_irq;
		goto err_get_isp_irq_num_failed;
	}

	ret = request_irq(g_isp_irq, fih_detect_isp_irq_handler,
			 IRQF_TRIGGER_RISING, "fih_isp_ISR", NULL);
	pr_err("g_isp_irq=%d\n",g_isp_irq);
	if (ret < 0)
		goto err_request_detect_isp_irq;

	return ret;

err_request_detect_isp_irq:
	pr_err("%s: err_request_detect_isp_irq\n", __func__);

err_get_isp_irq_num_failed:
	pr_err("%s: err_get_isp_irq_num_failed\n", __func__);

err_set_isp_int_gpio:
	pr_err("%s: err_set_isp_int_gpio\n", __func__);
	gpio_free(ISP_INTR_GPIO);

err_request_isp_int_gpio:
	pr_err("%s: err_request_isp_int_gpio\n", __func__);

	return ret;
}
#endif

void isp_imx135_inti_parms(void)
{
	pr_err("%s: +\n", __func__);
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
	pr_err("%s: -\n", __func__);
}

int32_t isp_imx135_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc=0;
	pr_err("%s: res=%d\n", __func__, res);

	switch(res){
		case MSM_SENSOR_RES_FULL: /*snapshot mode, 4160*3120*/
			isp_imx135_mode = MSM_SENSOR_RES_FULL;

		break;

        case MSM_SENSOR_RES_QTR: /*preview mode, 2080*1560*/
            isp_imx135_mode = MSM_SENSOR_RES_QTR;

        break;
	case MSM_SENSOR_RES_HD: /*preview mode, 1920*1080*/
		isp_imx135_mode = MSM_SENSOR_RES_HD;

	break;

	case MSM_SENSOR_RES_FULL_PREVIEW: /*preview mode, 4160*3120*/
		isp_imx135_mode=MSM_SENSOR_RES_FULL_PREVIEW;
		pr_err("%s: isp_imx135_mode=%d\n", __func__, isp_imx135_mode);
	break;
	default:
			pr_err("%s: Do not support this res=%d\n", __func__, res);
	}

	return rc;
}

static void isp_af_work(struct work_struct *work)
{
	g_caf_sensor_enable=1;

	//pr_err("enter %s ", __func__);
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
        SetCAF_gsensor(isp_imx135_s_ctrl.sensordata->gsensor_data().datax,
		isp_imx135_s_ctrl.sensordata->gsensor_data().datay,
		isp_imx135_s_ctrl.sensordata->gsensor_data().dataz);

	 SetCAF_gyro(isp_imx135_s_ctrl.sensordata->gyro_data().datax,
                isp_imx135_s_ctrl.sensordata->gyro_data().datay,
                isp_imx135_s_ctrl.sensordata->gyro_data().dataz);
#endif
	schedule_delayed_work(&CAF_work, 20);
}

static void isp_af_close(void)
{
	pr_err("disable sensor!");
	//isp_imx135_s_ctrl.sensordata->camera_sensor_disable();
}


//HL+{_20130911
void polling_isp_status (void)
{
	uint16_t status = 0x0;
	int count = 0;

	do{
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		pr_err("\n\n********** [HL] %s: status=%d, count=%d **********\n\n", __func__, status, count);
		count ++;
	}while((status != 0x04) && (count <= 400));

	/*status is not ready*/
	if(status != 0x04)
	{
		pr_err("\n\n********** [HL] %s: status = %d is not ready!!!!! **********\n\n", __func__, status);
	}
}
//HL+}_20130911

void wait_for_next_frame(void){
	u32 timeout_count = 1;
	uint16_t status = 0x0;
#ifdef ENABLE_CMDQ_DEBUG
	int i = 0;
#endif
#ifdef ENABLE_REG_DEBUG
	uint16_t gpio_status = 0x0;
#endif
	pr_err("%s: E\n",__func__);
	g_time_start = jiffies;
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 3*HZ);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72f8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: 0x72f8 status: %d\n", __func__, status);
	pr_err("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));

	if (!timeout_count || status!=0x04) {
		pr_err("%s: interrupt timedout or ISP error\n", __func__);
		pr_err("BBox::UEC; 9::4\n");
#ifdef ENABLE_CMDQ_DEBUG
		//dump command queue to debug, total 128 bytes
		pr_err("=======%s: Dump CmdQ START ===== \n",__func__);

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x4284, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x4284, status);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x4285, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x4285, status);

		for(i = 0 ; i < 0x04; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x2030+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x2030+i, status);
		}

		for(i = 0 ; i < 0x04; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x2058+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x2058+i, status);
		}

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x9400, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x9400, status);

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7000, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x7000, status);

		for(i = 0 ; i < 0x04; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7072+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x7072+i, status);
		}

		for(i = 0 ; i < 0x81; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7200+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x7200+i, status);
		}

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72c6, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x72c6, status);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72c7, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x72c7, status);

		pr_err("=======%s: Dump CmdQ END ===== \n",__func__);
#endif
#ifdef ENABLE_REG_DEBUG
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x72f8] = 0x%x\n", status);
		pr_err("[0x0024] = 0x%x\n", gpio_status);
#endif
                pr_err("%s: isp_enable_recovery=%d\n", __func__, isp_enable_recovery);
                if(isp_enable_recovery)
		recover_isp(&isp_imx135_s_ctrl);
		//iCatch_debug();
	} else {
		pr_err("%s interrupt done\n",__func__);
#ifdef ENABLE_GPIO_DEBUG
		//gpio_direction_output(DEBUG_GPIO,1);
		mdelay(5);
		gpio_direction_output(DEBUG_GPIO,0);
		pr_err("%s: DEBUG_GPIO: %d\n", __func__, gpio_get_value(DEBUG_GPIO));
#endif

#ifdef ENABLE_REG_DEBUG
	pr_err("=======%s START ===== \n",__func__);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_REG_DEBUG
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_err("======%s  END ====== \n",__func__);
#endif
	}
}

void wait_for_AE_ready(void)
{
#if 0//Rocky_00-{_20130924
	uint16_t AE_ready=0;
	int count=0;

	iCatch_first_open = false;
	do{
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72C3, &AE_ready, MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		pr_err("%s: AE_ready=%d, count=%d\n", __func__, AE_ready, count);
		count ++;
	}while(((AE_ready&0x01) != 0x01) && (count <=200));

	/*AE is not ready*/
	if((AE_ready&0x01) != 0x01)
	{
		pr_err("%s: AE_ready=%d is not ready!!!!!\n", __func__, AE_ready);
		pr_err("BBox::UEC; 9::8\n");
	}
#endif//Rocky_00-}_20130924
}


#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
int32_t isp_imx135_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	pr_err("%s: update_type=%d, res=%d\n", __func__, update_type, res);

	if (update_type == MSM_SENSOR_REG_INIT) {
		iCatch_first_open=true;
		Main_Camera_ISP_Ready=1;
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		//msm_sensor_write_init_settings(s_ctrl);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		//msm_sensor_write_res_settings(s_ctrl, res);
		isp_imx135_write_res_settings(s_ctrl, res);
		//v4l2_subdev_notify(&s_ctrl->msm_sd, CFG_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->output_settings[res].op_pixel_clk);//for test
		//isp_imx135_write_res_settings(s_ctrl, res);
	}
	return rc;
}
#endif
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
#define ISP_RESET 81
#define ISP_SUSPEND 72
static struct msm_cam_clk_info cam_8960_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_12HZ},
};
#endif
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

static struct msm_sensor_power_setting isp_imx135_power_setting[] = {
	//sensor power +++
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	//sensor power ---
	//ISP power +++
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_DVDD_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_DVDD_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	//ISP_DVDD_EN ---
	//ISP_AVDD_EN +++
	//HL+_20130910
	#if 1
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_AVDD_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	//HL+_20130910
	#endif
	//HL+_20130910
	#if 1
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_AVDD_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 3,	// Orig - 1 	//HL*_20130910
	},
	//HL+_20130910
	#endif
	//ISP_AVDD_EN ---
	//ISP_VCORE_EN +++
	//HL+_20130910
	#if 1
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_VCORE_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	//HL+_20130910
	#endif
	//index 10
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_VCORE_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	//ISP_VCORE_EN ---
	//ISP power ---
	//clock +++
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = MSM_SENSOR_MCLK_12HZ,
		.delay = 0,	//Orig - 1,	//HL*_20130911
	},
	//clock ---
	//ISP GPIO +++
	//ISP_RESET_N +++
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_RESET_N,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_RESET_N,
		.config_val = GPIO_OUT_HIGH,
		.delay = 6,
	},
	//ISP_RESET_N ---
	//ISP_SUSPEND +++
	//HL+_20130911
	#if 1
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_SUSPEND,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	//HL+_20130911
	#endif
	//index 15
	//HL+_20130911
	#if 1
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_SUSPEND,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	//HL+_20130911
	#endif
	//ISP_SUSPEND ---
	//ISP GPIO ---
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

//FIH, Vince, recover ISP+++
static int recover_isp(struct msm_sensor_ctrl_t *s_ctrl)
{
	//struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct device *dev = NULL;
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;//SW4-Rocky-Camera-PortingCamera_20130719_00
	int32_t rc = 0;
	int32_t i;//SW4-Rocky-Camera-PortingCamera_20130719_00
//    int temp_value = -1;//SW4-Rocky-Camera-PortingCamera_20130719_00
	u32 timeout_count = 1;
	//dev = isp_imx135_s_ctrl.pdev.dev;
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE)//MSM_SENSOR_PLATFORM_DEVICE
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;
//power down************************************************************
isp_imx135_power_down(s_ctrl);

#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
	pr_err("%s: power down****************\n", __func__);
	//ISP RESER and SUSPEND set
	pr_err("%s: set gpio ISP_SUSPEND to High\n", __func__);
	gpio_direction_output(ISP_SUSPEND, 1);
	pr_err("%s: set gpio ISP_RESET to low\n", __func__);
	gpio_direction_output(ISP_RESET_N, 0);
	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);
	if (s_ctrl->sensor_device_type == MSM_SENSOR_I2C_DEVICE)
		msm_cam_clk_enable(dev, cam_8960_clk_info, s_ctrl->cam_clk,
			ARRAY_SIZE(cam_8960_clk_info), 0);
#ifdef ENABLE_ISP_INTERRUPT
	if(g_irq_requested)
	{
		isp_disable_interrupt();
		gpio_free(ISP_INTR_GPIO);
		free_irq(g_isp_irq,0);
		g_irq_requested = 0;
	}
#endif
#if ENABLE_SENSOR_REGULATOR

	rc = msm_camera_enable_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);

	if (rc < 0) {
		pr_err("%s: disbale regulator failed\n", __func__);
	}

	rc = msm_camera_config_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	if (rc < 0) {
		pr_err("%s: regulator off failed\n", __func__);
	}
#endif
	cancel_delayed_work_sync(&CAF_work);
	//ISP AVDD and DVDD
	msm_camera_config_gpio_table(data, 0);
	msm_camera_request_gpio_table(data, 0);
#endif
//power up******************************************************
	isp_imx135_power_up(s_ctrl);
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
	pr_err("%s: power up****************\n", __func__);
	msleep(50);

	if (!s_ctrl->reg_ptr) {
		pr_err("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}

	pr_err("%s: set gpio ISP_RESET to Low\n", __func__);
	gpio_direction_output(ISP_RESET, 0);
	pr_err("%s: set gpio ISP_SUSPEND to High\n", __func__);
	gpio_direction_output(ISP_SUSPEND, 1);
	//ISP AVDD and DVDD gpio init
	rc = msm_camera_request_gpio_table(data, 1);
	// ISP config Camera sensor power
#if ENABLE_SENSOR_REGULATOR

	pr_err("%s: enable main camera sensor vreg\n", __func__);
	rc = msm_camera_config_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: regulator on failed\n", __func__);
	}

	rc = msm_camera_enable_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: enable regulator failed\n", __func__);
	}
#endif
	//ISP AVDD and DVDD enable
	rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
	}
        mdelay(5);//refine pwr on seq
	if (s_ctrl->sensor_device_type == MSM_SENSOR_I2C_DEVICE) {
		if (s_ctrl->clk_rate != 0)
			cam_8960_clk_info->clk_rate = s_ctrl->clk_rate;

		rc = msm_cam_clk_enable(dev, cam_8960_clk_info,
			s_ctrl->cam_clk, ARRAY_SIZE(cam_8960_clk_info), 1);
		if (rc < 0) {
			pr_err("%s: clk enable failed\n", __func__);
		}
	}
        mdelay(1);//refine pwr on seq
	//ISP RESER and SUSPEND set
	pr_err("%s: set gpio ISP_RESET to High\n", __func__);
	gpio_direction_output(ISP_RESET, 1);
	pr_err("%s: ISP_RESET: %d\n", __func__, gpio_get_value(ISP_RESET));
	mdelay(6);//refine pwr on seq.
	//msleep(6);
	pr_err("%s: set gpio ISP_SUSPEND to Low\n", __func__);
	gpio_direction_output(ISP_SUSPEND, 0);
	pr_err("%s: ISP_SUSPEND: %d\n", __func__, gpio_get_value(ISP_SUSPEND));
	if (!s_ctrl->power_seq_delay)
		usleep_range(1000, 2000);
	else if (s_ctrl->power_seq_delay < 20)
		usleep_range((s_ctrl->power_seq_delay * 1000),
			((s_ctrl->power_seq_delay * 1000) + 1000));
	else
		msleep(s_ctrl->power_seq_delay);

	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(1);
#endif
//switch mode**********************************************
	pr_err("%s: switch mode****************\n", __func__);
#ifdef ENABLE_REG_DEBUG

        pr_err("=======%s [1] START ===== \n",__func__);
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
	pr_err("%s: pre_res=%d, cur_res=%d\n", __func__, g_pre_res,isp_imx135_mode);
        msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_REG_DEBUG
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
        pr_err("======%s [1] END ====== \n",__func__);
#endif
	mdelay(10);
/*=========reload camera user setting start==========*/
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
    temp_value = g_cur_bestshot;
    g_cur_bestshot = -1;
    iCatch_set_scene_mode(temp_value);
    temp_value = g_cur_saturation;
    g_cur_saturation = -1;
    iCatch_set_saturation(temp_value);
    temp_value = g_cur_sharpness;
    g_cur_sharpness = -1;
    iCatch_set_sharpness(temp_value);
    temp_value = g_cur_contrast;
    g_cur_contrast = -1;
    iCatch_set_contrast(temp_value);
    temp_value = g_cur_ledflash;
    g_cur_ledflash = -1;
    iCatch_set_led_flash_mode(&isp_imx135_s_ctrl, temp_value);
    temp_value = g_cur_ev;
    g_cur_ev = -1;
    iCatch_set_exposure_compensation(temp_value);
    temp_value = g_cur_iso;
    g_cur_iso = -1;
    iCatch_set_iso(temp_value);
#endif
/*=========reload camera user setting end============*/
#if ENABLE_ISP_INTERRUPT
	if(!g_irq_requested)
	{
		rc = isp_init_interrupt();
		if(rc < 0)
			pr_err("%s: isp_init_interrupt fail. \n", __func__);
		g_irq_requested = 1;
		isp_enable_interrupt();
	}
#endif
#if 1//SW4-Rocky-Camera-PortingCamera_20130719_00
	switch(isp_imx135_mode)
	{
		case MSM_SENSOR_RES_FULL_PREVIEW:
		case MSM_SENSOR_RES_FULL:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7106, MAIN_RES_4160x3120, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode
			pr_err("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
			break;

		case MSM_SENSOR_RES_QTR:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7106, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7120, RES_2080x1560, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode

			pr_err("%s: waiting for MSM_SENSOR_RES_QTR Clean 0x72F8 \n", __func__);
			break;
		case MSM_SENSOR_RES_HD:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7106, MAIN_RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode

			/* It is recommended host utilize interrupt to wait for frame being ready */
			/* Here we use polling 0x72F8 = 0x04 to wait for frame being ready */
			pr_err("%s: waiting for MSM_SENSOR_RES_HD Clean 0x72F8 \n", __func__);
		break;
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_imx135_mode);
	}
#endif
	g_time_start = jiffies;
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 3*HZ);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72f8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: 0x72f8 status: %d\n", __func__, status);
	pr_err("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));
	if (!timeout_count) {
		pr_err("%s: interrupt timedout 2\n", __func__);
#ifdef ENABLE_CMDQ_DEBUG
		//dump command queue to debug, total 128 bytes
		pr_err("=======%s: Dump CmdQ START ===== \n",__func__);

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x4284, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x4284, status);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x4285, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x4285, status);

		for(i = 0 ; i < 0x04; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x2030+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x2030+i, status);
		}

		for(i = 0 ; i < 0x04; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x2058+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x2058+i, status);
		}

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x9400, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x9400, status);

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7000, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x7000, status);

		for(i = 0 ; i < 0x04; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7072+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x7072+i, status);
		}

		for(i = 0 ; i < 0x81; i++)
		{
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7200+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x7200+i, status);
		}

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72c6, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x72c6, status);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72c7, &status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x%x] = 0x%x\n", 0x72c7, status);

		pr_err("=======%s: Dump CmdQ END ===== \n",__func__);
#endif
#ifdef ENABLE_REG_DEBUG
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x72f8] = 0x%x\n", status);
		pr_err("[0x0024] = 0x%x\n", gpio_status);
#endif
	} else {
		pr_err("%s interrupt done\n",__func__);
#ifdef ENABLE_GPIO_DEBUG
		//gpio_direction_output(DEBUG_GPIO,1);
		mdelay(5);
		gpio_direction_output(DEBUG_GPIO,0);
		pr_err("%s: DEBUG_GPIO: %d\n", __func__, gpio_get_value(DEBUG_GPIO));
#endif

#ifdef ENABLE_REG_DEBUG
	pr_err("=======%s START ===== \n",__func__);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_REG_DEBUG
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_err("======%s  END ====== \n",__func__);
#endif
	}

	wait_for_AE_ready();
	return 0;
}
//FIH, Vince, recover ISP---

static const struct i2c_device_id isp_imx135_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&isp_imx135_s_ctrl},
	{ }
};

static int32_t isp_imx135_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &isp_imx135_s_ctrl);
}

static struct i2c_driver isp_imx135_i2c_driver = {
	.id_table = isp_imx135_i2c_id,
	.probe  = isp_imx135_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client isp_imx135_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id isp_imx135_dt_match[] = {
	{.compatible = "qcom,isp_imx135", .data = &isp_imx135_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, isp_imx135_dt_match);

static struct platform_driver isp_imx135_platform_driver = {
	.driver = {
		.name = "qcom,isp_imx135",
		.owner = THIS_MODULE,
		.of_match_table = isp_imx135_dt_match,
	},
};

static int32_t isp_imx135_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	pr_err("%s:%d++++of_match_device++++\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log
	match = of_match_device(isp_imx135_dt_match, &pdev->dev);
	pr_err("%s:%d----of_match_device----\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log
	pr_err("%s:%d++++msm_sensor_platform_probe++++\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_err("%s:%d----msm_sensor_platform_probe----\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log

	return rc;
}

static int __init isp_imx135_init_module(void)
{
	int32_t rc = 0;
	pr_info("[RK]%s:%d++++\n", __func__, __LINE__);

	INIT_DELAYED_WORK(&CAF_work, isp_af_work);
	isp_proc_dir = proc_mkdir(ISP_DBG_PROC_DIR, NULL);
	isp_proc_entry = create_proc_entry( "enable_recovery", 0644, isp_proc_dir);
        isp_proc_entry->read_proc = isp_proc_read;
        isp_proc_entry->write_proc = isp_proc_write;
        isp_proc_entry_i2c = create_proc_entry( "enable_i2c_dbg", 0644, isp_proc_dir);
        isp_proc_entry_i2c->read_proc = isp_proc_i2c_read;
        isp_proc_entry_i2c->write_proc = isp_proc_i2c_write;

	rc = platform_driver_probe(&isp_imx135_platform_driver,
		isp_imx135_platform_probe);

	pr_err("[RK]%s:%d rc %d\n", __func__, __LINE__, rc);

	if (!rc)
		return rc;

	rc = i2c_add_driver(&isp_imx135_i2c_driver);

	pr_err("[RK]%s:%d(%d)----\n", __func__, __LINE__, rc);

	return rc;

}

//*************************************
//ISP Function implement
//Anvoi 20130212
void iCatch_wait_AF_done(void)
{
	int retry=0;
	u16 status;

	//Wait AF interrupt
	do{
		mutex_unlock(isp_imx135_s_ctrl.msm_sensor_mutex);
		if(retry==0){
			msleep(120); // LiJen: wait for ISP AF process
		}else{
			msleep(15);
		}
		mutex_lock(isp_imx135_s_ctrl.msm_sensor_mutex);

		if(g_isAFCancel == true){
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("%s g_isAFCancel = ture\n",__func__);
			break;
		}

		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72a0, &status, MSM_CAMERA_I2C_BYTE_DATA);
		//need del
		//sensor_read_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x72a0, &status);
		pr_err("status=0x%X, retry=%d\n",status,retry);
		retry += 1;
	} while((status != 0x00) && (retry < 250));
}

void iCatch_set_touch_AF(kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w){
	u32 af_w=0x80, af_h=0x80, af_x=0x33, af_y=0x33;
	//const u32 roi_bytes = 11;
	//unsigned char data[roi_bytes];
	pr_err("%s +++\n",__func__);
	pr_err("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);

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
	pr_err("%s: af_x:0x%x af_y:0x%x af_w:0x%x af_h:0x%x g_TAEenable:%d\n", __func__, af_x, af_y, af_w, af_h,g_TAEenable);

	// set focus coodinate and retangle
	if(ISP_AF_MODE_CAF == mode){
		msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA); // iCatch:ROI only vaild in focus mode = auto
		//need del
		//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // iCatch:ROI only vaild in focus mode = auto
	}

	//AF ROI
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA); //ROI on

	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7140, (af_w >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7141, (af_w & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7142, (af_x >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7143, (af_x & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7144, (af_y >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7145, (af_y & 0xFF), MSM_CAMERA_I2C_BYTE_DATA);

	//need del
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7188, 0x01);//ROI on

	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7140, (af_w >> 8));
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7141, (af_w & 0xFF));
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7142, (af_x >> 8));
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7143, (af_x & 0xFF));
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7144, (af_y >> 8));
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7145, (af_y & 0xFF));

	//sensor_write_reg_bytes(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7140, data, roi_bytes);

	//AE trigger
	if(g_TAEenable == true){
		msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714e, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
		//need del
		//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x714e, 0x02);
	}

	// AF trigger
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x72a0, 0x01, MSM_CAMERA_I2C_BYTE_DATA);	// Clean AF state bit ??
	//need del
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7146, 0x01);
	//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x72a0, 0x01);     // Clean AF state bit

	pr_err("%s ---\n",__func__);
}

void iCatch_start_AF(bool on, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	//int retry=0;
	//u16 status;
	pr_info("%s +++ Param(%d,%d,%d,%d,%d,%d)\n",__func__,on,mode,coordinate_x,coordinate_y,rectangle_h,rectangle_w);
	g_isAFCancel = false;
	//"[A60K][8M][NA][Others]implement cancel autofocus in 8M camera with ISP"
	if (on) {
		switch (mode) {
			case ISP_AF_MODE_MACRO:
				caf_mode=false;
				msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
				//Any point focus setting
				iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
				break;
			case ISP_AF_MODE_CAF:
				caf_mode=true;
				//Any point focus setting
				iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
				msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
				break;
			case ISP_AF_MODE_AUTO:
				caf_mode=false;
				if (g_afmode == 0) {
					msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
				}
				else {
					msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
				}
				//Any point focus setting
				iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);

				//Wait AF done
				iCatch_wait_AF_done();
				break;
			case ISP_AF_MODE_NORMAL: // normal focus instead of full search af
				caf_mode=false;
				msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
				iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w);
				//Wait AF done
				//iCatch_wait_AF_done();
				break;
			case ISP_AF_MODE_INFINITY:
				caf_mode=false;
				msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
				goto end;
			case ISP_AF_MODE_UNCHANGED:
			case ISP_AF_MODE_MAX:
			default:
				pr_err("%s mode(%d) is not support \n",__func__,mode);
				goto end;
		}

			//Enable ROI debug
			if (true == g_enable_roi_debug) {
				if (ISP_AF_MODE_AUTO != mode) {
					//Wait AF done
					iCatch_wait_AF_done();
				}
				msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x243f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			}
	}
	else {
		//Cancel AutoFocus
		//AF_START: AF release
		g_isAFCancel = true;
		pr_err("Cancel autofocus\n");
	}
end:
	pr_info("%s ---\n",__func__);
}

#if 0
void iCatch_set_caf_mode(bool continuous)
{
	pr_err("%s +++ as %d\n",__func__,continuous);

	if(continuous){
		//restart CAF if is in streaming
		pr_err("pre_res=%d, caf_mode=%d\n",g_pre_res, caf_mode);
		//if(g_pre_res!=MSM_SENSOR_INVALID_RES && !caf_mode)
        if((g_pre_res == MSM_SENSOR_RES_QTR || g_pre_res == MSM_SENSOR_RES_HD) && caf_mode == false)
		{
			pr_err("%s: restart CAF\n",__func__);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7188, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7140, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7141, 0x47, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7142, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7143, 0x5d, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7144, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7145, 0x5d, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7146, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	        }
                caf_mode = true;
	}else{
		if(g_pre_res!=MSM_SENSOR_INVALID_RES){
		    pr_err("%s: lock Focus\n",__func__);
		    msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7105, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		    //msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		}
                caf_mode = false;
	}
	pr_err("%s ---\n",__func__);
}

void iCatch_set_scene_mode(kernel_camera_bestshot_mode_type mode)
{
    int temp_value = -1;
	pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_bestshot == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_bestshot = mode;
	switch(mode)
	{
        case ISP_CAMERA_BESTSHOT_NORMAL:
		case ISP_CAMERA_BESTSHOT_AUTO:
		case ISP_CAMERA_BESTSHOT_OFF:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x00);
			break;
		case ISP_CAMERA_BESTSHOT_LANDSCAPE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x06);
			break;
		case ISP_CAMERA_BESTSHOT_SNOW:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0B);
			break;
		case ISP_CAMERA_BESTSHOT_SUNSET:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x0E, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0E);
			break;
		case ISP_CAMERA_BESTSHOT_NIGHT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x07);
			break;
		case ISP_CAMERA_BESTSHOT_PORTRAIT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0A);
			break;
		case ISP_CAMERA_BESTSHOT_BACKLIGHT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x16, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x16);
			break;
		case ISP_CAMERA_BESTSHOT_SPORTS:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0C);
			break;
		case ISP_CAMERA_BESTSHOT_FLOWERS: //mapping vivid in asusAP
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x05);  // set effect vivid
			break;
		case ISP_CAMERA_BESTSHOT_PARTY:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x09);
			break;
		case ISP_CAMERA_BESTSHOT_BEACH:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x03);
			break;
		case ISP_CAMERA_BESTSHOT_ANTISHAKE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x0D, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0D);
			break;
		case ISP_CAMERA_BESTSHOT_CANDLELIGHT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x04);
			break;
		case ISP_CAMERA_BESTSHOT_FIREWORKS:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x05);
			break;
		case ISP_CAMERA_BESTSHOT_NIGHT_PORTRAIT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x08);
			break;
		case ISP_CAMERA_BESTSHOT_ACTION:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7109, 0x01);
			break;
        case ISP_CAMERA_BESTSHOT_TEXT:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_HIGHSENSITIVITY:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_LANDSCAPEPORTRAIT:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_KID:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x11, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_PET:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x12, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_FLOWER:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x13, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_SOFTFLOWINGWATER:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x14, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_FOOD:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x15, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case ISP_CAMERA_BESTSHOT_INDOOR:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7109, 0x17, MSM_CAMERA_I2C_BYTE_DATA);
            break;
		case ISP_CAMERA_BESTSHOT_THEATRE:
		case ISP_CAMERA_BESTSHOT_AR:
		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}
    if(mode == ISP_CAMERA_BESTSHOT_NORMAL || mode == ISP_CAMERA_BESTSHOT_OFF || mode == ISP_CAMERA_BESTSHOT_AUTO) {
        temp_value = g_cur_whitebalance;
        g_cur_whitebalance = -1;
        iCatch_set_wb(temp_value);
        temp_value = g_cur_effect;
        g_cur_effect = -1;
        iCatch_set_effect_mode(temp_value);
    }
	pr_err("%s ---\n",__func__);
}

void iCatch_set_effect_mode(int8_t mode)
{
	pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_effect == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_effect = mode;
	switch(mode)
	{
		case CAMERA_EFFECT_OFF:
		case CAMERA_EFFECT_NORMAL:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x00);
			break;
		case CAMERA_EFFECT_MONO:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x04);
			break;
		case CAMERA_EFFECT_NEGATIVE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x02);
			break;
		case CAMERA_EFFECT_SEPIA:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x03);
			break;
		case CAMERA_EFFECT_AQUA:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x01);
			break;
		case CAMERA_EFFECT_AURA:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x06);
			break;
		case CAMERA_EFFECT_VINTAGE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x07);
			break;
		case CAMERA_EFFECT_VINTAGE2:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x08);
			break;
		case CAMERA_EFFECT_LOMO:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x09);
			break;
		case CAMERA_EFFECT_RED:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0A);
			break;
		case CAMERA_EFFECT_BLUE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0B);
			break;
		case CAMERA_EFFECT_GREEN:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			//sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0C);
			break;
        case CAMERA_EFFECT_VIVID:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
            //sensor_write_reg(isp_imx135_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0C);
            break;
        case CAMERA_EFFECT_AURA_RED:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7119, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case CAMERA_EFFECT_AURA_ORANGE:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7119, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case CAMERA_EFFECT_AURA_YELLOW:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7119, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case CAMERA_EFFECT_AURA_GREEN:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7119, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case CAMERA_EFFECT_AURA_BLUE:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7119, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case CAMERA_EFFECT_AURA_VIOLET:
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7119, 0x20, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case CAMERA_EFFECT_YELLOW:
		case CAMERA_EFFECT_SOLARIZE:
		case CAMERA_EFFECT_POSTERIZE:
		case CAMERA_EFFECT_WHITEBOARD:
		case CAMERA_EFFECT_BLACKBOARD:
		case CAMERA_EFFECT_EMBOSS:
		case CAMERA_EFFECT_SKETCH:
		case CAMERA_EFFECT_NEON:
		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}
	pr_err("%s ---\n",__func__);
}
//**********************************************

//SW4-L1-HL-Camera-ImplementExposureCompensation-00+{_20130227
void iCatch_set_exposure_compensation(int8_t value)
{
	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s +++ value(%d)\n",__func__,value);
	if(g_cur_ev == value)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_ev = value;
	switch(value)
	{
		case CAMERA_EXPOSURE_COMPENSATION_LV0:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7103, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_EXPOSURE_COMPENSATION_LV1:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7103, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_EXPOSURE_COMPENSATION_LV2:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7103, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_EXPOSURE_COMPENSATION_LV3:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7103, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_EXPOSURE_COMPENSATION_LV4:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7103, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}

	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementExposureCompensation-00+}_20130227

//SW4-L1-HL-Camera-ImplementExposureMeter-00+{_20130227
void iCatch_set_aec_mode(int8_t mode)
{
	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_aec == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_aec = mode;
	switch(mode)
	{
		case CAMERA_SETAE_AVERAGE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710E, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			g_TAEenable = false;
			break;
		case CAMERA_SETAE_CENWEIGHT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710E, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			g_TAEenable = false;
			break;
		case CAMERA_SETAE_SPOT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710E, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			g_TAEenable = true;
			break;
		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}
	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementExposureMeter-00+}_20130227

//SW4-L1-HL-Camera-ImplementISO-00+{_20130304
void iCatch_set_iso(int8_t mode)
{
	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_iso == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_iso = mode;
	switch(mode)
	{
		case CAMERA_ISO_ISOAUTO:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_ISO_ISO100:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_ISO_ISO200:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_ISO_ISO400:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_ISO_ISO800:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		//SW4-L1-HL-Camera-ImplementISO-01+{_20130313
		case CAMERA_ISO_ISO1600:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_ISO_ISO50:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7110, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		//SW4-L1-HL-Camera-ImplementISO-01+}_20130313

		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}
	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementISO-00+}_20130304

//SW4-L1-HL-Camera-ImplementWhiteBalance-00+{_20130304
void iCatch_set_wb(int8_t mode)
{
	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(g_cur_whitebalance == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_whitebalance = mode;
	switch(mode)
	{
		case CAMERA_WB_AUTO:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710A, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_WB_DAYLIGHT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710A, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_WB_CLOUDY_DAYLIGHT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710A, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_WB_FLUORESCENT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710A, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		case CAMERA_WB_INCANDESCENT:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710A, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;

       case CAMERA_WB_SHADE:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x710A, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;

		default:
			pr_err("%s mode(%d) is not support \n",__func__,mode);
			break;
	}

	//SW4-Roger-Camera-remove_log-00-_20130705
	//pr_err("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementWhiteBalance-00+}_20130304
#endif
//SW4-L1-HL-Camera-ImplementSaturation-00+{_20130305
void iCatch_set_saturation(int8_t value)
{
	//SW4-Roger-Camera-remove_log-00-_20130705
	pr_err("%s +++ value(%d)\n",__func__,value);
	if(g_cur_saturation == value)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_saturation = value;
	switch(value)
	{
		case CAMERA_SATURATION_FIHLV0:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7117, 0xD3, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SATURATION_FIHLV1:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7117, 0xE9, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SATURATION_FIHLV2:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7117, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SATURATION_FIHLV3:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7117, 0x17, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SATURATION_FIHLV4:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7117, 0x2D, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}
	//SW4-Roger-Camera-remove_log-00-_20130705
	pr_err("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementSaturation-00+}_20130305

//SW4-L1-HL-Camera-ImplementSharpness-00+{_20130305
void iCatch_set_sharpness(int8_t value)
{
	//SW4-Roger-Camera-remove_log-00-_20130705
	pr_err("%s +++ value(%d)\n",__func__,value);
	if(g_cur_sharpness == value)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_sharpness = value;
	switch(value)
	{
		case CAMERA_SHARPNESS_FIHLV0:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7116, 0xA6, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SHARPNESS_FIHLV1:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7116, 0xD3, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SHARPNESS_FIHLV2:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7116, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SHARPNESS_FIHLV3:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7116, 0x2D, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case CAMERA_SHARPNESS_FIHLV4:
			msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x7116, 0x5A, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		default:
			pr_err("%s value(%d) is not support \n",__func__,value);
			break;
	}
	//SW4-Roger-Camera-remove_log-00-_20130705
	pr_err("%s ---\n",__func__);
}
//SW4-L1-HL-Camera-ImplementSharpness-00+}_20130305

void iCatch_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_err("%s : flash_mode : %d \n", __func__, mode);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
	if(g_cur_ledflash == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_ledflash = mode;
	if(mode==1)  //auto flash
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	else if(mode==2)  //flash on
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
	else if(mode==3)  //torch
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
	else  //0 & other value, flash off
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
}

//SW4-L1-HL-Camera-ImplementAntiBanding-00+{_20130307
void iCatch_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_err("%s +++ value(%d)\n",__func__,value);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
	if(g_cur_antibanding == value)
	{
		pr_err("%s: ignore setting\n",__func__);
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
}

void iCatch_set_wb(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
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
	pr_err("%s ---\n",__func__);
}

void iCatch_set_effect_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
	if(g_cur_effect == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_effect = mode;
	switch(mode)
	{
		case MSM_CAMERA_EFFECT_MODE_OFF:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_MODE_MONO:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_MODE_NEGATIVE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_MODE_SEPIA:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_MODE_AQUA:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_AURA:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_VINTAGE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_VINTAGE2:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_LOMO:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_RED:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_BLUE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_GREEN:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_EFFECT_VIVID:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7102, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
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
	pr_err("%s ---\n",__func__);
}

void iCatch_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	int temp_value = -1;
	pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
	if(g_cur_bestshot == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_bestshot = mode;

	if(mode == MSM_CAMERA_SCENE_MODE_HDR)
	{
		HDR_mode = 1;
		pr_err("%s HDR_mode : %d \n", __func__, mode);
	}
	else
	{
		HDR_mode = 0;
		pr_err("%s HDR_mode : %d \n", __func__, mode);
	}

	switch(mode)
	{
		case MSM_CAMERA_SCENE_MODE_AUTO:
		case MSM_CAMERA_SCENE_MODE_OFF:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_LANDSCAPE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_SNOW:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0B, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_SUNSET:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0E, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_NIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_PORTRAIT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_BACKLIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x16, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_SPORTS:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0C, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_FLOWERS:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x13, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_PARTY:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x09, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_BEACH:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_ANTISHAKE:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0D, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_CANDLELIGHT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_FIREWORKS:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_NIGHT_PORTRAIT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_ACTION:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_TEXT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_HIGHSENSITIVITY:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x0F, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_LANDSCAPEPORTRAIT:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_KID:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x11, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_PET:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x12, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_FLOWER:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x13, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_SOFTFLOWINGWATER:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x14, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_FOOD:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7109, 0x15, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_CAMERA_SCENE_MODE_INDOOR:
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
		//TODO: need to implement white balance and effect
		temp_value = g_cur_whitebalance;
		g_cur_whitebalance = -1;
		iCatch_set_wb(s_ctrl, temp_value);
		temp_value = g_cur_effect;
		g_cur_effect = -1;
		iCatch_set_effect_mode(s_ctrl, temp_value);
	}
	pr_err("%s ---\n",__func__);
}

void iCatch_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
	pr_err("%s +++ mode(%d)\n",__func__,mode);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
	if(g_cur_iso == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_iso = mode;
	if((mode >= MSM_CAMERA_ISO_MODE_AUTO) && (mode < MSM_CAMERA_ISO_MODE_MAX))
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7110, iCatch_iso_value_table[mode], MSM_CAMERA_I2C_BYTE_DATA);
	else
		pr_err("%s:%d: iso mode(%d) is not supported\n", __func__, __LINE__, mode);
	pr_err("%s ---\n",__func__);
}

void iCatch_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int8_t value)
{
	pr_err("%s +++ value(%d)\n",__func__,value);
	if(s_ctrl == NULL)
	{
		pr_err("%s: s_ctrl is a null pointer, return\n", __func__);
		return;
	}
	if(g_cur_contrast == value)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_contrast = value;
	if((value >= MSM_CAMERA_CONTRAST_FIHLV0) && (value < MSM_CAMERA_CONTRAST_FIHLVMAX))
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7115, iCatch_contrast_value_table[value], MSM_CAMERA_I2C_BYTE_DATA);
	else
		pr_err("%s:%d: contrast mode(%d) is not supported\n", __func__, __LINE__, value);
	pr_err("%s ---\n",__func__);
}

#if 0

//SW5-Webber-Camera-ImplementLedFlash-20130313-start
void iCatch_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    pr_err("flash_mode : %d \n", mode);
	if(g_cur_ledflash == mode)
	{
		pr_err("%s: ignore setting\n",__func__);
		return;
	}
	g_cur_ledflash = mode;
    if(mode==1)//auto flash
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
    else if(mode==2)//flash on
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
    else if(mode==3)//torch
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
    else//0 & other value, flash off
        msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7104, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
}
//SW5-Webber-Camera-ImplementLedFlash-20130313-end

//SW5-Marx-Camera-ImplementHDR-20130318-start
void iCatch_set_HDR_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    if(mode==1)
      HDR_mode=1;
    else
      HDR_mode=0;
    pr_err("HDR_mode : %d \n", mode);
}
//SW5-Marx-Camera-ImplementHDR-20130318-end

//SW5-Webber-Camera-ImplementCancelAF-20130331-begin
void iCatch_set_cancel_af(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    uint16_t status=0;
	pr_err("%s : E \n",__func__);
    msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7285, &status, MSM_CAMERA_I2C_BYTE_DATA);
    pr_err("%s: focus mode = %d, g_pre_res=%d\n", __func__, status, g_pre_res);
    if((status == 0x01|| status == 0x00) && g_pre_res != MSM_SENSOR_INVALID_RES)//af mode or macro mode
    {
        rc = msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x714f, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	if(rc < 0)
		pr_err("failed to cancel af \n");
    }

	pr_err("%s : X \n",__func__);
}
//SW5-Webber-Camera-ImplementCancelAF-20130331-end

void iCatch_set_aec_lock(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    pr_err("%s aec_lock: %d \n", __func__, mode);
    if(g_cur_aec_lock == mode)
    {
        pr_err("%s: ignore setting\n",__func__);
        return;
    }
    if(mode==1)
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x03, MSM_CAMERA_I2C_BYTE_DATA);
    else
      msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x71EB, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
    g_cur_aec_lock = mode;
}

void iCatch_set_awb_lock(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode)
{
    pr_err("%s awb_lock: %d \n", __func__, mode);
    if(g_cur_awb_lock == mode)
    //if(g_cur_aec_lock == mode)
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
#endif

void wait_for_iCatch_really_ready(void)
{
	uint16_t res1 = 0;
	uint16_t res2 = 0;
	uint16_t res3 = 0;
	uint16_t res4 = 0;
	int count = 0;

	do
	{
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7072, &res1, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7073, &res2, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7074, &res3, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7075, &res4, MSM_CAMERA_I2C_BYTE_DATA);

		msleep(5);

		//RL*-modify kernel log level_201301021
		//Orig--pr_err("%s: frame_ready=%d, count=%d\n", __func__, frame_ready, count);
		pr_debug("%s: res1 =0x%x, count = %d \n", __func__, res1, count);
		pr_debug("%s: res2 =0x%x, count = %d\n", __func__, res2, count);
		pr_debug("%s: res3 =0x%x, count = %d\n", __func__, res3, count);
		pr_debug("%s: res4 =0x%x, count = %d\n", __func__, res4, count);

		count ++;
	}while(((res1 != 0x40) || (res2 != 0x10) || (res3 != 0x30) || (res4 != 0x0c)) && (count <=200));

	/*frame is not ready*/
	if ((res1 != 0xC0) || (res2 != 0xC) || (res3 != 0x90) || (res4 != 0x9))
	{
		pr_debug("%s: res1 = 0x%x\n", __func__, res1);
		pr_debug("%s: res2 = 0x%x\n", __func__, res2);
		pr_debug("%s: res3 = 0x%x\n", __func__, res3);
		pr_debug("%s: res4 = 0x%x\n", __func__, res4);

		pr_err("BBox::UEC; 9::8\n");
	}
}
static void isp_imx135_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc=0;
#ifdef CONFIG_FIH_FTM
	int	count = 0;
	uint16_t mainSensorIdL = 0, mainSensorIdH = 0;
	uint16_t AF_status=0,AF_Pass=0;
#endif
#ifdef ENABLE_REG_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
        pr_err("=======%s [1] START ===== \n",__func__);
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif

#if 0 //temp disable this function for full resolution issues,
	pr_err("%s: pre_res=%d, cur_res=%d\n", __func__, g_pre_res,isp_imx135_mode);
	if(g_pre_res==isp_imx135_mode)
	{
		pr_err("%s: ignore mode change\n",__func__);
		if (giCatchStreamOff == 1)
		{
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
			pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
			giCatchStreamOff = 0;
		}
		return;
	}
#endif
        msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_REG_DEBUG
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
        pr_err("======%s [1] END ====== \n",__func__);
#endif
	mdelay(10);
	if(true == iCatch_first_open){
#if ENABLE_ISP_INTERRUPT
	if(!g_irq_requested)
	{
		rc = isp_init_interrupt();
		if(rc < 0)
			pr_err("%s: isp_init_interrupt fail. \n", __func__);
		g_irq_requested = 1;
		isp_enable_interrupt();
	}
#endif
		//iCatch_first_open = false;
		//set_isp_calibration_table(0x01);
	}
	switch(isp_imx135_mode)
	{
		case MSM_SENSOR_RES_FULL_PREVIEW:
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_4160x3120, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode

			pr_err("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
		break;

		case MSM_SENSOR_RES_FULL:

			//To capture mode
			if (HDR_mode == 1) {
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7127, 0x17, MSM_CAMERA_I2C_BYTE_DATA);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710F, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
				g_hdr_done = 1;
			}
			else {
				//Fix me: Android 4.4 always pick full resolution,For ICATCH ISP Case need to fix this issues.
				//Workaround:Set preview / snapshot resolution are same.
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, PREVIEW_RES, MAIN_RES_4160x3120, MSM_CAMERA_I2C_BYTE_DATA);
				msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, MODE_SET, PREVIEW_MODE, MSM_CAMERA_I2C_BYTE_DATA);
			}

			pr_err("%s: waiting for MSM_SENSOR_RES_FULL Clean 0x72F8 \n", __func__);
			//make sure TAE off for next preview.
			//msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x714E, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_SENSOR_RES_QTR:
		{
			if (g_hdr_done)	{
				pr_err("Skip switching mode again by HDR mode\n");
				g_hdr_done = 0;
				break;
			}
			//HL*_20130912
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, RES_2080x1560, MSM_CAMERA_I2C_BYTE_DATA);

#ifdef CONFIG_FIH_FTM
		do {
			msleep(5);
			msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72C6, &mainSensorIdL, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("%s: 0x72C6 mainSensorIdL: %d\n", __func__, mainSensorIdL);

			msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72C7, &mainSensorIdH, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("%s: 0x72C7 mainSensorIdH: %d\n", __func__, mainSensorIdH);
			count++;
		}while(mainSensorIdH == 0 && mainSensorIdL == 0 && count < 20);

		if (mainSensorIdL == 0x91 && mainSensorIdH == 0x0)
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode
		else
			break;
#else
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode
#endif
		pr_err("%s: waiting for MSM_SENSOR_RES_QTR Clean 0x72F8 \n", __func__);
		}
			break;
		case MSM_SENSOR_RES_HD:
			//msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, 0x0D, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode

			/* It is recommended host utilize interrupt to wait for frame being ready */
			/* Here we use polling 0x72F8 = 0x04 to wait for frame being ready */

		pr_err("%s: waiting for MSM_SENSOR_RES_HD Clean 0x72F8 \n", __func__);
		break;
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_imx135_mode);
	}
	if (giCatchStreamOff == 1)
	{
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
		giCatchStreamOff = 0;
	}
	#if 1//Rocky_00+20130924  //orig:0
	wait_for_next_frame();
	#else
	polling_isp_status();
	#endif
	//HL*}_20130911
	//HL*{_20130909
	#if 0
	if(true == iCatch_first_open){
		wait_for_AE_ready();
	}
	#else
	if ((isp_imx135_mode==MSM_SENSOR_RES_FULL) || (isp_imx135_mode==MSM_SENSOR_RES_FULL_PREVIEW))
	{
		wait_for_iCatch_really_ready();
		//Fix me: Android 4.4 always pick full resolution,For ICATCH ISP Case need to fix this issues.
		//Workaround:for Full size preview
		iCatch_start_AF(1,ISP_AF_MODE_AUTO,0,0,0,0);
	}
	pr_err("\n\n*************** [HL] %s, NO wait_for_AE_ready() ***********************\n \n", __func__);
	#endif
	//HL*}_20130909

	g_pre_res = isp_imx135_mode;
}

static void isp_imx135_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_err("%s\n", __func__);
	if (giCatchStreamOff == 0)
	{
		//Orig - //msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		pr_debug("%s, SEND STREAM OFF command to 7002A\n", __func__);
		giCatchStreamOff = 1;
	}
}

int isp_imx135_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	int res = 0;
	int32_t setting_lev;
	if(s_ctrl == NULL || argp == NULL) {
		pr_err("%s(%d): s_ctrl or argp is a null pointer, return\n", __func__, __LINE__);
		return -EFAULT;
	}
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {

	case CFG_GET_SENSOR_INFO:
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

	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;

	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
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
			power_setting_array->size; slave_index++) {
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
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

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
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
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
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t write_slave_addr = 0;
		uint16_t orig_slave_addr = 0;

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
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

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
				pr_err("%s calling power up\n", __func__);
				if (s_ctrl->func_tbl->sensor_power_up)
					rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
				else
					rc = -EFAULT;
			break;
		case CFG_POWER_DOWN:
				if (s_ctrl->func_tbl->sensor_power_down)
					rc = s_ctrl->func_tbl->sensor_power_down(
						s_ctrl);
				else
					rc = -EFAULT;
			break;
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
		pr_err("\n\n******************* [HL] %s, CFG_SET_INIT_SETTING *************************\n\n", __func__);
		//HL-{_20130906
		#if 0
		/* 1. Write Recommend settings */
		/* 2. Write change settings */
		pr_err("[RK]%s : CFG_SET_INIT_SETTING!!\n", __func__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, isp_sensor_recommend_settings,
			ARRAY_SIZE(isp_sensor_recommend_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		//pr_err("[RK]%s : isp_sensor_recommend_settings = %ld\n", __func__, rc);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			isp_sensor_config_change_settings,
			ARRAY_SIZE(isp_sensor_config_change_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		//pr_err("[RK]%s : isp_sensor_config_change_settings = %ld\n", __func__, rc);
		#endif
		pr_err("\n\n******************* [HL] %s, NO fake int setting *************************\n\n", __func__);
		//HL-}_20130906
		//HL-{_20130906
		//SW4-HL-Camera-FixCameraSwitchFailIssue-00*{_20130926
		if (!front_cam_is_power_on)
		{
			pr_err("\n\n******************* [HL] %s, front_cam_is_power_on is %d, set isp_imx135_mode = MSM_SENSOR_RES_QTR  *************************\n\n", __func__, front_cam_is_power_on);
			Main_Camera_ISP_Ready=1;
			isp_imx135_mode = MSM_SENSOR_RES_QTR;
		}
		else
		{
			pr_err("\n\n******************* [HL] %s, front_cam_is_power_on is %d, set isp_ov5648_mode = MSM_SENSOR_RES_QTR *************************\n\n", __func__, front_cam_is_power_on);
			isp_ov5648_mode = MSM_SENSOR_RES_QTR;
		}
		//SW4-HL-Camera-FixCameraSwitchFailIssue-00*}_20130926

		pr_err("[RK]%s : isp_imx135_mode = MSM_SENSOR_RES_QTR  <------  ForTest!!!!\n", __func__);
		rc = 0;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
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
		pr_err("[RK]%s : CFG_SET_RESOLUTION!!\n", __func__);
		if (copy_from_user(&res,
			(void *)cdata->cfg.setting, sizeof(res))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		if(!front_cam_is_power_on){
			isp_imx135_write_res_settings(s_ctrl,res);
		}
		else {
			isp_ov5648_mode = res;
		}
		pr_err("CFG_SET_RESOLUTION=%d \n",res);
		break;

		case CFG_SET_START_STREAM://orig:CFG_START_STREAM //SW4-Rocky-Camera-PortingCamera_20130719_00
		pr_err("[RK]%s : CFG_SET_START_STREAM!!\n", __func__);
			if (s_ctrl->func_tbl->sensor_start_stream == NULL) {
				rc = -EFAULT;
				pr_err("[RK]%s : sensor_start_stream == NULL!!\n", __func__);
				break;
			}
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			pr_err("[RK]%s : X!!\n", __func__);
			break;

		case CFG_SET_STOP_STREAM://orig: //SW4-Rocky-Camera-PortingCamera_20130719_00
		pr_err("[RK]%s : CFG_SET_STOP_STREAM!!\n", __func__);
			if (s_ctrl->func_tbl->sensor_stop_stream == NULL) {
				rc = -EFAULT;
				pr_err("[RK]%s : sensor_stop_stream == NULL!!\n", __func__);
				break;
			}
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			pr_err("[RK]%s : X!!\n", __func__);
			break;
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
		case CFG_GET_CSI_PARAMS:
			if (s_ctrl->func_tbl->sensor_get_csi_params == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->sensor_get_csi_params(
				s_ctrl,
				&cdata.cfg.csi_lane_params);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
#endif

#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
		case CFG_SET_VISION_MODE:
			if (s_ctrl->func_tbl->sensor_set_vision_mode)
				rc = s_ctrl->func_tbl->sensor_set_vision_mode(
					s_ctrl, cdata.cfg.vision_mode_enable);
			else
				rc = -EFAULT;
				break;
		case CFG_SET_VISION_AE:
			if (s_ctrl->func_tbl->sensor_set_vision_ae_control)
				rc = s_ctrl->func_tbl->
					sensor_set_vision_ae_control(
					s_ctrl, cdata.cfg.vision_ae);
			else
				rc = -EFAULT;
			break;
//*************************************
//ISP Function implement
//Anvoi 20130212

		case CFG_SET_ISP_CAF_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_caf_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_CAF_MODE mode(%d)\n",cdata.cfg.focus.af_continue);
			s_ctrl->func_tbl->
			sensor_set_isp_caf_mode(cdata.cfg.focus.af_continue);
			break;

		case CFG_SET_ISP_EFFECT_MODE:
			pr_err("%s:%d calling CFG_SET_ISP_EFFECT_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->
			sensor_set_isp_effect_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_ISP_EFFECT_MODE mode(%d)\n",cdata.cfg.effect);
			s_ctrl->func_tbl->
			sensor_set_isp_effect_mode(cdata.cfg.effect);
			break;
		case CFG_SET_ISP_SCENE_MODE:
			pr_err("%s:%d calling CFG_SET_ISP_SCENE_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->
			sensor_set_isp_scene_mode == NULL) {
				pr_err("%s:%d calling sensor_set_isp_scene_mode is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_ISP_SCENE_MODE mode(%d)\n",cdata.cfg.scene);
			s_ctrl->func_tbl->
			sensor_set_isp_scene_mode(cdata.cfg.scene);
			break;
#endif
		case CFG_SET_AUTOFOCUS://VKY:CFG_ISP_AF_START:
			pr_err("%s:%d CFG_ISP_AF_START case E: \n", __func__, __LINE__);

			if (s_ctrl->func_tbl->sensor_isp_af_start == NULL) {
				rc = -EFAULT;
				pr_err("%s:%d sensor_isp_af_start is NULL\n", __func__, __LINE__);
				pr_err("%s:%d CFG_ISP_AF_START case X: \n", __func__, __LINE__);
				break;
			}
			pr_err("CFG_ISP_AF_START afParam(%d,%d,%d,%d,%d,%d)\n",cdata->cfg.focus.af_enable,cdata->cfg.focus.mode,cdata->cfg.focus.coordinate_x,cdata->cfg.focus.coordinate_y,cdata->cfg.focus.rectangle_h,cdata->cfg.focus.rectangle_w);
			s_ctrl->func_tbl->sensor_isp_af_start(
				cdata->cfg.focus.af_enable,
				cdata->cfg.focus.mode,
				cdata->cfg.focus.coordinate_x,
				cdata->cfg.focus.coordinate_y,
				cdata->cfg.focus.rectangle_h,
				cdata->cfg.focus.rectangle_w);
				pr_err("%s:%d CFG_ISP_AF_START case X: \n", __func__, __LINE__);

			break;
#if 0
//****************************************

		//SW4-L1-HL-Camera-ImplementExposureCompensation-00+{_20130227
		case CFG_SET_EXPOSURE_COMPENSATION:
			//SW4-Roger-Camera-remove_log-00-_20130705
			//pr_err("%s:%d calling CFG_SET_EXPOSURE_COMPENSATION\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_exposure_compensation ==  NULL) {
				pr_err("%s:%d calling sensor_set_isp_exposure_compensation is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			//SW4-Roger-Camera-remove_log-00-_20130705
			//pr_err("CFG_SET_EXPOSURE_COMPENSATION mode(%d)\n",cdata.cfg.exp_compensation);
			s_ctrl->func_tbl->
			sensor_set_isp_exposure_compensation(cdata.cfg.exp_compensation);
			break;
		//SW4-L1-HL-Camera-ImplementExposureCompensation-00+}_20130227
		//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227
		case CFG_SET_EXPOSURE_MODE:
			//SW4-Roger-Camera-remove_log-00-_20130705
			//pr_err("%s:%d calling CFG_SET_EXPOSURE_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_aec_mode ==  NULL) {
				pr_err("%s:%d calling sensor_set_isp_aec_mode is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			//SW4-Roger-Camera-remove_log-00-_20130705
			//pr_err("CFG_SET_EXPOSURE_MODE mode(%d)\n",cdata.cfg.ae_mode);
			s_ctrl->func_tbl->
			sensor_set_isp_aec_mode(cdata.cfg.ae_mode);
			break;
		//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227

		//20130227@Rocky add iso/wb function[START]
		case CFG_SET_ISO:
			//SW4-Roger-Camera-remove_log-00-_20130705
			//pr_err("%s:%d calling CFG_SET_ISO\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_iso == NULL) {
				pr_err("%s:%d calling sensor_set_iso is NULL\n", __func__, __LINE__);

				rc = -EFAULT;
				break;
			}
			//SW4-Roger-Camera-remove_log-00-_20130705
			//pr_err("CFG_SET_ISO mode(%d)\n",cdata.cfg.iso_type);
			s_ctrl->func_tbl->sensor_set_iso(cdata.cfg.iso_type);
			break;

		case CFG_SET_WB:
			pr_err("%s:%d calling CFG_SET_WB\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_wb == NULL) {
				pr_err("%s:%d calling sensor_set_isp_wb is NULL\n", __func__, __LINE__);

				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_WB mode(%d)\n",cdata.cfg.wb_val);
			s_ctrl->func_tbl->sensor_set_isp_wb(cdata.cfg.wb_val);
			break;
		//20130227@Rocky add iso/wb function[END]
#endif
		//SW4-L1-HL-Camera-ImplementSaturation-00+{_20130305
		case CFG_SET_SATURATION:
			//SW4-Roger-Camera-remove_log-00-_20130705
			pr_err("%s:%d calling CFG_SET_SATURATION\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_saturation ==  NULL)
			{
				pr_err("%s:%d calling sensor_set_isp_saturation is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			//SW4-Roger-Camera-remove_log-00-_20130705
			pr_err("CFG_SET_SATURATION mode(%d)\n",cdata->cfg.saturation);
			s_ctrl->func_tbl->
			sensor_set_isp_saturation(cdata->cfg.saturation);
			break;
		//SW4-L1-HL-Camera-ImplementSaturation-00+}_20130305
		//SW4-L1-HL-Camera-ImplementSharpness-00+{_20130305
		case CFG_SET_SHARPNESS:
			//SW4-Roger-Camera-remove_log-00-_20130705
			pr_err("%s:%d calling CFG_SET_SHARPNESS\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_sharpness ==  NULL)
			{
				pr_err("%s:%d calling sensor_set_isp_sharpness is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			//SW4-Roger-Camera-remove_log-00-_20130705
			pr_err("CFG_SET_SHARPNESS mode(%d)\n",cdata->cfg.sharpness);
			s_ctrl->func_tbl->
			sensor_set_isp_sharpness(cdata->cfg.sharpness);
			break;
		//SW4-L1-HL-Camera-ImplementSharpness-00+}_20130305
		case CFG_SET_FLASH_MODE:
			pr_err("%s:%d calling CFG_SET_FLASH_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_led_flash_mode == NULL) {
				pr_err("%s:%d calling sensor_set_isp_led_flash_mode is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_err("%s:%d CFG_SET_FLASH_MODE mode(%d)\n", __func__, __LINE__, setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_led_flash_mode(s_ctrl, setting_lev);
			break;
		case CFG_SET_ANTIBANDING:
			pr_err("%s:%d calling CFG_SET_ANTIBANDING\n", __func__, __LINE__);
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
			pr_err("CFG_SET_ANTIBANDING value(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_antibanding(s_ctrl, setting_lev);
			break;
		case CFG_SET_EFFECT:
			pr_err("%s:%d calling CFG_SET_EFFECT\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_effect_mode == NULL) {
				rc = -EFAULT;
				pr_err("%s:%d calling sensor_set_isp_effect_mode is NULL\n", __func__, __LINE__);
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting, sizeof(int32_t)))
			{
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_EFFECT mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_effect_mode(s_ctrl, setting_lev);
			break;
		case CFG_SET_BESTSHOT_MODE:
			pr_err("%s:%d calling CFG_SET_BESTSHOT_MODE\n", __func__, __LINE__);
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
			pr_err("CFG_SET_BESTSHOT_MODE mode(%d)\n",setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_scene_mode(s_ctrl, setting_lev);
			break;
		case CFG_SET_WHITE_BALANCE:
			pr_err("%s:%d calling CFG_SET_WHITE_BALANCE\n", __func__, __LINE__);
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
			pr_err("CFG_SET_WB mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_wb(s_ctrl, setting_lev);
			break;
		case CFG_SET_ISO:
			pr_err("%s:%d calling CFG_SET_ISO\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_iso == NULL) {
				pr_err("%s:%d calling sensor_set_isp_iso is NULL\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			if (copy_from_user(&setting_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_ISO mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_iso(s_ctrl, setting_lev);
			break;
		case CFG_SET_CONTRAST:
			pr_err("%s:%d calling CFG_SET_CONTRAST\n", __func__, __LINE__);
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
			pr_err("CFG_SET_CONTRAST mode(%d)\n", setting_lev);
			s_ctrl->func_tbl->sensor_set_isp_contrast(s_ctrl, setting_lev);
			break;
#if 0
		//SW5-Webber-Camera-ImplementAutoFocus-20130311-start
        case CFG_GET_FOCUS_STATUS:
             //remove log
            //pr_err("%s:%d calling CFG_GET_FOCUS_STATUS\n", __func__, __LINE__);
            if(cdata.cfg.focus.af_get_choice==0)
            {
                msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72a0, &AF_done, MSM_CAMERA_I2C_BYTE_DATA);
                if(AF_done == 0x01) //in busy
                    cdata.cfg.focus.af_done=0;
                else if(AF_done == 0x00)//in idle
                    cdata.cfg.focus.af_done=1;
                else if(AF_done == 0x10) //CAF idle
                    cdata.cfg.focus.af_done=2;
                else if(AF_done == 0x11) //CAF busy
                    cdata.cfg.focus.af_done=3;
                 //remove log
                //pr_err("%s, cdata.cfg.focus.af_done : %d \n", __func__, cdata.cfg.focus.af_done);
            }
            if(cdata.cfg.focus.af_get_choice==1)
            {
                msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72a1, &AF_status, MSM_CAMERA_I2C_BYTE_DATA);
                if(AF_status == 0x01) //focus failed
                    cdata.cfg.focus.af_status=0;
                else//focus success
                    cdata.cfg.focus.af_status=1;
                pr_err("%s, cdata.cfg.focus.af_status : %d \n", __func__, cdata.cfg.focus.af_status);
            }
            if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
            break;
        //SW5-Webber-Camera-ImplementAutoFocus-20130311-end
        //SW5-Webber-Camera-ImplementLedFlash-20130313-start
		case CFG_SET_AUTOFLASH:
			pr_err("%s:%d calling CFG_SET_ISP_LED_FALSH_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_led_flash_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_ISP_LED_FALSH_MODE mode(%d)\n",cdata.cfg.led_flash_mode);
			s_ctrl->func_tbl->sensor_set_isp_led_flash_mode(s_ctrl, cdata.cfg.led_flash_mode);
			break;
		//SW5-Webber-Camera-ImplementLedFlash-20130313-start
        //SW5-Webber-Camera-ImplementGetIsoValue-20130314-start
        case CFG_GET_ISO_VALUE:
		    pr_err("%s:%d calling CFG_GET_ISO_VALUE\n", __func__, __LINE__);
		    msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72b7, &iso_value_low, MSM_CAMERA_I2C_BYTE_DATA);
		    msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72b8, &iso_value_high, MSM_CAMERA_I2C_BYTE_DATA);
		    result = (iso_value_high << 8) + iso_value_low;
		    pr_err("iso_value_low : %d, iso_value_high : %d, result : %d \n", iso_value_low, iso_value_high, result);
            cdata.cfg.iso_value = result;
            if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
		break;
        //SW5-Webber-Camera-ImplementGetIsoValue-20130314-end

    //FIHLX_VKY, Fix APPG2-624, Oscar Lin, 2013/03/14 {
		case CFG_GET_EXPOSURE_VALUE:
		  pr_err("%s:%d calling CFG_GET_EXPOSURE_VALUE\n", __func__, __LINE__);
		  msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72b0, &exposure_numerator,     MSM_CAMERA_I2C_BYTE_DATA);
		  msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72b1, &exposure_denominator_L, MSM_CAMERA_I2C_BYTE_DATA);
		  msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72b2, &exposure_denominator_M, MSM_CAMERA_I2C_BYTE_DATA);
		  msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72b3, &exposure_denominator_H, MSM_CAMERA_I2C_BYTE_DATA);

		  exposure_denominator = (exposure_denominator_H << 16) + (exposure_denominator_M << 8) + exposure_denominator_L;//FIHTDC@20130605 Rocky modify for exposure time

		  cdata.cfg.exposure_value.num = exposure_numerator;
		  cdata.cfg.exposure_value.denom= exposure_denominator;

		  pr_err("CFG_GET_EXPOSURE_VALUE, Exposure N/(H-M-L): %d/(%d-%d-%d) \n", exposure_numerator, exposure_denominator_L, exposure_denominator_M, exposure_denominator_H);

		  if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
		    rc = -EFAULT;

		  break;

		case CFG_GET_3A_INFO:
		  pr_err("%s:%d calling CFG_GET_3A_INFO\n", __func__, __LINE__);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72d8, &capExpIdx,           MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72d9, &capAgcIdx,           MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72da, &pvExpIdx,            MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72db, &pvAgcIdx,            MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72dc, &aeEntry,             MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72dd, &aeLuma,              MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72de, &aeStep,              MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72df, &aetarget,            MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e0, &awbRGain_LSB,        MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e1, &awbRGain_MSB,        MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e2, &awbBGain_LSB,        MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e3, &awbBGain_MSB,        MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e4, &awbiCandidateIdx,    MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e5, &awbDynamicIQCtIdx,   MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e6, &afVertical_step_LSB, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e7, &afVertical_step_MSB, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e8, &afMulti_Step_LSB,    MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72e9, &afMulti_Step_MSB,    MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72ea, &afFinal_Step_LSB,    MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72eb, &afFinal_Step_MSB,    MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72ec, &afFinal_Result_LSB,  MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72ed, &afFinal_Result_MSB,  MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72ee, &threeA_Version,      MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72ef, &threeA_ID,           MSM_CAMERA_I2C_BYTE_DATA);

		  cdata.cfg.threeA_info.capExpIdx = capExpIdx;
		  cdata.cfg.threeA_info.capAgcIdx = capAgcIdx;
		  cdata.cfg.threeA_info.pvExpIdx = pvExpIdx;
		  cdata.cfg.threeA_info.pvAgcIdx = pvAgcIdx;
		  cdata.cfg.threeA_info.aeEntry = aeEntry;
		  cdata.cfg.threeA_info.aeLuma = aeLuma;
		  cdata.cfg.threeA_info.aeStep = aeStep;
		  cdata.cfg.threeA_info.aetarget = aetarget;
		  cdata.cfg.threeA_info.awbRGain_LSB = awbRGain_LSB;
		  cdata.cfg.threeA_info.awbRGain_MSB = awbRGain_MSB;
		  cdata.cfg.threeA_info.awbBGain_LSB = awbBGain_LSB;
		  cdata.cfg.threeA_info.awbBGain_MSB = awbBGain_MSB;
		  cdata.cfg.threeA_info.awbiCandidateIdx = awbiCandidateIdx;
		  cdata.cfg.threeA_info.awbDynamicIQCtIdx = awbDynamicIQCtIdx;
		  cdata.cfg.threeA_info.afVertical_step_LSB = afVertical_step_LSB;
		  cdata.cfg.threeA_info.afVertical_step_MSB = afVertical_step_MSB;
		  cdata.cfg.threeA_info.afMulti_Step_LSB = afMulti_Step_LSB;
		  cdata.cfg.threeA_info.afMulti_Step_MSB = afMulti_Step_MSB;
		  cdata.cfg.threeA_info.afFinal_Step_LSB = afFinal_Step_LSB;
		  cdata.cfg.threeA_info.afFinal_Step_MSB = afFinal_Step_MSB;
		  cdata.cfg.threeA_info.afFinal_Result_LSB = afFinal_Result_LSB;
		  cdata.cfg.threeA_info.afFinal_Result_MSB = afFinal_Result_MSB;
		  cdata.cfg.threeA_info.threeA_Version = threeA_Version;
		  cdata.cfg.threeA_info.threeA_ID = threeA_ID;
          /*
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, capExpIdx           = %d \n", capExpIdx          );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, capAgcIdx           = %d \n", capAgcIdx          );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, pvExpIdx            = %d \n", pvExpIdx           );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, pvAgcIdx            = %d \n", pvAgcIdx           );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, aeEntry             = %d \n", aeEntry            );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, aeLuma              = %d \n", aeLuma             );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, aeStep              = %d \n", aeStep             );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, aetarget            = %d \n", aetarget           );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, awbRGain_LSB        = %d \n", awbRGain_LSB       );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, awbRGain_MSB        = %d \n", awbRGain_MSB       );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, awbBGain_LSB        = %d \n", awbBGain_LSB       );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, awbBGain_MSB        = %d \n", awbBGain_MSB       );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, awbiCandidateIdx    = %d \n", awbiCandidateIdx   );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, awbDynamicIQCtIdx   = %d \n", awbDynamicIQCtIdx  );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afVertical_step_LSB = %d \n", afVertical_step_LSB);
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afVertical_step_MSB = %d \n", afVertical_step_MSB);
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afMulti_Step_LSB    = %d \n", afMulti_Step_LSB   );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afMulti_Step_MSB    = %d \n", afMulti_Step_MSB   );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afFinal_Step_LSB    = %d \n", afFinal_Step_LSB   );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afFinal_Step_MSB    = %d \n", afFinal_Step_MSB   );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afFinal_Result_LSB  = %d \n", afFinal_Result_LSB );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, afFinal_Result_MSB  = %d \n", afFinal_Result_MSB );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, threeA_Version      = %d \n", threeA_Version     );
			pr_err("isp_imx135_config(), CFG_GET_3A_INFO, threeA_ID           = %d \n", threeA_ID          );
           */
			if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
			  rc = -EFAULT;
			break;
    //FIHLX_VKY, Fix APPG2-624, Oscar Lin, 2013/03/14 }
	//SW5-marx-Camera-ImplementHDR-20130318-start
		case CFG_SET_HDR:
			pr_err("%s:%d calling CFG_SET_ISP_HDR_MODE\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_HDR_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_ISP_LED_FALSH_MODE mode(%d)\n",cdata.cfg.hdr_mode);
			s_ctrl->func_tbl->sensor_set_isp_HDR_mode(s_ctrl, cdata.cfg.hdr_mode);
			break;
		//SW5-marx-Camera-ImplementHDR-20130318-end
#endif
        //SW5-Webber-Camera-ImplementCancelAF-20130331-begin
        case CFG_CANCEL_AUTOFOCUS://vky:CFG_SET_CANCEL_AF:
		    pr_err("%s:%d calling CFG_SET_CANCEL_AF\n", __func__, __LINE__);
		    if (s_ctrl->func_tbl->sensor_set_cancel_af ==  NULL) {
			    pr_err("%s:%d calling sensor_set_cancel_af is NULL\n", __func__, __LINE__);
			    rc = -EFAULT;
			    break;
		    }
		    s_ctrl->func_tbl->sensor_set_cancel_af(s_ctrl);
		    break;
        //SW5-Webber-Camera-ImplementCancelAF-20130331-end
#if 0
    //FIHLX_VKY, Fix VKY-9255, Oscar Lin, 2013/05/16 {
		case CFG_GET_FLASH_STATE:
			msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x72c3, &flash_state, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("isp_imx135_config(), CFG_GET_FLASH_STATE, flash_state(before) = %d \n", flash_state);
			flash_state = flash_state&0x04;
			pr_err("isp_imx135_config(), CFG_GET_FLASH_STATE, flash_state(after) = %d \n", flash_state);

		  cdata.cfg.flash_state = flash_state;

			if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
			  rc = -EFAULT;
			break;
    //FIHLX_VKY, Fix VKY-9255, Oscar Lin, 2013/05/16 }

//FIHTDC@20130530 Rocky add more EXIF info. {
	case CFG_GET_EXPOSURE_BIAS:
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7283, &exposureBias, MSM_CAMERA_I2C_BYTE_DATA);
		//SW4-Roger-Camera-remove_log-00-_20130705
		//pr_err("isp_imx135_config(), CFG_GET_EXPOSURE_BIAS, exposureBias = %d \n", exposureBias);
		cdata.cfg.exposureBias = exposureBias;
		if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
		 rc = -EFAULT;
		break;

	case CFG_GET_METERINGMODE:
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x728E, &MeteringMode, MSM_CAMERA_I2C_BYTE_DATA);
		//SW4-Roger-Camera-remove_log-00-_20130705
		//pr_err("isp_imx135_config(), CFG_GET_METERINGMODE, MeteringMode = %d \n", MeteringMode);
		cdata.cfg.MeteringMode = MeteringMode;
		if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
		 rc = -EFAULT;
		break;

	case CFG_GET_WHITEBALANCE:
		msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x728A, &WhiteBalance, MSM_CAMERA_I2C_BYTE_DATA);
		//SW4-Roger-Camera-remove_log-00-_20130705
		//pr_err("isp_imx135_config(), CFG_GET_WHITEBALANCE, WhiteBalance = %d \n", WhiteBalance);
		cdata.cfg.WhiteBalance = WhiteBalance;
		if(copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
		 rc = -EFAULT;
		break;
//FIHTDC@20130530 Rocky add more EXIF info. }
		case CFG_SET_AEC_LOCK:
			pr_err("%s:%d calling CFG_SET_AEC_LOCK\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_aec_lock == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_AEC_LOCK mode(%d)\n",cdata.cfg.aec_lock);
			s_ctrl->func_tbl->sensor_set_isp_aec_lock(s_ctrl, cdata.cfg.aec_lock);
			break;
		case CFG_SET_AWB_LOCK:
			pr_err("%s:%d calling CFG_SET_AWB_LOCK\n", __func__, __LINE__);
			if (s_ctrl->func_tbl->sensor_set_isp_awb_lock == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_err("CFG_SET_AWB_LOCK mode(%d)\n",cdata.cfg.aec_lock);
			s_ctrl->func_tbl->sensor_set_isp_awb_lock(s_ctrl, cdata.cfg.aec_lock);
			break;
#endif

		default:
		pr_err("\n\n******************* [HL] %s, default *************************\n\n", __func__);

			//HL*{_20130909
			#if 0
			rc = -EFAULT;
			pr_err("\n\n******************* [HL] %s, rc = -EFAULT *************************\n\n", __func__);
			#else
			pr_err("\n\n******************* [HL] %s, NO rc = -EFAULT *************************\n\n", __func__);
			#endif
			//HL*}_20130909
			break;
		}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	pr_err("\n\n******************* [HL] %s ---, rc = %ld *************************\n\n", __func__, rc);

	return rc;
}

static int32_t isp_imx135_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t isp_imx135_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

int32_t isp_imx135_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	s_ctrl->stop_setting_valid = 0;
//	struct device *dev = NULL;
//	uint16_t status=0;
//	int32_t rc = 0;
	pr_err("[RK]%s:%d\n", __func__, __LINE__);

	//HL+_20130910
	//pr_err("\n\n******************* [HL] %s, NO POWER DOWN, return 0 *************************\n\n", __func__);
	//return 0;

	//SW5-Anvoi-camera firmware sometime upgrade fail. 20130613,+++
	//Retrun error to avoid camera sequence mix if process firmware upgrade running . camera will back to normal when upgrade task done.
	if(isp_is_upgrading == ICATCH_FW_UPGRADING)
		return -1;
	//SW5-Anvoi-camera firmware sometime upgrade fail. 20130613,---

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (main_cam_is_power_on == 0)
		return 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402

#if 0//SW4-Rocky-Camera-PortingCamera_20130816_00
    //SW5-Webber-Add_for_turn_off_led_flash_20130416
	pr_err("[RK]%s:%d turn_off_led_flash ++\n", __func__, __LINE__);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9008, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9009, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x900a, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x900b, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9238, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9240, 0xC6, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9200, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9210, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9211, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client, 0x9204, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(100);
	pr_err("[RK]%s:%d turn_off_led_flash --\n", __func__, __LINE__);
#endif
#if 0//SW4-Rocky-Camera-PortingCamera_20130816_00
	pr_err("[RK]%s:%d ++\n", __func__, __LINE__);
	msm_camera_cci_i2c_read(isp_imx135_s_ctrl.sensor_i2c_client, 0x7000, &status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: ****************************ISP 0x7000 status = %d \n", __func__,status);
	pr_err("[RK]%s:%d --\n", __func__, __LINE__);
#endif

	pr_err("[RK]%s:%d i2c_util ++\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}
	pr_err("[RK]%s:%d i2c_util --\n", __func__, __LINE__);

	CDBG("%s:%d E\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	for (index = (power_setting_array->size - 1); index >= 0; index--) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= ISP_MAIN_SENSOR_GPIO_MAX ||//SW4-Rocky-Camera-PortingCamera_20130816_00
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					ISP_MAIN_SENSOR_GPIO_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				continue;
			}
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {//SW4-Rocky-Camera-PortingCamera_20130816_00
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				continue;
			}
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				isp_imx135_disable_i2c_mux(data->i2c_conf);
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
//Rocky_+{20130925
			//usleep_range(power_setting->delay * 1000, (power_setting->delay * 1000) + 1000);//orig
			pr_err("[RK]%s mdelay(%d)\n", __func__, power_setting->delay);
			mdelay(power_setting->delay*10);
//Rocky_+}20130925
		}
	}

#if ENABLE_ISP_INTERRUPT
	if(g_irq_requested)
	{
		isp_disable_interrupt();
		gpio_free(ISP_INTR_GPIO);
		free_irq(g_isp_irq,0);
		g_irq_requested = 0;
	}
#endif
	if(g_caf_sensor_enable){
		cancel_delayed_work_sync(&CAF_work);
		isp_af_close();
		g_caf_sensor_enable=0;
	}
#if 1
	msm_camera_request_gpio_table(
	data->gpio_conf->cam_gpio_req_tbl,
	data->gpio_conf->cam_gpio_req_tbl_size, 0);
#endif
	pr_err("[RK]%s:%d X\n", __func__, __LINE__);

	//kfree(s_ctrl->reg_ptr);
	//s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
	g_pre_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	main_cam_is_power_on = 0;
	return 0;
}


int32_t  isp_imx135_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;

	int testINT = 0;//SW4-Rocky-Camera-PortingCamera_20130816_00

	s_ctrl->stop_setting_valid = 0;

	pr_err("\n\n[RK]%s E %d\n\n", __func__, index);

	if(isp_is_upgrading == ICATCH_FW_UPGRADING)
	{
		pr_err("\n\n******************* [HL] %s, isp_is_upgrading == ICATCH_FW_UPGRADING *************************\n\n", __func__);
		return -1;
	}

	if(isp_is_power_on == 1)
	{
		pr_err("\n\n******************* [HL] %s, isp_is_power_on == 1 *************************\n\n", __func__);
		return 0;
	}

	if (main_cam_is_power_on == 1)
	{
		pr_err("\n\n******************* [HL] %s, main_cam_is_power_on == 1 *************************\n\n", __func__);
		return 0;
	}
	power_setting_array = &s_ctrl->power_setting_array;

	if (data->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			data->gpio_conf->cam_gpiomux_conf_tbl,
			data->gpio_conf->cam_gpiomux_conf_tbl_size);
	}
	//HL+_20130910
	else
	{
		pr_err("\n\n******************* [HL] %s, data->gpio_conf->cam_gpiomux_conf_tbl == NULL, no msm_gpiomux_install() *************************\n\n", __func__);
	}
	//HL+_20130910

	//Fixme: To avoid gpio request fail that need free gpio first
	msm_camera_request_gpio_table(
	data->gpio_conf->cam_gpio_req_tbl,
	data->gpio_conf->cam_gpio_req_tbl_size, 0);

	rc = msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
	//	return rc; //Temp
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
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
//Rocky add for test{
#if 1
		pr_err("[RK]%s index %d\n", __func__, index);
		if(power_setting->seq_type == SENSOR_CLK)
			pr_err("[RK]%s type %d(SENSOR_CLK)\n", __func__, power_setting->seq_type);
		else if(power_setting->seq_type == SENSOR_GPIO)
			pr_err("[RK]%s type %d(SENSOR_GPIO)\n", __func__, power_setting->seq_type);
		else if(power_setting->seq_type == SENSOR_VREG)
			pr_err("[RK]%s type %d(SENSOR_VREG)\n", __func__, power_setting->seq_type);
		else if(power_setting->seq_type == SENSOR_I2C_MUX)
			pr_err("[RK]%s type %d(SENSOR_I2C_MUX)\n", __func__, power_setting->seq_type);
		else
			pr_err("[RK]%s type %d\n", __func__, power_setting->seq_val);
#endif
//Rocky add for test}

		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			if (power_setting->seq_val >= s_ctrl->clk_info_size) {
				pr_err("%s clk index %d >= max %d\n", __func__,
					power_setting->seq_val,
					s_ctrl->clk_info_size);
				goto power_up_failed;
			}
//Rocky add for test{
#if 1
			if(power_setting->seq_val == SENSOR_CAM_MCLK)
				pr_err("[RK]%s seq_val %d(SENSOR_CAM_MCLK)\n", __func__, power_setting->seq_val);
			else if(power_setting->seq_val == SENSOR_CAM_CLK)
				pr_err("[RK]%s seq_val %d(SENSOR_CAM_CLK)\n", __func__, power_setting->seq_val);
			else if(power_setting->seq_val == SENSOR_CAM_CLK_MAX)
				pr_err("[RK]%s seq_val %d(SENSOR_CAM_CLK_MAX)\n", __func__, power_setting->seq_val);
			else
				pr_err("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);

			pr_err("[RK]%s config_val %ld\n", __func__, power_setting->config_val);
#endif
//Rocky add for test}

			if (power_setting->config_val)
				s_ctrl->clk_info[power_setting->seq_val].clk_rate = power_setting->config_val;
			//SW4-Rocky-Camera-PortingCamera_20130816_00_{
			for(testINT = 0; testINT<s_ctrl->clk_info_size; testINT++)
				pr_err("[RK]%s [%d] name:%s, rate:%ld\n", __func__, testINT, s_ctrl->clk_info[testINT].clk_name, s_ctrl->clk_info[testINT].clk_rate);
			//SW4-Rocky-Camera-PortingCamera_20130816_00_}

			rc = msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				1);
			if (rc < 0) {
				pr_err("%s: clk enable failed\n",
					__func__);
				goto power_up_failed;
			}
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= ISP_MAIN_SENSOR_GPIO_MAX ||
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					ISP_MAIN_SENSOR_GPIO_MAX);
				goto power_up_failed;
			}
//Rocky add for test{
#if 1
				if(power_setting->seq_val == ISP_SUSPEND)
					pr_err("[RK]%s seq_val %d(ISP_SUSPEND)\n", __func__, power_setting->seq_val);
				else if(power_setting->seq_val == ISP_RESET_N)
					pr_err("[RK]%s seq_val %d(ISP_RESET_N)\n", __func__, power_setting->seq_val);
				else if(power_setting->seq_val == ISP_INTR)
					pr_err("[RK]%s seq_val %d(ISP_INTR)\n", __func__, power_setting->seq_val);
				else if(power_setting->seq_val == ISP_AVDD_EN)
					pr_err("[RK]%s seq_val %d(ISP_AVDD_EN)\n", __func__, power_setting->seq_val);
				else if(power_setting->seq_val == ISP_DVDD_EN)
					pr_err("[RK]%s seq_val %d(ISP_DVDD_EN)\n", __func__, power_setting->seq_val);
				else if(power_setting->seq_val == ISP_VCORE_EN)
					pr_err("[RK]%s seq_val %d(ISP_VCORE_EN)\n", __func__, power_setting->seq_val);
			else if(power_setting->seq_val == SENSOR_GPIO_VDIG)
				pr_err("[RK]%s seq_val %d(SENSOR_GPIO_VDIG)\n", __func__, power_setting->seq_val);
			else if(power_setting->seq_val == SENSOR_GPIO_VANA)
				pr_err("[RK]%s seq_val %d(SENSOR_GPIO_VANA)\n", __func__, power_setting->seq_val);
				else
					pr_err("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);

				pr_err("[RK]%s:%d gpio set val %d\n", __func__, __LINE__, data->gpio_conf->gpio_num_info->gpio_num[power_setting->seq_val]);
				if(power_setting->config_val == GPIO_OUT_LOW)
					pr_err("[RK]%s config_val %ld(GPIO_OUT_LOW)\n", __func__, power_setting->config_val);
				else if(power_setting->config_val == GPIO_OUT_HIGH)
					pr_err("[RK]%s config_val %ld(GPIO_OUT_HIGH)\n", __func__, power_setting->config_val);
				else
					pr_err("[RK]%s config_val %ld\n", __func__, power_setting->config_val);
#endif
//Rocky add for test}

				pr_debug("%s:%d gpio set val %d\n", __func__, __LINE__,
					data->gpio_conf->gpio_num_info->gpio_num
					[power_setting->seq_val]);
				gpio_set_value_cansleep(
					data->gpio_conf->gpio_num_info->gpio_num
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
			if(power_setting->seq_val == CAM_VDIG)
				pr_err("[RK]%s seq_val %d(CAM_VDIG)\n", __func__, power_setting->seq_val);
			else
				pr_err("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);
#endif
//Rocky add for test}

			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				1);
			break;
		case SENSOR_I2C_MUX:
			pr_err("[RK]%s:%d SENSOR_I2C_MUX ++++\n", __func__, __LINE__);
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
			{
				pr_err("[RK]%s:%d Do isp_imx135_sensor_enable_i2c_mux!!\n", __func__, __LINE__);
				isp_imx135_enable_i2c_mux(data->i2c_conf);
			}
			pr_err("[RK]%s:%d SENSOR_I2C_MUX ----\n", __func__, __LINE__);

			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
//Rocky_+{20130925
			//usleep_range(power_setting->delay * 1000, (power_setting->delay * 1000) + 1000);//orig
			pr_err("[RK]%s mdelay(%d)\n", __func__, power_setting->delay);
			mdelay(power_setting->delay*10);
//Rocky_+}20130925
		}
	}

	pr_err("[RK]%s:%d i2c_util ++++\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		pr_err("[RK]%s:%d i2c_util [IN]\n", __func__, __LINE__);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
			goto power_up_failed;
		}
	}
	pr_err("[RK]%s:%d i2c_util ----\n", __func__, __LINE__);

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0) {
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
		goto power_up_failed;
	}

	//s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
	g_pre_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 1;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	main_cam_is_power_on = 1;
	isp_imx135_inti_parms();
	//Enable Flash i2c write/read
	gpio_direction_output(92,0);
	mdelay(50);
	gpio_direction_output(92,1);

	//set flicker mode to 50Hz for ftm
	msm_camera_cci_i2c_write(isp_imx135_s_ctrl.sensor_i2c_client,
		0x7101, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;

power_up_failed:
//pr_err("[RK]%s msm_sensor_power_down pass!!\n", __func__);
//return 0;

	pr_err("[RK]%s DO msm_sensor_power_down!!\n", __func__);
	pr_err("%s:%d failed\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index--; index >= 0; index--) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				isp_imx135_disable_i2c_mux(data->i2c_conf);
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
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 0);
	//kfree(s_ctrl->reg_ptr);
	return rc;
}

int32_t isp_imx135_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	int retval = 0;
	int count = 0;

	rc = msm_camera_cci_i2c_write(
		isp_imx135_s_ctrl.sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,0x88,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_write id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		pr_err("BBox::UEC; 9::1\n");
		return rc;
	}

	rc = msm_camera_cci_i2c_read(
		isp_imx135_s_ctrl.sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr, &chipid,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_read id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		pr_err("BBox::UEC; 9::1\n");
		return rc;
	}

	pr_err("%s: expected id : %x, read id : %x\n", __func__, s_ctrl->sensordata->slave_info->sensor_id, chipid);

    #if 0
        if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
                pr_err("msm_sensor_match_id chip id doesnot match\n");
                pr_err("BBox::UEC; 9::2\n");
                return -ENODEV;
        }
        #else
        if (chipid != s_ctrl->sensordata->slave_info->sensor_id)
        {
		pr_err("\n\n******************* [HL] %s, Match ID fail, START of Retry *************************\n\n", __func__);
                do
                {
                        mdelay(50);
                        rc = msm_camera_cci_i2c_read(
                                isp_imx135_s_ctrl.sensor_i2c_client,
                                s_ctrl->sensordata->slave_info->sensor_id_reg_addr, &chipid,
                                MSM_CAMERA_I2C_BYTE_DATA);
			if (rc < 0)
			{
				pr_err("%s: %s: msm_camera_cci_i2c_read id 0x88 failed\n", __func__,
				s_ctrl->sensordata->sensor_name);
				pr_err("BBox::UEC; 9::1\n");
				return -ENODEV;
			}
			pr_err("[RL]%s: expected id : %x, read id : %x, count = %d\n", __func__, s_ctrl->sensordata->slave_info->sensor_id, chipid, count);

			count ++;
                } while((chipid != s_ctrl->sensordata->slave_info->sensor_id) && (count < 30));

		if (chipid != s_ctrl->sensordata->slave_info->sensor_id)
		{
			pr_err("\n\n******************* [HL] %s, Match ID fail, End of Retry *************************\n\n", __func__);
			chipid = 0x88;
			pr_err("[RL]%s: hardcode_chipid = 0x88\n", __func__);
		}
        }
        #endif
        //RL*}_20130926

	//create sysfs
	if(example_kobj == NULL)
	{
		example_kobj = kobject_create_and_add("isp_control", kernel_kobj);
		retval = sysfs_create_group(example_kobj, &isp_attr_group);
		if (retval)
			kobject_put(example_kobj);
		//sysfs_create_group(&isp_imx135_sensor_i2c_client.client.dev->dev.kobj, &isp_attr_group);
	}
	return rc;
}


#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct v4l2_subdev_core_ops isp_imx135_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops isp_imx135_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops isp_imx135_subdev_ops = {
	.core = &isp_imx135_subdev_core_ops,
	.video  = &isp_imx135_subdev_video_ops,
};
#endif
static struct msm_sensor_fn_t isp_imx135_func_tbl = {
	.sensor_start_stream = isp_imx135_start_stream,
	.sensor_stop_stream = isp_imx135_stop_stream,
	//.sensor_setting = isp_imx135_setting,
	//.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	//.sensor_mode_init = msm_sensor_mode_init,
	//.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = isp_imx135_config,
//	.sensor_config = msm_sensor_config,
	.sensor_power_up = isp_imx135_power_up,
	.sensor_power_down = isp_imx135_power_down,
	//.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id = isp_imx135_match_id,
//Anvoi
	.sensor_isp_af_start = iCatch_start_AF,
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
	.sensor_set_isp_caf_mode =  iCatch_set_caf_mode,
	.sensor_set_isp_scene_mode =  iCatch_set_scene_mode,
	.sensor_set_isp_effect_mode = iCatch_set_effect_mode,
	//SW4-L1-HL-Camera-ImplementExposureCompensation-00+_20130227
	.sensor_set_isp_exposure_compensation = iCatch_set_exposure_compensation,
	//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227
	.sensor_set_isp_aec_mode = iCatch_set_aec_mode,
	//SW4-L1-HL-Camera-ImplementISO-00+_20130304
	.sensor_set_iso = iCatch_set_iso,
	//SW4-L1-HL-Camera-ImplementWhiteBalance-00+_20130304
	.sensor_set_isp_wb = iCatch_set_wb,
#endif

	//SW4-L1-HL-Camera-ImplementSaturation-00+_20130305
	.sensor_set_isp_saturation = iCatch_set_saturation,
	//SW4-L1-HL-Camera-ImplementSharpness-00+_20130305
	.sensor_set_isp_sharpness = iCatch_set_sharpness,
	//SW4-L1-HL-Camera-ImplementContrast-00+_20130305
	.sensor_set_isp_contrast = iCatch_set_contrast,
	.sensor_set_isp_led_flash_mode=iCatch_set_led_flash_mode,
	.sensor_set_isp_antibanding = iCatch_set_antibanding,
	.sensor_set_isp_scene_mode =  iCatch_set_scene_mode,
	.sensor_set_isp_effect_mode = iCatch_set_effect_mode,
	.sensor_set_isp_wb = iCatch_set_wb,
	.sensor_set_isp_iso = iCatch_set_iso,
	.sensor_set_isp_contrast = iCatch_set_contrast,
#if 0
	 //SW5-Webber-Camera-ImplementLedFlash-20130313-end
	  //SW5-Marx-Camera-ImplementHDR-20130318-start
	 .sensor_set_isp_HDR_mode=iCatch_set_HDR_mode,
	 //SW5-Marx-Camera-ImplementHDR-20130318-end
    //SW5-Webber-Camera-ImplementCancelAF-20130331-begin
    .sensor_set_cancel_af = iCatch_set_cancel_af,
    //SW5-Webber-Camera-ImplementCancelAF-20130331-end
    .sensor_set_isp_aec_lock = iCatch_set_aec_lock,
    .sensor_set_isp_awb_lock = iCatch_set_awb_lock,
    #endif
};
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_sensor_reg_t isp_imx135_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = isp_sensor_config_change_settings,
	.start_stream_conf_size = ARRAY_SIZE(isp_sensor_config_change_settings),
	.init_settings = &isp_imx135_init_conf[0],
	.init_size = ARRAY_SIZE(isp_imx135_init_conf),
	//.mode_settings = &isp_imx135_confs[0],
	.output_settings = &isp_imx135_dimensions[0],
	.num_conf = ARRAY_SIZE(isp_imx135_dimensions),
};
#endif
static struct msm_sensor_ctrl_t isp_imx135_s_ctrl = {
	//.msm_sensor_reg = &isp_imx135_regs,
	//.msm_sensor_v4l2_ctrl_info = isp_imx135_v4l2_ctrl_info,
	//.num_v4l2_ctrl = ARRAY_SIZE(isp_imx135_v4l2_ctrl_info),
	.sensor_i2c_client = &isp_imx135_sensor_i2c_client,
	//.sensor_i2c_addr = 0x78,
	.power_setting_array.power_setting = isp_imx135_power_setting,
	.power_setting_array.size = ARRAY_SIZE(isp_imx135_power_setting),
//#if ENABLE_SENSOR_REGULATOR
//	.vreg_seq = isp_imx135_veg_seq,
//	.num_vreg_seq = ARRAY_SIZE(isp_imx135_veg_seq),
//#endif
//	.sensor_output_reg_addr = &isp_imx135_reg_addr,
//	.sensor_id_info = &isp_imx135_id_info,
//	.cam_mode = MSM_SENSOR_MODE_INVALID,
//	.min_delay = 30,
//	.power_seq_delay = 0,
	.msm_sensor_mutex = &isp_imx135_mut,
	//.sensor_i2c_driver = &isp_imx135_i2c_driver,
	.sensor_v4l2_subdev_info = isp_imx135_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(isp_imx135_subdev_info),
	//.sensor_v4l2_subdev_ops = &isp_imx135_subdev_ops,
	.func_tbl = &isp_imx135_func_tbl,
	//.clk_rate = MSM_SENSOR_MCLK_12HZ,  /*1041 port*/
};

static void __exit isp_imx135_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (isp_imx135_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&isp_imx135_s_ctrl);
		platform_driver_unregister(&isp_imx135_platform_driver);
	} else
		i2c_del_driver(&isp_imx135_i2c_driver);
	return;
}

module_init(isp_imx135_init_module);
module_exit(isp_imx135_exit_module);

MODULE_DESCRIPTION("Aptina 1.26MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
