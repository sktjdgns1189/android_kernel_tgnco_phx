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

#include <mach/gpiomux.h>//SW4-Rocky-Camera-PortingCamera_20130719_00
#include <media/msm_cam_sensor.h>//SW4-Rocky-Camera-PortingCamera_20130719_00

#define SENSOR_NAME "isp_front_cam"
#define PLATFORM_DRIVER_NAME "msm_camera_isp_front_cam"
#define isp_front_cam_obj isp_front_cam_##obj

#define ENABLE_ISP_front_cam_INTERRUPT 1
#define ENABLE_ISP_FIRMWARE_UPGRADE 1
//#define ENABLE_ISP_front_cam_GPIO_DEBUG 1
//#define DEBUG_ISP_front_cam_GPIO 36
//#define ENABLE_ISP_front_cam_REG_DEBUG 1	//Orig - marked	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*_20131101
//#define ENABLE_ISP_front_cam_CMDQ_DEBUG 1
//#define ENABLE_SENSOR_REGULATOR 1

static unsigned int preview_resolution=MAIN_RES_1040x780;//fihtdc,derekcwwu, add for tx3
static unsigned int snapshot_resolution=MAIN_RES_3264x2448;	//RL*_20140710
static unsigned int hd_resolution=MAIN_RES_1280x720; //ChunJengAdd

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

extern struct device *dev_uevent;//fihtdc,derekcww,add for front cam when front cam can focus

//SW4-HL-Camera-FixCameraSwitchFailIssue-00*_20130926
extern int isp_is_power_on;
//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
//SW4-HL-Camera-FixCameraSwitchFailIssue-00*_20130926
extern int front_cam_is_power_on;//Orig - static int front_cam_is_power_on = 0;
//SW4-HL-Camera-ErrorHandling-00*{_20140410
static int isp_front_cam_mode;
static int g_pre_front_res = MSM_SENSOR_INVALID_RES;
static int giCatchStreamOff = 0;
//SW4-HL-Camera-ErrorHandling-00*}_20140410

int isp_front_cam_recover_isp(struct msm_sensor_ctrl_t *s_ctrl);
int isp_front_cam_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int isp_front_cam_power_down(struct msm_sensor_ctrl_t *s_ctrl);

extern int isp_ois_on;	//SW4-RL-Camera-implementOIS-00+_20140430
extern char camera_firmware_name[30];//fihtdc,derekcwwu, add for fix vna warning log
#if ENABLE_ISP_front_cam_INTERRUPT
static int g_time_start = 0;
extern struct completion g_iCatch_comp;//fihtdc,derekcww,add for front cam when front cam can focus
extern int interrup_error;//fihtdc,derekcww,add for front cam when front cam can focus
bool g_isp_front_cam_first_open = true;
#endif
DEFINE_MUTEX(isp_front_cam_mut);
static struct msm_sensor_ctrl_t isp_front_cam_s_ctrl;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

extern int32_t isp_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);
extern int32_t isp_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);
extern void isp_main_cam_inti_parms(void);

static struct v4l2_subdev_info isp_front_cam_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

int isp_front_cam_wait_for_next_frame(struct msm_sensor_ctrl_t *s_ctrl)
{
	u32 timeout_count = 1;
	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
	#endif

	pr_debug("%s: E\n",__func__);
	g_time_start = jiffies;
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 3*HZ);
	pr_debug("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));
	if ((!timeout_count))
	{
		pr_err("%s: interrupt timedout or ISP error, [wait_time] %d\n", __func__, jiffies_to_msecs(jiffies-g_time_start));
		pr_err("BBox::UEC; 9::4\n");
		return -1;
	}
	else
	{
		pr_debug("%s interrupt done\n",__func__);

		#ifdef ENABLE_ISP_front_cam_GPIO_DEBUG
		//gpio_direction_output(DEBUG_ISP_front_cam_GPIO,1);
		mdelay(5);
		gpio_direction_output(DEBUG_ISP_front_cam_GPIO,0);
		pr_debug("%s: DEBUG_ISP_front_cam_GPIO: %d\n", __func__, gpio_get_value(DEBUG_ISP_front_cam_GPIO));
		#endif

		#ifdef ENABLE_ISP_front_cam_REG_DEBUG
		pr_debug("=======%s START ===== \n",__func__);
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
		#endif

		//if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72F8, 0x04, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

		#ifdef ENABLE_ISP_front_cam_REG_DEBUG
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);

		pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
		pr_debug("======%s  END ====== \n",__func__);
		#endif
	}

	pr_debug("%s: X\n",__func__);

	return 0;
}
//SW4-HL-Camera-ErrorHandling-00*}_20140410

/* sensor switching */
int ISPI2C_SensorSelectSet_EVT(struct msm_sensor_ctrl_t *s_ctrl, unsigned char fwNumber)
{
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1011, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x001C, 0x08, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x001C, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1010, 0x02, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1010, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1306, fwNumber, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x1011, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	return 0;
}

//SW4-HL-Camera-ErrorHandling-00*{_20140410
int isp_front_cam_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_debug("%s: E\n", __func__);

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*{_20131101
	if (giCatchStreamOff == 0)
	{
		if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("%s, SEND STREAM OFF command to 7002A\n", __func__);
		giCatchStreamOff = 1;
	}
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*}_20131101

	pr_debug("%s: E\n", __func__);

	return 0;
}
//SW4-HL-Camera-ErrorHandling-00*}_20140410

int32_t isp_front_cam_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	int count = 0;

	rc = msm_camera_cci_i2c_write(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr, 0x88,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_write id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}

	pr_debug("[RL]%s: msm_camera_cci_i2c_write() done!\n", __func__);

	rc = msm_camera_cci_i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr, &chipid,
			MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_read id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}

	pr_debug("[RL]%s: msm_camera_cci_i2c_read() done!\n", __func__);

	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	pr_debug("%s: expected id : %x, read id : %x\n", __func__, s_ctrl->sensordata->slave_info->sensor_id, chipid);

        #if 0
	pr_err("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
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

	return rc;
}

int32_t isp_front_cam_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc=0;

	pr_debug("%s: res=%d\n", __func__, res);
	switch(res)
	{
		//SW4-HL-Camera-FixRecordingFailWhenResolutionIsHD-00*{_20140317
		case MSM_SENSOR_RES_FULL:
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_FULL *************************\n\n", __func__);
			isp_front_cam_mode = MSM_SENSOR_RES_FULL;
		break;

		case MSM_SENSOR_RES_QTR:
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_QTR *************************\n\n", __func__);
			isp_front_cam_mode = MSM_SENSOR_RES_QTR;
		break;

		case MSM_SENSOR_RES_FULL_PREVIEW:
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_FULL_PREVIEW *************************\n\n", __func__);
			isp_front_cam_mode = MSM_SENSOR_RES_FULL_PREVIEW;
		break;
		//SW4-L1-HL-Camera-FixPreviewFailAfterEnablingZSL-VKY-6805-00*}_20130423

		case MSM_SENSOR_RES_HD:
			pr_debug("\n\n******************* [HL] %s, MSM_SENSOR_RES_HD *************************\n\n", __func__);
			isp_front_cam_mode = MSM_SENSOR_RES_HD;
		break;
		//SW4-HL-Camera-FixRecordingFailWhenResolutionIsHD-00*}_20140317

		//SW4-RL-Camera-addForFHDRecording-00+{_20140423
		case MSM_SENSOR_RES_FHD:	//res = 4
			pr_debug("\n\n******************* [RL] %s, MSM_SENSOR_RES_FHD *************************\n\n", __func__);
			isp_front_cam_mode = MSM_SENSOR_RES_FHD;
		break;
		//SW4-RL-Camera-addForFHDRecording-00+}_20140423

		default:
			pr_err("%s: Do not support this res=%d\n", __func__, res);
		break;
	}

	return rc;
}

int wait_for_evt_front_AE_ready(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t AE_ready=0;
	int count=0;

	g_isp_front_cam_first_open = false;

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

int32_t isp_front_cam_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	pr_debug("\n\n******************* [HL] %s +++ *************************\n\n", __func__);
	pr_debug("%s: update_type=%d, res=%d\n", __func__, update_type, res);

	if (update_type == MSM_SENSOR_REG_INIT)
	{
		g_isp_front_cam_first_open = true;
		//msm_sensor_write_init_settings(s_ctrl);
	}
	else if (update_type == MSM_SENSOR_UPDATE_PERIODIC)
	{
		isp_front_cam_write_res_settings(s_ctrl, res);
	}

	pr_debug("\n\n******************* [HL] %s ---, rc = %d *************************\n\n", __func__, rc);

	return rc;
}

int isp_front_cam_recover_isp(struct msm_sensor_ctrl_t *s_ctrl)
{
	//struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct device *dev = NULL;
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*{_20131101
	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
	#endif
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00*}_20131101
	u32 timeout_count = 1;

	pr_debug("%s: E\n", __func__);

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE)//MSM_SENSOR_PLATFORM_DEVICE
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;

	if (isp_front_cam_power_down(s_ctrl) < 0)
	{
		pr_err("%s: power down failed!\n", __func__);
		return -1;
	}

	if (isp_front_cam_power_up(s_ctrl) < 0)
	{
		pr_err("%s: power up failed!\n", __func__);
		return -1;
	}

	/* 0: Main Cam, 1: Front Cam*/
	pr_debug("[RK]%s ISPI2C_SensorSelectSet_EVT!\n",__func__);
	if (ISPI2C_SensorSelectSet_EVT(s_ctrl, 1) < 0)	//after firmware 03.00.04 use "1"
	{
		pr_err("%s: ISPI2C_SensorSelectSet_EVT  failed!\n", __func__);
		return -1;
	}

	//switch mode**********************************************
	pr_debug("%s: switch mode****************\n", __func__);
	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	pr_debug("=======%s [1] START ===== \n",__func__);
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
	#endif

	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_debug("======%s [1] END ====== \n",__func__);
	#endif

	mdelay(10);

	switch(isp_front_cam_mode)
	{
		case MSM_SENSOR_RES_QTR:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, preview_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_QTR Clean 0x72F8 \n", __func__);
		break;
		case MSM_SENSOR_RES_FULL:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, snapshot_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, imx132 has 2M resolution
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FULL Clean 0x72F8 \n", __func__);
		break;
		case MSM_SENSOR_RES_FULL_PREVIEW:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, snapshot_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, imx132 has 2M resolution
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
		break;
		case MSM_SENSOR_RES_HD:
		//SW4-RL-Camera-modify_HD_RecordingResolution-00*_20140423
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, hd_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, imx132 has 2M resolution
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_HD Clean 0x72F8 \n", __func__);
		break;
		//SW4-RL-Camera-addFor_FHD_RecordingResolution-00+{_20140423
		case MSM_SENSOR_RES_FHD:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FHD Clean 0x72F8 \n", __func__);
		break;
		//SW4-RL-Camera-addFor_FHD_RecordingResolution-00+}_20140423
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_front_cam_mode);
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

		#ifdef ENABLE_ISP_front_cam_REG_DEBUG
		pr_debug("=======%s START ===== \n",__func__);
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
		#endif

		//if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72F8, 0x04, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

		#ifdef ENABLE_ISP_front_cam_REG_DEBUG
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
		pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
		pr_debug("======%s  END ====== \n",__func__);
		#endif
	}

	pr_debug("%s: X\n", __func__);

	return 0;
}

static const struct i2c_device_id isp_front_cam_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&isp_front_cam_s_ctrl},
	{ }
};

static int32_t isp_front_cam_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &isp_front_cam_s_ctrl);
}

static struct i2c_driver isp_front_cam_i2c_driver = {
	.id_table = isp_front_cam_i2c_id,
	.probe  = isp_front_cam_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client isp_front_cam_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id isp_front_cam_dt_match[] = {
	{.compatible = "qcom,isp_front_cam", .data = &isp_front_cam_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, isp_front_cam_dt_match);

static struct platform_driver isp_front_cam_platform_driver = {
	.driver = {
		.name = "qcom,isp_front_cam",
		.owner = THIS_MODULE,
		.of_match_table = isp_front_cam_dt_match,
	},
};
static int32_t isp_front_cam_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	int i=0;//derek
	const char *ret_data=NULL,*ret_data2=NULL;//derek
	struct device_node *src_node = NULL;//fihtdc,derekcww,add for front cam when front cam can focus

	pr_debug("%s:%d++++of_match_device++++\n", __func__, __LINE__);
	match = of_match_device(isp_front_cam_dt_match, &pdev->dev);
	pr_debug("%s:%d----of_match_device----\n", __func__, __LINE__);

	pr_debug("%s:%d++++power setting++++\n", __func__, __LINE__);
	ret_data = of_get_property(pdev->dev.of_node, "qcom,sensor-name", NULL);
	pr_err("camera sensor name=%s",ret_data);
	ret_data = of_get_property(pdev->dev.of_node, "firmware_name", NULL);
	//strcpy(camera_firmware_name,ret_data);
	//pr_err("camera firmware name=%s",camera_firmware_name);
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
			//fihtdc,derekcww,add for front cam when front cam can focus
			src_node = of_parse_phandle(pdev->dev.of_node, "qcom,actuator-src", 0);
			if (src_node) {
				pr_err("front cam can focus\n");
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_isp_af_start = iCatch_start_AF;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_set_isp_af_mode = iCatch_set_AF_Mode;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_set_isp_af_roi = iCatch_set_AF_ROI;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_set_isp_HDR_mode=iCatch_set_HDR_mode;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_set_isp_sensor_data = iCatch_set_sensor_data;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_start_stream = isp_main_cam_start_stream;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_stop_stream = isp_main_cam_stop_stream;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_power_up = isp_main_cam_power_up;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_power_down = isp_main_cam_power_down;
				((struct msm_sensor_ctrl_t *)(match->data))->func_tbl->sensor_setting = isp_main_cam_setting;
			}
			//fihtdc,derekcww,add for front cam when front cam can focus
			break;
		}
	}
	pr_debug("%s:%d----power setting----\n", __func__, __LINE__);

	pr_debug("%s:%d++++msm_sensor_platform_probe++++\n", __func__, __LINE__);
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_debug("%s:%d----msm_sensor_platform_probe----\n", __func__, __LINE__);

	return rc;
}

static int __init isp_front_cam_init_module(void)
{
	int32_t rc = 0;

	pr_debug("[RK]%s:%d++++\n", __func__, __LINE__);
	rc = platform_driver_probe(&isp_front_cam_platform_driver,
		isp_front_cam_platform_probe);
	pr_debug("[RK]%s:%d rc %d\n", __func__, __LINE__, rc);
	if (!rc)
		return rc;

	rc = i2c_add_driver(&isp_front_cam_i2c_driver);
	pr_debug("[RK]%s:%d(%d)----\n", __func__, __LINE__, rc);

	return rc;
}

int isp_front_cam_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
	#endif
	uint16_t ois_val=0;		//SW4-RL-Camera-implementOIS-00+_20140430

	pr_debug("%s: pre_res=%d, cur_res=%d\n", __func__, g_pre_front_res, isp_front_cam_mode);

	if(g_pre_front_res == isp_front_cam_mode)
	{
		pr_debug("%s: ignore mode change\n",__func__);
		if (giCatchStreamOff == 1)
		{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
			giCatchStreamOff = 0;
		}

		return rc;
	}
	else
	{
		pr_debug("\n\n*************** %s, Mode Changed!!! ***********************\n \n", __func__);
	}

	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	pr_debug("=======%s [1] START ===== \n",__func__);
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[before_clear] 0x0024 = 0x%x\n", gpio_status);
	#endif

        if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	if (msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
	pr_debug("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_debug("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_debug("======%s [1] END ====== \n",__func__);
	#endif

	pr_debug("%s delay 10 ms\n", __func__);
	mdelay(10);

//SW4-RL-Camera-implementOIS-00+{_20140430
	pr_err("%s camera_firmware_name = %s \n", __func__, camera_firmware_name);
	if(!strncmp (camera_firmware_name, "MO7", strlen("MO7")))
	{
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

	switch(isp_front_cam_mode)
	{
		case MSM_SENSOR_RES_QTR:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, preview_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_QTR Clean 0x72F8 \n", __func__);
		break;
		case MSM_SENSOR_RES_FULL:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, snapshot_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, imx132 has 2M resolution
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FULL Clean 0x72F8 \n", __func__);
		break;
		case MSM_SENSOR_RES_FULL_PREVIEW:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, snapshot_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, imx132 has 2M resolution
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
		break;
		case MSM_SENSOR_RES_HD:
		//SW4-RL-Camera-modify_HD_RecordingResolution-00*_20140423
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, hd_resolution, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;//fihtdc,derekcwwu, imx132 has 2M resolution
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_HD Clean 0x72F8 \n", __func__);
		break;
		//SW4-RL-Camera-addForFHDRecording-00+{_20140423
		case MSM_SENSOR_RES_FHD:
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7106, MAIN_RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
			pr_debug("%s: waiting for MSM_SENSOR_RES_FHD Clean 0x72F8 \n", __func__);
		break;
		//SW4-RL-Camera-addForFHDRecording-00+}_20140423
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_front_cam_mode);
			goto isp_error;
		break;
	}

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+{_20131101
	if (giCatchStreamOff == 1)
	{
		if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
		giCatchStreamOff = 0;
	}
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+}_20131101

	rc = isp_front_cam_wait_for_next_frame(s_ctrl);
	if (rc < 0)
	{
		rc = isp_front_cam_recover_isp(s_ctrl);
		if (rc < 0)
		{
			goto isp_error;
		}
	}

	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+{_20131101
	if(true == g_isp_front_cam_first_open)
	{
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+}_20131101
		rc = wait_for_evt_front_AE_ready(s_ctrl);
		if (rc < 0)
		{
			goto isp_error;
		}
	//SW4-HL-Camera-Fix8MSnapshotNotWorkIssue-00+_20131101
	}

	//Set front camera AE center for black fase
	if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x710E, 0x01, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;

	//SW5-Webber-20150528-set FHD and HD to min.24fps due to optical's requirement++++
	if(strncmp (camera_firmware_name, "VNA",strlen("VNA"))) {
		//For PHX and VN2, we set FHD and HD to minimum 24fps
		if ((isp_front_cam_mode==MSM_SENSOR_RES_HD) || (isp_front_cam_mode==MSM_SENSOR_RES_FHD)) {
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7125, 24, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
		else{
			if (msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7125, 0, MSM_CAMERA_I2C_BYTE_DATA) < 0)	return -1;
		}
	}
	//SW5-Webber-20150528-set FHD and HD to min.24fps due to optical's requirement----

	#ifdef ENABLE_ISP_front_cam_REG_DEBUG
	DumpICatchRegister(s_ctrl);//add registers you need
	#endif

	g_pre_front_res = isp_front_cam_mode;

	return rc;

isp_error:
	DumpICatchRegister(s_ctrl);//add registers you need

	return rc;
}
static int32_t isp_front_cam_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t isp_front_cam_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

//SW4-HL-Camera-ErrorHandling-00*{_20140410
int isp_front_cam_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct msm_camera_power_ctrl_t *power_info = &data->power_info;
	struct msm_camera_gpio_conf *gpio_conf = power_info->gpio_conf;

	pr_debug("%s: E\n", __func__);

	s_ctrl->stop_setting_valid = 0;

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (front_cam_is_power_on == 0)
		return 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0) {
			pr_err("%s cci_release failed\n", __func__);
			return -1;
		}
	}

	//fihtdc, derekcwwu, move to this for correct sequence
	#if ENABLE_ISP_front_cam_INTERRUPT
	isp_deinit_interrupt();//fihtdc,derekcww,add for front cam when front cam can focus
	#endif
	//fihtdc, derekcwwu, move to this for correct sequence

	power_setting_array = &s_ctrl->power_setting_array;

	for (index = (power_setting_array->size - 1); index >= 0; index--) {
		pr_debug("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		pr_debug("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(power_info->dev,
				&power_info->clk_info[0],
				(struct clk **)&power_setting->data[0],
				power_info->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= ISP_MAIN_SENSOR_GPIO_MAX ||//SW4-Rocky-Camera-PortingCamera_20130816_00	//Rocky_20131023
				!gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					ISP_MAIN_SENSOR_GPIO_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00	//Rocky_20131023
				continue;
			}
			gpio_set_value_cansleep(
				gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {//SW4-Rocky-Camera-PortingCamera_20130816_00	//Rocky_20131023
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00	//Rocky_20131023
				continue;
			}
			msm_camera_config_single_vreg(power_info->dev,
				&power_info->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (power_info->i2c_conf && power_info->i2c_conf->use_i2c_mux)
				isp_front_cam_disable_i2c_mux(power_info->i2c_conf);
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

	msm_camera_request_gpio_table(
	gpio_conf->cam_gpio_req_tbl,
	gpio_conf->cam_gpio_req_tbl_size, 0);

	g_pre_front_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	front_cam_is_power_on = 0;

	pr_debug("%s: X\n", __func__);

	return 0;
}

int  isp_front_cam_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct msm_camera_power_ctrl_t *power_info = &data->power_info;
	struct msm_camera_gpio_conf *gpio_conf = power_info->gpio_conf;
	unsigned int intr_gpio=0;//fihtdc,derekcww,add for front cam when front cam can focus

	pr_debug("%s: E\n", __func__);

	s_ctrl->stop_setting_valid = 0;

	if(isp_is_power_on == 1)
		return 0;

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (front_cam_is_power_on == 1)
		return 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402

	power_setting_array = &s_ctrl->power_setting_array;

	if (gpio_conf->cam_gpiomux_conf_tbl != NULL)
	{
		pr_debug("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			gpio_conf->cam_gpiomux_conf_tbl,
			gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	//Fixme: for mo7, need to do this
	if(!strncmp(camera_firmware_name,"MO7",strlen("MO7")))
	{//fihtdc,derekcwwu, add for fix vna warning log
		msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 0);
	}//fihtdc,derekcwwu, add for fix vna warning log

	rc = msm_camera_request_gpio_table(
		gpio_conf->cam_gpio_req_tbl,
		gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}


	for (index = 0; index < power_setting_array->size; index++) {
		power_setting = &power_setting_array->power_setting[index];

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
			//Rocky add for test}

			if (power_setting->config_val)
				power_info->clk_info[power_setting->seq_val].
					clk_rate = power_setting->config_val;

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
			if (power_setting->seq_val >= ISP_MAIN_SENSOR_GPIO_MAX ||//SW4-Rocky-Camera-PortingCamera_20130816_00
				!gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					ISP_MAIN_SENSOR_GPIO_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00	//Rocky_20131023
				goto power_up_failed;
			}
			//Rocky add for test{
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
			if (power_setting->seq_val >= CAM_VREG_MAX) {//SW4-Rocky-Camera-PortingCamera_20130816_00
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				goto power_up_failed;
			}
			//Rocky add for test{
			if(power_setting->seq_val == EVT_SUB_CAM_VDIG)
			{
				pr_debug("[RK]%s seq_val %d(EVT_SUB_CAM_VDIG)\n", __func__, power_setting->seq_val);
			}
			else
			{
				pr_debug("[RK]%s seq_val %d\n", __func__, power_setting->seq_val);
			}
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
				pr_err("[RK]%s:%d Do isp_front_cam_sensor_enable_i2c_mux!!\n", __func__, __LINE__);
				isp_front_cam_enable_i2c_mux(power_info->i2c_conf);
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

	//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*{_20130926
	/* 0: Main Cam, 1: Front Cam*/
	if (ISPI2C_SensorSelectSet_EVT(s_ctrl, 1) < 0)	//after firmware 03.00.04 use "1"
	{
		pr_err("%s: ISPI2C_SensorSelectSet_EVT  failed!\n", __func__);
		return -1;
	}
	//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*}_20130926

	//SW4-HL-Camera-EnhanceKernelLog-VNA-3295-00*{_20140409
	intr_gpio = gpio_conf->cam_gpio_req_tbl[ISP_INTR].gpio;
	pr_debug("\n\n*** [HL] %s,  ISP_INTR_GPIO = %d ***\n\n", __func__, intr_gpio);
	//SW4-HL-Camera-EnhanceKernelLog-VNA-3295-00*}_20140409

	#if ENABLE_ISP_front_cam_INTERRUPT
	rc = isp_init_interrupt(intr_gpio);//fihtdc,derekcww,add for front cam when front cam can focus
	if(rc < 0)
	{
		pr_err("%s: isp_init_interrupt fail. \n", __func__);
		return rc;
	}
	#endif

	g_pre_front_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 1;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	front_cam_is_power_on = 1;
	isp_main_cam_inti_parms();

	pr_debug("%s: X\n", __func__);

	return rc;

power_up_failed:
	pr_debug("[RK]%s DO msm_sensor_power_down!!\n", __func__);
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
				isp_front_cam_disable_i2c_mux(power_info->i2c_conf);
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

	pr_debug("%s: X\n", __func__);

	return rc;
}
//SW4-HL-Camera-ErrorHandling-00*}_20140410

static struct msm_sensor_fn_t isp_front_cam_func_tbl = {
		.sensor_start_stream = isp_front_cam_start_stream,
		.sensor_stop_stream = isp_front_cam_stop_stream,
		.sensor_setting = isp_front_cam_setting,	//Orig - marked	//HL*_20130927
		//.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
		//.sensor_mode_init = msm_sensor_mode_init,
		//.sensor_get_output_info = msm_sensor_get_output_info,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00*{_20130311
		//Orig -- .sensor_config = msm_sensor_config,
		.sensor_config = isp_main_cam_config,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00*}_20130311
		.sensor_power_up = isp_front_cam_power_up,
		.sensor_power_down = isp_front_cam_power_down,
		//.sensor_get_csi_params = msm_sensor_get_csi_params,
		.sensor_match_id = isp_front_cam_match_id,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00+{_20130311
		.sensor_set_isp_scene_mode =  iCatch_set_scene_mode,
		.sensor_set_isp_effect_mode = iCatch_set_effect_mode,
		.sensor_set_isp_exposure_compensation = iCatch_set_exposure_compensation,
		.sensor_set_isp_aec_mode = iCatch_set_aec_mode,
		.sensor_set_iso = iCatch_set_iso,
		.sensor_set_isp_wb = iCatch_set_wb,
		.sensor_set_isp_saturation = iCatch_set_saturation,
		.sensor_set_isp_sharpness = iCatch_set_sharpness,
		.sensor_set_isp_contrast = iCatch_set_contrast,
		.sensor_set_isp_antibanding = iCatch_set_antibanding,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00+}_20130311
		.sensor_get_isp_exposure_time = iCatch_get_exposure_time,//SW4-RK-Camera-SetEXIFInformation-00+_20131225
		.sensor_get_isp_iso_value = iCatch_get_iso_value,//SW4-RK-Camera-SetEXIFInformation-00+_20131230
		.sensor_set_isp_wdr = iCatch_set_wdr_mode, //SW4-RL-Camera-WDR-00*_20140117
		.sensor_get_isp_3a_info = iCatch_get_3a_info,//SW4-RK-Camera-SetEXIF3AInformation-00+_20140218
		.sensor_set_isp_aec_roi = iCatch_set_AEC_ROI,//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+_20140328
		.sensor_set_isp_aec_lock = iCatch_set_aec_lock,		//SW4-RL-Camera-implementAELock-00+_20140710
		.sensor_set_isp_awb_lock = iCatch_set_awb_lock,	//SW4-RL-Camera-implementAWBLock-00+_20140710
};

static struct msm_sensor_ctrl_t isp_front_cam_s_ctrl = {
	//.msm_sensor_reg = &isp_front_cam_regs,
	//.msm_sensor_v4l2_ctrl_info = isp_front_cam_v4l2_ctrl_info,
	//.num_v4l2_ctrl = ARRAY_SIZE(isp_front_cam_v4l2_ctrl_info),
	.sensor_i2c_client = &isp_front_cam_sensor_i2c_client,
	//.sensor_i2c_addr = 0x78,
	.power_setting_array.power_setting = NULL,
	.power_setting_array.size = 0,
//#if ENABLE_SENSOR_REGULATOR
//	.vreg_seq = isp_front_cam_veg_seq,
//	.num_vreg_seq = ARRAY_SIZE(isp_front_cam_veg_seq),
//#endif
//	.sensor_output_reg_addr = &isp_front_cam_reg_addr,
//	.sensor_id_info = &isp_front_cam_id_info,
//	.cam_mode = MSM_SENSOR_MODE_INVALID,
//	.min_delay = 30,
//	.power_seq_delay = 0,
	.msm_sensor_mutex = &isp_front_cam_mut,
	//.sensor_i2c_driver = &isp_front_cam_i2c_driver,
	.sensor_v4l2_subdev_info = isp_front_cam_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(isp_front_cam_subdev_info),
	//.sensor_v4l2_subdev_ops = &isp_front_cam_subdev_ops,
	.func_tbl = &isp_front_cam_func_tbl,
	//.clk_rate = MSM_SENSOR_MCLK_12HZ,  /*1041 port*/
};

static void __exit isp_front_cam_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (isp_front_cam_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&isp_front_cam_s_ctrl);
		platform_driver_unregister(&isp_front_cam_platform_driver);
	} else
		i2c_del_driver(&isp_front_cam_i2c_driver);
	return;
}

module_init(isp_front_cam_init_module);
module_exit(isp_front_cam_exit_module);

MODULE_DESCRIPTION("isp_front_cam");
MODULE_LICENSE("GPL v2");
