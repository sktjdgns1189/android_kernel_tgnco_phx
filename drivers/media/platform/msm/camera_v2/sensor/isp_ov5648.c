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
#include "isp_imx135.h"

#include <mach/gpiomux.h>//SW4-Rocky-Camera-PortingCamera_20130719_00
#include <media/msm_cam_sensor.h>//SW4-Rocky-Camera-PortingCamera_20130719_00

#include "icatch7002a.h" //ICATCH function

#define SENSOR_NAME "isp_ov5648"
#define PLATFORM_DRIVER_NAME "msm_camera_isp_ov5648"
#define isp_ov5648_obj isp_ov5648_##obj

#define ENABLE_ISP_ov5648_INTERRUPT 1
#define ENABLE_ISP_FIRMWARE_UPGRADE 1
//#define ENABLE_ISP_ov5648_GPIO_DEBUG 1
//#define DEBUG_ISP_ov5648_GPIO 36
//#define ENABLE_ISP_ov5648_REG_DEBUG 1
//#define ENABLE_ISP_ov5648_CMDQ_DEBUG 1
//#define ENABLE_SENSOR_REGULATOR 1

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

//SW4-HL-Camera-FixCameraSwitchFailIssue-00*_20130926
int isp_ov5648_mode;	//Orig - static int isp_ov5648_mode;
extern int isp_is_power_on;
//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
//SW4-HL-Camera-FixCameraSwitchFailIssue-00*_20130926
int front_cam_is_power_on = 0;//Orig - static int front_cam_is_power_on = 0;
extern int giCatchStreamOff;

static int isp_ov5648_recover_isp(struct msm_sensor_ctrl_t *s_ctrl);
int32_t isp_ov5648_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int32_t isp_ov5648_power_down(struct msm_sensor_ctrl_t *s_ctrl);


#if ENABLE_ISP_ov5648_INTERRUPT
static unsigned int g_isp_irq;
static unsigned int ISP_INTR_GPIO = 102;
static int isp_ov5648_irq_requested = 0;
static int g_time_start = 0;
static struct completion g_iCatch_comp;
static bool g_isp_ov5648_first_open = true;
#endif
DEFINE_MUTEX(isp_ov5648_mut);
static struct msm_sensor_ctrl_t isp_ov5648_s_ctrl;
typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef unsigned char UINT8;

extern int32_t isp_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);
extern int32_t isp_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf);
//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*}_20130926
#if 0
extern void I2CDataWrite(UINT32 addr, UINT16 value);
extern void hsI2CDataWrite(UINT32 addr, UINT16 value);
extern UINT32 I2CDataRead(UINT32 addr);
extern UINT32 hsI2CDataRead(UINT32 addr);
#endif
//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*}_20130926

extern void isp_imx135_inti_parms(void);
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_camera_i2c_reg_conf isp_ov5648_recommend_settings[] = {

};
#endif

static struct v4l2_subdev_info isp_ov5648_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_camera_i2c_reg_conf isp_ov5648_config_change_settings[] = {

};
#endif

#if ENABLE_ISP_ov5648_INTERRUPT
void isp_ov5648_disable_interrupt(void)
{
	pr_err("%s:\n", __func__);
	disable_irq(g_isp_irq);
}

void isp_ov5648_enable_interrupt(void)
{
	//HL+_20130909
	 struct irq_desc *desc;

	pr_err("%s\n", __func__);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72FC, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
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

static irqreturn_t isp_ov5648_irq_handler(int irq, void *dev_id)
{
	int value1;

	pr_err("%s +\n", __func__);
	//WARN_ON(1);
#ifdef ENABLE_ISP_ov5648_GPIO_DEBUG
        gpio_direction_output(DEBUG_ISP_ov5648_GPIO,1);
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

int  isp_ov5648_isp_init_interrupt(void)
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

	ret = request_irq(g_isp_irq, isp_ov5648_irq_handler,
			 IRQF_TRIGGER_RISING, "fih_isp_ov5648_ISR", NULL);
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
void isp_ov5648_wait_for_next_frame(void){
	u32 timeout_count = 1;
	uint16_t status = 0x0;
#ifdef ENABLE_ISP_ov5648_CMDQ_DEBUG
	int i = 0;
#endif
#ifdef ENABLE_ISP_ov5648_REG_DEBUG
	uint16_t gpio_status = 0x0;
#endif
	pr_err("%s: E\n",__func__);
	g_time_start = jiffies;
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 3*HZ);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72f8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: 0x72f8 status: %d\n", __func__, status);
	pr_err("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));

	if (!timeout_count || status!=0x04) {
		pr_err("%s: interrupt timedout or ISP error\n", __func__);
		pr_err("BBox::UEC; 9::4\n");
#ifdef ENABLE_ISP_ov5648_CMDQ_DEBUG
		//dump command queue to debug, total 128 bytes
		pr_err("=======%s: Dump CmdQ START ===== \n",__func__);
		for(i = 0 ; i < 0x31; i++)
		{
			msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7200+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x7200+i, status);
		}
		pr_err("=======%s: Dump CmdQ END ===== \n",__func__);
#endif
#ifdef ENABLE_ISP_ov5648_REG_DEBUG
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("[0x72f8] = 0x%x\n", status);
		pr_err("[0x0024] = 0x%x\n", gpio_status);
#endif
		isp_ov5648_recover_isp(&isp_ov5648_s_ctrl);
		//iCatch_debug();
	} else {
		pr_err("%s interrupt done\n",__func__);
#ifdef ENABLE_ISP_ov5648_GPIO_DEBUG
		//gpio_direction_output(DEBUG_ISP_ov5648_GPIO,1);
		mdelay(5);
		gpio_direction_output(DEBUG_ISP_ov5648_GPIO,0);
		pr_err("%s: DEBUG_ISP_ov5648_GPIO: %d\n", __func__, gpio_get_value(DEBUG_ISP_ov5648_GPIO));
#endif

#ifdef ENABLE_ISP_ov5648_REG_DEBUG
	pr_err("=======%s START ===== \n",__func__);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_ISP_ov5648_REG_DEBUG
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_err("======%s  END ====== \n",__func__);
#endif
	}
}


/* sensor switching */
void ISPI2C_SensorSelectSet(unsigned char fwNumber)
{
//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*{_20130926
#if 0
	//UINT32 temp;

	I2CDataWrite(0x1011, 1); /* CPU reset */

	I2CDataWrite(0x001C, 0x08);
	I2CDataWrite(0x001C, 0x00);
	I2CDataWrite(0x1010, 0x02);
	I2CDataWrite(0x1010, 0x00);

#if 0
	I2CDataWrite(0x101C, 0x1A);
	I2CDataWrite(0x1140, 0x00);
	I2CDataWrite(0x1150, 0x00);
	I2CDataWrite(0x1160, 0x00);
	I2CDataWrite(0x1170, 0x00);
	I2CDataWrite(0x1180, 0x00);
	I2CDataWrite(0x1190, 0x00);
	I2CDataWrite(0x1200, 0x02);
	I2CDataWrite(0x1310, 0x02);
	I2CDataWrite(0x1008, 0x00);
	I2CDataWrite(0x1009, 0x00);
	I2CDataWrite(0x100C, 0x00);
	I2CDataWrite(0x100D, 0x00);
	I2CDataWrite(0x100E, 0x00);
	I2CDataWrite(0x100F, 0x00);
	I2CDataWrite(0x1018, 0x00);
	I2CDataWrite(0x1004, 0xFF);
	I2CDataWrite(0x1005, 0xFF);
	I2CDataWrite(0x1006, 0xFF);
	I2CDataWrite(0x1007, 0xFF);

	I2CDataWrite(0x7010, 0x07);
	I2CDataWrite(0x7008, 0x00);
	I2CDataWrite(0x7009, 0x00);
	I2CDataWrite(0x700C, 0x00);
	I2CDataWrite(0x700F, 0x00);

	I2CDataWrite(0x941c, 0x00); /* disable LVDS */
	I2CDataWrite(0x9010, 1); /* sensor control reset */
	I2CDataWrite(0x9010, 0);
#endif
	I2CDataWrite(0x1306, fwNumber); /* Set which firmware will be loaded */
	/* turn off unused sensor, this pare can be omitted, since 7002 fw has covered this part*/
	#if 0
	if (fwNumber == 0) { /* switch to main sensor, make front sensor power down */
		temp = I2CDataRead(0x9033);
		temp &= ~0x04;
		I2CDataWrite(0x9033, temp);
	}
	else { /* switch to front sensor, make main sensor power down */
		temp = I2CDataRead(0x9032);
		temp &= ~0x30;
		I2CDataWrite(0x9032, temp);
	}
	#endif
	I2CDataWrite(0x1011, 0);
#else
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x1011, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x001C, 0x08, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x001C, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x1010, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x1010, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x1306, fwNumber, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x1011, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
#endif
//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*}_20130926
}

static void isp_ov5648_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_err("%s\n", __func__);
	if (giCatchStreamOff == 0)
	{
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
		pr_debug("%s, SEND STREAM OFF command to 7002A\n", __func__);
		giCatchStreamOff = 1;
	}
}

int32_t isp_ov5648_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	//RL-20130926
	 int count = 0;

	//test ++
	//while(1)
	//{
	rc = msm_camera_cci_i2c_write(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr, 0x88,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_write id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
//		return rc;
	}

	//RL-20130926
	pr_err("[RL]%s: msm_camera_cci_i2c_write() done!\n", __func__);

	rc = msm_camera_cci_i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr, &chipid,
			MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: msm_camera_cci_i2c_read id 0x88 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
//		return rc;
	//}
	}

	//RL-20130926
	pr_err("[RL]%s: msm_camera_cci_i2c_read() done!\n", __func__);

	//test --
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	//RL*{_20130926
	pr_err("%s: expected id : %x, read id : %x\n", __func__, s_ctrl->sensordata->slave_info->sensor_id, chipid);

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
		pr_err("\n\n******************* [HL] %s, Match ID fail, START of Retry *************************\n\n", __func__);
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
	return rc;
}

int32_t isp_ov5648_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc=0;
	//uint16_t AE_ready = 0;
	//uint16_t frame_ready = 0;
	//uint16_t count=0;

	pr_err("%s: res=%d\n", __func__, res);
	//here need to implement snapshot mode, preview mode,...etc.

	switch(res){
		//SW4-L1-HL-Camera-FixPreviewFailAfterEnablingZSL-VKY-6805-00*{_20130423
		case MSM_SENSOR_RES_FULL:
			isp_ov5648_mode = MSM_SENSOR_RES_FULL;
		break;

		case MSM_SENSOR_RES_QTR:
			isp_ov5648_mode = MSM_SENSOR_RES_QTR;
		break;

		case MSM_SENSOR_RES_FULL_PREVIEW:
			isp_ov5648_mode = MSM_SENSOR_RES_FULL_PREVIEW;
		break;
		//SW4-L1-HL-Camera-FixPreviewFailAfterEnablingZSL-VKY-6805-00*}_20130423

		default:
			pr_err("%s: Do not support this res=%d\n", __func__, res);
	}

	//rc = msm_sensor_write_output_settings(s_ctrl, res);
	//if (rc < 0)
		//return rc;

	return rc;
}

void wait_for_front_AE_ready(void)
{
#if 0//Rocky_00-{_20130924
	uint16_t AE_ready=0;
	int count=0;

	g_isp_ov5648_first_open = false;
	do{
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72C3, &AE_ready, MSM_CAMERA_I2C_BYTE_DATA);
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
int32_t isp_ov5648_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	pr_err("%s: update_type=%d, res=%d\n", __func__, update_type, res);

	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		g_isp_ov5648_first_open = true;
		//msm_sensor_write_init_settings(s_ctrl);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		//msm_sensor_write_res_settings(s_ctrl, res);
		isp_ov5648_write_res_settings(s_ctrl, res);
		v4l2_subdev_notify(&s_ctrl->msm_sd,
			CFG_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
		//isp_ov5648_write_res_settings(s_ctrl, res);
	}
	return rc;
}
#endif

static struct msm_sensor_power_setting isp_ov5648_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
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
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	//ISP power
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
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_AVDD_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_AVDD_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 3,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_VCORE_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_VCORE_EN,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = MSM_SENSOR_MCLK_12HZ,
		.delay = 0,
	},
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
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_SUSPEND,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = ISP_SUSPEND,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

//FIH, Vince, recover ISP+++
static int isp_ov5648_recover_isp(struct msm_sensor_ctrl_t *s_ctrl)
{
	//struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	struct device *dev = NULL;
	uint16_t status = 0x0;
	//uint16_t gpio_status = 0x0;//SW4-Rocky-Camera-PortingCamera_20130719_00
	int32_t rc = 0;
	int32_t i;
	u32 timeout_count = 1;
	//dev = isp_ov5648_s_ctrl.pdev.dev;
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE)//MSM_SENSOR_PLATFORM_DEVICE
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;
//power down************************************************************
isp_ov5648_power_down(s_ctrl);

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

#ifdef ENABLE_ISP_ov5648_INTERRUPT
	if(isp_ov5648_irq_requested)
	{
		isp_ov5648_disable_interrupt();
		//gpio_free(ISP_INTR_GPIO);
		free_irq(g_isp_irq,0);
		isp_ov5648_irq_requested = 0;
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

	//ISP AVDD and DVDD
	msm_camera_config_gpio_table(data, 0);
	msm_camera_request_gpio_table(data, 0);
#endif
//power up******************************************************
	isp_ov5648_power_up(s_ctrl);
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

	pr_err("%s: enable sub camera sensor vreg\n", __func__);
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
	pr_err("%s: set gpio ISP_SUSPEND to Low\n", __func__);
	gpio_direction_output(ISP_SUSPEND, 0);
	msleep(10);
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

//Rocky_00+{_20130924
	/* 0: Main Cam, 1: Front Cam*/
	pr_err("[RK]%s ISPI2C_SensorSelectSet!\n",__func__);
	ISPI2C_SensorSelectSet(1);
//Rocky_00+}_20130924

//switch mode**********************************************
	pr_err("%s: switch mode****************\n", __func__);
#ifdef ENABLE_ISP_ov5648_REG_DEBUG

        pr_err("=======%s [1] START ===== \n",__func__);
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
        msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_ISP_ov5648_REG_DEBUG
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
        pr_err("======%s [1] END ====== \n",__func__);
#endif
	mdelay(10);
#ifdef ENABLE_ISP_ov5648_INTERRUPT
	if(!isp_ov5648_irq_requested)
	{
		rc = isp_ov5648_isp_init_interrupt();
		if(rc < 0)
			pr_err("%s: isp_init_interrupt fail. \n", __func__);
		isp_ov5648_irq_requested = 1;
		isp_ov5648_enable_interrupt();
	}
#endif
#if 1//SW4-Rocky-Camera-PortingCamera_20130719_00
	switch(isp_ov5648_mode)
	{
		case MSM_SENSOR_RES_FULL_PREVIEW:
		case MSM_SENSOR_RES_QTR:
		case MSM_SENSOR_RES_FULL:
			//msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7106, MAIN_RES_4160x3120, MSM_CAMERA_I2C_BYTE_DATA);
			//msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //Set ICatch to preview mode
			msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7106, RES_2592x1944, MSM_CAMERA_I2C_BYTE_DATA);  /*1408*1408*/
			msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7120, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("%s: waiting for MSM_SENSOR_RES_FULL_PREVIEW Clean 0x72F8 \n", __func__);
		break;
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_ov5648_mode);
	}
#endif
	g_time_start = jiffies;
	timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 3*HZ);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72f8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: 0x72f8 status: %d\n", __func__, status);
	pr_err("[wait_time] %d\n",jiffies_to_msecs(jiffies-g_time_start));
	if (!timeout_count) {
		pr_err("%s: interrupt timedout 2\n", __func__);
		//dump command queue to debug, total 128 bytes
		pr_err("=======%s: Dump CmdQ START ===== \n",__func__);
		for(i = 0 ; i < 0x31; i++)
		{
			msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7200+i, &status, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("[0x%x] = 0x%x\n", 0x7200+i, status);
		}
		pr_err("=======%s: Dump CmdQ END ===== \n",__func__);

#ifdef ENABLE_ISP_ov5648_REG_DEBUG
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
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

#ifdef ENABLE_ISP_ov5648_REG_DEBUG
	pr_err("=======%s START ===== \n",__func__);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, 0x04, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_ISP_ov5648_REG_DEBUG
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
	pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
	pr_err("======%s  END ====== \n",__func__);
#endif
	}

	return 0;
}
//FIH, Vince, recover ISP---


#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_camera_i2c_conf_array isp_ov5648_init_conf[] = {
	{isp_ov5648_recommend_settings,
	ARRAY_SIZE(isp_ov5648_recommend_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{isp_ov5648_config_change_settings,
	ARRAY_SIZE(isp_ov5648_config_change_settings),
	0, MSM_CAMERA_I2C_WORD_DATA},
};
#endif

static const struct i2c_device_id isp_ov5648_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&isp_ov5648_s_ctrl},
	{ }
};

static int32_t isp_ov5648_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &isp_ov5648_s_ctrl);
}

static struct i2c_driver isp_ov5648_i2c_driver = {
	.id_table = isp_ov5648_i2c_id,
	.probe  = isp_ov5648_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client isp_ov5648_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id isp_ov5648_dt_match[] = {
	{.compatible = "qcom,isp_ov5648", .data = &isp_ov5648_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, isp_ov5648_dt_match);

static struct platform_driver isp_ov5648_platform_driver = {
	.driver = {
		.name = "qcom,isp_ov5648",
		.owner = THIS_MODULE,
		.of_match_table = isp_ov5648_dt_match,
	},
};
static int32_t isp_ov5648_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	pr_err("%s:%d++++of_match_device++++\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log
	match = of_match_device(isp_ov5648_dt_match, &pdev->dev);
	pr_err("%s:%d----of_match_device----\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log

	pr_err("%s:%d++++msm_sensor_platform_probe++++\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_err("%s:%d----msm_sensor_platform_probe----\n", __func__, __LINE__);//FIHTDC@20130401 Rocky add log

	return rc;
}

static int __init isp_ov5648_init_module(void)
{
	int32_t rc = 0;
	pr_info("[RK]%s:%d++++\n", __func__, __LINE__);
	rc = platform_driver_probe(&isp_ov5648_platform_driver,
		isp_ov5648_platform_probe);

	pr_err("[RK]%s:%d rc %d\n", __func__, __LINE__, rc);

	if (!rc)
		return rc;

	rc = i2c_add_driver(&isp_ov5648_i2c_driver);
	pr_err("[RK]%s:%d(%d)----\n", __func__, __LINE__, rc);

	return rc;
}

void wait_for_iCatch_really_ready_front(void)
{
	uint16_t res1 = 0;
	uint16_t res2 = 0;
	uint16_t res3 = 0;
	uint16_t res4 = 0;
	int count = 0;

	do
	{
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7072, &res1, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7073, &res2, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7074, &res3, MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x7075, &res4, MSM_CAMERA_I2C_BYTE_DATA);

		msleep(5);

		//RL*-modify kernel log level_201301021
		//Orig--pr_err("%s: frame_ready=%d, count=%d\n", __func__, frame_ready, count);
		pr_err("%s: res1 =0x%x, count = %d \n", __func__, res1, count);
		pr_err("%s: res2 =0x%x, count = %d\n", __func__, res2, count);
		pr_err("%s: res3 =0x%x, count = %d\n", __func__, res3, count);
		pr_err("%s: res4 =0x%x, count = %d\n", __func__, res4, count);

		count ++;
	}while(((res1 != 0x20) || (res2 != 0x0a) || (res3 != 0x98) || (res4 != 0x07)) && (count <=200));

	/*frame is not ready*/
	if ((res1 != 0x20) || (res2 != 0x0a) || (res3 != 0x98) || (res4 != 0x07))
	{
		pr_err("%s: res1 = 0x%x\n", __func__, res1);
		pr_err("%s: res2 = 0x%x\n", __func__, res2);
		pr_err("%s: res3 = 0x%x\n", __func__, res3);
		pr_err("%s: res4 = 0x%x\n", __func__, res4);

		pr_err("BBox::UEC; 9::8\n");
	}
}

static void isp_ov5648_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc=0;

#ifdef ENABLE_ISP_ov5648_REG_DEBUG
	uint16_t status = 0x0;
	uint16_t gpio_status = 0x0;
        pr_err("=======%s [1] START ===== \n",__func__);
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[before_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[before_clear] 0x0024 = 0x%x\n", gpio_status);
#endif
        msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef ENABLE_ISP_ov5648_REG_DEBUG
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x72F8, &status, MSM_CAMERA_I2C_BYTE_DATA);
        msm_camera_cci_i2c_read(isp_ov5648_s_ctrl.sensor_i2c_client, 0x0024, &gpio_status, MSM_CAMERA_I2C_BYTE_DATA);
        pr_err("[after_clear] 0x72f8 = 0x%x\n", status);
        pr_err("[after_clear] 0x0024 = 0x%x\n", gpio_status);
        pr_err("======%s [1] END ====== \n",__func__);
#endif
#ifdef ENABLE_ISP_ov5648_INTERRUPT
	if(!isp_ov5648_irq_requested)
	{
		rc = isp_ov5648_isp_init_interrupt();
		if(rc < 0)
			pr_err("%s: isp_init_interrupt fail. \n", __func__);
		isp_ov5648_irq_requested = 1;
		isp_ov5648_enable_interrupt();
	}
#endif
	pr_err("%s delay 10 ms\n", __func__);
	mdelay(10);

	switch(isp_ov5648_mode)
	{
		case MSM_SENSOR_RES_FULL_PREVIEW:
		case MSM_SENSOR_RES_FULL:
			//Fix me: Android 4.4 always pick full resolution,For ICATCH ISP Case need to fix this issues.
			//Workaround:Set preview / snapshot resolution are same.
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, PREVIEW_RES, RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, MODE_SET, PREVIEW_MODE, MSM_CAMERA_I2C_BYTE_DATA);
		break;

		case MSM_SENSOR_RES_QTR:
			if(!g_isp_ov5648_first_open)
				break;
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, PREVIEW_RES, RES_1920x1080, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, MODE_SET, PREVIEW_MODE, MSM_CAMERA_I2C_BYTE_DATA);
		break;
		default:
			pr_err("%s: Do not support this res=%d\n", __func__, isp_ov5648_mode);

	}

	if (giCatchStreamOff == 1)
	{
		msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x7121, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		pr_debug("%s, SEND STREAM ON command to 7002A\n", __func__);
		giCatchStreamOff = 0;
	}
	isp_ov5648_wait_for_next_frame();
	/* It is recommended host utilize interrupt to wait for frame being ready */
	/* Here we use polling 0x72F8 = 0x04 to wait for frame being ready */

	wait_for_front_AE_ready();

	//Fix me: Temp disable Full resolution check,Need to fix pick resolution issue first.
	#if 0
	if ((isp_ov5648_mode==MSM_SENSOR_RES_FULL) || (isp_ov5648_mode==MSM_SENSOR_RES_FULL_PREVIEW))
	{
		wait_for_iCatch_really_ready_front();
	}
	#endif

}
static int32_t isp_ov5648_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t isp_ov5648_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

int32_t isp_ov5648_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	s_ctrl->stop_setting_valid = 0;
//	struct device *dev = NULL;
//	uint16_t status=0;
//	int32_t rc = 0;

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (front_cam_is_power_on == 0)
		return 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402

    //SW5-Webber-Add_for_turn_off_led_flash_20130416
#if 0
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9008, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9009, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x900a, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x900b, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9238, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9240, 0xC6, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9200, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9210, 0x0A, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9211, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_cci_i2c_write(s_ctrl->sensor_i2c_client, 0x9204, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(100);

	msm_camera_cci_i2c_read(s_ctrl->sensor_i2c_client, 0x7000, &status, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("%s: ****************************ISP 0x7000 status =%d \n", __func__,status);
#endif
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	CDBG("%s:%d\n", __func__, __LINE__);
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
			if (power_setting->seq_val >= SENSOR_POWER_MAX ||
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_POWER_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				continue;
			}
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);
				continue;
			}
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				isp_ov5648_disable_i2c_mux(data->i2c_conf);
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

#ifdef ENABLE_ISP_ov5648_INTERRUPT
	if(isp_ov5648_irq_requested)
	{
		isp_ov5648_disable_interrupt();
		gpio_free(ISP_INTR_GPIO);
		free_irq(g_isp_irq,0);
		isp_ov5648_irq_requested = 0;
	}
#endif
	msm_camera_request_gpio_table(
	data->gpio_conf->cam_gpio_req_tbl,
	data->gpio_conf->cam_gpio_req_tbl_size, 0);

	//kfree(s_ctrl->reg_ptr);
	//s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
	//g_pre_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	front_cam_is_power_on = 0;
	return 0;
}


int32_t  isp_ov5648_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	s_ctrl->stop_setting_valid = 0;
	//struct device *dev = NULL;
	//int32_t rc = 0;

	if(isp_is_power_on == 1)
		return 0;

	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+{_20130402
	if (front_cam_is_power_on == 1)
		return 0;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+}_20130402
	power_setting_array = &s_ctrl->power_setting_array;

	if (data->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			data->gpio_conf->cam_gpiomux_conf_tbl,
			data->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	//Fixme: To avoid gpio request fail that need free gpio first
	msm_camera_request_gpio_table(data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 0);

	rc = msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	//pr_err("%s: platform device name: %s\n", __func__, s_ctrl->pdev->name);
	//pr_err("%s: i2c device name: %s\n", __func__, s_ctrl->msm_sensor_client->name);
	//pr_err("%s: platform device name: %s\n", __func__, s_ctrl->pdev->name);

	for (index = 0; index < power_setting_array->size; index++) {
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
				s_ctrl->clk_info[power_setting->seq_val].
					clk_rate = power_setting->config_val;

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
			if (power_setting->seq_val >= SENSOR_POWER_MAX ||//SW4-Rocky-Camera-PortingCamera_20130816_00
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_POWER_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
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
			if (power_setting->seq_val >= CAM_VREG_MAX) {//SW4-Rocky-Camera-PortingCamera_20130816_00
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					CAM_VREG_MAX);//SW4-Rocky-Camera-PortingCamera_20130816_00
				goto power_up_failed;
			}

			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				1);
			break;
		case SENSOR_I2C_MUX:
			pr_err("[RK]%s:%d SENSOR_I2C_MUX ++++\n", __func__, __LINE__);
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
			{
				pr_err("[RK]%s:%d Do isp_ov5648_sensor_enable_i2c_mux!!\n", __func__, __LINE__);
				isp_ov5648_enable_i2c_mux(data->i2c_conf);
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

	//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*{_20130926
	//Rocky_00+{_20130924
	/* 0: Main Cam, 1: Front Cam*/
	ISPI2C_SensorSelectSet(1);
	//Rocky_00+}_20130924
	pr_err("\n\n********************* [HL] %s, Call ISPI2C_SensorSelectSet AFTER  isp_ov5648_match_id() ******************************\n\n", __func__);
	//SW4-HL-Camera-FixI2CWriteErrorCauseKernelPanic-00*}_20130926

	//s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
	//g_pre_res = MSM_SENSOR_INVALID_RES;
	isp_is_power_on = 1;
	//SW4-L1-HL-Camera-FixCTSFailItem-testMultiCameraRelease-00+_20130402
	front_cam_is_power_on = 1;
	isp_imx135_inti_parms();
	msm_camera_cci_i2c_write(isp_ov5648_s_ctrl.sensor_i2c_client,
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
				isp_ov5648_disable_i2c_mux(data->i2c_conf);
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

#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct v4l2_subdev_core_ops isp_ov5648_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops isp_ov5648_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops isp_ov5648_subdev_ops = {
	.core = &isp_ov5648_subdev_core_ops,
	.video  = &isp_ov5648_subdev_video_ops,
};
#endif
static struct msm_sensor_fn_t isp_ov5648_func_tbl = {
		.sensor_start_stream = isp_ov5648_start_stream,
		.sensor_stop_stream = isp_ov5648_stop_stream,
		//.sensor_setting = isp_ov5648_setting,
		//.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
		//.sensor_mode_init = msm_sensor_mode_init,
		//.sensor_get_output_info = msm_sensor_get_output_info,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00*{_20130311
		//Orig -- .sensor_config = msm_sensor_config,
		.sensor_config = isp_imx135_config,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00*}_20130311
		.sensor_power_up = isp_ov5648_power_up,
		.sensor_power_down = isp_ov5648_power_down,
		//.sensor_get_csi_params = msm_sensor_get_csi_params,
		.sensor_match_id = isp_ov5648_match_id,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00+{_20130311
		//.sensor_set_isp_scene_mode =  iCatch_set_scene_mode,
		//.sensor_set_isp_effect_mode = iCatch_set_effect_mode,
		//.sensor_set_isp_exposure_compensation = iCatch_set_exposure_compensation,
		//.sensor_set_isp_aec_mode = iCatch_set_aec_mode,
		//.sensor_set_iso = iCatch_set_iso,
		//.sensor_set_isp_wb = iCatch_set_wb,
		//.sensor_set_isp_saturation = iCatch_set_saturation,
		//.sensor_set_isp_sharpness = iCatch_set_sharpness,
		//.sensor_set_isp_contrast = iCatch_set_contrast,
		//.sensor_set_antibanding = iCatch_set_antibanding,
		//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00+}_20130311
};
#if 0//SW4-Rocky-Camera-PortingCamera_20130719_00
static struct msm_sensor_reg_t isp_ov5648_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = isp_ov5648_config_change_settings,
	.start_stream_conf_size = ARRAY_SIZE(isp_ov5648_config_change_settings),
	.init_settings = &isp_ov5648_init_conf[0],
	.init_size = ARRAY_SIZE(isp_ov5648_init_conf),
	//.mode_settings = &isp_ov5648_confs[0],
	.output_settings = &isp_ov5648_dimensions[0],
	.num_conf = ARRAY_SIZE(isp_ov5648_dimensions),
};
#endif
static struct msm_sensor_ctrl_t isp_ov5648_s_ctrl = {
	//.msm_sensor_reg = &isp_ov5648_regs,
	//.msm_sensor_v4l2_ctrl_info = isp_ov5648_v4l2_ctrl_info,
	//.num_v4l2_ctrl = ARRAY_SIZE(isp_ov5648_v4l2_ctrl_info),
	.sensor_i2c_client = &isp_ov5648_sensor_i2c_client,
	//.sensor_i2c_addr = 0x78,
	.power_setting_array.power_setting = isp_ov5648_power_setting,
	.power_setting_array.size = ARRAY_SIZE(isp_ov5648_power_setting),
//#if ENABLE_SENSOR_REGULATOR
//	.vreg_seq = isp_ov5648_veg_seq,
//	.num_vreg_seq = ARRAY_SIZE(isp_ov5648_veg_seq),
//#endif
//	.sensor_output_reg_addr = &isp_ov5648_reg_addr,
//	.sensor_id_info = &isp_ov5648_id_info,
//	.cam_mode = MSM_SENSOR_MODE_INVALID,
//	.min_delay = 30,
//	.power_seq_delay = 0,
	.msm_sensor_mutex = &isp_ov5648_mut,
	//.sensor_i2c_driver = &isp_ov5648_i2c_driver,
	.sensor_v4l2_subdev_info = isp_ov5648_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(isp_ov5648_subdev_info),
	//.sensor_v4l2_subdev_ops = &isp_ov5648_subdev_ops,
	.func_tbl = &isp_ov5648_func_tbl,
	//.clk_rate = MSM_SENSOR_MCLK_12HZ,  /*1041 port*/
};

static void __exit isp_ov5648_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (isp_ov5648_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&isp_ov5648_s_ctrl);
		platform_driver_unregister(&isp_ov5648_platform_driver);
	} else
		i2c_del_driver(&isp_ov5648_i2c_driver);
	return;
}

module_init(isp_ov5648_init_module);
module_exit(isp_ov5648_exit_module);

MODULE_DESCRIPTION("isp_ov5648");
MODULE_LICENSE("GPL v2");
