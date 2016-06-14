//SW4-Rocky-Camera-PortingCamera_20130719_00_{
//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00+{_20130311
/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef ISP_MAIN_CAM_H
#define ISP_MAIN_CAM_H

int isp_main_cam_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp);

void iCatch_set_effect_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
//void iCatch_start_AF(struct msm_sensor_ctrl_t *s_ctrl, bool on, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
void iCatch_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);
void iCatch_set_aec_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_wb(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_saturation(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);
void iCatch_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);
void iCatch_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);
void iCatch_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);
//void iCatch_set_caf_mode(struct msm_sensor_ctrl_t *s_ctrl, bool continuous);
//void iCatch_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_AEC_ROI(struct msm_sensor_ctrl_t *s_ctrl, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+_20140328

void iCatch_get_exposure_time(struct msm_sensor_ctrl_t *s_ctrl, exposure_value_cfg *value);//SW4-RK-Camera-SetEXIFInformation-00+_20131225
void iCatch_get_iso_value(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *value);//SW4-RK-Camera-SetEXIFInformation-00+_20131225
void iCatch_get_3a_info(struct msm_sensor_ctrl_t *s_ctrl, threeA_info_get_cfg *value);//SW4-RK-Camera-SetEXIF3AInformation-00+_20140218

void DumpICatchRegister(struct msm_sensor_ctrl_t *s_ctrl);//Rocky_20131024
void iCatch_set_wdr_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);	//SW4-RL-Camera-WDR-00*_20140117

void iCatch_set_aec_lock(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);	//SW4-RL-Camera-implementAELock-00+_20140710
void iCatch_set_awb_lock(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);	//SW4-RL-Camera-implementAWBLock-00+_20140710

//fihtdc,derekcww,add for front cam when front cam can focus,start
void iCatch_start_AF(struct msm_sensor_ctrl_t *s_ctrl, bool on, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
void iCatch_set_AF_Mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_AF_ROI(struct msm_sensor_ctrl_t *s_ctrl, int16_t enable, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w, uint8_t isFaceDetectMode); //SW4-RL-implementFaceTracking-00*_20140919
void iCatch_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void iCatch_set_sensor_data(struct msm_sensor_ctrl_t *s_ctrl, sensor_data_get_t *value);
void iCatch_set_HDR_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
void isp_deinit_interrupt(void);
int  isp_init_interrupt(unsigned int intr_gpio);
int isp_main_cam_start_stream(struct msm_sensor_ctrl_t *s_ctrl);
int isp_main_cam_stop_stream(struct msm_sensor_ctrl_t *s_ctrl);
int  isp_main_cam_power_up(struct msm_sensor_ctrl_t *s_ctrl);
int isp_main_cam_power_down(struct msm_sensor_ctrl_t *s_ctrl);
int ISPI2C_SensorSelectSet_EVT(struct msm_sensor_ctrl_t *s_ctrl, unsigned char fwNumber);
int32_t isp_main_cam_setting(struct msm_sensor_ctrl_t *s_ctrl,int update_type, int res);
//fihtdc,derekcww,add for front cam when front cam can focus,end

#define CAPTURE_MODE 0
#define PREVIEW_MODE 1

//Ref ICATCH resolution table Register:0x7106
#define MAIN_RES_1280x960 0x00
#define MAIN_RES_3264x2448 0x01
#define MAIN_RES_1920x1080 0x02
#define MAIN_RES_320x240 0x03
#define MAIN_RES_1280x720 0x04
#define MAIN_RES_1040x780 0x05
#define MAIN_RES_2080x1560 0x06
#define MAIN_RES_3648x2736 0x07
#define MAIN_RES_4160x3120 0x08
#define MAIN_RES_3360x1890 0x09
#define MAIN_RES_2592x1944 0x0A
#define MAIN_RES_640x480 0x0B

struct fih_cam_project{
    char project_name[15];
    struct msm_sensor_power_setting *project_power_setting;
    uint16_t power_setting_size;
    unsigned int qtr_resolution;	//RL*_20140710
    unsigned int full_resolution;
    unsigned int hd_resolution;	//RL*_20140710
    unsigned int interrupt_gpio;
    unsigned int isp_firmware_size;
    int support_early_preview;//fihtdc,derekcwwu,20140828, improve take picture kpi
};


static struct msm_sensor_power_setting mo7evb_isp_imx135_power_setting[] = {
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
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VPH,
		.config_val = 0,
		.delay = 0,
	},
	//sensor power ---
	//flash led +++
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 7,//ISP_FLASH_EN
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 7,//ISP_FLASH_EN
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
/*
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 8,//ISP_FLASH_NOW
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 8,//ISP_FLASH_NOW
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
*/
	//flash led ---
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
static struct msm_sensor_power_setting mo7evb_f_isp_imx135_power_setting[] = {
	//sensor power +++
/*derek test
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = CAM_8M_2P8_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = CAM_8M_2P8_EN,
		.config_val = GPIO_OUT_HIGH,
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
		.seq_val = 2,//CAM_VAF
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = 3,//CAM_VPH,pull high
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
static struct msm_sensor_power_setting mo7evt2_isp_imx135_power_setting[] = {
	//sensor power +++
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 9,//VAAM_EN
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 9,//VAAM_EN
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	/*
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	*/
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
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VPH,
		.config_val = 0,
		.delay = 0,
	},
	//sensor power ---
	//flash led +++
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 7,//ISP_FLASH_EN
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 7,//ISP_FLASH_EN
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
/*
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 8,//ISP_FLASH_NOW
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 8,//ISP_FLASH_NOW
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
*/
	//flash led ---
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
static struct msm_sensor_power_setting mo7evt2_f_isp_imx135_power_setting[] = {
	//sensor power +++
/*derek test
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = CAM_8M_2P8_EN,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = CAM_8M_2P8_EN,
		.config_val = GPIO_OUT_HIGH,
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
		.seq_val = 2,//CAM_VAF
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = 3,//CAM_VPH,pull high
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
static struct msm_sensor_power_setting vnaevt_isp_imx135_power_setting[] = {
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
	//flash led +++
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 8,//ISP_FLASH_EN
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = 8,//ISP_FLASH_EN
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	//flash led ---
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
static struct msm_sensor_power_setting vnaevt_isp_ov5648_power_setting[] = {
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

static struct msm_sensor_power_setting vn2evt_isp_imx135_power_setting[] = {
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
		.delay = 3,	// Orig - 1 	//HL*_20130910
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
		.delay = 0,	//Orig - 1,	//HL*_20130911
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

static struct msm_sensor_power_setting vn2evt_isp_imx219_power_setting[] = {
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
	/*
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	*/
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

//This vn2evt2_5 power setting is for PHX EVT2.5 fixed focus camera using
//Since VN2 & PHXs' original camera hardware are the same, we use VN2 naming for this two project
static struct msm_sensor_power_setting vn2evt2_5_isp_imx219_power_setting[] = {
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

static struct fih_cam_project fih_cam_all_setting[]={
//SW5-Webber-20150425-Add PHX camera settings++
//Note : Since PHX's camera setting is the same as VN2 , we use VN2's setting
//          instead of creating other power settings the same as VN2
    {
        .project_name="PHXEVT",
        .project_power_setting=vn2evt_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt_isp_imx135_power_setting),
        .qtr_resolution = MAIN_RES_2080x1560,
        .full_resolution = MAIN_RES_4160x3120,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
        .isp_firmware_size=1024*1024
    },
    {
        .project_name="F_PHXEVT",
        .project_power_setting=vn2evt_isp_imx219_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt_isp_imx219_power_setting),
        .qtr_resolution = MAIN_RES_1040x780,
        .full_resolution = MAIN_RES_3264x2448,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
    },
    {
        .project_name="PHXEVT2_5",
        .project_power_setting=vn2evt_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt_isp_imx135_power_setting),
        .qtr_resolution = MAIN_RES_2080x1560,
        .full_resolution = MAIN_RES_4160x3120,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
        .isp_firmware_size=1024*1024
    },
    {
        .project_name="F_PHXEVT2_5",
        .project_power_setting=vn2evt2_5_isp_imx219_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt2_5_isp_imx219_power_setting),
        .qtr_resolution = MAIN_RES_1040x780,
        .full_resolution = MAIN_RES_3264x2448,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
    },
//SW5-Webber-20150425-Add PHX camera settings--
    {
        .project_name="VN2EVT2_5",
        .project_power_setting=vn2evt_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt_isp_imx135_power_setting),
        .qtr_resolution = MAIN_RES_2080x1560,
        .full_resolution = MAIN_RES_4160x3120,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
        .isp_firmware_size=1024*1024
    },
    {
        .project_name="VN2EVT",
        .project_power_setting=vn2evt_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt_isp_imx135_power_setting),
        .qtr_resolution = MAIN_RES_2080x1560,
        .full_resolution = MAIN_RES_4160x3120,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
        .isp_firmware_size=1024*1024
    },
    {
        .project_name="F_VN2EVT",
        .project_power_setting=vn2evt_isp_imx219_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt_isp_imx219_power_setting),
        .qtr_resolution = MAIN_RES_1040x780,
        .full_resolution = MAIN_RES_3264x2448,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
    },
    {
        .project_name="F_VN2EVT2_5",
        .project_power_setting=vn2evt2_5_isp_imx219_power_setting,
        .power_setting_size=ARRAY_SIZE(vn2evt2_5_isp_imx219_power_setting),
        .qtr_resolution = MAIN_RES_1040x780,
        .full_resolution = MAIN_RES_3264x2448,
        .hd_resolution = MAIN_RES_1280x720,
        .interrupt_gpio=102,
    },
    {
        .project_name="MO7EVB",
        .project_power_setting=mo7evb_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(mo7evb_isp_imx135_power_setting),
        .qtr_resolution=0,
        .interrupt_gpio=102,
        .isp_firmware_size=1024*1024
    },
    {
        .project_name="F_MO7EVB",
        .project_power_setting=mo7evb_f_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(mo7evb_f_isp_imx135_power_setting),
        .qtr_resolution=MAIN_RES_2080x1560,
        .interrupt_gpio=102,
    },
    {
        .project_name="MO7EVT2",
        .project_power_setting=mo7evt2_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(mo7evt2_isp_imx135_power_setting),
        .qtr_resolution=0,
        .interrupt_gpio=102,
        .isp_firmware_size=1024*1024
    },
    {
        .project_name="F_MO7EVT2",
        .project_power_setting=mo7evt2_f_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(mo7evt2_f_isp_imx135_power_setting),
        .qtr_resolution=MAIN_RES_2080x1560,
        .interrupt_gpio=102,
    },
    {
        .project_name="VNAEVT",
        .project_power_setting=vnaevt_isp_imx135_power_setting,
        .power_setting_size=ARRAY_SIZE(vnaevt_isp_imx135_power_setting),
        .qtr_resolution=0,
        .interrupt_gpio=102,
        .isp_firmware_size=512*1024,
    },
    {
        .project_name="F_VNAEVT",
        .project_power_setting=vnaevt_isp_ov5648_power_setting,
        .power_setting_size=ARRAY_SIZE(vnaevt_isp_ov5648_power_setting),
        .qtr_resolution=MAIN_RES_1040x780,
        .full_resolution=MAIN_RES_2592x1944,
        .hd_resolution=MAIN_RES_1280x720,
        .interrupt_gpio=102,
    },
    {
        .project_name="",
        .project_power_setting=NULL,
        .power_setting_size=0,
        .qtr_resolution=0,
        .interrupt_gpio=0,
    }
};

#endif
//SW4-L1-HL-Camera-ImplementFrontCameraSetting-00+}_20130311
//SW4-Rocky-Camera-PortingCamera_20130719_00_}
