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

#ifndef ISP_imx135_H
#define ISP_imx135_H

int isp_imx135_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp);
void iCatch_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int8_t value);

#if 0
void iCatch_set_effect_mode(int8_t mode);
void iCatch_set_scene_mode(kernel_camera_bestshot_mode_type mode);
void iCatch_start_AF(bool on, kernel_isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
void iCatch_set_exposure_compensation(int8_t value);
void iCatch_set_aec_mode(int8_t mode);
void iCatch_set_iso(int8_t mode);
void iCatch_set_wb(int8_t mode);
void iCatch_set_saturation(int8_t value);
void iCatch_set_sharpness(int8_t value);
void iCatch_set_contrast(int8_t value);
void iCatch_set_caf_mode(bool continuous);
void iCatch_set_led_flash_mode(struct msm_sensor_ctrl_t *s_ctrl, int8_t mode);
#endif

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

#endif

#ifdef ISP_IMX135_C

uint16_t iCatch_iso_value_table[MSM_CAMERA_ISO_MODE_MAX] = {0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 , 0x06};
uint16_t iCatch_contrast_value_table[MSM_CAMERA_CONTRAST_FIHLVMAX] = {0xEE, 0xF1, 0xF4, 0xF6, 0xFB, 0x00, 0x05, 0x0A, 0x0C, 0x0F, 0x12};

#else

extern uint16_t iCatch_iso_value_table[MSM_CAMERA_ISO_MODE_MAX];
extern uint16_t iCatch_contrast_value_table[MSM_CAMERA_CONTRAST_FIHLVMAX];

#endif
