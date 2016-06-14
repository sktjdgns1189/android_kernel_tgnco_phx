/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

#ifndef MSM_SENSOR_H
#define MSM_SENSOR_H

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <mach/camera2.h>
#include <media/msm_cam_sensor.h>
#include <media/v4l2-subdev.h>
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_sd.h"

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

struct msm_sensor_ctrl_t;

enum msm_sensor_state_t {
	MSM_SENSOR_POWER_DOWN,
	MSM_SENSOR_POWER_UP,
};

struct msm_sensor_fn_t {
	int (*sensor_config) (struct msm_sensor_ctrl_t *, void __user *);
	int (*sensor_power_down)
		(struct msm_sensor_ctrl_t *);
	int (*sensor_power_up) (struct msm_sensor_ctrl_t *);
	int32_t (*sensor_match_id)(struct msm_sensor_ctrl_t *s_ctrl);
	//SW4-Rocky-Camera-PortingCamera_00+{_20130913
	//SW4-HL-Camera-ErrorHandling-00*{_20140410
	int (*sensor_start_stream) (struct msm_sensor_ctrl_t *);
	int (*sensor_stop_stream) (struct msm_sensor_ctrl_t *);
	//SW4-HL-Camera-ErrorHandling-00*}_20140410
	int32_t (*sensor_mode_init) (struct msm_sensor_ctrl_t *,
		int, struct sensor_init_cfg *);
	int32_t (*sensor_setting) (struct msm_sensor_ctrl_t *,
			int update_type, int rt);
#if 0
	void (*sensor_group_hold_on) (struct msm_sensor_ctrl_t *);
	void (*sensor_group_hold_off) (struct msm_sensor_ctrl_t *);
	int32_t (*sensor_set_fps) (struct msm_sensor_ctrl_t *,
			struct fps_cfg *);
	int32_t (*sensor_write_exp_gain) (struct msm_sensor_ctrl_t *,
			uint16_t, uint32_t, int32_t, uint16_t);
	int32_t (*sensor_write_snapshot_exp_gain) (struct msm_sensor_ctrl_t *,
			uint16_t, uint32_t, int32_t, uint16_t);
	int32_t (*sensor_setting) (struct msm_sensor_ctrl_t *,
			int update_type, int rt);
	int32_t (*sensor_csi_setting) (struct msm_sensor_ctrl_t *,
			int update_type, int rt);
	int32_t (*sensor_set_sensor_mode)
			(struct msm_sensor_ctrl_t *, int, int);
	int32_t (*sensor_get_output_info) (struct msm_sensor_ctrl_t *,
		struct sensor_output_info_t *);
	void (*sensor_adjust_frame_lines) (struct msm_sensor_ctrl_t *s_ctrl);
	int32_t (*sensor_get_csi_params)(struct msm_sensor_ctrl_t *,
		struct csi_lane_params_t *);
	int (*sensor_set_vision_mode)(struct msm_sensor_ctrl_t *s_ctrl,
			int32_t vision_mode_enable);
	int (*sensor_set_vision_ae_control)(
			struct msm_sensor_ctrl_t *s_ctrl, int ae_mode);
	int32_t (*sensor_read_eeprom)(struct msm_sensor_ctrl_t *);
	int32_t (*sensor_hdr_update)(struct msm_sensor_ctrl_t *,
		 struct sensor_hdr_update_parm_t *);
#endif
//*************************************
//ISP Function implement
//Anvoi 20130212
#ifndef CONFIG_FIH_ISP_MAIN_CAM
        void (*sensor_isp_af_start) (bool, kernel_isp3a_af_mode_t, int16_t, int16_t, int16_t, int16_t);
#else
        void (*sensor_isp_af_start) (struct msm_sensor_ctrl_t *, bool, kernel_isp3a_af_mode_t, int16_t, int16_t, int16_t, int16_t);
#endif
#if 0
	void (*sensor_set_isp_caf_mode) (bool);
#endif
	void (*sensor_set_isp_scene_mode) (struct msm_sensor_ctrl_t *, int8_t);
	void (*sensor_set_isp_effect_mode) (struct msm_sensor_ctrl_t *, int8_t);
	//SW4-L1-HL-Camera-ImplementSceneMode-00+{_20130227
	void (*sensor_set_isp_exposure_compensation) (struct msm_sensor_ctrl_t *, int8_t);
	//SW4-L1-HL-Camera-ImplementExposureMeter-00+_20130227
	void (*sensor_set_isp_aec_mode) (struct msm_sensor_ctrl_t *, int8_t);
	void (*sensor_set_isp_af_mode) (struct msm_sensor_ctrl_t *, int8_t);//Rocky_20131005_FORTEST
	void (*sensor_set_isp_af_roi) (struct msm_sensor_ctrl_t *,int16_t, int16_t,  int16_t, int16_t,  int16_t, uint8_t);//Rocky_20131009_FORTEST	//SW4-RL-implementFaceTracking-00*_20140919
	void (*sensor_set_isp_aec_roi) (struct msm_sensor_ctrl_t *,int16_t,  int16_t, int16_t,  int16_t);//SW4-RK-Camera-SettingFrontCameraExposureMeteringMode-00+_20140328
	//20130227@Rocky add iso/wb function[START]
	void (*sensor_set_iso) (struct msm_sensor_ctrl_t *, int8_t);
	void (*sensor_set_isp_wb) (struct msm_sensor_ctrl_t *, int8_t);
	//20130227@Rocky add iso/wb function[END]
#ifndef CONFIG_FIH_ISP_MAIN_CAM
	void (*sensor_set_isp_saturation) (int8_t);
	void (*sensor_set_isp_sharpness) (int8_t);
	void (*sensor_set_isp_iso) (struct msm_sensor_ctrl_t *, int8_t);
#else
	//SW4-L1-HL-Camera-ImplementSaturation-00+_20130305
	void (*sensor_set_isp_saturation) (struct msm_sensor_ctrl_t *, int8_t);
	//SW4-L1-HL-Camera-ImplementSharpness-00+_20130305
	void (*sensor_set_isp_sharpness) (struct msm_sensor_ctrl_t *, int8_t);
	//SW4-L1-HL-Camera-ImplementContrast-00+_20130305
#endif
	void (*sensor_set_isp_contrast) (struct msm_sensor_ctrl_t *, int8_t);
	//SW4-L1-HL-Camera-ImplementAntiBanding-00+_20130307
	void (*sensor_set_isp_antibanding) (struct msm_sensor_ctrl_t *, int8_t);
    //SW5-Webber-Camera-ImplementLedFlash-20130313-start
    void (*sensor_set_isp_led_flash_mode) (struct msm_sensor_ctrl_t *, int8_t);
    //SW5-Webber-Camera-ImplementLedFlash-20130313-end
//SW5-marx-Camera-ImplementHDR-20130318-start
    void (*sensor_set_isp_HDR_mode) (struct msm_sensor_ctrl_t *, int8_t);
    //SW5-marx-Camera-ImplementHDR-20130318-end
    //SW5-Webber-Camera-ImplementCancelAF-20130331-begin
    void (*sensor_set_cancel_af)(struct msm_sensor_ctrl_t *);
    //SW5-Webber-Camera-ImplementCancelAF-20130331-end
    void (*sensor_get_isp_exposure_time) (struct msm_sensor_ctrl_t *, exposure_value_cfg *);//SW4-RK-Camera-SetEXIFInformation-00+_20131225
    void (*sensor_get_isp_iso_value) (struct msm_sensor_ctrl_t *, uint32_t *);//SW4-RK-Camera-SetEXIFInformation-00+_20131230
    void(*sensor_set_isp_wdr)(struct msm_sensor_ctrl_t *, int8_t); //SW4-RL-Camera-WDR-00*_20140117
    void (*sensor_set_isp_sensor_data) (struct msm_sensor_ctrl_t *, sensor_data_get_t *);//SW4-RK-Camera-GetGYRO_GsensorData-00+_20140116
    void (*sensor_get_isp_flash_status) (struct msm_sensor_ctrl_t *, uint32_t *); //FihtdcCode@AlanHZChang, add for flashLED status from ISP, 2014/02/19
    void (*sensor_get_isp_3a_info) (struct msm_sensor_ctrl_t *, threeA_info_get_cfg *);//SW4-RK-Camera-SetEXIF3AInformation-00+_20140218
    void(*sensor_set_isp_aec_lock)(struct msm_sensor_ctrl_t *, int8_t); //SW4-RL-Camera-implementAELock-00+_20140710
    void(*sensor_set_isp_awb_lock)(struct msm_sensor_ctrl_t *, int8_t); //SW4-RL-Camera-implementAWBLock-00+_20140710
	//SW4-Rocky-Camera-PortingCamera_00+}_20130913
};


struct msm_sensor_ctrl_t {
	struct platform_device *pdev;
	struct mutex *msm_sensor_mutex;

	enum msm_camera_device_type_t sensor_device_type;
	struct msm_camera_sensor_board_info *sensordata;
	struct msm_sensor_power_setting_array power_setting_array;
	struct msm_sensor_packed_cfg_t *cfg_override;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_i2c_master;

	struct msm_camera_i2c_client *sensor_i2c_client;
	struct device *dev;
	struct v4l2_subdev_info *sensor_v4l2_subdev_info;
	uint8_t sensor_v4l2_subdev_info_size;
	struct v4l2_subdev_ops *sensor_v4l2_subdev_ops;
	struct msm_sensor_fn_t *func_tbl;
	struct msm_camera_i2c_reg_setting stop_setting;
	bool stop_setting_valid;
	bool free_power_setting;
	struct msm_cam_clk_info *clk_info;
	uint16_t clk_info_size;
	void *misc_regulator;
	enum msm_sensor_state_t sensor_state;
	uint8_t is_probe_succeed;
	uint32_t id;
	struct device_node *of_node;
};

int msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp);

int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_check_id(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl);

int32_t msm_sensor_platform_probe(struct platform_device *pdev,
	void *data);

int msm_sensor_update_cfg(struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id, struct msm_sensor_ctrl_t *s_ctrl);

int msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl);

int32_t msm_sensor_init_default_params(struct msm_sensor_ctrl_t *s_ctrl);

int32_t msm_sensor_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size);

int32_t msm_sensor_get_dt_gpio_set_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size);

int32_t msm_sensor_init_gpio_pin_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size);
#endif
