#ifndef __DRV2603_VIBRATOR_H__
#define __DRV2603_VIBRATOR_H__

#define PWM_MODE 0
#define CLK_MODE 1

struct drv2603_vibrator_platform_data {
	int en_gpio;
	int pwm_gpio;
	unsigned int max_timeout_ms;
	unsigned int vib_break_ms;
	int mode_ctrl;
	unsigned int default_level;

	//pwm
	int pwm_channel;
	unsigned int pwm_period_us;
};

#endif //end of __DRV2603_VIBRATOR_H__
