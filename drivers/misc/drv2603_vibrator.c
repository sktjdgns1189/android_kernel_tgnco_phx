/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/qpnp/pwm.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include "../staging/android/timed_output.h"
#include <linux/drv2603_vibrator.h>


#define DRV2603_VIBRATOR_DEV_NAME	"drv2603_vib"

/* PWM */
#define DEFAULT_PWM_PERIOD_US 32

/* Option */
#define ERM_0V_DUTY	50
#define PWM_LEVEL_FULL	50
#define PWM_LEVEL_LOW	1
#define DEFAULT_VIB_BRAKE_INTERVAL	10
#define DEFAULT_MAX_TIMEOUT 15000

/* Black Box */
#define BBOX_VIBRATOR_PROBE_FAIL do {printk("BBox;%s: Probe fail\n", __func__); printk("BBox::UEC;19::0\n");} while (0);
#define BBOX_VIBRATOR_ENABLE_PWM_FAIL do {printk("BBox;%s: Enable PWM fail\n", __func__); printk("BBox::UEC;19::5\n");} while (0);
#define BBOX_VIBRATOR_DISABLE_PWM_FAIL do {printk("BBox;%s: Disable PWM fail\n", __func__); printk("BBox::UEC;19::6\n");} while (0);

struct drv2603_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	spinlock_t lock;
	struct work_struct work;
	struct device *dev;
	const struct drv2603_vibrator_platform_data *pdata;
	int state;
	unsigned int vib_break_ms;

	/* PWM mode */
	struct pwm_device *vib_pwm;
	unsigned int pwm_period_us;
	unsigned int pwm_freq_hz;
	unsigned int pwm_level;

	/* CLK mode */
	struct clk *pwm_clk;
	struct mutex lock_clk;
	bool clk_on;
};

static struct drv2603_vib *g_vib = NULL;

/* Static functions */
static int drv2603_vib_set(struct drv2603_vib *vib, int on)
{
	int ret = 0, duty_us = 0;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = vib->pdata->vib_break_ms*1000;

	if (on) {
		if (vib->pdata->mode_ctrl == PWM_MODE) {
			duty_us = (vib->pwm_period_us * (ERM_0V_DUTY + vib->pwm_level) / 100);
			ret = pwm_config_us(vib->vib_pwm, duty_us, vib->pwm_period_us);
			if (ret) {
				BBOX_VIBRATOR_ENABLE_PWM_FAIL;
				printk(KERN_ERR "%s: pwm_config() failed err=%d.\n", __func__, ret);
				return ret;
			}
			ret = pwm_enable(vib->vib_pwm);
			if (ret) {
				BBOX_VIBRATOR_ENABLE_PWM_FAIL;
				printk(KERN_ERR "%s: pwm_enable() failed err=%d\n", __func__, ret);
				return ret;
			}
		} else if (vib->pdata->mode_ctrl == CLK_MODE) {
			mutex_lock(&vib->lock_clk);
			if (!vib->clk_on && vib->pwm_clk) {
				ret = clk_prepare_enable(vib->pwm_clk);
				if (ret < 0) {
					pr_err("%s: clk enable failed\n", __func__);
					mutex_unlock(&vib->lock_clk);
					goto err_en_clk;
				}
				vib->clk_on = true;
			}
			mutex_unlock(&vib->lock_clk);
		}
	} else {
		if (vib->pdata->mode_ctrl == PWM_MODE) {
			//duty_us = (vib->pwm_period_us * (ERM_0V_DUTY - vib->pwm_level) / 100);
			duty_us = (vib->pwm_period_us / 2);
			ret = pwm_config_us(vib->vib_pwm, duty_us, vib->pwm_period_us);
			if (ret) {
				pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
				BBOX_VIBRATOR_DISABLE_PWM_FAIL;
				return ret;
			}
			sys_select(0, NULL, NULL, NULL, &tv);
			pwm_disable(vib->vib_pwm);
		} else if (vib->pdata->mode_ctrl == CLK_MODE) {
			mutex_lock(&vib->lock_clk);
			/* de-vote clock */
			if (vib->clk_on && vib->pwm_clk) {
				clk_disable_unprepare(vib->pwm_clk);
				vib->clk_on = false;
			}
			mutex_unlock(&vib->lock_clk);
		}
	}
	gpio_set_value(vib->pdata->en_gpio, !!on);

err_en_clk:
	return ret;
}

static void drv2603_vib_enable(struct timed_output_dev *dev, int value)
{
	struct drv2603_vib *vib = container_of(dev, struct drv2603_vib,
					timed_dev);
	unsigned long flags = 0;
retry:
	spin_lock_irqsave(&vib->lock, flags);

	if (hrtimer_try_to_cancel(&vib->vib_timer) < 0) {
		spin_unlock_irqrestore(&vib->lock, flags);
		cpu_relax();
		goto retry;
	}

	pr_debug("[%s] value:%d level:%d\n", __func__, value, vib->pwm_level);
	if (value == 0) {
		if (vib->pdata->mode_ctrl == PWM_MODE) {
			pwm_disable(vib->vib_pwm);
		} else if (vib->pdata->mode_ctrl == CLK_MODE) {
			mutex_lock(&vib->lock_clk);
			/* de-vote clock */
			if (vib->clk_on && vib->pwm_clk) {
				clk_disable_unprepare(vib->pwm_clk);
				vib->clk_on = false;
			}
			mutex_unlock(&vib->lock_clk);
		}
		gpio_set_value(vib->pdata->en_gpio, 0);
		spin_unlock_irqrestore(&vib->lock, flags);
		return;
	} else {
		value = (value > vib->pdata->max_timeout_ms ?
			vib->pdata->max_timeout_ms : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&vib->lock, flags);
	schedule_work(&vib->work);
}

static void drv2603_vib_update(struct work_struct *work)
{
	struct drv2603_vib *vib = container_of(work, struct drv2603_vib,
					 work);
	drv2603_vib_set(vib,vib->state);
}


static int drv2603_vib_get_time(struct timed_output_dev *dev)
{
	struct drv2603_vib *vib = container_of(dev, struct drv2603_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	}
	return 0;
}

static enum hrtimer_restart drv2603_vib_timer_func(struct hrtimer *timer)
{
	struct drv2603_vib *vib = container_of(timer, struct drv2603_vib,
							 vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);
	return HRTIMER_NORESTART;
}

/* Device parts */
static ssize_t level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct timed_output_dev *tdev = container_of(dev, struct timed_output_dev, dev);
	struct drv2603_vib *vib = g_vib;//container_of(dev, struct drv2603_vib, dev);

	return sprintf(buf, "%d\n", vib->pwm_level);
}

static ssize_t level_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
//	struct timed_output_dev *tdev = container_of(dev, struct timed_output_dev, dev);
	struct drv2603_vib *vib = g_vib;//container_of(tdev, struct drv2603_vib, timed_dev);
	int value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	if (value < PWM_LEVEL_LOW || value > PWM_LEVEL_FULL)
		return -EINVAL;

	vib->pwm_level = value;

	return size;
}
static DEVICE_ATTR(level, S_IRUGO | S_IWUSR, level_show, level_store);

/* Driver parts */
static int drv2603_init_gpio(struct drv2603_vib *vib)
{
	int rc = 0;

	rc = gpio_request(vib->pdata->en_gpio, "vib_enable_gpio");
	if (rc) {
		pr_err("[%s] request gpio(%d) failed, rc=%d\n", __func__, vib->pdata->en_gpio, rc);
		return -ENODEV;
	}
	gpio_direction_output(vib->pdata->en_gpio, 0);

	rc = gpio_request(vib->pdata->pwm_gpio, "vib_pwm_gpio");
	if (rc) {
		pr_err("[%s] request gpio(%d) failed, rc=%d\n", __func__, vib->pdata->pwm_gpio, rc);
		gpio_free(vib->pdata->en_gpio);
		return -ENODEV;
	}

	if (vib->pdata->mode_ctrl == PWM_MODE) {
		vib->vib_pwm = pwm_request(vib->pdata->pwm_channel, "Vib-pwm");
		if (vib->vib_pwm == NULL || IS_ERR(vib->vib_pwm)) {
			pr_err("[%s] pwm_request() failed. vib->vib_pwm=%d.\n", __func__, (int)vib->vib_pwm);
			vib->vib_pwm = NULL;
			gpio_free(vib->pdata->pwm_gpio);
			gpio_free(vib->pdata->en_gpio);
			return -EIO;
		}
	} else if (vib->pdata->mode_ctrl == CLK_MODE) {
		vib->pwm_clk = clk_get(vib->dev, "pwm_clk");
		if (IS_ERR(vib->pwm_clk)) {
			dev_err(vib->dev, "[%s] pwm_clk get failed\n", __func__);
			vib->pwm_clk = NULL;
		}
	} else {
		pr_err("[%s] Unknown mode:%d\n", __func__, vib->pdata->mode_ctrl);
		return -EINVAL;
	}

	return 0;
}

static void drv2603_deinit_gpio(struct drv2603_vib *vib)
{
	if (vib->pdata->mode_ctrl == PWM_MODE) {
		pwm_free(vib->vib_pwm);
	} else if (vib->pdata->mode_ctrl == CLK_MODE) {
		if (vib->pwm_clk)
			clk_put(vib->pwm_clk);
	}
	gpio_free(vib->pdata->pwm_gpio);
	gpio_free(vib->pdata->en_gpio);
}

static void parse_dts_to_pdata(struct drv2603_vibrator_platform_data *pdata, struct device_node *dev_node)
{
	int ret = 0;
	u32 tmp_value = 0;

	//must settings
	ret = of_get_named_gpio(dev_node, "drv2603,en_gpio", 0);
	if (ret < 0)
		pdata->en_gpio = -1;
	else
		pdata->en_gpio = ret;
	ret = of_get_named_gpio(dev_node, "drv2603,pwm_gpio", 0);
	if (ret < 0)
		pdata->pwm_gpio = -1;
	else
		pdata->pwm_gpio = ret;
	ret = of_property_read_u32(dev_node, "drv2603,mode_ctrl", &tmp_value);
	if (ret < 0)
		pdata->mode_ctrl = PWM_MODE;
	else
		pdata->mode_ctrl = tmp_value;

	//option settings
	ret = of_property_read_u32(dev_node, "drv2603,default_level", &tmp_value);
	if (ret < 0)
		pdata->default_level = 0;
	else
		pdata->default_level = tmp_value;
	ret = of_property_read_u32(dev_node, "drv2603,max_timeout_ms", &tmp_value);
	if (ret < 0)
		pdata->max_timeout_ms = DEFAULT_MAX_TIMEOUT;
	else
		pdata->max_timeout_ms = tmp_value;
	ret = of_property_read_u32(dev_node, "drv2603,vib_break_ms", &tmp_value);
	if (ret < 0)
		pdata->vib_break_ms = DEFAULT_VIB_BRAKE_INTERVAL;
	else
		pdata->vib_break_ms = tmp_value;

	//pwm mode
	ret = of_property_read_u32(dev_node, "drv2603,pwm_channel", &tmp_value);
	if (ret < 0)
		pdata->pwm_channel = -1;
	else
		pdata->pwm_channel = tmp_value;
	ret = of_property_read_u32(dev_node, "drv2603,pwm_period_us", &tmp_value);
	if (ret < 0)
		pdata->pwm_period_us = 0;
	else
		pdata->pwm_period_us = tmp_value;
}

static int __devinit drv2603_vib_probe(struct platform_device *pdev)
{
	struct drv2603_vibrator_platform_data *pdata =
						pdev->dev.platform_data;
	struct device_node *dev_node = pdev->dev.of_node;
	struct drv2603_vib *vib;
	int rc;
	int pdata_alloc = 0;

	pr_info("[%s]\n", __func__);

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
	if (!vib) {
		return -ENOMEM;
	}

	if (!pdata && !dev_node) {
		pr_err("[%s] No pdata and dts\n", __func__);
		goto err_dts;
	} else if (!pdata) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			goto err_pdata;
		pdata_alloc = 1;
	}

	if (dev_node)
		parse_dts_to_pdata(pdata, dev_node);

	vib->pdata	= pdata;
	vib->dev	= &pdev->dev;
	if (pdata->default_level)
		vib->pwm_level = pdata->default_level;
	else
		vib->pwm_level = PWM_LEVEL_FULL;
	if (pdata->mode_ctrl == PWM_MODE) {
		if (pdata->pwm_period_us)
			vib->pwm_period_us = pdata->pwm_period_us;
		else
			vib->pwm_period_us = DEFAULT_PWM_PERIOD_US;
		pr_info("[%s] vib_break:%u, pwm_period:%u, pwm_level:%d\n", __func__,
			vib->vib_break_ms, vib->pwm_period_us, vib->pwm_level);
	} else if (pdata->mode_ctrl == CLK_MODE) {
		vib->clk_on = false;
	}

	rc = drv2603_init_gpio(vib);
	if (rc)
		goto err_gpio;

	spin_lock_init(&vib->lock);
	mutex_init(&vib->lock_clk);
	INIT_WORK(&vib->work, drv2603_vib_update);
	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = drv2603_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = drv2603_vib_get_time;
	vib->timed_dev.enable = drv2603_vib_enable;
	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		goto err_read_vib;

	rc = device_create_file(vib->timed_dev.dev, &dev_attr_level);
	if (rc < 0)
		goto err_create_level;

	platform_set_drvdata(pdev, vib);
	g_vib = vib;
	return 0;

err_create_level:
	timed_output_dev_unregister(&vib->timed_dev);
err_read_vib:
err_gpio:
	if (pdata_alloc)
		kfree(pdata);
err_pdata:
err_dts:
	kfree(vib);
	BBOX_VIBRATOR_PROBE_FAIL;
	return rc;
}


static int __devexit drv2603_vib_remove(struct platform_device *pdev)
{
	struct drv2603_vib *vib = platform_get_drvdata(pdev);

	device_remove_file(vib->timed_dev.dev, &dev_attr_level);
	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	/* turn-off vibrator */
	drv2603_vib_set(vib, 0);
	timed_output_dev_unregister(&vib->timed_dev);
	drv2603_deinit_gpio(vib);
	platform_set_drvdata(pdev, NULL);
	g_vib = NULL;
	kfree(vib);

	return 0;
}

#ifdef CONFIG_PM
static int drv2603_vib_suspend(struct device *dev)
{
	struct drv2603_vib *vib = dev_get_drvdata(dev);
	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	drv2603_vib_set(vib, 0);

	return 0;
}

static const struct dev_pm_ops drv2603_vib_pm_ops = {
	.suspend = drv2603_vib_suspend,
};
#endif

static const struct of_device_id drv2603_match[] = {
	{.compatible = "ti,drv2603"},
	{}
};

static struct platform_driver drv2603_vib_driver = {
	.probe		= drv2603_vib_probe,
	.remove		= __devexit_p(drv2603_vib_remove),
	.driver		= {
		.name	= DRV2603_VIBRATOR_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = drv2603_match,
#ifdef CONFIG_PM
		.pm	= &drv2603_vib_pm_ops,
#endif
	},
};

static int __init drv2603_vib_init(void)
{
	int ret=0;
	pr_info("[%s]\n", __func__);

	ret = platform_driver_register(&drv2603_vib_driver);
	if(ret < 0)
		BBOX_VIBRATOR_PROBE_FAIL;

	return ret;
}
module_init(drv2603_vib_init);

static void __exit drv2603_vib_exit(void)
{
	platform_driver_unregister(&drv2603_vib_driver);
}
module_exit(drv2603_vib_exit);

MODULE_ALIAS("platform:" DRV2603_VIBRATOR_DEV_NAME);
MODULE_DESCRIPTION("DRV2603 vibrator driver");
MODULE_LICENSE("GPL v2");
