/*
 *  FIH headset device detection driver.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <asm/atomic.h>

#include <sound/soc.h>
#include <sound/jack.h>

#include <linux/of_gpio.h>


#define FIH_JACK_MASK (SND_JACK_HEADSET | SND_JACK_OC_HPHL | \
				SND_JACK_OC_HPHR | SND_JACK_LINEOUT | \
				SND_JACK_UNSUPPORTED)
#define FIH_JACK_BUTTON_MASK (SND_JACK_BTN_0 | SND_JACK_BTN_1 | \
				SND_JACK_BTN_2 | SND_JACK_BTN_3 | \
				SND_JACK_BTN_4 | SND_JACK_BTN_5 | \
				SND_JACK_BTN_6 | SND_JACK_BTN_7)

#define LEGACY_SWITCH_DEV_SUPPORT

#define IRQF_TRIGGER_HS_INSERTED IRQF_TRIGGER_HIGH
#define IRQF_TRIGGER_BTN_PRESSED IRQF_TRIGGER_LOW


struct fih_headset_platform_data {
	int headset_gpio;
	u32 headset_gpio_flags;
	int button_gpio;
	u32 button_gpio_flags;
	bool headset_gpio_insert_low;
	bool button_gpio_press_low;
};

struct fih_headset_priv {
#ifdef LEGACY_SWITCH_DEV_SUPPORT
	struct switch_dev sdev_insert;
	struct switch_dev sdev_button;
#endif
	struct snd_soc_jack headset_jack;
	struct snd_soc_jack button_jack;
	struct snd_soc_codec *codec;

	unsigned int headset_gpio;
	unsigned int headset_gpio_irq;
	int gpio_level_insert;
	unsigned int button_gpio;
	unsigned int button_gpio_irq;
	int gpio_level_press;

	atomic_t btn_state;
	u32 hph_status; /* track headhpone status */
	int micbias_force_enable;
	int ignore_btn;
	int bn_irq_enable;

	struct hrtimer headset_timer;
	ktime_t headset_debounce_time;

	struct hrtimer button_timer;
	ktime_t button_debounce_time;

	struct workqueue_struct *detection_workqueue;
	struct work_struct detection_work;
	struct wake_lock hs_wakelock;
};


static struct fih_headset_priv *fih_headset;


static int fih_headset_enable_micbias(struct snd_soc_codec *codec, bool enable)
{
	int rc;

	pr_debug("%s: enable=%d\n", __func__, enable);

	if (enable)
		rc = snd_soc_dapm_force_enable_pin(&codec->dapm,
			"MIC BIAS2 External");
	else
		rc = snd_soc_dapm_disable_pin(&codec->dapm,
			"MIC BIAS2 External");

	if (rc)
		pr_err("%s: dapm function fail\n", __func__);

	snd_soc_dapm_sync(&codec->dapm);
	fih_headset->micbias_force_enable = enable;

	return rc;
}

static void insert_headset(void)
{
	unsigned long irq_flags;

	pr_debug("%s: +\n", __func__);

	if (gpio_get_value(fih_headset->headset_gpio) != fih_headset->gpio_level_insert) {
		pr_info("%s: abort\n", __func__);
		goto exit;
	}

	/* On some non-standard headset adapters (usually those without a
	 * button) the btn line is pulled down at the same time as the detect
	 * line. We can check here by sampling the button line, if it is
	 * low then it is probably a bad adapter so ignore the button.
	 * If the button is released then we stop ignoring the button, so that
	 * the user can recover from the situation where a headset is plugged
	 * in with button held down.
	 */

	if (fih_headset->button_gpio) {
		fih_headset_enable_micbias(fih_headset->codec, true);
		msleep(100);

		fih_headset->ignore_btn = !gpio_get_value(fih_headset->button_gpio);

		if (gpio_get_value(fih_headset->button_gpio) == fih_headset->gpio_level_press) {
			pr_info("%s: headphone inserted\n", __func__);

#ifdef LEGACY_SWITCH_DEV_SUPPORT
			switch_set_state(&fih_headset->sdev_insert, 2);
#endif
			fih_headset->hph_status = SND_JACK_HEADPHONE;
			snd_soc_jack_report_no_dapm(&fih_headset->headset_jack, fih_headset->hph_status, FIH_JACK_MASK);

			if (fih_headset->bn_irq_enable == 1) {
				/* Disable button */
				local_irq_save(irq_flags);
				disable_irq(fih_headset->button_gpio_irq);
				irq_set_irq_wake(fih_headset->button_gpio_irq, 0);
				local_irq_restore(irq_flags);
				fih_headset->bn_irq_enable = 0;
			}
		} else {
			pr_info("%s: headset inserted\n", __func__);

#ifdef LEGACY_SWITCH_DEV_SUPPORT
			switch_set_state(&fih_headset->sdev_insert, 1);
#endif
			fih_headset->hph_status = SND_JACK_HEADSET;
			snd_soc_jack_report_no_dapm(&fih_headset->headset_jack, fih_headset->hph_status, FIH_JACK_MASK);

			/* Enable button irq */
			if (fih_headset->bn_irq_enable == 0) {
				local_irq_save(irq_flags);
				irq_set_irq_type(fih_headset->button_gpio_irq, (gpio_get_value(fih_headset->button_gpio) ?
						 IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
				enable_irq(fih_headset->button_gpio_irq);
				local_irq_restore(irq_flags);
				fih_headset->bn_irq_enable = 1;
			}
		}
	} else {
		pr_info("%s: headphone inserted\n", __func__);

#ifdef LEGACY_SWITCH_DEV_SUPPORT
		switch_set_state(&fih_headset->sdev_insert, 2);
#endif
		fih_headset->hph_status = SND_JACK_HEADPHONE;
		snd_soc_jack_report_no_dapm(&fih_headset->headset_jack, fih_headset->hph_status, FIH_JACK_MASK);

		if (fih_headset->bn_irq_enable == 1) {
			/* Disable button */
			local_irq_save(irq_flags);
			disable_irq(fih_headset->button_gpio_irq);
			irq_set_irq_wake(fih_headset->button_gpio_irq, 0);
			local_irq_restore(irq_flags);
			fih_headset->bn_irq_enable = 0;
		}
	}

exit:
	pr_debug("%s: -\n", __func__);
}

static void remove_headset(void)
{
	unsigned long irq_flags;

	pr_debug("%s: +\n", __func__);

	if (gpio_get_value(fih_headset->headset_gpio) == fih_headset->gpio_level_insert) {
		pr_info("%s: abort\n", __func__);
		goto exit;
	}

	if (fih_headset->button_gpio) {
		if (fih_headset->bn_irq_enable == 1) {
			/* Disable button */
			local_irq_save(irq_flags);
			disable_irq(fih_headset->button_gpio_irq);
			irq_set_irq_wake(fih_headset->button_gpio_irq, 0);
			local_irq_restore(irq_flags);
			fih_headset->bn_irq_enable = 0;
		}

		if (atomic_read(&fih_headset->btn_state)) {
#ifdef LEGACY_SWITCH_DEV_SUPPORT
			switch_set_state(&fih_headset->sdev_button, 0);
#endif
			atomic_set(&fih_headset->btn_state, 0);
			snd_soc_jack_report_no_dapm(&fih_headset->button_jack, 0, FIH_JACK_BUTTON_MASK);
		}

		fih_headset_enable_micbias(fih_headset->codec, false);
	}

	pr_info("%s: headphone removed\n", __func__);

#ifdef LEGACY_SWITCH_DEV_SUPPORT
	switch_set_state(&fih_headset->sdev_insert, 0);
#endif
	fih_headset->hph_status = 0;
	snd_soc_jack_report_no_dapm(&fih_headset->headset_jack, fih_headset->hph_status, FIH_JACK_MASK);

exit:
	pr_debug("%s: -\n", __func__);
}

static void detection_work(struct work_struct *work)
{
	pr_debug("%s: +\n", __func__);

	if (gpio_get_value(fih_headset->headset_gpio) != fih_headset->gpio_level_insert) {
		/* Headset not plugged in */
		if (fih_headset->hph_status != 0) {
			remove_headset();
		} else
			pr_info("%s: already removed\n", __func__);
	} else {
		if (fih_headset->hph_status == 0) {
			insert_headset();
		} else
			pr_info("%s: already inserted\n", __func__);
	}

	pr_debug("%s: -\n", __func__);
}

static enum hrtimer_restart button_event_timer_func(struct hrtimer *data)
{
	pr_debug("%s: +\n", __func__);

	if (fih_headset->hph_status == SND_JACK_HEADSET) {
		if (gpio_get_value(fih_headset->button_gpio) != fih_headset->gpio_level_press) {
			if (fih_headset->ignore_btn)
				fih_headset->ignore_btn = 0;
			else if (atomic_read(&fih_headset->btn_state)) {
				pr_info("%s: button released\n", __func__);

#ifdef LEGACY_SWITCH_DEV_SUPPORT
				switch_set_state(&fih_headset->sdev_button, 1);
#endif
				atomic_set(&fih_headset->btn_state, 0);
				snd_soc_jack_report_no_dapm(&fih_headset->button_jack, 0, FIH_JACK_BUTTON_MASK);
			}
		} else {
			if (!fih_headset->ignore_btn && !atomic_read(&fih_headset->btn_state)) {
				pr_info("%s: button pressed\n", __func__);

#ifdef LEGACY_SWITCH_DEV_SUPPORT
				switch_set_state(&fih_headset->sdev_button, 1);
#endif
				atomic_set(&fih_headset->btn_state, 1);
				snd_soc_jack_report_no_dapm(&fih_headset->button_jack, SND_JACK_BTN_0, FIH_JACK_BUTTON_MASK);
			}
		}
	}

	pr_debug("%s: -\n", __func__);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart headset_event_timer_func(struct hrtimer *data)
{
	pr_debug("%s: +\n", __func__);

	queue_work(fih_headset->detection_workqueue, &fih_headset->detection_work);

	pr_debug("%s: -\n", __func__);
	return HRTIMER_NORESTART;
}

static irqreturn_t headset_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	pr_debug("%s: +\n", __func__);

	do {
		value1 = gpio_get_value(fih_headset->headset_gpio);
		pr_debug("%s: value1=%d\n", __func__, value1);
		irq_set_irq_type(fih_headset->headset_gpio_irq, value1 ?
				 IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(fih_headset->headset_gpio);
		pr_debug("%s: value2=%d\n", __func__, value2);
	} while (value1 != value2 && retry_limit-- > 0);

	/*
	* If the sdev is NO_DEVICE, and we detect the headset has been plugged,
	* then we can do headset_insertion check.
	*/
	if ((fih_headset->hph_status == 0) ^ (value2 ^ fih_headset->gpio_level_insert)) {
		if (fih_headset->hph_status == SND_JACK_HEADSET)
			fih_headset->ignore_btn = 1;

		wake_lock_timeout(&fih_headset->hs_wakelock, 10 * HZ);
		/* Do the rest of the work in timer context */
		hrtimer_start(&fih_headset->headset_timer, fih_headset->headset_debounce_time, HRTIMER_MODE_REL);
	}

	pr_debug("%s: -\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	pr_debug("%s: +\n", __func__);

	do {
		value1 = gpio_get_value(fih_headset->button_gpio);
		pr_debug("%s: value1=%d\n", __func__, value1);
		irq_set_irq_type(fih_headset->button_gpio_irq, value1 ?
				 IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(fih_headset->button_gpio);
		pr_debug("%s: value2=%d\n", __func__, value2);
	} while (value1 != value2 && retry_limit-- > 0);

	hrtimer_start(&fih_headset->button_timer, fih_headset->button_debounce_time, HRTIMER_MODE_REL);

	pr_debug("%s: -\n", __func__);
	return IRQ_HANDLED;
}

int fih_hs_detect(struct snd_soc_codec *codec)
{
	int ret;

	pr_debug("%s: start detect\n", __func__);

	fih_headset->codec = codec;

	ret = snd_soc_jack_new(codec, "FIH Headset Jack",
			       (SND_JACK_HEADSET |  SND_JACK_LINEOUT |
				SND_JACK_OC_HPHL |  SND_JACK_OC_HPHR |
				SND_JACK_UNSUPPORTED),
			       &fih_headset->headset_jack);
	if (ret) {
		pr_err("%s: failed to create new jack\n", __func__);
		return ret;
	}

	ret = snd_soc_jack_new(codec, "FIH Button Jack",
			       FIH_JACK_BUTTON_MASK, &fih_headset->button_jack);
	if (ret) {
		pr_err("%s: failed to create new jack\n", __func__);
		return ret;
	}

	ret = snd_jack_set_key(fih_headset->button_jack.jack,
			       SND_JACK_BTN_0,
			       KEY_MEDIA);
	if (ret) {
		pr_err("%s: failed to set code for btn-0\n", __func__);
		return ret;
	}

	if (fih_headset->headset_gpio) {
		/* When headset inserted, gpio H->L, so we detect LOW level */
		ret = request_irq(fih_headset->headset_gpio_irq, headset_irq_handler,
				  IRQF_TRIGGER_HS_INSERTED, "headset_detect", NULL);
		if (ret < 0) {
			pr_err("%s: fail to request headset irq\n", __func__);
			goto err_free_headset_irq;
		}

		// Set headset_detect pin as wake up pin
		ret = irq_set_irq_wake(fih_headset->headset_gpio_irq, 1);
		if (ret < 0) {
			pr_err("%s: fail to set headset irq wake\n", __func__);
			goto err_free_headset_irq;
		}
	}

	if (fih_headset->button_gpio) {
		/* Disable button until plugged in */
		set_irq_flags(fih_headset->button_gpio_irq, IRQF_VALID | IRQF_NOAUTOEN);

		ret = request_irq(fih_headset->button_gpio_irq, button_irq_handler,
				  IRQF_TRIGGER_BTN_PRESSED, "button_detect", NULL);
		if (ret < 0) {
			pr_err("%s: fail to request button irq\n", __func__);
			goto err_free_button_irq;
		}

		if (gpio_get_value_cansleep(fih_headset->headset_gpio) == fih_headset->gpio_level_insert) {
			ret = irq_set_irq_wake(fih_headset->button_gpio_irq, 1);
			if (ret < 0) {
				pr_err("%s: fail to set button irq wake\n", __func__);
				goto err_free_button_irq;
			}
		}
	}

	/* Add the headset detection when booting */
	hrtimer_start(&fih_headset->headset_timer, ktime_set(1, 0) /* 1s */, HRTIMER_MODE_REL);

	return 0;

err_free_button_irq:
	if (fih_headset->button_gpio)
		free_irq(fih_headset->button_gpio_irq, 0);
err_free_headset_irq:
	if (fih_headset->headset_gpio)
		free_irq(fih_headset->headset_gpio_irq, 0);

	return ret;
}
EXPORT_SYMBOL_GPL(fih_hs_detect);

#ifdef CONFIG_OF
static int fih_headset_parse_dt(struct device *dev, struct fih_headset_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	/* headset detect gpio */
	pdata->headset_gpio = of_get_named_gpio_flags(np, "fih,headset-gpio",
				0, &pdata->headset_gpio_flags);
	pr_debug("%s: headset_gpio=%d\n", __func__, pdata->headset_gpio);

	/* button detect gpio */
	pdata->button_gpio = of_get_named_gpio_flags(np, "fih,button-gpio",
				0, &pdata->button_gpio_flags);
	pr_debug("%s: button_gpio=%d\n", __func__, pdata->button_gpio);

	pdata->headset_gpio_insert_low = of_property_read_bool(np, "fih,headset-gpio-insert-low");
	pr_debug("%s: headset_gpio_insert_low=%d\n", __func__, pdata->headset_gpio_insert_low);

	pdata->button_gpio_press_low = of_property_read_bool(np, "fih,button-gpio-press-low");
	pr_debug("%s: button_gpio_press_low=%d\n", __func__, pdata->button_gpio_press_low);

	return 0;
}
#else
static int fih_headset_parse_dt(struct device *dev, struct fih_headset_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int __devinit fih_headset_probe(struct platform_device *pdev)
{
	struct fih_headset_platform_data *pdata;
	int ret;

	pr_debug("%s: start probe\n", __func__);

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(struct fih_headset_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		ret = fih_headset_parse_dt(&pdev->dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = pdev->dev.platform_data;

	if (!pdata) {
		pr_err("%s: no platform data\n", __func__);
		return -ENODATA;
	}

	fih_headset = kzalloc(sizeof(struct fih_headset_priv), GFP_KERNEL);
	if (!fih_headset) {
		pr_err("%s: failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

#ifdef LEGACY_SWITCH_DEV_SUPPORT
	fih_headset->sdev_insert.name = "h2w";
	ret = switch_dev_register(&fih_headset->sdev_insert);
	if (ret)
		pr_err("%s: fail to register switch device for headset\n", __func__);

	fih_headset->sdev_button.name = "btn";
	ret = switch_dev_register(&fih_headset->sdev_button);
	if (ret)
		pr_err("%s: fail to register switch device for button\n", __func__);
#endif

	fih_headset->headset_gpio = pdata->headset_gpio;
	if (fih_headset->headset_gpio) {
		fih_headset->headset_gpio_irq = gpio_to_irq(fih_headset->headset_gpio);
		ret = gpio_request(fih_headset->headset_gpio, "headset_detect");
		if (ret < 0) {
			pr_err("%s: fail to request headset gpio %d\n", __func__, fih_headset->headset_gpio);
			goto err_free_headset_gpio;
		}
		gpio_direction_input(fih_headset->headset_gpio);

		fih_headset->gpio_level_insert = pdata->headset_gpio_insert_low ? 0 : 1;
	}

	fih_headset->button_gpio = pdata->button_gpio;
	if (fih_headset->button_gpio) {
		fih_headset->button_gpio_irq = gpio_to_irq(fih_headset->button_gpio);
		ret = gpio_request(fih_headset->button_gpio, "button_detect");
		if (ret < 0) {
			pr_err("%s: fail to request button gpio %d\n", __func__, fih_headset->button_gpio);
			goto err_free_button_gpio;
		}
		gpio_direction_input(fih_headset->button_gpio);

		fih_headset->gpio_level_press = pdata->button_gpio_press_low ? 0 : 1;
	}

	fih_headset->hph_status = 0;
	fih_headset->micbias_force_enable = 0;
	atomic_set(&fih_headset->btn_state, 0);
	fih_headset->ignore_btn = 0;
	fih_headset->bn_irq_enable = 0;

	fih_headset->detection_workqueue = create_workqueue("detection");
	if (fih_headset->detection_workqueue == NULL) {
		pr_err("%s: fail to create workqueue\n", __func__);
		goto err_destroy_workqueue;
	}

	INIT_WORK(&fih_headset->detection_work, detection_work);

	wake_lock_init(&fih_headset->hs_wakelock, WAKE_LOCK_SUSPEND, "hs_wakelock");

	if (fih_headset->headset_gpio) {
		hrtimer_init(&fih_headset->headset_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		fih_headset->headset_timer.function = headset_event_timer_func;

		fih_headset->headset_debounce_time = ktime_set(0, 500000000);  /* 500 ms */
	}
	if (fih_headset->button_gpio) {
		hrtimer_init(&fih_headset->button_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		fih_headset->button_timer.function = button_event_timer_func;

		fih_headset->button_debounce_time = ktime_set(0, 80000000); /* 80 ms */
	}

	return 0;

err_destroy_workqueue:
	destroy_workqueue(fih_headset->detection_workqueue);
err_free_button_gpio:
	if (fih_headset->button_gpio)
		gpio_free(fih_headset->button_gpio);
err_free_headset_gpio:
	if (fih_headset->headset_gpio)
		gpio_free(fih_headset->headset_gpio);

	return ret;
}

static int __devexit fih_headset_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id fih_headset_id[] = {
	{"fih_headset", 0},
	{},
};

#ifdef CONFIG_OF
static struct of_device_id fih_headset_match_table[] = {
	{ .compatible = "fih,headset",},
	{ },
};
#else
#define fih_headset_match_table NULL
#endif

static struct platform_driver fih_headset_driver = {
	.driver = {
		.name = "fih_headset",
		.owner = THIS_MODULE,
		.of_match_table = fih_headset_match_table,
	},
	.probe = fih_headset_probe,
	.remove = __devexit_p(fih_headset_remove),
	.id_table = fih_headset_id,
};

static int __init fih_headset_init(void)
{
	return platform_driver_register(&fih_headset_driver);
}

static void __exit fih_headset_exit(void)
{
	platform_driver_unregister(&fih_headset_driver);
}

module_init(fih_headset_init);
module_exit(fih_headset_exit);

MODULE_AUTHOR("Seven Lin <sevenlin@fihspec.com>");
MODULE_DESCRIPTION("FIH headset detection driver");
MODULE_LICENSE("Proprietary");
