#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

struct hall_info {
	unsigned irq_gpio;
	unsigned long irq_flags;
	int (*gpio_config)(unsigned gpio, bool configure);
	struct mutex hall_mutex;
	int irq;
	struct work_struct irq_work;
	struct input_dev *input;
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif
};

//enum backlight_status {
//	BL_OFF = 0,
//	BL_ON
//};

enum hall_status {
	HALL_CLOSED = 0,
	HALL_OPENED
};

struct hall_event_info {
//	enum backlight_status bl_status;
	enum hall_status current_status;
	unsigned int disable_event;
};

static struct hall_event_info gHallEventInfo;

static int hall_gpio_setup(unsigned gpio, bool configure)
{
	int retval=0;
	if (configure)
	{
		retval = gpio_request(gpio, "hall_sensor");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
				__func__, gpio, retval);
			return retval;
		}
		retval = gpio_direction_input(gpio);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
				__func__, gpio, retval);
			gpio_free(gpio);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
				__func__, gpio);
	}
	return retval;
}

static struct hall_info hallsensor_info = {
	.irq_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
	.irq_gpio = 0,
	.gpio_config = hall_gpio_setup,
};

static ssize_t enable_switch_store (struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int enable;
	unsigned long data;

	enable = (unsigned int)kstrtoul(buf, 10, &data);
	gHallEventInfo.disable_event = enable;

	return count;
}

static ssize_t enable_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int enable = gHallEventInfo.disable_event;

	return sprintf(buf, "%d", enable);
}


static ssize_t hall_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int hall_status;
	hall_status = gpio_get_value(hallsensor_info.irq_gpio);

	//printk(KERN_INFO "hall_status(%d)\n", hall_status);
	return sprintf(buf, "%d\n", hall_status);
}
static DEVICE_ATTR(Hall_status, 0644, hall_show, NULL);
static DEVICE_ATTR(Hall_enable, 0644, enable_switch_show, enable_switch_store);

static irqreturn_t hall_irq_handler(int irq, void *handle)
{
	struct hall_info *pdata = handle;

	if(pdata == NULL)
		return IRQ_HANDLED;

	schedule_work(&pdata->irq_work);
	//pr_info("[Hall sensor] : %s\n",__FUNCTION__);


	return IRQ_HANDLED;
}

static void irq_work_func(struct work_struct *work)
{
	struct hall_info *pdata = container_of((struct work_struct *)work,
                        struct hall_info, irq_work);

	pr_info("[Hall sensor] : irq triggered ! \n");
	pr_info("gpio status : %d\n" , gpio_get_value(hallsensor_info.irq_gpio));

	if(gHallEventInfo.disable_event)
		return;

	gHallEventInfo.current_status = gpio_get_value(hallsensor_info.irq_gpio);

	if(gHallEventInfo.current_status == HALL_CLOSED)
	{
		input_report_key(pdata->input, KEY_HALL_CLOSE, 1);
		input_sync(pdata->input);
		input_report_key(pdata->input, KEY_HALL_CLOSE, 0);
		input_sync(pdata->input);
	}
	else if(gHallEventInfo.current_status == HALL_OPENED)
	{
		input_report_key(pdata->input, KEY_HALL_OPEN, 1);
		input_sync(pdata->input);
		input_report_key(pdata->input, KEY_HALL_OPEN, 0);
		input_sync(pdata->input);
	}
	else
	{
		pr_err("[Hall sensor] : gpio status fail\n");
	}

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hall_early_suspend(struct early_suspend *h)
{
	//pr_info("Hall sensor early suspend\n");
	//gHallEventInfo.bl_status = BL_OFF;
	return;
}

static void hall_late_resume(struct early_suspend *h)
{
	//pr_info("Hall sensor late resume\n");
	//gHallEventInfo.bl_status = BL_ON;
	return;
}
#endif

static int hall_suspend(struct device *dev)
{
	//pr_info("Hall sensor suspend\n");
	//gHallEventInfo.bl_status = BL_OFF;
	return 0;
}

static int hall_resume(struct device *dev)
{
	//pr_info("Hall sensor resume\n");
	//gHallEventInfo.bl_status = BL_ON;
	return  0;
}

static  int hall_sensor_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np=pdev->dev.of_node;
	struct hall_info *pdata = &hallsensor_info;

	pr_info("[Hall sensor] : %s enter\n",__FUNCTION__);

	pdata->irq_gpio = of_get_named_gpio(np, "hallsensor,irq-gpio", 0);
	mutex_init(&(pdata->hall_mutex));

	if (pdata->gpio_config) {
		ret = pdata->gpio_config(pdata->irq_gpio, true);
		if (ret < 0) {
			pr_err("%s: Failed to configure GPIO\n", __func__);
			return ret;
		}
	}

	pdata->irq = gpio_to_irq(pdata->irq_gpio);
	pdata->input = input_allocate_device();
	if (!pdata->input) {
		pr_err( "Not enough memory for input device\n");
		return -ENOMEM;
	}
	pdata->input->name = "hallsensor";

	set_bit(EV_KEY,pdata->input->evbit);
	set_bit(KEY_HALL_OPEN, pdata->input->keybit);
	set_bit(KEY_HALL_CLOSE, pdata->input->keybit);

	ret = input_register_device(pdata->input);
	if (ret) {
		pr_err("%s: Failed to register input device\n", __func__);
		return ret;
	}

	ret = request_irq(pdata->irq, hall_irq_handler, pdata->irq_flags, "hall_sensor", pdata);
	if (ret < 0) {
		pr_err("%s: Failed to request_threaded_irq\n",__func__);
		return ret;
	}

	disable_irq_nosync(pdata->irq);
	enable_irq(pdata->irq);
	enable_irq_wake(pdata->irq);

	INIT_WORK(&pdata->irq_work, irq_work_func);

	ret = device_create_file(&pdata->input->dev, &dev_attr_Hall_status);
	if (ret) {
		dev_err(&pdev->dev, "%s: dev_attr_Hall_status failed\n", __func__);
		return ret;
	}
	ret = device_create_file(&pdata->input->dev, &dev_attr_Hall_enable);
	if (ret) {
		dev_err(&pdev->dev, "%s: dev_arrt_Hall_enable failed\n", __func__);
	}

	ret = sysfs_create_link(pdata->input->dev.kobj.parent,
				&pdata->input->dev.kobj,
				pdata->input->name);
	if (ret < 0) {
		pr_info("[Hall sensor] %s Can't create soft link in hall driver\n", __func__);
	}

	//gHallEventInfo.bl_status = BL_ON;
	gHallEventInfo.current_status = gpio_get_value(hallsensor_info.irq_gpio);
	gHallEventInfo.disable_event = false;

	printk(KERN_INFO "[Hall sensor] : %s exit\n",__FUNCTION__);

	return ret;
}

static int hall_sensor_remove(struct platform_device *pdev)
{
	struct hall_info *pdata = &hallsensor_info;

	printk(KERN_INFO "[Hall sensor] : %s\n",__FUNCTION__);
	input_unregister_device(pdata->input);
	free_irq(pdata->irq, 0);

	return 0;
}

static const struct of_device_id hall_sensor_device_of_match[]  = {
	{ .compatible = "fih,hallsensor", },
	{},
};

static const struct dev_pm_ops hall_sensor_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
        .suspend = hall_suspend,
        .resume = hall_resume,
#endif
};


static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = "hall_sensor",
		.owner = THIS_MODULE,
		.of_match_table = hall_sensor_device_of_match,
#ifdef CONFIG_PM
		.pm = &hall_sensor_pm_ops,
#endif
	},
	.probe    = hall_sensor_probe,
	.remove   = __devexit_p(hall_sensor_remove),
};

/*
static int __init hall_sensor_init(void)
{
	pr_info("F@SENSOR:hall_senso_init\n");
	return platform_driver_register(&hall_sensor_driver);
}


static void __exit hall_senso_exit(void)
{
	platform_driver_unregister(&hall_sensor_driver);
}

module_init(hall_sensor_init);
module_exit(hall_sensor_init);
*/
module_platform_driver(hall_sensor_driver);
MODULE_DEVICE_TABLE(of, hall_sensor_device_of_match);

MODULE_LICENSE("Proprietary");
