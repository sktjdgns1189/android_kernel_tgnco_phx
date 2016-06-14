/* drivers/leds/rt4501_backlight.c
 * RT4501 LED Backlight Driver
 *
 * Copyright (C) 2013 Richtek Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <mach/msm_iomap.h>

#include <linux/platform_data/rt4501_bl.h>
#include "../msm/mdss/mdss_dsi.h"

/* Enable it to create debug fs. (Please DON'T enable it unless you know what are you doing.) */
//#define CONFIG_DEBUG_RT4501_FS

static unsigned char rt4501_init_data[] ={
	0x87, /* reg 0x02 */
	0x40, /* reg 0x03 */
};

#ifdef CONFIG_DEBUG_RT4501_FS
//debugfs
struct driver_dbg {
	void *data_ptr;
	int id;
};

struct rt4501_debug {
	unsigned char reg;
	struct driver_dbg dri_dbg[3];
};
//
#endif /* #ifdef CONFIG_DEBUG_RT4501_FS */

struct rt4501_chip
{
#ifdef CONFIG_DEBUG_RT4501_FS
	struct rt4501_debug debug;
#endif /* #ifdef CONFIG_DEBUG_RT4501_FS */
	struct i2c_client *client;
	struct mutex io_lock;
	struct led_classdev led_dev;
	unsigned int enable;
	unsigned int gpio_enable_pin;
};

static void rt4501_i2c_write(struct i2c_client *client, unsigned char reg, unsigned int data)
{
	unsigned int ret;
	struct rt4501_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_write_byte_data(client, reg, data);
	if(ret < 0)
	{
		dev_err(&client->dev,"i2c write fail\n");
		mutex_unlock(&chip->io_lock);
		return;
	}
	mutex_unlock(&chip->io_lock);
}

static int rt4501_i2c_read(struct i2c_client *client, unsigned char reg)
{
	unsigned int ret;
	struct rt4501_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->io_lock);
	return ret;
}

static void rt4501_i2c_setbit(struct i2c_client *client ,unsigned char reg, unsigned int mask)
{
	unsigned int ret;
	struct rt4501_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_read_byte_data(client,reg);
	i2c_smbus_write_byte_data(client, reg, ret | mask);
	mutex_unlock(&chip->io_lock);
}

static void rt4501_i2c_clearbit(struct i2c_client *client, unsigned char reg, unsigned int mask)
{
	unsigned int ret;
	struct rt4501_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->io_lock);
	ret = i2c_smbus_read_byte_data(client,reg);
	i2c_smbus_write_byte_data(client, reg, ret & ~mask);
	mutex_unlock(&chip->io_lock);
}

#ifdef CONFIG_DEBUG_RT4501_FS
//  debugfs
struct dentry *rt4501_dbg_dir;
struct dentry *rt4501_dbg[3];

static int de_open(struct inode* inode, struct file* file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t de_read(struct file *file, char __user *user_buffer, size_t count, loff_t *position)
{
	struct driver_dbg *db = file->private_data;
	struct rt4501_chip *chip = db->data_ptr;
	char tmp[200] ={0};
	int i, ret;
	// use db.id to switch which node is connected 0:reg  1:data  2:regs
	pr_info("rt45 read\n");
	switch (db->id)
	{
		case 0:
			sprintf(tmp,"0x%02x\n", chip->debug.reg);
			break;
		case 1:
			ret = rt4501_i2c_read(chip->client, chip->debug.reg);
			if(ret < 0)
			{
				dev_err(&chip->client->dev, "i2c read fail\n");
			}
			sprintf(tmp,"0x%02x\n",ret);
			break;
		case 2:
			for (i=0; i<RT4501_MAX_REG; i++)
			{
				ret = rt4501_i2c_read(chip->client, i);
				sprintf(tmp+strlen(tmp), "0x%02x\n", ret);
			}
			break;
		default:
			break;
	}

	return simple_read_from_buffer(user_buffer,count,position,tmp,strlen(tmp));
}

static ssize_t de_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *position)
{
	struct driver_dbg *db = file->private_data;
	struct rt4501_chip *chip = db->data_ptr;
	char buf[200] = {0};
	unsigned long yo;
	int ret;

	simple_write_to_buffer(buf, sizeof(buf), position, user_buffer, count);
	//yo stored in hex
	ret = kstrtoul(buf, 16, &yo);

	pr_info("yo = 0x%02x\n", (unsigned int)yo);
	switch (db->id)
	{
		case 0:
			if(yo<0 || yo > RT4501_MAX_REG)
			{
				pr_info("out of range\n");
				return -1;
			}
			chip->debug.reg = yo;
			break;
		case 1:
			rt4501_i2c_write(chip->client, chip->debug.reg, yo);
			break;
		default:
			return -EINVAL;
	}
	return count;
}

static struct file_operations debugfs_fops =
{
	.owner = THIS_MODULE,
	.open = de_open,
	.read = de_read,
	.write = de_write,
};
// end of debugfs
#endif /* #ifdef CONFIG_DEBUG_RT4501_FS */

// set enalbe
static void rt4501_chip_enable(struct rt4501_chip *chip, int enable)
{
	int ret;
	// if enable != 0 : set enable,  if enable = 0 : set disable
	if(enable)
	{
		//set enable
		if (!chip->enable)
		{
			ret = rt4501_i2c_read(chip->client, RT4501_CONFIG);
			if(ret < 0)
			{
				pr_info("read enable fail\n");
				return;
			}
			rt4501_i2c_setbit(chip->client, RT4501_CONFIG, 0x01);
		}
	}
	else
	{
		//set disable
		if (chip->enable)
		{
			ret = rt4501_i2c_read(chip->client,RT4501_CONFIG);
			if(ret < 0)
			{
				pr_info( "read enable fail\n");
				return;
			}
			rt4501_i2c_clearbit(chip->client, RT4501_CONFIG, 0x01);
		}
	}

	chip->enable = enable;
}

//init register map
static int rt4501_init_reg(struct rt4501_chip *chip)
{
	rt4501_i2c_write(chip->client, RT4501_CONFIG, rt4501_init_data[0]);
	rt4501_i2c_write(chip->client, RT4501_TIMING, rt4501_init_data[1]);
	return 0;
}

static void rt4501_set_brightness(struct rt4501_chip *chip, int brightness)
{
	rt4501_i2c_write(chip->client, RT4501_BACKLIGHT_CTRL, brightness);
	chip->led_dev.brightness = brightness;
	if( brightness != 0)
		rt4501_chip_enable(chip, 1); //enalbe
	else
		rt4501_chip_enable(chip, 0); //disable
}

int mapping_func(int brightness) {
	int max_bl = 185;

	if (brightness)
		brightness = (int)(((brightness*max_bl*10 + 5*255)/255)/10);

	return brightness;
}

struct i2c_client *client_const;
static int pr_bl_lv = 0;
void rt4501_set_backlight_level(int enable, int brightness, struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct rt4501_chip *chip = i2c_get_clientdata(client_const);

	/* When max currnet be set as 30mA, use mapping function to make sure max current is 21mA */
	if (rt4501_init_data[0] && RT4501_MAXCURR_MASK)
		brightness = mapping_func(brightness);

	pr_debug("%s: level=%d, enable=0x%X\n", __func__, brightness, enable);
	if (brightness == 0) {
		pr_info("%s: level=%d, enable=0x%X\n", __func__, brightness, enable);
		pr_bl_lv = 0;
	}
	else if (pr_bl_lv == 0){
		pr_info("%s: level=%d, enable=0x%X\n", __func__, brightness, enable);
		pr_bl_lv = 1;
	}

	if (rt4501_init_data[0] && RT4501_PWMEN_MASK) {
		/* PWM Control */
		switch (enable){
			case 0:
				gpio_direction_output(chip->gpio_enable_pin, 0);
				chip->enable = 0;
				break;
			case 1:
				gpio_direction_output(chip->gpio_enable_pin, 1);
				chip->enable = 1;
				rt4501_init_reg(chip);
				mdss_dsi_panel_bklt_dcs(ctrl, brightness);
				break;
			default:
				mdss_dsi_panel_bklt_dcs(ctrl, brightness);
				break;
		}
	}
	else {
		/* I2C Control */
		switch (enable){
			case 0:
				gpio_direction_output(chip->gpio_enable_pin, 0);
				chip->enable = 0;
				break;
			case 1:
				gpio_direction_output(chip->gpio_enable_pin, 1);
				chip->enable = 1;
				rt4501_init_reg(chip);
				rt4501_set_brightness(chip, brightness);
				break;
			default:
				rt4501_set_brightness(chip, brightness);
				break;
		}
	}
}
EXPORT_SYMBOL(rt4501_set_backlight_level);

static void rt4501_brightness_set(struct led_classdev *led, enum led_brightness val)
{
	struct rt4501_chip *chip;
	chip = container_of(led, struct rt4501_chip , led_dev);
	if(val > 0xB9) val = 0XB9;	/* To make sure max current is 21mA (level = 185) */
	if(val < 0x00) val = 0x00;

	rt4501_set_brightness(chip, val);

	return;
}

static enum led_brightness rt4501_brightness_get(struct led_classdev *led)
{
	int brightness = 0;
	struct rt4501_chip *chip;
	chip = container_of(led, struct rt4501_chip, led_dev);
	mutex_lock(&chip->io_lock);
	brightness = chip->led_dev.brightness;
	mutex_unlock(&chip->io_lock);
	return brightness;
}

//device tree
static int rt_parse_dt(struct rt4501_chip *chip, struct device *dev)
{
#ifdef CONFIG_OF
	u32 tmp = 0;
	struct device_node *np = dev->of_node;
	chip->gpio_enable_pin = of_get_named_gpio(np, "rt,en_pin", 0);

	if (of_property_read_bool(np, "rt,ovp_sel"))
		rt4501_init_data[0] |= RT4501_OVPSEL_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_OVPSEL_MASK;

	if (of_property_read_bool(np, "rt,pwm_en"))
		rt4501_init_data[0] |= RT4501_PWMEN_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_PWMEN_MASK;

	if (of_property_read_bool(np, "rt,pwm_pol"))
		rt4501_init_data[0] |= RT4501_PWMPOL_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_PWMPOL_MASK;

	if (of_property_read_bool(np, "rt,max_curr"))
		rt4501_init_data[0] |= RT4501_MAXCURR_MASK;
	else
		rt4501_init_data[0] &= ~RT4501_MAXCURR_MASK;


	if (of_property_read_u32(np, "rt,up_ramprate", &tmp) >= 0)
	{
		rt4501_init_data[1] &= ~RT4501_UPRAMP_MASK;
		rt4501_init_data[1] |= (tmp << RT4501_UPRAMP_SHFT);
	}
	if (of_property_read_u32(np, "rt,down_ramprate", &tmp) >= 0)
	{
		rt4501_init_data[1] &= ~RT4501_DNRAMP_MASK;
		rt4501_init_data[1] |= (tmp << RT4501_DNRAMP_SHFT);
	}
#endif /* #ifdef CONFIG_OF */
	return 0;
}

//not device tree
static int rt_parse_pdata(struct rt4501_chip *chip, struct device *dev)
{
	struct rt4501_platform_data *pdata = dev->platform_data;
	chip->gpio_enable_pin = pdata->en_pin;
	if(!pdata->ovp_sel)
		rt4501_init_data[0] &= ~RT4501_OVPSEL_MASK;
	else
		rt4501_init_data[0] |= RT4501_OVPSEL_MASK;

	if(!pdata->pwm_en)
		rt4501_init_data[0] &= ~RT4501_PWMEN_MASK;
	else
		rt4501_init_data[0] |= RT4501_PWMEN_MASK;

	if(!pdata->pwm_pol)
		rt4501_init_data[0] &= ~RT4501_PWMPOL_MASK;
	else
		rt4501_init_data[0] |= RT4501_PWMPOL_MASK;

	if(!pdata->max_curr)
		rt4501_init_data[0] &= ~RT4501_MAXCURR_MASK;
	else
		rt4501_init_data[0] |= RT4501_MAXCURR_MASK;

	rt4501_init_data[1] &= ~RT4501_UPRAMP_MASK;
	rt4501_init_data[1] |= (pdata->up_ramprate << RT4501_UPRAMP_SHFT);

	rt4501_init_data[1] &= ~RT4501_DNRAMP_MASK;
	rt4501_init_data[1] |= (pdata->down_ramprate << RT4501_DNRAMP_SHFT);

	return 0;
}

static int skt_restart(void)
{
	unsigned int *p = NULL;
	unsigned int rere = 0;

	/* read apr->rere */
	p = (unsigned int *)ioremap(0x10880248, sizeof(unsigned int));
	if (p == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (-1);
	}
	rere = *p;
	iounmap(p);

	if (rere != 0x21534b54) {
		return (0);
	}

	pr_info("rt4501_bl: SKT Silence Reboot\n");
	return (1);
}

static int rt4501_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rt4501_chip *chip;
	//device tree node
	bool use_dt = client->dev.of_node;
	int ret = 0;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	// device tree
	if (use_dt)
		rt_parse_dt(chip, &client->dev);
	else  //not device tree
	{
		if (!client->dev.platform_data)
			return -EINVAL;
		rt_parse_pdata(chip, &client->dev);
	}

	//chip data initialize
	chip->client = client;
	i2c_set_clientdata(client, chip);

	//request gpio
	ret = gpio_request(chip->gpio_enable_pin, "rt4501 enable pin");
	if(ret < 0)
	{
		dev_err(&client->dev, "rt4501 request gpio fail\n");
		return -EINVAL;
	}

	// gpio init = 1 (enable)
	if (0 < skt_restart()) {
		/* no enable backlight when skt silence boot */
		gpio_direction_output(chip->gpio_enable_pin, 0);
	} else {
		gpio_direction_output(chip->gpio_enable_pin, 1);
	}
	chip->enable = 1;

	//start mutex
	mutex_init(&chip->io_lock);

	//init register map
	rt4501_init_reg(chip);

	chip->led_dev.name = "rt4501_led_cdev";
	// led brightness init = 255 (max)
	chip->led_dev.brightness = 255;
	chip->led_dev.brightness_set = rt4501_brightness_set;
	chip->led_dev.brightness_get = rt4501_brightness_get;
	ret = led_classdev_register(&client->dev, &chip->led_dev);
	if(ret < 0)
	{
		dev_err(&client->dev, "RT4501 : register led class device fail\n");
		return -ENODEV;
	}
	client_const = client;
	pr_info("classdev led register	 success\n");

#ifdef CONFIG_DEBUG_RT4501_FS
	// create debug node
	chip->debug.reg = 1;
	rt4501_dbg_dir = debugfs_create_dir(RT4501_DEV_NAME, NULL);
	chip->debug.dri_dbg[0].data_ptr = chip;
	chip->debug.dri_dbg[0].id = 0;
	rt4501_dbg[0] = debugfs_create_file("reg", 0644, rt4501_dbg_dir, &chip->debug.dri_dbg[0], &debugfs_fops);

	chip->debug.dri_dbg[1].data_ptr = chip;
	chip->debug.dri_dbg[1].id = 1;
	rt4501_dbg[1] = debugfs_create_file("data", 0644, rt4501_dbg_dir, &chip->debug.dri_dbg[1], &debugfs_fops);

	chip->debug.dri_dbg[2].data_ptr = chip;
	chip->debug.dri_dbg[2].id = 2;
	rt4501_dbg[2] = debugfs_create_file("regs", 0644, rt4501_dbg_dir, &chip->debug.dri_dbg[2], &debugfs_fops);
#endif /* #ifdef CONFIG_DEBUG_RT4501_FS */

	return 0;
}

static int rt4501_remove(struct i2c_client *client)
{
	struct rt4501_chip *chip = i2c_get_clientdata(client);
	if(chip)
	{
		mutex_destroy(&chip->io_lock);
		gpio_free(chip->gpio_enable_pin);
		led_classdev_unregister(&chip->led_dev);
	}
#ifdef CONFIG_DEBUG_RT4501_FS
	debugfs_remove_recursive(rt4501_dbg_dir);
#endif
	return 0;
}

//device tree id table
static struct of_device_id rt_match_table[] = {
	{ .compatible = "rt,rt4501", },
	{},
};

static const struct i2c_device_id rt_id[] = {
	{ RT4501_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver rt4501_driver =
{
	.driver =
	{
		.owner = THIS_MODULE,
		.name = RT4501_DEV_NAME,
		.of_match_table = rt_match_table,//device tree
	},
	.probe = rt4501_probe,
	.remove = rt4501_remove,
	.id_table = rt_id,
};


static int __init rt4501_init(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	return i2c_add_driver(&rt4501_driver);
}

static void __exit rt4501_exit(void)
{
	i2c_del_driver(&rt4501_driver);
}
module_init(rt4501_init);
module_exit(rt4501_exit);

MODULE_DESCRIPTION("Richtek RT4501 LED Backlight Driver");
MODULE_AUTHOR("Richtek Technology Corp.");
MODULE_VERSION(RT4501_DRV_VER);
MODULE_LICENSE("GPL");
