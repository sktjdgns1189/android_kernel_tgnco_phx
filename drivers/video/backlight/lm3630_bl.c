/*
* Simple driver for Texas Instruments LM3630 Backlight driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/platform_data/lm3630_bl.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define REG_CTRL	0x00
#define REG_CONFIG	0x01
#define REG_BOOST_CTRL	0x02
#define REG_BRT_A	0x03
#define REG_BRT_B	0x04
#define REG_CURRENT_A	0x05
#define REG_CURRENT_B	0x06
#define REG_ON_OFF_RAMP	0x07
#define REG_RUN_RAMP	0x08
#define REG_INT_STATUS	0x09
#define REG_INT_EN	0x0A
#define REG_FAULT	0x0B
#define REG_RESET	0x0F
#define REG_PWM_OUTLOW	0x12
#define REG_PWM_OUTHIGH	0x13
#define REG_REVISION	0x1F
#define REG_FILTER_STR	0x50
#define REG_MAX		0x50

/* Black Box */
#define BBOX_BACKLIGHT_PWM_FAIL do {printk("BBox;%s: PWM fail\n", __func__); printk("BBox::UEC;1::0\n");} while (0);
#define BBOX_BACKLIGHT_I2C_FAIL do {printk("BBox;%s: I2C fail\n", __func__); printk("BBox::UEC;1::1\n");} while (0);

static struct i2c_client *lm3630_i2c_client;

struct lm3630_chip_data {
	struct device *dev;
	struct lm3630_platform_data *pdata;
	struct regmap *regmap;
};

static int reg_read(struct regmap *map, unsigned int reg, unsigned int *val)
{
	int ret;
	int retry = 0;

	do {
		ret = regmap_read(map, reg, val);
		if (ret < 0) {
			pr_err("%s: fail, reg=%d\n", __func__, reg);
			retry++;
			msleep(10);
		}
	} while ((ret < 0) && (retry <= 3));

	if (ret < 0) {
		BBOX_BACKLIGHT_I2C_FAIL
	}

	return ret;
}

static int reg_write(struct regmap *map, unsigned int reg, unsigned int val)
{
	int ret;
	int retry = 0;

	do {
		ret = regmap_write(map, reg, val);
		if (ret < 0) {
			pr_err("%s: fail, reg=%d, val=0x%X\n", __func__,
				reg, val);
			retry++;
			msleep(10);
		}
	} while ((ret < 0) && (retry <= 3));

	if (ret < 0) {
		BBOX_BACKLIGHT_I2C_FAIL
	}

	return ret;
}

static int reg_update_bits(struct regmap *map, unsigned int reg, unsigned int mask, unsigned int val)
{
	int ret;
	int retry = 0;

	do {
		ret = regmap_update_bits(map, reg, mask, val);
		if (ret < 0) {
			pr_err("%s: fail, reg=%d, mask=0x%X, val=0x%X\n",
				__func__, reg, mask, val);
			retry++;
			msleep(10);
		}
	} while ((ret < 0) && (retry <= 3));

	if (ret < 0) {
		BBOX_BACKLIGHT_I2C_FAIL
	}

	return ret;
}

/* initialize chip */
static int lm3630_chip_init(struct lm3630_chip_data *pchip)
{
	int ret = 0;
	struct lm3630_platform_data *pdata = pchip->pdata;

	/* Filter Strength */
	ret = reg_write(pchip->regmap, REG_FILTER_STR, pdata->def_filter_str);
	if (ret < 0)
		goto out;

	/* Configuration	(PWM enable bank A, PWM keep low) */
	ret = reg_write(pchip->regmap, REG_CONFIG, pdata->def_config);
	if (ret < 0)
		goto out;

	/* Boost Control	(40V, 1.2A, 500kHz) */
	ret = reg_write(pchip->regmap, REG_BOOST_CTRL, pdata->def_boost_ctrl);
	if (ret < 0)
		goto out;

	/* Current A 		(Hysteresis, Lower Bound, 21mA) */
	ret = reg_write(pchip->regmap, REG_CURRENT_A, pdata->def_current);
	if (ret < 0)
		goto out;

	/* Control 			(Enable backlight, Linear output mode) */
	ret = reg_write(pchip->regmap, REG_CTRL, pdata->def_ctrl);
	if (ret < 0)
		goto out;

	/* Brightness A 	(Default backlight level = 96)*/
	ret = reg_write(pchip->regmap, REG_BRT_A, pdata->def_brt);
	if (ret < 0)
		goto out;

	return ret;

out:
	pr_err("%s: i2c failed to access register\n", __func__);

	return ret;
}

static const struct regmap_config lm3630_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int pr_bl_lv = 0;
int lm3630_lcd_backlight_set_level(int level, int enable)
{
	struct	i2c_client *client = lm3630_i2c_client;
	struct	lm3630_chip_data *pchip;
	struct	lm3630_platform_data *pdata;
	int	ret = 0;

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return -1;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return -1;
	}

	pdata = pchip->pdata;
	if (!pdata) {
		pr_err("%s: pdata is NULL\n", __func__);
		return -1;
	}

	pr_debug("%s: level=%d, enable=0x%X\n", __func__, level, enable);

	if (level == 0) {
		pr_info("%s: level=%d, enable=0x%X\n", __func__, level, enable);
		pr_bl_lv = 0;
	}
	else if (pr_bl_lv == 0){
		pr_info("%s: level=%d, enable=0x%X\n", __func__, level, enable);
		pr_bl_lv = 1;
	}

	if (level > pdata->max_brt_led)
		level = pdata->max_brt_led;

	if (enable == 0) {
		/* disable output , enter sleep */
		ret = reg_update_bits(pchip->regmap, REG_CTRL, 0xFF, 0x80);
		if (ret < 0)
			goto out;
	} else {
		/* leave sleep */
		ret = reg_update_bits(pchip->regmap, REG_CTRL, 0x80, 0x00);
		if (ret < 0)
			goto out;

		msleep(1);

		/* update brightness */
		ret = reg_write(pchip->regmap, REG_BRT_A, level);
		if (ret < 0)
			goto out;
		/* enable output */
		ret = reg_update_bits(pchip->regmap, REG_CTRL, 0x1F, 0x15);
		if (ret < 0)
			goto out;
	}

	return ret;
out:
	pr_err("%s: i2c failed to access register\n", __func__);
	BBOX_BACKLIGHT_PWM_FAIL
	return ret;
}
EXPORT_SYMBOL(lm3630_lcd_backlight_set_level);

int lm3630_lcd_backlight_get_level(int *level)
{
	struct	i2c_client *client = lm3630_i2c_client;
	struct	lm3630_chip_data *pchip;
	int	ret = 0;
	unsigned int reg_val;

	if (!client) {
		pr_err("%s: client is NULLn", __func__);
		return -1;
	}

	pchip = i2c_get_clientdata(client);
	if (!pchip) {
		pr_err("%s: pchip is NULL\n", __func__);
		return -1;
	}

	if (reg_read(pchip->regmap, REG_PWM_OUTLOW, &reg_val) < 0)
		goto out;

	*level = reg_val;

	if (reg_read(pchip->regmap, REG_PWM_OUTHIGH, &reg_val) < 0)
		goto out;

	if (reg_val & 0x01)
		*level = 256;

	return ret;
out:
	pr_err("%s: i2c failed to access register\n", __func__);
	return ret;
}
EXPORT_SYMBOL(lm3630_lcd_backlight_get_level);

static ssize_t lm3630_bl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int bl_val=0;
	int ret = 0;

	lm3630_lcd_backlight_get_level(&bl_val);
	pr_debug("bl_val = %d\n",bl_val);
	ret += sprintf(buf + ret, "%d\n", bl_val);
	return ret;
}
static ssize_t lm3630_bl_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err =0;
	unsigned int reg_val =0;

	pr_debug("lm3630_bl_store\n");
	if (sscanf(buf, "%d", &reg_val) <= 0) {
		pr_err("[%s] get user-space data failed\n", __func__);
		return -EINVAL;
	}
	pr_debug("reg_val = %d\n",reg_val);
	if(reg_val != 0)
		err = lm3630_lcd_backlight_set_level(reg_val, 1);
	else
		err = lm3630_lcd_backlight_set_level(0, 0);

	return count;
}

static ssize_t lm3630_register_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lm3630_chip_data *pchip;
	int ret = 0;
	unsigned int reg_val;

	BUG_ON(lm3630_i2c_client == NULL);
	pchip = i2c_get_clientdata(lm3630_i2c_client);

	ret += sprintf(buf + ret, "Register:\n");
	if (reg_read(pchip->regmap, REG_CTRL, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x00 REG_CTRL        =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_CONFIG, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x01 REG_CONFIG      =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_BOOST_CTRL, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x02 REG_BOOST_CTRL  =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_BRT_A, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x03 REG_BRT_A       =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_BRT_B, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x04 REG_BRT_B       =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_CURRENT_A, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x05 REG_CURRENT_A   =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_CURRENT_B, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x06 REG_CURRENT_B   =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_ON_OFF_RAMP, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x07 REG_ON_OFF_RAMP =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_RUN_RAMP, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x08 REG_RUN_RAMP    =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_INT_STATUS, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x09 REG_INT_STATUS  =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_INT_EN, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x0A REG_INT_EN      =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_FAULT, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x0B REG_FAULT       =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_RESET, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x0F REG_RESET       =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_PWM_OUTLOW, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x12 REG_PWM_OUTLOW  =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_PWM_OUTHIGH, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x13 REG_PWM_OUTHIGH =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_REVISION, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x1F REG_REVISION    =0x%02X\n", reg_val);
	if (reg_read(pchip->regmap, REG_FILTER_STR, &reg_val) < 0)
		goto out;
	ret += sprintf(buf + ret, " 0x50 REG_FILTER_STR  =0x%02X\n", reg_val);

	return ret;
out:
	pr_err("%s: i2c failed to access register\n", __func__);
	return ret;
}

static ssize_t lm3630_register_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct lm3630_chip_data *pchip;
	unsigned int reg, reg_val;

	BUG_ON(lm3630_i2c_client == NULL);
	pchip = i2c_get_clientdata(lm3630_i2c_client);

	if (sscanf(buf, "%x %x", &reg, &reg_val) <= 0) {
		pr_err("%s: get user-space data failed\n", __func__);
		return -EINVAL;
	}

	if (reg_write(pchip->regmap, reg, reg_val) < 0)
		goto out;

	return count;
out:
	pr_err("%s: i2c failed to access register\n", __func__);
	return count;
}

static DEVICE_ATTR(level, 0644, lm3630_bl_show,  lm3630_bl_store);
static DEVICE_ATTR(register, 0644, lm3630_register_show, lm3630_register_store);

#ifdef CONFIG_OF
static int lm3630_parse_dt(struct device *dev, struct lm3630_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	rc = of_property_read_u32(np, "ti,max-brt-led", &temp_val);
	if (rc) {
		pr_err("%s: Unable to read max-brt-led\n", __func__);
		return rc;
	} else
		pdata->max_brt_led = (u8)temp_val;

	/* reset gpio */
	pdata->hwen_gpio = of_get_named_gpio_flags(np, "ti,hwen-gpio",
				0, &pdata->hwen_gpio_flags);

	rc = of_property_read_u32(np, "def_ctrl", &temp_val);
	if (rc) {
		pr_err("%s:%d, def_ctrl not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pdata->def_ctrl = (!rc ? temp_val : 0);

	rc = of_property_read_u32(np, "def_config", &temp_val);
	if (rc) {
		pr_err("%s:%d, def_config not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pdata->def_config = (!rc ? temp_val : 0);

	rc = of_property_read_u32(np, "def_boost_ctrl", &temp_val);
	if (rc) {
		pr_err("%s:%d, def_boost_ctrl not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pdata->def_boost_ctrl = (!rc ? temp_val : 0);

	rc = of_property_read_u32(np, "def_brt", &temp_val);
	if (rc) {
		pr_err("%s:%d, def_brt not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pdata->def_brt = (!rc ? temp_val : 0);

	rc = of_property_read_u32(np, "def_current", &temp_val);
	if (rc) {
		pr_err("%s:%d, def_current not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pdata->def_current = (!rc ? temp_val : 0);

	rc = of_property_read_u32(np, "def_filter_str", &temp_val);
	if (rc) {
		pr_err("%s:%d, def_filter_str not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pdata->def_filter_str = (!rc ? temp_val : 0);

	return 0;
}
#else
static int lm3630_parse_dt(struct device *dev, struct lm3630_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int lm3630_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3630_platform_data *pdata;
	struct lm3630_chip_data *pchip;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: fail : i2c functionality check...\n", __func__);
		return -EOPNOTSUPP;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct lm3630_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		ret = lm3630_parse_dt(&client->dev, pdata);
		if (ret) {
			pr_err("%s: fail : parse dt\n", __func__);
			return ret;
		}
	} else
		pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pr_err("%s: fail : no platform data.\n", __func__);
		return -ENODATA;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3630_chip_data),
			     GFP_KERNEL);
	if (!pchip) {
		pr_err("%s: fail : kzalloc pchip\n", __func__);
		return -ENOMEM;
	}

	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	if (gpio_request(pdata->hwen_gpio, "lm3630_en")) {
		pr_err("%s: fail: gpio %d unavailable\n", __func__,
			pdata->hwen_gpio);
		return -EOPNOTSUPP;
	}

	gpio_direction_output(pdata->hwen_gpio, 1);
	mdelay(50);

	pchip->regmap = devm_regmap_init_i2c(client, &lm3630_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		pr_err("%s: fail : allocate register map: %d\n", __func__,
			ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	/* chip initialize */
	ret = lm3630_chip_init(pchip);
	if (ret < 0) {
		pr_err("%s: fail : init chip\n", __func__);
		goto err_chip_init;
	}

	lm3630_i2c_client = client;

	ret = device_create_file(&client->dev, &dev_attr_register);
	if (ret)
		pr_err("%s: Failed to register devive %s:(%d)\n", __func__,
			dev_attr_register.attr.name, ret);

	ret = device_create_file(&client->dev, &dev_attr_level);
	if (ret)
		pr_err("%s: Failed to register devive %s:(%d)\n", __func__,
			dev_attr_register.attr.name, ret);

	pr_info("%s: LM3630 backlight register OK.\n", __func__);
	return 0;

err_chip_init:
	return ret;
}

static int lm3630_remove(struct i2c_client *client)
{
	int ret;
	struct lm3630_chip_data *pchip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_register);

	ret = reg_write(pchip->regmap, REG_BRT_A, 0);
	if (ret < 0)
		pr_err("%s: i2c failed to access register\n", __func__);

	return 0;
}

static void lm3630_shutdown(struct i2c_client *client)
{
	pr_debug("%s: enter\n", __func__);
	lm3630_lcd_backlight_set_level(0, PWM_CTRL_DISABLE);
}

static const struct i2c_device_id lm3630_id[] = {
	{ "lm3630_bl", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3630_id);
#ifdef CONFIG_OF
static struct of_device_id lm3630_match_table[] = {
	{ .compatible = "ti,lm3630",},
	{ },
};
#else
#define lm3630_match_table NULL
#endif

static struct i2c_driver lm3630_i2c_driver = {
	.driver = {
		.name = "lm3630_bl",
		.owner	= THIS_MODULE,
		.of_match_table = lm3630_match_table,
	},
	.probe = lm3630_probe,
	.shutdown = lm3630_shutdown,
	.remove = lm3630_remove,
	.id_table = lm3630_id,
};

module_i2c_driver(lm3630_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Backlight driver for LM3630");
MODULE_AUTHOR("G.Shark Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");
