/*
 * Driver about battery protection mechanism
 *
 * Copyright (C) 2014 FIH.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author:  PinyCHWu <pinychwu@fih-foxconn.com>
 *	    January 2014
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/skbuff.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#endif
#include <linux/power/battery_protection.h>

static int disable_flag = 0;
module_param(disable_flag, int, 0644);

static struct batt_protect_chip *the_chip = NULL;

static int check_temp_status(struct batt_protect_chip *chip, int temp)
{
	if (disable_flag)
		return TEMP_NORMAL;

	if ((chip->setting[TEMP_HOT].enabled) && (temp > chip->setting[TEMP_HOT].temp_c))
		return TEMP_HOT;
	else if ((chip->setting[TEMP_WARM].enabled) && (temp > chip->setting[TEMP_WARM].temp_c))
		return TEMP_WARM;
	else if ((chip->setting[TEMP_COLD].enabled) && (temp < chip->setting[TEMP_COLD].temp_c))
		return TEMP_COLD;
	else if ((chip->setting[TEMP_COOL].enabled) && (temp < chip->setting[TEMP_COOL].temp_c))
		return TEMP_COOL;
	else
		return TEMP_NORMAL;
}

static int batt_protect_action(struct power_supply *psy, int vol_mv, int cur_ma)
{
	union power_supply_propval val = {0,};
	int retval = 0;

	pr_debug("[%s] Set limit: %dmV, %dmA\n", __func__, vol_mv, cur_ma);

	//set voltage limit
	val.intval = vol_mv;
	retval = psy->set_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (retval < 0) {
		pr_err("[%s] set charge max voltage(%d) failed\n", __func__, vol_mv);
		return -1;
	}

	//set current limit
	val.intval = cur_ma;
	retval = psy->set_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (retval < 0) {
		pr_err("[%s] set charge max current(%d) failed\n", __func__, cur_ma);
		return -2;
	}

	return 0;
}

static int batt_protect_mechanism(struct batt_protect_chip *chip)
{
	union power_supply_propval ret = {0,};
	int retval = 0;
	int temp = 0;
	int status = 0;

	retval = chip->bat_psy->get_property(chip->bat_psy, POWER_SUPPLY_PROP_TEMP, &ret);
	if (retval >= 0)
		temp = ret.intval;
	else
		return chip->status;

	status = check_temp_status(chip, temp);

	pr_debug("[%s] temp:%d, status:%d\n", __func__, temp, status);
	if (status != chip->status) {
		retval = batt_protect_action(chip->chg_psy, chip->setting[status].vol_mv,
							chip->setting[status].cur_ma);
		if (retval < 0) {
			return chip->status;
		}
	}

	return status;
}

int batt_protection_detect(void)
{
	struct batt_protect_chip *chip = the_chip;

	if (!chip)
		return TEMP_NORMAL;

	if (!chip->bat_psy) {
		chip->bat_psy = power_supply_get_by_name(chip->bat_psy_name);
		if (!chip->bat_psy) {
			pr_debug("[%s] No battery psy\n", __func__);
			return TEMP_NORMAL;
		}
	}
	if (!chip->bat_psy->get_property) {
		pr_debug("[%s] No battery get property\n", __func__);
		return TEMP_NORMAL;
	}

	if (!chip->chg_psy) {
		chip->chg_psy = power_supply_get_by_name(chip->chg_psy_name);
		if (!chip->chg_psy) {
			pr_debug("[%s] No charger psy\n", __func__);
			return TEMP_NORMAL;
		}
	}
	if (!chip->chg_psy->set_property) {
		pr_debug("[%s] No charger set property\n", __func__);
		return TEMP_NORMAL;
	}

	chip->status = batt_protect_mechanism(chip);

	return chip->status;
}
EXPORT_SYMBOL(batt_protection_detect);

/* ************************* Sysfs parts *********************** */
static ssize_t batt_protect_show_settings(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct batt_protect_chip* const chip = dev_get_drvdata(dev);
	int i, len;

	for (i = 0, len = 0; i < TEMP_STATUS_COUNT; i++) {
		len += sprintf(buf+len, "%d %d %d %d\n", chip->setting[i].enabled,
			chip->setting[i].temp_c, chip->setting[i].vol_mv, chip->setting[i].cur_ma);
	}

	return len;
}

static DEVICE_ATTR(settings, S_IWUSR | S_IRUSR,
		   batt_protect_show_settings, NULL);

static struct attribute *batt_protect_attributes[] = {
	&dev_attr_settings.attr,
	NULL,
};

static const struct attribute_group batt_protect_attr_group = {
	.attrs = batt_protect_attributes,
};

/* ************************* Driver parts *********************** */
#ifdef CONFIG_OF
static int get_child_property(struct batt_protect_platform_data *pdata, struct device_node *dev_node, int type, int neg_offset)
{
	int ret = 0;

	ret = of_property_read_u32(dev_node, "action,enabled", &pdata->setting[type].enabled);
	if (ret < 0)
		return -1;
	ret = of_property_read_u32(dev_node, "action,temp_c", &pdata->setting[type].temp_c);
	if (ret < 0)
		return -1;
	if ((type != TEMP_NORMAL) && pdata->setting[type].enabled)
		pdata->setting[type].temp_c -= neg_offset;
	ret = of_property_read_u32(dev_node, "action,vol_mv", &pdata->setting[type].vol_mv);
	if (ret < 0)
		return -1;
	ret = of_property_read_u32(dev_node, "action,cur_ma", &pdata->setting[type].cur_ma);
	if (ret < 0)
		return -1;

	return 0;
}

static void parse_dts_to_pdata(struct batt_protect_platform_data *pdata, struct device_node *dev_node)
{
	struct device_node *child_node = NULL;
	int type = 0;
	int ret = 0;
	const char *bat_name, *chg_name;
	u32 neg_offset = 0;

	ret = of_property_read_string(dev_node, "batt_protect,bat_name", &bat_name);
	if (ret < 0) {
		pr_err("[%s] get bat_name failed(%d)\n", __func__, ret);
		memset(pdata->bat_psy_name, 0, PSY_NAME_LEN);
	} else {
		pr_info("[%s] bat_psy:%s\n", __func__, bat_name);
		memcpy(pdata->bat_psy_name, bat_name, PSY_NAME_LEN);
	}

	ret = of_property_read_string(dev_node, "batt_protect,chg_name", &chg_name);
	if (ret < 0) {
		pr_err("[%s] get chg_name failed(%d)\n", __func__, ret);
		memset(pdata->chg_psy_name, 0, PSY_NAME_LEN);
	} else {
		pr_info("[%s] chg_psy:%s\n", __func__, chg_name);
		memcpy(pdata->chg_psy_name, chg_name, PSY_NAME_LEN);
	}

	ret = of_property_read_u32(dev_node, "batt_protect,negative_offset", &neg_offset);
	if (ret < 0) {
		pr_err("[%s] get negative offset failed(%d)\n", __func__, ret);
		neg_offset = 0;
	}
	pr_info("[%s] Use negative offset = %u\n", __func__, neg_offset);

	while ((child_node = of_get_next_child(dev_node, child_node)) != NULL) {
		ret = of_property_read_u32(child_node, "action,type", &type);
		if (ret < 0) {
			pr_err("[%s] Get type failed\n", __func__);
			continue;
		}
		if ((type >= TEMP_NORMAL) && (type <= TEMP_COLD)) {
			ret = get_child_property(pdata, child_node, type, neg_offset);
			if (ret < 0) {
				pr_err("[%s] Get child property (type:%d) failed\n", __func__, type);
				pdata->setting[type].enabled = 0;
				pdata->setting[type].temp_c = 0;
				pdata->setting[type].vol_mv = 0;
				pdata->setting[type].cur_ma = 0;
			}
		}
	}
}
#endif

static int __devinit batt_protect_probe(struct platform_device *pdev)
{
	int rc = 0, i;
	struct batt_protect_chip *chip;
	struct batt_protect_platform_data *pdata = pdev->dev.platform_data;
#ifdef CONFIG_OF
	struct device_node *dev_node = pdev->dev.of_node;
	int pdata_alloc = 0;
#endif

	pr_info("%s\n", __func__);

	chip = kzalloc(sizeof(struct batt_protect_chip), GFP_KERNEL);
	if (!chip) {
		pr_err("%s: Cannot allocate batt_protect_chip\n", __func__);
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (!pdata && !dev_node) {
		pr_err("[%s] No pdata and dts\n", __func__);
		goto err_pdata;
	} else if (!pdata) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			goto err_pdata;
		pdata_alloc = 1;
	}

	if (dev_node)
		parse_dts_to_pdata(pdata, dev_node);
#else
	if (!pdata) {
		pr_err("[%s] No pdata\n", __func__);
		goto err_pdata;
	}
#endif

	chip->dev = &pdev->dev;

	/* Set up default value */
	chip->status = TEMP_NORMAL;
	for (i = 0; i < TEMP_STATUS_COUNT; i++) {
		chip->setting[i] = pdata->setting[i];
	}
	memcpy(chip->bat_psy_name, pdata->bat_psy_name, PSY_NAME_LEN);
	memcpy(chip->chg_psy_name, pdata->chg_psy_name, PSY_NAME_LEN);
	chip->bat_psy = NULL;
	chip->chg_psy = NULL;

	platform_set_drvdata(pdev, chip);
	the_chip = chip;

	/* Register the sysfs nodes */
	if ((rc = sysfs_create_group(&chip->dev->kobj,
					&batt_protect_attr_group))) {
		dev_err(chip->dev, "Unable to create sysfs group rc = %d\n", rc);
		goto err_sysfs;
	}

#ifdef CONFIG_OF
	if (pdata_alloc)
		kfree(pdata);
#endif

	return 0;

err_sysfs:
	the_chip = NULL;
	platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_OF
	if (pdata_alloc)
		kfree(pdata);
#endif
err_pdata:
	kfree(chip);
	return rc;
}

static int __devexit batt_protect_remove(struct platform_device *pdev)
{
	struct batt_protect_chip *chip = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);
	the_chip = NULL;
	sysfs_remove_group(&chip->dev->kobj, &batt_protect_attr_group);
	platform_set_drvdata(pdev, NULL);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id batt_protect_match[] = {
	{.compatible = BATT_PROTECT_DEV_NAME},
	{}
};
#endif

static struct platform_driver batt_protect_driver = {
	.probe = batt_protect_probe,
	.remove = __devexit_p(batt_protect_remove),
	.driver = {
		.name = BATT_PROTECT_DEV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = batt_protect_match,
#endif
	},
};

static int __init batt_protect_init(void)
{
	return platform_driver_register(&batt_protect_driver);
}

static void __exit batt_protect_exit(void)
{
	platform_driver_unregister(&batt_protect_driver);
}

late_initcall(batt_protect_init);
module_exit(batt_protect_exit);

MODULE_AUTHOR("PinyCHWu <pinychwu@fih-foxconn.com>");
MODULE_DESCRIPTION("Battery Protection Driver");
MODULE_LICENSE("GPL v2");
