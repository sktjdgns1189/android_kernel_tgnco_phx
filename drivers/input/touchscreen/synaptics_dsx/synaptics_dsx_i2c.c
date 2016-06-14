/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/input/synaptics_dsx_fih.h>
#include "synaptics_dsx_core.h"
#include <linux/of_gpio.h>

#define SYN_I2C_RETRY_TIMES 10
#define DSX_COORDS_ARR_SIZE  4

static void synaptics_rmi4_harware_reset(struct synaptics_rmi4_data *rmi4_data)
{
		int reset_on;
		reset_on = rmi4_data->hw_if->board_data->reset_on_state;

		/*pull reset pin to reset touch ic*/
		gpio_set_value(rmi4_data->hw_if->board_data->reset_gpio, reset_on);
		msleep(rmi4_data->hw_if->board_data->reset_active_ms);
		gpio_set_value(rmi4_data->hw_if->board_data->reset_gpio, !reset_on);
		msleep(rmi4_data->hw_if->board_data->reset_delay_ms);
}

static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);

	page = ((addr >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C read over retry limit\n",
				__func__);
		if (rmi4_data->hw_if->board_data->reset_gpio >= 0) {
			synaptics_rmi4_harware_reset(rmi4_data);

			for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
				if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
					retval = length;
					break;
				}
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			}
			if (retry == SYN_I2C_RETRY_TIMES) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C read over retry limit\n",
						__func__);
				retval = -EIO;
			}
		}
		else {
			pr_err("no hardware reset support\n");
			retval = -EIO;
		}
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	if(retval < 0)
		printk("BBox::UEC; 7::1\n");
	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		if (rmi4_data->hw_if->board_data->reset_gpio >= 0) {
			synaptics_rmi4_harware_reset(rmi4_data);

			for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
				if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
					retval = length;
					break;
				}
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				msleep(20);
			}
			if (retry == SYN_I2C_RETRY_TIMES) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C write over retry limit\n",
						__func__);
				retval = -EIO;
			}
		}
		else {
			pr_err("no hardware reset support\n");
			retval = -EIO;
		}
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	if(retval < 0)
		printk("BBox::UEC; 7::2\n");
	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};

static struct synaptics_dsx_hw_interface hw_if;

static struct platform_device *synaptics_dsx_i2c_device;

static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(synaptics_dsx_i2c_device);

	return;
}

#ifdef CONFIG_OF
int synaptics_dsx_get_dt_coords(struct device *dev, char *name,
				struct synaptics_dsx_board_data *pdata)
{
	u32 coords[DSX_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != DSX_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc) {
		pr_err("Unable to read %s\n", name);
		return rc;
	}

	pdata->panel_minx = coords[0];
	pdata->panel_miny = coords[1];
	pdata->panel_maxx = coords[2];
	pdata->panel_maxy = coords[3];

	return 0;

}

static int synaptics_rmi4_parse_dt(struct device *dev, struct synaptics_dsx_board_data *pdata)
{
	//int retval; //TODO: error checking
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 tmp = 0;
	int err = 0;
	int rc = 0;
	int blen = 0;

	pdata->swap_axes = of_property_read_bool(np, "synaptics,swap-axes");

	/* regulator info */
	pdata->reg_en = of_property_read_bool(np, "synaptics,reg-en");
	pdata->i2c_pull_up = of_property_read_bool(np, "synaptics,i2c-pull-up");
	pdata->led_en = of_property_read_bool(np, "synaptics,led-en");
	/* reset, irq gpio info */

	pdata->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, NULL);
	pdata->irq_flags = IRQF_TRIGGER_FALLING;

	if (of_find_property(np, "synaptics,power-gpio", NULL))
		pdata->power_gpio = of_get_named_gpio_flags(np, "synaptics,power-gpio", 0, NULL);
	else
		pdata->power_gpio = -1;

	if (of_find_property(np, "synaptics,reset-gpio", NULL))
		pdata->reset_gpio = of_get_named_gpio_flags(np, "synaptics,reset-gpio", 0, NULL);
	else
		pdata->reset_gpio = -1;

	rc = synaptics_dsx_get_dt_coords(dev, "synaptics,panel-coords",
				pdata);
	if (rc)
		pr_err("[Synaptics]: ==Can not get device tree panel-coords\n");

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,x-flip", &tmp);
	if (!err)
		pdata->x_flip = tmp;
	else
		pdata->x_flip = 0;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,y-flip", &tmp);
	if (!err)
		pdata->y_flip = tmp;
	else
		pdata->y_flip = 0;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,power-on-status", &tmp);
	if (!err)
		pdata->power_on_state = tmp;
	else
		pdata->power_on_state = 1;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,reset-on-status", &tmp);
	if (!err)
		pdata->reset_on_state = tmp;
	else
		pdata->reset_on_state = 0;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,virtual-key", &tmp);
	if (!err)
		pdata->virtual_key = tmp;
	else
		pdata->virtual_key = 0;
	tmp = 0;
	err = of_property_read_u32(np, "synaptics,power-delay-ms", &tmp);
	if (!err)
		pdata->power_delay_ms = tmp;
	else
		pdata->power_delay_ms = 0;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,reset-delay-ms", &tmp);
	if  (!err)
		pdata->reset_delay_ms = tmp;
	else
		pdata->reset_delay_ms = 0;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,reset-active-ms", &tmp);
	if (!err)
		pdata->reset_active_ms = tmp;
	else
		pdata->reset_active_ms = 0;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,config-id-addr", &tmp);
	if (!err)
		pdata->config_id_addr = tmp;
	else
		pdata->config_id_addr = 0x0009;

	prop = of_find_property(np, "synaptics,button-map", NULL);
	if (prop) {
		pdata->cap_button_map = devm_kzalloc(dev, sizeof(*pdata->cap_button_map), GFP_KERNEL);
		if (!pdata->cap_button_map)
			return -ENOMEM;

		pdata->cap_button_map->map = devm_kzalloc(dev,
			sizeof(*pdata->cap_button_map->map) * MAX_NUMBER_OF_BUTTONS,
			GFP_KERNEL);
		if (!pdata->cap_button_map->map)
			return -ENOMEM;

		pdata->cap_button_map->nbuttons = prop->length / sizeof(tmp);

		err = of_property_read_u32_array(np,
			"synaptics,button-map", (unsigned int *)pdata->cap_button_map->map,
			pdata->cap_button_map->nbuttons);
		if (err) {
			dev_err(dev, "Unable to read key codes\n");
			pdata->cap_button_map->map = NULL;
		}
	}

	if (pdata->reg_en) {
		pdata->vcc = regulator_get(dev, "vcc");
		if (IS_ERR(pdata->vcc)) {
			rc = PTR_ERR(pdata->vcc);
			dev_err(dev,
				"Regulator get failed vcc rc=%d\n", rc);
			pdata->reg_en = false;
		}
	}

	if (pdata->i2c_pull_up) {
		pdata->vcc_i2c = regulator_get(dev, "vcc_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			dev_err(dev,
				"Regulator get failed vcc_i2c rc=%d\n", rc);
			pdata->i2c_pull_up = false;
		}
	}

	if(pdata->led_en) {
		pdata->v_led = regulator_get(dev, "v_led");
		if (IS_ERR(pdata->v_led)){
			rc = PTR_ERR(pdata->v_led);
			dev_err(dev,
				"Regulator get failed v_led rc=%d\n", rc);
			pdata->led_en = false;
		}
	}

	pdata->upper_bound = of_get_property(np, "synaptics,upper-bound", &blen);
	if (!pdata->upper_bound) {
		pr_err("[Synaptics]: ==Can not read the upper bound array\n");
	}

	pdata->upper_bound_array_size = blen/2;

	pdata->lower_bound = of_get_property(np, "synaptics,lower-bound", &blen);
	if (!pdata->lower_bound) {
		pr_err("[Synaptics]: ==Can not read the lower bound array\n");
	}

	pdata->lower_bound_array_size = blen/2;

	tmp = 0;
	err = of_property_read_u32(np, "synaptics,rx-of-key", &tmp);
	if (!err)
		pdata->rx_of_key = tmp;
	else
		pdata->rx_of_key = -1;

	err = of_property_read_string(np, "synaptics,project-name", &pdata->project_name);
	if(err)
		pdata->project_name = "none";

	return 0;
}
#else
static int synaptics_rmi4_parse_dt(struct device *dev, struct synaptics_dsx_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

#ifndef CONFIG_OF
	hw_if.board_data = client->dev.platform_data;
#else
	if (client->dev.of_node) {
		hw_if.board_data = devm_kzalloc(&client->dev,
			sizeof(struct synaptics_dsx_board_data), GFP_KERNEL);
		if (!hw_if.board_data) {
			dev_err(&client->dev, "%s: Failed to allocate memory for pdata\n", __func__);
			return -ENOMEM;
		}
		synaptics_rmi4_parse_dt(&client->dev, hw_if.board_data);
	}
#endif
	hw_if.bus_access = &bus_access;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = 0;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = &hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

	return 0;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(synaptics_dsx_i2c_device);

	return 0;
}

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id synaptics_match_table[] = {
	{ .compatible = "synaptics,dsx",},
	{ },
};
#else
#define synaptics_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.probe = synaptics_rmi4_i2c_probe,
	.remove = __devexit_p(synaptics_rmi4_i2c_remove),
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_match_table,
	},
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
	int ret;
	ret = i2c_add_driver(&synaptics_rmi4_i2c_driver);
	return ret;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
