/*
 *
 * FocalTech ft5x06 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/input/ft5x06_ts.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MAJ_VER	0xB1
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		32768

/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0

#define FT_FW_CHECK(x)		\
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& (((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& (((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF)))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100

#define FT_UPGRADE_LOOP		10
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_DEBUG_DIR_NAME	"ts_debug"

#define FIH_TOUCH_I2C_VTG_MIN_UV	1800000
#define FIH_TOUCH_I2C_VTG_MAX_UV	1800000

#define FTS_NULL		0x0
#define FTS_TRUE		0x01
#define FTS_FALSE		0x0
#define I2C_CTPM_ADDRESS	0x70
//register address
typedef enum
{
	ERR_OK,
	ERR_MODE,
	ERR_READID,
	ERR_ERASE,
	ERR_STATUS,
	ERR_ECC,
	ERR_DL_ERASE_FAIL,
	ERR_DL_PROGRAM_FAIL,
	ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;	//8 bit
typedef unsigned short        FTS_WORD;	//16 bit
typedef unsigned int          FTS_DWRD;	//16 bit
typedef unsigned char         FTS_BOOL;	//8 bit

#define FT5x0x_TX_NUM	24
#define FT5x0x_RX_NUM	14
#define	FTS_PACKET_LENGTH        128
#define FTM_RAW_DATA_DIVISION 18
#define FTM_RAWDATA_MAX  10000
#define FTM_RAWDATA_MIN  7000
#define FTM_RAWDATA_DIFF 60
#define FTM_RD_RANGE_VALUE (FTM_RAWDATA_MAX - FTM_RAWDATA_MIN)/(FTM_RAW_DATA_DIVISION-2)
u16 RawData_1[FT5x0x_TX_NUM][FT5x0x_RX_NUM];
u16 RawData_2[FT5x0x_TX_NUM][FT5x0x_RX_NUM];
static u16 RDNumber[FTM_RAW_DATA_DIVISION];
static u16 RawDataLimit[2];
static int st_result = 0;
static char test_result[20]={0};
static char upgrade_result[20]={0};
static int RawDataDiffMin = 0;
static int RawDataValue_1 = 0;
static int RawDataValue_2 = 0;
static struct i2c_client *this_client;
static unsigned char CTPM_FW[]=
{
	#include "ft5x06_fw_0D.h"
};

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct ft5x06_ts_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	int upgrade_flag;
	struct mutex device_mode_mutex;
	struct workqueue_struct *ts_workqueue;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};

static char chip_name[12];
static char fih_touch[35] = {0};

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_MAJ_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x06_ts_data *data = dev_id;
	struct input_dev *ip_dev;
	int rc, i;
	u32 id, x, y, pressure, status, num_touches;
	u8 reg = 0x00, *buf;
	bool update_input = false;

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
		return IRQ_HANDLED;
	}

	ip_dev = data->input_dev;
	buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1,
			buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		return IRQ_HANDLED;
	}

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;
		if (id >= FT_MAX_ID)
			break;

		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;

		num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;

		x = (x * data->pdata->x_max)/data->pdata->panel_maxx;

		y = (y * data->pdata->y_max)/data->pdata->panel_maxy;

		if(data->pdata->x_filp)
			x = data->pdata->x_max - x;

		if(data->pdata->y_filp)
			y = data->pdata->y_max - y;

		pr_debug("[FTS]: ==x = %d , y = %d\n", x, y);

		/* invalid combination */
		if (!num_touches && !status && !id)
			break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) {
			pressure = FT_PRESS;
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(ip_dev, ABS_MT_PRESSURE, pressure);
		} else {
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
			input_report_abs(ip_dev, ABS_MT_PRESSURE, 0);
		}
	}

	if (update_input) {
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}

	return IRQ_HANDLED;
}

static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc = 0;

	if(on) {
		if(data->pdata->reg_enable) {
			rc = regulator_enable(data->vdd);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
				goto error;
			}
		}
		else {
			if (gpio_is_valid(data->pdata->enable_gpio))
				gpio_set_value(data->pdata->enable_gpio, 1);
			else {
				rc = -1;
				goto error;
			}
		}

		if(!data->pdata->vcc_i2c_always_on) {
			rc = regulator_enable(data->vcc_i2c);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator vcc_i2c enable failed rc=%d\n", rc);
				regulator_disable(data->vdd);
			}
		}
	}
	else {
		if(data->pdata->reg_enable) {
			rc = regulator_disable(data->vdd);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator vdd disable failed rc=%d\n", rc);
				goto error;
			}
		}
		else {
			if (gpio_is_valid(data->pdata->enable_gpio))
				gpio_set_value(data->pdata->enable_gpio, 0);
			else {
				rc = -1;
				goto error;
			}
		}

		if(!data->pdata->vcc_i2c_always_on) {
			rc = regulator_disable(data->vcc_i2c);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator vcc_i2c disable failed rc=%d\n", rc);
				regulator_enable(data->vdd);
			}
		}
	}

error:
	return rc;
}

static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	if (data->pdata->reg_enable)
	{
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
						   FT_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}
	else {
		if (gpio_is_valid(data->pdata->enable_gpio)) {
			rc = gpio_request(data->pdata->enable_gpio, "ft5x06_enable_gpio");
			if (rc) {
				pr_err("enable gpio request failed\n");
				goto enable_request_fail;
			}
		}
		else
			pr_err("[FTS]: enable gpio is not valid\n");
	}

	if(!data->pdata->vcc_i2c_always_on) {
		data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&data->client->dev,
				"Regulator get failed vcc_i2c rc=%d\n", rc);
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
						   FT_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				goto reg_vcc_i2c_put;
			}
		}
	}

	return 0;

reg_vcc_i2c_put:
	if(!data->pdata->vcc_i2c_always_on)
		regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if ((regulator_count_voltages(data->vdd) > 0) && (data->pdata->reg_enable))
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	if(data->pdata->reg_enable)
		regulator_put(data->vdd);
	return rc;
enable_request_fail:
pwr_deinit:
	if(!data->pdata->vcc_i2c_always_on) {
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

		regulator_put(data->vcc_i2c);
	}

	if (data->pdata->reg_enable) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

		regulator_put(data->vdd);
	}
	return 0;
}

#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2], i;
	int err;

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

	disable_irq(data->pdata->irq_gpio);

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}
	else
		pr_err("[FTS]: reset gpio is not valid\n");

	err = ft5x06_power_on(data, false);
	if (err) {
		dev_err(dev, "power off failed");
		goto pwr_off_fail;
	}

	data->suspended = true;

	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->pdata->irq_gpio);
	return err;
}

static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;

	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	err = ft5x06_power_on(data, true);
	if (err) {
		dev_err(dev, "power on failed");
		return err;
	}

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	else
		pr_err("[FTS]: reset gpio is not valid\n");

	msleep(data->pdata->soft_rst_dly);

	enable_irq(data->pdata->irq_gpio);

	data->suspended = false;

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft5x06_ts_resume(&ft5x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};
#endif


static int fih_touch_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;

	len = snprintf(page, PAGE_SIZE, "%s\n", fih_touch);
	return len;
}

static int fih_info_init(struct device *dev)
{
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 fwver;

	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );

	sprintf(chip_name, "FocalFt5406");

	if(ft5x0x_read_reg(client, FT_REG_FW_VER, &fwver) >= 0)
	{
		pr_info("[FTS]: the fw ver is 0x%02x\n", fwver);
		snprintf(fih_touch, PAGE_SIZE, "%s-V%d.%d", chip_name, fwver >> 4, fwver & 0xf);
	}
	else
	{
		pr_err("[FTS]: %s, read touch firmware failed\n", __func__);
		snprintf(fih_touch, PAGE_SIZE, "Unknown");
	}

	if (create_proc_read_entry("AllHWList/Touch", 0, NULL, fih_touch_read_proc, NULL) == NULL)
	{
		proc_mkdir("AllHWList", NULL);
		if (create_proc_read_entry("AllHWList/Touch", 0, NULL, fih_touch_read_proc, NULL) == NULL)
			printk(KERN_ERR "fail to create proc/%s\n", "AllHWList/Touch");
	}
	return 0;
}

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int rc, i, j, temp;
	u32 pkt_num, pkt_len;
	u8 fw_ecc;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		msleep(info.delay_55);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		w_buf[1] = FT_UPGRADE_AA;
		do {
			j++;
			rc = ft5x06_i2c_write(client, w_buf, 2);
			msleep(FT_RETRY_DLY);
		} while (rc <= 0 && j < FT_MAX_TRIES);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d)\n", i);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	w_buf[0] = FT_ERASE_PANEL_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	data_len = data_len - 8;
	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	for (i = 0; i < 6; i++) {
		temp = FT_FW_LAST_PKT + i;
		pkt_buf[2] = (u8) (temp >> 8);
		pkt_buf[3] = (u8) temp;
		temp = 1;
		pkt_buf[4] = (u8) (temp >> 8);
		pkt_buf[5] = (u8) temp;
		pkt_buf[6] = data[data_len + i];
		fw_ecc ^= pkt_buf[6];
		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min;
	bool fw_upgrade = false;

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%d)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force) {
		fw_upgrade = true;
	} else if (data->fw_ver[0] == fw_file_maj) {
			if (data->fw_ver[1] < fw_file_min)
				fw_upgrade = true;
			else if (data->fw_ver[2] < fw_file_sub_min)
				fw_upgrade = true;
			else
				dev_info(dev, "No need to upgrade\n");
	} else
		dev_info(dev, "Firmware versions do not match\n");

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (FT_FW_CHECK(fw)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

rel_fw:
	release_firmware(fw);
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;

	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret<=0)
	{
		pr_debug("[FTS]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret<=0)
	{
		pr_debug("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}

FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
	FTS_BYTE write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
	return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
	return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp;
	unsigned char i ;

	struct ft5x06_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);
	struct i2c_client *client = ft5x0x_ts->client;


	pr_debug("[FTS]: start auto CLB.\n");
	msleep(200);
	ft5x0x_write_reg(client, 0, 0x40);
	mdelay(100);   //make sure already enter factory mode
	ft5x0x_write_reg(client, 2, 0x4);  //write command to start calibration
	mdelay(300);
	for(i=0;i<100;i++)
	{
		ft5x0x_read_reg(client, 0,&uc_temp);
		if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
		{
			break;
		}
		mdelay(200);
		pr_debug("[FTS]: waiting calibration %d\n",i);
	}
	pr_debug("[FTS]: calibration OK.\n");

	msleep(300);
	ft5x0x_write_reg(client, 0, 0x40);  //goto factory mode
	mdelay(100);   //make sure already enter factory mode
	ft5x0x_write_reg(client, 2, 0x5);  //store CLB result
	mdelay(300);
	ft5x0x_write_reg(client, 0, 0x0); //return to normal mode
	msleep(300);
	pr_debug("[FTS]: store CLB result OK.\n");
	return 0;
}

static int ft5x0x_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];memset(filepath, 0, sizeof(filepath));

	//sprintf(filepath, "/sdcard/%s", firmware_name);
	sprintf(filepath, "%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}


E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
	FTS_BYTE reg_val[2] = {0};
	FTS_DWRD i = 0;

	FTS_DWRD  packet_number;
	FTS_DWRD  j;
	FTS_DWRD  temp;
	FTS_DWRD  lenght;
	FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
	FTS_BYTE  auc_i2c_write_buf[10];
	FTS_BYTE bt_ecc;
	FTS_DWRD  reDL_count = 1;

	struct ft5x06_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);
	struct i2c_client *client = ft5x0x_ts->client;

	while(reDL_count>0 && reDL_count<=3)//most re-try 3 times
	{
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc*/
		ft5x0x_write_reg(client, 0xfc,0xaa);
		mdelay(50);
		/*write 0x55 to register 0xfc*/
		ft5x0x_write_reg(client, 0xfc,0x55);
		pr_debug("[FTS] Step 1: Reset CTPM test\n");

		mdelay(30);

		/*********Step 2:Enter upgrade mode *****/
		pr_debug("[FTS] Step 2:enter new update mode\n");
		auc_i2c_write_buf[0] = 0x55;
		auc_i2c_write_buf[1] = 0xaa;

		do
		{
			i ++;
			ft5x0x_i2c_txdata(&auc_i2c_write_buf[0], 1);
			msleep(5);
			ft5x0x_i2c_txdata(&auc_i2c_write_buf[1], 1);
			msleep(5);
		}while( i < 5 );

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		cmd_write(0x90,0x00,0x00,0x00,4);
		byte_read(reg_val,2);
		pr_debug("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x, Download times=%d\n",reg_val[0],reg_val[1],reDL_count);

		if (reg_val[0] == 0x79 && reg_val[1] == 0x11)
			break;
		else
			reDL_count++;
	}
	if (reg_val[0] == 0x79 && reg_val[1] == 0x11)
	{
		pr_debug("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	}
	else
	{
		return ERR_READID;
	}

	cmd_write(0xcd,0x0,0x00,0x00,1);
	byte_read(reg_val,1);
	pr_debug("[FTS] bootloader version = 0x%x\n", reg_val[0]);

	/*********Step 4:erase app and panel paramenter area ********************/
	pr_debug("[FTS] Step 4: erase app and panel paramenter area. \n");
	cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
	mdelay(2000);
	cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
	mdelay(100);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	pr_debug("[FTS] Step 5: write firmware (FW) to ctpm flash \n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(lenght>>8);
		packet_buf[5] = (FTS_BYTE)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
			packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
		mdelay(FTS_PACKET_LENGTH/6 + 1);
		if ((j * FTS_PACKET_LENGTH % 1024) == 0)
		{
			pr_debug("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;

		for (i=0;i<temp;i++)
		{
			packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6+i];
		}

		byte_write(&packet_buf[0],temp+6);
		mdelay(20);
	}
	//send the last six byte
	for (i = 0; i<6; i++)
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (FTS_BYTE)(temp>>8);
		packet_buf[3] = (FTS_BYTE)temp;
		temp =1;
		packet_buf[4] = (FTS_BYTE)(temp>>8);
		packet_buf[5] = (FTS_BYTE)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i];
		bt_ecc ^= packet_buf[6];

		byte_write(&packet_buf[0],7);
		mdelay(20);
	}
	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	cmd_write(0xcc,0x00,0x00,0x00,1);
	byte_read(reg_val,1);
	pr_debug("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
	if(reg_val[0] != bt_ecc)
	{
		return ERR_ECC;
	}

	/*********Step 7: reset the new FW***********************/
	cmd_write(0x07,0x00,0x00,0x00,1);

	msleep(300);  //make sure CTP startup normally

	return ERR_OK;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
	FTS_BYTE* pbt_buf = FTS_NULL;
	int i_ret;
	int fw_len = sizeof(CTPM_FW);
	unsigned char uc_temp;

	struct ft5x06_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);
	struct i2c_client *client = ft5x0x_ts->client;

	/*Judge the FW, if illegal, stop upgrade and return*/

	if (fw_len < 8 || fw_len > 32*1024) {
		pr_debug("[FTS] %s:FW length error\n", __func__);
		return -EIO;
	}

	if ((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
		&& (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
		&& (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF) {

		//=========FW upgrade========================*/
		pr_debug("[FTS] %s\n",__FUNCTION__);
		ft5x0x_read_reg(client, 0xA8,&uc_temp);

		pbt_buf = CTPM_FW;
		/*Call upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));

		if (i_ret != 0)
		{
			pr_debug("[FTS] upgrade failed i_ret = %d.\n", i_ret);
		}
		else
		{
			pr_debug("[FTS] upgrade successfully.\n");
			fts_ctpm_auto_clb();  //start auto CLB
		}
	}
	else {
		pr_debug("%s:FW format error\n", __func__);
		return -EBADFD;
	}

	return i_ret;
}

static int ft5x0x_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
		}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
		}
	inode=pfile->f_dentry->d_inode;
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}


int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
	FTS_BYTE*     pbt_buf = FTS_NULL;
	int i_ret; u8 fwver;
	int fwsize = ft5x0x_GetFirmwareSize(firmware_name);

	struct ft5x06_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);
	struct i2c_client *client = ft5x0x_ts->client;

	if(fwsize <= 0)
	{
		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
	}
	//=========FW upgrade========================*/
	pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ft5x0x_ReadFirmware(firmware_name, pbt_buf))
	{
		pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
		kfree(pbt_buf);
		return -1;
	}
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(pbt_buf, fwsize);
	if (i_ret != 0)
	{
		pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
		//error handling ...
		//TBD
	}
	else
	{
		pr_info("[FTS] upgrade successfully.\n");
		if(ft5x0x_read_reg(client, FT_REG_FW_VER, &fwver)>=0)
			pr_info("the new fw ver is 0x%02x\n", fwver);
		fts_ctpm_auto_clb();  //start auto CLB
	}
	kfree(pbt_buf);
	return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
	unsigned int ui_sz;

	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2)
	{
		return CTPM_FW[ui_sz - 2];
	}
	return 0x00; //default value
}

static void ft5x06_reset(struct ft5x06_ts_data *ts)
{   // Active RESET pin to wakeup ts
	pr_debug("FT5x06:start reset\n");
	gpio_direction_output(ts->pdata->reset_gpio, 0);
	mdelay(5);
	gpio_direction_output(ts->pdata->reset_gpio, 1);
	mdelay(195);
	pr_debug("FT5x06:end reset\n");
}

static u8 ft5x0x_enter_factory(struct ft5x06_ts_data *ft5x0x_ts)
{
	u8 regval;

	flush_workqueue(ft5x0x_ts->ts_workqueue);
	disable_irq_nosync(ft5x0x_ts->pdata->irq_gpio);
	ft5x0x_write_reg(ft5x0x_ts->client,0x00, 0x40);  //goto factory mode
	mdelay(100);   //make sure already enter factory mode
	if(ft5x0x_read_reg(ft5x0x_ts->client,0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x40)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	return 0;

}

static void ft5x0x_rx_cap(struct i2c_client *client, u8 cap)
{
	int i=0;
	u8 reg_value;

	for(i=0 ; i<FT5x0x_RX_NUM ; i++)
		ft5x0x_write_reg(client, 0xA0+i, cap);   /* set rx cap */
		for(i=0 ; i<FT5x0x_RX_NUM ; i++){
			ft5x0x_read_reg(client, 0xA0+i, &reg_value);
			pr_debug("[TOUCH] register %d set as %d\n", 0xA0+i, reg_value);
		}
	pr_debug("Set RX cap success!\n");
}

static u8 ft5x0x_enter_work(struct ft5x06_ts_data *ft5x0x_ts)
{
	u8 regval;
	ft5x0x_write_reg(ft5x0x_ts->client,0x00, 0x00); //return to normal mode
	msleep(100);

	if(ft5x0x_read_reg(ft5x0x_ts->client,0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x00)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	enable_irq(ft5x0x_ts->pdata->irq_gpio);
	return 0;
}

static int fts_Get_RawData(u16 RawData[][FT5x0x_RX_NUM], int step)
{
	int retval  = 0;
	int i       = 0;
	//u16 dataval = 0x0000;
	u8  devmode = 0x00;
	u8  rownum  = 0x00;

	u8 read_buffer[FT5x0x_RX_NUM * 2];
	u8 read_len = 0;
	//u8 write_buffer[2];
	struct ft5x06_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);

	struct i2c_client *client = ft5x0x_ts->client;
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 1,
			.len	= FT5x0x_RX_NUM * 2,
			.buf	= read_buffer,
		}
	};

	if(ft5x0x_enter_factory(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR: could not enter factory mode", __FUNCTION__);
		retval = -1;
		goto error_return;
	}

	mdelay(100);

	if(step==1){
		ft5x0x_rx_cap(client,0x14);
		mdelay(200);
	}

	for(i = 0; i < 3; i++){
		devmode = 0x00;
		//scan
		if(ft5x0x_read_reg(client, 0x00, &devmode)<0)
		{
			pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
			retval = -1;
			goto error_return;
		}
		devmode |= 0x80;
		if(ft5x0x_write_reg(client, 0x00, devmode)<0)
		{
			pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
			retval = -1;
			goto error_return;
		}
		msleep(20);
		if(ft5x0x_read_reg(client, 0x00, &devmode)<0)
		{
			pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
			retval = -1;
			goto error_return;
		}
		if(0x00 != (devmode&0x80))
		{
			pr_err("%s %d ERROR: could not scan", __FUNCTION__, __LINE__);
			retval = -1;
			goto error_return;
		}
	}
	pr_debug("Read rawdata .......\n");
	for(rownum=0; rownum<FT5x0x_TX_NUM; rownum++)
	{
		memset(read_buffer, 0x00, (FT5x0x_RX_NUM * 2));

		if(ft5x0x_write_reg(client, 0x01, rownum)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		msleep(1);
		read_len = FT5x0x_RX_NUM * 2;
		if(ft5x0x_write_reg(client, 0x10, read_len)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		retval = i2c_transfer(client->adapter, msgs, 1);
		if (retval < 0)
		{
			pr_err("%s ERROR:Could not read row %u raw data", __FUNCTION__, rownum);
			retval = -1;
			goto error_return;
		}
		for(i=0; i<FT5x0x_RX_NUM; i++)
		{
			RawData[rownum][i] = (read_buffer[i<<1]<<8) + read_buffer[(i<<1)+1];
			pr_debug("i = %x , RawData[rownum][i]= %x\n", i, RawData[rownum][i]);
		}
	}
error_return:
	if(step==1){
		ft5x0x_rx_cap(client,0x41);
		mdelay(200);
	}

	if(ft5x0x_enter_work(ft5x0x_ts)<0)
	{
		pr_err("%s ERROR:could not enter work mode ", __FUNCTION__);
		retval = -1;
	}
	return retval;
}

/* read chip firmware version */
static ssize_t ft5x0x_tpfwver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = NULL;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ssize_t num_read_chars = 0;
	u8	   fwver = 0;
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);

	if(ft5x0x_read_reg(client, FT_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}
static ssize_t ft5x0x_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = NULL;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ssize_t num_read_chars = 0;
	int retval;
	unsigned long wmreg=0;u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5];
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&data->device_mode_mutex);
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			pr_info("[FTS]: please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);

	if (0 != retval)
	{
		pr_err("[FTS]: %s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}

	if(2 == num_read_chars)
	{
		//read register
		regaddr = wmreg;
		if(ft5x0x_read_reg(client, regaddr, &regvalue) < 0)
			pr_err("[FTS]: Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("[FTS]: the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if(ft5x0x_write_reg(client, regaddr, regvalue)<0)
			pr_err("[FTS]: Could not write the register(0x%02x)\n", regaddr);
		else
			pr_info("[FTS]: Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}


static ssize_t ft5x0x_fwupdate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return sprintf(buf, "%s\n", upgrade_result);
}
//upgrade from *.i
static ssize_t ft5x0x_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = NULL;
	u8 uc_host_fm_ver;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int i_ret, err;
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	pr_debug("[FTS]: ft5x0x_fwupdate start\n");

	err = ft5x06_power_on(data, true);
	if (err) {
		dev_err(dev, "power on failed");
		return err;
	}
	ft5x06_reset(data);

	mutex_lock(&data->device_mode_mutex);

	disable_irq(data->pdata->irq_gpio);
	data->upgrade_flag = 1;	//close early-suspend
	i_ret = fts_ctpm_fw_upgrade_with_i_file();
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		pr_debug("[FTS]: %s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
		sprintf(upgrade_result,"0");
	}
	else
	{
		pr_debug("[FTS]: %s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
		sprintf(upgrade_result,"1");
	}
	data->upgrade_flag = 0; //Reset early-suspend flag
	enable_irq(data->pdata->irq_gpio);

	mutex_unlock(&data->device_mode_mutex);
	pr_debug("[FTS]: ft5x0x_fwupdate end\n");
	return count;
}

static ssize_t ft5x0x_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char fwname[128];
	int err=0;
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	err = ft5x06_power_on(data, true);
	if (err) {
		dev_err(dev, "power on failed");
		return err;
	}
	ft5x06_reset(data);

	mutex_lock(&data->device_mode_mutex);
	disable_irq(data->pdata->irq_gpio);
	data->upgrade_flag = 1; //close early-suspend

	fts_ctpm_fw_upgrade_with_app_file(fwname);

	data->upgrade_flag = 0; //enable early-suspend
	enable_irq(data->pdata->irq_gpio);
	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x0x_rawdata_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = NULL;
	ssize_t num_read_chars = 0;
	int i=0, j=0;
	u16 RawData[FT5x0x_TX_NUM][FT5x0x_RX_NUM];

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );

	mutex_lock(&data->device_mode_mutex);

	if(fts_Get_RawData(RawData, 0)<0)
		sprintf(buf, "%s", "could not get rawdata\n");
	else
	{
		for(i=0; i<FT5x0x_TX_NUM; i++)
		{
			for(j=0; j<FT5x0x_RX_NUM; j++)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", RawData[i][j]);
			}
			buf[num_read_chars-1] = '\n';
		}
	}
	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ft5x0x_tpfwver_show, NULL);
/* upgrade from *.i */
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ft5x0x_fwupdate_show, ft5x0x_fwupdate_store);
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, NULL, ft5x0x_tprwreg_store);
/* upgrade from app.bin */
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, NULL, ft5x0x_fwupgradeapp_store);
/* read panel raw-data from self-test results */
static DEVICE_ATTR(ftsrawdatashow, S_IRUGO|S_IWUSR, ft5x0x_rawdata_show, NULL);

static struct attribute *ft5x0x_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsrawdatashow.attr,
	NULL
};

static struct attribute_group ft5x0x_attribute_group = {
	.attrs = ft5x0x_attributes
};

static ssize_t ftm_tp_rawdata_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i=0;
	u16 value=0;

	value = FT5x0x_TX_NUM * FT5x0x_RX_NUM;

	pr_debug("[FTS]: %s: START\n",__FUNCTION__);

	ret += sprintf(&(buf[ret]), "Data=3;(D,%d,%d,%d,%d,",FTM_RAW_DATA_DIVISION,value,FTM_RAWDATA_MAX,FTM_RAWDATA_MIN);

	for(i =0; i<FTM_RAW_DATA_DIVISION;i++){
		if(i==0)
			ret += sprintf(&(buf[ret]), "%d",RDNumber[i]);
		else
			ret += sprintf(&(buf[ret]), ",%d",RDNumber[i]);
	}

	ret += sprintf(&(buf[ret]), ");(M,2,%d,%d)",RawDataLimit[0],RawDataLimit[1]);

	ret += sprintf(&(buf[ret]), ";(V,%d,%d,%d)", RawDataValue_1, RawDataValue_2, RawDataDiffMin);

	pr_debug("[FTS]: ret = %d\n\n@@",ret);
	for(i=0;i<ret;i++)
		pr_debug("%c",buf[i]);
	pr_debug("\n\n");

	pr_debug("[FTS]: %s: END\n",__FUNCTION__);
	return ret;
}

static int FTM_Distribution_set(u16* raw, u16* range, u16 size, u16 up_value, u16 low_value, u16 rval, u16* max, u16* min)
{
	int i = 0;
	u16 k =0;
	u16 temp_max, temp_min;
	temp_max = low_value;
	temp_min = up_value;
	pr_debug("========== %d,%d,%d,%d\n",FTM_RAW_DATA_DIVISION, rval,up_value,low_value);
	for(i=0;i<size;i++){
		if(raw[i]>up_value){
			range[FTM_RAW_DATA_DIVISION-1]=range[FTM_RAW_DATA_DIVISION-1]+1;
			pr_debug("raw=%d,rang=%d::\n", raw[i],FTM_RAW_DATA_DIVISION-1);
		}
		else if(raw[i]<low_value){
			range[0]=range[0]+1;
			pr_debug("raw=%d,rang=0::\n", raw[i]);
		}
		else
		{
			k = (raw[i]- low_value)/rval;
			if((raw[i]- low_value)%rval > 0)
				k = k+1;
			if(k==FTM_RAW_DATA_DIVISION-1)
				k = FTM_RAW_DATA_DIVISION-2;
			range[k]=range[k]+1;
			pr_debug("raw=%d,rang=%d::\n", raw[i],k);
		}
		if(raw[i]>temp_max)
			temp_max = raw[i];
		if(raw[i]<temp_min)
			temp_min = raw[i];
	}
	*max = temp_max;
	*min = temp_min;
	pr_debug("@@@ 2 max=%d,min=%d\n", *max,*min);
	return 0;
}

static int FTM_RawDataDistribution(void)
{
	int i=0;
	u16 max_1 =0, min_1 =0;
	pr_debug("[FTS]: %s\n", __func__);

	RawDataLimit[0] = FTM_RAWDATA_MIN;
	RawDataLimit[1] = FTM_RAWDATA_MAX;
	memset(RDNumber, 0x0, sizeof(RDNumber));
	//Mutual raw data
	for(i = 0; i < FT5x0x_TX_NUM; i++){
		if(FTM_Distribution_set(&RawData_1[i][0], RDNumber, FT5x0x_RX_NUM, FTM_RAWDATA_MAX, FTM_RAWDATA_MIN,FTM_RD_RANGE_VALUE,&max_1,&min_1)){
			pr_debug("[FTS]: %s: Self Test Fail on Mutual Raw, %d!\n", __func__, i);
		}
		if(RawDataLimit[0]<max_1)
			RawDataLimit[0] = max_1;
		if(RawDataLimit[1]>min_1)
			RawDataLimit[1] = min_1;
	}
	return 0;
}

void ft5x0x_selftest(void)
{
	u16 temp=0;
	int i=0, j=0;
	int itest =0;
	int min = FTM_RAWDATA_MAX;

	struct ft5x06_ts_data * ft5x0x_ts =  i2c_get_clientdata(this_client);
	st_result = 0;

	disable_irq(ft5x0x_ts->pdata->irq_gpio);
	pr_debug("[FTS]: test start 1\n");
	if(fts_Get_RawData(RawData_1, 0)<0)
		pr_debug("[FTS]: could not get rawdata\n");

	pr_debug("[FTS]: test start 2\n");
	if(fts_Get_RawData(RawData_2, 1)<0)
		pr_debug("[FTS]: could not get rawdata\n");

	for(i=0; i<FT5x0x_TX_NUM; i++)
	{
		for(j=0; j<FT5x0x_RX_NUM; j++)
		{
			temp = abs(RawData_1[i][j]-RawData_2[i][j]);
			if((temp<FTM_RAWDATA_DIFF) || (RawData_1[i][j] > FTM_RAWDATA_MAX) || (RawData_1[i][j] < FTM_RAWDATA_MIN))
			{
				itest = 1;
				pr_err("[FTS]: i = %d, j=%d, RawData_1[i][j] =%d, RawData_2[i][j]=%d\n", i, j, RawData_1[i][j], RawData_2[i][j]);
			}
			if(temp<min){	/* Store min diff value and related information  */
				min = temp;
				RawDataValue_1 = RawData_1[i][j];
				RawDataValue_2 = RawData_2[i][j];
			}
		}
	}

	RawDataDiffMin = min;

	if(itest ==0)
	{
		pr_debug("[FTS]: Self-test Pass\n");
		sprintf(test_result,"0");
	}
	else if(itest ==1)
	{
		pr_debug("[FTS]: Self-test Fail\n");
		sprintf(test_result,"1");
		st_result |= 1;
	}
	enable_irq(ft5x0x_ts->pdata->irq_gpio);
	pr_debug("[FTS]: test end\n");
}

static struct device_attribute ft5xx6_dev_attr_test = __ATTR(tp_rawdata, S_IRUGO|S_IWUSR,
		ftm_tp_rawdata_show, NULL);

static struct attribute *ft5xx6_attributes[] = {
	&ft5xx6_dev_attr_test.attr,
	NULL
};

static struct attribute_group ft5xx6_attribute_group = {
	.attrs = ft5xx6_attributes
};

static ssize_t ft5x06_test_show(struct device *dev,
	struct device_attribute *attr,
	char *buf){
	return sprintf(buf, "%s\n", test_result);
}

static ssize_t ft5x06_test_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count){
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int value = simple_strtoul(buf, NULL, 10);
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );

	if (value == 0)
	{
		pr_debug("[FTS]: [Cmd:%d] Run Self-test.\n", value);
		ft5x0x_selftest();
		FTM_RawDataDistribution();
	}
	else
		pr_err("[FTS]: [Cmd:%d] Unknow command.\n", value);

	ft5x06_reset(data);

	return count;
}
static DEVICE_ATTR(test, 0664, ft5x06_test_show, ft5x06_test_store);

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	/* get panel name info */
	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	/* panel resolution */
	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	/* vdd, reset, irq gpio info */
	pdata->vcc_i2c_always_on = of_property_read_bool(np, "focaltech,vcc-i2c-always-on");

	pdata->reg_enable = of_property_read_bool(np, "focaltech,regulator-enable");
	if (!(pdata->reg_enable))
	{
		pdata->enable_gpio = of_get_named_gpio_flags(np, "vdd_enable",
					0, &pdata->enable_gpio_flags);
		if (pdata->enable_gpio < 0)
			return pdata->enable_gpio;
	}

	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	rc = of_property_read_u32(np, "focaltech,x-filp",
							&temp_val);
	if (!rc)
		pdata->x_filp = temp_val;
	else
		pdata->x_filp = 0;

	rc = of_property_read_u32(np, "focaltech,y-filp",
							&temp_val);
	if (!rc)
		pdata->y_filp = temp_val;
	else
		pdata->y_filp = 0;

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}
	data->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!data->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	mutex_init(&data->device_mode_mutex);
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	this_client = client;
	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, FT_PRESS, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	/* power pin init */
	err = ft5x06_power_init(data, true);
	if (err) {
		dev_err(&client->dev, "power init failed, %d", __LINE__);
		goto unreg_inputdev;
	}

	/* power on sequence */
	err = ft5x06_power_on(data, true);
	if (err) {
		dev_err(&client->dev, "power on failed, %d", __LINE__);
		goto pwr_deinit;
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}
	else
		pr_err("[FTS]: irq gpio is not valid\n");

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	else
		pr_err("[FTS]: reset gpio is not valid\n");

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_reset_gpio;
	}

	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if (pdata->family_id != reg_value) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_reset_gpio;
	}

	data->family_id = reg_value;

	err = request_threaded_irq(client->irq, NULL,
				   ft5x06_ts_interrupt, pdata->irqflags,
				   client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_reset_gpio;
	}

	err = sysfs_create_group(&client->dev.kobj,&ft5xx6_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ft5xx6_attribute_group);
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_test);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_test_sys;
	}

	err = sysfs_create_group(&client->dev.kobj, &ft5x0x_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ft5x0x_attribute_group);
	}
	else
	{
		pr_debug("ft5x0x:%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	/* Add to read touch info for FQC and FTM */
	fih_info_init(&client->dev);

	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_test_sys:
	device_remove_file(&client->dev, &dev_attr_test);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);

	return err;
}

static int __devexit ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);

	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);

	destroy_workqueue(data->ts_workqueue);
	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5406",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = __devexit_p(ft5x06_ts_remove),
	.driver = {
			.name = "ft5x06_ts",
			.owner = THIS_MODULE,
			.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
			.pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
