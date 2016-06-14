/* drivers/input/touchscreen/IT7280_ts_i2c.c
 *
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "it7256_ts_i2c.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>

#define IT7280_I2C_NAME "it7280_ts_i2c"
#define MAX_SUPPORT_POINT 10

#define ITE_VTG_MIN_UV		2700000
#define ITE_VTG_MAX_UV		3300000
#define ITE_I2C_VTG_MIN_UV	1800000
#define ITE_I2C_VTG_MAX_UV	1800000

#define WTEMP_SIZE 8000
#define GTEMP_SIZE 16000

#define GOLDEN_SAMPLE_FILE "/system/etc/g_sample.bin"
#define GOLDEN_SAMPLE2_FILE "/system/etc/g_sample_second.bin"

#define FW_BUF_SIZE 0x8000
#define CFG_BUF_SIZE 0x500

#define APP_IT7256_FW_FILE "/data/it7256.fw"
#define APP_IT7256_CFG_FILE "/data/it7256.cfg"

#define TP_TEST_RESULT_FILE "/data/TPtest_result"

static int ite7280_major = 0;	// dynamic major by default
static int ite7280_minor = 0;
static struct cdev ite7280_cdev;
static struct class *ite7280_class = NULL;
static dev_t ite7280_dev;
static struct input_dev *input_dev;
static int max_slot = 0;
static int fingerIDUseBefore[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int fingerIDUseNow[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int fw_upgrade_success = 0;
static int selftest_success = -1;
static u8 *wTemp;
static u8 *gTemp;
static int tx = 0;
static int rx = 0;
static int thres = 0;
static char config_id[10];
static char fih_touch_buildid[20];
static char it7256_fw_name[256] = "it7256.fw";
static char it7256_cfg_name[256] = "it7256.cfg";

static int suspend_status = -1;

static int Source_flag = -1;

#if 0
static char configuration_ver[5]={0};
static char chip_configuration_ver[5]={0};
#endif

static int i2cAdvancedWriteToIt7280(struct i2c_client *client, unsigned char bufferIndex,unsigned char const dataBuffer[], unsigned short dataLength);
static int i2cAdvancedReadFromIt7280(struct i2c_client *client, unsigned char bufferIndex,unsigned char dataBuffer[], unsigned short dataLength);
static bool fnEnterFirmwareUpgradeMode(void);

static char TouchKeyCode[4] = {
	KEY_MENU,	// Menu Key
	KEY_HOME,	// Home Key
	KEY_BACK,	// Back Key
	KEY_SEARCH,	// Search Key
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7280_ts_early_suspend(struct early_suspend *h);
static void IT7280_ts_late_resume(struct early_suspend *h);
#endif

static int fih_touch_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;

	snprintf(fih_touch_buildid, PAGE_SIZE, "ITE-V%d.%d.%d%d\n", config_id[1], config_id[2],config_id[3], config_id[4]);
	len = snprintf(page, PAGE_SIZE, "%s", fih_touch_buildid);
	return len;
}

static int fih_init(void)
{
	if (create_proc_read_entry("AllHWList/Touch", 0, NULL, fih_touch_read_proc, NULL) == NULL) {
		proc_mkdir("AllHWList", NULL);
		if (create_proc_read_entry("AllHWList/Touch", 0, NULL, fih_touch_read_proc, NULL) == NULL) {
			pr_err("fail to create proc/AllHWList/Touch\n");
			return -1;
		}
	}
	return 0;
}

static int i2cReadFromIt7280(struct i2c_client *client, unsigned char bufferIndex, unsigned char dataBuffer[], unsigned short dataLength)
{
	int ret;
	struct i2c_msg msgs[2] = { {
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &bufferIndex
		}, {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = dataLength,
		.buf = dataBuffer
		}
	};
	memset(dataBuffer, 0xFF, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);

	return ret;
}

static int i2cWriteToIt7280(struct i2c_client *client, unsigned char bufferIndex, unsigned char const dataBuffer[], unsigned short dataLength)
{
	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = { {
		.addr = client->addr,
		.flags = 0,
		.len = dataLength + 1,
		.buf = buffer4Write
		}
	};

	if(dataLength < 256) {
		buffer4Write[0] = bufferIndex;
		memcpy(&(buffer4Write[1]), dataBuffer, dataLength);

		return i2c_transfer(client->adapter, msgs, 1);
	}
	else
		return -1;
}

static int i2cAdvancedReadFromIt7280(struct i2c_client *client, unsigned char bufferIndex, unsigned char dataBuffer[], unsigned short dataLength)
{
	int ret;
	struct i2c_msg msgs[2] = { {
		.addr = client->addr,
		.flags = I2C_M_NOSTART,
		.len = 1,
		.buf = &bufferIndex
		}, {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = dataLength,
		.buf = dataBuffer
		}
	};

	memset(dataBuffer, 0xFF, dataLength);
	ret = i2c_transfer(client->adapter, msgs, 2);

	return ret;
}

static int i2cAdvancedWriteToIt7280(struct i2c_client *client, unsigned char bufferIndex, unsigned char const dataBuffer[], unsigned short dataLength)
{
	unsigned char buffer4Write[256];
	struct i2c_msg msgs[1] = { {
		.addr = client->addr,
		.flags = 0,
		.len = dataLength + 1,
		.buf = buffer4Write
		}
	};

	if(dataLength < 256) {
		buffer4Write[0] = bufferIndex;
		memcpy(&(buffer4Write[1]), dataBuffer, dataLength);

		return i2c_transfer(client->adapter, msgs, 1);
	}
	else
		return -1;
}

static bool waitCommandDone(void)
{
	unsigned char ucQuery = 0xFF;
	unsigned int count = 0;

	do{
		if((i2cReadFromIt7280(gl_ts->client, 0x80, &ucQuery, 1)) <= 0){
			ucQuery = 0x01;
		}
		count++;
	}while(ucQuery & 0x01 && count < 500);

	if( ucQuery == 0){
		return  true;
	}else{
		return  false;
	}
}

static bool fnFirmwareReinitialize(void)
{
	u8 pucBuffer[2];
	u8 wCommandResponse[2];

	pr_info("[IT7256] : %s \n", __func__);

	waitCommandDone();
	pucBuffer[0] = 0x0C;
	if((i2cWriteToIt7280(gl_ts->client, 0x20, pucBuffer, 1)) <= 0) {
		return false;
	}

	waitCommandDone();
	if((i2cReadFromIt7280(gl_ts->client, 0xA0, wCommandResponse, 2 )) <= 0) {
		return false;
	}

	if(wCommandResponse[0] != 0x00 && wCommandResponse[1] != 0x00)
	{
		return false;
	}

	return true;
}

static bool fnEnterFirmwareUpgradeMode(void)
{
	char pucBuffer[MAX_BUFFER_SIZE];
	u8 wCommandResponse[2];

	pr_info("[IT7256] : %s \n", __func__);

	waitCommandDone();
	pucBuffer[0] = 0x08;
	pucBuffer[1] = 0x00;

	if((i2cWriteToIt7280(gl_ts->client, 0x20, pucBuffer, 2)) <= 0) {
		return false;
	}

	waitCommandDone();
	if((i2cReadFromIt7280(gl_ts->client, 0xA0, wCommandResponse, 2)) <= 0) {
		return false;
	}

	if(wCommandResponse[0] != 0x00 && wCommandResponse[1] != 0x00){
		return false;
	}

	return true;
}

static bool fnExitFirmwareUpgradeMode(void)
{
	char pucBuffer[MAX_BUFFER_SIZE];
	u8 wCommandResponse[2];

	pr_info("[IT7256] : %s \n", __func__);

	waitCommandDone();
	pucBuffer[0] = 0x08;
	pucBuffer[1] = 0x80;

	if((i2cWriteToIt7280(gl_ts->client, 0x20, pucBuffer, 2)) <= 0) {
		return false;
	}

	waitCommandDone();

	if((i2cReadFromIt7280(gl_ts->client, 0xA0, wCommandResponse, 2)) <= 0) {
		return false;
	}

	if(wCommandResponse[0] != 0x00 && wCommandResponse[1] != 0x00) {
		return false;
	}

	return true;
}

static bool fnSetStartOffset(unsigned short wOffset)
{
	u8 pucBuffer[MAX_BUFFER_SIZE];
	u8 wCommandResponse[2];

	waitCommandDone();

	pucBuffer[0] = 0x09;
	pucBuffer[1] = 0;
	pucBuffer[2] = ( wOffset & 0x00FF );
	pucBuffer[3] = (( wOffset & 0xFF00 ) >> 8 );

	if((i2cWriteToIt7280(gl_ts->client, 0x20, pucBuffer, 4)) <= 0) {
		return false;
	}

	waitCommandDone();

	if((i2cReadFromIt7280(gl_ts->client, 0xA0, wCommandResponse, 2)) <= 0) {
		return false;
	}

	if(wCommandResponse[0] != 0x00 && wCommandResponse[1] != 0x00) {
		return false;
	}

	return true;
}

static bool fnWriteAndCompareFlash(unsigned int nLength, u8* pnBuffer, unsigned short nStartOffset)
{
	unsigned int nIndex = 0;
	unsigned char buffer[130] = {0};
	unsigned char bufWrite[130] = {0};
	unsigned char bufRead[130] = {0};
	unsigned char nReadLength;
	int retryCount;
	int i;

	nReadLength = 128;

	while ( nIndex < nLength ) {
		retryCount = 0;
		do {
			fnSetStartOffset(nStartOffset + nIndex);

			buffer[0] = 0x0A;
			buffer[1] = nReadLength;
			for (i = 0; i < nReadLength; i++) {
				bufWrite[i] = buffer[2 + i] = pnBuffer[nIndex + i];
			}
			i2cWriteToIt7280(gl_ts->client, 0x20, buffer, 130);

			// Read from Flash
			buffer[0] = 0x0B;
	  		buffer[1] = nReadLength;

	 		fnSetStartOffset(nStartOffset + nIndex);
			i2cWriteToIt7280(gl_ts->client, 0x20, buffer, 2);
			waitCommandDone();
			i2cReadFromIt7280(gl_ts->client, 0xA0, bufRead, nReadLength);

			// Compare
			for (i = 0; i < nReadLength; i++) {
				if (bufRead[i] != bufWrite[i]) {
					pr_info("[IT7256] : %s retryCount = %d\n", __func__,retryCount);
					break;
				}
			}
			if (i == nReadLength) break;
		} while ( retryCount++ < 4 );

		if ( retryCount == 4 && i != nReadLength )
			return false;
		nIndex += nReadLength;
	}

	return true;
}

static bool fnFirmwareDownload(unsigned int unFirmwareLength, u8* pFirmware, unsigned int unConfigLength, u8* pConfig)
{
	pr_info("[IT7256] : %s \n", __func__);

	if((unFirmwareLength == 0 || pFirmware == NULL) && (unConfigLength == 0 || pConfig == NULL)) {
		return false;
	}

	if(!fnEnterFirmwareUpgradeMode()) {
		return false;
	}

	if((unFirmwareLength != 0) && (pFirmware != NULL)) {
		/*Download firmware*/
		if(!fnWriteAndCompareFlash(unFirmwareLength, pFirmware, 0)) {
			return false;
		}
	}

	if((unConfigLength != 0) && (pConfig != NULL)) {
		/*Download configuration*/
		unsigned short wFlashSize = FW_BUF_SIZE;
		pr_info("[IT7256] : %d %d %d %d  xxxxxxxxxx\n", pConfig[unConfigLength-8], pConfig[unConfigLength-7], pConfig[unConfigLength-6], pConfig[unConfigLength-5] );
		pr_info("[IT7256] : %d %d %d %d  xxxxxxxxxx\n", pConfig[unConfigLength-8], pConfig[unConfigLength-7], pConfig[unConfigLength-6], pConfig[unConfigLength-5] );
		pr_info("[IT7256] : %d %d %d %d  xxxxxxxxxx\n", pConfig[unConfigLength-8], pConfig[unConfigLength-7], pConfig[unConfigLength-6], pConfig[unConfigLength-5] );
		if(!fnWriteAndCompareFlash(unConfigLength, pConfig, wFlashSize - (unsigned short)unConfigLength)) {
			return false;
		}
	}

	if(!fnExitFirmwareUpgradeMode()) {
		return false;
	}

	if(!fnFirmwareReinitialize()) {
		return false;
	}

	return true;
}

static int Upgrade_FW_CFG(struct device *dev)
{
	const struct firmware *cfg = NULL;
	const struct firmware *fw = NULL;
	int ret = 0;
	u8 *config_buf;
	u8 *fw_buf;
	int fw_size =0, cfg_size = 0;
	unsigned char ucQuery = 0;

	pr_info("[IT7256] : %s \n", __func__);

	fw_buf = kzalloc(FW_BUF_SIZE, GFP_KERNEL);
	if (fw_buf == NULL) {
		pr_err("[IT7256] : fw_buf kzalloc failed\n");
		return 1;
	}

	config_buf = kzalloc(CFG_BUF_SIZE, GFP_KERNEL);
	if (config_buf == NULL) {
		pr_err("[IT7256] : config_buf kzalloc failed\n");
		kfree(fw_buf);
		return 1;
	}

	ret = request_firmware(&fw, it7256_fw_name, dev);
	if (ret < 0) {
		pr_info("[IT7256] : open /system/etc/firmware/%s failed\n", it7256_fw_name);
	}
	else {
		memcpy(fw_buf ,fw->data, fw->size);
		pr_info("[IT7256] : fw_ver : %d, %d, %d, %d\n",*(fw->data + 8), *(fw->data + 9), *(fw->data + 10), *(fw->data + 11));
		pr_info("[IT7256] : fw_ver : %d, %d, %d, %d\n",fw_buf[8], fw_buf[9], fw_buf[10], fw_buf[11]);
		pr_info("[IT7256] : --------------------- fw_size = %x\n", fw->size);
		fw_size = fw->size;
	}

	ret = request_firmware(&cfg, it7256_cfg_name, dev);
	if (ret  < 0) {
		pr_info("[IT7256] : open /system/etc/firmware/%s failed\n",it7256_cfg_name);

		kfree(config_buf);
		kfree(fw_buf);

		release_firmware(fw);
		return 1;
	}
	else {
		memcpy(config_buf ,cfg->data, cfg->size);
		pr_info("[IT7256] : cfg_ver : %d, %d, %d, %d\n",*(cfg->data + (cfg->size-8)), *(cfg->data + (cfg->size-7)), *(cfg->data + (cfg->size-6)), *(cfg->data + (cfg->size-5)));
		pr_info("[IT7256] : cfg_ver : %d, %d, %d, %d\n",config_buf[cfg->size-8], config_buf[cfg->size-7], config_buf[cfg->size-6], config_buf[cfg->size-5]);
		pr_info("[IT7256] : --------------------- config_size = %x\n", cfg->size);
		cfg_size = cfg->size;
	}

#if 0
	snprintf(configuration_ver, PAGE_SIZE, "%d%d%d%d",config_buf[cfg->size-8], config_buf[cfg->size-7],config_buf[cfg->size-6], config_buf[cfg->size-5]);
	pr_info("[IT7256] : chip_configuration_ver = %s\n", chip_configuration_ver);
	if(!strncmp(chip_configuration_ver,configuration_ver,4)) {
		pr_info("[IT7256] : %s(): Config is the same\n", __func__);
		enable_irq(gl_ts->client->irq);

		kfree(config_buf);
		kfree(fw_buf);

		release_firmware(fw);
		release_firmware(cfg);
		return 1;
	}
#endif

	disable_irq(gl_ts->client->irq);
	if (fnFirmwareDownload(fw_size, fw_buf, cfg_size, config_buf) == false) {
		/*fail*/
		enable_irq(gl_ts->client->irq);

		kfree(config_buf);
		kfree(fw_buf);

		release_firmware(fw);
		release_firmware(cfg);
		return 1;
	}
	else {
		/*success*/
		enable_irq(gl_ts->client->irq);
		kfree(config_buf);
		kfree(fw_buf);

		release_firmware(fw);
		release_firmware(cfg);

		/*Get configuration information*/
		config_id[0] = 0x01;
		config_id[1] = 0x06;
		i2cWriteToIt7280(gl_ts->client, 0x20, config_id, 2);

		do {
			i2cReadFromIt7280(gl_ts->client, 0x80, &ucQuery, 1);
		} while (ucQuery & 0x01);

		memset(&config_id, 0, sizeof(config_id));
		i2cReadFromIt7280(gl_ts->client, 0xA0, config_id, 7);
		pr_info("[IT7256] : %s() chip-version : %d.%d.%d%d\n", __func__,config_id[1], config_id[2],config_id[3], config_id[4]);

		return 0;
	}
}

static ssize_t IT7280_init_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("[IT7256] : %s\n", __func__);

	if(!Source_flag)
		return snprintf(buf, PAGE_SIZE, "Main-Source\n");
	else
		return snprintf(buf, PAGE_SIZE, "Second-Source\n");
}

static int AppUpgrade_FW_CFG(struct device *dev)
{
	u8 *fw_buf;
	struct file* cfg = NULL;
	struct file* fw = NULL;
	u8 *config_buf;
	off_t fw_size = 0;
	off_t cfg_size = 0;
	struct inode *inode;
	mm_segment_t fs;
	unsigned char ucQuery = 0;

	pr_info("[IT7256] : %s \n", __func__);

	fs = get_fs();
	set_fs(get_ds());

	fw_buf = kzalloc(FW_BUF_SIZE, GFP_KERNEL);
	if (fw_buf == NULL) {
		pr_err("[IT7256] : %s fw_buf kzalloc failed\n",__func__);
		set_fs(fs);
		return 1;
	}

	config_buf = kzalloc(CFG_BUF_SIZE, GFP_KERNEL);
	if (config_buf == NULL) {
		pr_err("[IT7256] : %s config_buf kzalloc failed\n",__func__);
		set_fs(fs);
		kfree(fw_buf);
		return 1;
	}

	if(NULL == fw)
		fw = filp_open(APP_IT7256_FW_FILE, O_RDONLY, 0);

	if(IS_ERR(fw)) {
		pr_info("[IT7256] : %s open /data/it7280.fw failed = %ld\n", __func__, PTR_ERR(fw));
	}

	else {
		inode=fw->f_dentry->d_inode;
		fw_size = inode->i_size;
		fw->f_op->read(fw, fw_buf, fw_size, &fw->f_pos);
		pr_info("[IT7256] : %s fw_ver : %d, %d, %d, %d\n",__func__,fw_buf[8], fw_buf[9], fw_buf[10], fw_buf[11]);
		pr_info("[IT7256] : %s --------------------- fw_size = %ld\n", __func__,fw_size);
	}

	if(NULL == cfg) {
		cfg = filp_open(APP_IT7256_CFG_FILE, O_RDONLY, 0);
	}

	if(IS_ERR(cfg)) {
		pr_info("[IT7256] : %s open /data/it7280.cfg failed = %ld\n", __func__, PTR_ERR(cfg));
		pr_info("[IT7256] : open /system/etc/firmware/it7280.cfg failed\n");
		if(!IS_ERR(fw)) {
			filp_close(fw, NULL);
		}

		kfree(config_buf);
		kfree(fw_buf);

		set_fs(fs);

		return 1;
	}
	else {
		inode=cfg->f_dentry->d_inode;
		cfg_size = inode->i_size;
		cfg->f_op->read(cfg, config_buf, cfg_size, &cfg->f_pos);
		pr_info("[IT7256] : %s cfg_ver : %d, %d, %d, %d\n",__func__,config_buf[cfg_size-8], config_buf[cfg_size-7], config_buf[cfg_size-6], config_buf[cfg_size-5]);
		pr_info("[IT7256] : %s --------------------- cfg_size = %ld\n", __func__,cfg_size);
	}

	if(!IS_ERR(cfg))
		filp_close(cfg, NULL);

	if(!IS_ERR(fw))
		filp_close(fw, NULL);

	set_fs(fs);

	disable_irq(gl_ts->client->irq);
	if (fnFirmwareDownload(fw_size, fw_buf, cfg_size, config_buf) == false) {
		/*upgrade firmware fail*/
		enable_irq(gl_ts->client->irq);
		kfree(config_buf);
		kfree(fw_buf);
		return 1;
	}
	else {
		/*upgrade firmware success*/
		enable_irq(gl_ts->client->irq);
		kfree(config_buf);
		kfree(fw_buf);

		/*Get configuration information*/
		config_id[0] = 0x01;
		config_id[1] = 0x06;
		i2cWriteToIt7280(gl_ts->client, 0x20, config_id, 2);

		do {
			i2cReadFromIt7280(gl_ts->client, 0x80, &ucQuery, 1);
		} while (ucQuery & 0x01);

		memset(&config_id, 0, sizeof(config_id));
		i2cReadFromIt7280(gl_ts->client, 0xA0, config_id, 7);
		pr_info("[IT7256] : %s() chip-version : %d.%d.%d%d\n", __func__,config_id[1], config_id[2],config_id[3], config_id[4]);

		return 0;
	}
}

static ssize_t IT7280_upgrade_show_temp(char *buf)
{
	pr_info("[IT7256] : %s \n", __func__);

	return sprintf(buf, "%d", fw_upgrade_success);
}

static ssize_t IT7280_upgrade_store_temp(struct device *dev)
{
	pr_info("[IT7256] : %s \n", __func__);

	if(!Upgrade_FW_CFG(dev)) {
		pr_info("[IT7256] : IT7280_upgrade_OK\n\n");
		fw_upgrade_success = 1;
		return 0;
	}
	else {
		pr_info("[IT7256] : IT7280_upgrade_failed or Not need to upgrade\n");
		fw_upgrade_success = -1;
		return -1;
	}
}

static ssize_t IT7280_upgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("[IT7256] : %s():\n", __func__);

	return IT7280_upgrade_show_temp(buf);
}

static ssize_t IT7280_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("[IT7256] : %s():\n", __func__);

	mdelay(10);
	IT7280_upgrade_store_temp(dev);
	return count;
}

static ssize_t IT7280_appfwupgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("[IT7256] : %s():\n", __func__);

	return sprintf(buf, "%d\n", fw_upgrade_success);
}

static ssize_t IT7280_appfwupgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("[IT7256] : %s():\n", __func__);

	mdelay(10);
	if(!AppUpgrade_FW_CFG(dev)) {
		pr_info("[IT7256] : IT7280_appupgrade_OK\n\n");
		fw_upgrade_success = 1;
	}
	else {
		pr_info("[IT7256] : IT7280_appupgrade_failed or Not need to upgrade\n");
		fw_upgrade_success = -1;
	}
	return count;
}

static ssize_t IT7280_selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	mm_segment_t fs;
	struct file *TPtest_result;

	pr_info("[IT7256] : %s():\n", __func__);

	fs = get_fs();
	set_fs(get_ds());
	TPtest_result = filp_open(TP_TEST_RESULT_FILE, O_TRUNC | O_RDWR, (S_IRUGO | S_IWUSR) );
	if(IS_ERR(TPtest_result)) {
		pr_info("[IT7256] : %s open /data/TPtest_result.bin failed = %ld\n", __func__, PTR_ERR(TPtest_result));
		set_fs(fs);
		return -1;
	}
	filp_close(TPtest_result, NULL);
	set_fs(fs);

	return snprintf(buf, PAGE_SIZE, "%d\n",selftest_success);
}

static ssize_t Rawdata_Sorting(void)
{
	ssize_t ret = 0;
	int i = 0, j = 0, k = 0;
	u8 temp;
	int coordi_x = 0, coordi_y = 0;
	struct file *TPtest_result, *G_sample_file;
	mm_segment_t fs;
	const char *g_sample_name[2] = {GOLDEN_SAMPLE_FILE, GOLDEN_SAMPLE2_FILE};
	int offset = -1;
	char *buf;
	selftest_success = -1;

	pr_info("[IT7256] : %s():\n", __func__);

	/*there are tx*rx channels, each channel need upper bound and lower bound with 2 bytes*/
	if(gl_ts->gold_sample_size != (tx*rx*4)) {
		pr_err("[IT7256]: the golden sample size is not matched to the rawdata\n");
		return -1;
	}

	/*example of a channel raw data: ...(C,13,21,9000,7000,8000,P);..., each symbol, alphabet, digit cost 1 byte*/
	buf = kzalloc((30*tx*rx+50), GFP_KERNEL);
	if(buf == NULL) {
		pr_err("[IT7256] : %s() buffer is NULL\n", __func__);
		return -1;
	}

	if(!Source_flag) {
		pr_info("[IT7256] : %s Main source FW & CFG path\n", __func__);
		offset = 0;
	}
	else if(Source_flag == 1) {
		pr_info("[IT7256] : %s Second source FW & CFG path\n", __func__);
		offset = 1;
	}
	offset = 0;
	pr_info("[IT7256] : %s g_sample_name[%d] = %s\n", __func__, offset, g_sample_name[offset]);

	fs = get_fs();
	set_fs(get_ds());

	G_sample_file = filp_open(g_sample_name[offset], O_RDONLY, 0 );
	if(IS_ERR(G_sample_file))
		pr_info("[IT7256]: Can not open the golden sample file, use the golden sample in device tree\n");
	else
		pr_info("[IT7256]: Use the golden sample file to selftest\n");

	TPtest_result = filp_open(TP_TEST_RESULT_FILE, O_CREAT | O_RDWR, (S_IRUGO | S_IWUSR) );
	if(IS_ERR(TPtest_result)) {
		pr_err("[IT7256] : %s open /data/TPtest_result.bin failed = %ld\n", __func__, PTR_ERR(TPtest_result));
		set_fs(fs);
		kfree(buf);
		if(!IS_ERR(G_sample_file))
			filp_close(G_sample_file, NULL);
		return -1;
	}


	for( i = 1 ; i < tx*rx*2 ; i+=2 ){
		temp = wTemp[i-1];
		wTemp[i-1] = wTemp[i];
		wTemp[i] = temp;
	}

	ret += snprintf(&(buf[ret]), PAGE_SIZE, "Data=%d;", tx*rx);

	if(IS_ERR(G_sample_file)) {
		for(j = 0; j < (tx*rx*2*2); j+=2)
		{
			if( j == 0) {
				coordi_x = 0;
				coordi_y = 0;
				ret += snprintf(&(buf[ret]), PAGE_SIZE, "(C,%d,%d,%02x,%02x,%02x",coordi_x ,coordi_y ,((gl_ts->gold_sample[j]<<8)+gl_ts->gold_sample[j+1])+thres ,((gl_ts->gold_sample[j+(2*rx)]<<8)+gl_ts->gold_sample[j+(2*rx)+1])-thres ,((wTemp[k]<<8)+wTemp[k+1]));
			}
			else {
				if(!(j % (rx*2))) {
					coordi_x +=1;
					coordi_y = 0;
					j = (rx*2*2)*coordi_x;
					if( j >= (tx*rx*2*2))
						break;
				}
				else
					coordi_y++;

				ret += snprintf(&(buf[ret]), PAGE_SIZE, ";(C,%d,%d,%02x,%02x,%02x",coordi_x ,coordi_y ,((gl_ts->gold_sample[j]<<8)+gl_ts->gold_sample[j+1])+thres ,((gl_ts->gold_sample[j+(2*rx)]<<8)+gl_ts->gold_sample[j+(2*rx)+1])-thres ,((wTemp[k]<<8)+wTemp[k+1]));
			}
			if( ((wTemp[k]<<8)+wTemp[k+1]) > ((gl_ts->gold_sample[j]<<8)+gl_ts->gold_sample[j+1]+thres) || ((wTemp[k]<<8)+wTemp[k+1]) < ((gl_ts->gold_sample[j+(2*rx)]<<8)+gl_ts->gold_sample[j+(2*rx)+1]-thres) ) {
				selftest_success = 1;
				ret += snprintf(&(buf[ret]),PAGE_SIZE, ",%c)", 'F');
				pr_err("[IT7256]: tx=%d , rx=%d, upper bound=%02x, lower bound=%02x, rawdata=%02x\n",coordi_x ,coordi_y ,((gl_ts->gold_sample[j]<<8)+gl_ts->gold_sample[j+1])+thres ,((gl_ts->gold_sample[j+(2*rx)]<<8)+gl_ts->gold_sample[j+(2*rx)+1])-thres ,((wTemp[k]<<8)+wTemp[k+1]));
			}
			else
				ret += snprintf(&(buf[ret]),PAGE_SIZE,",%c)", 'P');

			k+=2;
		}
	}
	else {
		G_sample_file->f_op->read(G_sample_file, gTemp, GTEMP_SIZE, &G_sample_file->f_pos);

		for(j = 0; j < (tx*rx*2*2); j+=2)
		{
			if( j == 0) {
				coordi_x = 0;
				coordi_y = 0;
				ret += snprintf(&(buf[ret]), PAGE_SIZE, "(C,%d,%d,%02x,%02x,%02x",coordi_x ,coordi_y ,((gTemp[j]<<8)+gTemp[j+1])+thres ,((gTemp[j+(2*rx)]<<8)+gTemp[j+(2*rx)+1])-thres ,((wTemp[k]<<8)+wTemp[k+1]));
			}
			else {
				if(!(j % (rx*2))) {
					coordi_x +=1;
					coordi_y = 0;
					j = (rx*2*2)*coordi_x;
					if( j >= (tx*rx*2*2))
						break;
				}
				else
					coordi_y++;

				ret += snprintf(&(buf[ret]), PAGE_SIZE, ";(C,%d,%d,%02x,%02x,%02x",coordi_x ,coordi_y ,((gTemp[j]<<8)+gTemp[j+1])+thres ,((gTemp[j+(2*rx)]<<8)+gTemp[j+(2*rx)+1])-thres ,((wTemp[k]<<8)+wTemp[k+1]));
			}
			if( ((wTemp[k]<<8)+wTemp[k+1]) > ((gTemp[j]<<8)+gTemp[j+1]+thres) || ((wTemp[k]<<8)+wTemp[k+1]) < ((gTemp[j+(2*rx)]<<8)+gTemp[j+(2*rx)+1]-thres) ) {
				selftest_success = 1;
				ret += snprintf(&(buf[ret]),PAGE_SIZE, ",%c)", 'F');
			}
			else
				ret += snprintf(&(buf[ret]),PAGE_SIZE,",%c)", 'P');
			k+=2;
		}

	}

	ret += snprintf(&(buf[ret]),PAGE_SIZE,"\n");

	TPtest_result->f_op->write(TPtest_result, buf, ret, &TPtest_result->f_pos);

	if(selftest_success != 1)
		selftest_success = 0;

	filp_close(TPtest_result, NULL);

	set_fs(fs);
	kfree(buf);

	if(!IS_ERR(G_sample_file))
		filp_close(G_sample_file, NULL);

	return ret;
}

static ssize_t IT7280_selftest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, txrx = 0, mode = -1;
	unsigned int offset = 0;
	int cnt = 0;
	u8 get_2DRes[2] = { 0x01, 0x02 };
						//    Read|Count|Disable|Byte|			 Offset		| Interval
	u8 pucBuffer_CDC[10] = { 0x0E, 0x01, 0x00, 0x02, 0x00, 0x80, 0x04, 0x00, 0x04, 0x00 };					// 0x48000
	u8 set_test[7] = { 0x1A, 0x05, 0x02, 0x08, 0x00, 0x00, 0x00 };// Tx > Rx single

	sscanf(buf, "%d", &mode);

	if( mode == 1)
		set_test[2] = 0x03;

	disable_irq(gl_ts->client->irq);
	mdelay(100);

	/*the size of wTemp and gTemp can be reduced, need to discuss with vendor*/
	wTemp = kzalloc(WTEMP_SIZE*sizeof(u8), GFP_KERNEL);
	if(wTemp == NULL) {
		pr_err("[IT7256]: ==wTemp kzalloc failed\n");
		enable_irq(gl_ts->client->irq);
		return -1;
	}

	gTemp = kzalloc(GTEMP_SIZE*sizeof(u8), GFP_KERNEL);
	if(gTemp == NULL) {
		pr_err("[IT7256]: ==gTemp kzalloc failed\n");
		kfree(wTemp);
		enable_irq(gl_ts->client->irq);
		return -1;
	}

	waitCommandDone();
	i2cWriteToIt7280(gl_ts->client, 0x20, get_2DRes, 2);
	waitCommandDone();
	i2cReadFromIt7280(gl_ts->client, 0xA0, wTemp, 12);
	tx = wTemp[6];
	rx = wTemp[7];
	pr_info("[IT7256]: ==tx=%d , rx=%d\n",tx,rx);
	//sscanf(buf, "%u,%u,%u", &tx, &rx, &thres);
	//sscanf(buf, "%u", &thres);

	if(!Source_flag)
		thres = /*1280*/4096;
	else if(Source_flag == 1)
		thres = 8192;

	thres = 0x0900;
	pr_info("[IT7256] : %s tx=%d rx=%d thres=%d\n",__func__,tx,rx,thres);

	waitCommandDone();
	i2cWriteToIt7280(gl_ts->client, 0x20, set_test, 7);

	mdelay(100);

	cnt = 0;
	txrx = tx*rx;

	do{
		offset = (pucBuffer_CDC[4] + (pucBuffer_CDC[5]<<8)) + (cnt*4);
		pucBuffer_CDC[4] = (u8)(offset & 0x00FF);
		pucBuffer_CDC[5] = (u8)((offset & 0xFF00)>>8);

		if((txrx) > 70){
			cnt = 70;
			txrx = txrx - 70;
		}else{
			cnt = txrx;
			txrx = 0;
		}
		pucBuffer_CDC[1] = (u8)(cnt & 0x00FF);
		pucBuffer_CDC[2] = (u8)((cnt & 0x0F00)>>8);

		waitCommandDone();
		i2cWriteToIt7280(gl_ts->client, 0x20, pucBuffer_CDC, 10);
		waitCommandDone();
		i2cReadFromIt7280(gl_ts->client, 0xA0, wTemp+(i*70*2), cnt*2);
		i++;
	} while(txrx > 0);

	mdelay(200);
	fnFirmwareReinitialize();
	mdelay(100);

	Rawdata_Sorting();

	kfree(wTemp);
	kfree(gTemp);

	enable_irq(gl_ts->client->irq);

	return count;
}

static ssize_t IT7280_tp_goldsample_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0, j = 0, k = 0, txrx = 0, mode = -1;
	unsigned int offset = 0;
	int cnt = 0;
	struct file *g_sample;
	u8 temp;
	int coordi_x = 0, coordi_y = 0;
	mm_segment_t fs;

	u8 get_2DRes[2] = { 0x01, 0x02 };
						//    Read|Count|Disable|Byte|			 Offset		| Interval
	u8 pucBuffer_CDC[10] = { 0x0E, 0x01, 0x00, 0x02, 0x00, 0x80, 0x04, 0x00, 0x04, 0x00 };					// 0x48000
	u8 set_test[7] = { 0x1A, 0x05, 0x02, 0x08, 0x00, 0x00, 0x00 };// Tx > Rx single

	sscanf(buf, "%d", &mode);

	if( mode == 1)
		set_test[2] = 0x03;

	wTemp = kzalloc(WTEMP_SIZE*sizeof(u8), GFP_KERNEL);
	if(wTemp == NULL) {
		pr_err("[IT7256]: ==wTemp kzalloc failed\n");
		return -1;
	}

	gTemp = kzalloc(GTEMP_SIZE*sizeof(u8), GFP_KERNEL);
	if(gTemp == NULL) {
		pr_err("[IT7256]: ==gTemp kzalloc failed\n");
		kfree(wTemp);
		return -1;
	}

	pr_info("[IT7256] : %s():\n", __func__);

	disable_irq(gl_ts->client->irq);
	mdelay(100);

	waitCommandDone();
	i2cWriteToIt7280(gl_ts->client, 0x20, get_2DRes, 2);
	waitCommandDone();
	i2cReadFromIt7280(gl_ts->client, 0xA0, wTemp, 12);
	tx = wTemp[6];
	rx = wTemp[7];

	pr_err("[IT7256] : %s() golden sample tx=%d rx=%d\n",__func__,tx,rx);

	waitCommandDone();
	i2cWriteToIt7280(gl_ts->client, 0x20, set_test, 7);

	mdelay(100);

	cnt = 0;
	txrx = tx*rx;
	do{
		offset = (pucBuffer_CDC[4] + (pucBuffer_CDC[5]<<8)) + (cnt*4);
		pucBuffer_CDC[4] = (u8)(offset & 0x00FF);
		pucBuffer_CDC[5] = (u8)((offset & 0xFF00)>>8);

		if((txrx) > 70){
			cnt = 70;
			txrx = txrx - 70;
		}else{
			cnt = txrx;
			txrx = 0;
		}

		pucBuffer_CDC[1] = (u8)(cnt & 0x00FF);
		pucBuffer_CDC[2] = (u8)((cnt & 0x0F00)>>8);

		waitCommandDone();
		i2cWriteToIt7280(gl_ts->client, 0x20, pucBuffer_CDC, 10);
		waitCommandDone();
		i2cReadFromIt7280(gl_ts->client, 0xA0, wTemp+(i*70*2), cnt*2);
		i++;
	}while(txrx > 0);

	mdelay(100);
	fnFirmwareReinitialize();
	mdelay(100);

	for( i = 1 ; i < tx*rx*2 ; i+=2 ){
		temp = wTemp[i-1];
		wTemp[i-1] = wTemp[i];
		wTemp[i] = temp;
	}

	fs = get_fs();
	set_fs(get_ds());

	g_sample = filp_open(GOLDEN_SAMPLE_FILE, O_RDWR, (S_IRUGO | S_IWUGO) );
	if(IS_ERR(g_sample)) {
		pr_err("[IT7256] : %s open /data/g_sample.bin failed = %ld\n", __func__, PTR_ERR(g_sample));
		kfree(wTemp);
		kfree(gTemp);
		set_fs(fs);
		enable_irq(gl_ts->client->irq);
		return -1;
	}

	memset(gTemp, 0, sizeof(gTemp));
	g_sample->f_op->read(g_sample, gTemp, GTEMP_SIZE, &g_sample->f_pos);

	for(j = 0; j < (tx*rx*2*2); j+=2)
	{
		if( j == 0)
		{
			coordi_x = 0;
			coordi_y = 0;
		}
		else
		{
			if(!(j % (rx*2)))
			{
				coordi_x +=1;
				coordi_y = 0;
				j = (rx*2*2)*coordi_x;
				if( j >= (tx*rx*2*2))
				{
					pr_info("[IT7256] : %s [IT7280] Out of range\n",__func__);
					break;
				}
			}
			else
				coordi_y++;
		}
		if( ((wTemp[k]<<8)+wTemp[k+1]) > ((gTemp[j]<<8)+gTemp[j+1]) )
		{
			gTemp[j] = wTemp[k];
			gTemp[j+1] = wTemp[k+1];
		}
		if( ((wTemp[k]<<8)+wTemp[k+1]) < ((gTemp[j+(2*rx)]<<8)+gTemp[j+(2*rx)+1]) )
		{
			gTemp[j+(2*rx)] = wTemp[k];
			gTemp[j+(2*rx)+1] = wTemp[k+1];
		}

		k+=2;
	}

	g_sample->f_op->llseek(g_sample, 0, 0);
	g_sample->f_op->write(g_sample, gTemp, tx*rx*2*2, &g_sample->f_pos);

	set_fs(fs);
	filp_close(g_sample, NULL);

	kfree(wTemp);
	kfree(gTemp);

	enable_irq(gl_ts->client->irq);

	return count;
}

static ssize_t firmware_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", it7256_fw_name);
}

static ssize_t firmware_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if(count > 256) {
		pr_err("%s : input string is too long, count = %d\n", __func__, count);
		return -EINVAL;
	}

	sprintf(it7256_fw_name, "%s", buf);
	it7256_fw_name[count-1] = '\0';

	return count;
}

static ssize_t config_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", it7256_cfg_name);
}

static ssize_t config_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if(count > 256) {
		pr_err("%s : input string is too long, count = %d\n", __func__, count);
		return -EINVAL;
	}

	sprintf(it7256_cfg_name, "%s", buf);
	it7256_cfg_name[count-1] = '\0';

	return count;
}

static ssize_t suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", suspend_status);
}

static ssize_t suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -1;

	if(input == 0) {
		if(suspend_status != 0) {
			char ret = 0;
			struct i2c_client *client = gl_ts->client;
			u8 cmdbuf[] = {0xF8, 0x00};

			if ((i2cWriteToIt7280(client, 0x20, cmdbuf, 2)) > 0)
				ret = count;
			else
				return -1;

			/*reset touch panel*/
			mdelay(10);
			gpio_direction_output(gl_ts->reset_gpio, 0);
			mdelay(20);
			gpio_direction_output(gl_ts->reset_gpio, 1);
			mdelay(20);

			enable_irq(client->irq);

			suspend_status = 0;
			return ret;
		}
	}
	else if(input == 1) {
		if(suspend_status != 1) {
			char ret = 0;
			struct i2c_client *client = gl_ts->client;
			u8 cmdbuf[] = { 0x04, 0x00, 0x02 };

			disable_irq(client->irq);

			if ((i2cWriteToIt7280(client, 0x20, cmdbuf, 3)) > 0)
				ret = count;
			else
				ret = -1;

			suspend_status = 1;
			return ret;
		}
	}

	return -1;
}

static DEVICE_ATTR(upgrade, 0644, IT7280_upgrade_show, IT7280_upgrade_store);
static DEVICE_ATTR(test, 0644, IT7280_selftest_show, IT7280_selftest_store);
static DEVICE_ATTR(goldsample, 0644, NULL, IT7280_tp_goldsample_store);
static DEVICE_ATTR(init_source, 0644, IT7280_init_source_show, NULL);
static DEVICE_ATTR(appfwupgrade, 0644, IT7280_appfwupgrade_show, IT7280_appfwupgrade_store);
static DEVICE_ATTR(firmware_name, 0644, firmware_name_show, firmware_name_store);
static DEVICE_ATTR(config_name, 0644, config_name_show, config_name_store);
static DEVICE_ATTR(suspend, 0644, suspend_show, suspend_store);

static struct attribute *it7280_attributes[] = {
	&dev_attr_upgrade.attr,
	&dev_attr_test.attr,
	&dev_attr_goldsample.attr,
	&dev_attr_init_source.attr,
	&dev_attr_appfwupgrade.attr,
	&dev_attr_firmware_name.attr,
	&dev_attr_config_name.attr,
	&dev_attr_suspend.attr,
	NULL
};

static const struct attribute_group it7280_attr_group = {
	.attrs = it7280_attributes,
};

static void Read_Point(struct IT7280_ts_data *ts)
{
	unsigned char ucQuery[2] = {0,0};
	unsigned char pucPoint[52];
	int ret = 0;
	int xraw = 0, yraw = 0, xtmp = 0, ytmp = 0,xscreen = 0,yscreen = 0,xres = 0,yres = 0, pressure = 0;
	int fingerID =0;
	int j = 0;
	unsigned long end_time;
	unsigned int msec;
	int key_idx = 0;

	xscreen = ts->panel_maxx;
	yscreen = ts->panel_maxy;
	xres    = ts->x_max;
	yres    = ts->y_max;

	ret=i2cReadFromIt7280(ts->client, 0x80, ucQuery, 2);

	if ((ret > 0) && ((ucQuery[0] & 0x80)==0x80))
	{
		ret = i2cAdvancedReadFromIt7280(ts->client, 0xE0, pucPoint, ucQuery[1]);
		if (ret > 0)
		{
			/*button*/
			if(pucPoint[0] == 0x41)
			{
				key_idx = pucPoint[2];

				if(pucPoint[3] == 1)
				{
					input_report_key(ts->input_dev,TouchKeyCode[key_idx],1);
				}
				else
				{
					input_report_key(ts->input_dev,TouchKeyCode[key_idx],0);
				}

				return;
			}
			/*point*/
			else if(((pucPoint[0] & 0xF0) == 0x00))
			{
				int touch_number;
				int report_type;
				int i;

				report_type = (pucPoint[1] & 0X80) >> 7;
				touch_number = ((pucPoint[1] & 0x7F)-2)/4;

				if(touch_number > 0) {
					for(i = 0; i < touch_number; i++)
					{
						/*get point i's X-coordinate & Y-coordinate*/
						if(report_type == 0) {
							xraw = ((pucPoint[i * 4 + 4] & 0x0F) << 8) + pucPoint[i * 4 + 3];
							yraw = ((pucPoint[i * 4 + 4] & 0xF0) << 4) + pucPoint[i * 4 + 5];
							fingerID = ((pucPoint[i * 4 + 2] & 0xF8)>>3); //0x11111000
							pressure = pucPoint[i * 4 + 2] & 0x07;
						}
						else if(report_type == 1) {
							xraw = ((pucPoint[i * 5 + 5] & 0x0F) << 8) + pucPoint[i * 5 + 4];
							yraw = ((pucPoint[i * 5 + 5] & 0xF0) << 4) + pucPoint[i * 5 + 6];
							fingerID = ((pucPoint[i * 5 + 2] & 0xF8)>>3); //0x11111000
							pressure = pucPoint[i * 5 + 3] & 0x07;
						}

						xtmp = xraw * xscreen / xres;
						ytmp = yraw * yscreen / yres;

						if(xtmp < 0)
							xtmp = 0;
						else if(xtmp > xscreen)
							xtmp = xscreen;

						if(ytmp < 0)
							ytmp = 0;
						else if(ytmp > yscreen)
							ytmp = yscreen;

						pr_debug("[IT7256]: ==Read_Point read --fingerID[%x]--point[%d][%d]=\n", fingerID,xtmp,ytmp);

						input_mt_slot(ts->input_dev,fingerID);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, xtmp);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ytmp);
						input_report_key(ts->input_dev, BTN_TOUCH, 1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,  i + 1);
						input_report_abs(ts->input_dev, ABS_PRESSURE, 1 );

						if(touch_number >= max_slot)
							max_slot = touch_number;

						fingerIDUseNow[fingerID] = 1;
					}

					for(j = 0; j < 10; j++)
					{
						if((fingerIDUseBefore[j] == 1) && (fingerIDUseNow[j] == 0))
						{
							input_mt_slot(ts->input_dev,j);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
						}
						fingerIDUseBefore[j] = fingerIDUseNow[j];
						fingerIDUseNow[j] = 0;
					}
					/*commit all point*/
					input_sync(ts->input_dev);
				}
				else {
					/*total release*/
					for(j = 0; j < 10; j++)
					{
						if((fingerIDUseBefore[j] == 1) && (fingerIDUseNow[j] == 0))
						{
							input_mt_slot(ts->input_dev,j);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
							input_report_key(ts->input_dev, BTN_TOUCH, 0);
						}
						fingerIDUseBefore[j] =0;
						fingerIDUseNow[j] = 0;
					}
					pressure = pucPoint[i * 4 + 2] & 0x07;
					input_sync(ts->input_dev);

					end_time = jiffies;
					msec = jiffies_to_msecs(end_time);
					max_slot = 0;
				}
			}
		}
	}
}

static irqreturn_t IT7280_ts_work_func(int irq, void *dev_id)
{
	disable_irq_nosync(gl_ts->client->irq);
	Read_Point(gl_ts);
	enable_irq(gl_ts->client->irq);
	return IRQ_HANDLED;
}

/*this chip does not need to calibration*/
#if 0
void sendCalibrationCmd(void)
{
	int ret = 0;
	struct IT7280_ts_data *ts = gl_ts;
	unsigned char data[] = { 0x13, 0x00, 0x00, 0x00, 0x00 };
	unsigned char resp[2];

	pr_info("[IT7256] : %s \n", __func__);
	ret = i2cWriteToIt7280(ts->client, 0x20, data, 5);
	pr_info("[IT7256] : IT7280 sent calibration command [%d]!!!\n", ret);

	//TODO:
	//MUST sleep 5 seconds here!
	mdelay(5000);
	//Read out response to clear interrupt.
	ret = i2cReadFromIt7280(ts->client, 0xA0, resp, 2);

	pr_info("[IT7256] : IT7280 sent calibration end %d\n", ret);
}

EXPORT_SYMBOL( sendCalibrationCmd);
#endif

static int IT7280_ts_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int ret;
	unsigned char buffer[10];

	memset(&buffer[0], 0, sizeof(buffer));
	ret = i2cReadFromIt7280(client, 0xA0, buffer, 8);

	if(ret < 0)
		return -ENODEV;

	strlcpy((info->type),IT7280_I2C_NAME, I2C_NAME_SIZE);

	return 0;
}

struct ite7280_data {
	rwlock_t lock;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

/*this function is for firmware upgrade, but we do not use this command to upgrade*/
static long ite7280_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
	int retval = 0;
	int i;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168 data;
	unsigned char datalen;
	bool bIRQ = true;

	memset(&data, 0, sizeof(struct ioctl_cmd168));

	pr_info("[IT7256] : %s \n", __func__);

	switch (cmd) {
	case IOCTL_SET:
		disable_irq_nosync(gl_ts->client->irq);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}

		pr_info("[IT7256] : %s data.bufferIndex=%d data.length=%d\n", __func__, data.bufferIndex, data.length);
		buffer[0] = (unsigned char) data.bufferIndex;
		for (i = 1; i < data.length + 1; i++) {
			buffer[i] = (unsigned char) data.buffer[i - 1];
			pr_info("[IT7256] :  %s buffer[%d]=%.2X\n", __func__, i, buffer[i]);
		}

		pr_info("[IT7256] : =================================================\n");
		pr_info("[IT7256] : name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);

		datalen = (unsigned char) (data.length + 1);

		if(buffer[1]==0x0c)
			bIRQ=false;
		retval = i2cAdvancedWriteToIt7280(gl_ts->client,
				(unsigned char) data.bufferIndex, &(buffer[1]), datalen - 1);
		retval = 0;
		break;

	case IOCTL_GET:
		pr_info("[IT7256] : =IOCTL_GET=\n");
		disable_irq_nosync(gl_ts->client->irq);

		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			goto done;
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}

		pr_info("[IT7256] : =================================================\n");
		pr_info("[IT7256] : name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
		pr_info("[IT7256] : %s data.bufferIndex=%d data.length=%d\n", __func__, data.bufferIndex, data.length);

		retval = i2cAdvancedReadFromIt7280(gl_ts->client,
				(unsigned char) data.bufferIndex, (unsigned char*) buffer,
				(unsigned char) data.length);

		retval = 0;
		for (i = 0; i < data.length; i++) {
			data.buffer[i] = (unsigned short) buffer[i];
		}

		if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			goto done;
		}
		break;

	default:
		retval = -ENOTTY;
		break;
	}

done:
	if(bIRQ)
		enable_irq(gl_ts->client->irq);
	else {
		mdelay(500);
		enable_irq(gl_ts->client->irq);
		bIRQ=true;
	}

	return (retval);
}

static int ite7280_open(struct inode *inode, struct file *filp)
{
	struct ite7280_data *dev;

	dev = kzalloc(sizeof(struct ite7280_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	rwlock_init(&dev->lock);
	memset(dev->buffer, 0xFF, MAX_BUFFER_SIZE);

	filp->private_data = dev;

	return 0;
}

static int ite7280_close(struct inode *inode, struct file *filp)
{
	struct ite7280_data *dev = filp->private_data;

	if (dev) {
		kfree(dev);
	}

	return 0;
}

struct file_operations ite7280_fops = {
	.owner = THIS_MODULE,
	.open = ite7280_open,
	.release = ite7280_close,
	.unlocked_ioctl = ite7280_ioctl,
};

static int get_config_ver(void)
{
	unsigned char ucQuery = 0xFF;
	int count = 0, retry_count;

	retry_count = (gl_ts->hw_dvt)? 10 : 100;

	do {
		i2cReadFromIt7280(gl_ts->client, 0x80, &ucQuery, 1);
		count++;
	} while ((ucQuery & 0x01) && (count < retry_count));

	pr_info("[IT7256]: count = %d\n",count);

	config_id[0] = 0x01;
	config_id[1] = 0x06;
	i2cWriteToIt7280(gl_ts->client, 0x20, config_id, 2);

	count = 0;
	do {
		i2cReadFromIt7280(gl_ts->client, 0x80, &ucQuery, 1);
		count++;
	} while ((ucQuery & 0x01) && (count < retry_count));

	if(count == retry_count) {
		pr_err("[IT7256]: ==Can not read config version, count = %d\n", count);
		return count;
	}

	pr_info("[IT7256]: count = %d\n", count);
	memset(&config_id, 0, sizeof(config_id));
	i2cReadFromIt7280(gl_ts->client, 0xA0, config_id, 7);
	pr_info("[IT7256] : %s() chip-version : %d.%d.%d%d\n", __func__,config_id[1], config_id[2],config_id[3], config_id[4]);

	return 0;
}

static int it7280_get_dt_coords(struct device *dev, char *name,
				struct IT7280_ts_data *pdata)
{
	u32 coords[4];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != 4) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "ite,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	}
	else if (!strcmp(name, "ite,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}
	else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int it7280_parse_dt(struct device *dev,
			struct IT7280_ts_data *pdata)
{
	int rc, blen;
	struct device_node *np = dev->of_node;

	/* get panel name info */
	pdata->name = "ite";
	rc = of_property_read_string(np, "ite,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	/* panel resolution */
	rc = it7280_get_dt_coords(dev, "ite,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = it7280_get_dt_coords(dev, "ite,display-coords", pdata);
	if (rc)
		return rc;

	/* vdd, reset, irq gpio info */
	pdata->reg_enable = of_property_read_bool(np, "ite,regulator-enable");
	if (!(pdata->reg_enable))
	{
		pdata->enable_gpio = of_get_named_gpio_flags(np, "vdd_enable",
					0, &pdata->enable_gpio_flags);
		if (pdata->enable_gpio < 0)
			return pdata->enable_gpio;
	}

	pdata->reset_gpio = of_get_named_gpio_flags(np, "ite,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "ite,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->gold_sample = of_get_property(np, "ite,golden-sample", &blen);
	if (!pdata->gold_sample) {
		pr_err("[IT7256]: ==Can not read the golden sample array\n");
	}
	pdata->gold_sample_size = blen;

	pdata->hw_dvt = of_property_read_bool(np, "ite,hw-dvt");

	return 0;
}

static int it7280_power_init(struct IT7280_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	if(data->reg_enable) {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if(regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, ITE_VTG_MIN_UV, ITE_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}
	else {
		if (gpio_is_valid(data->enable_gpio)) {
			rc = gpio_request(data->enable_gpio, "it7280_enable_gpio");
			if (rc) {
				pr_err("enable gpio request failed\n");
				goto reg_vdd_set_vtg;
			}
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	rc = regulator_set_optimum_mode(data->vcc_i2c, 100000);
	if (rc) {
		pr_info("[IT7256]: set_optimum_mode vcc_i2c failed, rc = %d\n", rc);
		return -EINVAL;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, ITE_I2C_VTG_MIN_UV, ITE_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if ((regulator_count_voltages(data->vdd) > 0))
		regulator_set_voltage(data->vdd, 0, ITE_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
pwr_deinit:
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, ITE_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int it7280_power_on(struct IT7280_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	if(data->reg_enable) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	}
	else {
		if (gpio_is_valid(data->enable_gpio))
			gpio_set_value(data->enable_gpio, 1);
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	if(data->reg_enable) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	else {
		if (gpio_is_valid(data->enable_gpio))
			gpio_set_value(data->enable_gpio, 0);
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		regulator_enable(data->vdd);
	}

	return rc;
}

static int IT7280_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct IT7280_ts_data *ts;
	int ret = 0;
	int err;
	dev_t dev = MKDEV(ite7280_major, 0);
	int key,code;
	struct device *class_dev = NULL;
	Source_flag = -1;

	pr_info("[IT7256]: ==IT7256 enter probe\n");

	if (client->dev.of_node) {
		ts = devm_kzalloc(&client->dev,
			sizeof(struct IT7280_ts_data), GFP_KERNEL);
		if (!ts) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = it7280_parse_dt(&client->dev, ts);
		if (err) {
			dev_err(&client->dev, "Data parsing failed\n");
			goto err_parse;
		}
	}
	else
		ts = client->dev.platform_data;

	err = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (err) {
		pr_err("[IT7256]: IT7280 cdev can't get major number\n");
		goto err_alloc_dev;
	}
	ite7280_major = MAJOR(dev);

	/*allocate the character device*/
	cdev_init(&ite7280_cdev, &ite7280_fops);
	ite7280_cdev.owner = THIS_MODULE;
	ite7280_cdev.ops = &ite7280_fops;
	err = cdev_add(&ite7280_cdev, MKDEV(ite7280_major, ite7280_minor), 1);
	if(err) {
		goto err_add_dev;
	}

	/*register class*/
	ite7280_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ite7280_class)) {
		pr_err("[IT7256]: failed in creating class.\n");
		goto err_create_class;
	}

	ite7280_dev = MKDEV(ite7280_major, ite7280_minor);
	class_dev = device_create(ite7280_class, NULL, ite7280_dev, NULL, DEVICE_NAME);
	if(class_dev == NULL) {
		pr_err("[IT7256]: failed IT7280 in creating device.\n");
		goto err_create_dev;
	}

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("[IT7256]: failed to allocate input device\n");
		goto err_alloc_input_dev;
	}

	input_dev->name = IT7280_I2C_NAME;
	input_dev->id.bustype = BUS_I2C;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	/*this command is necessary for touch screen*/
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	for (key = 0; key < 4 ; key++) {
		code = TouchKeyCode[key];
		set_bit(code & KEY_MAX, input_dev->keybit);
	}
	input_mt_init_slots(input_dev, 10);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, ts->x_min, ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, ts->y_min, ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 2, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_X, ts->panel_minx, ts->panel_maxx, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, ts->panel_miny, ts->panel_maxy, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	err = input_register_device(input_dev);
	if(err) {
		pr_err("[IT7256]: device register error\n");
		goto dev_reg_err;
	}

	mutex_init(&ts->device_mode_mutex);
	ts->client = client;

	i2c_set_clientdata(client, ts);

	if (gpio_is_valid(ts->reset_gpio)) {
		err = gpio_request(ts->reset_gpio, "ITE_reset_gpio");
		if (err) {
			pr_err("[IT7256]: can not request reset gpio\n");
			goto free_reset_gpio;
		}
	}
	else
		pr_err("[IT7256]: ==reset gpio error\n");

	if (gpio_is_valid(ts->irq_gpio)) {
		err = gpio_request(ts->irq_gpio, "ITE_irq_gpio");
		if (err) {
			pr_err("[IT7256]: can not request reset gpio\n");
			goto free_irq_gpio;
		}

		err = gpio_direction_input(ts->irq_gpio);
		if (err) {
			pr_err("[IT7256]: ==irq_gpio direction_input set failed\n");
			goto free_irq_gpio;
		}
	}
	else
		pr_err("[IT7256]: ==irq gpio error\n");

	/* power init */
	err = it7280_power_init(ts, true);
	if (err) {
		dev_err(&client->dev, "power init failed, %d", __LINE__);
		goto err_pwr_init;
	}

	/* power on sequence */
	err = it7280_power_on(ts, true);
	if (err) {
		dev_err(&client->dev, "power on failed, %d", __LINE__);
		goto err_pwr_on;
	}

	gpio_direction_output(ts->reset_gpio, 0);
	mdelay(10);
	gpio_direction_output(ts->reset_gpio, 1);
	mdelay(20);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("[IT7256] : IT7280_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts->ts_workqueue = create_singlethread_workqueue(IT7280_I2C_NAME);
	if (!ts->ts_workqueue)
		goto err_create_queue;

	ts->input_dev = input_dev;
	gl_ts = ts;

	if (ts->irq_gpio) {
		pr_info("[IT7256]: ==irq = %d , gpio = %d\n",gpio_to_irq(ts->irq_gpio), ts->irq_gpio);
		ret = request_threaded_irq(client->irq, NULL,
				   IT7280_ts_work_func, ts->irqflags,
				   IT7280_I2C_NAME, ts);
		if (ret == 0)
			ts->use_irq = 1;
		else {
			dev_err(&client->dev, "[IT7256] : request_irq failed\n");
			goto err_request_irq;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = IT7280_ts_early_suspend;
	ts->early_suspend.resume = IT7280_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	ret = sysfs_create_group(&input_dev->dev.kobj, &it7280_attr_group);
	if (ret) {
		pr_err("[IT7256] : sysfs_create_group: Error to create calibration attribute\n");
		goto err_sysfs_create_group;
	}

	ret = sysfs_create_link(input_dev->dev.kobj.parent, &input_dev->dev.kobj, "touch");
	if(ret)
	{
		pr_err("[IT7256]: sysfs_create_link failed create [touch] link\n");
		goto err_sysfs_create_link;
	}

	/*to ensure the TP is connected to i2c*/
	mdelay(150);
	ret = get_config_ver();
	if(ret)
		goto err_get_ver;

	/*create /proc/AllHWList*/
	ret = fih_init();
	if(ret)
		goto err_create_allhwlist;

	pr_info("[IT7256]: ==IT7256 probe end\n");
	return 0;

err_create_allhwlist:
err_get_ver:
err_sysfs_create_link:
	sysfs_remove_group(&input_dev->dev.kobj, &it7280_attr_group);
err_sysfs_create_group:
	free_irq(client->irq, ts);
err_request_irq:
	destroy_workqueue(ts->ts_workqueue);
err_create_queue:
err_check_functionality_failed:
	it7280_power_on(ts, false);
err_pwr_on:
	it7280_power_init(ts, false);
err_pwr_init:
	gpio_free(ts->irq_gpio);
free_irq_gpio:
	gpio_free(ts->reset_gpio);
free_reset_gpio:
	input_unregister_device(input_dev);
dev_reg_err:
	input_free_device(input_dev);
err_alloc_input_dev:
	device_destroy(ite7280_class, ite7280_dev);
err_create_dev:
	class_destroy(ite7280_class);
err_create_class:
	cdev_del(&ite7280_cdev);
err_add_dev:
err_alloc_dev:
err_parse:
	devm_kfree(&client->dev, ts);
	return ret;
}

static int IT7280_ts_remove(struct i2c_client *client)
{
	sysfs_remove_group(&input_dev->dev.kobj, &it7280_attr_group);
	destroy_workqueue(gl_ts->ts_workqueue);
	it7280_power_on(gl_ts, false);
	it7280_power_init(gl_ts, false);
	gpio_free(gl_ts->irq_gpio);
	gpio_free(gl_ts->reset_gpio);
	input_unregister_device(input_dev);
	input_free_device(input_dev);
	device_destroy(ite7280_class, ite7280_dev);
	class_destroy(ite7280_class);
	cdev_del(&ite7280_cdev);
	devm_kfree(&client->dev, gl_ts);

	return 0;
}

#if 0
static int IT7280_ts_suspend(struct device *dev)
{
	char ret = 0;
	struct i2c_client *client = gl_ts->client;
	u8 cmdbuf[] = { 0x04, 0x00, 0x02 };

	if(1)
		return 0;

	disable_irq(client->irq);

	if ((i2cWriteToIt7280(client, 0x20, cmdbuf, 3)) > 0)
		ret = 0;
	else
		ret = -1;

	return ret;
}

static int IT7280_ts_resume(struct device *dev)
{
	char ret = 0;
	struct i2c_client *client = gl_ts->client;
	u8 cmdbuf[] = {0xF8, 0x00};

	if(1)
		return 0;

	if ((i2cWriteToIt7280(client, 0x20, cmdbuf, 2)) > 0)
		ret = 0;
	else
		return -1;

	/*reset touch panel*/
	mdelay(10);
	gpio_direction_output(gl_ts->reset_gpio, 0);
	mdelay(20);
	gpio_direction_output(gl_ts->reset_gpio, 1);
	mdelay(20);

	enable_irq(client->irq);

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void IT7280_ts_early_suspend(struct early_suspend *h)
{
	struct IT7280_ts_data *ts;

	ts = container_of(h, struct IT7280_ts_data, early_suspend);
	IT7280_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void IT7280_ts_late_resume(struct early_suspend *h)
{
	struct IT7280_ts_data *ts;
	int j = 0;

	ts = container_of(h, struct IT7280_ts_data, early_suspend);

	input_sync(ts->input_dev);

	IT7280_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id IT7280_ts_id[] = {
	{ IT7280_I2C_NAME, 0 },
	{ }
};

static struct of_device_id it7280_match_table[] = {
	{ .compatible = "ITE,7256",},
	{ },
};

#if 0
static const struct dev_pm_ops it7256_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = IT7280_ts_suspend,
	.resume = IT7280_ts_resume,
#endif
};
#endif

static struct i2c_driver IT7280_ts_driver = {
		.class = I2C_CLASS_HWMON,
		.probe = IT7280_ts_probe,
		.remove = IT7280_ts_remove,
		.id_table = IT7280_ts_id,
		.driver = {
			.name = IT7280_I2C_NAME,
			.of_match_table = it7280_match_table,
#ifdef CONFIG_PM
#if 0
			.pm = &it7256_pm_ops,
#endif
#endif
		},
		.detect	= IT7280_ts_detect,
};

static int __init IT7280_ts_init(void)
{
	return i2c_add_driver(&IT7280_ts_driver);
}

static void __exit IT7280_ts_exit(void)
{
	dev_t dev = MKDEV(ite7280_major, ite7280_minor);

	pr_info("[IT7256] : %s \n", __func__);

	/*unregister class*/
	device_destroy(ite7280_class, ite7280_dev);
	class_destroy(ite7280_class);

	/*unregister driver handle*/
	cdev_del(&ite7280_cdev);
	unregister_chrdev_region(dev, 1);

	i2c_del_driver(&IT7280_ts_driver);
}

module_init( IT7280_ts_init);
module_exit( IT7280_ts_exit);

MODULE_DESCRIPTION("IT7280 Touchscreen Driver");
MODULE_LICENSE("GPL");
