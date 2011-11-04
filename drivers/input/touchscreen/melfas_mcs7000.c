/* drivers/input/touchscreen/melfas_ts_i2c_tsi.c
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
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>

#include <linux/slab.h>

#include "mcs7000_download.h"
#include "melfas_ts.h"

#define INPUT_INFO_REG 0x10
#define IRQ_TOUCH_INT   MSM_GPIO_TO_INT(GPIO_TOUCH_INT)

#define NEW_FIRMWARE_VERSION 0x60 //0x58//0x57 // 0x07
#define FINGER_NUM	      2 //for multi touch
#undef CONFIG_CPU_FREQ
#undef CONFIG_MOUSE_OPTJOY

#ifdef CONFIG_CPU_FREQ
#include <plat/s3c64xx-dvfs.h>
#endif

static int debug_level = 5; 
#define debugprintk(level,x...)  if(debug_level>=level) printk(x)

extern int mcsdl_mcs7000_download_binary_data(void);//eunsuk test  [int hw_ver -> void]
#ifdef CONFIG_MOUSE_OPTJOY
extern int get_sending_oj_event();
#endif

#define NOISE_TEST     0  // MBsmhan

extern struct class *sec_class_1;  //HYH_20110314
//HYH_20110314  EXPORT_SYMBOL(mcs7000_sec_class);

struct input_info {
	int max_x;
	int max_y;
	int state;
	int x;
	int y;
	int z;
	int x2; 
	int y2;
	int z2;
	int width;
	int finger_id; 
};

struct mcs7000_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
	struct input_info info;//[FINGER_NUM+1];
	int suspended;
	struct early_suspend	early_suspend;
};

struct mcs7000_ts_driver *melfas_mcs7000_ts = NULL;
struct i2c_driver mcs7000_ts_i2c;
struct workqueue_struct *melfas_mcs7000_ts_wq;

static struct vreg *vreg_touch;

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs7000_ts_early_suspend(struct early_suspend *h);
void melfas_mcs7000_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

//#define TOUCH_HOME	KEY_HOME
//#define TOUCH_MENU	KEY_MENU
//#define TOUCH_BACK	KEY_BACK
//#define TOUCH_SEARCH  KEY_SEARCH

//int melfas_ts_tk_keycode[] =
//{ TOUCH_HOME, TOUCH_MENU, TOUCH_BACK, TOUCH_SEARCH, };

struct device *mcs7000_ts_dev;

void mcsdl_vdd_on_mcs7000(void)
{ 
  vreg_set_level(vreg_touch, OUT3300mV);//h/w request 2.8 -> 3.3
  vreg_enable(vreg_touch);
  mdelay(25); //MUST wait for 25ms after vreg_enable() 
}

void mcsdl_vdd_off_mcs7000(void)
{
  vreg_disable(vreg_touch);
  mdelay(100); //MUST wait for 100ms before vreg_enable() 
}

static int melfas_mcs7000_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{

	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}
	
	return 0;
}

static void melfas_mcs7000_read_version(void)
{
	u8 buf[2] = {0,};
	
	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_MODULE_VER_REG, buf, 2))
	{

		melfas_mcs7000_ts->hw_rev = buf[0];
		melfas_mcs7000_ts->fw_ver = buf[1];
		
		printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
	}
	else
	{
		melfas_mcs7000_ts->hw_rev = 0;
		melfas_mcs7000_ts->fw_ver = 0;
		
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}
}

static void melfas_mcs7000_read_resolution(void)
{
	
	uint16_t max_x=0, max_y=0;	

	u8 buf[3] = {0,};
	
	if(0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_RESOL_HIGH_REG , buf, 3)){

		printk("%s :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,buf[0],buf[1],buf[2]);

		if((buf[0] == 0)||(buf[0] == 0)||(buf[0] == 0)){
			melfas_mcs7000_ts->info.max_x = 320;
			melfas_mcs7000_ts->info.max_y = 480;
			
			printk("%s : Can't find Resolution!\n", __func__);
			}
		
		else{
			max_x = buf[1] | ((uint16_t)(buf[0] & 0x0f) << 8); 
			max_y = buf[2] | (((uint16_t)(buf[0] & 0xf0) >> 4) << 8); 
			melfas_mcs7000_ts->info.max_x = max_x;
		    melfas_mcs7000_ts->info.max_y = max_y;

			printk("%s :max_x: %d, max_y: %d\n", __func__, melfas_mcs7000_ts->info.max_x, melfas_mcs7000_ts->info.max_y);
			}
		}

	else
	{
		melfas_mcs7000_ts->info.max_x = 320;
		melfas_mcs7000_ts->info.max_y = 480;
		
		printk("%s : Can't find Resolution!\n", __func__);
	}
}

static ssize_t registers_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 buf1[2] = {0,};
	u8 buf2[2] = {0,};

	int status, mode_ctl, hw_rev, fw_ver;

	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_STATUS_REG, buf1, 2))
	{
		status = buf1[0];
		mode_ctl = buf1[1];	 
	}
	else
	{
		printk("%s : Can't find status, mode_ctl!\n", __func__); 
	}

	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_MODULE_VER_REG, buf2, 2))
	{
		hw_rev = buf2[0];
		fw_ver = buf2[1];	 
	}
	else
	{
		printk("%s : Can't find HW Ver, FW ver!\n", __func__); 
	}

	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status)  : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (mode_ctl): 0x%08x\n", buf, mode_ctl);
	sprintf(buf, "%sRegister 0x30 (hw_rev)  : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x31 (fw_ver)  : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);

}

static ssize_t registers_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		
	    ret = i2c_smbus_write_byte_data(melfas_mcs7000_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW"); 
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
    mcsdl_vdd_on_mcs7000();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
    mcsdl_vdd_off_mcs7000();
		printk("[TOUCH] disable.\n");
	}
	
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
    mcsdl_vdd_off_mcs7000();
		mdelay(500);
    mcsdl_vdd_on_mcs7000();
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}


static ssize_t firmware_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	u8 buf1[2] = {0,};
	int hw_rev, fw_ver;


	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_MODULE_VER_REG, buf1, 2))
	{
		hw_rev = buf1[0];
		fw_ver = buf1[1];	 
		sprintf(buf,"HW Ver : 0x%02x, FW Ver : 0x%02x\n", hw_rev, fw_ver);
	}
	else
	{	 
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}

return sprintf(buf, "%s", buf); 
}


static ssize_t firmware_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {
		printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", melfas_mcs7000_ts->hw_rev);
		printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs7000_ts->fw_ver);
		if( melfas_mcs7000_ts->fw_ver != NEW_FIRMWARE_VERSION ) { 
		disable_irq(melfas_mcs7000_ts->client->irq);

		printk("[F/W D/L] Entry gpio_tlmm_config\n");
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		
		printk("[F/W D/L] Entry mcsdl_mcs7000_download_binary_data\n");
		ret = mcsdl_mcs7000_download_binary_data(); //eunsuk test [melfas_mcs7000_ts->hw_rev -> ()]
		
		enable_irq(melfas_mcs7000_ts->client->irq);
		
		melfas_mcs7000_read_version();
			
		if(ret > 0){
				if (melfas_mcs7000_ts->hw_rev < 0) {
					printk(KERN_ERR "i2c_transfer failed\n");;
				}
				
				if (melfas_mcs7000_ts->fw_ver < 0) {
					printk(KERN_ERR "i2c_transfer failed\n");
				}
				
				printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs7000_ts->hw_rev, melfas_mcs7000_ts->fw_ver);

		}
		else {
			printk("[TOUCH] Firmware update failed.. RESET!\n");
      mcsdl_vdd_off_mcs7000();
			mdelay(500);
      mcsdl_vdd_on_mcs7000();
			mdelay(200);
		}
	}
}
#endif

	return size;
}



static ssize_t debug_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d", debug_level);
}

static ssize_t debug_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf[0]>'0' && buf[0]<='9') {
		debug_level = buf[0] - '0';
	}

	return size;
}


static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show_mcs7000, gpio_store_mcs7000);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show_mcs7000, registers_store_mcs7000);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR | S_IWGRP /*S_IWUGO*/, firmware_show_mcs7000, firmware_store_mcs7000);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show_mcs7000, debug_store_mcs7000);





#if NOISE_TEST // [ MBsmhan -  *#80# noise test


static ssize_t set_delta0_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_delta=0;
    if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_DELTA_INTENSITY_1, &melfas_delta, 1))
    {
        if(melfas_delta < 32767){
     	    status = sprintf(buf, "%u\n", melfas_delta);
        }
        else {
	    melfas_delta = 65535 - melfas_delta;
	    status = sprintf(buf, "-%u\n", melfas_delta);
        }
        return status;
    }
    else
    {	 
	printk("%s : Fail I2C Read\n", __func__);
    }
}

static ssize_t set_delta1_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_delta=0;
    if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_DELTA_INTENSITY_2, &melfas_delta, 1))
    {
        if(melfas_delta < 32767){
     	    status = sprintf(buf, "%u\n", melfas_delta);
        }
        else {
	    melfas_delta = 65535 - melfas_delta;
	    status = sprintf(buf, "-%u\n", melfas_delta);
        }
        return status;
    }
    else
    {	 
	printk("%s : Fail I2C Read\n", __func__);
    }
}

static ssize_t set_delta2_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_delta=0;
    if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_DELTA_INTENSITY_3, &melfas_delta, 1))
    {
        if(melfas_delta < 32767){
     	    status = sprintf(buf, "%u\n", melfas_delta);
        }
        else {
	    melfas_delta = 65535 - melfas_delta;
	    status = sprintf(buf, "-%u\n", melfas_delta);
        }
        return status;
    }
    else
    {	 
	printk("%s : Fail I2C Read\n", __func__);
    }
}

static ssize_t set_delta3_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_delta=0;
    if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_DELTA_INTENSITY_4, &melfas_delta, 1))
    {
        if(melfas_delta < 32767){
     	    status = sprintf(buf, "%u\n", melfas_delta);
        }
        else {
	    melfas_delta = 65535 - melfas_delta;
	    status = sprintf(buf, "-%u\n", melfas_delta);
        }
        return status;
    }
    else
    {	 
	printk("%s : Fail I2C Read\n", __func__);
    }
}

static ssize_t set_delta4_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_delta=0;
    if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_DELTA_INTENSITY_5, &melfas_delta, 1))
    {
        if(melfas_delta < 32767){
     	    status = sprintf(buf, "%u\n", melfas_delta);
        }
        else {
	    melfas_delta = 65535 - melfas_delta;
	    status = sprintf(buf, "-%u\n", melfas_delta);
        }
        return status;
    }
    else
    {	 
	printk("%s : Fail I2C Read\n", __func__);
    }
}


static ssize_t set_enable0_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    ssize_t status = 0;
    ret = i2c_smbus_write_byte_data(melfas_mcs7000_ts->client, 0xA0, 0x1E);
    if (ret < 0) {
        printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
    }
    status = sprintf(buf, " ");    
    mdelay(50);
    return status;
}

static ssize_t set_disable0_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    mdelay(50);
    int ret = 0;
    ssize_t status = 0;
    ret = i2c_smbus_write_byte_data(melfas_mcs7000_ts->client, 0xA0, 0x00);
    if (ret < 0) {
        printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
    }
    status = sprintf(buf, " "); 
    return status;
}


static ssize_t set_threshold_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 melfas_threshold=0;

    if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_THRESHOLD, &melfas_threshold, 1))
    {
        return sprintf(buf, "%u\n", melfas_threshold);
    }
    else
    {	 
	printk("%s : Fail I2C Read\n", __func__);
    }
}


/* --------------------------------------------- */

static ssize_t set_refer0_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_1_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_1_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer1_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_2_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_2_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer2_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_3_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_3_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer3_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_4_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_4_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer4_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_5_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_5_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer5_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_6_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_6_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer6_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_7_H, &melfas_refer_h, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_7_L, &melfas_refer_l, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_refer7_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u16 melfas_refer=0;
    u8 melfas_refer_h=0;
    u8 melfas_refer_l=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_8_H, &melfas_refer_h, 1))
    {
	printk("%s : Fail I2C Read\n", __func__);
    }
    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_REFER_DATA_8_L, &melfas_refer_l, 1))
    {
	printk("%s : Fail I2C Read\n", __func__);
    }
    melfas_refer = melfas_refer_h;
    melfas_refer = melfas_refer << 8;
    melfas_refer = (melfas_refer | melfas_refer_l);
    status = sprintf(buf, "%u\n", melfas_refer);

    return status;
}


static ssize_t set_sensing_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_sensing=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_SENSING_CH_NUM, &melfas_sensing, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    status = sprintf(buf, "%u\n", melfas_sensing);
    return status;
}

static ssize_t set_exciting_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t status = 0;
    u8 melfas_exciting=0;

    if (0 != melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_EXCITING_CH_NUM, &melfas_exciting, 1))
    {
        printk("%s : Fail I2C Read\n", __func__);
    }
    status = sprintf(buf, "%u\n", melfas_exciting);
    return status;
}


static ssize_t set_enable_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    ssize_t status = 0;
    ret = i2c_smbus_write_byte_data(melfas_mcs7000_ts->client, 0xA0, 0x1F);
    if (ret < 0) {
        printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
    }
    status = sprintf(buf, " ");
    mdelay(50);
    return status;
}

static ssize_t set_disable_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    mdelay(50);
    int ret = 0;
    ssize_t status = 0;
    ret = i2c_smbus_write_byte_data(melfas_mcs7000_ts->client, 0xA0, 0x00);
    if (ret < 0) {
        printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
    }
    status = sprintf(buf, " ");
    return status;
}


/* ---------------------------------------- */

static DEVICE_ATTR(set_delta0, S_IRUGO | S_IWUSR , set_delta0_mcs7000, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO | S_IWUSR , set_delta1_mcs7000, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO | S_IWUSR , set_delta2_mcs7000, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO | S_IWUSR , set_delta3_mcs7000, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO | S_IWUSR , set_delta4_mcs7000, NULL);
static DEVICE_ATTR(set_enable0, S_IRUGO | S_IWUSR , set_enable0_mcs7000, NULL);
static DEVICE_ATTR(set_disable0, S_IRUGO | S_IWUSR , set_disable0_mcs7000, NULL);
static DEVICE_ATTR(set_threshould, S_IRUGO | S_IWUSR , set_threshold_mcs7000, NULL);

static DEVICE_ATTR(set_refer0, S_IRUGO | S_IWUSR , set_refer0_mcs7000, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO | S_IWUSR , set_refer1_mcs7000, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO | S_IWUSR , set_refer2_mcs7000, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO | S_IWUSR , set_refer3_mcs7000, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO | S_IWUSR , set_refer4_mcs7000, NULL);
static DEVICE_ATTR(set_refer5, S_IRUGO | S_IWUSR , set_refer5_mcs7000, NULL);
static DEVICE_ATTR(set_refer6, S_IRUGO | S_IWUSR , set_refer6_mcs7000, NULL);
static DEVICE_ATTR(set_refer7, S_IRUGO | S_IWUSR , set_refer7_mcs7000, NULL);
static DEVICE_ATTR(set_sensing, S_IRUGO | S_IWUSR , set_sensing_mcs7000, NULL);
static DEVICE_ATTR(set_exciting, S_IRUGO | S_IWUSR , set_exciting_mcs7000, NULL);
static DEVICE_ATTR(set_enable, S_IRUGO | S_IWUSR , set_enable_mcs7000, NULL);
static DEVICE_ATTR(set_disable, S_IRUGO | S_IWUSR , set_disable_mcs7000, NULL);

/* ---------------------------------------- */


#endif // ] MBsmhan


void melfas_mcs7000_upgrade(void)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	
	printk("[TOUCH] Melfas	H/W version: 0x%02x.\n", melfas_mcs7000_ts->hw_rev);
	printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs7000_ts->fw_ver);

	disable_irq(melfas_mcs7000_ts->client->irq);

	printk("[F/W D/L] Entry gpio_tlmm_config\n");
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	
	printk("[F/W D/L] Entry mcsdl_mcs7000_download_binary_data\n");
	ret = mcsdl_mcs7000_download_binary_data(); //eunsuk test [melfas_mcs7000_ts->hw_rev -> ()]
	
	enable_irq(melfas_mcs7000_ts->client->irq);
	
	melfas_mcs7000_read_version();
		
	if(ret > 0){
			if (melfas_mcs7000_ts->hw_rev < 0) {
				printk(KERN_ERR "i2c_transfer failed\n");;
			}
			
			if (melfas_mcs7000_ts->fw_ver < 0) {
				printk(KERN_ERR "i2c_transfer failed\n");
			}
			
			printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs7000_ts->hw_rev, melfas_mcs7000_ts->fw_ver);

	}
	else {
		printk("[TOUCH] Firmware update failed.. RESET!\n");
  		mcsdl_vdd_off_mcs7000();
		mdelay(500);
  		mcsdl_vdd_on_mcs7000();
		mdelay(200);
	}
#endif
}

void melfas_mcs7000_ts_work_func(struct work_struct *work)
{
  int ret;
  int ret1; 
  int i = 0;
  u8 id = 0;


  struct i2c_msg msg[2];
  
  uint8_t start_reg;
  uint8_t buf1[10];

  msg[0].addr = melfas_mcs7000_ts->client->addr;
  msg[0].flags = 0; 
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  start_reg = MCSTS_INPUT_INFO_REG;
  msg[1].addr = melfas_mcs7000_ts->client->addr;
  msg[1].flags = I2C_M_RD; 
  msg[1].len = sizeof(buf1);
  msg[1].buf = buf1;
  

  ret  = i2c_transfer(melfas_mcs7000_ts->client->adapter, &msg[0], 1);
  ret1 = i2c_transfer(melfas_mcs7000_ts->client->adapter, &msg[1], 1);

  if((ret < 0) ||  (ret1 < 0)) 
  	{
  	printk(KERN_ERR "==melfas_mcs7000_ts_work_func: i2c_transfer failed!!== ret:%d ,ret1:%d\n",ret,ret1);
	}
  else
  	{    
    int x = buf1[4] | (((uint16_t)(buf1[3] & 0xf0) >> 4) << 8); 
    int y = buf1[5] | ((uint16_t)(buf1[3] & 0x0f) << 8); 
    int z = buf1[6];	
    int x2 = buf1[8] | (((uint16_t)(buf1[7] & 0xf0) >> 4) << 8); 
    int y2 = buf1[9] | ((uint16_t)(buf1[7] & 0x0f) << 8);  
    int z2 = buf1[10];
    int finger = buf1[2]| ((uint16_t)(buf1[1] & 0x0f) << 8); 
	int touchaction = buf1[0]; //Touch action	

#ifdef CONFIG_CPU_FREQ
    set_dvfs_perf_level();
#endif
	int do_report1=false, do_report2=false;
	  printk("===finger_id : 0x%02x===\n",finger);

      switch(finger) {
        case 0x0: // Non-touched state
			melfas_mcs7000_ts->info.x = -1;
			melfas_mcs7000_ts->info.y = -1;
			melfas_mcs7000_ts->info.z = -1;
			melfas_mcs7000_ts->info.x2 = -1;
			melfas_mcs7000_ts->info.y2 = -1;
			melfas_mcs7000_ts->info.z2 = -1;
			melfas_mcs7000_ts->info.finger_id = finger; 
			do_report1= false;
			do_report2=false;
			z2 = 0;
          break;

        case 0x1: // finger 1
			melfas_mcs7000_ts->info.x = x;
			melfas_mcs7000_ts->info.y = y;
			melfas_mcs7000_ts->info.z = z;		  
			melfas_mcs7000_ts->info.finger_id = finger; 
			do_report1 = true;
			do_report2 = false;
			z2 = 0;
		  	break;
		  
        case 0x2: //finger 2	
			melfas_mcs7000_ts->info.x2 = x2;
			melfas_mcs7000_ts->info.y2 = y2;
			melfas_mcs7000_ts->info.z2 = z2;
			melfas_mcs7000_ts->info.finger_id = finger; 
			do_report1 = false;
			do_report2 = true;
          	break;

         case 0x3: // dual touch
			melfas_mcs7000_ts->info.x = x;
			melfas_mcs7000_ts->info.y = y;
			melfas_mcs7000_ts->info.z = z;
			melfas_mcs7000_ts->info.x2 = x2;
			melfas_mcs7000_ts->info.y2 = y2;
			melfas_mcs7000_ts->info.z2 = z2;
			melfas_mcs7000_ts->info.finger_id = finger; 
			do_report1 = true;
			do_report2 = true;
          	break;

      }
        
      melfas_mcs7000_ts->info.state = touchaction;

		{
		if(do_report1){
			debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, melfas_mcs7000_ts->info.x);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, melfas_mcs7000_ts->info.y);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, melfas_mcs7000_ts->info.z);		
			input_mt_sync(melfas_mcs7000_ts->input_dev);
			}
		if(do_report2){
			debugprintk(5,"[TOUCH_MT2] x2: %4d, y2: %4d, z2: %4d, finger: %4d,\n", x2, y2, z2, finger);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, melfas_mcs7000_ts->info.x2);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, melfas_mcs7000_ts->info.y2);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, melfas_mcs7000_ts->info.z2); 	
			input_mt_sync(melfas_mcs7000_ts->input_dev);
			}
		
		if(melfas_mcs7000_ts->info.finger_id == 0)
			input_mt_sync(melfas_mcs7000_ts->input_dev);
		}

    input_sync(melfas_mcs7000_ts->input_dev);
  }
  
  enable_irq(melfas_mcs7000_ts->irq);
}


irqreturn_t melfas_mcs7000_ts_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(melfas_mcs7000_ts->irq);
	//disable_irq(melfas_mcs7000_ts->irq);
	queue_work(melfas_mcs7000_ts_wq, &melfas_mcs7000_ts->work);
	return IRQ_HANDLED;
}

int melfas_mcs7000_ts_probe(void)
{
	int ret = 0;
	uint16_t max_x=0, max_y=0;
	//int fw_ver = 0;
	//int hw_rev = 0;
	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] PROBE       =========");
	printk("\n====================================================\n");


	if (!i2c_check_functionality(melfas_mcs7000_ts->client->adapter, I2C_FUNC_I2C/*I2C_FUNC_SMBUS_BYTE_DATA*/)) {
		printk(KERN_ERR "melfas_mcs7000_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	INIT_WORK(&melfas_mcs7000_ts->work, melfas_mcs7000_ts_work_func);
	
	melfas_mcs7000_read_version(); 
	if(melfas_mcs7000_ts->fw_ver != NEW_FIRMWARE_VERSION)//cha_temp
	melfas_mcs7000_upgrade(); 

	
	printk(KERN_INFO "[TOUCH] Melfas  H/W version: 0x%x.\n", melfas_mcs7000_ts->hw_rev);
	printk(KERN_INFO "[TOUCH] Current F/W version: 0x%x.\n", melfas_mcs7000_ts->fw_ver);
	
	melfas_mcs7000_read_resolution(); 
	max_x = melfas_mcs7000_ts->info.max_x ;
	max_y = melfas_mcs7000_ts->info.max_y ;

	melfas_mcs7000_ts->input_dev = input_allocate_device();
	if (melfas_mcs7000_ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_mcs7000_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	melfas_mcs7000_ts->input_dev->name = "mcs7000_ts_input";

	set_bit(EV_SYN, melfas_mcs7000_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_mcs7000_ts->input_dev->evbit);
	//set_bit(TOUCH_HOME, melfas_mcs7000_ts->input_dev->keybit);
	//set_bit(TOUCH_MENU, melfas_mcs7000_ts->input_dev->keybit);
	//set_bit(TOUCH_BACK, melfas_mcs7000_ts->input_dev->keybit);
	//set_bit(TOUCH_SEARCH, melfas_mcs7000_ts->input_dev->keybit);

	//melfas_mcs7000_ts->input_dev->keycode = melfas_ts_tk_keycode;	
	set_bit(BTN_TOUCH, melfas_mcs7000_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_mcs7000_ts->input_dev->evbit);

	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	printk("melfas_mcs7000_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	ret = input_register_device(melfas_mcs7000_ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_mcs7000_ts_probe: Unable to register %s input device\n", melfas_mcs7000_ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	melfas_mcs7000_ts->irq = melfas_mcs7000_ts->client->irq; //add by KJB
	ret = request_irq(melfas_mcs7000_ts->client->irq, melfas_mcs7000_ts_irq_handler, IRQF_DISABLED, "melfas_ts irq", 0);
	//ret = request_threaded_irq(melfas_mcs7000_ts->client->irq, NULL, melfas_mcs7000_ts_irq_handler,IRQF_ONESHOT,"melfas_ts irq", 0);
	if(ret == 0) {
		printk(KERN_INFO "melfas_mcs7000_ts_probe: Start touchscreen %s \n", melfas_mcs7000_ts->input_dev->name);
	}
	else {
		printk("request_irq failed\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_mcs7000_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_mcs7000_ts->early_suspend.suspend = melfas_mcs7000_ts_early_suspend;
	melfas_mcs7000_ts->early_suspend.resume = melfas_mcs7000_ts_late_resume;
	register_early_suspend(&melfas_mcs7000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	return 0;
err_input_register_device_failed:
	input_free_device(melfas_mcs7000_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(melfas_mcs7000_ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}

int melfas_mcs7000_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs7000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs7000_ts->irq, 0);
	input_unregister_device(melfas_mcs7000_ts->input_dev);
	return 0;
}

int melfas_mcs7000_ts_gen_touch_up(void)
{
  // report up key if needed
  int i;
  for ( i= 1; i<FINGER_NUM; ++i ){
  if(melfas_mcs7000_ts->info.state == 0x1) //down state
  {
    melfas_mcs7000_ts->info.state = 0x0;
	int finger = melfas_mcs7000_ts->info.finger_id;
    int x = melfas_mcs7000_ts->info.x;
    int y = melfas_mcs7000_ts->info.y;
    int z = melfas_mcs7000_ts->info.z;
    printk("[TOUCH] GENERATE UP KEY x: %4d, y: %4d, z: %4d\n", x, y, z);
	input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TRACKING_ID, finger);
    if (x) 	input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, x);
    if (y)	input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(melfas_mcs7000_ts->input_dev, ABS_PRESSURE, z);

    input_sync(melfas_mcs7000_ts->input_dev);
  }    
	}
}

int melfas_mcs7000_ts_suspend(pm_message_t mesg)
{
  melfas_mcs7000_ts->suspended = true;
  melfas_mcs7000_ts_gen_touch_up();
  disable_irq(melfas_mcs7000_ts->irq);
  
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  
  mcsdl_vdd_off_mcs7000();
  gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
  gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
  
  return 0;
}

int melfas_mcs7000_ts_resume()
{
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

  mcsdl_vdd_on_mcs7000();
  gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
  gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN    
  msleep(250); //sdk_ec28 500->250 by Lee Sanghwa (Melfas, Inc.)
  melfas_mcs7000_ts->suspended = false;
  enable_irq(melfas_mcs7000_ts->irq);  

  return 0;
}
#if 0 // blocked for now.. we will gen touch when suspend func is called
int tsp_preprocess_suspend(void)
{
  // this function is called before kernel calls suspend functions
  // so we are going suspended if suspended==false
  if(melfas_mcs7000_ts->suspended == false) {  
    // fake as suspended
    melfas_mcs7000_ts->suspended = true;
    
    //generate and report touch event
    melfas_mcs7000_ts_gen_touch_up();
  }
  return 0;
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs7000_ts_early_suspend(struct early_suspend *h)
{
	melfas_mcs7000_ts_suspend(PMSG_SUSPEND);
}

void melfas_mcs7000_ts_late_resume(struct early_suspend *h)
{
	melfas_mcs7000_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */


int melfas_mcs7000_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_mcs7000_ts->client = client;
	i2c_set_clientdata(client, melfas_mcs7000_ts);
	return 0;
}

static int __devexit melfas_mcs7000_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs7000_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs7000_ts->client->irq, 0);
	input_unregister_device(melfas_mcs7000_ts->input_dev);
   
	melfas_mcs7000_ts = i2c_get_clientdata(client);
	kfree(melfas_mcs7000_ts);
	return 0;
}

struct i2c_device_id melfas_mcs7000_id[] = {
	{ "mcs7000_i2c", 0 },
	{ }
};

struct i2c_driver mcs7000_ts_i2c = {
	.driver = {
		.name	= "mcs7000_i2c",
		.owner	= THIS_MODULE,
	},
	.probe 		= melfas_mcs7000_i2c_probe,
	.remove		= __devexit_p(melfas_mcs7000_i2c_remove),
	.id_table	= melfas_mcs7000_id,
};


void init_hw_setting_mcs7000(void)
{
	int ret;

	vreg_touch = vreg_get(NULL, "ldo19"); /* VTOUCH_2.8V */

	ret = vreg_enable(vreg_touch);
	if (ret) { 
		printk(KERN_ERR "%s: vreg_touch enable failed (%d)\n", __func__, ret);
		return;//-EIO;
	}
	else {
		printk(KERN_INFO "%s: vreg_touch enable success!\n", __func__);
	}

	mdelay(100);
	
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_EDGE_FALLING);

	mdelay(10);

}

struct platform_driver mcs7000_ts_driver =  {
	.probe	= melfas_mcs7000_ts_probe,
	.remove = melfas_mcs7000_ts_remove,
	.driver = {
		.name = "mcs7000-ts",
		.owner	= THIS_MODULE,
	},
};

#if NOISE_TEST // [ MBsmhan -  *#80# noise test
struct device *melfas_noise_test;
struct device *melfas_tsp_test;
#endif // ]

int __init melfas_mcs7000_ts_init(void)
{
#if defined(CONFIG_MACH_ROOKIE)
    if(hw_version < 3)
    {
		melfas_ts_init();
    }
	else
#endif
{
	int ret;

	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] INIT        =========");
	printk("\n====================================================\n");

	init_hw_setting_mcs7000();

       //HYH_20110314	mcs7000_sec_class = class_create(THIS_MODULE, "sec"); //sdk_eb25

	mcs7000_ts_dev = device_create(sec_class_1, NULL, 0, NULL, "ts"); //sdk_eb25
	if (IS_ERR(mcs7000_ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(mcs7000_ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);

#if NOISE_TEST // [ MBsmhan -  *#80# noise test


	melfas_noise_test = device_create(sec_class_1, NULL, 0, NULL, "melfas_noise_test");
	if (IS_ERR(melfas_noise_test))
		printk("Failed to create device(melfas_noise_test)!\n");

	if (device_create_file(melfas_noise_test, &dev_attr_set_delta0) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta0.attr.name);
	if (device_create_file(melfas_noise_test, &dev_attr_set_delta1) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta1.attr.name);
	if (device_create_file(melfas_noise_test, &dev_attr_set_delta2) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta2.attr.name);
	if (device_create_file(melfas_noise_test, &dev_attr_set_delta3) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta3.attr.name);
	if (device_create_file(melfas_noise_test, &dev_attr_set_delta4) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta4.attr.name);	

	if (device_create_file(melfas_noise_test, &dev_attr_set_enable0) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_enable0.attr.name);
	if (device_create_file(melfas_noise_test, &dev_attr_set_disable0) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_disable0.attr.name);

	if (device_create_file(melfas_noise_test, &dev_attr_set_threshould) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_threshould.attr.name);

	melfas_tsp_test = device_create(sec_class_1, NULL, 0, NULL, "melfas_tsp_test");
	if (IS_ERR(melfas_tsp_test))
		printk("Failed to create device(melfas_tsp_test)!\n");
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer0) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer0.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer1) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer1.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer2) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer2.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer3) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer3.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer4) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer4.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer5) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer5.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer6) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer6.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_refer7) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer7.attr.name);

	if (device_create_file(melfas_tsp_test, &dev_attr_set_enable) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_enable.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_disable) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_disable.attr.name);

	if (device_create_file(melfas_tsp_test, &dev_attr_set_sensing) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_sensing.attr.name);
	if (device_create_file(melfas_tsp_test, &dev_attr_set_exciting) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_exciting.attr.name);


 #endif // ] MBsmhan


	melfas_mcs7000_ts = kzalloc(sizeof(struct mcs7000_ts_driver), GFP_KERNEL);
	if(melfas_mcs7000_ts == NULL) {
		return -ENOMEM;
	}

	ret = i2c_add_driver(&mcs7000_ts_i2c);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	if(!melfas_mcs7000_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&mcs7000_ts_i2c);
		return 0;
	}
	melfas_mcs7000_ts_wq = create_singlethread_workqueue("melfas_mcs7000_ts_wq");
	if (!melfas_mcs7000_ts_wq)
		return -ENOMEM;

	return platform_driver_register(&mcs7000_ts_driver);
}
}

void __exit melfas_mcs7000_ts_exit(void)
{
#if defined(CONFIG_MACH_ROOKIE)
	if(hw_version < 3)
	{
		melfas_ts_exit();
	}
	else
#endif
	{
		i2c_del_driver(&mcs7000_ts_i2c);

		if (melfas_mcs7000_ts_wq)
			destroy_workqueue(melfas_mcs7000_ts_wq);
	}
}
late_initcall(melfas_mcs7000_ts_init);
//module_init(melfas_mcs7000_ts_init);
module_exit(melfas_mcs7000_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
