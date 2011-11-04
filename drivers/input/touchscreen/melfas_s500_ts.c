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
//#include <mach/vreg.h>

#include <linux/slab.h>

#include "melfas_download.h"
#if defined(CONFIG_MACH_ROOKIE)
#include "melfas_ts.h"
#endif

//#define MELFAS_I2C_ADDR 0x40
#define NEW_FIRMWARE_VERSION 0x06
#define INPUT_INFO_REG 0x10

#define IRQ_TOUCH_INT   MSM_GPIO_TO_INT(GPIO_TOUCH_INT)
//#define IRQ_TOUCH_INT IRQ_EINT(8)

#undef CONFIG_CPU_FREQ
#undef CONFIG_MOUSE_OPTJOY

#ifdef CONFIG_CPU_FREQ
#include <plat/s3c64xx-dvfs.h>
#endif

static int debug_level = 5; //TEMP
#define debugprintk(level,x...)  if(debug_level>=level) printk(x)

extern int mcsdl_download_binary_data(int hw_ver);
#ifdef CONFIG_MOUSE_OPTJOY
extern int get_sending_oj_event();
#endif

extern struct class *sec_class_1;
//EXPORT_SYMBOL(sec_class);

struct input_info {
	int max_x;
	int max_y;
	int state;
	int x;
	int y;
	int z;
	int width;
	int finger_id; //for chief
};

struct melfas_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
#ifdef USES_PINCH_DIST_MT
	int mt_support; // multitouch support
	int mt_disable;
#endif
	struct input_info info;
	int suspended;
	struct early_suspend	early_suspend;
};

struct melfas_ts_driver *melfas_ts = NULL;
struct i2c_driver melfas_ts_i2c;
struct workqueue_struct *melfas_ts_wq;

//static struct vreg *vreg_touch;

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_ts_early_suspend(struct early_suspend *h);
void melfas_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

//#define TOUCH_HOME	KEY_HOME
//#define TOUCH_MENU	KEY_MENU
//#define TOUCH_BACK	KEY_BACK
//#define TOUCH_SEARCH  KEY_SEARCH


//int melfas_ts_tk_keycode[] =
//{ TOUCH_HOME, TOUCH_MENU, TOUCH_BACK, TOUCH_SEARCH, };

struct device *ts_dev;

void mcsdl_vdd_on(void)
{ 
//  vreg_set_level(vreg_touch, OUT2800mV);
//  vreg_enable(vreg_touch);
  mdelay(25); //MUST wait for 25ms after vreg_enable()
}

void mcsdl_vdd_off(void)
{
//  vreg_disable(vreg_touch);
  // mdelay(100); //MUST wait for 100ms before vreg_enable()
}

#ifdef USES_PINCH_DIST_MT
int melfas_check_mt_support(int do_update)
{
	static int first = true;

	if(do_update || first) {
		if(melfas_ts->fw_ver >= 0x04)
			melfas_ts->mt_support = true;
		//else
		//	melfas_ts->mt_support = false;

		first = false;
	}
#if defined(CONFIG_MACH_CHIEF)  
	melfas_ts->mt_support = true;
#endif

	return melfas_ts->mt_support;

	//if(do_update || first) {
	//	melfas_ts->mt_support = 0;
	//}
	//return melfas_ts->mt_support;

}
#endif

static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status, mode_ctl, hw_rev, fw_ver;
	
	status  = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_STATUS_REG);
	if (status < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	mode_ctl = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODE_CONTROL_REG);
	if (mode_ctl < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODULE_VER_REG);
	if (hw_rev < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_FIRMWARE_VER_REG);
	if (fw_ver < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	
	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status)  : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (mode_ctl): 0x%08x\n", buf, mode_ctl);
	sprintf(buf, "%sRegister 0x30 (hw_rev)  : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x31 (fw_ver)  : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);
}

static ssize_t registers_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		
	    ret = i2c_smbus_write_byte_data(melfas_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	//sprintf(buf, "%sGPIO TOUCH_EN  : %s\n", buf, gpio_get_value(GPIO_TOUCH_EN)? "HIGH":"LOW");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW"); 
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
    mcsdl_vdd_on();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
    mcsdl_vdd_off();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
		printk("[TOUCH] disable.\n");
	}
	
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
    mcsdl_vdd_off();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
		mdelay(500);
    mcsdl_vdd_on();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}
//MBjclee 2011.02.11 For Rookie's firmware update.
#if 1 //for chief by melfas 
static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int hw_rev, fw_ver;
	
	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODULE_VER_REG);
	if (hw_rev < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}
	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_FIRMWARE_VER_REG);
	if (fw_ver < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}

	sprintf(buf, "H/W rev. 0x%x F/W ver. 0x%x\n", hw_rev, fw_ver);
	return sprintf(buf, "%s", buf);
}

static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {
		printk("[TOUCH] Melfas  H/W version: 0x%x.\n", melfas_ts->hw_rev);
		printk("[TOUCH] Current F/W version: 0x%x.\n", melfas_ts->fw_ver);

		disable_irq(melfas_ts->client->irq);

		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

		ret = mcsdl_download_binary_data(melfas_ts->hw_rev);

		enable_irq(melfas_ts->client->irq);

		if(ret) {
			int hw_rev, fw_ver;
			hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODULE_VER_REG);
			if (hw_rev < 0) {
				printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
			}
			else {
				melfas_ts->hw_rev = hw_rev;
			}
			fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_FIRMWARE_VER_REG);
			if (fw_ver < 0) {
				printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
			}
			else {
				melfas_ts->fw_ver = fw_ver;
			}
			printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%x., Current F/W version: 0x%x.]\n", hw_rev, fw_ver);
		}
		else {
			printk("[TOUCH] Firmware update failed.. RESET!\n");
      mcsdl_vdd_off();
			//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_LOW);
			mdelay(500);
      mcsdl_vdd_on();
			//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
			mdelay(200);
		}
	}
#endif

	return size;
}

#endif

static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d", debug_level);
}

static ssize_t debug_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf[0]>'0' && buf[0]<='9') {
		debug_level = buf[0] - '0';
	}

	return size;
}

#ifdef USES_PINCH_DIST_MT
static ssize_t multitouch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%s", melfas_ts->mt_disable?"disabled":"enabled");
}

static ssize_t multitouch_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "enable", 6) == 0) {
		melfas_ts->mt_disable = false;
		printk("[TOUCH] Multi-touch enabled\n");
	}
	else if(strncmp(buf, "disable", 7) == 0) {
		melfas_ts->mt_disable = true;
		printk("[TOUCH] Multi-touch disabled\n");
	}

	return size;
}

static DEVICE_ATTR(multitouch, S_IRUGO | S_IWUSR, multitouch_show, multitouch_store);
#endif


static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show, registers_store);

//for chief
static DEVICE_ATTR(firmware, S_IRUGO | /*S_IWUGO |*/ S_IWUSR | S_IWGRP, firmware_show, firmware_store);	//MBjclee 2011.02.11 for Rookie's fimware update.

static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show, debug_store);


void melfas_ts_work_func(struct work_struct *work)
{
  int ret;
  
  struct i2c_msg msg[2];

  uint8_t start_reg;
  uint8_t buf1[8];
  
  msg[0].addr = melfas_ts->client->addr;
  msg[0].flags = 0; //write
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  start_reg = MCSTS_INPUT_INFO_REG;
  
  msg[1].addr = melfas_ts->client->addr;
  msg[1].flags = I2C_M_RD; //read
  msg[1].len = sizeof(buf1);
  msg[1].buf = buf1;

  ret = i2c_transfer(melfas_ts->client->adapter, msg, 2);

  if (ret < 0)
  {
    printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
  }
  else
  {    
    int x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8; 
    int y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8; 
    int z = buf1[5];
    int finger = buf1[0] & 0x01;
    int width = buf1[6];
    int touchtype = buf1[0] & (BIT(0)|BIT(1)|BIT(2));
#ifdef USES_PINCH_DIST_MT
    int gesture_code = (buf1[0] & (BIT(3)|BIT(4)|BIT(5)))>>3;
    static int x2,y2,z2;
#endif

#ifdef CONFIG_CPU_FREQ
    set_dvfs_perf_level();
#endif

#if 0
    if(buf1[0] == 0x80) {
        // ESD !!!
        // turn TSP power off => (100ms) => turn TSP power back on

        printk(KERN_ERR "[TOUCH] ESD detected resetting TSP!!!");
        mcsdl_vdd_off();
        //gpio_set_value(GPIO_TOUCH_EN, 0);
        mdelay(100);
        mcsdl_vdd_on();
        //gpio_set_value(GPIO_TOUCH_EN, 1);

        enable_irq(melfas_ts->irq);
        return;
    }
#endif

#if 0 //for chief
    if(buf1[0] & 0xC0) {
      msg1[0].addr = melfas_ts->client->addr;
      msg1[0].flags = 0;
      msg1[0].len = 1;
      msg1[0].buf = &start_reg;
      start_reg = 0x25;
      msg1[1].addr = melfas_ts->client->addr;
      msg1[1].flags = I2C_M_RD;
      msg1[1].len = sizeof(buf2);
      msg1[1].buf = buf2;
      
      ret = i2c_transfer(melfas_ts->client->adapter, msg1, 2);
      
      button = buf2[0]; //key:1 home key:2 menu key:3 back
      
      switch(button) {
        case 0x01 : 
        case 0x09 :
          keycode = TOUCH_MENU;
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;
        
        case 0x02 : 
        case 0x0A :
          keycode = TOUCH_HOME;
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;
        
        case 0x03 : 
        case 0x0B :
          keycode = TOUCH_BACK;
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;        

        case 0x04 : 
        case 0x0C :
          keycode = TOUCH_SEARCH;
          printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
          break;
        
        default :
          printk("[TOUCH_KEY] undefined button: 0x%02x.\n", button);
          enable_irq(melfas_ts->irq);
          return;
      }

      if(!(keycode == TOUCH_MENU /*&& get_sending_oj_event()*/))
      {
        if(button & 0x08)
          keypress = 0;
        else 
          keypress = 1;
        debugprintk(1,"[TOUCH_KEY] keycode: %4d, keypress: %4d\n", keycode, keypress); 
        //dprintk(TSP_KEY," keycode: %4d, keypress: %4d\n", keycode, keypress); 
        input_report_key(melfas_ts->input_dev, keycode, keypress);
      }
    }
#endif

//for chief
//    else
//    {
      int do_report = false;
#ifdef USES_PINCH_DIST_MT
      static int do_report2 = false; // report a second touch point
#endif

      switch(touchtype) {
        case 0x0: // Non-touched state
          melfas_ts->info.x = -1;
          melfas_ts->info.y = -1;
          melfas_ts->info.z = -1;
          melfas_ts->info.width = -1;
          z = 0;
          do_report = true;
#ifdef USES_PINCH_DIST_MT
          if(do_report2 == true) {
            z2 = 0;
          }
#endif
          break;

        case 0x1: // Single-point Touch
          melfas_ts->info.x = x;
          melfas_ts->info.y = y;
          melfas_ts->info.z = z;
          melfas_ts->info.width = width;
          do_report = true;
#ifdef USES_PINCH_DIST_MT
          do_report2 = false;
#endif
          break;

        case 0x2: // Dual-point Touch
#ifdef USES_PINCH_DIST_MT
          if(likely(melfas_ts->mt_support==true) && melfas_ts->mt_disable==false) {
            if(gesture_code) {
              melfas_ts->info.x = x;
              melfas_ts->info.y = y;
              melfas_ts->info.z = z;
              x2 = x;
              if(unlikely(width > melfas_ts->info.max_y))
                width = melfas_ts->info.max_y;
              if(y+width > melfas_ts->info.max_y)
                y2 = y - width;
              else
                y2 = y + width;
              z2 = z;
              do_report = true;
              do_report2 = true;
            }
          }
          printk(KERN_DEBUG "[TOUCH] Dual-point Touch %s!\n",melfas_ts->mt_support?"received":"ignored");
#endif
          break;

        case 0x3: // Palm Touch
          printk(KERN_DEBUG "[TOUCH] Palm Touch!\n");
          break;

        case 0x7: // Proximity
          printk(KERN_DEBUG "[TOUCH] Proximity!\n");
          break;
      }
        
      melfas_ts->info.state = touchtype;
		
      if(do_report) {
      
#ifdef USES_PINCH_DIST_MT
        if(likely(melfas_ts->mt_support == true)) {
          debugprintk(5,"[TOUCH_MT1] x1: %4d, y1: %4d, z1: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width); 
          input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, x);
          input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, y);
          input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
          //input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, width);
          input_mt_sync(melfas_ts->input_dev);
          
          if(do_report2) {
            debugprintk(5,"[TOUCH_MT2] x2: %4d, y2: %4d, z2: %4d, finger: %4d, width: %4d\n", x2, y2, z2, finger, width); 
            input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, x2);
            input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, y2);
            input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, z2);
            //input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, width);
            input_mt_sync(melfas_ts->input_dev);
          }
        }
        else {
          debugprintk(5,"[TOUCH_ST] x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width); 
          //dprintk(TSP_ABS," x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width); 
          if (x) 	input_report_abs(melfas_ts->input_dev, ABS_X, x);
          if (y)	input_report_abs(melfas_ts->input_dev, ABS_Y, y);
          
          input_report_abs(melfas_ts->input_dev, ABS_PRESSURE, z);
          input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width);
          input_report_key(melfas_ts->input_dev, BTN_TOUCH, finger); //for chief
          //input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width); //for chief
          //input_report_abs(melfas_ts->input_dev, BTN_TOUCH, finger); // for chief
        }
#else
        debugprintk(5,"[TOUCH_ABS] x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width);
        //dprintk(TSP_ABS," x: %4d, y: %4d, z: %4d, finger: %4d, width: %4d\n", x, y, z, finger, width); 
        if (x) 	input_report_abs(melfas_ts->input_dev, ABS_X, x);
        if (y)	input_report_abs(melfas_ts->input_dev, ABS_Y, y);
        
        input_report_abs(melfas_ts->input_dev, ABS_PRESSURE, z);
        input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width);
        input_report_key(melfas_ts->input_dev, BTN_TOUCH, finger);
#endif
      }
//    }

    input_sync(melfas_ts->input_dev);
  }

  enable_irq(melfas_ts->client->irq); // block irq_enable & disable, joon
}


irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(melfas_ts->client->irq); // block irq_enable & disable, joon
	//disable_irq(melfas_ts->client->irq);	
	queue_work(melfas_ts_wq, &melfas_ts->work);
	return IRQ_HANDLED;
}

void melfas_ts_upgrade(void)
{
	int ret = 0;
#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	

	ret = mcsdl_download_binary_data(melfas_ts->hw_rev);

	if(ret) 
	{
		int hw_rev, fw_ver;
		
		hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODULE_VER_REG);
		if (hw_rev < 0) 
		{
			printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		}
		else
		{
			melfas_ts->hw_rev = hw_rev;
		}
		fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_FIRMWARE_VER_REG);
		if (fw_ver < 0) {
			printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		}
		else {
			melfas_ts->fw_ver = fw_ver;
		}
		printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%x., Current F/W version: 0x%x.]\n", hw_rev, fw_ver);
	}
	else 
	{
		printk("[TOUCH] Firmware update failed.. RESET!\n");
		mcsdl_vdd_off();
		mdelay(500);
		mcsdl_vdd_on();
		mdelay(200);
	}
#endif
}

//int melfas_ts_probe(struct i2c_client *client)
int melfas_ts_probe()
{
	int ret = 0;
	uint16_t max_x=0, max_y=0;
	int fw_ver = 0;
	int hw_rev = 0;
	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] PROBE       =========");
	printk("\n====================================================\n");

	if (!i2c_check_functionality(melfas_ts->client->adapter, I2C_FUNC_SMBUS_BYTE_DATA/*I2C_FUNC_I2C*/)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	INIT_WORK(&melfas_ts->work, melfas_ts_work_func);

#if 0	
    ret = i2c_smbus_write_byte_data(melfas_ts->client, 0x01, 0x01); /* device command = reset */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
	}
#endif

	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_FIRMWARE_VER_REG);
	if (fw_ver < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		//this is not critical.. let's go on... goto err_detect_failed;
	}

	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODULE_VER_REG);
	if (hw_rev < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");;
	}

	printk(KERN_INFO "[TOUCH] Melfas  H/W version: 0x%x.\n", hw_rev);
	printk(KERN_INFO "[TOUCH] Current F/W version: 0x%x.\n", fw_ver);	

	melfas_ts->fw_ver = fw_ver;
	melfas_ts->hw_rev = hw_rev;
	//melfas_ts->irq = IRQ_TOUCH_INT;
	
	// firmware updgrade
    if(melfas_ts->fw_ver < NEW_FIRMWARE_VERSION)
		melfas_ts_upgrade();

#ifdef USES_PINCH_DIST_MT
	melfas_check_mt_support(1);
#endif

	ret = i2c_smbus_read_word_data(melfas_ts->client, MCSTS_RESOL_X_LOW_REG);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		//goto err_detect_failed;
	}
	else {
		max_x = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);
	}

	ret = i2c_smbus_read_word_data(melfas_ts->client, MCSTS_RESOL_Y_LOW_REG);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		//goto err_detect_failed;
	}
	else {
		max_y = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);
	}

	if(max_x == 0 || max_y == 0) {
		max_x = 320; //240;  for chief
		max_y = 480; //400;
	}

	melfas_ts->input_dev = input_allocate_device();
	if (melfas_ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	melfas_ts->input_dev->name = "melfas_ts_input";

	set_bit(EV_SYN, melfas_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_ts->input_dev->evbit);
	//set_bit(TOUCH_HOME, melfas_ts->input_dev->keybit);
	//set_bit(TOUCH_MENU, melfas_ts->input_dev->keybit);
	//set_bit(TOUCH_BACK, melfas_ts->input_dev->keybit);
	//set_bit(TOUCH_SEARCH, melfas_ts->input_dev->keybit);

	//melfas_ts->input_dev->keycode = melfas_ts_tk_keycode;	
	set_bit(BTN_TOUCH, melfas_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_ts->input_dev->evbit);

	melfas_ts->info.max_x = max_x;
	melfas_ts->info.max_y = max_y;

#ifdef USES_PINCH_DIST_MT
	if(melfas_ts->mt_support == true) {
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	}
	else {
		input_set_abs_params(melfas_ts->input_dev, ABS_X, 0, max_x, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_Y, 0, max_y, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(melfas_ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
	}
#else
	input_set_abs_params(melfas_ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
#endif

	printk("melfas_ts_probe: max_x %d, max_y %d\n", max_x, max_y);

	/* melfas_ts->input_dev->name = melfas_ts->keypad_info->name; */
	ret = input_register_device(melfas_ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Unable to register %s input device\n", melfas_ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	
  //ret = request_irq(melfas_ts->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, melfas_ts->input_dev->name, melfas_ts);
	ret = request_irq(melfas_ts->client->irq, melfas_ts_irq_handler, IRQF_DISABLED, "melfas_ts irq", 0);
	if(ret == 0) {
		printk(KERN_INFO "melfas_ts_probe: Start touchscreen %s \n", melfas_ts->input_dev->name);
	}
	else {
		printk("request_irq failed\n");
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_ts->early_suspend.suspend = melfas_ts_early_suspend;
	melfas_ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	return 0;

err_input_register_device_failed:
	input_free_device(melfas_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(melfas_ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}

int melfas_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
    if(melfas_ts->client->irq)
		free_irq(melfas_ts->client->irq, 0);
	input_unregister_device(melfas_ts->input_dev);
	return 0;
}

int melfas_ts_gen_touch_up(void)
{
  // report up key if needed
  if(melfas_ts->info.state == 0x1) //down state
  {
    melfas_ts->info.state = 0x0;
    int x = melfas_ts->info.x;
    int y = melfas_ts->info.y;
    int z = melfas_ts->info.z;
    int width = melfas_ts->info.width;
    printk("[TOUCH] GENERATE UP KEY x: %4d, y: %4d, z: %4d\n", x, y, z); 
    if (x) 	input_report_abs(melfas_ts->input_dev, ABS_X, x);
    if (y)	input_report_abs(melfas_ts->input_dev, ABS_Y, y);

    input_report_abs(melfas_ts->input_dev, ABS_PRESSURE, z);
    input_report_abs(melfas_ts->input_dev, ABS_TOOL_WIDTH, width);
    input_report_key(melfas_ts->input_dev, BTN_TOUCH, 0);    

    input_sync(melfas_ts->input_dev);
  }    
}

//int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
int melfas_ts_suspend(pm_message_t mesg)
{
  melfas_ts->suspended = true;
  melfas_ts_gen_touch_up();
  //disable_irq(melfas_ts->irq);
  disable_irq(melfas_ts->client->irq);
  
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  
  mcsdl_vdd_off();
  gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
  gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
  
  return 0;
}

//int melfas_ts_resume(struct i2c_client *client)
int melfas_ts_resume()
{
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

  mcsdl_vdd_on();
  gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
  gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN    
  msleep(300);
  melfas_ts->suspended = false;
  //enable_irq(melfas_ts->irq);
   enable_irq(melfas_ts->client->irq);

  return 0;
}

int tsp_preprocess_suspend(void)
{
#if 0 // blocked for now.. we will gen touch when suspend func is called
  // this function is called before kernel calls suspend functions
  // so we are going suspended if suspended==false
  if(melfas_ts->suspended == false) {  
    // fake as suspended
    melfas_ts->suspended = true;
    
    //generate and report touch event
    melfas_ts_gen_touch_up();
  }
#endif
  return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_ts_early_suspend(struct early_suspend *h)
{
	melfas_ts_suspend(PMSG_SUSPEND);
}

void melfas_ts_late_resume(struct early_suspend *h)
{
	melfas_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */


#if 0
unsigned short melfas_ts_i2c_normal[] = {(MELFAS_I2C_ADDR >> 1), I2C_CLIENT_END};
unsigned short melfas_ts_i2c_ignore[] = {1, (MELFAS_I2C_ADDR >> 1), I2C_CLIENT_END};
unsigned short melfas_ts_i2c_probe[] = {I2C_CLIENT_END};

struct i2c_client_address_data melfas_ts_addr_data = {
	.normal_i2c = melfas_ts_i2c_normal,
	.ignore = melfas_ts_i2c_ignore,
	.probe = melfas_ts_i2c_probe,
};

int melfas_ts_attach(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *c;
	int ret;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if(!c)
		return -ENOMEM;
	memset(c, 0, sizeof(struct i2c_client));
	
	strcpy(c->name, "melfas_ts");
	c->addr = addr;
	c->adapter = adap;
	c->driver = &melfas_ts_i2c;
	
	if((ret = i2c_attach_client(c)) < 0)
		goto error;

	melfas_ts->client = c;
	printk("melfas_ts_i2c is attached..\n");
error:
	return ret;
}

int melfas_ts_attach_adapter(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &melfas_ts_addr_data, melfas_ts_attach);
}

struct i2c_driver melfas_ts_i2c = {
	.driver = {
		.name	= "melfas_ts_i2c",
	},
	.id	= MELFAS_I2C_ADDR,
	.attach_adapter = melfas_ts_attach_adapter,
};
#else
int melfas_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_ts->client = client;
	i2c_set_clientdata(client, melfas_ts);
	return 0;
}

static int __devexit melfas_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
    if(melfas_ts->client->irq)
	free_irq(melfas_ts->client->irq, 0);
	input_unregister_device(melfas_ts->input_dev);
   
	melfas_ts = i2c_get_clientdata(client);
	kfree(melfas_ts);
	return 0;
}

struct i2c_device_id melfas_id[] = {
	{ "melfas_ts_i2c", 0 },
	{ }
};

struct i2c_driver melfas_ts_i2c = {
	.driver = {
		.name	= "melfas_ts_i2c",
		.owner	= THIS_MODULE,
	},
	.probe 		= melfas_i2c_probe,
	.remove		= __devexit_p(melfas_i2c_remove),
	.id_table	= melfas_id,
};
#endif

void init_hw_setting(void)
{
	int ret;

	//vreg_touch = vreg_get(NULL, "ldo19"); /* VTOUCH_2.8V */
#if 0 //LDO control routine is transferred to the CP side due to H/W's request
	ret = vreg_set_level(vreg_touch, OUT2800mV);
	if (ret) {
		printk(KERN_ERR "%s: vreg_touch set level failed (%d)\n", __func__, ret);
		return -EIO;
	}


	ret = vreg_enable(vreg_touch);
	if (ret) { 
		printk(KERN_ERR "%s: vreg_touch enable failed (%d)\n", __func__, ret);
		return -EIO;
	}
	else {
		printk(KERN_INFO "%s: vreg_touch enable success!\n", __func__);
	}

	mdelay(100);
#endif	

#if 1 // for chief
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
#endif
	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_EDGE_FALLING);

	mdelay(10);

}

struct platform_driver melfas_ts_driver =  {
	.probe	= melfas_ts_probe,
	.remove = melfas_ts_remove,
	.driver = {
		.name = "melfas-ts",
		.owner	= THIS_MODULE,
	},
};


int __init melfas_ts_init(void)
{
	int ret;
	printk("\n======================================================");
	printk("\n=======         [TOUCH SCREEN] INIT        =========");
	printk("\n====================================================\n");

	init_hw_setting();

//  sec_class = class_create(THIS_MODULE, "sec");
	ts_dev = device_create(sec_class_1, NULL, 0, NULL, "ts");

	if (IS_ERR(ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
#if 1 //for chief	//MBjclee 2011.02.11 For rookie firmware update.
	if (device_create_file(ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
#endif

	if (device_create_file(ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
#ifdef USES_PINCH_DIST_MT
	if (device_create_file(ts_dev, &dev_attr_multitouch) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_multitouch.attr.name);
#endif

	melfas_ts = kzalloc(sizeof(struct melfas_ts_driver), GFP_KERNEL);
	if(melfas_ts == NULL) {
		return -ENOMEM;
	}

	ret = i2c_add_driver(&melfas_ts_i2c);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	if(!melfas_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&melfas_ts_i2c);
		return 0;
	}
	melfas_ts_wq = create_singlethread_workqueue("melfas_ts_wq");
	if (!melfas_ts_wq)
		return -ENOMEM;

	return platform_driver_register(&melfas_ts_driver);
}

void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_i2c);
	if (melfas_ts_wq)
		destroy_workqueue(melfas_ts_wq);
}
//late_initcall(melfas_ts_init);
//module_init(melfas_ts_init);
//module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
