/* drivers/input/touchscreen/Melfas_mcs8000.c
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

#include "mcs8000_download.h"

#define SEC_TSP 
#define TSP_FACTORY_TEST

#define INPUT_INFO_REG			0x10
#define IRQ_TOUCH_INT			MSM_GPIO_TO_INT(GPIO_TOUCH_INT)
#define MELFAS_MAX_TOUCH		5
#define MELFAS_MAX_KEYS			5
#define TS_READ_REGS_LEN		30
#define TS_READ_START_ADDR		0x10
#define PRESS_KEY				1
#define RELEASE_KEY				0
#define DEBUG_PRINT				0
#define QT_ATCOM_TEST


#define MCSTS_WINDOW_VER_REG    0x65
#define MELFAS_WINDOW_55T		0x41
#define MELFAS_WINDOW_70T		0x42

#if defined(CONFIG_MACH_ANCORA_TMO)
#define NEW_FIRMWARE_VERSION 	 0x13
#define NEW_FIRMWARE_VERSION_55T 0x06
#else
#define NEW_FIRMWARE_VERSION 	 0x14
#define NEW_FIRMWARE_VERSION_GFF NEW_FIRMWARE_VERSION
#define NEW_FIRMWARE_VERSION_G2  0x15
#endif
#define TS_READ_VERSION_ADDR	0x63

static int debug_level = 5; 
static struct vreg *vreg_ldo2;

#define debugprintk(level,x...)  if(debug_level>=level) printk(x)

int melfas_mcs8000_ts_suspend(pm_message_t mesg);
int melfas_mcs8000_ts_resume();
extern int mcsdl_download_binary_data(void);
extern int mcsdl_ISC_download_binary_data(void);

#if defined(CONFIG_MACH_ANCORA_TMO)
extern int mcsdl_download_binary_data_55T(void);
#endif
#if defined(CONFIG_MACH_ANCORA) //#if !defined(CONFIG_MACH_ANCORA_TMO)  
extern int mcsdl_download_binary_data_G2(void);
extern int mcsdl_ISC_download_binary_data_G2(void);
#endif

extern struct class *sec_class;
extern int board_hw_revision;

enum {
	TKEY_LED_OFF,
	TKEY_LED_ON,
	TKEY_LED_SUSPEND,
	TKEY_LED_RESUME,
	TKEY_LED_FORCEDOFF
};
static unsigned int led_state = TKEY_LED_OFF;

static int touchkey_status[MELFAS_MAX_KEYS]={0,};
static int preKeyID = 0;

static const int touchkey_keycodes[] = {
			KEY_MENU,
			KEY_HOME,
			KEY_BACK,
			KEY_SEARCH,
};

static const int touchkey_keycodes_rev04[] = {
			KEY_MENU,
			KEY_BACK,
};

struct muti_touch_info
{
    int state;
    int strength;
    int width;
    int posX;
    int posY;
};

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

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

struct mcs8000_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
	int window_ver;
	struct input_info info;
	int suspended;
	struct early_suspend	early_suspend;
};

struct mcs8000_ts_driver *melfas_mcs8000_ts = NULL;
struct i2c_driver mcs8000_ts_i2c;
struct workqueue_struct *melfas_mcs8000_ts_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs8000_ts_early_suspend(struct early_suspend *h);
void melfas_mcs8000_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef SEC_TSP
static int testmode = 0;
#endif
#define TOUCH_HOME	KEY_HOME
#define TOUCH_MENU	KEY_MENU
#define TOUCH_BACK	KEY_BACK
#define TOUCH_SEARCH  KEY_SEARCH

int melfas_ts_tk_keycode[] =
{ TOUCH_MENU, TOUCH_HOME, TOUCH_BACK, TOUCH_SEARCH, };

struct device *mcs8000_ts_dev;

#ifdef SEC_TSP
struct device *sec_touchscreen;
#define TS_READ_EXCITING_CH_ADDR	0x2E
#define TS_READ_SENSING_CH_ADDR	    0x2F
#define TS_WRITE_REGS_LEN		    16
#define RMI_ADDR_TESTMODE           0xA0
#define UNIVERSAL_CMD_RESULT_SIZE   0xAE
#define UNIVERSAL_CMD_RESULT        0xAF
#ifdef TSP_FACTORY_TEST
struct device *qt602240_noise_test;
#endif
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);
#endif

//#define AUTO_POWER_ON_OFF_FLAG //for auto power-onoff test 2011-07-12 hc.hyun
#ifdef AUTO_POWER_ON_OFF_FLAG
static struct timer_list poweroff_touch_timer;
static void poweroff_touch_timer_handler2(unsigned long data)
{
	/* click "yes" on second pop up menu */
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, 124);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, 506);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 40);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_WIDTH_MAJOR, 4);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 0);
	input_mt_sync(melfas_mcs8000_ts->input_dev);
	input_sync(melfas_mcs8000_ts->input_dev);

	mdelay(100);

	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, 124);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, 506);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_WIDTH_MAJOR, 4);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 0);

	input_mt_sync(melfas_mcs8000_ts->input_dev);
	input_sync(melfas_mcs8000_ts->input_dev);

}

static void poweroff_touch_timer_handler(unsigned long data)
{
	/* click "power off" on first pop up menu */
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, 282);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, 640);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 40);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_WIDTH_MAJOR, 4);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 0);
	input_mt_sync(melfas_mcs8000_ts->input_dev);
	input_sync(melfas_mcs8000_ts->input_dev);

	mdelay(100);

	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, 282);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, 640);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_WIDTH_MAJOR, 4);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 0);
	input_mt_sync(melfas_mcs8000_ts->input_dev);
	input_sync(melfas_mcs8000_ts->input_dev);

	poweroff_touch_timer.function = poweroff_touch_timer_handler2;
	mod_timer(&poweroff_touch_timer,jiffies + 5*HZ);
}
#endif

void mcsdl_vdd_on(void)
{ 
	gpio_set_value( TSP_LDO_ON, 1 ); 
	mdelay(10);
}

void mcsdl_vdd_off(void)
{
	gpio_set_value( TSP_LDO_ON, 0 ); 
	mdelay(10);
}

static int melfas_mcs8000_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{

	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

    //printk("%s p_client->addr[[ %x  ]\n", __func__, p_client->addr);

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

static int melfas_mcs8000_read_version(void)
{
    int ret;
	u8 buf[2] = {0,};

    ret = melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_MODULE_VER_REG, buf, 2);
	if (ret == 0)
	{
		melfas_mcs8000_ts->hw_rev = buf[0];
		melfas_mcs8000_ts->fw_ver = buf[1];

		if( buf[0] == 0x00 && buf[1] == 0x00 ) {
			melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_VERSION_ADDR, buf, 2);

			melfas_mcs8000_ts->hw_rev = buf[0];
			melfas_mcs8000_ts->fw_ver = buf[1];
		}
		
		printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
	}
	else
	{
		melfas_mcs8000_ts->hw_rev = 0;
		melfas_mcs8000_ts->fw_ver = 0;
		
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}

    ret = melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_WINDOW_VER_REG, buf, 1);
	if (ret == 0)
	{
		melfas_mcs8000_ts->window_ver=buf[0];
		printk("%s :Window Ver : %d\n", __func__, buf[0]);
	}
	else
	{
		melfas_mcs8000_ts->window_ver=0;
		printk("%s :Window Ver : error\n", __func__);
	}	

    return ret;
}

static void melfas_mcs8000_read_resolution(void)
{
	
	uint16_t max_x=0, max_y=0;	

	u8 buf[3] = {0,};
	
	if(0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_RESOL_HIGH_REG , buf, 3)){

		printk("%s :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,buf[0],buf[1],buf[2]);

		if((buf[0] == 0)||(buf[0] == 0)||(buf[0] == 0)){
			melfas_mcs8000_ts->info.max_x = 320;
			melfas_mcs8000_ts->info.max_y = 480;
			
			printk("%s : Can't find Resolution!\n", __func__);
			}
		
		else{
			max_x = buf[1] | ((uint16_t)(buf[0] & 0x0f) << 8); 
			max_y = buf[2] | (((uint16_t)(buf[0] & 0xf0) >> 4) << 8); 
			melfas_mcs8000_ts->info.max_x = max_x;
		    melfas_mcs8000_ts->info.max_y = max_y;

			printk("%s :max_x: %d, max_y: %d\n", __func__, melfas_mcs8000_ts->info.max_x, melfas_mcs8000_ts->info.max_y);
			}
		}

	else
	{
		melfas_mcs8000_ts->info.max_x = 320;
		melfas_mcs8000_ts->info.max_y = 480;
		
		printk("%s : Can't find Resolution!\n", __func__);
	}
}

static ssize_t registers_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 buf1[2] = {0,};
	u8 buf2[2] = {0,};

	int status, mode_ctl, hw_rev, fw_ver;

	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_STATUS_REG, buf1, 2))
	{
		status = buf1[0];
		mode_ctl = buf1[1];	 
	}
	else
	{
		printk("%s : Can't find status, mode_ctl!\n", __func__); 
	}

	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_MODULE_VER_REG, buf2, 2))
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

static ssize_t registers_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		
	    ret = i2c_smbus_write_byte_data(melfas_mcs8000_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW"); 
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store_mcs8000(
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
		printk("[TOUCH] disable.\n");
	}
	
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
    mcsdl_vdd_off();
		mdelay(500);
    mcsdl_vdd_on();
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}


static ssize_t firmware_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	u8 buf1[2] = {0,};
	int hw_rev, fw_ver;


	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_MODULE_VER_REG, buf1, 2))
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


static ssize_t firmware_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{

    // Unused    
	return size;
}



static ssize_t debug_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d", debug_level);
}

static ssize_t debug_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf[0]>'0' && buf[0]<='9') {
		debug_level = buf[0] - '0';
	}

	return size;
}

#define N_SENS_COUNT        3
static uint8_t  tkey_sensitivity_read(void)
{
    uint8_t     tkey_sens, sens_min, sens_max ;
    int         n, sens_sum ;

    sens_min = 0xFF ;   /* Document errata!!! */
    sens_max = 0x00 ;
    for (sens_sum = n = 0 ; n < 3 ; n++)
    {
        if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_RT_STRNTH_REG, &tkey_sens, 1) != 0)
        {
            printk(KERN_ERR "tkey_sensitivity_read read fail!!!\n") ;
            break ;
        }
        // tkey_sens &= 0x1F ; Document errata!!!
        printk(KERN_DEBUG "tkey_sens : %d\n", tkey_sens) ;
        sens_sum += tkey_sens ;
        if (sens_min > tkey_sens)
            sens_min = tkey_sens ;
        if (sens_max < tkey_sens)
            sens_max = tkey_sens ;
    }
    tkey_sens = sens_sum - sens_min - sens_max ;    /* median */
    printk(KERN_ERR "tkey_sensitivity_read key(%d)\n", tkey_sens) ;

    return tkey_sens ;
}

/* Touch key Sensitivity ******************************************************/
static ssize_t tkey_sensitivity_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t     tkey_sens, tkey_touched ;

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_INFORM_REG, &tkey_touched, 1) != 0 /* Key State */
            || (tkey_touched & 0x08) == 0) /* no touched */
        return sprintf(buf, "-1") ;

    /* else touched */
    msleep(100) ;
    tkey_sens = tkey_sensitivity_read() ;
	return sprintf(buf, "%d", tkey_sens);
}

/* Touch key Sensitivity ******************************************************/
static ssize_t tkey_sens_pline_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t     tkey_sens ;
    uint8_t     tkey_touched ;
    uint8_t     noise_thd = 0 ;
    int         n ;
    char        str[20] ;

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_THD_REG, &noise_thd, 1) != 0)
    {
        printk(KERN_ERR "[tkey_sens_pline_show] Fail to read noise_threshold<0x3F>!\n") ;
        return sprintf(buf, "0,0,0,0") ;
    }

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_INFORM_REG, &tkey_touched, 1) != 0 /* Key State */
            || (tkey_touched & 0x08) == 0) /* no touched */
    {
        printk(KERN_ERR "[tkey_sens_pline_show] Fail to read key_state<0x1C> or untouched(%d)!\n",
                (tkey_touched & 0x08)) ;
        return sprintf(buf, "%d,%d,%d,%d", noise_thd, noise_thd, noise_thd, noise_thd) ;
    }

    tkey_touched &= 0x07 ;  /* clear touch state bit (0x08) */
    printk(KERN_INFO "[tkey_sens_pline_show] touched =0x%02x\n", tkey_touched) ;
    msleep(100) ;
    tkey_sens = tkey_sensitivity_read() ;

    /* tkey_touched = (1, 2, 3, 4) */
    sprintf(str, "%d", tkey_touched == 1 ? tkey_sens : noise_thd) ;
    printk(KERN_INFO "%s", str) ;
    strcpy(buf, str) ;
    for (n = 2 ; n <= 4 ; n++)
    {
        sprintf(str, ",%d", tkey_touched == n ? tkey_sens : noise_thd) ;
        printk("%s", str) ;
	    strcat(buf, str);
    }
    printk("\n") ;

	return strlen(buf) ;
}

/* Noise Threshold : Touch Key default Sensitivity ****************************/
static ssize_t tkey_noise_thd_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   noise_thd ;

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_THD_REG, &noise_thd, 1) == 0)
    {
        noise_thd &= 0x1F ;
        printk("TKEY_THRESHOLD : %02d\n", noise_thd) ;
        return sprintf(buf, "%d", noise_thd) ;
    }
    /* else */
    printk(KERN_ERR "TKEY_THRESHOLD : error\n") ;
    return sprintf(buf, "0") ;  // TODO:
}

static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 data = 0x10;
	int rc;
	
	if (sscanf(buf, "%d\n", &data) == 1)
		printk("%d\n", data);

#if DEBUG_PRINT
    printk("touch_led_control : %d\n", data);
#endif

    if(data == 1 && led_state == TKEY_LED_OFF)
	{
		rc = vreg_enable(vreg_ldo2);

		if (rc) {
			pr_err("%s: LDO2 vreg enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		led_state = TKEY_LED_ON;
	}
	else if(data == 2 && led_state == TKEY_LED_ON)
	{
		rc = vreg_disable(vreg_ldo2);

		if (rc) {
			pr_err("%s: LDO2 vreg disable failed (%d)\n",
			       __func__, rc);
			return rc;
		}
        
		led_state = TKEY_LED_OFF;
	}

	return size;
}
#ifdef QT_ATCOM_TEST

static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{    
    return sprintf(buf, "%d\n", 32);
}

static ssize_t key_threshold_store(
        struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
	/* threshold change code*/
    return 0;
}
#endif
static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show_mcs8000, gpio_store_mcs8000);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show_mcs8000, registers_store_mcs8000);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR |S_IWGRP , firmware_show_mcs8000, firmware_store_mcs8000);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show_mcs8000, debug_store_mcs8000);
static DEVICE_ATTR(tkey_sensitivity, S_IRUGO, tkey_sensitivity_show_mcs8000, NULL);
static DEVICE_ATTR(tkey_sens_pline, S_IRUGO, tkey_sens_pline_show_mcs8000, NULL);
static DEVICE_ATTR(tkey_noise_thd, S_IRUGO, tkey_noise_thd_show_mcs8000, NULL);
static DEVICE_ATTR(brightness, 0666, NULL, touch_led_control);
#ifdef QT_ATCOM_TEST

static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);
#endif

#ifdef QT_ATCOM_TEST
struct device *qt602240_atcom_test;
//struct work_struct qt_touch_update_work;

unsigned int qt_firm_status_data=0;


void firmware_update()
{
	int ret;

	printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", melfas_mcs8000_ts->hw_rev);
	printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs8000_ts->fw_ver);

	disable_irq(melfas_mcs8000_ts->client->irq);
	local_irq_disable();

	printk("[F/W D/L] Entry gpio_tlmm_config\n");
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TSP_LDO_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	
	printk("[F/W D/L] Entry mcsdl_download_binary_data\n");

#if defined(CONFIG_MACH_ANCORA_TMO)
    if(melfas_mcs8000_ts->window_ver==MELFAS_WINDOW_55T)
    {
		ret = mcsdl_download_binary_data_55T(); 
    }	
	else//if(melfas_mcs8000_ts->window_ver==MELFAS_WINDOW_70T)
    {	
		ret = mcsdl_download_binary_data(); 
    }		
#else
    if(melfas_mcs8000_ts->hw_rev == 0x03) // GFF
    {	
		ret = mcsdl_download_binary_data(); 
    } 
    else //if(melfas_mcs8000_ts->hw_rev == 0x50) // G2
    {
		ret = mcsdl_download_binary_data_G2();    
    }
#endif
	
	local_irq_enable();
	enable_irq(melfas_mcs8000_ts->client->irq);
	
	melfas_mcs8000_read_version();

    printk("[TSP] firmware update result. ret = %d\n", ret);
    
	if(ret > 0){
		if (melfas_mcs8000_ts->hw_rev < 0) {
			printk(KERN_ERR "i2c_transfer failed\n");;
		}
		
		if (melfas_mcs8000_ts->fw_ver < 0) {
			printk(KERN_ERR "i2c_transfer failed\n");
		}
		
		printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs8000_ts->hw_rev, melfas_mcs8000_ts->fw_ver);
		qt_firm_status_data=2;			
	}
	else {
		qt_firm_status_data=3;
		printk("[TOUCH] Firmware update failed.. %d RESET!\n", ret);
		mcsdl_vdd_off();
		mdelay(500);
		mcsdl_vdd_on();
		mdelay(200);
	}
}

void firmware_update_manual()
{
	int ret;

    printk("[TOUCH] firmware_update_manual \n");
	printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", melfas_mcs8000_ts->hw_rev);
	printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs8000_ts->fw_ver);

	disable_irq(melfas_mcs8000_ts->client->irq);
	local_irq_disable();

#if defined(CONFIG_MACH_ANCORA) //#if !defined(CONFIG_MACH_ANCORA_TMO)  
    if (melfas_mcs8000_ts->hw_rev == 0x50)
    	ret = mcsdl_ISC_download_binary_data_G2();    
    else 
#endif
        ret = mcsdl_ISC_download_binary_data();
    
	local_irq_enable();
	enable_irq(melfas_mcs8000_ts->client->irq);
	
	melfas_mcs8000_read_version();

    printk("[TSP] firmware update result. ret = %d\n", ret);
    
	if(ret > 0){
		if (melfas_mcs8000_ts->hw_rev < 0) {
			printk(KERN_ERR "i2c_transfer failed\n");
		}
		
		if (melfas_mcs8000_ts->fw_ver < 0) {
			printk(KERN_ERR "i2c_transfer failed\n");
		}
		
		printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs8000_ts->hw_rev, melfas_mcs8000_ts->fw_ver);
		qt_firm_status_data=2;			
	}
	else {
		qt_firm_status_data=3;
		printk("[TOUCH] Firmware update failed.. %d RESET!\n", ret);
		mcsdl_vdd_off();
		mdelay(500);
		mcsdl_vdd_on();
		mdelay(200);
	}
}

static ssize_t set_qt_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;

    printk(" %s : START \n",__func__); 
    qt_firm_status_data=2;    //start firmware updating
    
	/* firmware update code */	 
    qt_firm_status_data=1;
    firmware_update_manual();
       
    if(qt_firm_status_data == 3) {
        count = sprintf(buf,"FAIL\n");
    }
    else
        count = sprintf(buf,"OK\n");
    
    return count;
}

static ssize_t set_qt_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{    
#if defined(CONFIG_MACH_ANCORA)    
    if(melfas_mcs8000_ts->hw_rev == 0x03) // GFF TSP
	{	
        return sprintf(buf, "0x%02x\n", NEW_FIRMWARE_VERSION_GFF); // kernel source version    
    }  
    else if(melfas_mcs8000_ts->hw_rev == 0x50) // G2 TSP
	{	
        return sprintf(buf, "0x%02x\n", NEW_FIRMWARE_VERSION_G2); // kernel source version    
    }  
#else
	if(melfas_mcs8000_ts->window_ver==MELFAS_WINDOW_55T)	// Window panel 0.55T
    {
        return sprintf(buf, "0x%02x\n", NEW_FIRMWARE_VERSION_55T);// kernel source version
    }  
#endif     
    else // Window panel 0.7T
	{	
        return sprintf(buf, "0x%02x\n", NEW_FIRMWARE_VERSION);// kernel source version    
    }         
}

static ssize_t set_qt_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0x%02x\n", melfas_mcs8000_ts->fw_ver);// IC firmware version
}

static ssize_t set_qt_hw_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", melfas_mcs8000_ts->hw_rev);// IC hw rev
}

static ssize_t set_qt_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;

    if(qt_firm_status_data == 1) {
        count = sprintf(buf,"Downloading\n");
    }
    else if(qt_firm_status_data == 2) {
        count = sprintf(buf,"PASS\n");
    }
    else if(qt_firm_status_data == 3) {
        count = sprintf(buf,"FAIL\n");
    }
    else
        count = sprintf(buf,"PASS\n");

    return count;
}

static DEVICE_ATTR(set_qt_update, 0664, set_qt_update_show, NULL);
static DEVICE_ATTR(set_qt_firm_version, 0664, set_qt_firm_version_show, NULL);
static DEVICE_ATTR(set_qt_firm_status, 0664, set_qt_firm_status_show, NULL);
static DEVICE_ATTR(set_qt_firm_version_read, 0664, set_qt_firm_version_read_show, NULL);
static DEVICE_ATTR(set_qt_hw_version_read, 0664, set_qt_hw_version_read_show, NULL);

#endif

void melfas_mcs8000_upgrade(void)
{
// unused
}

static void melfas_ts_work_func(struct work_struct *work)
{
//    struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
    int ret = 0, i;
    uint8_t buf[30];
    uint8_t output_rawbuf[100];
    int touchNumber = 0, touchPosition = 0, posX = 0, posY = 0, width = 0, strength = 0;
    int keyEvent = 0, keyState = 0, keyID = 0, keystrength = 0;

#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_work_func\n");

    if (melfas_mcs8000_ts == NULL)
        printk(KERN_ERR "melfas_ts_work_func : TS NULL\n");
#endif


    //if(testmode) return IRQ_HANDLED;
    
    /**
    Simple send transaction:
    	S Addr Wr [A]  Data [A] Data [A] ... [A] Data [A] P
    Simple recv transaction:
    	S Addr Rd [A]  [Data] A [Data] A ... A [Data] NA P
    */

    buf[0] = TS_READ_START_ADDR;
    for (i = 0; i < 10; i++)
    {
        ret = i2c_master_send(melfas_mcs8000_ts->client, buf, 1);
#if DEBUG_PRINT
        printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
#endif
        if (ret >= 0)
        {
            ret = i2c_master_recv(melfas_mcs8000_ts->client, buf, TS_READ_REGS_LEN);
#if DEBUG_PRINT
            printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);
#endif
            for(i=0;i<TS_READ_REGS_LEN;i++)
                sprintf(output_rawbuf+(i*3),"%02X ",buf[i]);
#if DEBUG_PRINT			
            printk(KERN_ERR "melfas_ts_work_func: raw[%s]\n",output_rawbuf);
#endif
            if (ret >= 0)
                break; // i2c success
        }
    }


    if (ret < 0)
    {
        printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
        enable_irq(melfas_mcs8000_ts->client->irq);
        return ;
    }
    else // Five Multi Touch Interface
    {
    	if(buf[0] == 0x0f) { // ESD check
	  		mcsdl_vdd_off();
			mdelay(500);
			mcsdl_vdd_on();
			mdelay(200);

			printk("[TSP] TSP reset by ESD state enabled");
            enable_irq(melfas_mcs8000_ts->client->irq);
			return;
   		}
			
        touchNumber = buf[0] & 0x0F;
        touchPosition = buf[1] & 0x1F;

        keyID = buf[5*MELFAS_MAX_TOUCH + 2] & 0x07;
        keyState = (buf[5*MELFAS_MAX_TOUCH + 2] >> 3) & 0x01;
        keyEvent = (buf[5*MELFAS_MAX_TOUCH + 2] >> 4) & 0x01;
        keystrength = (buf[5*MELFAS_MAX_TOUCH + 3]);

        for (i = 0; i < MELFAS_MAX_TOUCH; i++)
        {
            g_Mtouch_info[i].posX = ((buf[2 + 5*i] >> 4)   << 8) + buf[3 + 5*i];
            g_Mtouch_info[i].posY = ((buf[2 + 5*i] & 0x0F) << 8) + buf[4 + 5*i];
            g_Mtouch_info[i].width = buf[5 + 5*i];
            g_Mtouch_info[i].strength = buf[6 + 5*i];

            if (g_Mtouch_info[i].width != 0)
            {
                g_Mtouch_info[i].state = 1;
				keyEvent = 0; // during touch event, ignore key event
            }
            else
            {
                g_Mtouch_info[i].state = 0;
            }
        }
        
        if (touchNumber > MELFAS_MAX_TOUCH)
        {
            enable_irq(melfas_mcs8000_ts->client->irq);
            return;
        }

        for (i = 0; i < MELFAS_MAX_TOUCH; i++)
        {
            if ((g_Mtouch_info[i].posX == 0) || (g_Mtouch_info[i].posY == 0))
                continue;

            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, i);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
            input_mt_sync(melfas_mcs8000_ts->input_dev);
            
#if DEBUG_PRINT
            printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d, x: %d, y: %d, z: %d w: %d\n",
                   i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
#endif
        }
#if DEBUG_PRINT
        printk("melfas_ts_work_func: keyEvent : %d, touchkey_status[%d] : %d, keyState: %d\n", keyEvent, keyID, touchkey_status[keyID], keyState);
#endif
        if ( (keyID != preKeyID) && (touchkey_status[preKeyID] != 0)) 
        {
#if defined(CONFIG_MACH_ANCORA_TMO)
        	input_report_key(melfas_mcs8000_ts->input_dev, touchkey_keycodes[preKeyID-1], 0);
#else
        	input_report_key(melfas_mcs8000_ts->input_dev, touchkey_keycodes_rev04[preKeyID-1], 0);
#endif
#if DEBUG_PRINT            
            printk("melfas_ts_work_func: Tkey forced released keyID : %d, keyState: %d\n", preKeyID, touchkey_status[preKeyID]);
#endif
			touchkey_status[preKeyID] = 0;            
		}     
        
        if (keyEvent && (touchkey_status[keyID] != keyState))
        {
#if defined(CONFIG_MACH_ANCORA_TMO)

	       	input_report_key(melfas_mcs8000_ts->input_dev, touchkey_keycodes[keyID-1], keyState);
			touchkey_status[keyID] = keyState;
            preKeyID = keyID;
#else
        	if(board_hw_revision < 4)
       		{
	        	input_report_key(melfas_mcs8000_ts->input_dev, touchkey_keycodes[keyID-1], keyState);
				touchkey_status[keyID] = keyState;
                preKeyID = keyID;
       		}
			else if(board_hw_revision >= 4)
			{
	        	input_report_key(melfas_mcs8000_ts->input_dev, touchkey_keycodes_rev04[keyID-1], keyState);
				touchkey_status[keyID] = keyState;
                preKeyID = keyID;
			}
#endif
#if DEBUG_PRINT
            printk(KERN_ERR "[TKEY] ID : %d, keyState: %d\n", keyID, keyState);
#endif
        }

        input_sync(melfas_mcs8000_ts->input_dev);
    }

    enable_irq(melfas_mcs8000_ts->client->irq);
}


void melfas_mcs8000_ts_work_func(struct work_struct *work)
{
// Don't used
}

irqreturn_t melfas_mcs8000_ts_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(melfas_mcs8000_ts->client->irq);
	queue_work(melfas_mcs8000_ts_wq, &melfas_mcs8000_ts->work);
	return IRQ_HANDLED;
}
#ifdef SEC_TSP
static ssize_t set_tsp_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf);

static int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;

	if  (i2c_transfer(adapter, msg, 2) == 2)
		return 0;
	else
		return -EIO;

}


static int melfas_i2c_write(struct i2c_client *client, char *buf, int length)
{
	int i;
	char data[TS_WRITE_REGS_LEN];

	if (length > TS_WRITE_REGS_LEN) {
		pr_err("[TSP] size error - %s\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < length; i++)
		data[i] = *buf++;

	i = i2c_master_send(client, (char *)data, length);

	if (i == length)
		return length;
	else
		return -EIO;
}

#ifdef TSP_FACTORY_TEST
static bool debug_print = true;
static u16 inspection_data[176] = { 0, };
static u16 lntensity_data[176] = { 0, };
static u16 CmDelta_data[176] = { 0, };
#if defined(CONFIG_MACH_ANCORA_TMO)
static u16 inspection_data_max[176] = 
  { 3417, 3328, 3289, 3266, 3241, 3224, 3209, 3202, 3192, 3186, 3184, 3181, 3179, 3179, 3179, 3176, 
     3418, 3326, 3289, 3262, 3235, 3213, 3198, 3184, 3172, 3164, 3159, 3154, 3152, 3150, 3150, 3159, 
     3419, 3335, 3296, 3272, 3248, 3231, 3216, 3208, 3200, 3193, 3191, 3189, 3186, 3186, 3186, 3184, 
     3420, 3333, 3296, 3272, 3248, 3230, 3216, 3207, 3199, 3194, 3191, 3189, 3187, 3186, 3186, 3184, 
     3420, 3336, 3297, 3271, 3242, 3221, 3205, 3192, 3180, 3172, 3167, 3163, 3161, 3159, 3158, 3167, 
     3418, 3335, 3298, 3273, 3250, 3231, 3217, 3209, 3201, 3196, 3193, 3192, 3189, 3189, 3188, 3186, 
     3420, 3336, 3297, 3271, 3242, 3221, 3205, 3192, 3180, 3172, 3168, 3163, 3161, 3159, 3158, 3170, 
     3421, 3335, 3298, 3273, 3250, 3231, 3217, 3208, 3202, 3197, 3194, 3193, 3190, 3189, 3189, 3187, 
     3417, 3332, 3294, 3268, 3246, 3228, 3214, 3204, 3199, 3194, 3191, 3189, 3187, 3186, 3186, 3184, 
     3420, 3330, 3292, 3266, 3238, 3216, 3201, 3188, 3177, 3170, 3165, 3161, 3159, 3157, 3156, 3166, 
     3415, 3326, 3290, 3261, 3241, 3224, 3210, 3199, 3196, 3190, 3188, 3186, 3183, 3182, 3183, 3181 };

static u16 inspection_data_min[176] = 
  { 2278, 2219, 2193, 2177, 2161, 2149, 2139, 2135, 2128, 2124, 2122, 2120, 2119, 2119, 2119, 2117, 
     2279, 2217, 2193, 2175, 2156, 2142, 2132, 2123, 2115, 2109, 2106, 2103, 2102, 2100, 2100, 2106, 
     2280, 2223, 2198, 2182, 2166, 2154, 2144, 2139, 2133, 2129, 2127, 2126, 2124, 2124, 2124, 2122, 
     2280, 2222, 2198, 2181, 2166, 2153, 2144, 2138, 2133, 2129, 2128, 2126, 2124, 2124, 2124, 2122, 
     2280, 2224, 2198, 2180, 2161, 2148, 2137, 2128, 2120, 2114, 2112, 2108, 2107, 2106, 2105, 2112, 
     2279, 2223, 2199, 2182, 2167, 2154, 2145, 2139, 2134, 2131, 2129, 2128, 2126, 2126, 2126, 2124, 
     2280, 2224, 2198, 2181, 2161, 2147, 2137, 2128, 2120, 2115, 2112, 2109, 2108, 2106, 2106, 2113, 
     2280, 2223, 2199, 2182, 2167, 2154, 2145, 2139, 2134, 2132, 2129, 2129, 2127, 2126, 2126, 2124, 
     2278, 2221, 2196, 2178, 2164, 2152, 2143, 2136, 2133, 2129, 2127, 2126, 2124, 2124, 2124, 2122, 
     2280, 2220, 2194, 2177, 2159, 2144, 2134, 2125, 2118, 2113, 2110, 2107, 2106, 2105, 2104, 2110, 
     2277, 2217, 2193, 2174, 2161, 2149, 2140, 2133, 2131, 2127, 2125, 2124, 2122, 2121, 2122, 2121};

#else
static u16 inspection_data_max[176] = 
  { 3410, 3322, 3284, 3264, 3239, 3224, 3210, 3203, 3193, 3189, 3185, 3184, 3183, 3182, 3181, 3178, 
    3415, 3317, 3288, 3258, 3233, 3214, 3198, 3185, 3174, 3168, 3162, 3160, 3157, 3156, 3155, 3166, 
    3414, 3325, 3292, 3268, 3243, 3228, 3215, 3207, 3198, 3194, 3191, 3189, 3188, 3187, 3186, 3183, 
    3413, 3321, 3292, 3265, 3243, 3227, 3215, 3205, 3199, 3195, 3191, 3189, 3188, 3187, 3187, 3184, 
    3408, 3321, 3289, 3265, 3241, 3226, 3213, 3204, 3197, 3193, 3190, 3188, 3187, 3186, 3186, 3182, 
    3408, 3320, 3291, 3264, 3241, 3226, 3214, 3204, 3198, 3194, 3191, 3188, 3188, 3187, 3187, 3183, 
    3408, 3322, 3289, 3265, 3242, 3226, 3214, 3205, 3198, 3194, 3191, 3189, 3189, 3188, 3187, 3184, 
    3412, 3321, 3290, 3264, 3242, 3226, 3214, 3205, 3200, 3195, 3192, 3190, 3189, 3189, 3188, 3185, 
    3410, 3322, 3289, 3265, 3243, 3226, 3215, 3206, 3201, 3197, 3194, 3191, 3191, 3190, 3189, 3186, 
    3417, 3321, 3290, 3260, 3236, 3217, 3201, 3188, 3181, 3174, 3169, 3166, 3163, 3162, 3161, 3173, 
    3403, 3310, 3277, 3254, 3232, 3216, 3205, 3196, 3193, 3188, 3185, 3182, 3182, 3180, 3181, 3179 };

static u16 inspection_data_min[176] = 
  { 2273, 2215, 2189, 2176, 2159, 2149, 2140, 2135, 2129, 2126, 2124, 2123, 2122, 2121, 2121, 2119, 
    2277, 2212, 2192, 2172, 2156, 2143, 2132, 2123, 2116, 2112, 2108, 2106, 2104, 2104, 2103, 2110, 
    2276, 2217, 2194, 2178, 2162, 2152, 2143, 2138, 2132, 2129, 2127, 2126, 2125, 2124, 2124, 2122, 
    2275, 2214, 2195, 2177, 2162, 2151, 2143, 2137, 2132, 2130, 2128, 2126, 2126, 2125, 2125, 2123, 
    2272, 2214, 2193, 2177, 2161, 2150, 2142, 2136, 2131, 2129, 2127, 2125, 2125, 2124, 2124, 2122, 
    2272, 2213, 2194, 2176, 2161, 2150, 2142, 2136, 2132, 2129, 2127, 2125, 2125, 2125, 2124, 2122, 
    2272, 2214, 2193, 2177, 2161, 2151, 2143, 2137, 2132, 2130, 2128, 2126, 2126, 2125, 2125, 2123, 
    2274, 2214, 2194, 2176, 2161, 2151, 2143, 2137, 2133, 2130, 2128, 2127, 2126, 2126, 2126, 2124, 
    2273, 2215, 2193, 2177, 2162, 2151, 2143, 2138, 2134, 2131, 2129, 2127, 2127, 2127, 2126, 2124, 
    2278, 2214, 2194, 2174, 2157, 2145, 2134, 2126, 2121, 2116, 2112, 2111, 2109, 2108, 2108, 2115, 
    2269, 2207, 2185, 2169, 2155, 2144, 2136, 2131, 2129, 2125, 2123, 2122, 2121, 2120, 2121, 2119 };
#endif
static ssize_t set_tsp_module_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;	

	ret = melfas_mcs8000_ts_resume();
	
	if (ret  = 0)
		*buf = '1';
	else
		*buf = '0';

	return 0;
}

static ssize_t set_tsp_module_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;

	ret = melfas_mcs8000_ts_suspend(PMSG_SUSPEND);

	if (ret  = 0)
		*buf = '1';
	else
		*buf = '0';

	return 0;
}

static int check_debug_data(struct mcs8000_ts_driver *ts)
{
    u8 read_buf[2] = {0,};
    u8 write_command[2];
    u8 read_data_buf[50] = {0,};
    int exciting_ch, sensing_ch;
    int ret = 0;
    int read_data_len = 0;
    int i, j, status;
    int gpio = GPIO_TOUCH_INT;
    

    printk("%s : enter \n", __func__);

	disable_irq(ts->client->irq);
    
    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_EXCITING_CH_ADDR, read_buf, 1)) {  
        exciting_ch = read_buf[0]; // 16  
        printk("Excitin Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read EXCITING CH \n");
    }

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_SENSING_CH_ADDR, read_buf, 1)) {  
        sensing_ch = read_buf[0]; // 11
        printk("Sensing Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read SENSING CH \n");
    }
    
    /* Read Reference Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x02;  // Reference Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);    

    for(i = 0; i < sensing_ch; i++ ) {
        
        printk("sensing_ch is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}

        read_data_len = exciting_ch * 2;  // 32 : 2 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf, read_data_len)) {        
            printk("[0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x ", read_data_buf[0], read_data_buf[1], read_data_buf[2]);
            printk("[3] = 0x%02x ... [%d] = 0x%02x \n", read_data_buf[3], read_data_len-1, read_data_buf[read_data_len-1]);
        }
        else {
            printk("can't read Reference Data \n");
        }
        
        for( j = 0; j <  (read_data_len/2) ; j++) {
            inspection_data[(i * exciting_ch) + j] = (read_data_buf[j*2] << 8) | read_data_buf[j*2+1];
            
        if ((inspection_data[(i * exciting_ch) + j] >= inspection_data_min[(i * exciting_ch) + j]) &&
            (inspection_data[(i * exciting_ch) + j] <= inspection_data_max[(i * exciting_ch) + j]))
            status = 0;
        else 
            status = 1;
        }

    }

#if 1
    printk("[TSP] inspection_data");
	
	for (i = 0; i < 11*16; i++) {
		if (0 == i % 16)
			printk("\n");
		printk("0x%4x, ", inspection_data[i]);
    }
    printk("\n");
#endif
     
	msleep(200);

	enable_irq(ts->client->irq);

	msleep(200);
	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
	msleep(200);
	melfas_mcs8000_ts_resume();
    
    printk("%s : end \n", __func__);   

    return status;
}

static ssize_t set_all_refer_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;

	status = check_debug_data(melfas_mcs8000_ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);

	return sprintf(buf, "%u\n", status);
}

static int index =0;

static int atoi(char *str)
{
	int result = 0;
	int count = 0;
	if( str == NULL ) 
		return -1;
	while( str[count] != NULL && str[count] >= '0' && str[count] <= '9' )
	{		
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
}

ssize_t disp_all_refdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n",  inspection_data[index]);
}

ssize_t disp_all_refdata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index = atoi(buf);

	printk(KERN_ERR "%s : value %d\n", __func__, index);

  	return size;
}

static void check_delta_data(struct mcs8000_ts_driver *ts)
{
    u8 read_buf[2] = {0,};
    u8 write_command[4];
    u8 read_data_buf[2] = {0,};
    int exciting_ch, sensing_ch;
    int ret = 0;
    int read_data_len = 0;
    int i,j;
    int gpio = GPIO_TOUCH_INT;

    printk("%s : enter \n", __func__);

	disable_irq(ts->client->irq);
    
    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_EXCITING_CH_ADDR, read_buf, 1)) {  
        exciting_ch = read_buf[0]; // 16  
        printk("Excitin Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read EXCITING CH \n");
    }

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_SENSING_CH_ADDR, read_buf, 1)) {  
        sensing_ch = read_buf[0]; // 11
        printk("Sensing Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read SENSING CH \n");
    }
              
    /* Read Cm Delta Data */
    for (i = 0; i < sensing_ch; i++)
    {
        for (j = 0; j < exciting_ch; j++)
        {
            write_command[0] = RMI_ADDR_TESTMODE; 
            write_command[1] = 0x42; //cmd id
            write_command[2] = j; //TX CH.(exciting ch.)
            write_command[3] = i; //RX CH.(sensing ch.)
            
            ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 4);
            
        	/* wating for the interrupt */    
        	while (gpio_get_value(gpio)) {
        		printk(".");
        		udelay(100);
        	}


            if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, UNIVERSAL_CMD_RESULT_SIZE , read_buf, 1)) {  
                read_data_len = read_buf[0];
                printk("read_data_len = 0x%02x \n ", read_data_len);            
            }
            else {
                printk("can't read UNIVERSAL_CMD_RESULT_SIZE \n");
            }
            
            if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, UNIVERSAL_CMD_RESULT , read_data_buf, read_data_len)) {     
                printk("[0] = 0x%02x, [1] = 0x%02x \n", read_data_buf[0], read_data_buf[1]);
            }
            else {
                printk("can't read UNIVERSAL_CMD_RESULT Data \n");
            }

            CmDelta_data[(i * exciting_ch) + j] = (read_data_buf[1] << 8) + read_data_buf[0];

        }
    }

#if 1
    printk("[TSP] CmDelta_data");
	
	for (i = 0; i < 11*16; i++) {
		if (0 == i % 16)
			printk("\n");
		printk("0x%04x, ", CmDelta_data[i]);
    }
    printk("\n");    
#endif     

	msleep(200);

	enable_irq(ts->client->irq);

	msleep(200);
	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
	msleep(200);
	melfas_mcs8000_ts_resume();
    
    printk("%s : end \n", __func__);  
    
}

static ssize_t set_all_delta_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;
	//struct mcs8000_ts_driver *ts = dev_get_drvdata(dev);

	check_delta_data(melfas_mcs8000_ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);

	return sprintf(buf, "%u\n", status);
}

ssize_t disp_all_deltadata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("disp_all_deltadata_show : value %d\n", CmDelta_data[index]);
    return sprintf(buf, "%u\n",  CmDelta_data[index]);
}


ssize_t disp_all_deltadata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index = atoi(buf);
	printk(KERN_ERR "Delta data %d", index);
  	return size;
}

static void check_intensity_data(struct mcs8000_ts_driver *ts)
{
    u8 read_buf[2] = {0,};
    u8 write_command[2];
    u8 read_data_buf[50] = {0,};    
    int exciting_ch, sensing_ch;
    int ret = 0;
    int read_data_len = 0;
    int i,j;
    int gpio = GPIO_TOUCH_INT;

    printk("%s : enter \n", __func__);

	disable_irq(ts->client->irq);
    
    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_EXCITING_CH_ADDR, read_buf, 1)) {  
        exciting_ch = read_buf[0]; // 16  
        printk("Excitin Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read EXCITING CH \n");
    }

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_SENSING_CH_ADDR, read_buf, 1)) {  
        sensing_ch = read_buf[0]; // 11
        printk("Sensing Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read SENSING CH \n");
    }
    

    /* Read Reference Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x02;  // Reference Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);    

    for(i = 0; i < sensing_ch; i++ ) {
        
        printk("sensing_ch is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}

        read_data_len = (exciting_ch*2);  // 32 : 2 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf, read_data_len)) {        
            printk("[0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x ", read_data_buf[0], read_data_buf[1], read_data_buf[2]);
            printk("[3] = 0x%02x ... [%d] = 0x%02x \n", read_data_buf[3], read_data_len-1, read_data_buf[read_data_len-1]);
        }
        else {
            printk("can't read Reference Data \n");
        }
        
        for( j = 0; j <  read_data_len/2 ; j++)
            inspection_data[(i * exciting_ch) + j] = (read_data_buf[j*2] << 8) + read_data_buf[j*2+1];

    }

#if 1
    printk("[TSP] inspection_data");
	
	for (i = 0; i < 11*16; i++) {
		if (0 == i % 16)
			printk("\n");
		printk("0x%4x, ", inspection_data[i]);
    }
    printk("\n");    
#endif

    /* Read Dummy Data */
    printk("Read Dummy Data : START \n");
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x04;  // Intensity Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);

    for(i = 0; i < sensing_ch; i++ ) {
        
        printk("sensing_ch is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}

        read_data_len = exciting_ch;  // 16 : 1 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf, read_data_len)) {     
            printk("[0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x ", read_data_buf[0], read_data_buf[1], read_data_buf[2]);
            printk("[3] = 0x%02x ... [%d] = 0x%02x \n", read_data_buf[3], read_data_len-1, read_data_buf[read_data_len-1]);
        }
        else {
            printk("can't read Dummy Data \n");
        }
    }
    printk("Read Dummy Data : END \n");

    /* Read Intensity Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x04;  // Intensity Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);

    for(i = 0; i < sensing_ch; i++ ) {
        
        printk("sensing_ch is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}

        read_data_len = exciting_ch;  // 16 : 1 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf, read_data_len)) {     
            printk("[0] = 0x%02x, [1] = 0x%02x, [2] = 0x%02x ", read_data_buf[0], read_data_buf[1], read_data_buf[2]);
            printk("[3] = 0x%02x ... [%d] = 0x%02x \n", read_data_buf[3], read_data_len-1, read_data_buf[read_data_len-1]);
        }
        else {
            printk("can't read Intensity Data \n");
        }
        
        for( j = 0; j <  read_data_len; j++)
            lntensity_data[(i * exciting_ch) + j] = read_data_buf[j];

    }

#if 1
    printk("[TSP] lntensity_data");
	
	for (i = 0; i < 11*16; i++) {
		if (0 == i % 16)
			printk("\n");
		printk("0x%02x, ", lntensity_data[i]);
    }
    printk("\n");    
#endif 

    enable_irq(ts->client->irq);

	msleep(200);
	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
	msleep(200);
	melfas_mcs8000_ts_resume();
    
    printk("%s : end \n", __func__);

}

static ssize_t set_all_intensity_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;

	check_intensity_data(melfas_mcs8000_ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);

	return sprintf(buf, "%u\n", status);
}

ssize_t disp_all_intendata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("disp_all_intendata_show : value %d\n", lntensity_data[index]);
    return sprintf(buf, "%u\n",  lntensity_data[index]);
}


ssize_t disp_all_intendata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index = atoi(buf);
	printk(KERN_ERR "Intensity data %d", index);
  	return size;
}

static ssize_t set_refer0_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
    
	check_intensity_data(melfas_mcs8000_ts);

	refrence = inspection_data[14];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer1_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[144];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer2_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[97];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer3_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[25];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer4_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[155];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_intensity0_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[14];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity1_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[144];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity2_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[97];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity3_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[25];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity4_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[155];
	return sprintf(buf, "%u\n", intensity);
}

static DEVICE_ATTR(set_module_on, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_module_on_show, NULL);
static DEVICE_ATTR(set_module_off, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_module_off_show, NULL);
static DEVICE_ATTR(set_all_refer, S_IRUGO | S_IWUSR | S_IWGRP, set_all_refer_mode_show, NULL);
#if 0  // Change authority for CTS, but if you want to make LCD TEST MODE and then enable.
static DEVICE_ATTR(disp_all_refdata, S_IRWXU|S_IRWXG|S_IRWXO, disp_all_refdata_show, disp_all_refdata_store);
#else
static DEVICE_ATTR(disp_all_refdata, S_IRUGO|S_IWUSR|S_IWGRP, disp_all_refdata_show, disp_all_refdata_store);
#endif
static DEVICE_ATTR(set_all_delta, S_IRUGO | S_IWUSR | S_IWGRP, set_all_delta_mode_show, NULL);
static DEVICE_ATTR(disp_all_deltadata, S_IRUGO | S_IWUSR | S_IWGRP, disp_all_deltadata_show, disp_all_deltadata_store);
static DEVICE_ATTR(set_all_intensity, S_IRUGO | S_IWUSR | S_IWGRP, set_all_intensity_mode_show, NULL);
static DEVICE_ATTR(disp_all_intendata, S_IRUGO | S_IWUSR | S_IWGRP, disp_all_intendata_show, disp_all_intendata_store);
static DEVICE_ATTR(set_refer0, S_IRUGO | S_IWUSR | S_IWGRP, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO | S_IWUSR | S_IWGRP, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO | S_IWUSR | S_IWGRP, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO | S_IWUSR | S_IWGRP, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO | S_IWUSR | S_IWGRP, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity4_mode_show, NULL);
static DEVICE_ATTR(set_threshould, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_threshold_mode_show, NULL);	/* touch threshold return */

static struct attribute *sec_touch_facotry_attributes[] = {
	&dev_attr_set_module_on.attr,
	&dev_attr_set_module_off.attr,
	&dev_attr_set_all_refer.attr,
	&dev_attr_disp_all_refdata.attr,
	&dev_attr_set_all_delta.attr,
	&dev_attr_disp_all_deltadata.attr,
	&dev_attr_set_all_intensity.attr,
	&dev_attr_disp_all_intendata.attr,	
	&dev_attr_set_refer0.attr,
	&dev_attr_set_delta0.attr,
	&dev_attr_set_refer1.attr,
	&dev_attr_set_delta1.attr,
	&dev_attr_set_refer2.attr,
	&dev_attr_set_delta2.attr,
	&dev_attr_set_refer3.attr,
	&dev_attr_set_delta3.attr,
	&dev_attr_set_refer4.attr,
	&dev_attr_set_delta4.attr,
	&dev_attr_set_threshould.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};
#endif //TSP_FACTORY_TEST

static ssize_t set_tsp_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
// Read TSP version of Phone
    return sprintf(buf, "%d\n", NEW_FIRMWARE_VERSION);// kernel source version
}

static ssize_t set_tsp_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    u8 read_ver[2] = {0,};
    
    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_VERSION_ADDR, read_ver, 2)) {        
        melfas_mcs8000_ts->hw_rev = read_ver[0];
        melfas_mcs8000_ts->fw_ver = read_ver[1];
        printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, read_ver[0], read_ver[1]);
    }
    else {
        printk("%s : can't read TSP F/W version \n",__func__);
    }

	return sprintf(buf, "%#02x\n", melfas_mcs8000_ts->fw_ver);
}

static ssize_t set_tsp_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct mcs8000_ts_driver *ts = dev_get_drvdata(dev);
	u8 threshold=32;

	return sprintf(buf, "%d\n", threshold);
}

static ssize_t tsp_touchtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char temp[15];

	sprintf(temp, "MMS136\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_show, NULL);/* PHONE*/	/* firmware version resturn in phone driver version */
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_read_show, NULL);/*PART*/	/* firmware version resturn in TSP panel version */
static DEVICE_ATTR(tsp_threshold, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_threshold_mode_show, NULL);
static DEVICE_ATTR(mxt_touchtype, S_IRUGO | S_IWUSR | S_IWGRP,	tsp_touchtype_show, NULL);

static struct attribute *sec_touch_attributes[] = {
	&dev_attr_tsp_firm_version_phone.attr,
	&dev_attr_tsp_firm_version_panel.attr,
	&dev_attr_tsp_threshold.attr,
	&dev_attr_mxt_touchtype.attr,
	NULL,
};

static struct attribute_group sec_touch_attr_group = {
	.attrs = sec_touch_attributes,
};

/* Touch Reference ************************************************************/
static ssize_t reference_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf1)
{
    u8 read_buf[2] = {0,};
    u8 write_command[2];
    u8 read_data_buf[50] = {0,};    
    u16 ref[11][16]={{0,},};
    int Tx_Channel, Rx_Channel;
    int ret = 0;
    int read_data_len = 0;
    int i,j;
    int gpio = GPIO_TOUCH_INT;
    
    
    printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

   if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_EXCITING_CH_ADDR, read_buf, 1)) {  
        Tx_Channel = read_buf[0]; // 16  
        printk("Excitin Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read Tx_Channel \n");
    }

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_SENSING_CH_ADDR, read_buf, 1)) {  
        Rx_Channel = read_buf[0]; // 11
        printk("Sensing Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read Rx_Channel \n");
    }
    

    /* Read Reference Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x02;  // Reference Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);    

    for(i = 0; i < Rx_Channel; i++ ) {
        
        printk("Rx_Channel is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}

        read_data_len = (Tx_Channel*2);  // 32 : 2 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf, read_data_len)) {        
            printk("read_data[0] = 0x%02x, read_buf[1] = 0x%02x, read_data[2] = 0x%02x ", read_data_buf[0], read_data_buf[1], read_data_buf[2]);
            printk("read_buf[3] = 0x%02x \n read_buf[%d] = 0x%02x \n", read_data_buf[3], read_data_len-1, read_data_buf[read_data_len-1]);
        }
        else {
            printk("can't read Reference Data \n");
        }
        
        for(j = 0 ; j < Tx_Channel ; j++)
        {
            ref[i][j] = (read_data_buf[j*2] <<8) + read_data_buf[j*2+1];

        }
    }
    
    for (i = 0; i < Rx_Channel ; i++)
    {
        printk("[TSP]");
        for(j = 0 ; j < Tx_Channel ; j++)
        {
		printk(" %5d", ref[i][j]);
        }
        printk("\n");
    }

    return 0;

}

/* Touch Reference ************************************************************/
static ssize_t raw_store_mcs8000(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);
	if(strncasecmp(buf, "start", 5) == 0)
	{
		testmode = 1;
		printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);
	}
	else if(strncasecmp(buf, "stop", 4) == 0)
	{

		printk("[TSP] %s stop. line : %d, \n", __func__,__LINE__);
        
    	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
    	msleep(200);
    	melfas_mcs8000_ts_resume();
    	msleep(300);    
		testmode = 0;

	}
      else
      {
      		printk("[TSP] %s error-unknown command. line : %d, \n", __func__,__LINE__);
      }

    return size ;
}

static ssize_t raw_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{

    //if(!testmode) return 0;
	
    u8 read_buf[2] = {0,};
    u8 write_command[2];
    u8 read_data_buf1[50] = {0,};    
    u8 read_data_buf2[50] = {0,};
    int Tx_Channel, Rx_Channel;
    int ret = 0;
    int read_data_len = 0;
    int i,j,k;
    int gpio = GPIO_TOUCH_INT;
    
    uint16_t ref1[11][16]={{0,},};
    uint16_t ref2[11][16]={{0,},};
    int written_bytes = 0 ;  /* & error check */

	disable_irq(melfas_mcs8000_ts->client->irq);

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_EXCITING_CH_ADDR, read_buf, 1)) {  
        Tx_Channel = read_buf[0]; // 16  
        printk("Tx_Channel : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read Tx_Channel \n");
    }

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_SENSING_CH_ADDR, read_buf, 1)) {  
        Rx_Channel = read_buf[0]; // 11
        printk("Rx_Channel : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read Rx_Channel \n");
    }
    
//////////////////////// Read garbage Data of 0xB2 Data buffer ///////////////////
    /* Read Raw Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x01;  // Raw Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);
    printk("Read Raw Data \n");

     for(i = 0; i < Rx_Channel; i++ ) {
        
        printk("Rx_Channel is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}
        printk("\n");

        read_data_len = (Tx_Channel*2);  // 32 : 2 byte data

        for (k = 0; k < 10; k++)
        {
            if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf1, read_data_len)) {        
                printk("read_data[0]=0x%02x, [1]=0x%02x, [2]=0x%02x ", read_data_buf1[0], read_data_buf1[1], read_data_buf1[2]);
                printk("[3]=0x%02x ... [%d]=0x%02x \n", read_data_buf1[3], read_data_len-1, read_data_buf1[read_data_len-1]);
                break;
            }
            else {
                printk("can't read Reference Data \n");
            }
        }
    	    
    }
//////////////////////////////////////////////////////////////////////////////////

    /* Read Raw Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x01;  // Raw Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);
    printk("Read Raw Data \n");

     for(i = 0; i < Rx_Channel; i++ ) {
        
        printk("Rx_Channel is %d \n", i);
        
    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}
        printk("\n");

        read_data_len = (Tx_Channel*2);  // 32 : 2 byte data

        for (k = 0; k < 10; k++)
        {
            if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf1, read_data_len)) {        
                printk("read_data[0]=0x%02x, [1]=0x%02x, [2]=0x%02x ", read_data_buf1[0], read_data_buf1[1], read_data_buf1[2]);
                printk("[3]=0x%02x ... [%d]=0x%02x \n", read_data_buf1[3], read_data_len-1, read_data_buf1[read_data_len-1]);
                break;
            }
            else {
                printk("can't read Reference Data \n");
            }
        }
    
	    printk("[TSP]");
        for(j = 0 ; j < Tx_Channel ; j++)
        {
            ref1[i][j] = (read_data_buf1[j*2] <<8) + read_data_buf1[j*2+1];
		printk(" %5d", ref1[i][j]);
        }
	       printk("\n");
    }

//////////////////////// Read garbage Data of 0xB2 Data buffer ///////////////////
    /* Read Intensity Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x04;  // Intensity Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);
    printk("Read Intensity Data \n");
    for (i = 0; i < Rx_Channel ; i++)
    {

    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}
        printk("\n");


        read_data_len = Tx_Channel;  // 16 : 1 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf2, read_data_len)) {     
            printk("read_data[0]=0x%02x, [1]=0x%02x, [2]=0x%02x ", read_data_buf2[0], read_data_buf2[1], read_data_buf2[2]);
            printk("[3]=0x%02x ... [%d]=0x%02x \n", read_data_buf2[3], read_data_len-1, read_data_buf2[read_data_len-1]);
        }
        else {
            printk("can't read Intensity Data \n");
        }        
    }
//////////////////////////////////////////////////////////////////////////////////

    /* Read Intensity Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x04;  // Intensity Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);
    printk("Read Intensity Data \n");
    for (i = 0; i < Rx_Channel ; i++)
    {

    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}
        printk("\n");


        read_data_len = Tx_Channel;  // 16 : 1 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf2, read_data_len)) {     
            printk("read_data[0]=0x%02x, [1]=0x%02x, [2]=0x%02x ", read_data_buf2[0], read_data_buf2[1], read_data_buf2[2]);
            printk("[3]=0x%02x ... [%d]=0x%02x \n", read_data_buf2[3], read_data_len-1, read_data_buf2[read_data_len-1]);
        }
        else {
            printk("can't read Intensity Data \n");
        }
        

	    printk("[TSP]");
        for(j = 0 ; j < Tx_Channel ; j++)
        {
            ref2[i][j] = read_data_buf2[j];
	        printk(" %5d", ref2[i][j]);
        }
	    printk("\n");
    }
    

    for (i = 0; i < Rx_Channel ; i++)
    {
        for(j = 0 ; j < Tx_Channel ; j++)
        {
		written_bytes += sprintf(buf+written_bytes, "%d %d\n", ref1[i][j], ref2[i][j]) ;
        }
    }

/*    
	msleep(20);
	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
	msleep(200);
	melfas_mcs8000_ts_resume();
*/

	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
	gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
	mcsdl_vdd_off();
   
    gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
    gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN    
    msleep(50);
    mcsdl_vdd_on();
    msleep(200);

    enable_irq(melfas_mcs8000_ts->client->irq);
    
    printk("%s : end \n", __func__);    
    
    if (written_bytes > 0)
        return written_bytes ;
    
    return sprintf(buf, "-1") ;      
}

static ssize_t diff_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf1)
{
    u8 read_buf[2] = {0,};
    u8 write_command[2];
    u8 read_data_buf[50] = {0,};    
    u16 ref[11][16]={{0,},};
    int Tx_Channel, Rx_Channel;
    int ret = 0;
    int read_data_len = 0;
    int i,j;
    int gpio = GPIO_TOUCH_INT;
    
    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_EXCITING_CH_ADDR, read_buf, 1)) {  
        Tx_Channel = read_buf[0]; // 16  
        printk("Excitin Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read Tx_Channel \n");
    }

    if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, TS_READ_SENSING_CH_ADDR, read_buf, 1)) {  
        Rx_Channel = read_buf[0]; // 11
        printk("Sensing Ch : read_buf[0] = 0x%02x \n", read_buf[0]);
    }
    else {
        printk("can't read Rx_Channel \n");
    }
    
    /* Read Intensity Data */
    write_command[0] = 0x09;  // Low level data output mode 
    write_command[1] = 0x04;  // Intensity Data
    ret = melfas_i2c_write(melfas_mcs8000_ts->client, (char *)write_command, 2);

    for (i = 0; i < Rx_Channel ; i++)
    {

    	/* wating for the interrupt */    
    	while (gpio_get_value(gpio)) {
    		printk(".");
    		udelay(100);
    	}


        read_data_len = Tx_Channel;  // 16 : 1 byte data

        if( 0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, 0xB2 , read_data_buf, read_data_len)) {     
            printk("read_data[0] = 0x%02x, read_buf[1] = 0x%02x, read_data[2] = 0x%02x ", read_data_buf[0], read_data_buf[1], read_data_buf[2]);
            printk("read_buf[3] = 0x%02x \n read_buf[%d] = 0x%02x \n", read_data_buf[3], read_data_len-1, read_data_buf[read_data_len-1]);
        }
        else {
            printk("can't read Intensity Data \n");
        }
        
        for(j = 0 ; j < Tx_Channel ; j++)
        {
            ref[i][j] = read_data_buf[j];
        }
    }        
        
    for (i = 0; i < Rx_Channel ; i++)
    {
        printk("[TSP]");
        for(j = 0 ; j < Tx_Channel ; j++)
        {
		printk(" %5d", ref[i][j]);
        }
        printk("\n");
    }

    return 0;
}

static ssize_t set_tsp_test_mode_enable0(struct device *dev, struct device_attribute *attr, char *buf)
{
    testmode = 1; 
    printk("set_tsp_test_mode_enable0 \n");
    return 0;
}

static ssize_t set_tsp_test_mode_disable0(struct device *dev, struct device_attribute *attr, char *buf)
{
    melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
    msleep(200);
    melfas_mcs8000_ts_resume();
    msleep(300);    
    printk("set_tsp_test_mode_disable0 \n");
    
    testmode = 0;
    
    return 0;
}

static DEVICE_ATTR(reference,  S_IRUGO | S_IWUSR | S_IWGRP, reference_show_mcs8000, NULL) ;
static DEVICE_ATTR(raw,  S_IRUGO | S_IWUSR | S_IWGRP, raw_show_mcs8000, raw_store_mcs8000) ;
static DEVICE_ATTR(diff,  S_IRUGO | S_IWUSR | S_IWGRP, diff_show_mcs8000, NULL) ;
static DEVICE_ATTR(versname,  S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_read_show, NULL) ;
static DEVICE_ATTR(enable0,  S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_test_mode_enable0, NULL) ;
static DEVICE_ATTR(disable0,  S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_test_mode_disable0, NULL) ;

#endif //SEC_TSP

int melfas_mcs8000_ts_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
	int ret = 0;
	int rc, i;
	uint16_t max_x=0, max_y=0;
	int version_read = FALSE;
    
	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] PROBE       =========");
	printk("\n====================================================\n");

	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TSP_LDO_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	gpio_set_value( GPIO_I2C0_SCL , 1 ); 
	gpio_set_value( GPIO_I2C0_SDA , 1 ); 
	gpio_set_value( GPIO_TOUCH_INT , 1 );
	msleep(10);
	gpio_set_value( TSP_LDO_ON, 1 ); 
	msleep(500);


	// KEYLED vdd
	vreg_ldo2 = vreg_get(NULL, "xo_out");
	if (IS_ERR(vreg_ldo2)) {
		rc = PTR_ERR(vreg_ldo2);
		pr_err("%s: LDO2 vreg get failed (%d)\n",
		       __func__, rc);
	}

	rc = vreg_set_level(vreg_ldo2, 3300);
	if (rc) {
		pr_err("%s: LDO2 vreg set level failed (%d)\n",
		       __func__, rc);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C/*I2C_FUNC_SMBUS_BYTE_DATA*/)) {
		printk(KERN_ERR "melfas_mcs8000_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	melfas_mcs8000_ts = kzalloc(sizeof(*melfas_mcs8000_ts), GFP_KERNEL);
	if (melfas_mcs8000_ts == NULL) {
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	melfas_mcs8000_ts->client = client;
	i2c_set_clientdata(client, melfas_mcs8000_ts);
         
    for(i=0; i<5; i++) {
        if(0 == melfas_mcs8000_read_version()) {
            version_read = TRUE;
            printk("[TSP] Read Version Success !!\n");
            break;
        }
        else {            
            version_read = FALSE;            
            printk("[TSP] Read Version Failed !!\n");
        }
    }
    
    if(version_read == FALSE) {
        ret = -ENODEV;
        printk("[TSP] ERROR : There is no valid TSP ID \n");
        goto err_check_functionality_failed;
    }
         
    printk("[TSP] Window_ver. 70T : hw rev %d, fw ver : %d, new fw ver %d\n",
				melfas_mcs8000_ts->hw_rev, melfas_mcs8000_ts->fw_ver, NEW_FIRMWARE_VERSION);
      
#if defined(CONFIG_MACH_ANCORA_TMO)
    if(melfas_mcs8000_ts->window_ver==MELFAS_WINDOW_55T)	// Window panel 0.55T
    {	
        if((board_hw_revision >= 4) && (melfas_mcs8000_ts->fw_ver < NEW_FIRMWARE_VERSION_55T) && (version_read == TRUE))
        {
            printk("[TSP] firmware update window 0.55T\n");
            firmware_update();
        }
    }
    else //if(melfas_mcs8000_ts->window_ver==MELFAS_WINDOW_70T) // Window panel 0.7T
    {	
        if((melfas_mcs8000_ts->hw_rev == 0x08)&&(board_hw_revision >= 4) && (melfas_mcs8000_ts->fw_ver < NEW_FIRMWARE_VERSION) && (version_read == TRUE))
        {
            printk("[TSP] firmware update window 0.7T\n");
            firmware_update();
        }
    }
    INIT_WORK(&melfas_mcs8000_ts->work, melfas_ts_work_func);
	
#else
    if ((melfas_mcs8000_ts->hw_rev == 0x03) && (melfas_mcs8000_ts->fw_ver < NEW_FIRMWARE_VERSION_GFF) && (version_read == TRUE))
    {
		printk("[TSP] firmware update window 0.7T TSP H/W rev.03 GFF \n");
		firmware_update();        
    }
	else if((melfas_mcs8000_ts->hw_rev == 0x50) && (melfas_mcs8000_ts->fw_ver < NEW_FIRMWARE_VERSION_G2) && (version_read == TRUE))
	{
		printk("[TSP] firmware update window 0.7T TSP H/W rev.50 G2 \n");
		firmware_update();
	}
    else 
    {
        printk("[TSP] Have the Latest firmware \n");
    }

	INIT_WORK(&melfas_mcs8000_ts->work, melfas_ts_work_func);
#endif
				
	// firmware updgrade	
	printk(KERN_INFO "[TOUCH] Melfas  H/W version: 0x%x.\n", melfas_mcs8000_ts->hw_rev);
	printk(KERN_INFO "[TOUCH] Current F/W version: 0x%x.\n", melfas_mcs8000_ts->fw_ver);
	
	melfas_mcs8000_read_resolution(); 
	max_x = 480; //melfas_mcs8000_ts->info.max_x ;
	max_y = 800; //melfas_mcs8000_ts->info.max_y ;
	printk("melfas_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	melfas_mcs8000_ts->input_dev = input_allocate_device();
	if (melfas_mcs8000_ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_mcs8000_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	melfas_mcs8000_ts->input_dev->name = "sec_touchscreen";

	set_bit(EV_SYN, melfas_mcs8000_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_mcs8000_ts->input_dev->evbit);
	set_bit(TOUCH_HOME, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(TOUCH_MENU, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(TOUCH_BACK, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(TOUCH_SEARCH, melfas_mcs8000_ts->input_dev->keybit);

	melfas_mcs8000_ts->input_dev->keycode = melfas_ts_tk_keycode;	
	set_bit(BTN_TOUCH, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_mcs8000_ts->input_dev->evbit);

	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	printk("melfas_mcs8000_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	ret = input_register_device(melfas_mcs8000_ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_mcs8000_ts_probe: Unable to register %s input device\n", melfas_mcs8000_ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk("[Touch] irq : %d, irq : %d\n", melfas_mcs8000_ts->client->irq, client->irq);

	irq_set_irq_type(melfas_mcs8000_ts->client->irq, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(melfas_mcs8000_ts->client->irq, melfas_mcs8000_ts_irq_handler, IRQF_DISABLED  , melfas_mcs8000_ts->client->name, melfas_mcs8000_ts);
	if(ret == 0) {
		printk(KERN_INFO "melfas_mcs8000_ts_probe: Start touchscreen %s \n", melfas_mcs8000_ts->input_dev->name);
	}
	else {
		printk("request_irq failed\n");
	}

	printk("[Melfas] ret : %d, melfas_mcs8000_ts->client name : [%s]  [%d] [0x%x]\n",ret,melfas_mcs8000_ts->client->name, melfas_mcs8000_ts->client->irq, melfas_mcs8000_ts->client->addr);

#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_mcs8000_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_mcs8000_ts->early_suspend.suspend = melfas_mcs8000_ts_early_suspend;
	melfas_mcs8000_ts->early_suspend.resume = melfas_mcs8000_ts_late_resume;
	register_early_suspend(&melfas_mcs8000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef AUTO_POWER_ON_OFF_FLAG
	init_timer(&poweroff_touch_timer);
	poweroff_touch_timer.function = poweroff_touch_timer_handler;
	poweroff_touch_timer.expires = jiffies + 45*HZ;
	add_timer(&poweroff_touch_timer);
#endif

	return 0;
err_input_register_device_failed:
	input_free_device(melfas_mcs8000_ts->input_dev);

err_input_dev_alloc_failed:
	kfree(melfas_mcs8000_ts);
err_check_functionality_failed:
	return ret;

}

int melfas_mcs8000_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs8000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs8000_ts->client->irq, 0);
	input_unregister_device(melfas_mcs8000_ts->input_dev);
	return 0;
}

void melfas_mcs8000_ts_gen_touch_up(void)
{
    int i=0;
    
  // report up key if needed
    for (i = 0; i < MELFAS_MAX_TOUCH; i++)
    {
        printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d, state: %d \n", i, g_Mtouch_info[i].state);
        
        if ((g_Mtouch_info[i].posX == 0) || (g_Mtouch_info[i].posY == 0))
            continue;
        
        if(g_Mtouch_info[i].state == 1) {
            g_Mtouch_info[i].state = 0;
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, i);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
            input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
            input_sync(melfas_mcs8000_ts->input_dev);
        }
    }   
}

int melfas_mcs8000_ts_suspend(pm_message_t mesg)
{
	int rc = 0;
	
	printk("%s\n", __func__);
	melfas_mcs8000_ts->suspended = true;	
	disable_irq(melfas_mcs8000_ts->client->irq);
    melfas_mcs8000_ts_gen_touch_up();

	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	mcsdl_vdd_off();

	if(led_state == TKEY_LED_ON) {
		rc = vreg_disable(vreg_ldo2);
        
        printk("touch_led_control : ts_suspend forced off! rc = %d \n", rc);              

		if (rc) 
			pr_err("%s: LDO2 vreg disable failed (%d)\n", __func__, rc);		
        else 
            led_state = TKEY_LED_FORCEDOFF;
	}
        
	gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
	gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
  
#if defined(CONFIG_MACH_ANCORA_TMO)
       //release home key before entering suspend mode
	if(touchkey_status[2]) 
	input_report_key(melfas_mcs8000_ts->input_dev, touchkey_keycodes[1], 0);
#endif
			
  return 0;
}

int melfas_mcs8000_ts_resume()
{
    int rc = 0;
    printk("%s\n", __func__);
    gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

    gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
    gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN    
    msleep(100);
    mcsdl_vdd_on();
    msleep(200);

    melfas_mcs8000_ts->suspended = false;
    enable_irq(melfas_mcs8000_ts->client->irq);  

    if(led_state == TKEY_LED_FORCEDOFF) { 
        rc = vreg_enable(vreg_ldo2); 

        printk("%s TKEY_LED_FORCEDOFF  rc = %d \n", __func__,rc);               

        if (rc)  
            pr_err("%s: LDO2 vreg disable failed (%d)\n", __func__, rc);		 
        else  
            led_state = TKEY_LED_ON; 
    } 
   
    return 0;
}
#if 0 // blocked for now.. we will gen touch when suspend func is called
int tsp_preprocess_suspend(void)
{
  // this function is called before kernel calls suspend functions
  // so we are going suspended if suspended==false
  if(melfas_mcs8000_ts->suspended == false) {  
    // fake as suspended
    melfas_mcs8000_ts->suspended = true;
    
    //generate and report touch event
    melfas_mcs8000_ts_gen_touch_up();
  }
  return 0;
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs8000_ts_early_suspend(struct early_suspend *h)
{
	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
}

void melfas_mcs8000_ts_late_resume(struct early_suspend *h)
{
	melfas_mcs8000_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */


int melfas_mcs8000_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_mcs8000_ts->client = client;
	i2c_set_clientdata(client, melfas_mcs8000_ts);
	return 0;
}

#define USE_TS_TA_DETECT_CHANGE_REG 1
#if USE_TS_TA_DETECT_CHANGE_REG 
int set_tsp_for_ta_detect(int state)
{
    return 1;
}
EXPORT_SYMBOL(set_tsp_for_ta_detect);
#endif

static int __devexit melfas_mcs8000_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs8000_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs8000_ts->client->irq, 0);
	input_unregister_device(melfas_mcs8000_ts->input_dev);
   
	melfas_mcs8000_ts = i2c_get_clientdata(client);
	kfree(melfas_mcs8000_ts);
	return 0;
}

struct i2c_device_id melfas_mcs8000_id[] = {
	{ "mcs8000_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, melfas_mcs8000_id);

struct i2c_driver mcs8000_ts_i2c = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mcs8000_i2c",
	},
	.id_table	= melfas_mcs8000_id,
	.probe	= melfas_mcs8000_ts_probe,
	.remove = melfas_mcs8000_ts_remove,
};


int __init melfas_mcs8000_ts_init(void)
{
	int ret;

	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] INIT        =========");
	printk("\n====================================================\n");

	melfas_mcs8000_ts_wq = create_singlethread_workqueue("melfas_mcs8000_ts_wq");
	if (!melfas_mcs8000_ts_wq)
		return -ENOMEM;
	
	melfas_mcs8000_ts = kzalloc(sizeof(struct mcs8000_ts_driver), GFP_KERNEL);
	if(melfas_mcs8000_ts == NULL) {
		return -ENOMEM;
	}

	ret = i2c_add_driver(&mcs8000_ts_i2c);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	printk("[Melfas] ret : %d, melfas_mcs8000_ts->client name : %s\n",ret,melfas_mcs8000_ts->client->name);

	if(!melfas_mcs8000_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&mcs8000_ts_i2c);
		return 0;
	}

	if (sec_class == NULL)
	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	mcs8000_ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
	
	if (IS_ERR(mcs8000_ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(mcs8000_ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);

	if (device_create_file(mcs8000_ts_dev, &dev_attr_tkey_sensitivity) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_sensitivity.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_tkey_sens_pline) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_sens_pline.attr.name);
    if (device_create_file(mcs8000_ts_dev, &dev_attr_tkey_noise_thd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_noise_thd.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_brightness) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
#ifdef QT_ATCOM_TEST

    if (device_create_file(mcs8000_ts_dev, &dev_attr_key_threshold) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_key_threshold.attr.name);

	qt602240_atcom_test = device_create(sec_class, NULL, 0, NULL, "qt602240_atcom_test");
	if (IS_ERR(qt602240_atcom_test))
		printk("Failed to create device(qt602240_atcom_test)!\n");

	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_update)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_update.attr.name);
	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_firm_version)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_firm_version.attr.name);
	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_firm_status)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_firm_status.attr.name);
	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_firm_version_read)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_firm_version_read.attr.name);
	if (device_create_file(qt602240_atcom_test, &dev_attr_set_qt_hw_version_read)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_qt_hw_version_read.attr.name);
#endif

#ifdef SEC_TSP
	sec_touchscreen = device_create(sec_class, NULL, 0, NULL, "sec_touchscreen");
	if (IS_ERR(sec_touchscreen))
		pr_err("[TSP] Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&sec_touchscreen->kobj, &sec_touch_attr_group);
	if (ret)
		pr_err("[TSP] Failed to create sysfs group\n");
    
#ifdef TSP_FACTORY_TEST
	qt602240_noise_test = device_create(sec_class, NULL, 0, NULL, "qt602240_noise_test");

	if (IS_ERR(qt602240_noise_test))
		pr_err("[TSP] Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&qt602240_noise_test->kobj, &sec_touch_factory_attr_group);
	if (ret)
		pr_err("[TSP] Failed to create sysfs group\n");
#endif

     /* sys fs */

	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_versname) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_versname.attr.name);

    if (device_create_file(firmware_dev, &dev_attr_reference) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_reference.attr.name);
    if (device_create_file(firmware_dev, &dev_attr_raw) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
    if (device_create_file(firmware_dev, &dev_attr_diff) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_diff.attr.name);

    if (device_create_file(firmware_dev, &dev_attr_enable0) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_enable0.attr.name);
    if (device_create_file(firmware_dev, &dev_attr_disable0) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_disable0.attr.name);

#endif

	return 0; //platform_driver_register(&mcs8000_ts_driver);

}

void __exit melfas_mcs8000_ts_exit(void)
{
	i2c_del_driver(&mcs8000_ts_i2c);

	if (melfas_mcs8000_ts_wq)
		destroy_workqueue(melfas_mcs8000_ts_wq);
}
//late_initcall(melfas_mcs8000_ts_init);
module_init(melfas_mcs8000_ts_init);
module_exit(melfas_mcs8000_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
