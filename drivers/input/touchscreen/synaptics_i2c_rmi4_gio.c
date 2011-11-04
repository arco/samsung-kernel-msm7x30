/* drivers/input/keyboard/synaptics_i2c_rmi.c
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/device.h>
#include <linux/i2c/gio_tsp_gpio.h>
#include <asm/gpio.h>


#define FIRM_TEST 0

/* firmware - update */
#if FIRM_TEST
#include "./CYPRESS_I2C_ISSP/issp_define.h"
#endif

#include <linux/firmware.h>
/* firmware - update */

#define MAX_X	320 
#define MAX_Y	480 

#define MAX_KEYS	2
#define MAX_USING_FINGER_NUM 2

static int prev_wdog_val = -1;
static int check_ic_counter = 3;

static const int touchkey_keycodes[] = {
			KEY_MENU,
			KEY_BACK,
};

static int touchkey_status[MAX_KEYS];

#define TK_STATUS_PRESS		1
#define TK_STATUS_RELEASE		0


static struct workqueue_struct *synaptics_wq;
static struct workqueue_struct *check_ic_wq;


typedef struct
{
	uint8_t id;	/*!< (id>>8) + size */
	int8_t status; /*!< dn>0, up=0, none=-1 */ 
	uint8_t z;	/*!width */
	uint16_t x;			/*!< X */
	uint16_t y;			/*!< Y */
} report_finger_info_t;

static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM + 1]={0,};

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	struct work_struct  work_timer;
	struct early_suspend early_suspend;
};

struct synaptics_ts_data *ts_global;
int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

/* firmware - update */
static int firmware_ret_val = -1;
static int HW_ver = -1;
static int pre_ta_stat = 0;
static int pre_prox_stat = 0;

#if FIRM_TEST
unsigned char g_pTouchFirmware[TSP_TOTAL_LINES*TSP_LINE_LENGTH];
unsigned int g_FirmwareImageSize = 0;

static void issp_request_firmware(char* update_file_name);
#endif

int firm_update( void );
extern int cypress_update( int );
extern int board_hw_revision;
//extern int tsp_charger_type_status;
extern int tsp_proximity_irq_status;

#if FIRM_TEST
unsigned char pSocData[]= {
	"fdjkjklijk"
		//#include "./CYPRESS_I2C_ISSP/Europa_ver02_100218" 	
};
#endif
/* firmware - update */

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

int set_tsp_for_ta_detect(int);
int set_tsp_for_prox_enable(int);
static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static DEVICE_ATTR(firmware	, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret	, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, firmware_ret_show, firmware_ret_store);
/* sys fs */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

int tsp_reset( void )
{
	int ret=1, key = 0;
#if 0	
	uint8_t i2c_addr = 0x07;
	uint8_t buf[1];
#endif
	struct vreg *vreg_touch;
	printk("[TSP] %s+\n", __func__ );

	vreg_touch = vreg_get(NULL, "ldo19");

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

	if (ts_global->use_irq)
	{
		disable_irq(ts_global->client->irq);
	}

	ret = vreg_disable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		ret=-EIO;
		goto tsp_reset_out;
	}

	//gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
	//gpio_configure( TSP_SDA, GPIOF_DRIVE_OUTPUT );
	//gpio_configure( TSP_INT, GPIOF_INPUT );
#if 1
	gpio_tlmm_config(GPIO_CFG( TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG( TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG( TSP_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);
#endif

	gpio_set_value( TSP_SCL , 0 ); 
	gpio_set_value( TSP_SDA , 0 ); 
	gpio_set_value( TSP_INT , 0 ); 

	msleep( 5 );

	gpio_set_value( TSP_SCL , 1 ); 
	gpio_set_value( TSP_SDA , 1 ); 
	gpio_set_value( TSP_INT , 1 ); 

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		ret=-EIO;
		goto tsp_reset_out;
	}
		
	msleep(200);
#if 0
	while (ts_global->use_irq)
	{
		msleep(10);

		ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));
		if (ret <= 0) {
			printk("[TSP] %d : i2c_transfer failed\n", __LINE__);
		}
		else
		{
			printk("[TSP] %s:%d, ver SW=%x\n", __func__,__LINE__, buf[0] );
			ret=1;
			break;
		}
	}
#endif

tsp_reset_out:
	if (ts_global->use_irq)
	{
		enable_irq(ts_global->client->irq);
	}
	printk("[TSP] %s-\n", __func__ );

	return ret;
}

static void process_key_event(uint8_t tsk_msg)
{
	int i;
	int keycode= 0;
	int st_old, st_new;

	//check each key status
	for(i = 0; i < MAX_KEYS; i++)
	{
		st_old = touchkey_status[i];
		st_new = (tsk_msg>>(i+6)) & 0x1;
		keycode = touchkey_keycodes[i];

		touchkey_status[i] = st_new;	// save status

		if(st_new > st_old)
		{
			// press event
			printk("[TSP] press keycode: %4d, keypress: %4d\n", keycode, 1);
			input_report_key(ts_global->input_dev, keycode, 1);
		}
		else if(st_old > st_new)
		{
			// release event
			printk("[TSP] release keycode: %4d, keypress: %4d\n", keycode, 0);
			input_report_key(ts_global->input_dev, keycode, 0);
		}
	}

}

void TSP_forced_release_forkey(void)
{
	int i;
	int temp_value=0;
	
	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	{
		if(fingerInfo[i].id >=1)
		{
			fingerInfo[i].status = -2; // force release
		}

		if(fingerInfo[i].status != -2) continue;
		
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
		input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].z);
		input_mt_sync(ts_global->input_dev);

		printk("[TSP] force release\n");
		temp_value++;
	}

	if(temp_value>0)
		input_sync(ts_global->input_dev);

	
}
EXPORT_SYMBOL(TSP_forced_release_forkey);

#define ABS(a,b) (a>b?(a-b):(b-a))
static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret=0;
	uint8_t buf[12];// 02h ~ 0Dh
	uint8_t i2c_addr = 0x02;
	int i = 0;
	uint8_t finger = 0;

	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	memset(buf, 0, sizeof(buf));
	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	if (ret <= 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		goto work_func_out;
	}

	finger = buf[0] & 0x07;	
	
	fingerInfo[0].x = (buf[1] << 8) |buf[2];
	fingerInfo[0].y = (buf[3] << 8) |buf[4];
	fingerInfo[0].z = buf[5]/2;
	fingerInfo[0].id = (buf[6] >>4)& 0x0f;

	fingerInfo[1].x = (buf[7] << 8) |buf[8];
	fingerInfo[1].y = (buf[9] << 8) |buf[10];
	fingerInfo[1].z = buf[11]/2;
	fingerInfo[1].id = buf[6] & 0x0f;
/*
	// scaling for module on emul board
	fingerInfo[0].x = (fingerInfo[0].x * 320) / 240;
	fingerInfo[0].y = (fingerInfo[0].y * 480) / 320;
	fingerInfo[1].x = (fingerInfo[1].x * 320) / 240;
	fingerInfo[1].y = (fingerInfo[1].y * 480) / 320;
*/

	//	print message
//	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
//		printk("[TSP] finger[%d].x = %d, finger[%d].y = %d, finger[%d].z = %x, finger[%d].id = %x\n", i, fingerInfo[i].x, i, fingerInfo[i].y, i, fingerInfo[i].z, i, fingerInfo[i].id);
/*
	if(fingerInfo[0].status != 1 && fingerInfo[1].status != 1)
		printk("[TSP] < press > [%d].x = %d, [%d].y = %d, [%d].z = %x, [%d].id = %x\n", i, fingerInfo[i].x, i, fingerInfo[i].y, i, fingerInfo[i].z, i, fingerInfo[i].id);
	if(fingerInfo[0].id != 1 && fingerInfo[0].id != 2)
		printk("[TSP] <release> [%d].x = %d, [%d].y = %d, [%d].z = %x, [%d].id = %x\n", i, fingerInfo[i].x, i, fingerInfo[i].y, i, fingerInfo[i].z, i, fingerInfo[i].id);
*/


	/* check key event*/
	if(fingerInfo[0].status != 1 && fingerInfo[1].status != 1)
		process_key_event(buf[0]);



	/* check touch event */
	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	{
		if(fingerInfo[i].id >=1) // press interrupt
		{
			if(i==0 && fingerInfo[1].status != 1)
			{
				if((fingerInfo[2].id != fingerInfo[0].id)&&(fingerInfo[2].id != 0))// no release with finger id change
				{
		//			if(fingerInfo[1].id ==0)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, fingerInfo[2].x, fingerInfo[2].y, fingerInfo[2].z);
						fingerInfo[1].status = -1;
					}
				}
				else if(fingerInfo[2].id != 0) // check x or y jump with same finger id
				{
					
					if(ABS(fingerInfo[2].x,fingerInfo[0].x)>180)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, fingerInfo[2].x, fingerInfo[2].y, fingerInfo[2].z);
						fingerInfo[1].status = -1;	
					}
					else if(ABS(fingerInfo[2].y,fingerInfo[0].y)>180)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, fingerInfo[2].x, fingerInfo[2].y, fingerInfo[2].z);
						fingerInfo[1].status = -1;	
					}
					else // no jump
					{
						if(fingerInfo[i].status != -2) // force release
							fingerInfo[i].status = 1;
						else
							fingerInfo[i].status = -2;
					}
				}
				else // single touch with normal condition
				{
					if(fingerInfo[i].status != -2) // force release
						fingerInfo[i].status = 1;
					else
						fingerInfo[i].status = -2;
				}
			}
			else
			{
				if(fingerInfo[i].status != -2) // force release
					fingerInfo[i].status = 1;
				else
					fingerInfo[i].status = -2;
			}
		}
		else if(fingerInfo[i].id ==0) // release interrupt (only first finger)
		{
			if(fingerInfo[i].status == 1) // prev status is press
				fingerInfo[i].status = 0;
			else if(fingerInfo[i].status == 0 || fingerInfo[i].status == -2) // release already or force release
				fingerInfo[i].status = -1;				
		}

		if(fingerInfo[i].status < 0) continue;
		
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].status);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].z);
		input_mt_sync(ts->input_dev);

		printk("[TSP] [%d] %d (%d,	%d,	%x)		%x\n", i, fingerInfo[i].id, fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].z, fingerInfo[i].status);		
	}



//	printk("\n");
//	printk("%d	%d\n", fingerInfo[0].status, fingerInfo[1].status);
	input_sync(ts->input_dev);
	
	fingerInfo[2].x = fingerInfo[0].x;
	fingerInfo[2].y = fingerInfo[0].y;
	fingerInfo[2].z = fingerInfo[0].z;
	fingerInfo[2].id = fingerInfo[0].id;	
	
work_func_out:
	if (ts->use_irq)
	{
		enable_irq(ts->client->irq);
	}
}

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;//I2C_M_WR;
	rmsg.len = 1;
	rmsg.buf = &start_reg;
	start_reg = reg;
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	if(ret>=0) {
		rmsg.flags = I2C_M_RD;
		rmsg.len = buf_size;
		rmsg.buf = rbuf;
		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1 );
	}

	if( ret < 0 )
		{
		printk("[TSP] Error code : %d\n", __LINE__ );
//		printk("[TSP] reset ret=%d\n", tsp_reset( ) );
	}

	return ret;
}
int tsp_i2c_write(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	unsigned char data[2];

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;
	rmsg.len = 2;
	rmsg.buf = data;
	data[0] = reg;
	data[1] = rbuf[0];
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	return ret;
}
static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}
static void check_ic_work_func(struct work_struct *work)
{
	int ret=0;
	uint8_t i2c_addr = 0x1F;
	uint8_t wdog_val[1];

	wdog_val[0] = 1;
	//if( pre_ta_stat != tsp_charger_type_status )
	//{
	//	set_tsp_for_ta_detect(tsp_charger_type_status);
	//}

	if( pre_prox_stat != tsp_proximity_irq_status )
	{
		set_tsp_for_prox_enable(tsp_proximity_irq_status);
	}

	if(check_ic_counter == 0)
	{
		ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));
		if (ret <= 0) {
			tsp_reset();
			printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		}
		else if(wdog_val[0] == (uint8_t)prev_wdog_val || wdog_val[0] == 0x0 ||wdog_val[0] == 0xff)
		{
			printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
			tsp_reset();
			prev_wdog_val = -1;
		}
		else
		{
//			printk("[TSP] %s counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
			prev_wdog_val = wdog_val[0];
		}
		
		check_ic_counter = 3;	
	}
	else
	{
		check_ic_counter--;
	}
}

static enum hrtimer_restart cypress_watchdog_timer_func(struct hrtimer *timer)
{
	queue_work(check_ic_wq, &ts_global->work_timer);
	hrtimer_start(&ts_global->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static int synaptics_ts_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0, key = 0;
	struct vreg *vreg_touch;
	uint8_t i2c_addr = 0x1B;
	uint8_t buf[3]={0,};

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	vreg_touch = vreg_get(NULL, "ldo19");
	ret = vreg_set_level(vreg_touch, OUT3000mV);
	if (ret) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

	msleep(700);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_WORK(&ts->work_timer, check_ic_work_func );
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = cypress_watchdog_timer_func;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	printk(KERN_INFO "synaptics_ts_probe: max_x: 320, max_y: 480\n");
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	for(key = 0; key < MAX_KEYS ; key++)
		input_set_capability(ts->input_dev, EV_KEY, touchkey_keycodes[key]);

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	printk("[TSP] %s, irq=%d\n", __func__, client->irq );
	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler,/* IRQF_TRIGGER_RISING |*/ IRQF_TRIGGER_FALLING , client->name, ts);
		if (ret == 0) 
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	//	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	//	ts->timer.function = synaptics_ts_timer_func;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	/* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);

	/* sys fs */

	/* Check point - i2c check - start */
	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));
	HW_ver = buf[1];
	
	printk("[TSP] %s: %d, buf[1]=%x, buf[2]=%x\n", __func__, __LINE__, buf[1], buf[2]);
		
	if( buf[1]==0 || buf[2]==0 )  
	{
		printk("[TSP] emergency firmware UPDATE\n");	
		firm_update();
	}

	if (ret <= 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

		if (ret <= 0) 
		{
			printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
			goto err_check_functionality_failed;
		}
	}
	
	
	/* Check point - i2c check - end */
	/* Check point - Firmware */
	printk("[TSP] %s, ver CY=%x\n", __func__ , buf[0] );
	printk("[TSP] %s, ver HW=%x\n", __func__ , buf[1] );
	printk("[TSP] %s, ver SW=%x\n", __func__ , buf[2] );

	printk(KERN_INFO "synaptics_ts_probe: Manufacturer ID: %x, HW ver=%d\n", buf[0], HW_ver);

/*
		if ( board_hw_revision >= 0x3 )
		{

			if(buf[1] == 1 && buf[2] < 0x3 )
			{
				firm_update( );
			}
		}
*/

	if(HW_ver >= 1)
		hrtimer_start(&ts->timer, ktime_set(5, 0), HRTIMER_MODE_REL);
	
#if 0
	if ( ( 0x00 < buf[2]) && (buf[2] < 0x04) )
	{
		printk("[TSP] %s, ver SW=%x\n", __func__ , buf[2] );
		printk("[TSP] %s, firm_update was blocked!!\n", __func__ );
		//firm_update( );
	}
#endif	
	/* Check point - Firmware */

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	//	else
	//		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	struct vreg *vreg_touch;
	printk("[TSP] %s+\n", __func__ );

	vreg_touch = vreg_get(NULL, "ldo19");

	if (ts->use_irq)
	{
		disable_irq(client->irq);
	}
	
//	gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
//	gpio_configure( TSP_SDA, GPIOF_DRIVE_OUTPUT );
//	gpio_configure( TSP_INT, GPIOF_INPUT );
#if 1
	gpio_tlmm_config(GPIO_CFG( TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG( TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG( TSP_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);
#endif

	gpio_set_value( TSP_SCL , 0 ); 
	gpio_set_value( TSP_SDA , 0 ); 
	gpio_set_value( TSP_INT , 0 ); 

	ret = cancel_work_sync(&ts->work_timer);

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}

	hrtimer_cancel(&ts->timer);

	ret = vreg_disable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}
	msleep(400);

	TSP_forced_release_forkey();

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret, key, retry_count;
	struct vreg *vreg_touch;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	uint8_t i2c_addr = 0x1D;
	uint8_t buf[1];

	printk("[TSP] %s+\n", __func__ );

	gpio_set_value( TSP_SCL , 1 ); 
	gpio_set_value( TSP_SDA , 1 ); 
	gpio_set_value( TSP_INT , 1 ); 

	vreg_touch = vreg_get(NULL, "ldo19");

	ret = vreg_enable(vreg_touch);
	
	if (ret)
	{
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
				__func__, ret);
		return -EIO;
	}

//	msleep(130);
	msleep(500);

	// for TSK
	for(key = 0; key < MAX_KEYS; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

	fingerInfo[0].status = -1;
        fingerInfo[1].status = -1;
        fingerInfo[2].id = 0;


	retry_count = 0;

	while (ts->use_irq)
	{
		ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));
		
		if (ret <= 0) 
		{
			retry_count++;
//			printk("[TSP] %d : i2c_transfer failed\n", __LINE__);
		}
		else if	( buf[0] == 0 )
		{
//			continue;
			retry_count++;
		}
		else
		{
			printk("[TSP] %s:%d, ver SW=%x\n", __func__,__LINE__, buf[0] );
			enable_irq(client->irq);
			break;
		}

		if(retry_count > 5){
			printk("[TSP] %s: %d, retry_count=%d\n", __func__, __LINE__, retry_count);
			break;
		}
	
		msleep(20);
	}

	prev_wdog_val = -1;

	//if(tsp_charger_type_status == 1)
	//{
	//	set_tsp_for_ta_detect(tsp_charger_type_status);
	//}
	
	if( tsp_proximity_irq_status == 1)
	{
		set_tsp_for_prox_enable(tsp_proximity_irq_status);
	}

	if(HW_ver >= 1)
		hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "synaptics-rmi-ts", 1 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, synaptics_ts_id);

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "synaptics-rmi-ts",
	},
};

int set_tsp_for_ta_detect(int state)
{
	
	int ret=0;
	uint8_t i2c_addr = 0x00;
	uint8_t wdog_val[1];

	ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));
	if(ret == 0)
	{
		printk("[TSP] i2c failed\n");
	}
	
	if(state)
	{
				//printk("[TSP] [1] set_tsp_for_ta_detect!!! state=1\n");
		wdog_val[0] = wdog_val[0] | (1 << 3);
		ret = tsp_i2c_write( i2c_addr, wdog_val, sizeof(wdog_val));
		pre_ta_stat = 1;
	}
	else
	{
				//printk("[TSP] [2] set_tsp_for_ta_detect!!! state=0\n");
		wdog_val[0] = wdog_val[0] & (0 << 3);
		ret = tsp_i2c_write( i2c_addr, wdog_val, sizeof(wdog_val));
		pre_ta_stat = 0;
	}
	
	ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));                 
	printk("[TSP] %s = %x\n", __func__, (wdog_val[0] & 0x08));
} 
EXPORT_SYMBOL(set_tsp_for_ta_detect);
int set_tsp_for_prox_enable(int state)
{
	
	int ret=0;
	uint8_t i2c_addr = 0x1e;
	uint8_t wdog_val[1];

	ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));
	if(ret == 0)
	{
		printk("[TSP] i2c failed\n");
	}
	
	if(state)
	{
				//printk("[TSP] [1] set_tsp_for_ta_detect!!! state=1\n");
		wdog_val[0] = wdog_val[0] | (1 << 7);
		ret = tsp_i2c_write( i2c_addr, wdog_val, sizeof(wdog_val));
		pre_prox_stat = 1;
	}
	else
	{
				//printk("[TSP] [2] set_tsp_for_ta_detect!!! state=0\n");
		wdog_val[0] = wdog_val[0] & (0 << 7);
		ret = tsp_i2c_write( i2c_addr, wdog_val, sizeof(wdog_val));
		pre_prox_stat = 0;
	}
	
	ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));                 
	printk("[TSP] %s = %x\n", __func__, (wdog_val[0] & 0x80));
} 
EXPORT_SYMBOL(set_tsp_for_prox_enable);
static int __devinit synaptics_ts_init(void)
{
	printk("[TSP] %s\n", __func__ );
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		return -ENOMEM;

	check_ic_wq = create_singlethread_workqueue("check_ic_wq");	
	if (!check_ic_wq)
		return -ENOMEM;
	
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);

	if (check_ic_wq)
		destroy_workqueue(check_ic_wq);
	
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i2c_addr = 0x1C;
	uint8_t buf_tmp[2] = {0};
	int phone_ver = 0;

	printk("[TSP] %s\n",__func__);

	 		
	if ( HW_ver == 1  || HW_ver == 2 || HW_ver == 3)
	{
		/* for glass */
		phone_ver = 1;  /* SW Ver.4 - change this value if New firmware be released */ 	
	}
	else if ( HW_ver == 17 )
	{
		/* for glass */
		phone_ver = 1;  /* SW Ver.4 - change this value if New firmware be released */ 	
	}
	else if ( HW_ver == 33 )
	{
		/* for glass */
		phone_ver = 1;  /* SW Ver.4 - change this value if New firmware be released */ 	
	}
	else
	{
		phone_ver = 2; // Acryl type
		printk("[TSP] %s:%d,HW_ver is wrong!!\n", __func__,__LINE__ );
	}
	
	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));
	printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, buf_tmp[1], buf_tmp[0] );

	/* below protocol is defined with App. ( juhwan.jeong@samsung.com )
		The TSP Driver report like XY as decimal.
		The X is the Firmware version what phone has.
		The Y is the Firmware version what TSP has. */

	if ( buf_tmp[0] > 0x9 )
		sprintf(buf, "%x%x%x\n", phone_ver, buf_tmp[0], buf_tmp[1]  );
	else
		sprintf(buf, "%x0%x%x\n", phone_ver, buf_tmp[0],buf_tmp[1] );

	return sprintf(buf, "%s", buf );
}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	firmware_ret_val = -1;

	if ( value == 1 )
	{
		printk("[TSP] Firmware update start!!\n" );

		if(HW_ver >= 2)
			firm_update( );
		else
			firmware_ret_val = 0; // firm_update not executed, failed.
			
#if FIRM_TEST
		printk("[TSP] start update cypress touch firmware !!\n");
		g_FirmwareImageSize = CYPRESS_FIRMWARE_IMAGE_SIZE;

		if(g_pTouchFirmware == NULL)
		{
			printk("[TSP][ERROR] %s() kmalloc fail !! \n", __FUNCTION__);
			return -1;
		}


		/* ready for firmware code */
		size = issp_request_firmware("touch.hex");

		/* firmware update */
		//	issp_upgrade();

		g_FirmwareImageSize = 0;

		// step.1 power off/on

		// step.2 enable irq


#endif
		return size;
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk("[TSP] %s!\n", __func__);

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk("[TSP] %s, operate nothing!\n", __func__);

	return size;
}


int firm_update( void )
{
	int ret;
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	printk("[TSP] disable_irq : %d\n", __LINE__ );
	disable_irq(ts_global->client->irq);
	local_irq_disable();
	
	// TEST
	//gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
	gpio_tlmm_config(GPIO_CFG( TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);

	ret = cancel_work_sync(&ts_global->work_timer);
	hrtimer_cancel(&ts_global->timer);

	firmware_ret_val = cypress_update( HW_ver );

/*	if( HW_ver==1 || HW_ver==2 ||HW_ver==3 )
	{
		firmware_ret_val = cypress_update( HW_ver );
	}
	else	
	{
		printk(KERN_INFO "[TSP] %s, %d cypress_update blocked, HW ver=%d\n", __func__, __LINE__, HW_ver);
		firmware_ret_val = 0; // Fail
	}
*/
	msleep(1000);
	if( firmware_ret_val )
		printk(KERN_INFO "[TSP] %s success, %d\n", __func__, __LINE__);
	else	
		printk(KERN_INFO "[TSP] %s fail, %d\n", __func__, __LINE__);

	//gpio_configure( TSP_SCL, GPIOF_DRIVE_OUTPUT );
	gpio_tlmm_config(GPIO_CFG( TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE);

	printk("[TSP] enable_irq : %d\n", __LINE__ );
	local_irq_enable();

	enable_irq(ts_global->client->irq);

	prev_wdog_val = -1;

	if(HW_ver >= 1)
		hrtimer_start(&ts_global->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	return 0;
} 

#if FIRM_TEST
static void issp_request_firmware(char* update_file_name)
{
	int idx_src = 0;
	int idx_dst = 0;
	int line_no = 0;
	int dummy_no = 0;
	char buf[2];
	int ret = 0;

	struct device *dev = &ts_global->input_dev->dev;	
	const struct firmware * fw_entry;

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	printk("[TSP] firmware file name : %s\n", update_file_name);

	ret = request_firmware(&fw_entry, update_file_name, dev);
	if ( ret )
	{
		printk("[TSP] request_firmware fail, ln=%d\n", ret );
		return ;
	}
	else
	{
		printk("[TSP] request_firmware success, ln=%d\n", ret );
		printk("[TSP][DEBUG] ret=%d, firmware size=%d\n", ret, fw_entry->size);
		printk("[TSP] %c %c %c %c %c\n", fw_entry->data[0], fw_entry->data[1], fw_entry->data[2], fw_entry->data[3], fw_entry->data[4]);
	}

	do {
		if(fw_entry->data[idx_src] == ':') // remove prefix
		{
			idx_src+=9;
			dummy_no++;

			if(dummy_no != line_no+1)
			{
				printk("[ERROR] Can not skip semicolon !! line_no(%d), dummy_no(%d)\n", line_no, dummy_no);
			}
		}
		else if(fw_entry->data[idx_src] == '\r') // return code
		{
			idx_src+=2; idx_dst--; line_no++;

			if( idx_dst > TSP_LINE_LENGTH*line_no)
			{
				printk("[ERROR] length buffer over error !! line_no(%d), idx_dst(%d)\n", line_no, idx_dst);
			}
		}
		else if(fw_entry->data[idx_src] == 0x0a) // return code
		{
			idx_src+=1; idx_dst--; line_no++;

			if( idx_dst > TSP_LINE_LENGTH*line_no)
			{
				printk("[ERROR] length buffer over error !! line_no(%d), idx_dst(%d)\n", line_no, idx_dst);
			}
		}
		else
		{
			sprintf(buf, "%c%c", fw_entry->data[idx_src], fw_entry->data[idx_src+1]);
			if(idx_dst > TSP_TOTAL_LINES*TSP_LINE_LENGTH)
			{
				printk("[ERROR] buffer over error !!  line_no(%d), idx_dst(%d)\n", line_no, idx_dst);
			}
			g_pTouchFirmware[idx_dst] = simple_strtol(buf, NULL, 16);
			idx_src+=2; idx_dst++;
		}
	} while ( line_no < TSP_TOTAL_LINES );

	release_firmware(fw_entry);
}
#endif

/* firmware - update */

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
