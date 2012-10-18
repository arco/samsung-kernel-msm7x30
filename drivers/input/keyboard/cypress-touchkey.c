/*
 * Copyright 2006-2010, Cypress Semiconductor Corporation.
 * Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/input/cypress-touchkey.h>
#include <mach/gpio.h>

#define TOUCH_UPDATE
#if defined(TOUCH_UPDATE)
#include <linux/irq.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#endif

#include <linux/wakelock.h>

#define SCANCODE_MASK		0x07
#define UPDOWN_EVENT_MASK	0x08
#define ESD_STATE_MASK		0x10

#define BACKLIGHT_ON		0x1
#define BACKLIGHT_OFF		0x2

#define DEVICE_NAME "melfas_touchkey"

int bl_on = 0;
static DECLARE_MUTEX(enable_sem);
static DECLARE_MUTEX(i2c_sem);

static int bl_timeout = 1600; // This gets overridden by userspace AriesParts
static int bl_wakelock = 0;
static struct timer_list bl_timer;
static void bl_off(struct work_struct *bl_off_work);
static DECLARE_WORK(bl_off_work, bl_off);

#define _3_TOUCH_INT     84
#define _3_TOUCH_SCL_28V 124
#define _3_TOUCH_SDA_28V 125
#define _3_TOUCH_EN      126

#define _3_TOUCH_MAXKEYS 2

struct cypress_touchkey_devdata {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct touchkey_platform_data *pdata;
	struct early_suspend early_suspend;
	u8 backlight_on;
	u8 backlight_off;
	bool is_dead;
	bool is_powering_on;
	bool has_legacy_keycode;
	bool is_sleeping;
};

static struct cypress_touchkey_devdata *devdata_global;
static struct hrtimer cypress_wdog_timer;
static struct wake_lock bln_wake_lock;

static int touchkey_status[_3_TOUCH_MAXKEYS] ={0,};



static int i2c_touchkey_read_byte(struct cypress_touchkey_devdata *devdata,
					u8 *val)
{
	int ret;
	int retry = 5;

	down(&i2c_sem);

	while (true) {
		ret = i2c_smbus_read_byte(devdata->client);
		if (ret >= 0) {
			*val = ret;
                        up(&i2c_sem);
			return 0;
		}

		dev_err(&devdata->client->dev, "i2c read error\n");
		if (!retry--)
			break;
		msleep(10);
	}

        up(&i2c_sem);

	return ret;
}

static int i2c_touchkey_write_byte(struct cypress_touchkey_devdata *devdata,
					u8 val)
{
	int ret;
	int retry = 5;

        down(&i2c_sem);

	while (true) {
		ret = i2c_smbus_write_byte(devdata->client, val);
		if (!ret) {
			up(&i2c_sem);
			return 0;
		}

		dev_err(&devdata->client->dev, "i2c write error\n");
		if (!retry--)
			break;
		msleep(10);
	}

        up(&i2c_sem);

	return ret;
}

static void all_keys_up(struct cypress_touchkey_devdata *devdata)
{
	int i;

	for (i = 0; i < devdata->pdata->keycode_cnt; i++)
	{
		input_report_key(devdata->input_dev,
						devdata->pdata->keycode[i], 0);
		touchkey_status[i] = 0;
	}
	
	input_sync(devdata->input_dev);

	if(hrtimer_active(&cypress_wdog_timer))
		hrtimer_cancel(&cypress_wdog_timer);
}

static int recovery_routine(struct cypress_touchkey_devdata *devdata)
{
	int ret = -1;
	int retry = 10;
	u8 data;
	int irq_eint;

	if (unlikely(devdata->is_dead)) {
		dev_err(&devdata->client->dev, "%s: Device is already dead, "
				"skipping recovery\n", __func__);
		return -ENODEV;
	}

	irq_eint = devdata->client->irq;
        down(&enable_sem);

	all_keys_up(devdata);

	disable_irq_nosync(irq_eint);
	while (retry--) {
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
		ret = i2c_touchkey_read_byte(devdata, &data);
		if (!ret) {
			if (!devdata->is_sleeping)
				enable_irq(irq_eint);
			goto out;
		}
		dev_err(&devdata->client->dev, "%s: i2c transfer error retry = "
				"%d\n", __func__, retry);
	}
	devdata->is_dead = true;
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	dev_err(&devdata->client->dev, "%s: touchkey died\n", __func__);
out:
        up(&enable_sem);
	return ret;
}

static void bl_set_timeout(void) {
	if (bl_timeout > 0) {
		mod_timer(&bl_timer, jiffies + msecs_to_jiffies(bl_timeout));
	}
}

extern unsigned int touch_state_val;
extern void TSP_forced_release(void);

static irqreturn_t touchkey_interrupt_thread(int irq, void *touchkey_devdata)
{
	u8 data;
	int i;
	int ret;
	int scancode;
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;

	ret = i2c_touchkey_read_byte(devdata, &data);
	if (ret || (data & ESD_STATE_MASK)) {
		ret = recovery_routine(devdata);
		if (ret) {
			dev_err(&devdata->client->dev, "%s: touchkey recovery "
					"failed!\n", __func__);
			goto err;
		}
	}

	if (data & UPDOWN_EVENT_MASK) {
		scancode = (data & SCANCODE_MASK) - 1;

		if(touchkey_status[scancode] == 1)
		{		
			input_report_key(devdata->input_dev,
				devdata->pdata->keycode[scancode], 0);
			input_sync(devdata->input_dev);
			touchkey_status[scancode] = 0;
			
			if(hrtimer_active(&cypress_wdog_timer))
				hrtimer_cancel(&cypress_wdog_timer);

#ifdef KERNEL_DEBUG_SEC
			printk("[TSK]keycode : %d, state : %d	%d\n", devdata->pdata->keycode[scancode], 0, __LINE__);
#endif
		}
	} else {
		if (!touch_state_val) {
			if (devdata->has_legacy_keycode) {
				scancode = (data & SCANCODE_MASK) - 1;
				if (scancode < 0 || scancode >= devdata->pdata->keycode_cnt) {
					dev_err(&devdata->client->dev, "%s: scancode is out of "
						"range\n", __func__);
					goto err;
				}
				if (scancode == 1)
					TSP_forced_release();
				input_report_key(devdata->input_dev,
					devdata->pdata->keycode[scancode], 1);
				
				touchkey_status[scancode] = 1;
				hrtimer_start(&cypress_wdog_timer, ktime_set(30,0), HRTIMER_MODE_REL);
#ifdef KERNEL_DEBUG_SEC
				printk("[TSK]keycode : %d, state : %d	%d\n", devdata->pdata->keycode[scancode], 1, __LINE__);
#endif
			} else {
				for (i = 0; i < devdata->pdata->keycode_cnt; i++){
				input_report_key(devdata->input_dev,
					devdata->pdata->keycode[i],
					!!(data & (1U << i)));
				touchkey_status[i] = !!(data & (1U << i));
#ifdef KERNEL_DEBUG_SEC
				printk("[TSK]keycode : %d, state : %d	%d\n", devdata->pdata->keycode[i], !!(data & (1U << i)), __LINE__);
#endif
				}
			}
			input_sync(devdata->input_dev);
		}
	}

	bl_set_timeout();

err:
	return IRQ_HANDLED;
}

static irqreturn_t touchkey_interrupt_handler(int irq, void *touchkey_devdata)
{
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;

	if (devdata->is_powering_on) {
		dev_dbg(&devdata->client->dev, "%s: ignoring spurious boot "
					"interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_touchkey_early_suspend(struct early_suspend *h)
{
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

        down(&enable_sem);

	printk("[TSK] +%s\n", __func__);
	devdata->is_powering_on = true;

	if (unlikely(devdata->is_dead))
        {
		up(&enable_sem);
		return;
	}

	disable_irq(devdata->client->irq);

	if (!bl_on)
	       	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);

	all_keys_up(devdata);
	printk("[TSK] -%s\n", __func__);
        devdata->is_sleeping = true;

        up(&enable_sem);
}

static void cypress_touchkey_early_resume(struct early_suspend *h)
{
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

	printk("[TSK] +%s\n", __func__);

	// Avoid race condition with LED notification disable
	down(&enable_sem);

	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
#if 0
	if (i2c_touchkey_write_byte(devdata, devdata->backlight_on)) {
		devdata->is_dead = true;
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		dev_err(&devdata->client->dev, "%s: touch keypad not responding"
				" to commands, disabling\n", __func__);
		return;
	}
#endif
	devdata->is_dead = false;
	enable_irq(devdata->client->irq);
	devdata->is_powering_on = false;
	devdata->is_sleeping = false; 
	printk("[TSK] -%s\n", __func__);

	up(&enable_sem);
	bl_set_timeout();
}
#endif

#if defined(TOUCH_UPDATE)
extern int get_touchkey_firmware(char *version);
extern int ISSP_main(void);
static int touchkey_update_status = 0;
struct work_struct touch_update_work;
struct workqueue_struct *touchkey_wq;
#define IRQ_TOUCH_INT  MSM_GPIO_TO_INT(_3_TOUCH_INT) 
#define KEYCODE_REG 0x00
#define I2C_M_WR 0              /* for i2c */

static void init_hw(void)
{
	gpio_tlmm_config(GPIO_CFG(_3_TOUCH_EN, 0, GPIO_CFG_OUTPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(_3_TOUCH_EN, 1);
	msleep(200);
	irq_set_irq_type(IRQ_TOUCH_INT, IRQF_TRIGGER_FALLING);
}

static int touchkey_update_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t touchkey_update_read(struct file * filp, char *buf, size_t count,
			     loff_t * f_pos)
{
	char data[3] = { 0, };

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

#if 0
extern int mcsdl_download_binary_file(unsigned char *pData,
				      unsigned short nBinary_length);
ssize_t touchkey_update_write(struct file *filp, const char *buf, size_t count,
			      loff_t * f_pos)
{
	unsigned char *pdata;

	disable_irq(IRQ_TOUCH_INT);
	printk("count = %d\n", count);
	pdata = kzalloc(count, GFP_KERNEL);
	if (pdata == NULL) {
		printk("memory allocate fail \n");
		return 0;
	}
	if (copy_from_user(pdata, buf, count)) {
		printk("copy fail \n");
		kfree(pdata);
		return 0;
	}

	mcsdl_download_binary_file((unsigned char *)pdata,
				   (unsigned short)count);
	kfree(pdata);

	init_hw();
	enable_irq(IRQ_TOUCH_INT);
	return count;
}
#endif

static int touchkey_update_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations touchkey_update_fops = {
	.owner = THIS_MODULE,
	.read = touchkey_update_read,
      //.write   = touchkey_update_write,
	.open = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	//.name = "melfas_touchkey",DEVICE_NAME
	.name = DEVICE_NAME,
	.fops = &touchkey_update_fops,
};

static int i2c_touchkey_read(struct cypress_touchkey_devdata *devdata, 
					u8 reg, u8 * val, unsigned int len)
{
	int err;
	int retry = 10;
	struct i2c_msg msg[1];
        
        down(&i2c_sem);

	while (retry--) {
		msg->addr = devdata->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(devdata->client->adapter, msg, 1);
		if (err >= 0) {
			up(&i2c_sem);
			return 0;
		}
		printk("%s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
		mdelay(10);
	}

        up(&i2c_sem);

	return err;

}

static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;
	struct cypress_touchkey_devdata *devdata;

//	devdata = dev->platform_data;
	devdata = devdata_global;

	init_hw();

//	if (get_touchkey_firmware(data) != 0)
	i2c_touchkey_read(devdata, KEYCODE_REG, data, 3);

	count = sprintf(buf, "0x%x\n", data[1]);

	printk("touch_version_read 0x%x\n", data[1]);
	return count;
}

static ssize_t touch_version_write(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}

static void touchkey_update_func(struct work_struct *p)
{
	int retry = 10;
	touchkey_update_status = 1;
	printk("%s start\n", __FUNCTION__);
	while (retry--) {
		if (ISSP_main() == 0) {
			touchkey_update_status = 0;
			printk("touchkey_update succeeded\n");
			enable_irq(IRQ_TOUCH_INT);
			return;
		}
	}
	touchkey_update_status = -1;
	printk("touchkey_update failed\n");
	return;
}

static ssize_t touch_update_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	printk("touchkey firmware update \n");
	if (*buf == 'S') {
		disable_irq(IRQ_TOUCH_INT);
		INIT_WORK(&touch_update_work, touchkey_update_func);
		queue_work(touchkey_wq, &touch_update_work);
	}
	return size;
}

static ssize_t touch_update_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk("touch_update_read: touchkey_update_status %d\n",
	       touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "FAIL\n");
	}

	return count;
}

static int i2c_touchkey_write(struct cypress_touchkey_devdata *devdata, 
						u8 * val, unsigned int len)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 2;

        down(&i2c_sem);

	while (retry--) {
		data[0] = *val;
		msg->addr = devdata->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = data;
		err = i2c_transfer(devdata->client->adapter, msg, 1);
		if (err >= 0)
		{
			up(&i2c_sem);
			return 0;
		}
		printk(KERN_DEBUG "%s %d i2c transfer error\n", __func__,
		       __LINE__);
		mdelay(10);
	}

        up(&i2c_sem);

	return err;
}

struct cypress_touchkey_devdata *tempdata;
static ssize_t touch_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	u8 data = 0x10;
	if (sscanf(buf, "%d\n", (int *)&data) == 1) {
		if (!tempdata->is_powering_on && !tempdata->is_sleeping) {
			//printk(KERN_DEBUG "touch_led_control: %d \n", data);
			if (data || !bl_on) // Deactivate led only if BLN is inactive
				i2c_touchkey_write(tempdata, &data, sizeof(u8));
		}
	} else
		printk("touch_led_control Error\n");

	return size;
}

static ssize_t touchkey_enable_disable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
#if 0
	printk("touchkey_enable_disable %c \n", *buf);
	if (*buf == '0') {
		set_touchkey_debug('d');
		disable_irq(IRQ_TOUCH_INT);
		gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
		touchkey_enable = -2;
	} else if (*buf == '1') {
		if (touchkey_enable == -2) {
			set_touchkey_debug('e');
			gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
			touchkey_enable = 1;
			enable_irq(IRQ_TOUCH_INT);
		}
	} else {
		printk("touchkey_enable_disable: unknown command %c \n", *buf);
	}
#endif
	return size;
}

static DEVICE_ATTR(touch_version, 0664,
		   touch_version_read, touch_version_write);
static DEVICE_ATTR(touch_update,0664,
		   touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, 0664, NULL,
		   touch_led_control);
static DEVICE_ATTR(enable_disable, 0664, NULL,
		   touchkey_enable_disable);

#endif
static enum hrtimer_restart cypress_wdog_timer_func(struct hrtimer *timer)
{
	int i;

	for( i = 0 ; i < _3_TOUCH_MAXKEYS ; i++ )
	{
		if(touchkey_status[i] == 1)
		{
			input_report_key(devdata_global->input_dev, devdata_global->pdata->keycode[i], 0);
			touchkey_status[i] = 0;
#ifdef KERNEL_DEBUG_SEC
			printk("[TSK]keycode : %d, state : %d	%d\n", devdata_global->pdata->keycode[i], 0, __LINE__);
#endif
		}
	}
		
	return HRTIMER_NORESTART;
}

static void notify_led_on(void) {
	if (unlikely(devdata_global->is_dead) || bl_on) {
		return;
        }

        down(&enable_sem);

	if (devdata_global->is_sleeping) {
	//	devdata_global->pdata->touchkey_sleep_onoff(TOUCHKEY_ON);
		devdata_global->pdata->touchkey_onoff(TOUCHKEY_ON);
	}
	i2c_touchkey_write_byte(devdata_global, devdata_global->backlight_on);
	bl_on = 1;

   if( !wake_lock_active(&bln_wake_lock) && bl_wakelock ){
        printk(KERN_DEBUG "[TouchKey] touchkey get wake_lock\n");
        wake_lock(&bln_wake_lock);
    }

        up(&enable_sem);

	bl_set_timeout();

	printk(KERN_DEBUG "%s: notification led enabled\n", __FUNCTION__);
}

static void notify_led_off(void) {
	if (unlikely(devdata_global->is_dead) || !bl_on) {
		return;
        }

	// Avoid race condition with touch key resume
	down(&enable_sem);

	if (bl_on && bl_timer.expires < jiffies) // Don't disable if there's a timer scheduled
		i2c_touchkey_write_byte(devdata_global, devdata_global->backlight_off);

	//devdata_global->pdata->touchkey_sleep_onoff(TOUCHKEY_OFF);
	if (devdata_global->is_sleeping)
		devdata_global->pdata->touchkey_onoff(TOUCHKEY_OFF);

	bl_on = 0;

	/* we were using a wakelock, unlock it */
    if( wake_lock_active(&bln_wake_lock) ){
        printk(KERN_DEBUG "[TouchKey] touchkey clear wake_lock\n");
        wake_unlock(&bln_wake_lock);
    }


	up(&enable_sem);

	printk(KERN_DEBUG "%s: notification led disabled\n", __FUNCTION__);
}

static void bl_off(struct work_struct *bl_off_work)
{
	notify_led_off();
}

void bl_timer_callback(unsigned long data)
{
	schedule_work(&bl_off_work);
}

static ssize_t led_status_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%u\n", bl_on);
}

static ssize_t led_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data)) {
		if (data == 1)
			notify_led_on();
		else
			notify_led_off();
	}
	return size;
}

static ssize_t bl_timeout_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%d\n", bl_timeout);
}

static ssize_t bl_timeout_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &bl_timeout);
	return size;
}

static ssize_t bl_wakelock_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%d\n", bl_wakelock);
}

static ssize_t bl_wakelock_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &bl_wakelock);
	return size;
}

static DEVICE_ATTR(led, S_IRUGO | S_IWUGO , led_status_read, led_status_write);
static DEVICE_ATTR(bl_timeout, S_IRUGO | S_IWUGO, bl_timeout_read, bl_timeout_write);
static DEVICE_ATTR(wakelock, S_IRUGO | S_IWUGO, bl_wakelock_read, bl_wakelock_write);

static struct attribute *bl_led_attributes[] = {
		&dev_attr_led.attr,
		&dev_attr_bl_timeout.attr, // Not the best place, but creating a new device is more trouble that it's worth
		&dev_attr_wakelock.attr,
		NULL
};

static struct attribute_group bl_led_group = {
		.attrs  = bl_led_attributes,
};

static struct miscdevice bl_led_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "notification",
};

extern int charging_boot;
static int cypress_touchkey_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	struct cypress_touchkey_devdata *devdata;
	u8 data[3];
	int err;
	int cnt;
#if defined(TOUCH_UPDATE)
	int ret;
	int retry = 10;
#endif

	if (!dev->platform_data) {
		dev_err(dev, "%s: Platform data is NULL\n", __func__);
		return -EINVAL;
	}

	devdata = kzalloc(sizeof(*devdata), GFP_KERNEL);
	if (devdata == NULL) {
		dev_err(dev, "%s: failed to create our state\n", __func__);
		return -ENODEV;
	}

	// release timer, case of TSK never release
	hrtimer_init(&cypress_wdog_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cypress_wdog_timer.function = cypress_wdog_timer_func;

	devdata->client = client;
	i2c_set_clientdata(client, devdata);

	devdata_global = devdata;
	
	devdata->pdata = client->dev.platform_data;
#if defined(TOUCH_UPDATE)
	tempdata = devdata;
#endif
	if (!devdata->pdata->keycode) {
		dev_err(dev, "%s: Invalid platform data\n", __func__);
		err = -EINVAL;
		goto err_null_keycodes;
	}

	strlcpy(devdata->client->name, DEVICE_NAME, I2C_NAME_SIZE);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_input_alloc_dev;
	}

	devdata->input_dev = input_dev;
	dev_set_drvdata(&input_dev->dev, devdata);
	input_dev->name = DEVICE_NAME;
	input_dev->id.bustype = BUS_HOST;

	for (cnt = 0; cnt < devdata->pdata->keycode_cnt; cnt++)
		input_set_capability(input_dev, EV_KEY,
					devdata->pdata->keycode[cnt]);

	err = input_register_device(input_dev);
	if (err)
		goto err_input_reg_dev;

	devdata->is_powering_on = true;
        devdata->is_sleeping = false;

	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);

	err = i2c_master_recv(client, data, sizeof(data));
	if (err < sizeof(data)) {
		if (err >= 0)
			err = -EIO;
		dev_err(dev, "%s: error reading hardware version\n", __func__);
		goto err_read;
	}

	dev_info(dev, "%s: hardware rev1 = %#02x, rev2 = %#02x\n", __func__,
				data[1], data[2]);

	devdata->backlight_on = BACKLIGHT_ON;
	devdata->backlight_off = BACKLIGHT_OFF;

	devdata->has_legacy_keycode = 1;
#if 0
	err = i2c_touchkey_write_byte(devdata, devdata->backlight_on);
	if (err) {
		dev_err(dev, "%s: touch keypad backlight on failed\n",
				__func__);
		goto err_backlight_on;
	}
#endif
	if (request_threaded_irq(client->irq, touchkey_interrupt_handler,
				touchkey_interrupt_thread, IRQF_TRIGGER_FALLING,
				DEVICE_NAME, devdata)) {
		dev_err(dev, "%s: Can't allocate irq.\n", __func__);
		goto err_req_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	devdata->early_suspend.suspend = cypress_touchkey_early_suspend;
	devdata->early_suspend.resume = cypress_touchkey_early_resume;
#endif
	register_early_suspend(&devdata->early_suspend);

	devdata->is_powering_on = false;
#if defined(TOUCH_UPDATE)
	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk("%s misc_register fail\n", __FUNCTION__);
	}

	dev_set_drvdata(touchkey_update_device.this_device, devdata);

	/* wake lock for LED Notify */
    wake_lock_init(&bln_wake_lock, WAKE_LOCK_SUSPEND, "bln_wake_lock");

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_touch_version) < 0) {
		printk("%s device_create_file fail dev_attr_touch_version\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_touch_version.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_touch_update) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_touch_update.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_brightness) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_brightness.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device,
	     &dev_attr_enable_disable) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n",
		       __FUNCTION__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_enable_disable.attr.name);
	}

	touchkey_wq = create_singlethread_workqueue(DEVICE_NAME);
	if (!touchkey_wq)
		return -ENOMEM;

	while (retry--) {
		if (get_touchkey_firmware(data) == 0)	//melfas need delay for multiple read
			break;
	}
	printk("%s F/W version: 0x%x, Module version:0x%x\n", __FUNCTION__,
	       data[1], data[2]);
#endif

	if (charging_boot)
	{	
 	   	printk("cypress_touchkey_probe skip (LPM mode)\n");		

		devdata->is_powering_on = true;

		if (unlikely(devdata->is_dead))
			return -EINVAL;

		disable_irq(devdata->client->irq);
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		all_keys_up(devdata);
	}

	if (misc_register(&bl_led_device))
		printk("%s misc_register(%s) failed\n", __FUNCTION__, bl_led_device.name);
	else {
		if (sysfs_create_group(&bl_led_device.this_device->kobj, &bl_led_group) < 0)
			pr_err("failed to create sysfs group for device %s\n", bl_led_device.name);
	}

	setup_timer(&bl_timer, bl_timer_callback, 0);


	return 0;

err_req_irq:
#if 0
err_backlight_on:
#endif
err_read:
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	input_unregister_device(input_dev);
	goto err_input_alloc_dev;
err_input_reg_dev:
	input_free_device(input_dev);
err_input_alloc_dev:
err_null_keycodes:
	kfree(devdata);
	return err;
}

static int __devexit i2c_touchkey_remove(struct i2c_client *client)
{
	struct cypress_touchkey_devdata *devdata = i2c_get_clientdata(client);

	misc_deregister(&bl_led_device);
	wake_lock_destroy(&bln_wake_lock);

	unregister_early_suspend(&devdata->early_suspend);
	/* If the device is dead IRQs are disabled, we need to rebalance them */
	if (unlikely(devdata->is_dead))
		enable_irq(client->irq);
	else
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	free_irq(client->irq, devdata);
	all_keys_up(devdata);
	input_unregister_device(devdata->input_dev);
	del_timer(&bl_timer);
	kfree(devdata);
	return 0;
}

static const struct i2c_device_id cypress_touchkey_id[] = {
	{ CYPRESS_TOUCHKEY_DEV_NAME, 0 },
};

MODULE_DEVICE_TABLE(i2c, cypress_touchkey_id);

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "cypress_touchkey_driver",
	},
	.id_table = cypress_touchkey_id,
	.probe = cypress_touchkey_probe,
	.remove = __devexit_p(i2c_touchkey_remove),
};

static int __init touchkey_init(void)
{
	int ret = 0;
	//int retry = 3;
#if 0
	//update version "eclair/vendor/samsung/apps/Lcdtest/src/com/sec/android/app/lcdtest/touch_firmware.java"
	//if ((data[1] >= 0xa1) && (data[1] < 0xa9)) {
		//set_touchkey_debug('U');
		while (retry--) {
			if (ISSP_main() == 0) {
				printk("touchkey_update succeeded\n");
				//set_touchkey_debug('C');
				break;
			}
			printk("touchkey_update failed... retry...\n");
			//set_touchkey_debug('f');
		}
		if (retry <= 0) {
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
#if !defined(CONFIG_ARIES_NTT)
			gpio_direction_output(_3_GPIO_TOUCH_CE, 0);
#endif
			msleep(300);
		}
		init_hw();	//after update, re initalize.
	//}
#endif
	ret = i2c_add_driver(&touchkey_i2c_driver);
	if (ret)
		pr_err("%s: cypress touch keypad registration failed. (%d)\n",
				__func__, ret);

	return ret;
}

static void __exit touchkey_exit(void)
{
#if defined(TOUCH_UPDATE)
	misc_deregister(&touchkey_update_device);
#endif
	i2c_del_driver(&touchkey_i2c_driver);
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("cypress touch keypad");

