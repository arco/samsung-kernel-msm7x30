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
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/input/cypress-touchkey.h>
#include <mach/gpio.h>

#define TOUCH_UPDATE
#if defined(TOUCH_UPDATE)
#include <linux/irq.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#endif

#ifdef CONFIG_GENERIC_BLN
#include <linux/bln.h>
#endif

#include <linux/wakelock.h>

#ifdef CONFIG_PHANTOM_KP_FILTER
#include <linux/phantom_kp_filter.h>
#endif

#define SCANCODE_MASK		0x07
#define UPDOWN_EVENT_MASK	0x08
#define ESD_STATE_MASK		0x10

#define BACKLIGHT_ON		0x10
#define BACKLIGHT_OFF		0x20

#define OLD_BACKLIGHT_ON	0x1
#define OLD_BACKLIGHT_OFF	0x2

#define DEVICE_NAME "melfas_touchkey"

/* Macro and attributes for timeout management of touchkeys backlight */
#define DEFAULT_BACKLIGHT_TIMEOUT	1600
static int backlight_timeout = DEFAULT_BACKLIGHT_TIMEOUT;
static struct timer_list backlight_timer;
static void backlight_off(struct work_struct *backlight_off_work);
static DECLARE_WORK(backlight_off_work, backlight_off);

#ifdef CONFIG_PHANTOM_KP_FILTER
/* Possible key event codes */
enum key_data_code {
	KEY_MENU_DOWN = 0x1,
	KEY_BACK_DOWN = 0x2,
	KEY_NOT_VALID = 0x7,
	KEY_MENU_UP   = 0x9,
	KEY_BACK_UP   = 0xA
};

u8 prev_data = KEY_NOT_VALID;		/* Previous key event */
unsigned int keypress_errors = 0;	/* Key press errors before a valid key press */
unsigned long first_error_time;		/* Time (jiffies) of the first key press error occurrence */
unsigned long last_error_time;		/* Time (jiffies) of the last key press error occurrence */
#endif

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

static int touchkey_status[_3_TOUCH_MAXKEYS] = {0,};

static int i2c_touchkey_read_byte(struct cypress_touchkey_devdata *devdata, u8 *val)
{
	int ret;
	int retry = 2;

	while (true) {
		ret = i2c_smbus_read_byte(devdata->client);
		if (ret >= 0) {
			*val = ret;
			ret = 0;
			break;
		}

		if (!retry--) {
			dev_err(&devdata->client->dev, "i2c read error\n");
			break;
		}
		msleep(10);
	}

	return ret;
}

static int i2c_touchkey_write_byte(struct cypress_touchkey_devdata *devdata, u8 val)
{
	int ret;
	int retry = 2;

	while (true) {
		ret = i2c_smbus_write_byte(devdata->client, val);
		if (!ret) {
			ret = 0;
			break;
		}

		if (!retry--) {
			dev_err(&devdata->client->dev, "i2c write error\n");
			break;
		}
		msleep(10);
	}

	return ret;
}

static void all_keys_up(struct cypress_touchkey_devdata *devdata)
{
	int i;

	for (i = 0; i < devdata->pdata->keycode_cnt; i++) {
		input_report_key(devdata->input_dev,
						 devdata->pdata->keycode[i], 0);
		touchkey_status[i] = 0;
	}

	input_sync(devdata->input_dev);

	if (hrtimer_active(&cypress_wdog_timer))
		hrtimer_cancel(&cypress_wdog_timer);
}

/*
 * Turns off the touchkeys backlight
 */
static void backlight_off(struct work_struct *backlight_off_work) {
	bool bln = false;

#ifdef CONFIG_GENERIC_BLN
	/* Check if there are notifications */
	bln = bln_is_ongoing();
#endif

	/* Don't turn off the backlight if there is a notification or
	 * if the device is dead, is powering or is sleeping */
	if (bln || devdata_global == NULL || unlikely(devdata_global->is_dead) ||
		devdata_global->is_powering_on || devdata_global->is_sleeping) {
		pr_debug("%s: TouchKey backlight off not performed...", __func__);
		return;
	}

	/* Turn off the touchkeys backlight */
	i2c_touchkey_write_byte(devdata_global, devdata_global->backlight_off);
}

/*
 * Manages the timeout of touchkeys backlight by turning off the leds
 */
void backlight_timer_callback(unsigned long data) {
	schedule_work(&backlight_off_work);
}

/*
 * Sets the timeout of touchkeys backlight
 */
static void backlight_set_timeout(void) {
	/* If the timeout is greater than 0, then set the timer */
	if (backlight_timeout > 0)
		mod_timer(&backlight_timer, jiffies + msecs_to_jiffies(backlight_timeout));
	/* Else directly call the timeout management */
	else
		backlight_timer_callback(0);
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

	all_keys_up(devdata);

	disable_irq_nosync(irq_eint);
	while (retry--) {
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
		ret = i2c_touchkey_read_byte(devdata, &data);
		if (!ret) {
			if (!devdata->is_sleeping) {
				enable_irq(irq_eint);
			}
			goto out;
		}
		dev_err(&devdata->client->dev, "%s: i2c transfer error retry = "
				"%d\n", __func__, retry);
	}
	devdata->is_dead = true;
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	dev_err(&devdata->client->dev, "%s: touchkey died\n", __func__);
out:
	dev_err(&devdata->client->dev, "%s: recovery_routine\n", __func__);
	return ret;
}

extern unsigned int touch_state_val;
extern void TSP_forced_release(void);

#ifdef CONFIG_PHANTOM_KP_FILTER
/*
 * Function to check if the incoming interrupt from the melfas chip is valid
 * Returns false for the ones caused by the radio high activity interference (bad signal)
 */
static bool is_valid_interrupt(void)
{
	int val;
	u8 i;

	/* Check the RFI/EMI generated interrupts for phantom_interrupt_checks times */
	for (i = 0; i < pkf_menuback->interrupt_checks; i++) {
		/* Get the _3_TOUCH_INT interrupt value */
		val = gpio_get_value(_3_TOUCH_INT);

		/* If the input pin is not zero then it's not a valid interrupt from
		 * the chip and should be ignored */
		if (val & 1) return false;
	}

	return true;
}

/*
 * Function to check if the incoming key event is valid
 */
static bool is_valid_key_press(u8 data)
{
	switch (data) {
		/* If it's a key down event from MENU or BACK key */
		case KEY_MENU_DOWN:
		case KEY_BACK_DOWN:
			/* Check if the previous event was a key up */
			if (prev_data == KEY_MENU_UP || prev_data == KEY_BACK_UP)
				return true;
			/* If there are multiple rapid errors, avoid key presses if within
			 * wait_time ms of the first error or if within wait_time ms of the last error */
			else
				return !!(keypress_errors <= 1 ||
						  (time_after(jiffies, first_error_time + msecs_to_jiffies(pkf_menuback->first_err_wait)) &&
						   time_after(jiffies, last_error_time + msecs_to_jiffies(pkf_menuback->last_err_wait))));
		/* If it's a key up event from MENU or BACK key */
		case KEY_MENU_UP:
		case KEY_BACK_UP:
			/* Check if the previous event was a key down of the same key */
			return !!((prev_data == KEY_MENU_DOWN || prev_data == KEY_BACK_DOWN) &&
					  (data - prev_data) == UPDOWN_EVENT_MASK);
		default:
			/* Unknown key event, this is not a valid key */
			return false;
	}
}
#endif

static irqreturn_t touchkey_interrupt_thread(int irq, void *touchkey_devdata)
{
	u8 data = 0xff;
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

	if (devdata->has_legacy_keycode) {
		scancode = (data & SCANCODE_MASK) - 1;
		if (scancode < 0 || scancode >= devdata->pdata->keycode_cnt) {
			dev_err(&devdata->client->dev, "%s: scancode %d is out of "
						"range, data = %d\n", __func__, scancode, data);
			goto err;
		}

#ifdef CONFIG_PHANTOM_KP_FILTER
		if (pkf_menuback->enabled) {
			if (is_valid_key_press(data)) {
				keypress_errors = 0;
				prev_data = data;
			} else {
				dev_dbg(&devdata->client->dev, "%s: Not valid key press "
							"(data = %d)\n", __func__, data);
				goto err;
			}
		}
#endif

		/* Don't send down event while the touch screen is being pressed
		 * to prevent accidental touch key hit.
		 */
		if ((data & UPDOWN_EVENT_MASK) || !touch_state_val) {
			if (scancode == 1)
				TSP_forced_release();
			input_report_key(devdata->input_dev,
							 devdata->pdata->keycode[scancode],
							 !(data & UPDOWN_EVENT_MASK));

			touchkey_status[scancode] = 1;
			hrtimer_start(&cypress_wdog_timer, ktime_set(30,0), HRTIMER_MODE_REL);
#ifdef KERNEL_DEBUG_SEC
			printk("[TSK]keycode : %d, state : %d	%d\n", devdata->pdata->keycode[scancode], 1, __LINE__);
#endif
		}
	} else {
		for (i = 0; i < devdata->pdata->keycode_cnt; i++) {
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

	/* Set the timeout of touchkeys backlight */
	backlight_set_timeout();

	return IRQ_HANDLED;
err:
#ifdef CONFIG_PHANTOM_KP_FILTER
	if (pkf_menuback->enabled) {
		count_ignored_menuback_kp();
		prev_data = KEY_NOT_VALID;
		keypress_errors++;
		if (keypress_errors == 1) first_error_time = jiffies;
		last_error_time = jiffies;
	}
#endif

	return IRQ_HANDLED;
}

static irqreturn_t touchkey_interrupt_handler(int irq, void *touchkey_devdata)
{
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;

#ifdef CONFIG_PHANTOM_KP_FILTER
	/* Prevent phantom key press caused by bad interrupt */
	if (pkf_menuback->enabled && !is_valid_interrupt()) {
		count_ignored_menuback_kp();
		dev_dbg(&devdata->client->dev, "%s: possible phantom key press, "
					"ignoring bad interrupt\n", __func__);
		return IRQ_HANDLED;
	}
#endif

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

	printk("[TSK] +%s\n", __func__);

	devdata->is_powering_on = true;

	if (unlikely(devdata->is_dead)) {
		return;
	}

	disable_irq(devdata->client->irq);

#ifdef CONFIG_GENERIC_BLN
	/*
	 * Disallow powering off the touchkey controller
	 * while a led notification is ongoing
	 */
	if (!bln_is_ongoing())
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
#endif

	all_keys_up(devdata);
	devdata->is_sleeping = true;
	printk("[TSK] -%s\n", __func__);
}

static void cypress_touchkey_early_resume(struct early_suspend *h)
{
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

	printk("[TSK] +%s\n", __func__);

	// Avoid race condition with LED notification disable
	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
	if (i2c_touchkey_write_byte(devdata, devdata->backlight_on)) {
		devdata->is_dead = true;
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		dev_err(&devdata->client->dev, "%s: touch keypad not responding"
				" to commands, disabling\n", __func__);
		return;
	}
	devdata->is_dead = false;
	enable_irq(devdata->client->irq);
	devdata->is_powering_on = false;
	devdata->is_sleeping = false;

	/* Set the timeout of touchkeys backlight */
	backlight_set_timeout();

#ifdef CONFIG_GENERIC_BLN
	/* release the possible pending wakelock for bln */
	bln_wakelock_release();
#endif
}
#endif

/*
 * Function to get the timeout of touchkeys backlight
 */
static ssize_t backlight_timeout_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%d\n", backlight_timeout);
}

/*
 * Function to set the timeout of touchkeys backlight
 */
static ssize_t backlight_timeout_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%d\n", &backlight_timeout);
	pr_info("%s: TouchKey backlight timeout changed to %d\n",__func__, backlight_timeout);
	return size;
}

static DEVICE_ATTR(bl_timeout, S_IRUGO | S_IWUGO, backlight_timeout_read, backlight_timeout_write);

static struct attribute *backlight_attributes[] = {
	&dev_attr_bl_timeout.attr,
	NULL
};

static struct attribute_group backlight_group = {
		.attrs  = backlight_attributes,
};

static struct miscdevice backlight_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "notification",
};

#ifdef CONFIG_GENERIC_BLN
static void enable_touchkey_backlights(void)
{
	i2c_touchkey_write_byte(devdata_global, devdata_global->backlight_on);
}

static void disable_touchkey_backlights(void)
{
	i2c_touchkey_write_byte(devdata_global, devdata_global->backlight_off);
}

static void cypress_touchkey_enable_led_notification(void)
{
	/* is_powering_on signals whether touchkey lights are used for touchmode */
	if (devdata_global->is_powering_on) {
		/* acquire wakelock for bln */
		bln_wakelock_acquire();

		/*
		 * power on the touchkey controller
		 * This is actually not needed, but it is intentionally
		 * left for the case that the early_resume() function
		 * did not power on the touchkey controller for some reasons
		 */
		devdata_global->pdata->touchkey_onoff(TOUCHKEY_ON);

		/* write to i2cbus, enable backlights */
		enable_touchkey_backlights();
	} else
		pr_info("%s: cannot set notification led, touchkeys are enabled\n", __func__);
}

static void cypress_touchkey_disable_led_notification(void)
{
	/* if touchkeys lights are not used for touchmode */
	if (devdata_global->is_powering_on) {
		disable_touchkey_backlights();

		/* we were using a wakelock, unlock it */
		bln_wakelock_release();
	}
}

static struct bln_implementation cypress_touchkey_bln = {
	.enable = cypress_touchkey_enable_led_notification,
	.disable = cypress_touchkey_disable_led_notification,
};
#endif

#if defined(TOUCH_UPDATE)
extern int get_touchkey_firmware(char *version);
extern int ISSP_main(void);
static int touchkey_update_status = 0;
struct work_struct touch_update_work;
struct workqueue_struct *touchkey_wq;
#define IRQ_TOUCH_INT  MSM_GPIO_TO_INT(_3_TOUCH_INT) 
#define KEYCODE_REG 0x00
#define I2C_M_WR 0	/* for i2c */

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

static ssize_t touchkey_update_read(struct file * filp, char *buf, size_t count, loff_t * f_pos)
{
	char data[3] = { 0, };

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

static int touchkey_update_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations touchkey_update_fops = {
	.owner = THIS_MODULE,
	.read = touchkey_update_read,
	.open = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &touchkey_update_fops,
};

static int i2c_touchkey_read(struct cypress_touchkey_devdata *devdata, u8 reg, u8 * val, unsigned int len)
{
	int err;
	int retry = 10;
	struct i2c_msg msg[1];

	while (retry--) {
		msg->addr = devdata->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(devdata->client->adapter, msg, 1);
		if (err >= 0) {
			return 0;
		}
		printk("%s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
		mdelay(10);
	}

	return err;

}

static ssize_t touch_version_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;
	struct cypress_touchkey_devdata *devdata;

	devdata = devdata_global;

	init_hw();

	i2c_touchkey_read(devdata, KEYCODE_REG, data, 3);

	count = sprintf(buf, "0x%x\n", data[1]);

	printk("touch_version_read 0x%x\n", data[1]);
	return count;
}

static ssize_t touch_version_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}

static void touchkey_update_func(struct work_struct *p)
{
	int retry = 10;
	touchkey_update_status = 1;
	printk("%s start\n", __func__);
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

static ssize_t touch_update_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("touchkey firmware update \n");
	if (*buf == 'S') {
		disable_irq(IRQ_TOUCH_INT);
		INIT_WORK(&touch_update_work, touchkey_update_func);
		queue_work(touchkey_wq, &touch_update_work);
	}
	return size;
}

static ssize_t touch_update_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk("touch_update_read: touchkey_update_status %d\n", touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "FAIL\n");
	}

	return count;
}

static int i2c_touchkey_write(struct cypress_touchkey_devdata *devdata,  u8 * val, unsigned int len)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 2;

	while (retry--) {
		data[0] = *val;
		msg->addr = devdata->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = data;
		err = i2c_transfer(devdata->client->adapter, msg, 1);
		if (err >= 0)
			return 0;

		printk(KERN_DEBUG "%s %d i2c transfer error\n", __func__, __LINE__);
		mdelay(10);
	}

	return err;
}

static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 data = 0x10;
	if (sscanf(buf, "%d\n", (int *)&data) == 1) {
		if (!devdata_global->is_powering_on && !devdata_global->is_sleeping) {
			if (data) // Deactivate led only if BLN is inactive
				i2c_touchkey_write(devdata_global, &data, sizeof(u8));
		}
	} else
		printk("touch_led_control Error\n");

	return size;
}

static ssize_t touchkey_enable_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(touch_version, 0664, touch_version_read, touch_version_write);
static DEVICE_ATTR(touch_update,0664, touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, 0664, NULL, touch_led_control);
static DEVICE_ATTR(enable_disable, 0664, NULL, touchkey_enable_disable);
#endif

static enum hrtimer_restart cypress_wdog_timer_func(struct hrtimer *timer)
{
	int i;

	for (i = 0; i < _3_TOUCH_MAXKEYS; i++) {
		if (touchkey_status[i] == 1) {
			input_report_key(devdata_global->input_dev, devdata_global->pdata->keycode[i], 0);
			touchkey_status[i] = 0;
#ifdef KERNEL_DEBUG_SEC
			printk("[TSK]keycode : %d, state : %d	%d\n", devdata_global->pdata->keycode[i], 0, __LINE__);
#endif
		}
	}

	return HRTIMER_NORESTART;
}

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
		input_set_capability(input_dev, EV_KEY, devdata->pdata->keycode[cnt]);

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

	dev_info(dev, "%s: hardware rev1 = %#02x, rev2 = %#02x\n", __func__, data[1], data[2]);

	devdata->backlight_on = BACKLIGHT_ON;
	devdata->backlight_off = BACKLIGHT_OFF;

	devdata->has_legacy_keycode = 1;

	if (data[1] < 0xc4 && (data[1] >= 0x8 ||
		(data[1] == 0x8 && data[2] >= 0x9)) &&
		 devdata->has_legacy_keycode == false) {
		devdata->backlight_on = BACKLIGHT_ON;
		devdata->backlight_off = BACKLIGHT_OFF;
	} else {
		devdata->backlight_on = OLD_BACKLIGHT_ON;
		devdata->backlight_off = OLD_BACKLIGHT_OFF;
	}

	err = i2c_touchkey_write_byte(devdata, devdata->backlight_on);

	if (err) {
		dev_err(dev, "%s: touch keypad backlight on failed\n", __func__);
		/* The device may not be responding because of bad firmware */
		goto err_backlight_on;
	}

	if (request_threaded_irq(client->irq, touchkey_interrupt_handler,
				touchkey_interrupt_thread, IRQF_TRIGGER_FALLING,
				DEVICE_NAME, devdata)) {
		dev_err(dev, "%s: Can't allocate irq.\n", __func__);
		goto err_req_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	devdata->early_suspend.suspend = cypress_touchkey_early_suspend;
	devdata->early_suspend.resume = cypress_touchkey_early_resume;
	register_early_suspend(&devdata->early_suspend);
#endif

	devdata->is_powering_on = false;

#if defined(TOUCH_UPDATE)
	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk("%s misc_register fail\n", __func__);
	}

	dev_set_drvdata(touchkey_update_device.this_device, devdata);

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_version) < 0) {
		printk("%s device_create_file fail dev_attr_touch_version\n", __func__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touch_version.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_update) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n", __func__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_touch_update.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_brightness) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n", __func__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_enable_disable) < 0) {
		printk("%s device_create_file fail dev_attr_touch_update\n", __func__);
		pr_err("Failed to create device file(%s)!\n", dev_attr_enable_disable.attr.name);
	}

	touchkey_wq = create_singlethread_workqueue(DEVICE_NAME);
	if (!touchkey_wq)
		return -ENOMEM;

	while (retry--) {
		if (get_touchkey_firmware(data) == 0)	//melfas need delay for multiple read
			break;
	}
	printk("%s F/W version: 0x%x, Module version:0x%x\n", __func__, data[1], data[2]);
#endif

#ifdef CONFIG_GENERIC_BLN
	register_bln_implementation(&cypress_touchkey_bln);
#endif

	/* Register the device and sysfs interface for backlight timeout */
	pr_info("%s: Registering %s device\n", __func__, backlight_device.name);
	ret = misc_register(&backlight_device);
	if (ret)
		pr_err("%s: Failed to register %s device\n", __func__, backlight_device.name);
	else if (sysfs_create_group(&backlight_device.this_device->kobj, &backlight_group) < 0)
		pr_err("%s: Failed to create sysfs group for %s device\n", __func__, backlight_device.name);

	if (charging_boot) {
		printk("cypress_touchkey_probe skip (LPM mode)\n");

		devdata->is_powering_on = true;

		if (unlikely(devdata->is_dead))
			return -EINVAL;

		disable_irq(devdata->client->irq);
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		all_keys_up(devdata);
	}

	/* Setup the timer for timeout management of touchkeys backlight */
	setup_timer(&backlight_timer, backlight_timer_callback, 0);

	return 0;

err_req_irq:
err_backlight_on:
	input_unregister_device(input_dev);
	goto touchkey_off;
err_input_reg_dev:
err_read:
	input_free_device(input_dev);
touchkey_off:
	devdata->is_powering_on = false;
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
err_input_alloc_dev:
err_null_keycodes:
	kfree(devdata);
	return err;
}

static int __devexit i2c_touchkey_remove(struct i2c_client *client)
{
	struct cypress_touchkey_devdata *devdata = i2c_get_clientdata(client);

	bln_wakelock_destroy();

	/* Deregister the touchkeys backlight device */
	misc_deregister(&backlight_device);

	unregister_early_suspend(&devdata->early_suspend);
	/* If the device is dead IRQs are disabled, we need to rebalance them */
	if (unlikely(devdata->is_dead))
		enable_irq(client->irq);
	else {
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		devdata->is_powering_on = false;
	}
	free_irq(client->irq, devdata);
	all_keys_up(devdata);
	input_unregister_device(devdata->input_dev);

	/* Delete the timer to manage the timeout of touchkeys backlight */
	del_timer(&backlight_timer);

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

	ret = i2c_add_driver(&touchkey_i2c_driver);
	if (ret)
		pr_err("%s: cypress touch keypad registration failed. (%d)\n", __func__, ret);

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

