/* drivers/misc/bln.c
 *
 * Copyright (C) 2011         Michael Richter (alias neldar)
 *           (C) 2011         Adam Kent <adam@semicircular.net>
 *           (C) 2012 - 2013  Stefano Gottardo <whiteflash@email.it>
 *           (C) 2013 - 2014  Cristoforo Cataldo <cristoforo.cataldo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/earlysuspend.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/bln.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#define BACKLIGHTNOTIFICATION_VERSION     10      /* BLN module version */

#define OFF                                0      /* Lights off */
#define ON                                 1      /* Lights on */

#define MAX_ALLOWED_STATIC_TIME         3600      /* MAX 3600 seconds = 1 hour */
#define MAX_ALLOWED_BLINK_TIME          3600      /* MAX 3600 seconds = 1 hour */
#define MAX_ALLOWED_BLINK_INTERVAL      5000      /* MAX 5000 milliseconds = 5 seconds */

static struct bln_implementation *bln_imp = NULL; /* BLN implementation */
static bool bln_enabled = false;                  /* Activation status of BLN feature */
static bool bln_in_kernel_static = false;         /* Activation status of in-kernel static lights */
static uint bln_static_max_time = 60;             /* Timeout in seconds for in-kernel static lights (default: 1 minute) */
static bool bln_ongoing = false;                  /* Status of ongoing led notification */
static uint bln_blink_control = OFF;              /* Status of blinking */
static bool bln_in_kernel_blink = false;          /* Activation status of in-kernel blinking lights */
static uint bln_blink_interval_on = 500;          /* Interval in milliseconds for blink ON (default: 500 ms) */
static uint bln_blink_interval_off = 500;         /* Interval in milliseconds for blink OFF (default: 500 ms) */
static uint bln_blink_max_time = 60;              /* Timeout in seconds for in-kernel blinking (default: 1 minute) */
static uint bln_status_after_blink = OFF;         /* Status to set after the timeout of in-kernel blinking (default: OFF) */
static bool bln_suspended = false;                /* Flag indicating if system is suspended */
static unsigned long bln_blink_end_time;          /* Calculated end-time of blinking */

static struct wake_lock bln_wake_lock;            /* Wakelock to keep lights on until is requested */

/* Timer implementation for static lights */
void static_timer_callback(unsigned long data);
static struct timer_list static_timer =
	TIMER_INITIALIZER(static_timer_callback, 0, 0);

/* Timer implementation for blinking lights */
void blink_timer_callback(unsigned long data);
static struct timer_list blink_timer =
	TIMER_INITIALIZER(blink_timer_callback, 0, 0);

/* Workqueue and schedulable works */
static struct workqueue_struct *bln_workqueue;
static void static_stop(struct work_struct *work);
static DECLARE_WORK(static_stop_work, static_stop);
static void blink_continue(struct work_struct *work);
static DECLARE_WORK(blink_continue_work, blink_continue);
static void blink_stop(struct work_struct *work);
static DECLARE_WORK(blink_stop_work, blink_stop);

/*
 * Call implemented routine to turn lights on
 */
static void bln_enable_backlights(void)
{
	if (bln_imp)
		bln_imp->enable();
}

/*
 * Call implemented routine to turn lights off
 */
static void bln_disable_backlights(void)
{
	if (bln_imp)
		bln_imp->disable();
}

/*
 * Start the timer for in-kernel static lights
 */
static void start_static_timer(void) {
	/* Update the static lights timer */
	mod_timer(&static_timer, jiffies +
			msecs_to_jiffies(bln_static_max_time * 1000));
}

/*
 * Start the timer for in-kernel blinking lights
 */
static void start_blink_timer(void) {
	/* Setup blinking lights timer expiration */
	bln_blink_end_time = jiffies +
			msecs_to_jiffies(bln_blink_max_time * 1000);

	/* Update the blinking lights timer */
	mod_timer(&blink_timer, jiffies +
			msecs_to_jiffies(bln_blink_interval_on));
}

/*
 * Turn the lights on for an incoming notification
 */
static void enable_led_notification(void)
{
	/* If BLN is not enabled, then exit */
	if (!bln_enabled)
		return;

	/* Acquire wakelock */
	bln_wakelock_acquire();

	/* If in-kernel blinking or static lights are enabled,
	 * start timer for in-kernel blinking or static lights */
	if (bln_in_kernel_blink && bln_blink_max_time > 0)
		start_blink_timer();
	else if (bln_in_kernel_static && bln_static_max_time > 0)
		start_static_timer();

	/* Turn the lights on to report incoming notification */
	bln_enable_backlights();
	bln_ongoing = true;
	pr_info("%s: notification led enabled\n", __func__);
}

/*
 * Turn the lights off after an incoming notification
 */
static void disable_led_notification(void)
{
	pr_info("%s: notification led disabled\n", __func__);

	/* Reset blinking status and notification status */
	bln_blink_control = OFF;
	bln_ongoing = false;

	/* Delete the timers for blinking and static lights
	 * Note: del_timer checks if the timer is pending */
	del_timer(&blink_timer);
	del_timer(&static_timer);

	/* Cancel pending works */
	if (work_pending(&static_stop_work))
		cancel_work_sync(&static_stop_work);
	if (work_pending(&blink_continue_work))
		cancel_work_sync(&blink_continue_work);
	if (work_pending(&blink_stop_work))
		cancel_work_sync(&blink_stop_work);

	/* If the system is suspended, turn off the lights */
	if (bln_suspended)
		bln_disable_backlights();

	/* Release wakelock */
	bln_wakelock_release();
}

/*
 * Manage early suspend callback
 */
static void bln_early_suspend(struct early_suspend *h)
{
	bln_suspended = true;
}

/*
 * Manage late resume callback
 */
static void bln_late_resume(struct early_suspend *h)
{
	/* Turn off the lights */
	if (bln_ongoing)
		disable_led_notification();
	bln_suspended = false;
}

/* Early suspend and resume struct implementation */
static struct early_suspend bln_suspend_data = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = bln_early_suspend,
	.resume = bln_late_resume,
};

/*
 * Get the current status of BLN feature
 */
static ssize_t bln_status_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (bln_enabled ? 1 : 0));
}

/*
 * Store the status of BLN feature
 */
static ssize_t bln_status_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		pr_devel("%s: %u \n", __func__, data);
		if (data == 1) {
			pr_info("%s: BLN function enabled\n", __func__);
			bln_enabled = true;
		} else if (data == 0) {
			pr_info("%s: BLN function disabled\n", __func__);
			bln_enabled = false;
			if (bln_ongoing)
				disable_led_notification();
		} else {
			pr_info("%s: invalid input range %u\n", __func__, data);
		}
	} else {
		pr_info("%s: invalid input\n", __func__);
	}

	return size;
}

/*
 * Get the current status of notifications
 */
static ssize_t notification_led_status_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", (bln_ongoing ? 1 : 0));
}

/*
 * Store the status of notifications
 */
static ssize_t notification_led_status_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data == 1)
			enable_led_notification();
		else if (data == 0)
			disable_led_notification();
		else
			pr_info("%s: wrong input %u\n", __func__, data);
	} else {
		pr_info("%s: input error\n", __func__);
	}

	return size;
}

/*
 * Get the current status of in-kernel static lights feature
 */
static ssize_t in_kernel_static_status_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", (bln_in_kernel_static ? 1 : 0));
}

/*
 * Store the status of in-kernel static lights feature
 */
static ssize_t in_kernel_static_status_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1)
		bln_in_kernel_static = !!data;
	else
		pr_info("%s: input error\n", __func__);

	return size;
}

/*
 * Get the max duration of static lights
 */
static ssize_t static_maxtime_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bln_static_max_time);
}

/*
 * Store the max duration of static lights
 */
static ssize_t static_maxtime_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0)
			bln_static_max_time = (data <= MAX_ALLOWED_STATIC_TIME) ?
			                       data : MAX_ALLOWED_STATIC_TIME;
		else
			pr_info("%s: wrong input %u\n", __func__, data);
	} else {
		pr_info("%s: input error\n", __func__);
	}

	return size;
}

/*
 * Get the current status of in-kernel blinking lights feature
 */
static ssize_t in_kernel_blink_status_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", (bln_in_kernel_blink ? 1 : 0));
}

/*
 * Store the status of in-kernel blinking lights feature
 */
static ssize_t in_kernel_blink_status_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1)
		bln_in_kernel_blink = !!data;
	else
		pr_info("%s: input error\n", __func__);

	return size;
}

/*
 * Get the current blinking status
 */
static ssize_t blink_control_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bln_blink_control);
}

/*
 * Store the blinking status
 */
static ssize_t blink_control_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (bln_ongoing) {
		if (sscanf(buf, "%u\n", &data) == 1) {
			if (data == 1) {
				bln_blink_control = ON;
				bln_disable_backlights();
			} else if (data == 0) {
				bln_blink_control = OFF;
				bln_enable_backlights();
			} else {
				pr_info("%s: wrong input %u\n", __func__, data);
			}
		} else {
			pr_info("%s: input error\n", __func__);
		}
	}

	return size;
}

/*
 * Get the current blinking interval (for compatibility with some apps)
 */
static ssize_t blink_interval_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bln_blink_interval_on);
}

/*
 * Store the blinking interval (for compatibility with some apps)
 */
static ssize_t blink_interval_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0) {
			bln_blink_interval_on = (data <= MAX_ALLOWED_BLINK_INTERVAL) ?
			                         data : MAX_ALLOWED_BLINK_INTERVAL;
			bln_blink_interval_off = bln_blink_interval_on;
		} else {
			pr_info("%s: wrong input %u\n", __func__, data);
		}
	} else {
		pr_info("%s: input error\n", __func__);
	}

	return size;
}

/*
 * Get the current blink ON interval
 */
static ssize_t blink_interval_on_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bln_blink_interval_on);
}

/*
 * Store the blink ON interval
 */
static ssize_t blink_interval_on_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0)
			bln_blink_interval_on = (data <= MAX_ALLOWED_BLINK_INTERVAL) ?
			                         data : MAX_ALLOWED_BLINK_INTERVAL;
		else
			pr_info("%s: wrong input %u\n", __func__, data);
	} else {
		pr_info("%s: input error\n", __func__);
	}

	return size;
}

/*
 * Get the current blink OFF interval
 */
static ssize_t blink_interval_off_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bln_blink_interval_off);
}

/*
 * Store the blink OFF interval
 */
static ssize_t blink_interval_off_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0)
			bln_blink_interval_off = (data <= MAX_ALLOWED_BLINK_INTERVAL) ?
			                          data : MAX_ALLOWED_BLINK_INTERVAL;
		else
			pr_info("%s: wrong input %u\n", __func__, data);
	} else {
		pr_info("%s: input error\n", __func__);
	}

	return size;
}

/*
 * Get the max duration of blinking lights
 */
static ssize_t blink_maxtime_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", bln_blink_max_time);
}

/*
 * Store the max duration of blinking lights
 */
static ssize_t blink_maxtime_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1) {
		if (data > 0)
			bln_blink_max_time = (data <= MAX_ALLOWED_BLINK_TIME) ?
			                      data : MAX_ALLOWED_BLINK_TIME;
		else
			pr_info("%s: wrong input %u\n", __func__, data);
	} else {
		pr_info("%s: input error\n", __func__);
	}

	return size;
}

/*
 * Get the status to set after the blinking end time
 */
static ssize_t status_after_blinking_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%u\n", bln_status_after_blink);
}

/*
 * Store the status to set after the blinking end time
 */
static ssize_t status_after_blinking_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	uint data;

	if (sscanf(buf, "%u\n", &data) == 1)
		bln_status_after_blink = (data ? ON : OFF);
	else
		pr_info("%s: input error\n", __func__);

	return size;
}

/*
 * Get BLN module version
 */
static ssize_t bln_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", BACKLIGHTNOTIFICATION_VERSION);
}

/* Definition of sysfs interface attributes for BLN module */
static DEVICE_ATTR(blink_control, S_IRUGO | S_IWUGO,
		blink_control_read,
		blink_control_write);
static DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO,
		bln_status_read,
		bln_status_write);
static DEVICE_ATTR(notification_led, S_IRUGO | S_IWUGO,
		notification_led_status_read,
		notification_led_status_write);
static DEVICE_ATTR(in_kernel_static, S_IRUGO | S_IWUGO,
		in_kernel_static_status_read,
		in_kernel_static_status_write);
static DEVICE_ATTR(static_maxtime, S_IRUGO | S_IWUGO,
		static_maxtime_read,
		static_maxtime_write);
static DEVICE_ATTR(in_kernel_blink, S_IRUGO | S_IWUGO,
		in_kernel_blink_status_read,
		in_kernel_blink_status_write);
static DEVICE_ATTR(blink_interval, S_IRUGO | S_IWUGO,
		blink_interval_read,
		blink_interval_write);
static DEVICE_ATTR(blink_interval_on, S_IRUGO | S_IWUGO,
		blink_interval_on_read,
		blink_interval_on_write);
static DEVICE_ATTR(blink_interval_off, S_IRUGO | S_IWUGO,
		blink_interval_off_read,
		blink_interval_off_write);
static DEVICE_ATTR(blink_maxtime, S_IRUGO | S_IWUGO,
		blink_maxtime_read,
		blink_maxtime_write);
static DEVICE_ATTR(status_after_blinking, S_IRUGO | S_IWUGO,
		status_after_blinking_read,
		status_after_blinking_write);
static DEVICE_ATTR(version, S_IRUGO, bln_version, NULL);

static struct attribute *bln_notification_attributes[] = {
	&dev_attr_blink_control.attr,
	&dev_attr_enabled.attr,
	&dev_attr_notification_led.attr,
	&dev_attr_in_kernel_static.attr,
	&dev_attr_static_maxtime.attr,
	&dev_attr_in_kernel_blink.attr,
	&dev_attr_blink_interval.attr,
	&dev_attr_blink_interval_on.attr,
	&dev_attr_blink_interval_off.attr,
	&dev_attr_blink_maxtime.attr,
	&dev_attr_status_after_blinking.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group bln_notification_group = {
	.attrs  = bln_notification_attributes,
};

/* BLN device definition */
static struct miscdevice bln_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "backlightnotification",
};

/*
 * Register BLN implementation routines to turn the lights on/off
 */
void register_bln_implementation(struct bln_implementation *imp)
{
	bln_imp = imp;
}
EXPORT_SYMBOL(register_bln_implementation);

/*
 * Get the notification status
 */
bool bln_is_ongoing(void)
{
	return bln_ongoing;
}
EXPORT_SYMBOL(bln_is_ongoing);

/*
 * Acquire the wakelock to prevent the system to go in suspend mode
 */
void bln_wakelock_acquire(void) {
	if (!wake_lock_active(&bln_wake_lock)) {
		pr_info("%s: Acquiring BLN wakelock\n", __func__);
		wake_lock(&bln_wake_lock);
	}
}
EXPORT_SYMBOL(bln_wakelock_acquire);

/*
 * Release the wakelock to let the system to go in suspend mode again
 */
void bln_wakelock_release(void) {
	if (wake_lock_active(&bln_wake_lock)) {
		pr_info("%s: Releasing BLN wakelock\n", __func__);
		wake_unlock(&bln_wake_lock);
	}
}
EXPORT_SYMBOL(bln_wakelock_release);

/*
 * Work callback to stop static lights
 */
static void static_stop(struct work_struct *work)
{
	pr_info("%s: static lights notification timed out\n", __func__);

	/* Turn the lights off */
	disable_led_notification();
}

/*
 * Work callback to manage blinking lights status
 */
static void blink_continue(struct work_struct *work)
{
	/* If there's no more incoming notification, then exit */
	if (!bln_ongoing)
		return;

	/* Turn the lights on/off according to blinking status
	 * value and update the status for next blink */
	if (bln_blink_control == ON) {
		bln_enable_backlights();
		bln_blink_control = OFF;
	} else {
		bln_disable_backlights();
		bln_blink_control = ON;
	}
}

/*
 * Work callback to stop blinking lights
 */
static void blink_stop(struct work_struct *work)
{
	pr_info("%s: blinking lights notification timed out\n", __func__);

	/* If the status to be set after the blinking end is ON */
	if (bln_ongoing &&
	    bln_status_after_blink == ON &&
	    bln_static_max_time > 0) {
		/* Delete the timer for blinking */
		del_timer(&blink_timer);

		/* Reset blinking status */
		bln_blink_control = OFF;

		/* Start the timer for static light*/
		start_static_timer();

		/* Turn the lights on */
		bln_enable_backlights();
	} else {
		/* Turn the lights off */
		disable_led_notification();
	}
}

/*
 * Callback to manage the timer expiration for static lights
 */
void static_timer_callback(unsigned long data)
{
	/* Stop static lights */
	queue_work(bln_workqueue, &static_stop_work);
}

/*
 * Callback to manage the timer expiration for blinking lights interval
 */
void blink_timer_callback(unsigned long data)
{
	uint blink_interval;

	/* If the blinking time end has not been reached */
	if (bln_ongoing && time_before(jiffies, bln_blink_end_time)) {
		/* Get the blinking interval to use for next transition */
		if (bln_blink_control == ON)
			blink_interval = bln_blink_interval_on;
		else
			blink_interval = bln_blink_interval_off;

		/* Queue blinking work */
		queue_work(bln_workqueue, &blink_continue_work);

		/* Update the timer with the next interval */
		mod_timer(&blink_timer, jiffies +
					msecs_to_jiffies(blink_interval));
	} else {
		/* Remove pending blinking work */
		if (work_pending(&blink_continue_work))
			cancel_work_sync(&blink_continue_work);
		flush_workqueue(bln_workqueue);

		/* Stop blinking lights */
		queue_work(bln_workqueue, &blink_stop_work);
	}
}

/*
 * BLN module initializer
 */
static int __init bln_control_init(void)
{
	int ret;

	/* Register BLN module */
	pr_info("%s misc_register(%s)\n", __func__, bln_device.name);
	ret = misc_register(&bln_device);
	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __func__,
				bln_device.name);
		return 1;
	}

	/* Create sysfs interface for module attributes */
	if (sysfs_create_group(&bln_device.this_device->kobj,
				&bln_notification_group) < 0) {
		pr_err("%s sysfs_create_group fail\n", __func__);
		pr_err("Failed to create sysfs group for device (%s)!\n",
				bln_device.name);
	}

	/* Register early suspend/resume management */
	register_early_suspend(&bln_suspend_data);

	/* Initialize wakelock */
	wake_lock_init(&bln_wake_lock, WAKE_LOCK_SUSPEND, "bln_wake_lock");

	/* Initialize blink woorkqueue */
	bln_workqueue = create_singlethread_workqueue("bln_blink");
	if (!bln_workqueue)
		return -ENOMEM;

	return 0;
}

/*
 * BLN module destructor
 */
static void __exit bln_control_exit(void)
{
	del_timer(&blink_timer);
	del_timer(&static_timer);
	destroy_workqueue(bln_workqueue);
	wake_lock_destroy(&bln_wake_lock);
	unregister_early_suspend(&bln_suspend_data);
	misc_deregister(&bln_device);
}

device_initcall(bln_control_init);
module_exit(bln_control_exit);

MODULE_AUTHOR("Michael Richter (alias neldar)");
MODULE_AUTHOR("Adam Kent <adam@semicircular.net>");
MODULE_AUTHOR("Stefano Gottardo <whiteflash@email.it>");
MODULE_AUTHOR("Cristoforo Cataldo <cristoforo.cataldo@gmail.com>");
MODULE_LICENSE("GPL");
