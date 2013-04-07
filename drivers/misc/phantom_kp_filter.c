/* drivers/misc/phantom_keypress_filter.c
 *
 * Copyright (C) 2013, Cristoforo Cataldo <cristoforo.cataldo@gmail.com>
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
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/phantom_kp_filter.h>
#include <linux/slab.h>

/********************** ATTRIBUTES DECLARATIONS **********************/

struct pkf_menuback_data *pkf_menuback;			/* MENU and BACK keys filtering data */
struct pkf_home_data *pkf_home;					/* HOME key filtering data */
struct pkf_home_incoming_kp *pkf_home_inc_kp;	/* Incoming HOME key presses data */

/************************** INLINE FUNCTIONS *************************/

/*
 * Function to get a bool value from the buffer
 */
static inline unsigned int get_bool_value(const char *buf)
{
	int value;
	sscanf(buf, "%d", &value);
	return !!value;
}

/*
 * Function to get a unsigned value from the buffer between the min-max range
 */
static inline unsigned int get_uint_value(const char *buf, unsigned int min, unsigned int max)
{
	unsigned int value;
	sscanf(buf, "%u", &value);
	return (value < min) ? min : ((value > max) ? max : value);
}

/************************** SYSFS INTERFACE **************************/

/*
 * Function to get the enabled status of the phantom MENU and BACK key presses filtering
 */
static ssize_t pkf_menuback_enabled_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_menuback->enabled);
}

/*
 * Function to set the enabled status of the phantom MENU and BACK key presses filtering
 */
static ssize_t pkf_menuback_enabled_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_menuback->enabled = get_bool_value(buf);
	pr_info("%s: %s phantom MENU and BACK key presses filtering\n", __func__,
			(pkf_menuback->enabled) ? "Enabled" : "Disabled");
	return size;
}

/*
 * Function to get the interrupt checks number to be performed for the incoming MENU and BACK key presses
 */
static ssize_t pkf_menuback_interrupt_checks_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_menuback->interrupt_checks);
}

/*
 * Function to set the interrupt checks number to be performed for the incoming MENU and BACK key presses
 */
static ssize_t pkf_menuback_interrupt_checks_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_menuback->interrupt_checks = (u8)get_uint_value(buf, MIN_MENUBACK_INTERRUPT_CHECKS, MAX_MENUBACK_INTERRUPT_CHECKS);
	pr_info("%s: MENU and BACK keys interrupt_checks value changed to %u\n",__func__, pkf_menuback->interrupt_checks);
	return size;
}

/*
 * Function to get the time in ms to wait after the first error on MENU or BACK keys
 */
static ssize_t pkf_menuback_first_err_wait_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_menuback->first_err_wait);
}

/*
 * Function to set the time in ms to wait after the first error on MENU or BACK keys
 */
static ssize_t pkf_menuback_first_err_wait_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_menuback->first_err_wait = (u16)get_uint_value(buf, MIN_MENUBACK_FIRST_ERROR_WAIT, MAX_MENUBACK_FIRST_ERROR_WAIT);
	pr_info("%s: MENU and BACK keys first_err_wait value changed to %u\n",__func__, pkf_menuback->first_err_wait);
	return size;
}

/*
 * Function to get the time in ms to wait after the last error on MENU or BACK keys
 */
static ssize_t pkf_menuback_last_err_wait_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_menuback->last_err_wait);
}

/*
 * Function to set the time in ms to wait after the last error on MENU or BACK keys
 */
static ssize_t pkf_menuback_last_err_wait_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_menuback->last_err_wait = (u16)get_uint_value(buf, MIN_MENUBACK_LAST_ERROR_WAIT, MAX_MENUBACK_LAST_ERROR_WAIT);
	pr_info("%s: MENU and BACK keys last_err_wait value changed to %u\n", __func__, pkf_menuback->last_err_wait);
	return size;
}

/*
 * Function to get the ignored possible phantom MENU and BACK key presses
 */
static ssize_t pkf_menuback_ignored_kp_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", pkf_menuback->ignored_kp);
}

/*
 * Function to get the enabled status of the phantom HOME key presses filtering
 */
static ssize_t pkf_home_enabled_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_home->enabled);
}

/*
 * Function to set the enabled status of the phantom HOME key presses filtering
 */
static ssize_t pkf_home_enabled_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_home->enabled = get_bool_value(buf);
	pr_info("%s: %s phantom HOME key presses filtering\n", __func__,
			(pkf_home->enabled) ? "Enabled" : "Disabled");
	return size;
}

/*
 * Function to get the allowed numbers of incoming irqs for a HOME key press
 */
static ssize_t pkf_home_allowed_irqs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_home->allowed_irqs);
}

/*
 * Function to set the allowed numbers of incoming irqs for a HOME key press
 */
static ssize_t pkf_home_allowed_irqs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_home->allowed_irqs = (u8)get_uint_value(buf, MIN_HOME_ALLOWED_IRQS, MAX_HOME_ALLOWED_IRQS);
	pr_info("%s: HOME key allowed_irqs value changed to %u\n", __func__, pkf_home->allowed_irqs);
	return size;
}

/*
 * Function to get the wait time in ms before report the collected HOME key presses
 */
static ssize_t pkf_home_report_wait_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", pkf_home->report_wait);
}

/*
 * Function to set the wait time in ms before report the collected HOME key presses
 */
static ssize_t pkf_home_report_wait_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	pkf_home->report_wait = (u8)get_uint_value(buf, MIN_HOME_REPORT_WAIT, MAX_HOME_REPORT_WAIT);
	pr_info("%s: HOME key report_wait value changed to %u\n", __func__, pkf_home->report_wait);
	return size;
}

/*
 * Function to get the ignored possible phantom HOME key presses
 */
static ssize_t pkf_home_ignored_kp_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", pkf_home->ignored_kp);
}

/*
 * Function to get the current module version
 */
static ssize_t pkf_version_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", PKF_VERSION);
}

/* Sysfs attribute entries, group and kobj declarations */
static DEVICE_ATTR(menuback_enabled, S_IRUGO | S_IWUGO, pkf_menuback_enabled_read, pkf_menuback_enabled_write);
static DEVICE_ATTR(menuback_interrupt_checks, S_IRUGO | S_IWUGO, pkf_menuback_interrupt_checks_read, pkf_menuback_interrupt_checks_write);
static DEVICE_ATTR(menuback_first_err_wait, S_IRUGO | S_IWUGO, pkf_menuback_first_err_wait_read, pkf_menuback_first_err_wait_write);
static DEVICE_ATTR(menuback_last_err_wait, S_IRUGO | S_IWUGO, pkf_menuback_last_err_wait_read, pkf_menuback_last_err_wait_write);
static DEVICE_ATTR(menuback_ignored_kp, S_IRUGO, pkf_menuback_ignored_kp_read, NULL);
static DEVICE_ATTR(home_enabled, S_IRUGO | S_IWUGO, pkf_home_enabled_read, pkf_home_enabled_write);
static DEVICE_ATTR(home_allowed_irqs, S_IRUGO | S_IWUGO, pkf_home_allowed_irqs_read, pkf_home_allowed_irqs_write);
static DEVICE_ATTR(home_report_wait, S_IRUGO | S_IWUGO, pkf_home_report_wait_read, pkf_home_report_wait_write);
static DEVICE_ATTR(home_ignored_kp, S_IRUGO, pkf_home_ignored_kp_read, NULL);
static DEVICE_ATTR(version, S_IRUGO, pkf_version_read, NULL);

static struct attribute *pkf_attrs[] = {
	&dev_attr_menuback_enabled.attr,
	&dev_attr_menuback_interrupt_checks.attr,
	&dev_attr_menuback_first_err_wait.attr,
	&dev_attr_menuback_last_err_wait.attr,
	&dev_attr_menuback_ignored_kp.attr,
	&dev_attr_home_enabled.attr,
	&dev_attr_home_allowed_irqs.attr,
	&dev_attr_home_report_wait.attr,
	&dev_attr_home_ignored_kp.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group pkf_attrs_group = {
	.attrs = pkf_attrs
};

static struct miscdevice pkf_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "phantom_kp_filter",
};

/************************* MODULE INIT AND EXIT METHODS **************************/

/*
 * Function for module initialization
 */
static int __init pkf_init(void)
{
	int ret;

	/* Device registration */
	pr_info("%s: Registering %s device\n", __func__, pkf_device.name);
	ret = misc_register(&pkf_device);
	if (ret) {
		pr_err("%s: Failed to register %s device\n", __func__, pkf_device.name);
		return -ENOMEM;
	}

	/* Allocate the memory for MENU and BACK keys filtering data */
	pkf_menuback = kzalloc(sizeof(struct pkf_menuback_data), GFP_KERNEL);
	if (!pkf_menuback) {
		pr_err("%s: Failed to allocate memory for pkf_menuback data\n", __func__);
		return -ENOMEM;
	}

	/* Initialize the MENU and BACK keys filtering data with the default values */
	pkf_menuback->enabled			= DEFAULT_MENUBACK_ENABLED;
	pkf_menuback->interrupt_checks	= DEFAULT_MENUBACK_INTERRUPT_CHECKS;
	pkf_menuback->first_err_wait	= DEFAULT_MENUBACK_FIRST_ERROR_WAIT;
	pkf_menuback->last_err_wait		= DEFAULT_MENUBACK_LAST_ERROR_WAIT;

	/* Allocate the memory for HOME key filtering data */
	pkf_home = kzalloc(sizeof(struct pkf_home_data), GFP_KERNEL);
	if (!pkf_home) {
		pr_err("%s: Failed to allocate memory for pkf_home data\n", __func__);
		return -ENOMEM;
	}

	/* Initialize HOME key filtering data with the default values */
	pkf_home->enabled				= DEFAULT_HOME_ENABLED;
	pkf_home->allowed_irqs			= DEFAULT_HOME_ALLOWED_IRQS;
	pkf_home->report_wait			= DEFAULT_HOME_REPORT_WAIT;

	/* Allocate the memory for incoming HOME key presses data */
	pkf_home_inc_kp = kzalloc(sizeof(struct pkf_home_incoming_kp), GFP_KERNEL);
	if (!pkf_home_inc_kp) {
		pr_err("%s: Failed to allocate memory for pkf_home_inc_kp data\n", __func__);
		return -ENOMEM;
	}

	/* Initialize HOME key filtering data with the default values */
	pkf_home_inc_kp->states			= 0;
	pkf_home_inc_kp->states_count	= 0;
	pkf_home_inc_kp->irqs_count		= 0;

	/* Sysfs group creation */
	if (sysfs_create_group(&pkf_device.this_device->kobj, &pkf_attrs_group) < 0) {
		pr_err("%s: Failed to create sysfs group for %s device\n", __func__, pkf_device.name);
		return -ENOMEM;
	}

	return ret;
}

MODULE_AUTHOR("Cristoforo Cataldo <cristoforo.cataldo@gmail.com>");
MODULE_DESCRIPTION("Phantom Key Press Filter - Allow to filter out the HOME, MENU, BACK key presses "
                   "caused by bad interrupts induced by antenna interference");
MODULE_LICENSE("GPL");

device_initcall(pkf_init);
