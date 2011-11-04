/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/mdm.h>
#include "mdm_ioctls.h"
#include "msm_watchdog.h"

#define CHARM_MODEM_TIMEOUT	2000
#define CHARM_MODEM_DELTA	100

static void (*power_on_charm)(void);
static void (*power_down_charm)(void);

static int charm_debug_on;

#define CHARM_DBG(...)	do { if (charm_debug_on) \
					pr_info(__VA_ARGS__); \
			} while (0);

static void __soc_restart(void)
{
	lock_kernel();
	kernel_restart(NULL);
	unlock_kernel();
}

static int charm_panic_prep(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	CHARM_DBG("%s: setting AP2MDM_ERRFATAL high\n", __func__);
	gpio_set_value(AP2MDM_ERRFATAL, 1);
	return NOTIFY_DONE;
}

static struct notifier_block charm_panic_blk = {
	.notifier_call  = charm_panic_prep,
};

static int successful_boot;

static long charm_modem_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{

	int ret = 0;

	if (_IOC_TYPE(cmd) != CHARM_CODE) {
		pr_err("%s: invalid ioctl code\n", __func__);
		return -EINVAL;
	}

	gpio_request(MDM2AP_STATUS, "MDM2AP_STATUS");
	gpio_direction_input(MDM2AP_STATUS);

	CHARM_DBG("%s: Entering ioctl cmd = %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case WAKE_CHARM:
		/* turn the charm on */
		power_on_charm();
		break;

	case RESET_CHARM:
		/* put the charm back in reset */
		power_down_charm();
		break;

	case CHECK_FOR_BOOT:
		if (gpio_get_value(MDM2AP_STATUS) == 0)
			put_user(1, (unsigned long __user *) arg);
		else {
			put_user(0, (unsigned long __user *) arg);
			if (!successful_boot) {
				successful_boot = 1;
				pr_info("%s: sucessful_boot = 1. Monitoring \
					for mdm interrupts\n",
					__func__);
			}
		}
		break;

	case WAIT_FOR_BOOT:
		/* wait for status to be high */
		while (gpio_get_value(MDM2AP_STATUS) == 0)
			;
		break;
	default:
		ret = -EINVAL;
	}

	gpio_free(MDM2AP_STATUS);

	return ret;
}

static int charm_modem_open(struct inode *inode, struct file *file)
{

	CHARM_DBG("%s: successful_boot = 0\n", __func__);
	successful_boot = 0;
	return 0;
}

static const struct file_operations charm_modem_fops = {
	.owner		= THIS_MODULE,
	.open		= charm_modem_open,
	.unlocked_ioctl	= charm_modem_ioctl,
};


struct miscdevice charm_modem_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdm",
	.fops	= &charm_modem_fops
};



static void charm_status_fn(struct work_struct *work)
{
	WARN("%s: Status went low!\n", __func__);
	__soc_restart();
}

static DECLARE_WORK(charm_status_work, charm_status_fn);

static void charm_fatal_fn(struct work_struct *work)
{
	WARN("%s: Got an error fatal!\n", __func__);
	__soc_restart();
}

static DECLARE_WORK(charm_fatal_work, charm_fatal_fn);

static irqreturn_t charm_errfatal(int irq, void *dev_id)
{
	pr_info("%s: charm got errfatal interrupt\n", __func__);
	if (successful_boot) {
		pr_info("%s: scheduling work now\n", __func__);
		schedule_work(&charm_fatal_work);
		disable_irq_nosync(MSM_GPIO_TO_INT(MDM2AP_ERRFATAL));
	}
	return IRQ_HANDLED;
}

static irqreturn_t charm_status_change(int irq, void *dev_id)
{

	pr_info("%s: Charm status went low!\n", __func__);
	if (successful_boot) {
		pr_info("%s: scheduling work now\n", __func__);
		schedule_work(&charm_status_work);
		disable_irq_nosync(MSM_GPIO_TO_INT(MDM2AP_STATUS));
	}
	return IRQ_HANDLED;
}

static int charm_debug_on_set(void *data, u64 val)
{
	charm_debug_on = val;
	return 0;
}

static int charm_debug_on_get(void *data, u64 *val)
{
	*val = charm_debug_on;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(charm_debug_on_fops,
			charm_debug_on_get,
			charm_debug_on_set, "%llu\n");

static int charm_debugfs_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("charm_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("debug_on", 0644, dent, NULL,
			&charm_debug_on_fops);
	return 0;
}

static int __init charm_modem_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct charm_platform_data *d = pdev->dev.platform_data;

	gpio_request(AP2MDM_STATUS, "AP2MDM_STATUS");
	gpio_request(AP2MDM_ERRFATAL, "AP2MDM_ERRFATAL");
	gpio_request(AP2MDM_KPDPWR_N, "AP2MDM_KPDPWR_N");
	gpio_request(AP2MDM_PMIC_RESET_N, "AP2MDM_PMIC_RESET_N");

	gpio_direction_output(AP2MDM_STATUS, 1);
	gpio_direction_output(AP2MDM_ERRFATAL, 0);

	power_on_charm = d->charm_modem_on;
	power_down_charm = d->charm_modem_off;

	gpio_request(MDM2AP_ERRFATAL, "MDM2AP_ERRFATAL");
	gpio_direction_input(MDM2AP_ERRFATAL);

	atomic_notifier_chain_register(&panic_notifier_list, &charm_panic_blk);
	charm_debugfs_init();

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_ERRFATAL IRQ resource. \
			error=%d No IRQ will be generated on errfatal.",
			__func__, irq);
		goto errfatal_err;
	}

	ret = request_irq(irq, charm_errfatal,
		IRQF_TRIGGER_RISING , "charm errfatal", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_ERRFATAL IRQ#%d request failed with error=%d\
			. No IRQ will be generated on errfatal.",
			__func__, irq, ret);
	}

errfatal_err:

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		pr_err("%s: could not get MDM2AP_STATUS IRQ resource. \
			error=%d No IRQ will be generated on status change.",
			__func__, irq);
		goto status_err;
	}

	ret = request_threaded_irq(irq, NULL, charm_status_change,
		IRQF_TRIGGER_FALLING, "charm status", NULL);

	if (ret < 0) {
		pr_err("%s: MDM2AP_STATUS IRQ#%d request failed with error=%d\
			. No IRQ will be generated on status change.",
			__func__, irq, ret);
	}

status_err:
	pr_info("%s: Registering charm modem\n", __func__);

	return misc_register(&charm_modem_misc);
}


static int __devexit charm_modem_remove(struct platform_device *pdev)
{

	return misc_deregister(&charm_modem_misc);
}

static void charm_modem_shutdown(struct platform_device *pdev)
{
	int irq, i;

	pr_info("%s: setting AP2MDM_STATUS low for a graceful restart\n",
		__func__);

	irq = platform_get_irq(pdev, 0);
	disable_irq_nosync(irq);

	irq = platform_get_irq(pdev, 1);
	disable_irq_nosync(irq);

	gpio_set_value(AP2MDM_STATUS, 0);

	for (i = 0; i < CHARM_MODEM_TIMEOUT; i += CHARM_MODEM_DELTA) {
		pet_watchdog();
		msleep(CHARM_MODEM_DELTA);
		if (gpio_get_value(MDM2AP_STATUS) == 0)
			break;
	}

	if (i >= CHARM_MODEM_TIMEOUT) {
		pr_err("%s: MDM2AP_STATUS never went low.\n",
			 __func__);
		gpio_direction_output(AP2MDM_PMIC_RESET_N, 0);
	}
}

static struct platform_driver charm_modem_driver = {
	.remove         = charm_modem_remove,
	.shutdown	= charm_modem_shutdown,
	.driver         = {
		.name = "charm_modem",
		.owner = THIS_MODULE
	},
};

static int __init charm_modem_init(void)
{
	return platform_driver_probe(&charm_modem_driver, charm_modem_probe);
}

static void __exit charm_modem_exit(void)
{
	platform_driver_unregister(&charm_modem_driver);
}

module_init(charm_modem_init);
module_exit(charm_modem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("msm8660 charm modem driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("charm_modem")


