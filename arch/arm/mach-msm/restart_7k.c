/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <asm/system_misc.h>
#include <mach/proc_comm.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/msm_iomap.h>

#include "devices-msm7x2xa.h"
#include "smd_rpcrouter.h"

static uint32_t restart_reason = 0x776655AA;

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
struct smem_info {
	unsigned int info;
};

static DEFINE_SPINLOCK(msm_reboot_lock);
extern struct smem_info *smem_flag;
extern void request_phone_power_off_reset(int flag);
static void __iomem *reset_base;
static void __iomem *msm_m_gpio2_owner_base;
int power_off_done;
#define RESET_ALL		0xab800200
#define MSM_M_GPIO2_OWNER	0xabd00504
#define RESET_ALL_VAL		0x1
int (*set_recovery_mode)(void);
EXPORT_SYMBOL(set_recovery_mode);
int (*set_recovery_mode_done)(void);
EXPORT_SYMBOL(set_recovery_mode_done);
extern int curr_usb_status;
extern int curr_ta_status;
typedef struct
{
	unsigned int command;
	unsigned int status;
	unsigned int data1;
	unsigned int data2;
} smem_proc_comm_data_type;
volatile smem_proc_comm_data_type *proc_comm;
#endif

static void msm_pm_power_off(void)
{
#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	msm_rpcrouter_close();
	proc_comm = (volatile smem_proc_comm_data_type *)MSM_SHARED_RAM_BASE;
	proc_comm[4].command = curr_usb_status || curr_ta_status;
	/* debug level value set to 0 to avoid entering silent reset mode when restarting */
	smem_flag->info = 0x0;

	printk("%s: send PCOM_POWER_DOWN\n",__func__);
	msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
	power_off_done = 1;

	printk("%s: call request_phone_power_off\n",__func__);
	request_phone_power_off_reset(1);

	printk("%s: do nothing!!\n",__func__);

	/* block irq after machine off 2011-05-17 hc.hyun */
	spin_lock_irq(&msm_reboot_lock);

	/* if machine does not off after 30sec from PCOM_POWER_DOWN, RESET register set to 0x1 2011-05-17 hc.hyun */
	mdelay(30000);
	printk("%s: retry with setting PS_HOLD to low.\n",__func__);
	writel((readl(msm_m_gpio2_owner_base) | (1<<13)), msm_m_gpio2_owner_base);
	writel((readl(msm_m_gpio2_owner_base + 0x30) | (1<<13)), msm_m_gpio2_owner_base + 0x30);
	gpio_set_value(29, 0);

	for (;;)
		;
#else
	msm_rpcrouter_close();
	msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
	for (;;)
		;
#endif
}

static void msm_pm_restart(char str, const char *cmd)
{
#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	msm_rpcrouter_close();

	/* debug level value set to 0 to avoid entering silent reset mode when restarting */
	smem_flag->info = 0x0;

	printk("%s: send PCOM_RESET_CHIP\n",__func__);

	msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);
	//msm_proc_comm(PCOM_RESET_CHIP_IMM, &restart_reason, 0);
	power_off_done = 1;

	printk("%s: do nothing!!\n",__func__);

	/* block irq after machine off 2011-05-17 hc.hyun */
	spin_lock_irq(&msm_reboot_lock);

	/* if machine does not off after 30sec from PCOM_RESET_CHIP, RESET register set to 0x1 2011-05-17 hc.hyun */
	mdelay(30000);
	printk("%s: retry with  setting PS_HOLD to low.\n",__func__);
	writel((readl(msm_m_gpio2_owner_base) | (1<<13)), msm_m_gpio2_owner_base);
	writel((readl(msm_m_gpio2_owner_base + 0x30) | (1<<13)), msm_m_gpio2_owner_base + 0x30);
	gpio_set_value(29, 0);

	for (;;)
	;
#else
	msm_rpcrouter_close();
	pr_debug("The reset reason is %x\n", restart_reason);

	/* Disable interrupts */
	local_irq_disable();
	local_fiq_disable();

	/*
	 * Take out a flat memory mapping  and will
	 * insert a 1:1 mapping in place of
	 * the user-mode pages to ensure predictable results
	 * This function takes care of flushing the caches
	 * and flushing the TLB.
	 */
	setup_mm_for_reboot();

	msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);

	for (;;)
		;
#endif
}

static int msm_reboot_call
	(struct notifier_block *this, unsigned long code, void *_cmd)
{
	if ((code == SYS_RESTART) && _cmd) {
		char *cmd = _cmd;
		if (!strncmp(cmd, "bootloader", 10)) {
			restart_reason = 0x77665500;
		} else if (!strncmp(cmd, "recovery", 8)) {
			restart_reason = 0x77665502;
		} else if (!strcmp(cmd, "recovery_done")) {
			restart_reason = 0x77665503;
		} else if (!strcmp(cmd, "download")) {
			mdelay(1500);
			restart_reason = 0x776655FF;
		} else if (!strncmp(cmd, "eraseflash", 10)) {
			restart_reason = 0x776655EF;
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			int res;
			res = kstrtoul(cmd + 4, 16, &code);
			code &= 0xff;
			restart_reason = 0x6f656d00 | code;
		} else if (!strncmp(cmd, "arm11_fota", 10)) {
			restart_reason = 0x77665504;
		} else {
			restart_reason = 0x77665501;
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block msm_reboot_notifier = {
	.notifier_call = msm_reboot_call,
};

static int __init msm_pm_restart_init(void)
{
	int ret;

	pm_power_off = msm_pm_power_off;
	arm_pm_restart = msm_pm_restart;

	ret = register_reboot_notifier(&msm_reboot_notifier);
	if (ret)
		pr_err("Failed to register reboot notifier\n");

	return ret;

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
        reset_base = ioremap(RESET_ALL, PAGE_SIZE);
        msm_m_gpio2_owner_base = ioremap(MSM_M_GPIO2_OWNER, PAGE_SIZE);
#endif
}
late_initcall(msm_pm_restart_init);
