/*  drivers/misc/fm_si4709/Si4709_main.c
 *
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/stat.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
//#include <mach/gpio-aries.h>

#include "Si4709_i2c_drv.h"
#include "Si4709_dev.h"
#include "Si4709_ioctl.h"
#include "Si4709_common.h"

#define FM_IRQ_INT MSM_GPIO_TO_INT(GPIO_FM_INT)

static int Si4709_open(struct inode *, struct file *);
static int Si4709_release(struct inode *, struct file *);
static int Si4709_ioctl(struct inode *, struct file *, unsigned int,
	unsigned long);

static irqreturn_t Si4709_isr(int irq, void *unused);

static struct file_operations Si4709_fops = {
	.owner = THIS_MODULE,
	.open = Si4709_open,
	.ioctl = Si4709_ioctl,
	.release = Si4709_release,
};

static struct miscdevice Si4709_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fmradio",
	.fops = &Si4709_fops,
};
/* dummy struct which is used as a cookie for FM Radio interrupt */
typedef struct {
	int i;
	int j;
} fm_radio;
fm_radio fm_radio_1;

wait_queue_head_t Si4709_waitq;
static int Si4709_open(struct inode *inode, struct file *filp)
{
	printk("%s:\n", __func__);

	return nonseekable_open(inode, filp);
}

static int Si4709_release(struct inode *inode, struct file *filp)
{
	printk("%s:\n", __func__);

	return 0;
}

static int Si4709_ioctl(struct inode *inode, struct file *filp,
	unsigned int ioctl_cmd,  unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	if (_IOC_TYPE(ioctl_cmd) != Si4709_IOC_MAGIC) {
		pr_err("%s: invalid ioctl 1 0x%x\n", __func__, ioctl_cmd);
		return -ENOTTY;
	}

	if (_IOC_NR(ioctl_cmd) > Si4709_IOC_NR_MAX) {
		pr_err("%s: invalid ioctl 2 0x%x\n", __func__, ioctl_cmd);
		return -ENOTTY;
	}

	switch (ioctl_cmd) {
	case Si4709_IOC_POWERUP:
		printk("%s: IOC_POWERUP called\n", __func__);
		ret = Si4709_dev_powerup();
		if (ret < 0)
			pr_warn("%s: IOC_POWERUP failed\n", __func__);
		break;

	case Si4709_IOC_POWERDOWN:
		printk("%s: IOC_POWERDOWN called\n", __func__);
		ret = Si4709_dev_powerdown();
		if (ret < 0)
			pr_warn("%s: IOC_POWERDOWN failed\n", __func__);
		break;

	case Si4709_IOC_BAND_SET:
	{
		int band;
		printk("%s: IOC_BAND_SET called\n", __func__);

		if (copy_from_user((void *) &band, argp, sizeof(int)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_band_set(band);
			if (ret < 0)
				pr_warn("%s: IOC_BAND_SET failed\n", __func__);
		}
	}
		break;

	case Si4709_IOC_CHAN_SPACING_SET:
	{
		int ch_spacing;
		printk("%s: IOC_CHAN_SPACING_SET called\n", __func__);
		if (copy_from_user((void *) &ch_spacing, argp, sizeof(int)))
			ret = -EFAULT;
		else {
			ret =  Si4709_dev_ch_spacing_set(ch_spacing);
			if (ret < 0)
				pr_warn("%s: IOC_CHAN_SPACING_SET failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_CHAN_SELECT:
	{
		u32 frequency;
		printk("%s; IOC_CHAN_SELECT called\n", __func__);
		if (copy_from_user((void *) &frequency, argp, sizeof(u32)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_chan_select(frequency);
			if (ret < 0)
				pr_warn("%s: IOC_CHAN_SELECT failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_CHAN_GET:
	{
		u32 frequency;
		printk("%s: IOC_CHAN_GET called\n", __func__);
		ret = Si4709_dev_chan_get(&frequency);
		if (ret < 0)
			printk("%s: IOC_CHAN_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &frequency, sizeof(u32)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_SEEK_UP:
	{
		u32 frequency;
		printk("%s: IOC_SEEK_UP called\n", __func__);

		ret = Si4709_dev_seek_up(&frequency);
		if (ret < 0)
			pr_warn("%s: IOC_SEEK_UP failed\n", __func__);
		else if (copy_to_user(argp, (void *) &frequency, sizeof(u32)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_SEEK_DOWN:
	{
		u32 frequency;
		printk("%s: IOC_SEEK_DOWN called\n", __func__);

		ret = Si4709_dev_seek_down(&frequency);
		if (ret < 0)
			pr_warn("%s: IOC_SEEK_DOWN failed\n", __func__);
		else if (copy_to_user(argp, (void *) &frequency, sizeof(u32)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_RSSI_SEEK_TH_SET:
	{
		u8 RSSI_seek_th;
		printk("%s: IOC_RSSI_SEEK_TH_SET called\n", __func__);

		if (copy_from_user((void *) &RSSI_seek_th, argp, sizeof(u8)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_RSSI_seek_th_set(RSSI_seek_th);
			if (ret < 0)
				pr_warn("%s: IOC_RSSI_SEEK_TH_SET failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_SEEK_SNR_SET:
	{
		u8 seek_SNR_th;
		printk("%s:_IOC_SEEK_SNR_SET called\n", __func__);

		if (copy_from_user((void *) &seek_SNR_th, argp, sizeof(u8)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_seek_SNR_th_set(seek_SNR_th);
			if (ret < 0)
				pr_warn("%s: IOC_SEEK_SNR_SET failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_SEEK_CNT_SET:
	{
		u8 seek_FM_ID_th;
		printk("%s: IOC_SEEK_CNT_SET called\n", __func__);

		if (copy_from_user((void *) &seek_FM_ID_th, argp, sizeof(u8)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_seek_FM_ID_th_set(seek_FM_ID_th);
			if (ret < 0)
				pr_warn("%s: IOC_SEEK_CNT_SET failed",
					__func__);
		}
	}
		break;

	case Si4709_IOC_CUR_RSSI_GET:
	{
		rssi_snr_t  data;
		printk("%s: IOC_CUR_RSSI_GET called\n", __func__);

		ret = Si4709_dev_cur_RSSI_get(&data);
		if (ret < 0)
			pr_warn("%s: IOC_CUR_RSSI_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &data, sizeof(rssi_snr_t)))
			ret = -EFAULT;

		printk("curr_rssi:%d\ncurr_rssi_th:%d\ncurr_snr:%d\n",
			data.curr_rssi, data.curr_rssi_th, data.curr_snr);
	}
		break;

	case Si4709_IOC_VOLEXT_ENB:
		printk("%s: IOC_VOLEXT_ENB called\n", __func__);
		ret = Si4709_dev_VOLEXT_ENB();
		if (ret < 0)
			pr_warn("%s: IOC_VOLEXT_ENB failed\n", __func__);
		break;

	case Si4709_IOC_VOLEXT_DISB:
		printk("%s: IOC_VOLEXT_DISB called\n", __func__);
		ret = Si4709_dev_VOLEXT_DISB();
		if (ret < 0)
			pr_warn("%s: IOC_VOLEXT_DISB failed\n", __func__);
		break;

	case Si4709_IOC_VOLUME_SET:
	{
		u8 volume;
		printk("%s: IOC_VOLUME_SET called\n", __func__);

		if (copy_from_user((void *) &volume, argp, sizeof(u8)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_volume_set(volume);
			if (ret < 0)
				pr_warn("%s: IOC_VOLUME_SET failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_VOLUME_GET:
	{
		u8 volume;
		printk("%s: IOC_VOLUME_GET called\n", __func__);

		ret = Si4709_dev_volume_get(&volume);
		if (ret < 0)
			pr_warn("%s: IOC_VOLUME_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &volume, sizeof(u8)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_DSMUTE_ON:
		printk("%s: IOC_DSMUTE_ON called\n", __func__);
		ret = Si4709_dev_DSMUTE_ON();
		if (ret < 0)
			pr_err("%s: IOC_DSMUTE_ON failed\n", __func__);
		break;

	case Si4709_IOC_DSMUTE_OFF:
		printk("%s: IOC_DSMUTE_OFF called\n", __func__);
		ret = Si4709_dev_DSMUTE_OFF();
		if (ret < 0)
			pr_err("%s: IOC_DSMUTE_OFF failed\n", __func__);
		break;

	case Si4709_IOC_MUTE_ON:
		printk("%s: IOC_MUTE_ON called\n", __func__);
		ret = Si4709_dev_MUTE_ON();
		if (ret < 0)
			pr_warn("%s: IOC_MUTE_ON failed\n", __func__);
		break;

	case Si4709_IOC_MUTE_OFF:
		printk("%s: IOC_MUTE_OFF called\n", __func__);
		ret = Si4709_dev_MUTE_OFF();
		if (ret < 0)
			pr_warn("%s: IOC_MUTE_OFF failed\n", __func__);
		break;

	case Si4709_IOC_MONO_SET:
		printk("%s: IOC_MONO_SET called\n", __func__);
		ret = Si4709_dev_MONO_SET();
		if (ret < 0)
			pr_warn("%s: IOC_MONO_SET failed\n", __func__);
		break;

	case Si4709_IOC_STEREO_SET:
		printk("%s: IOC_STEREO_SET called\n", __func__);
		ret = Si4709_dev_STEREO_SET();
		if (ret < 0)
			pr_warn("%s: IOC_STEREO_SET failed\n", __func__);
		break;

	case Si4709_IOC_RSTATE_GET:
	{
		dev_state_t dev_state;

		printk("%s: IOC_RSTATE_GET called\n", __func__);
		ret = Si4709_dev_rstate_get(&dev_state);
		if (ret < 0)
			pr_warn("%s: IOC_RSTATE_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &dev_state,
			sizeof(dev_state_t)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_RDS_DATA_GET:
	{
		radio_data_t data;
		printk("%s: IOC_RDS_DATA_GET called\n", __func__);
		ret = Si4709_dev_RDS_data_get(&data);
		if (ret < 0)
			pr_warn("%s: IOC_RDS_DATA_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &data,
			sizeof(radio_data_t)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_RDS_ENABLE:
		printk("%s: IOC_RDS_ENABLE called\n", __func__);

		ret = Si4709_dev_RDS_ENABLE();
		if (ret < 0)
			pr_warn("%s: IOC_RDS_ENABLE failed\n", __func__);
		break;

	case Si4709_IOC_RDS_DISABLE:
		printk("%s: IOC_RDS_DISABLE called\n", __func__);

		ret = Si4709_dev_RDS_DISABLE();
		if (ret < 0)
			pr_warn("%s :IOC_RDS_DISABLE failed\n", __func__);
		break;

	case Si4709_IOC_RDS_TIMEOUT_SET:
	{
		u32  time_out;
		printk("%s: IOC_RDS_TIMEOUT_SET called\n", __func__);

		if (copy_from_user((void *) &time_out, argp, sizeof(u32)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_RDS_timeout_set(time_out);
			if (ret < 0)
				pr_warn("%s: IOC_RDS_TIMEOUT_SET failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_SEEK_CANCEL:
		printk("%s: IOC_SEEK_CANCEL called\n", __func__);
		if (Si4709_dev_wait_flag == SEEK_WAITING) {
			Si4709_dev_wait_flag = SEEK_CANCEL;
			wake_up_interruptible(&Si4709_waitq);
		}
		break;

	/* Switch Case statements for calling functions which reads device-id,
	 * chip-id,power configuration, system configuration2 registers
	 */
	case Si4709_IOC_CHIP_ID_GET:
	{
		chip_id chp_id;
		printk("%s: IOC_CHIP_ID called\n", __func__);
		ret = Si4709_dev_chip_id(&chp_id);
		if (ret < 0)
			pr_warn("%s: IOC_CHIP_ID failed\n", __func__);
		else if (copy_to_user(argp, (void *) &chp_id, sizeof(chip_id)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_DEVICE_ID_GET:
	{
		device_id dev_id;
		printk("%s: IOC_DEVICE_ID called\n", __func__);

		ret = Si4709_dev_device_id(&dev_id);
		if (ret < 0)
			pr_warn("%s: IOC_DEVICE_ID failed\n", __func__);
		else if (copy_to_user(argp, (void *) &dev_id,
			sizeof(device_id)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_SYS_CONFIG2_GET:
	{
		sys_config2 sys_conf2;
		printk("%s: IOC_SYS_CONFIG2 called\n", __func__);

		ret = Si4709_dev_sys_config2(&sys_conf2);
		if (ret < 0)
			pr_warn("%s: IOC_SYS_CONFIG2 failed\n", __func__);
		else if (copy_to_user(argp, (void *) &sys_conf2,
			sizeof(sys_config2)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_SYS_CONFIG3_GET:
	{
		sys_config3 sys_conf3;
		printk("%s: IOC_SYS_CONFIG3 called", __func__);

		ret = Si4709_dev_sys_config3(&sys_conf3);
		if (ret < 0)
			pr_warn("%s: IOC_SYS_CONFIG3 failed\n", __func__);
		else if (copy_to_user(argp, (void *) &sys_conf3,
			sizeof(sys_config3)))
			ret = -EFAULT;
	}
		break;

	case Si4709_IOC_POWER_CONFIG_GET:
	{
		power_config pow_conf;
		printk("%s: IOC_POWER_CONFIG called", __func__);

		ret = Si4709_dev_power_config(&pow_conf);
		if (ret < 0)
			pr_warn("%s: IOC_POWER_CONFIG failed\n", __func__);
		else if (copy_to_user(argp, (void *) &pow_conf,
			sizeof(power_config)))
			ret = -EFAULT;
	}
		break;
	/* VNVS:END */
	/* VNVS:START 18-NOV'09 */
	/* Reading AFCRL Bit */
	case Si4709_IOC_AFCRL_GET:
	{
		u8 afc;
		printk("%s: IOC_AFCRL_GET called\n", __func__);

		ret = Si4709_dev_AFCRL_get(&afc);
		if (ret < 0)
			printk("%s: IOC_AFCRL_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &afc, sizeof(u8)))
			ret = -EFAULT;
	}
		break;
	/* Setting DE-emphasis Time Constant.
	 * For DE=0,TC=50us(Europe,Japan,Australia) and DE=1,TC=75us(USA)
	 */
	case Si4709_IOC_DE_SET:
	{
		u8 de_tc;
		printk("%s: IOC_DE_SET called\n", __func__);

		if (copy_from_user((void *) &de_tc, argp, sizeof(u8)))
			ret = -EFAULT;
		else {
			ret = Si4709_dev_DE_set(de_tc);
			if (ret < 0)
				pr_warn("%s: IOC_DE_SET failed\n", __func__);
		}
	}
		break;

	case Si4709_IOC_STATUS_RSSI_GET:
	{
		status_rssi status;
		printk("%s: IOC_STATUS_RSSI_GET called\n", __func__);

		ret = Si4709_dev_status_rssi(&status);
		if (ret < 0)
			pr_warn("%s: IOC_STATUS_RSSI_GET failed\n", __func__);
		else if (copy_to_user(argp, (void *) &status,
			sizeof(status_rssi)))
			ret = -EFAULT;
	}
		break;


	case Si4709_IOC_SYS_CONFIG2_SET:
	{
		sys_config2 sys_conf2;
		unsigned long n;
		printk("%s: IOC_SYS_CONFIG2_SET called\n", __func__);

		n = copy_from_user((void *) &sys_conf2, argp,
			sizeof(sys_config2));
		if (n) {
			pr_err("%s: Failed to read [%d] byes!\n", __func__, n);
			ret = -EFAULT;
		} else {
			ret = Si4709_dev_sys_config2_set(&sys_conf2);
			if (ret < 0)
				pr_warn("%s: IOC_SYS_CONFIG2_SET failed\n",
					__func__);
		}
	}
		break;

	case Si4709_IOC_SYS_CONFIG3_SET:
	{
		sys_config3 sys_conf3;
		unsigned long n;
		printk("%s: IOC_SYS_CONFIG3_SET called", __func__);
		n = copy_from_user((void *) &sys_conf3, argp,
			sizeof(sys_config3));
		if (n < 0) {
			pr_err("%s: Failed to read [%d] byes!\n", __func__, n);
			ret = -EFAULT;
		} else {
			ret = Si4709_dev_sys_config3_set(&sys_conf3);
			if (ret < 0)
				pr_warn("%s: IOC_SYS_CONFIG3_SET failed\n",
					__func__);
		}
	}
	break;

	/*Resetting the RDS Data Buffer*/
	case Si4709_IOC_RESET_RDS_DATA:
		printk("%s: IOC_RESET_RDS_DATA called\n", __func__);
		ret = Si4709_dev_reset_rds_data();
		if (ret < 0)
			pr_warn("%s: IOC_RESET_RDS_DATA failed\n", __func__);
		break;

	default:
		pr_warn("%s: ioctl default\n", __func__);
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static irqreturn_t Si4709_isr(int irq, void *unused)
{
	printk("%s: irq=%d\n", __func__, irq);
#ifdef RDS_INTERRUPT_ON_ALWAYS
	if ((Si4709_dev_wait_flag == SEEK_WAITING)
		|| (Si4709_dev_wait_flag == TUNE_WAITING)) {
		printk("%s: SEEK or TUNE waiting\n", __func__);
		Si4709_dev_wait_flag = WAIT_OVER;
		wake_up_interruptible(&Si4709_waitq);
	} else if (Si4709_RDS_flag == RDS_WAITING) { /* RDS Interrupt */
		printk("%s: RDS waiting\n", __func__);
		RDS_Data_Available++;
		RDS_Groups_Available_till_now++;

		printk("%s: RDS_Available_till_now b/w Power ON/OFF : %d",
			__func__, RDS_Groups_Available_till_now);

		if (RDS_Data_Available > 1)
			RDS_Data_Lost++;

		if (!work_pending(&Si4709_work))
			queue_work(Si4709_wq, &Si4709_work);
	}
#else
	if (Si4709_dev_wait_flag == SEEK_WAITING
		|| Si4709_dev_wait_flag == TUNE_WAITING
		|| Si4709_dev_wait_flag == RDS_WAITING) {
		Si4709_dev_wait_flag = WAIT_OVER;
		wake_up_interruptible(&Si4709_waitq);
	}
#endif
	return IRQ_HANDLED;
}

void debug_ioctls(void)
{

	printk("------------------------------------------------");

	printk("Si4709_IOC_POWERUP 0x%x", Si4709_IOC_POWERUP);

	printk("Si4709_IOC_POWERDOWN 0x%x", Si4709_IOC_POWERDOWN);

	printk("Si4709_IOC_BAND_SET 0x%x", Si4709_IOC_BAND_SET);

	printk("Si4709_IOC_CHAN_SPACING_SET 0x%x", Si4709_IOC_CHAN_SPACING_SET);

	printk("Si4709_IOC_CHAN_SELECT 0x%x", Si4709_IOC_CHAN_SELECT);

	printk("Si4709_IOC_CHAN_GET 0x%x", Si4709_IOC_CHAN_GET);

	printk("Si4709_IOC_SEEK_UP 0x%x", Si4709_IOC_SEEK_UP);

	printk("Si4709_IOC_SEEK_DOWN 0x%x", Si4709_IOC_SEEK_DOWN);

	printk("Si4709_IOC_RSSI_SEEK_TH_SET 0x%x", Si4709_IOC_RSSI_SEEK_TH_SET);

	printk("Si4709_IOC_SEEK_SNR_SET 0x%x", Si4709_IOC_SEEK_SNR_SET);

	printk("Si4709_IOC_SEEK_CNT_SET 0x%x", Si4709_IOC_SEEK_CNT_SET);

	printk("Si4709_IOC_CUR_RSSI_GET 0x%x", Si4709_IOC_CUR_RSSI_GET);

	printk("Si4709_IOC_VOLEXT_ENB 0x%x", Si4709_IOC_VOLEXT_ENB);

	printk("Si4709_IOC_VOLEXT_DISB 0x%x", Si4709_IOC_VOLEXT_DISB);

	printk("Si4709_IOC_VOLUME_SET 0x%x", Si4709_IOC_VOLUME_SET);

	printk("Si4709_IOC_VOLUME_GET 0x%x", Si4709_IOC_VOLUME_GET);

	printk("Si4709_IOC_MUTE_ON 0x%x", Si4709_IOC_MUTE_ON);

	printk("Si4709_IOC_MUTE_OFF 0x%x", Si4709_IOC_MUTE_OFF);

	printk("Si4709_IOC_MONO_SET 0x%x", Si4709_IOC_MONO_SET);

	printk("Si4709_IOC_STEREO_SET 0x%x", Si4709_IOC_STEREO_SET);

	printk("Si4709_IOC_RSTATE_GET 0x%x", Si4709_IOC_RSTATE_GET);

	printk("Si4709_IOC_RDS_DATA_GET 0x%x", Si4709_IOC_RDS_DATA_GET);

	printk("Si4709_IOC_RDS_ENABLE 0x%x", Si4709_IOC_RDS_ENABLE);

	printk("Si4709_IOC_RDS_DISABLE 0x%x", Si4709_IOC_RDS_DISABLE);

	printk("Si4709_IOC_RDS_TIMEOUT_SET 0x%x", Si4709_IOC_RDS_TIMEOUT_SET);

	printk("Si4709_IOC_DEVICE_ID_GET 0x%x", Si4709_IOC_DEVICE_ID_GET);

	printk("Si4709_IOC_CHIP_ID_GET 0x%x", Si4709_IOC_CHIP_ID_GET);

	printk("Si4709_IOC_SYS_CONFIG2_GET 0x%x", Si4709_IOC_SYS_CONFIG2_GET);

	printk("Si4709_IOC_POWER_CONFIG_GET 0x%x", Si4709_IOC_POWER_CONFIG_GET);

	printk("Si4709_IOC_AFCRL_GET 0x%x", Si4709_IOC_AFCRL_GET);

	printk("Si4709_IOC_DE_SET 0x%x", Si4709_IOC_DE_SET);

	printk("Si4709_IOC_DSMUTE_ON 0x%x", Si4709_IOC_DSMUTE_ON);

	printk("Si4709_IOC_DSMUTE_OFF 0x%x", Si4709_IOC_DSMUTE_OFF);

	printk("Si4709_IOC_RESET_RDS_DATA 0x%x", Si4709_IOC_RESET_RDS_DATA);

	printk("------------------------------------------------");

}




int Si4709_driver_init(void)
{
	int ret = 0;

	printk("[diony]%s:\n", __func__);

	/*Initialize the Si4709 dev mutex*/
	Si4709_dev_mutex_init();

	/*misc device registration*/
	ret = misc_register(&Si4709_misc_device);
	if (ret < 0) {
		pr_err("%s: misc_register failed\n", __func__);
		return ret;
	}
#if 0
	s3c_gpio_cfgpin(GPIO_FM_INT, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_FM_INT, S3C_GPIO_PULL_NONE);
#endif
if (gpio_tlmm_config(GPIO_CFG(GPIO_FM_INT, 0, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, GPIO_FM_INT);

	set_irq_type(FM_IRQ_INT, IRQ_TYPE_EDGE_FALLING);

	ret = request_irq(FM_IRQ_INT, Si4709_isr, IRQF_DISABLED, "Si4709",
		NULL);
	if (ret) {
		pr_err("%s: request_irq failed %d\n", __func__, GPIO_FM_INT);
		goto MISC_DREG;
	}

	if (gpio_is_valid(FM_RESET)) {
		if (gpio_request(FM_RESET, "GPJ2"))
			printk(KERN_ERR "Failed to request FM_RESET!\n");
		gpio_direction_output(FM_RESET, 0);
	}

	/* VNVS: 13-OCT'09---- Initially Pulling the interrupt pin HIGH
	 * as the FM Radio device gives 5ms low pulse
	 */
#if 0
	s3c_gpio_setpull(GPIO_FM_INT, S3C_GPIO_PULL_UP);
#endif
if (gpio_tlmm_config(GPIO_CFG(GPIO_FM_INT, 0, GPIO_CFG_INPUT,
				  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, GPIO_FM_INT);
	/* Resetting the device */
	gpio_set_value(FM_RESET, 0);
	gpio_set_value(FM_RESET, 1);
	/* Freeing the FM_RESET pin */
	gpio_free(FM_RESET);

	/* Add the i2c driver */
	ret = Si4709_i2c_drv_init();
	if (ret < 0)
		goto MISC_IRQ_DREG;

	init_waitqueue_head(&Si4709_waitq);

	printk("Si4709_driver_init successful");

	return ret;

MISC_IRQ_DREG:
	free_irq(FM_IRQ_INT, NULL);
MISC_DREG:
	misc_deregister(&Si4709_misc_device);

    return ret;
}


void Si4709_driver_exit(void)
{
	printk("%s:\n", __func__);

	/*Delete the i2c driver*/
	Si4709_i2c_drv_exit();
	free_irq(FM_IRQ_INT, NULL);

	/*misc device deregistration*/
	misc_deregister(&Si4709_misc_device);
}

module_init(Si4709_driver_init);
module_exit(Si4709_driver_exit);
MODULE_AUTHOR("Varun Mahajan <m.varun@samsung.com>");
MODULE_DESCRIPTION("Si4709 FM tuner driver");
MODULE_LICENSE("GPL");

