/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, SAMSUNG. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>

#include "lsi_mc7.h"
#include "mc7_firmware.h"
#include "mc7_fl_loader.h"
#include "mc7_fl_writer.h"

/* Used for firmware update through sysfs */
#define FIRMWARE_TOTAL_LEN	380928		/* Current length of firmware */
#define FIRMWARE_STORE_ADDR	0x68000000	/* Internal RAm addr of MC7 where firware is downloaded */

/* FIRMWARE_TOTAL_LEN = (FIRMWARE_PACKET_LEN * 11) + LAST_PKT_LEN */
#define FIRMWARE_PACKET_LEN	32768	/* Firmware is divided into packets of 32KB */
#define LAST_PKT_LEN		20480	/* The remaining bytes are collected in a smaller packet */
#define TOTAL_PKTS		12	/* 11 packets of length 32KB and 1 packet of length 20480 */

/* Last member is used to mark the end of the array */
static struct packet dynamic_firmware_array[TOTAL_PKTS + 1];

/*
 * If the firmware length matches FIRMWARE_TOTAL_LEN, mc7_firmware_upd will be
 * TRUE and the firmware will be updated during the next open of camera
 */
static int mc7_firmware_upd;
static unsigned char *firmware_area = NULL;
static unsigned char *f_curr_p = NULL;

static unsigned int firmware_array_len = 0; /* length of one firmware array */
static unsigned int total_firmware_len = 0; /* counter for total length */
static unsigned int total_firmware_pkt = 0; /* total no of firmware packets */
static const char firmware_ver[] =  {'2', '9', 'o', 'c', 't', '1', '0', 0};

/* Used to save the i2c client structure */
static struct i2c_client *mc7_i2c_client;
static struct i2c_client *lp8720_i2c_client;

static unsigned int config_csi;
static struct mc7_ctrls_t *mc7_ctrls;

static DECLARE_WAIT_QUEUE_HEAD(mc7_irq_wait);
static int mc7_gpio_int;

static void mc7_worker_fun(struct work_struct *work)
{
	mc7_gpio_int = 1;
	wake_up(&mc7_irq_wait);
}
static DECLARE_WORK(mc7_work, mc7_worker_fun);

static irqreturn_t mc7_int_handler(int irq, void *dev_id)
{
	FUNC_ENTER();
	schedule_work(&mc7_work);
	return IRQ_HANDLED;
}

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
	int rc;
	unsigned char buf[2];
	struct i2c_msg msg = {
		.addr = lp8720_i2c_client->addr,
		.flags = 0,
		.len = 2,
		.buf = buf,
	};

	buf[0] = addr;
	buf[1] = data;

	rc = i2c_transfer(lp8720_i2c_client->adapter, &msg, 1);
	return (rc == 1) ? 0 : -EIO;
}

static int mc7_catparam_i2c_read(unsigned char len, unsigned char cate, unsigned char byte,
		unsigned int *val)
{
	int rc;
	unsigned char tx_buf[5];
	unsigned char rx_buf[len + 1];
	struct i2c_msg msg = {
		.addr = mc7_i2c_client->addr,
		.flags = 0,
		.len = 5,
		.buf = tx_buf,
	};

	tx_buf[0] = 0x05;
	tx_buf[1] = 0x01;
	tx_buf[2] = cate;
	tx_buf[3] = byte;
	tx_buf[4] = len;

	rc = i2c_transfer(mc7_i2c_client->adapter, &msg, 1);
	if (likely(rc == 1)) {
		msg.flags = I2C_M_RD;
		msg.len = len + 1;
		msg.buf = rx_buf;
		rc = i2c_transfer(mc7_i2c_client->adapter, &msg, 1);
	} else {
		printk(KERN_ERR "LSI-MC7: %s: failed at byte=%x, category=%x \n", __func__, byte, cate);
		return -EIO;
	}

	if (likely(rc == 1)) {
		MC7DBG("No of bytes read = %x\n", rx_buf[0]);
		if (len == 1)
			*val = rx_buf[1];
		else if (len == 2)
			*(unsigned short *)val = be16_to_cpu(*(unsigned short *)(&rx_buf[1]));
		else
			*val = be32_to_cpu(*(unsigned int *)(&rx_buf[1]));

		return 0;
	}

	printk(KERN_ERR "LSI-MC7: %s: failed at byte=%x, category=%x \n", __func__, byte, cate);
	return -EIO;
}

static int mc7_catparam_i2c_write(unsigned char len, unsigned char cate, unsigned char byte,
		unsigned int val)
{
	int rc;
	unsigned char tx_buf[len + 4];
	struct i2c_msg msg = {
		.addr = mc7_i2c_client->addr,
		.flags = 0,
		.len = len + 4,
		.buf = tx_buf,
	};

	tx_buf[0] = len + 4;
	tx_buf[1] = 0x02;
	tx_buf[2] = cate;
	tx_buf[3] = byte;

	if (len == 1)
		tx_buf[4] = (unsigned char)(val & 0xff);
	else if (len == 2)
		*(unsigned short *)(&tx_buf[4]) = cpu_to_be16((unsigned short)val);
	else
		*(unsigned int *)(&tx_buf[4]) = cpu_to_be32(val);

	rc = i2c_transfer(mc7_i2c_client->adapter, &msg, 1);
	if (unlikely (rc != 1))
		printk(KERN_ERR "LSI-MC7: %s: failed at byte=%x, category=%x \n", __func__, byte, cate);

	return (rc == 1) ? 0 : -EIO;
}

#if 0
static inline int mc7_memory_32bit_i2c_read(unsigned int addr, unsigned short len, unsigned int *data)
{
	int rc;
	unsigned char tx_buf[8];
	unsigned char rx_buf[len + 3];
	struct i2c_msg msg = {
		.addr = mc7_i2c_client->addr,
		.flags = 0,
		.len = 8,
		.buf = tx_buf,
	};

	if (len != 4) {
		printk(KERN_ERR "LSI-MC7: data length other than 4 is not supported now\n");
		return -1;
	}

	tx_buf[0] = 0x00;
	tx_buf[1] = 0x07;
	*(unsigned int *)(&tx_buf[2]) = cpu_to_be32(addr);
	*(unsigned short *)(&tx_buf[6]) = cpu_to_be16(len);

	rc = i2c_transfer(mc7_i2c_client->adapter, &msg, 1);
	if (likely(rc == 1)) {
		msg.flags = I2C_M_RD;
		msg.len = len + 3;
		msg.buf = rx_buf;
		rc = i2c_transfer(mc7_i2c_client->adapter, &msg, 1);
	} else {
		printk(KERN_ERR "LSI-MC7: %s: failed at addr=%x, len= %x\n", __func__, addr, len);
		return -EIO;
	}

	if (likely(rc == 1)) {
		MC7DBG("No of bytes read = 0x%x%x\n", rx_buf[1], rx_buf[2]);
		*data = be32_to_cpu(*(unsigned int *)(&rx_buf[3]));
		return 0;
	}

	printk(KERN_ERR "LSI-MC7: %s: failed at addr=%x, len= %x\n", __func__, addr, len);
	return -EIO;
}
#endif

static int firmware_write(unsigned short len, unsigned char *firmware)
{
	int rc;
	struct i2c_msg msg = {
		.addr = mc7_i2c_client->addr,
		.flags = 0,
		.len = len,
		.buf = firmware,
	};
	
	MC7DBG("%s : writing length = %d\n", __func__, len);
	rc = i2c_transfer(mc7_i2c_client->adapter, &msg, 1);
	if (unlikely (rc != 1))
		printk(KERN_ERR "LSI-MC7: %s: failed\n", __func__);

	return (rc == 1) ? 0 : -EIO;
}

static inline int poll_mc7_int(int delay_sec)
{
	int i, rc = 0;
	FUNC_ENTER();

	if (gpio_request(175, "lsi_mc7"))
		printk(KERN_ERR "LSI-MC7: %s: gpio request 'sensor_pwd' failed\n", __func__);

	gpio_direction_input(175);

	for (i = 0; i < (delay_sec * 1000); i++) {
		if (gpio_get_value(175)) {
			MC7DBG("Interrupt occured\n");
			break;
		} else
			msleep(1);
	}

	if (i == (delay_sec * 1000)) {
		printk(KERN_ERR "LSI-MC7: %s: No interrupt from MC7 waited for %d seconds\n", __func__, delay_sec);
		rc = -1;
	}

	gpio_free(175);
	return rc;
}

static inline int wait_mc7_int(int delay_sec)
{
	int rc;
	FUNC_ENTER();

	rc = wait_event_interruptible_timeout(mc7_irq_wait, (mc7_gpio_int == 1), msecs_to_jiffies(1000 * delay_sec));
	mc7_gpio_int = 0;

	if (unlikely(rc <= 0)) {
		if (rc == 0)
			printk(KERN_ERR "LSI-MC7: Waiting for the event timed out\n");
		else
			printk(KERN_ERR "LSI-MC7: Waiting for the event interrupted by signal\n");
		rc = -1;
	} else {
		MC7DBG("waiting for the event successful\n");
		rc = 0;
	}
	return rc;
}

static int mc7_sensor_power_on_off(int onoff, const struct msm_camera_sensor_info *data)
{
	unsigned int mclk_cfg; 
	int rc = 0;

	//FUNC_ENTER();
	
	if (gpio_request(data->vcm_enable, "lsi_mc7"))
		printk(KERN_ERR "LSI-MC7: %s: gpio request 'vcm_enable' failed\n", __func__);

	if (gpio_request(data->sensor_reset, "lsi_mc7"))
		printk(KERN_ERR "LSI-MC7: %s: gpio request 'sensor_reset' failed\n", __func__);

	if (onoff) {
		printk(KERN_ERR "LSI-MC7: %s: POWER ON.\n", __func__);
		
		mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		
		gpio_direction_output(data->vcm_enable, 0);
		gpio_direction_output(data->sensor_reset, 0);

		/* By default BUCK is 1.2V so no need to configure the voltage; only change the start up time from NO start */
		rc = lp8720_i2c_write(0x06, 0x1A); /* BUCK: no delay for 1.2v */
		if (rc < 0)
			printk(KERN_ERR "%s: lp8720_i2c_write failed: %d\n",__func__, rc);		
		lp8720_i2c_write(0x07, 0x09); /* BUCK:1.2v ; Not required by default it is 0x09 */
		lp8720_i2c_write(0x05, 0x2C); /* LDO5:1.8V, delay of 1 'time step' ts */
		lp8720_i2c_write(0x03, 0x2C); /* for janice 0.0 board 2011-01-20*/

		lp8720_i2c_write(0x01, 0x19); /* LDO1:2.8v, no delay; For AF */
		lp8720_i2c_write(0x02, 0x19); /* LDO2:2.8v, no delay; For the sensor */

		//lp8720_i2c_write(0x08, 0xB3); /* Enable all voltages except LDO3 and LDO4 */
		lp8720_i2c_write(0x08, 0xB7); /* for janice 0.0 board 2011-01-20*/
		
		gpio_set_value(data->vcm_enable, 1);
		usleep(200);

		/* Enable MCLK */
		gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
		udelay(50);

		/* Make RST pin high */
		gpio_set_value(data->sensor_reset, 1);
	} else {
		printk(KERN_ERR "LSI-MC7: %s: POWER OFF.\n", __func__);
		gpio_direction_output(data->vcm_enable, 1);
		gpio_direction_output(data->sensor_reset, 1);

		mclk_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

		/* Make RST pin low */
		gpio_set_value(data->sensor_reset, 0);

		/* Disable MCLK */
		gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
		udelay(50);
		
		/* Disable 1.8v, AF 2.8v and Sensor 2.8v */
		lp8720_i2c_write(0x08, 0xA0);
		udelay(50);

		/* Disable 1.2v */
		lp8720_i2c_write(0x08, 0x80);

		/* Disable lp8720 */
		gpio_set_value(data->vcm_enable, 0);
	}

	gpio_free(data->vcm_enable);
	gpio_free(data->sensor_reset);
	return rc;
}

/* Used to bring an empty MC7 to working stage */
#define DOWNLOAD_IF_MC7_EMPTY
static int mc7_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	unsigned int val;
	struct packet *i2c_pkt;

	FUNC_ENTER();

	/* Reset MIPI configuration flag */
	config_csi = 0;

	if (mc7_i2c_client->irq)
		rc = request_irq(mc7_i2c_client->irq, mc7_int_handler,
			       IRQF_DISABLED | IRQF_TRIGGER_RISING, mc7_i2c_client->name, NULL);
	if (rc)
		printk(KERN_ERR "LSI-MC7: %s: cannot request irq\n", __func__);

	mc7_ctrls = kzalloc(sizeof(struct mc7_ctrls_t), GFP_KERNEL);
	if (!mc7_ctrls) {
		printk(KERN_ERR "LSI-MC7: %s: probe from VFE failed\n", __func__);
		rc = -ENOMEM;
		goto vfe_probe_fail0;
	}

	if (data)
		mc7_ctrls->sensordata = data;

	msm_camio_clk_rate_set(24000000);

	mc7_gpio_int = 0;
	mc7_sensor_power_on_off(1 ,data);
	rc = wait_mc7_int(60);

	/* If the user has set update firmware flag then update the firmware first */
	if (mc7_firmware_upd) {
		mc7_firmware_upd = 0;
		/* write PLL divider value */
		mc7_catparam_i2c_write(2, 0xf, 0x24, 0x001A);

		/* firmware is send first */
		for (i2c_pkt = &dynamic_firmware_array[0]; i2c_pkt->length != 0; i2c_pkt++)
			firmware_write(i2c_pkt->length, i2c_pkt->firmware);
		/* Now send the flash writer program */
		for (i2c_pkt = &mc7_fl_write_array[0]; i2c_pkt->length != 0; i2c_pkt++)
			firmware_write(i2c_pkt->length, i2c_pkt->firmware);

		mc7_catparam_i2c_write(4, 0xf, 0x0C, 0x6807E000);
		mc7_catparam_i2c_write(1, 0xf, 0x12, 0x01);

		rc = wait_mc7_int(300);
		if (!rc)
			printk(KERN_ERR "LSI-MC7: firmware successfully writen to serial flash\n");
		else {
			printk(KERN_ERR "LSI-MC7: firmware update failed\n");
			return rc;
		}
		vfree(firmware_area);
		firmware_area = NULL;
	}

	/* Set the PLL divider value */
	mc7_catparam_i2c_write(2, 0xf, 0x24, 0x001A);

	/* Now send the flash loader program */
	firmware_write(sizeof(mc7_fl_load_1), mc7_fl_load_1);

	/* Set the camera start address */
	mc7_catparam_i2c_write(4, 0xf, 0x0C, 0x6807E000);

	/* start the loader program */
	mc7_catparam_i2c_write(1, 0xf, 0x12, 0x01);
	rc = wait_mc7_int(1);

#ifdef DOWNLOAD_IF_MC7_EMPTY
	if (rc) {
		printk(KERN_ERR "LSI-MC7: serial flash is empty, going to write it\n");
		/* Power recycle MC7 */
		mc7_sensor_power_on_off(0 ,data);
		mc7_sensor_power_on_off(1 ,data);
		rc = wait_mc7_int(60);

		/* write PLL divider value */
		mc7_catparam_i2c_write(2, 0xf, 0x24, 0x001A);

		/* firmware is send first */
		for (i2c_pkt = &mc7_firmware_array[0]; i2c_pkt->length != 0; i2c_pkt++)
			firmware_write(i2c_pkt->length, i2c_pkt->firmware);

		/* Now send the flash writer program */
		for (i2c_pkt = &mc7_fl_write_array[0]; i2c_pkt->length != 0; i2c_pkt++)
			firmware_write(i2c_pkt->length, i2c_pkt->firmware);

		/* write the address of the program */
		mc7_catparam_i2c_write(4, 0xf, 0x0C, 0x6807E000);

		/* start the writer program */
		mc7_catparam_i2c_write(1, 0xf, 0x12, 0x01);

		rc = wait_mc7_int(300);
		if (!rc)
			printk(KERN_ERR "LSI-MC7: firmware successfully writen to serial flash\n");
		else
			printk(KERN_ERR "LSI-MC7: firmware update failed\n");

		/* Set the PLL divider value */
		mc7_catparam_i2c_write(2, 0xf, 0x24, 0x001A);

		/* Now send the flash loader program */
		firmware_write(sizeof(mc7_fl_load_1), mc7_fl_load_1);

		/* Set the camera start address */
		mc7_catparam_i2c_write(4, 0xf, 0x0C, 0x6807E000);

		/* start the loader program */
		mc7_catparam_i2c_write(1, 0xf, 0x12, 0x01);
		rc = wait_mc7_int(1);
	}
#endif
	/* Read the interrupt status */
	mc7_catparam_i2c_read(1, 0x0, 0x1c, &val);
	MC7DBG("Loader program successfully booted the firmware\n");

	return rc;
vfe_probe_fail0:
	return rc;
}

static int return_from_capture;
static int mc7_monitor_mode(int mode)
{
	int rc = 0;
	unsigned int val;
	struct msm_camera_csi_params mc7_csi_params;

	FUNC_ENTER();

	if (!config_csi) {
		mc7_csi_params.lane_cnt = 2;
		mc7_csi_params.data_format = CSI_8BIT;
		mc7_csi_params.lane_assign = 0xe4;
		mc7_csi_params.dpcm_scheme = 0;
		mc7_csi_params.settle_cnt = 0x14;
		rc = msm_camio_csi_config(&mc7_csi_params);
		if (rc < 0)
			printk(KERN_ERR "config csi controller failed \n");
		config_csi = 1;
	}

	if (return_from_capture) {
		/* Stop sending large frames */
		mc7_catparam_i2c_write(1, 0x0c, 0x09, 0x02);
		return_from_capture = 0;
	}

	mc7_catparam_i2c_write(1, 0x01, 0x01, 0x17);
	mc7_catparam_i2c_write(1, 0x00, 0x10, 0x01);
	mc7_catparam_i2c_write(1, 0x00, 0x0B, 0x02);
	wait_mc7_int(60);
	mc7_catparam_i2c_read(1, 0x0, 0x1c, &val);
	/* Now MC7 starts outputing YUV data */
	MC7DBG("successfully entered monitor mode\n");

	/* Move the lens to infinity */
	mc7_catparam_i2c_write(1, 0x0a, 0x10, 0x01);

	return rc;
}

static int mc7_capture_mode(int mode)
{
	int rc = 0;
	unsigned int val;

	FUNC_ENTER();

	mc7_catparam_i2c_write(1, 0x0B, 0x00, 0x00);
	mc7_catparam_i2c_write(1, 0x0B, 0x01, 0x1F);
	mc7_catparam_i2c_write(1, 0x00, 0x10, 0x08);
	mc7_catparam_i2c_write(1, 0x00, 0x0B, 0x03);
	wait_mc7_int(60);
	MC7DBG("Completed image processing\n");
	/* Now MC7 has completed processing the image */

	mc7_catparam_i2c_read(1, 0x0, 0x1c, &val); /* clear interrupt */

	/* Send command to start image output */
	mc7_catparam_i2c_write(1, 0x0c, 0x09, 0x01);
	return_from_capture = 1;

	return rc;
}

static int mc7_set_sensor_mode(int mode, int res)
{
	int rc = 0;
	FUNC_ENTER();
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mc7_monitor_mode(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = mc7_capture_mode(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int mc7_af_operation(void)
{
	unsigned int val;

	FUNC_ENTER();

	mc7_catparam_i2c_write(1, 0x0B, 0x40, 0x01);
	mc7_catparam_i2c_write(1, 0x0B, 0x41, 0x01);

	mc7_catparam_i2c_write(1, 0x0a, 0x10, 0x01);
	mc7_catparam_i2c_write(1, 0x0a, 0x02, 0x01);

	msleep(100);
	mc7_catparam_i2c_read(1, 0x0a, 0x03, &val);
	while(1) {
		if((val & 0xff) != 0x00)
			break;
		else
			MC7DBG("AF in progress\n");
		mc7_catparam_i2c_read(1, 0x0a, 0x03, &val);
	}

	mc7_catparam_i2c_write(1, 0x0a, 0x02, 0x00);
	MC7DBG("AF result = %d\n", val & 0xff);
	return (val & 0xff);
}

int mc7_sensor_ext_config(void __user *argp)
{
	sensor_ext_cfg_data cfg_data;
	int rc = 0;

	FUNC_ENTER();

	if (copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
		return -EFAULT;

	MC7DBG("mc7_sensor_ext_config, cmd = %d ",cfg_data.cmd);

	switch(cfg_data.cmd) {
	case EXT_CFG_SET_FLASH:
	case EXT_CFG_SET_SCENE:
	case EXT_CFG_SET_SHARPNESS:
	case EXT_CFG_SET_SATURATION:
	case EXT_CFG_SET_ISO:
	case EXT_CFG_SET_WB:
	case EXT_CFG_SET_CONTRAST:
	case EXT_CFG_SET_BRIGHTNESS:
	case EXT_CFG_SET_ZOOM:
	case EXT_CFG_SET_FPS:
	case EXT_CFG_SET_AF_MODE:
	case EXT_CFG_SET_AF_START:
		rc = mc7_af_operation();
		break;
	case EXT_CFG_SET_AF_STOP:
	case EXT_CFG_GET_AF_STATUS:
	case EXT_CFG_SET_FACE_DETECT:
	case EXT_CFG_SET_METERING:
	case EXT_CFG_SET_CONTINUOUS_AF:
	case EXT_CFG_SET_PREVIEW_SIZE:
	case EXT_CFG_SET_PICTURE_SIZE:
	case EXT_CFG_SET_JPEG_QUALITY:
	case EXT_CFG_SET_TOUCHAF_POS:
	case EXT_CFG_SET_ANTISHAKE:
	case EXT_CFG_SET_WDR:
	case EXT_CFG_SET_EXIF:
	default:
		break;
	}

	return rc;
}

static int mc7_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	int rc = 0;
	FUNC_ENTER();
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	MC7DBG("mc7_sensor_config: cfgtype = %d\n", cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		rc = mc7_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	default:
		break;
	}
	return 0;
}

static int mc7_sensor_release(void)
{
	FUNC_ENTER();
	kfree(mc7_ctrls);
	free_irq(mc7_i2c_client->irq, NULL);
	mc7_ctrls = NULL;
	return 0;
}

static int cam_pm_lp8720_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	FUNC_ENTER();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "LSI-MC7: %s: mc7 i2c probe failed\n", __func__);
		return -1;
	}

	lp8720_i2c_client = client;
	MC7DBG("lp8720 i2c probe successful\n");
	return 0;
}

static int cam_pm_lp8720_remove(struct i2c_client *client)
{
	FUNC_ENTER();
	lp8720_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id cam_pm_lp8720_id[] = {
	{ "cam_pm_lp8720", 0 },
	{ }
};

static struct i2c_driver lp8720_i2c_driver = {
	.id_table 	= cam_pm_lp8720_id,
	.probe  	= cam_pm_lp8720_probe,
	.remove 	= cam_pm_lp8720_remove,
	.driver 	= {
		.name = "cam_pm_lp8720",
	},
};

static ssize_t mc7_firmware_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	ret = snprintf(buf, sizeof(firmware_ver) + 1, "%s\n", firmware_ver);
	return ret;
}

static ssize_t mc7_firmware_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (firmware_area == NULL) {
		printk("Storing firmware started\n");

		/* Allocate 400KB to hold the firmware */
		firmware_area = vmalloc(400 * 1024);

		/* Reset the variables */
		f_curr_p = firmware_area;
		firmware_array_len = 0;
		total_firmware_len = 0;
		total_firmware_pkt = 0;
	} else if (mc7_firmware_upd == 1) {
		/* If the user tries to write another firmware without opening the camera it comes here */
		printk("Going to overwrite the previously saved firmware\n");

		/* Reset the variables */
		f_curr_p = firmware_area;
		firmware_array_len = 0;
		total_firmware_len = 0;
		total_firmware_pkt = 0;
		mc7_firmware_upd = 0;
	}

	/* Collect 32KB data and form the packet; the last packet will be of less length */
	if ((firmware_array_len + count) <= FIRMWARE_PACKET_LEN) {
		if (firmware_array_len == 0) {
			/* Make the packet header for the new packet */
			printk("New packet no = %d\n", total_firmware_pkt + 1);

			/* Store the starting address of the packet */
			dynamic_firmware_array[total_firmware_pkt].firmware = f_curr_p;

			/*
			 * Packet header length is 8bytes
			 * byte[0] = 0x00; Always same value
			 * byte[1] = 0x04; Always same value
			 * byte[2,3,4,5] = Ram address of MC7 to store the firmware
			 * byte[6,7] = size of the firmware data in the current packet
			 */
			*f_curr_p++ = 0x00;
			*f_curr_p++ = 0x04;

			*(unsigned int *)(f_curr_p) = cpu_to_be32(FIRMWARE_STORE_ADDR + (FIRMWARE_PACKET_LEN * total_firmware_pkt));
			printk("Packet will be written to MC7 RAM address  = %x\n", be32_to_cpu(*(unsigned int *)(f_curr_p)));

			f_curr_p += 4;

			/* The last packet is of length LAST_PKT_LEN and every other packet is of length FIRMWARE_PACKET_LEN */
			if (total_firmware_pkt == (TOTAL_PKTS - 1)) {
				*(unsigned short *)(f_curr_p) = cpu_to_be16(LAST_PKT_LEN);
				/* Store the total length of the i2c packet; data + header */
				dynamic_firmware_array[total_firmware_pkt].length = LAST_PKT_LEN + 8;
				/* Mark the ending */
				dynamic_firmware_array[total_firmware_pkt + 1].length = 0;
			} else {
				*(unsigned short *)(f_curr_p) = cpu_to_be16(FIRMWARE_PACKET_LEN);
				dynamic_firmware_array[total_firmware_pkt].length = FIRMWARE_PACKET_LEN + 8;
			}

			f_curr_p += 2;

			total_firmware_pkt++;
		}

		/* Copy the firmware */
		memcpy(f_curr_p, buf, count);

		/* Increment the counters */
		firmware_array_len = firmware_array_len + count;
		total_firmware_len = total_firmware_len + count;

		f_curr_p = f_curr_p + count;

		/* reset the length when it reaches 32KB */
		if (firmware_array_len == FIRMWARE_PACKET_LEN)
			firmware_array_len = 0;

		/* Ready to upload the firmware */
		if (total_firmware_len == FIRMWARE_TOTAL_LEN) {
			printk("Firmware will be updated during next open of camera\n");
			mc7_firmware_upd = 1;
		}
	} else {
		printk("Error invalid window size\n");
		return -1;
	}
	return count;
}

static DEVICE_ATTR(mc7_firmware, S_IWUSR | S_IRUGO, mc7_firmware_show, mc7_firmware_store);

static int mc7_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	FUNC_ENTER();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "LSI-MC7: %s: mc7 i2c probe failed\n", __func__);
		return -1;
	}

	rc = sysfs_create_file(&client->dev.kobj, &dev_attr_mc7_firmware.attr);
	if (rc)
		printk(KERN_ERR "LSI-MC7: %s: sysfs create entry failed\n", __func__);

	mc7_i2c_client = client;
	MC7DBG("mc7 i2c probe successful\n");
	return 0;
}

static int mc7_i2c_remove(struct i2c_client *client)
{
	FUNC_ENTER();
	sysfs_remove_file(&client->dev.kobj, &dev_attr_mc7_firmware.attr);
	mc7_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id mc7_id[] = {
	{"lsi_mc7", 0},
	{ }
};

static struct i2c_driver mc7_i2c_driver = {
	.id_table	= mc7_id,
	.probe  	= mc7_i2c_probe,
	.remove 	= mc7_i2c_remove,
	.driver 	= {
		.name = "lsi_mc7",
	},
};

/* Define this to update MC7 firmware while the board is booting */
//#define DOWNLOAD_WHILE_BOOT
static int mc7_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	unsigned int val = 0;
#ifdef DOWNLOAD_WHILE_BOOT
	struct packet *i2c_pkt;
#endif

	FUNC_ENTER();
	rc = i2c_add_driver(&mc7_i2c_driver);
	if (rc < 0) {
		printk(KERN_ERR "LSI-MC7: %s: sensor probe failed\n", __func__);
		goto probe_fail0;
	}

	rc = i2c_add_driver(&lp8720_i2c_driver);
	if (rc < 0) {
		printk(KERN_ERR "LSI-MC7: %s: lp8720 probe failed\n", __func__);
		goto probe_fail1;
	}

	msm_camio_clk_rate_set(24000000);

	/* Check whether sensor is present and its powering on */
	mc7_sensor_power_on_off(1 ,info);
	rc = poll_mc7_int(5);
	if (!rc) {
		mc7_catparam_i2c_read(4, 0xf, 0x1c, &val);
		if (val != 0x68000000) {
			printk(KERN_ERR "LSI-MC7:%s: Invalid read value= %x\n", __func__, val);
			goto probe_fail2;
		}

#ifdef DOWNLOAD_WHILE_BOOT
		/* write PLL divider value */
		mc7_catparam_i2c_write(2, 0xf, 0x24, 0x001A);

		/* firmware is send first */
		for (i2c_pkt = &mc7_firmware_array[0]; i2c_pkt->length != 0; i2c_pkt++)
			firmware_write(i2c_pkt->length, i2c_pkt->firmware);

		/* Now send the flash writer program */
		for (i2c_pkt = &mc7_fl_write_array[0]; i2c_pkt->length != 0; i2c_pkt++)
			firmware_write(i2c_pkt->length, i2c_pkt->firmware);

		/* write the address of the program */
		mc7_catparam_i2c_write(4, 0xf, 0x0C, 0x6807E000);

		/* start the writer program */
		mc7_catparam_i2c_write(1, 0xf, 0x12, 0x01);

		rc = poll_mc7_int(300);
		if (!rc)
			printk(KERN_ERR "LSI-MC7: firmware successfully writen to serial flash\n");
		else
			printk(KERN_ERR "LSI-MC7: firmware update failed\n");
#endif

		/* Power off the sensor */
		mc7_sensor_power_on_off(0 ,info);

		s->s_init = mc7_sensor_open_init;
		s->s_release = mc7_sensor_release;
		s->s_config  = mc7_sensor_config;

		MC7DBG("mc7 sensor probe successful\n");
		return rc;
	}

probe_fail2:
	mc7_sensor_power_on_off(0 ,info);
probe_fail1:
	i2c_del_driver(&lp8720_i2c_driver);
probe_fail0:
	i2c_del_driver(&mc7_i2c_driver);
	printk(KERN_ERR "LSI-MC7: %s:probe failed\n", __func__);
	return rc;
}

static int __mc7_probe(struct platform_device *pdev)
{
	FUNC_ENTER();
	return msm_camera_drv_start(pdev, mc7_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mc7_probe,
	.driver = {
		.name = "msm_camera_mc7",
		.owner = THIS_MODULE,
	},
};

static int __init mc7_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mc7_init);
MODULE_DESCRIPTION("Fujitsu 5MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

