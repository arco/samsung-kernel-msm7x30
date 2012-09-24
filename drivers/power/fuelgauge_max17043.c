/*
 *
 * Copyright (C) 2009 SAMSUNG ELECTRONICS.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Slave address */
#define MAX17040_SLAVE_ADDR	0x6D

/* Register address */
#define VCELL0_REG			0x02
#define VCELL1_REG			0x03
#define SOC0_REG			0x04
#define SOC1_REG			0x05
#define MODE0_REG			0x06
#define MODE1_REG			0x07
#define RCOMP0_REG			0x0C
#define RCOMP1_REG			0x0D
#define CMD0_REG			0xFE
#define CMD1_REG			0xFF

#include <linux/slab.h>

extern bool power_down;

static struct i2c_driver fg_i2c_driver;
struct fg_i2c_chip {
	struct i2c_client		*client;
};
static struct fg_i2c_chip *fg_max17043 = NULL;

static int is_reset_soc = 0;
static int is_attached = 0;

#ifdef MAX17043_FUEL_GAUGE
static int is_alert = 0;	// ALARM_INT
#endif

static int fg_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[1];
	struct i2c_msg msg[2];


	buf[0] = reg; 

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) 
		return -EIO;

	*data = buf[0];
	
	return 0;
}

static int fg_i2c_write(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[3];
	struct i2c_msg msg[1];


	buf[0] = reg;
	buf[1] = *data;
	buf[2] = *(data + 1);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 3;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) 
		return -EIO;

	return 0;
}

// hyeokseon.yu
unsigned int fg_read_vcell(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	u32 vcell = 0;


	if (!client)
		return -1;

	if (fg_i2c_read(client, VCELL0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read VCELL0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, VCELL1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read VCELL1\n", __func__);
		return -1;
	}

#if defined(CONFIG_BATT_MICROVOLT_UNIT)
	vcell = (((data[0] << 4) & 0xFF0) | ((data[1] >> 4) & 0xF)) * 1250;
#else
	vcell = ((((data[0] << 4) & 0xFF0) | ((data[1] >> 4) & 0xF)) * 125)/100;
#endif

	if(!power_down)
		pr_debug("%s: VCELL=%d\n", __func__, vcell);

	return vcell;
}

// hyeokseon.yu
unsigned int fg_read_soc(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	int FGPureSOC = 0;
	int FGAdjustSOC = 0;
	int FGSOC = 0;

	if (!client)
		return -1;

	if (fg_i2c_read(client, SOC0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read SOC0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, SOC1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read SOC1\n", __func__);
		return -1;
	}

	if (is_reset_soc) {
		pr_info("%s: Reseting SOC\n", __func__);
		return -1;
	}

	/*Adjusted SOC(Ancora)
	**RCOMP : B0h, FULL : 96.7, EMPTY : 0.0
	**Adjusted SOC(Aries VE)
	**RCOMP : D0h, FULL : 97.7, EMPTY : 0.4
	**Adj_soc = (SOC%-EMPTY)/(FULL-EMPTY)*100
	*/

	// calculating soc
	FGPureSOC = data[0]*100+((data[1]*100)/256);

	// hsil for get Adjusted SOC%
	if(FGPureSOC >= 0)
#ifdef CONFIG_MACH_ARIESVE
		FGAdjustSOC = ((FGPureSOC*10000)-40)/9730;
#else
		FGAdjustSOC = ((FGPureSOC*10000)-0)/9670;
#endif
	else
		FGAdjustSOC = 0;

	// rounding off and Changing to percentage.
	FGSOC=FGAdjustSOC/100;

#ifdef CONFIG_MACH_ARIESVE
	if(FGAdjustSOC%100 >= 50)
#else
	if(FGAdjustSOC%100 >= 50 && FGSOC > 1)
#endif
		FGSOC+=1;

	if(FGSOC>=100)
		FGSOC=100;

	if (FGSOC < 0)
		FGSOC = 0;

	if(!power_down)
		printk("[MAX17043] FGPureSOC = %d (%d.%d)\tFGAdjustSOC = %d\tFGSOC = %d\n", FGPureSOC, data[0], (data[1]*100)/256, FGAdjustSOC, FGSOC); 


	return FGSOC;
}

// hyeokseon.yu
unsigned int fg_read_raw_vcell(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	u32 vcell = 0;

	if (!client)
		return -1;

	if (fg_i2c_read(client, VCELL0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read VCELL0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, VCELL1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read VCELL1\n", __func__);
		return -1;
	}

	vcell = data[0] << 8 | data[1];
	vcell = (vcell >> 4) * 125 * 1000;

	printk("SEXYKYU %s: VCELL=%d\n", __func__, vcell);

	return vcell;
}

unsigned int fg_read_raw_soc(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];

	if (!client)
		return -1;

	if (fg_i2c_read(client, SOC0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read SOC0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, SOC1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read SOC1\n", __func__);
		return -1;
	}

	if (data[0] < 0)	data[0] = 0;
	if (data[0] > 100)	data[0] = 100;

	pr_debug("%s: SOC [0]=%d [1]=%d\n", __func__, data[0], data[1]);

	if (is_reset_soc) {
		pr_info("%s: Reseting SOC\n", __func__);
		return -1;
	} else {
		return data[0];
	}
}

// hyeokseon.yu
unsigned int fg_reset_soc(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 rst_cmd[2];
	s32 ret = 0;


	if (!client)
		return -1;

	is_reset_soc = 1;
	/* Quick-start */
	rst_cmd[0] = 0x40;
	rst_cmd[1] = 0x00;

	ret = fg_i2c_write(client, MODE0_REG, rst_cmd);
	if (ret)
		pr_err("[BATT] %s: failed reset SOC(%d)\n", __func__, ret);

	msleep(500);
	is_reset_soc = 0;
	return ret;
}

// hyeokseon.yu 
void fuel_gauge_rcomp(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 rst_cmd[2];
	s32 ret = 0;


	if (!client)
		return ;
#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_APACHE)
	rst_cmd[0] = 0xC0; /* MAXIM recommend */
#else
	rst_cmd[0] = 0xB0;
#endif

#ifdef MAX17043_FUEL_GAUGE
	rst_cmd[1] = 0x1F;
#else
	rst_cmd[1] = 0x00;
#endif

	ret = fg_i2c_write(client, RCOMP0_REG, rst_cmd);
	if (ret)
		pr_err("[BATT] %s: failed fuel_gauge_rcomp(%d)\n", __func__, ret);

	//msleep(500);
}

#ifdef MAX17043_FUEL_GAUGE
int (*fg_alert_handler)(int) = NULL;
static irqreturn_t fg_interrupt_handler(int irq, void *data)	// ALARM_INT
{
	struct i2c_client *client = fg_max17043->client;
	u8 rst_cmd[2];
	int ret = 0;
	
	if (!client)
		return IRQ_HANDLED;

	rst_cmd[0] = 0x00;
	rst_cmd[1] = 0x00;

	if (fg_i2c_read(client, RCOMP0_REG, &rst_cmd[0]) < 0)
	{
		pr_err("[BATT] %s failed!\n", __func__);
		return IRQ_HANDLED;
	}
	if (fg_i2c_read(client, RCOMP1_REG, &rst_cmd[1]) < 0)
	{
		pr_err("[BATT] %s failed!\n", __func__);
		return IRQ_HANDLED;
	}

#ifdef DEBUG
	pr_info("\n-----------------------------------------------------\n");
	pr_info(" << %s (vcell:%d, soc:%d, rcomp:0x%x,0x%x) >> \n", __func__, fg_read_vcell(), fg_read_soc(), rst_cmd[0], rst_cmd[1]);
	pr_info("-----------------------------------------------------\n\n");
#endif

	if(fg_alert_handler) {
		ret = (*fg_alert_handler)(1);
	
		if (ret)
			pr_info("[BATT]: %s: low battery alert, ready to power down (0x%x, 0x%x)\n", __func__, rst_cmd[0], rst_cmd[1]);
		else
			// ignore alert
			pr_info("[BATT] %s: Ignore low battery alert during charging... \n", __func__);
	}

	// Clear ALRT bit to prevent another low battery interrupt...
	rst_cmd[1] = rst_cmd[1] & 0xDF;

#ifdef DEBUG
	pr_info("[FG] %s: clear the bit = 0x%x, 0x%x \n", __func__, rst_cmd[0], rst_cmd[1]);
#endif

	if (fg_i2c_write(client, RCOMP0_REG, rst_cmd))
		pr_err("[BATT] %s: failed write rcomp\n", __func__);

	return IRQ_HANDLED;
}
#endif

static int __devinit fg_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);


	if (fg_max17043 == NULL ) {
		fg_max17043 = kzalloc(sizeof(struct fg_i2c_chip), GFP_KERNEL);
		if (fg_max17043 == NULL ) {
			return -ENOMEM;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	fg_max17043->client = client;

	i2c_set_clientdata(client, fg_max17043);

	fuel_gauge_rcomp();

#ifdef MAX17043_FUEL_GAUGE
	if (request_threaded_irq(client->irq, NULL,
		fg_interrupt_handler,
		IRQF_ONESHOT|IRQF_TRIGGER_FALLING, "ALARM_INT", client))
	{
		free_irq(client->irq, NULL);
		pr_err("[BATT] fg_interrupt_handler can't register the handler! and passing....\n");
	}
#endif

	is_attached = 1;

	pr_debug("[BATT] %s : success!\n", __func__);
	return 0;
}

static int __devexit fg_i2c_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);


	pr_info("[BATT] %s\n", __func__);

	i2c_set_clientdata(client, NULL);
	kfree(chip);
	fg_max17043->client = NULL;
	return 0;
}

#define fg_i2c_suspend NULL
#define fg_i2c_resume NULL


static const struct i2c_device_id fg_i2c_id[] = {
	{ "fuelgauge_max17043", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver fg_i2c_driver = {
	.driver	= {
		.name	= "fuelgauge_max17043",
	},
	.probe		= fg_i2c_probe,
	.remove		= __devexit_p(fg_i2c_remove),
	.suspend	= fg_i2c_suspend,
	.resume		= fg_i2c_resume,
	.id_table	= fg_i2c_id,
};

