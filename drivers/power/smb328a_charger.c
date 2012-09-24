/*
 *  SMB328A-charger.c
 *  SMB328A charger interface driver
 *
 *  Copyright (C) 2011 Samsung Electronics
 *
 *  <jongmyeong.ko@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>
#include <linux/smb328a_charger.h>
#include <linux/i2c/fsa9480.h>

/* Register define */
#define SMB328A_INPUT_AND_CHARGE_CURRENTS	0x00
#define	SMB328A_CURRENT_TERMINATION			0x01
#define SMB328A_FLOAT_VOLTAGE				0x02
#define SMB328A_FUNCTION_CONTROL_A1			0x03
#define SMB328A_FUNCTION_CONTROL_A2			0x04
#define SMB328A_FUNCTION_CONTROL_B			0x05
#define SMB328A_OTG_PWR_AND_LDO_CONTROL		0x06
#define SMB328A_VARIOUS_CONTROL_FUNCTION_A	0x07
#define SMB328A_CELL_TEMPERATURE_MONITOR	0x08
#define SMB328A_INTERRUPT_SIGNAL_SELECTION	0x09
#define SMB328A_I2C_BUS_SLAVE_ADDRESS		0x0A

#define SMB328A_CLEAR_IRQ					0x30
#define SMB328A_COMMAND						0x31
#define SMB328A_INTERRUPT_STATUS_A			0x32
#define SMB328A_BATTERY_CHARGING_STATUS_A	0x33
#define SMB328A_INTERRUPT_STATUS_B			0x34
#define SMB328A_BATTERY_CHARGING_STATUS_B	0x35
#define SMB328A_BATTERY_CHARGING_STATUS_C	0x36
#define SMB328A_INTERRUPT_STATUS_C			0x37
#define SMB328A_BATTERY_CHARGING_STATUS_D	0x38
#define SMB328A_AUTOMATIC_INPUT_CURRENT_LIMMIT_STATUS	0x39

enum {
	BAT_NOT_DETECTED,
	BAT_DETECTED
};

enum {
	CHG_MODE_NONE,
	CHG_MODE_AC,
	CHG_MODE_USB,
	CHG_MODE_MISC
};

struct smb328a_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		psy_bat;
	struct smb328a_platform_data	*pdata;

	int chg_mode;
	unsigned int batt_vcell;
};

static enum power_supply_property smb328a_battery_props[] = {
#ifndef CONFIG_MACH_APACHE
	POWER_SUPPLY_PROP_STATUS,
#endif
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

// Check batt init
extern struct work_struct *p_batt_init;
extern int board_hw_revision;


static int smb328a_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

static int smb328a_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

#if 0
static void smb328a_print_reg(struct i2c_client *client, int reg)
{
	u8 data = 0;

	data = i2c_smbus_read_byte_data(client, reg);

	if (data < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, data);
	else
		printk("%s : reg (0x%x) = 0x%x\n", __func__, reg, data);
}

static void smb328a_print_all_regs(struct i2c_client *client)
{
	smb328a_print_reg(client, 0x31);
	smb328a_print_reg(client, 0x32);
	smb328a_print_reg(client, 0x33);
	smb328a_print_reg(client, 0x34);
	smb328a_print_reg(client, 0x35);
	smb328a_print_reg(client, 0x36);
	smb328a_print_reg(client, 0x37);
	smb328a_print_reg(client, 0x38);
	smb328a_print_reg(client, 0x39);
	smb328a_print_reg(client, 0x00);
	smb328a_print_reg(client, 0x01);
	smb328a_print_reg(client, 0x02);
	smb328a_print_reg(client, 0x03);
	smb328a_print_reg(client, 0x04);
	smb328a_print_reg(client, 0x05);
	smb328a_print_reg(client, 0x06);
	smb328a_print_reg(client, 0x07);
	smb328a_print_reg(client, 0x08);
	smb328a_print_reg(client, 0x09);
	smb328a_print_reg(client, 0x0a);
}
#endif

static void smb328a_allow_volatile_writes(struct i2c_client *client)
{
	int val;
	u8 data;

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if ((val >= 0) && !(val&0x80)) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		data |= (0x1 << 7);
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0)
			pr_err("%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_COMMAND);
		if (val >= 0) {
			data = (u8)data;
			pr_info("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		}
	}
}

static void smb328a_set_command_reg(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);
	int val;
	u8 data;

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		if (chip->chg_mode == CHG_MODE_AC)
//			data = 0xad;
			data = 0x8C;
		else
			data = 0x88; /* usb or misc or unknown */
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0)
			pr_err("%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_COMMAND);
		if (val >= 0) {
			data = (u8)data;
			pr_info("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		}
	}
}

static void smb328a_charger_function_conrol(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);
	int val;
	u8 data, set_data;

	smb328a_allow_volatile_writes(client);
	
	/* Clear IRQ register*/
	set_data = 0xAA;

	if (smb328a_write_reg(client, SMB328A_CLEAR_IRQ, set_data) < 0)
			pr_err("%s : write error!\n", __func__);
	else
		printk("%s : Clear IRQ register.\n", __func__);
	
	

	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
#if 0
		if (chip->chg_mode == CHG_MODE_AC) {
			set_data = 0x97;
		} else
			set_data = 0x17;
#endif
		set_data = 0x75;
		if (data != set_data) { /* this can be changed with top-off setting */
			data = set_data;
			if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_CURRENT_TERMINATION, data);
#if 0
		if (chip->chg_mode == CHG_MODE_AC) {
			set_data = 0x90;
		} else
			set_data = 0x10;
#endif
#ifdef CONFIG_MACH_APACHE
		set_data = 0x54;/* HW req : chg current 600mA -> 700 mA */
#else
		set_data = 0x34;
#endif
		if (data != set_data) { /* AICL enable */
			data = set_data;
			if (smb328a_write_reg(client, SMB328A_CURRENT_TERMINATION, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_CURRENT_TERMINATION, data);
			}
		}
	}


	val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FLOAT_VOLTAGE, data);
#ifdef CONFIG_MACH_ANCORA
        if (data != 0xCC) {
            data = 0xCC; /* 4.22V float voltage *//* hw requirements 'ejuni@samsung.com'*/
#else
		if (data != 0xCA) {
			data = 0xCA; /* 4.2V float voltage */
#endif
			if (smb328a_write_reg(client, SMB328A_FLOAT_VOLTAGE, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FLOAT_VOLTAGE, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A1);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A1, data);
		if (data != 0xDA) {
			data = 0xDA;
			if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_A1, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A1);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A1, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A2);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A2, data);
		if (data != 0x4F) {
			data = 0x4F;
			if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_A2, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A2);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A2, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
		if (data != 0x00) {
			data = 0x00;
			if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_B, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
#if defined (CONFIG_TARGET_LOCALE_USA)
		set_data = 0x4d;
#else
		set_data = 0xC5;
#endif			
		if (data != set_data) {
			data = set_data;
			if (smb328a_write_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
		if (data != 0xF6) { /* this can be changed with top-off setting */
			data = 0xF6;
			if (smb328a_write_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_CELL_TEMPERATURE_MONITOR, data);
		if (data != 0x00) {
			data = 0x00;
			if (smb328a_write_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_CELL_TEMPERATURE_MONITOR, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_INTERRUPT_SIGNAL_SELECTION, data);
		if (data != 0x00) {
			data = 0x20;
			if (smb328a_write_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION);
			if (val >= 0) {
				data = (u8)val;
				dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INTERRUPT_SIGNAL_SELECTION, data);
			}
		}
	}
}

static int smb328a_check_charging_status(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	int ret = -1;

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);

		ret = (data&(0x3<<1))>>1;
		dev_info(&client->dev, "%s : status = 0x%x\n", __func__, data);
	}

	return ret;
}

static bool smb328a_check_is_charging(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);

		if (data&0x1)
			ret = true; /* charger enabled */
	}

	return ret;
}

static bool smb328a_check_bat_full(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8)val;
		//printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);

		if (data&(0x1<<6))
			ret = true; /* full */
	}

	return ret;
}

/* vf check */
static bool smb328a_check_bat_missing(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_B);

//	printk("[SSAM] %s value : %d \n", __func__, val);

#if 0
	if(val > 0)
		ret = true; /* missing battery */
#endif

	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_B, data);

		if (data&0x1) {
			pr_info("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_B, data);
			ret = true; /* missing battery */
		}
	}

	return ret;
}

static bool smb328a_read_chg_status(struct i2c_client *client, unsigned int *status)
{
	int status_A=0,status_B=0,status_C=0, int_status_C=0;
	bool ret = false;

	status_A = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_A);
	status_B = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_B);
	status_C = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	int_status_C = smb328a_read_reg(client, SMB328A_INTERRUPT_STATUS_C);
	
	if( status_A < 0 || status_B < 0 || status_C < 0 || int_status_C < 0)
		return false;
	else
		*status = int_status_C << 24 | status_A << 16 | status_B << 8 | status_C ;

	//printk("%s : %x %x %x %x , %x\n", __func__, int_status_C, status_A, status_B, status_C, *status);

	return true;
}

/* whether valid dcin or not */
static bool smb328a_check_vdcin(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_A);
	if (val >= 0) {
		data = (u8)val;
		//printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_A, data);

		if (data&(0x1<<1))
			ret = true;
	}
	
	return ret;
}

static bool smb328a_check_bmd_disabled(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);

		if (data&(0x1<<7)) {
			ret = true;
			pr_info("%s : return ture : reg(0x%x)=0x%x (0x%x)\n", __func__,
				SMB328A_FUNCTION_CONTROL_B, data, data&(0x1<<7));
		}
	}

#if !defined (CONFIG_TARGET_LOCALE_USA)
	val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
	if (val >= 0) {
		data = (u8)val;
		//printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);

		if ((data&(0x1<<7))==0) {
			ret = true;
			pr_info("%s : return ture : reg(0x%x)=0x%x (0x%x)\n", __func__,
				SMB328A_OTG_PWR_AND_LDO_CONTROL, data, data&(0x1<<7));
		}
	}
#endif

	return ret;
}

static int smb328a_chg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct smb328a_chip *chip = container_of(psy,
				struct smb328a_chip, psy_bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
#ifdef CONFIG_MACH_APACHE
		if(smb328a_read_chg_status(chip->client , &val->intval)==false)
			return -EIO;
#else
		if (smb328a_check_vdcin(chip->client))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
#endif
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (smb328a_check_bat_missing(chip->client))
			val->intval = BAT_NOT_DETECTED;
		else
			val->intval = BAT_DETECTED;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		//printk("%s : check bmd available\n", __func__);
		//smb328a_print_all_regs(chip->client);
		/* check VF check available */
		if (smb328a_check_bmd_disabled(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
//		printk("smb328a_check_bmd_disabled is %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (smb328a_check_bat_full(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (smb328a_check_charging_status(chip->client)) {
			case 0:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
				break;
			case 1:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
				break;
			case 2:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
				break;
			case 3:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
				break;
			default:
				pr_err("%s : get charge type error!\n", __func__);
				return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (smb328a_check_is_charging(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int smb328a_set_top_off(struct i2c_client *client, int top_off)
{
	int val, set_val = 0;
	u8 data;

	printk("%s : \n", __func__);

	smb328a_allow_volatile_writes(client);
	
	set_val = top_off/25;
	set_val -= 1;

	if (set_val < 0 || set_val > 7) {
		pr_err("%s: invalid topoff set value(%d)\n", __func__, set_val);
		return -EINVAL;
	}

	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
		data |= (set_val << 0);
		if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
	}

	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
		data |= (set_val << 5);
		if (smb328a_write_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
	}
	
	return 0;
}

static int smb328a_set_charging_current(struct i2c_client *client, int chg_current)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s : \n", __func__);

	if (chg_current < 200 || chg_current > 950)
		return -EINVAL;

	if (chg_current == 600) {
		chip->chg_mode = CHG_MODE_AC;
	} else if (chg_current == 450) {
		chip->chg_mode = CHG_MODE_USB;
	} else {
		pr_err("%s : error! invalid setting current\n", __func__);
		chip->chg_mode = CHG_MODE_NONE;
		return -1;
	}
	
	return 0;
}

static int smb328a_enable_otg(struct i2c_client *client)
{
	int val;
	u8 data;

	dev_info(&client->dev, "%s : \n", __func__);
	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		if (data != 0x9A)
		{
			dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
			data = 0x9A;
			if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
				pr_err("%s : error!\n", __func__);
				return -1;
			}
			msleep(100);
		
			data = smb328a_read_reg(client, SMB328A_COMMAND);
			dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		}
	}
	return 0;
}

static int smb328a_disable_otg(struct i2c_client *client)
{
	int val;
	u8 data;

	dev_info(&client->dev, "%s : \n", __func__);
	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
		data = 0x80;
		if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_B, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}
		msleep(100);
		data = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
						
	}
	
	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		data = 0x98;
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}		
		msleep(100);
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		//fsa9480_otg_detach();
	}
	return 0;
}

static void smb328a_ldo_disable(struct i2c_client *client)
{
	int val;
	u8 data;

	dev_info(&client->dev, "%s : \n", __func__);
	
	smb328a_allow_volatile_writes(client);

	val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);

		data |= (0x1 << 5);
		if (smb328a_write_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL, data) < 0)
			pr_err("%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
		if (val >= 0) {
			data = (u8)val;
			dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
		}
	}
}

static int smb328a_chgen_bit_control(struct i2c_client *client, bool enable)
{
	int val;
	u8 data;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		if (enable)
			data &= ~(0x1 << 4); /* "0" turn off the charger */
		else
			data |= (0x1 << 4); /* "1" turn off the charger */
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		pr_info("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
	}

	return 0;
}

static int smb328a_enable_charging(struct i2c_client *client)
{
	int val;
	u8 data;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		if (chip->chg_mode == CHG_MODE_AC)
			data = 0x8C;
		else if (chip->chg_mode == CHG_MODE_USB)
			data = 0x88;

		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		pr_info("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
	}

	return 0;
}

static int smb328a_disable_charging(struct i2c_client *client)
{
	int val;
	u8 data;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s : \n", __func__);

	smb328a_allow_volatile_writes(client);
	
	/* Write register for charging termination */
	
	data = 0x75;
	
	if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
	}

#ifdef CONFIG_MACH_APACHE
	data = 0x54;/* HW req : chg current 600mA -> 700 mA */
#else
	data = 0x34;
#endif

	if (smb328a_write_reg(client, SMB328A_CURRENT_TERMINATION, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_CURRENT_TERMINATION, data);
	}
	
#ifdef CONFIG_MACH_ANCORA
	data = 0xCC; /* 4.22V float voltage *//* hw requirements 'ejuni@samsung.com'*/
#else
	data = 0xCA; /* 4.2V float voltage */
#endif

	if (smb328a_write_reg(client, SMB328A_FLOAT_VOLTAGE, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FLOAT_VOLTAGE, data);
	}

	data = 0xDA;

	if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_A1, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A1);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A1, data);
	}

	data = 0x4D;

	if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_A2, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A2);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A2, data);
	}

	data = 0x00;

	if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_B, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
	}

#if defined (CONFIG_TARGET_LOCALE_USA)
	data = 0x4d;
#else
	data = 0xC5;
#endif

	if (smb328a_write_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
	}

	data = 0xF6;

	if (smb328a_write_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
	}

	data = 0x00;

	if (smb328a_write_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_CELL_TEMPERATURE_MONITOR, data);
	}

	data = 0x20;

	if (smb328a_write_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION, data) < 0)
			pr_err("%s : error!\n", __func__);
	val = smb328a_read_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INTERRUPT_SIGNAL_SELECTION, data);
	}

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		data = 0x98;
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			pr_err("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		pr_info("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
	}

	return 0;
}

static int smb328a_chg_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct smb328a_chip *chip = container_of(psy,
				struct smb328a_chip, psy_bat);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW: /* step1) Set charging current */
		//smb328a_set_command_reg(chip->client);
		smb328a_charger_function_conrol(chip->client);
		ret = smb328a_set_charging_current(chip->client, val->intval);
		//smb328a_print_all_regs(chip->client);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: /* step2) Set top-off current */
		if (val->intval < 25 || val->intval > 200) {
			pr_err("%s: invalid topoff current(%d)\n",
					__func__, val->intval);
			return -EINVAL;
		}
		ret = smb328a_set_top_off(chip->client, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: /* step3) Notify Vcell Now */
		chip->batt_vcell = val->intval;
		pr_info("%s : vcell(%d)\n", __func__, chip->batt_vcell);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_STATUS: /* step4) Enable/Disable charging */
		if (val->intval == POWER_SUPPLY_STATUS_CHARGING) {
#if 0
			if (chip->chg_mode != CHG_MODE_USB)
				smb328a_ldo_disable(chip->client);
#endif
			ret = smb328a_enable_charging(chip->client);
#if 0
			if (chip->batt_vcell > 3900) {
				smb328a_chgen_bit_control(chip->client, false);
				smb328a_chgen_bit_control(chip->client, true);
			}
#endif
		} else

			ret = smb328a_disable_charging(chip->client);
		//smb328a_print_all_regs(chip->client);
		break;
#if 0
	case POWER_SUPPLY_PROP_OTG:	
		if (val->intval == POWER_SUPPLY_CAPACITY_OTG_ENABLE)
		{
			smb328a_charger_function_conrol(chip->client);		
			ret = smb328a_enable_otg(chip->client);
		}
		else
			ret = smb328a_disable_otg(chip->client);
		break;
#endif
	default:
		return -EINVAL;
	}
	return ret;
}

static ssize_t sec_smb328a_show_property(struct device *dev,
				    struct device_attribute *attr, char *buf);

#define SEC_SMB328A_ATTR(_name)			\
{						\
	.attr = { .name = #_name,		\
		  .mode = S_IRUGO | S_IWUSR |S_IWGRP,			\
		  .owner = THIS_MODULE },	\
	.show = sec_smb328a_show_property,		\
	.store = NULL,			\
}

static struct device_attribute sec_smb328a_attrs[] = {
	SEC_SMB328A_ATTR(smb_read_36h),
};

enum {
	SMB_READ_36H = 0,
};

static ssize_t sec_smb328a_show_property(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct smb328a_chip *chip = container_of(psy,
						  struct smb328a_chip,
						  psy_bat);

	int i = 0;
	const ptrdiff_t off = attr - sec_smb328a_attrs;
	int val;
	u8 data = 0;

	switch (off) {
	case SMB_READ_36H:
		val = smb328a_read_reg(chip->client, SMB328A_BATTERY_CHARGING_STATUS_C);
		if (val >= 0) {
			data = (u8)val;
			//printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);
			i += scnprintf(buf + i, PAGE_SIZE - i, "0x%x (bit6 : %d)\n",
					data, (data&0x40)>>6);
		} else {
			i = -EINVAL;
		}
		break;
	default:
		i = -EINVAL;
	}

	return i;
}

static int smb328a_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sec_smb328a_attrs); i++) {
		rc = device_create_file(dev, &sec_smb328a_attrs[i]);
		if (rc)
			goto smb328a_attrs_failed;
	}
	goto succeed;

smb328a_attrs_failed:
	while (i--)
		device_remove_file(dev, &sec_smb328a_attrs[i]);
succeed:
	return rc;
}

static int __devinit smb328a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb328a_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	pr_info("%s: SMB328A driver Loading! \n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

//	chip->pdata->hw_init(); /* important */
	
	chip->psy_bat.name = "smb328a-charger",
	chip->psy_bat.type = POWER_SUPPLY_TYPE_BATTERY,
	chip->psy_bat.properties = smb328a_battery_props,
	chip->psy_bat.num_properties = ARRAY_SIZE(smb328a_battery_props),
	chip->psy_bat.get_property = smb328a_chg_get_property,
	chip->psy_bat.set_property = smb328a_chg_set_property,
	ret = power_supply_register(&client->dev, &chip->psy_bat);
	if (ret) {
		pr_err("Failed to register power supply psy_bat\n");
		goto err_kfree;
	}

	chip->chg_mode = CHG_MODE_NONE;
	//smb328a_charger_function_conrol(client);
	//smb328a_print_all_regs(client);

//	printk("[SSAM] smb328a syfs create!!! \n");

	/* create smb328a attributes */
	smb328a_create_attrs(chip->psy_bat.dev);

	if(board_hw_revision >= CONFIG_HW_REV_USING_SMB328)
	{
		/* Enable batt init */
		if(p_batt_init != NULL)
		{
	//		printk("[SSAM] Run workqueue.\n");
			if (work_pending(p_batt_init))
			{
	//			printk("[SSAM] check pending workqueue.\n");
				cancel_delayed_work(p_batt_init);
				schedule_work(p_batt_init);
			}
		}
	}		
	return 0;

err_kfree:
	kfree(chip);
	return ret;
}

static int __devexit smb328a_remove(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy_bat);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int smb328a_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	return 0;
}

static int smb328a_resume(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	return 0;
}
#else
#define smb328a_suspend NULL
#define smb328a_resume NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id smb328a_id[] = {
	{ "smb328a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb328a_id);

static struct i2c_driver smb328a_i2c_driver = {
	.driver	= {
		.name	= "smb328a",
	},
	.probe		= smb328a_probe,
	.remove		= __devexit_p(smb328a_remove),
	.suspend	= smb328a_suspend,
	.resume		= smb328a_resume,
	.id_table	= smb328a_id,
};

static int __init smb328a_init(void)
{
	return i2c_add_driver(&smb328a_i2c_driver);
}
module_init(smb328a_init);

static void __exit smb328a_exit(void)
{
	i2c_del_driver(&smb328a_i2c_driver);
}
module_exit(smb328a_exit);


MODULE_DESCRIPTION("SMB328A charger control driver");
MODULE_AUTHOR("<jongmyeong.ko@samsung.com>");
MODULE_LICENSE("GPL");
