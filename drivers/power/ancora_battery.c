/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */

// #define DEBUG 1
#include <linux/device.h>

#define MAX17043_FUEL_GAUGE		// Support low battery alert

// For LPM mode
extern int charging_boot;

// Check hw revision
extern int board_hw_revision;

// For SMB328A charger IC
struct work_struct *p_batt_init;


/* ***** Test Features ***** */

//#define __BATT_TEST_DEVICE__
//#define __AUTO_TEMP_TEST__ 
//#define __FULL_CHARGE_TEST__


#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/wakelock.h>

#ifdef CONFIG_WIRELESS_CHARGING
#define IRQ_WC_DETECT PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, (PM8058_GPIO(35)))
#define GPIO_WC_DETECT PM8058_GPIO_PM_TO_SYS(PM8058_GPIO(35))
#endif

static struct wake_lock vbus_wake_lock;

extern bool power_down;

#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_info
#endif

#ifdef CONFIG_MAX17043_FUEL_GAUGE
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/time.h>
#include "fuelgauge_max17043.c"
#endif

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1     0x00040001
#define BATTERY_RPC_VER_5_1     0x00050001

#define BATTERY_RPC_CB_PROG	(BATTERY_RPC_PROG | 0x01000000)

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VER_1_1		0x00010001
#define CHG_RPC_VER_1_3		0x00010003
#define CHG_RPC_VER_2_2		0x00020002
#define CHG_RPC_VER_3_1         0x00030001
#define CHG_RPC_VER_4_1         0x00040001

#define BATTERY_REGISTER_PROC                          	2
#define BATTERY_MODIFY_CLIENT_PROC                     	4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_READ_MV_PROC 				12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC 		14

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC 		1
#define BATTERY_CB_ID_ALL_ACTIV       	1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW            	3400	//2800
#define BATTERY_HIGH           	4200	//4300

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC 	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC 	0xffffffff

#define BATT_RPC_TIMEOUT    5000	/* 5 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))

//#define USE_SMB_VF_CHECK
#define BATTERY_STATUS__INVALID 4

/*******************************/
/* Charging control settings */

extern unsigned int boot_check_wrong_battery;

/* charging disable when OVP situation */
#define BATTERY_CHECK_OVP

typedef enum {
	STOP_CHARGING,
	START_CHARGING
} chg_enable_type;


const int temp_table[][2] =  {
	/* ADC, Temperature (C) */
	{ 1980,		-200},
	{ 1914,		-150},
	{ 1845,		-100},
	{ 1760,		-50	},
	{ 1738,		-40	},
	{ 1718, 	-30	},
	{ 1696, 	-20	},
	{ 1682, 	-10	},
	{ 1658,		0	},		
	{ 1637,		10	},
	{ 1590, 	30 },
	{ 1542, 	50	},
	{ 1483, 	70	},		
	{ 1424, 	100	},
	{ 1364, 	130 },		
	{ 1303, 	150 },
	{ 1235, 	170 },		
	{ 1167, 	200 },
	{ 1100, 	230 },
	{ 1034, 	250 },
	{  973, 	270 },		
	{  911, 	300 },
	{  855, 	330 },		
	{  800, 	350 },
	{  741, 	370 },		
	{  689, 	400 },
	{  654, 	420 },
	{  632, 	430 },		
	{  615, 	440 },				
	{  601, 	450 },		
	{  517, 	500 },
	{  468, 	550 },		
	{  390, 	600 },
	{  352, 	640 },
	{  345, 	650 },
	{  333, 	660 },
};

int batt_temp_adc_info = -1;

#define AVERAGE_COUNT		10

#define TIME_UNIT_SECOND	(HZ)
#define TIME_UNIT_MINUTE	(60*HZ)
#define TIME_UNIT_HOUR		(60*60*HZ)

#ifdef __FULL_CHARGE_TEST__
#define TOTAL_CHARGING_TIME			(1 * TIME_UNIT_MINUTE)
#define TOTAL_RECHARGING_TIME		(1 * TIME_UNIT_MINUTE)
#else
#define TOTAL_CHARGING_TIME			(6 * TIME_UNIT_HOUR)
#define TOTAL_RECHARGING_TIME		(90 * TIME_UNIT_MINUTE)
#endif
#define TOTAL_WATING_TIME				(20 * TIME_UNIT_SECOND)	// wait for full-charging and recharging

#define TEMP_TABLE_OFFSET		30
#define BATT_TEMP_HIGH_BLOCK		348		//	65`C   +- 2
#define BATT_TEMP_HIGH_RECOVER		623		//	43` C  +- 2
#define BATT_TEMP_LOW_BLOCK			1708	// 	-3`C   +- 2
#define BATT_TEMP_LOW_RECOVER		1670	//	0`C    +- 2
 
#define BATT_FULL_CHARGING_VOLTAGE	4190
#define BATT_FULL_CHARGING_CURRENT	180
#define BATT_FULL_CHARGING_CURRENT_REV_5	360

#define BATT_RECHARGING_VOLTAGE_1	4140
#define BATT_RECHARGING_VOLTAGE_2	4000

#ifdef __BATT_TEST_DEVICE__
static int temp_test_adc = 0;
#endif



enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
    /* The charger is good      */
    CHARGER_STATUS_GOOD,
    /* The charger is bad       */
    CHARGER_STATUS_BAD,
    /* The charger is weak      */
    CHARGER_STATUS_WEAK,
    /* Invalid charger status.  */
    CHARGER_STATUS_INVALID
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
    /* The charger is removed                 */
    CHARGER_TYPE_NONE,
    /* The charger is a regular wall charger   */
    CHARGER_TYPE_WALL,
    /* The charger is a PC USB                 */
    CHARGER_TYPE_USB_PC,
    /* The charger is a wall USB charger       */
    CHARGER_TYPE_USB_WALL,
    /* The charger is a USB carkit             */
    CHARGER_TYPE_USB_CARKIT,
    /* Invalid charger hardware status.        */
    CHARGER_TYPE_INVALID
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
    /* The battery is good        */
    BATTERY_STATUS_GOOD,
    /* The battery is cold/hot    */
    BATTERY_STATUS_BAD_TEMP,
    /* The battery is bad         */
    BATTERY_STATUS_BAD,
	/* The battery is removed     */
	BATTERY_STATUS_REMOVED,		/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
    /* Invalid battery status.    */
    BATTERY_STATUS_INVALID
};

/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
    /* The battery voltage is dead/very low (less than 3.2V)        */
    BATTERY_LEVEL_DEAD,
    /* The battery voltage is weak/low (between 3.2V and 3.4V)      */
    BATTERY_LEVEL_WEAK,
    /* The battery voltage is good/normal(between 3.4V and 4.2V)  */
    BATTERY_LEVEL_GOOD,
    /* The battery voltage is up to full (close to 4.2V)            */
    BATTERY_LEVEL_FULL,
    /* Invalid battery voltage level.                               */
    BATTERY_LEVEL_INVALID
};

//fatory jig check 2010.08.06 Huh Won
int batt_jig_on_status=0;
EXPORT_SYMBOL(batt_jig_on_status);

/* sys fs */
struct class *jig_class;
EXPORT_SYMBOL(jig_class);
struct device *jig_dev;
EXPORT_SYMBOL(jig_dev);
 
static ssize_t jig_show(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(jig , S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, jig_show, NULL);

static ssize_t jig_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", batt_jig_on_status );
}
/* sys fs */



struct rpc_reply_batt_chg_v1 {
	struct rpc_reply_hdr hdr;
	u32 	more_data;

	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32	battery_voltage;
	u32	battery_temp;
	u32	chg_current;
#ifdef CONFIG_WIRELESS_CHARGING
	u32	wc_adc;
#endif
};

struct rpc_reply_batt_chg_v2 {
	struct rpc_reply_batt_chg_v1	v1;

	u32	is_charger_valid;
	u32	is_charging;
	u32	is_battery_valid;
	u32	ui_event;
};

union rpc_reply_batt_chg {
	struct rpc_reply_batt_chg_v1	v1;
	struct rpc_reply_batt_chg_v2	v2;
};

static union rpc_reply_batt_chg rep_batt_chg;

struct msm_battery_info {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;
	u32 batt_api_version;

	u32 avail_chg_sources;
	u32 current_chg_source;	// NC (charging_source)

	u32 batt_status;
	u32 batt_health;
	u32 charger_valid;		// NC
	u32 batt_valid;
	u32 batt_capacity; 		/* in percentage */

	u32 charger_status;		// NC
	u32 charger_type;
	u32 battery_status;		// NC
	u32 battery_level;		// NC (batt_capacity)
	u32 battery_voltage;
	u32 battery_voltage_adc; // Volatage ADC

	u32 fg_soc;				// NC
	u32 batt_vol;			// NC (battery_voltage)
	u32 batt_temp_check;
	u32 batt_full_check;
	u32 charging_source;
	
	u32 battery_temp;		/* in celsius */
	u32 battery_temp_adc;	/* ADC code from CP */
	u32 chg_current_adc;	// ICHG ADC code (charging current)
	u32 batt_recharging;

	u32 batt_wireless;		// CONFIG_WIRELESS_CHARGING
	u32 wc_adc;				// CONFIG_WIRELESS_CHARGING

	u32 chargingblock_clear;
	u32 batt_voltage_now;	// for low batt

	u32(*calculate_capacity) (u32 voltage);	// NC

	s32 batt_handle;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;
	struct power_supply *current_ps;		// NC

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	struct workqueue_struct *msm_batt_wq;
	struct timer_list timer;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;			// NC

	struct early_suspend early_suspend;

#ifdef BATTERY_CHECK_OVP
    u32 batt_ovp;
    u32 batt_ovp_chg_block;
#endif
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_type = CHARGER_TYPE_NONE,
	.battery_voltage = BATTERY_HIGH,
	.batt_capacity = 100,
	.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid  = 1,
	.battery_temp = 230,	// 23.0`C
#ifdef CONFIG_WIRELESS_CHARGING
	.batt_wireless = 0,
	.wc_adc = 0,
#endif
#ifdef BATTERY_CHECK_OVP
    .batt_ovp = 0,
    .batt_ovp_chg_block = 0,
#endif
};

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

#define BATT_CHECK_INTERVAL	(5 * TIME_UNIT_SECOND) // every 5 sec

static unsigned int charging_start_time = 0;

static int msm_batt_driver_init = 0;
static int msm_batt_unhandled_interrupt = 0;

//Temp for USB OTG charging problem
enum chg_type {
	USB_CHG_TYPE__SDP,
	USB_CHG_TYPE__CARKIT,
	USB_CHG_TYPE__WALLCHARGER,
	USB_CHG_TYPE__INVALID
};

extern void hsusb_chg_connected_ext(enum chg_type chgtype);
extern void hsusb_chg_vbus_draw_ext(unsigned mA);
extern int fsa9480_get_charger_status(void);
extern int fsa9480_get_jig_status(void);

#ifdef CONFIG_MAX17043_FUEL_GAUGE
static u32 get_voltage_from_fuelgauge(void);
static u32 get_level_from_fuelgauge(void);
#endif

//------------------------------

int batt_restart(void);

//------------------------------
static ssize_t msm_batt_show_property(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);
static ssize_t msm_batt_store_property(struct device *dev, 
				       struct device_attribute *attr,
				       const char *buf, size_t count);

static int msm_batt_average_chg_current(int chg_current_adc);

//hyeokseon.yu
static void msm_batt_delay_init(struct work_struct *work);
static DECLARE_DELAYED_WORK(msm_batt_work_init, msm_batt_delay_init);

static int msm_batt_cleanup(void);
static int msm_batt_init_rpc(void);


static void msm_batt_check_event(struct work_struct *work);
static void msm_batt_cable_status_update(void);


/* charging absolute time control */
static void msm_batt_set_charging_start_time(chg_enable_type enable);
static int msm_batt_is_over_abs_time(void);

static void msm_batt_update_psy_status(void);
static DECLARE_WORK(msm_batt_work, msm_batt_check_event);

static void msm_batt_delay_init(struct work_struct *work)
{	
	int rc;

//	printk("[SSAM] %s enter!\n", __func__);

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);
	
}

static void batt_timeover(unsigned long arg )
{
	queue_work(msm_batt_info.msm_batt_wq, &msm_batt_work);
	mod_timer(&msm_batt_info.timer, (jiffies + BATT_CHECK_INTERVAL));
}

static void msm_batt_check_event(struct work_struct *work)
{
	msm_batt_update_psy_status();
}

#define MSM_BATTERY_ATTR(_name)		\
{			\
	.attr = { .name = #_name, .mode = 0444 },	\
	.show = msm_batt_show_property,			\
	.store = msm_batt_store_property,		\
}

static struct device_attribute ancora_battery_attrs[] = {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	MSM_BATTERY_ATTR(fg_soc),
	MSM_BATTERY_ATTR(reset_soc),
#endif
	MSM_BATTERY_ATTR(batt_temp_check),
	MSM_BATTERY_ATTR(charging_source),
	MSM_BATTERY_ATTR(batt_chg_current),	// ICHG ADC code (charging current)
#ifdef __BATT_TEST_DEVICE__
	MSM_BATTERY_ATTR(batt_temp_test_adc),
#endif
#ifdef CONFIG_WIRELESS_CHARGING
	MSM_BATTERY_ATTR(wc_status),
	MSM_BATTERY_ATTR(wc_adc),
#endif
	MSM_BATTERY_ATTR(chargingblock_clear),
#ifdef BATTERY_CHECK_OVP
    MSM_BATTERY_ATTR(batt_check_ovp),
    MSM_BATTERY_ATTR(batt_check_ovp_chg_block),
#endif
};

enum {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	FG_SOC,
	RESET_SOC,
#endif
	BATT_TEMP_CHECK,
	CHARGING_SOURCE,
	BATT_CHG_CURRENT,
#ifdef __BATT_TEST_DEVICE__
	BATT_TEMP_TEST_ADC,
#endif
#ifdef CONFIG_WIRELESS_CHARGING
	WC_STATUS,
	WC_ADC,
#endif
	CHARGINGBLOCK_CLEAR,
#ifdef BATTERY_CHECK_OVP
    BATT_OVP_STATUS,
    BATT_OVP_CHG_STATUS,
#endif
};

static int msm_batt_create_attrs(struct device * dev)
{
	int i, rc;


	for (i = 0; i < ARRAY_SIZE(ancora_battery_attrs); i++)
	{
		rc = device_create_file(dev, &ancora_battery_attrs[i]);
		if (rc)
			goto failed;
	}
	goto succeed;

failed:
	while (i--)
		device_remove_file(dev, &ancora_battery_attrs[i]);

succeed:
	return rc;
}

static void msm_batt_remove_attrs(struct device * dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ancora_battery_attrs); i++)
	{
		device_remove_file(dev, &ancora_battery_attrs[i]);
	}
}

#if 1
static ssize_t msm_batt_show_property(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int i = 0;
	const ptrdiff_t offset = attr - ancora_battery_attrs;

	switch (offset) {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
		case FG_SOC:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				get_level_from_fuelgauge());
			break;
		case RESET_SOC :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				fg_reset_soc());
			break;				
#endif
		case BATT_TEMP_CHECK:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				msm_batt_info.batt_temp_check);
			break;
		case CHARGING_SOURCE:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				msm_batt_info.charging_source);
			break;
		case BATT_CHG_CURRENT: // ICHG ADC code (charging current)	
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				msm_batt_info.chg_current_adc);
			break;
#ifdef __BATT_TEST_DEVICE__
		case BATT_TEMP_TEST_ADC:
				i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", temp_test_adc);
			break;
#endif
#ifdef CONFIG_WIRELESS_CHARGING
		case WC_STATUS:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				msm_batt_info.batt_wireless);
			break;
		case WC_ADC:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				msm_batt_info.wc_adc);
			break;
#endif
		case CHARGINGBLOCK_CLEAR:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", msm_batt_info.chargingblock_clear);
			break;

#ifdef BATTERY_CHECK_OVP
        case BATT_OVP_STATUS:
            i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                msm_batt_info.batt_ovp);
            break;

        case BATT_OVP_CHG_STATUS:
            i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                msm_batt_info.batt_ovp_chg_block);
            break;
#endif

		default:
			i = -EINVAL;
	}

	return i;
}

static ssize_t msm_batt_store_property(struct device *dev, 
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t offset = attr - ancora_battery_attrs;


	switch (offset) {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	case RESET_SOC:
 		if (sscanf(buf, "%d\n", &x) == 1) {
			fg_reset_soc();	// rilactionservice.java...
			ret = count;
		}
		break;
#endif
#ifdef __BATT_TEST_DEVICE__
	case BATT_TEMP_TEST_ADC:
 		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 0)
				temp_test_adc = 0;
			else
			{
				temp_test_adc = x;
			}
			ret = count;
		}
		break;
#endif
	case CHARGINGBLOCK_CLEAR:
 		if (sscanf(buf, "%d\n", &x) == 1) {
			pr_debug("\n[BATT] %s: chargingblock_clear -> write 0x%x\n\n", __func__, x);
			msm_batt_info.chargingblock_clear = x;
			ret = count;
		}
		break;
	default:
		return -EINVAL;
	}	/* end of switch */

 	return ret;
}
#endif

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = msm_batt_info.charging_source & AC_CHG
			    ? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = msm_batt_info.charging_source & USB_CHG
			    ? 1 : 0;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_BATT_TEMP,
	POWER_SUPPLY_PROP_BATT_TEMP_ADC,
	POWER_SUPPLY_PROP_BATT_VOL,
	POWER_SUPPLY_PROP_BATT_VOL_ADC,
	POWER_SUPPLY_PROP_BATT_VF_ADC,
	POWER_SUPPLY_PROP_BATT_VOL_ADC_AVER,
	POWER_SUPPLY_PROP_BATT_TEMP_ADC_AVER,
	POWER_SUPPLY_PROP_BATT_VOL_AVER,
	POWER_SUPPLY_PROP_BATT_TEMP_AVER,
	POWER_SUPPLY_PROP_BATT_TYPE,
	POWER_SUPPLY_PROP_BATT_FULL,
};

#ifdef CONFIG_MAX17043_FUEL_GAUGE
static u32 get_voltage_from_fuelgauge(void)
{
	if (is_attached)
		return (fg_read_vcell());	// +0 (voltage drop compensation)
	return 3700;	// default 
}

static u32 get_level_from_fuelgauge(void)
{
	if (is_attached)
		return fg_read_soc();
	return 100;	// default 
}
#endif

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_batt_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;

#ifdef BATTERY_CHECK_OVP
        if (board_hw_revision >= 0x06) {
            if (msm_batt_info.batt_ovp == 1 && msm_batt_info.charging_source != NO_CHG) {
                val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            }
        }
#endif
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = msm_batt_info.batt_voltage_now;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
	case POWER_SUPPLY_PROP_BATT_TEMP:
		val->intval = msm_batt_info.battery_temp;
		break;
	case POWER_SUPPLY_PROP_BATT_TEMP_ADC:
		val->intval = msm_batt_info.battery_temp_adc;
		break;		
	case POWER_SUPPLY_PROP_BATT_VOL:
		val->intval = msm_batt_info.battery_voltage;
		break;
	case POWER_SUPPLY_PROP_BATT_VOL_ADC:
		val->intval = msm_batt_info.battery_voltage_adc;
		break;
	case POWER_SUPPLY_PROP_BATT_VF_ADC:
//		val->intval = msm_batt_info.battery_vf;
		break;		
	case POWER_SUPPLY_PROP_BATT_VOL_ADC_AVER:
		val->intval = msm_batt_info.battery_level;
		break;
	case POWER_SUPPLY_PROP_BATT_TEMP_ADC_AVER:
		val->intval = msm_batt_info.battery_temp;
		break;
	case POWER_SUPPLY_PROP_BATT_VOL_AVER:
		val->intval = msm_batt_info.battery_voltage;
		break;
	case POWER_SUPPLY_PROP_BATT_TEMP_AVER:
		val->intval = msm_batt_info.battery_temp_adc;
		break;
	case POWER_SUPPLY_PROP_BATT_TYPE:
		break;
	case POWER_SUPPLY_PROP_BATT_FULL:
		val->intval = msm_batt_info.batt_full_check;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};

#ifdef MAX17043_FUEL_GAUGE
struct timer_list fg_alert_timer;
extern int (*fg_alert_handler)(int);
static void fg_set_alert_ext(unsigned long arg)
{
	
	if (msm_batt_info.charging_source == NO_CHG)
	{
		pr_info("[BATT] %s: low battery, power off...\n", __func__);
		is_alert = 1;
		wake_lock_timeout(&vbus_wake_lock, 30 * TIME_UNIT_SECOND);
	}
	else
		is_alert = 0;
}

static int fg_set_alert(int value)
{
	
	if (value)
	{
		is_alert = 
		mod_timer(&fg_alert_timer, (jiffies + (1 * TIME_UNIT_MINUTE)));
//		is_alert = 1;
//		wake_lock_timeout(&vbus_wake_lock, 30 * TIME_UNIT_SECOND);
	}
	else
	{
		// clear alert flag
		is_alert = 0;
	}

	return is_alert;
}


#endif	/* MAX17043_FUEL_GAUGE */

static void msm_batt_chg_en(chg_enable_type enable)
{
#ifdef CONFIG_CHARGER_SMB328A	
	struct power_supply *psy = power_supply_get_by_name("smb328a-charger");
   	union power_supply_propval val_type, val_chg_current, val_topoff;
	int ret;
#endif
	
	if (enable == START_CHARGING)
	{
		if (msm_batt_info.charging_source == NO_CHG)	// *Note: DO NOT USE "&" operation for NO_CHG (0x0), it returns FALSE always.
		{
			pr_err("[BATT] %s: charging_source not defined!\n", __func__);
			return ;
		}

		// Set charging current (ICHG; mA)
		if (msm_batt_info.charging_source & AC_CHG)
		{
			pr_info("[BATT] %s: Start charging! (charging_source = AC, wireless = %d)\n", __func__, msm_batt_info.batt_wireless);
			hsusb_chg_connected_ext(USB_CHG_TYPE__WALLCHARGER);
#ifdef CONFIG_WIRELESS_CHARGING
			if (msm_batt_info.batt_wireless)
				hsusb_chg_vbus_draw_ext(500);	// wireless charging	(450mA)
			else
#endif

			if (board_hw_revision >= 0x06)
				val_chg_current.intval = 600;
			else
				hsusb_chg_vbus_draw_ext(650);	// TA charging	(600mA)

		}
		else // USB_CHG
		{
			pr_info("[BATT] %s: Start charging! (charging_source = USB)\n", __func__);
			hsusb_chg_connected_ext(USB_CHG_TYPE__SDP);
	
			if (board_hw_revision >= 0x06)
				val_chg_current.intval = 450;
			else
				hsusb_chg_vbus_draw_ext(500); // USB charging	(450mA)
		}

		msm_batt_set_charging_start_time(START_CHARGING);
	
		if (board_hw_revision >= 0x06)
		{
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			
			/* Step1. */
			/* Set charging current */
 			ret = psy->set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW,
   						&val_chg_current);
	    	if (ret) {
    				printk("[BATT] %s fail to set charging cur(%d)\n",
    					__func__, ret);
    				return ret;
	    	}

			/* Step2 */
			/* Set topoff current */
	 		/* from 25mA to 200mA, in 25mA step */
 			val_topoff.intval = 150;
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL,
 						&val_topoff);
	 		if (ret) {
 					printk("[BATT] %s fail to set topoff cur(%d)\n",
 					__func__, ret);
 				return ret;
	 		}

			/*Step3 */
			ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &val_type);
		 	if (ret) {
 				printk("[BATT] %s: fail to set charging status(%d)\n",
 					__func__, ret);
		 		return ret;
		 	}
		}

#ifdef MAX17043_FUEL_GAUGE
		fg_set_alert(0);
#endif
	}
	else	// STOP_CHARGING
	{
		msm_batt_set_charging_start_time(STOP_CHARGING);

		if (msm_batt_info.charging_source == NO_CHG)
		{
			if(board_hw_revision >= 0x06)
			{
				val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;
				
				ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &val_type);
				if (ret) {
					printk("[BATT] %s: fail to set charging status(%d)\n",
						__func__, ret);
					return ret;
				}
			}
			else
				hsusb_chg_connected_ext(USB_CHG_TYPE__INVALID); // not charging
		}
		else // Charging
		{
			if(board_hw_revision >= 0x06)
			{
				val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;
				
				ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &val_type);
				if (ret) {
					printk("[BATT] %s: fail to set charging status(%d)\n",
						__func__, ret);
					return ret;
				}
			}
			else
				hsusb_chg_vbus_draw_ext(0); // discharging
		}
		
		msm_batt_average_chg_current(-1);	// Initialize all current data sampling

		pr_info("[BATT] %s: Stop charging! (charging_source = 0x%x, full_check = %d)\n",
			__func__, msm_batt_info.charging_source, msm_batt_info.batt_full_check);
	}
}

static int msm_batt_average_chg_current(int chg_current_adc)
{
	static int history[AVERAGE_COUNT] = {0};
	static int count = 0;
	static int index = 0;
	int i, sum, max, min, ret;

	

	if (chg_current_adc == 0)
		return 0;

	if (chg_current_adc < 0)	// initialize all data
	{
		count = 0;
		index = 0;
		for (i=0; i<AVERAGE_COUNT; i++)	history[i] = 0;

		return 0;
	}

	if (count == 0)	// no data
	{
		for (i=0; i<AVERAGE_COUNT; i++)	history[i] = chg_current_adc;
	}

	if (index >= count)	count++;

	max = min = history[0];
	sum = 0;

	for (i=0; i<AVERAGE_COUNT; i++)
	{
		if (i == index)
		{
			history[i] = chg_current_adc;
		}

		if (max < history[i])	max = history[i];
		if (min > history[i])	min = history[i];

		sum += history[i];
	}

	ret = ((sum-max-min) / (AVERAGE_COUNT-2));

	index++;
	if (index == AVERAGE_COUNT)
	{
		history[0] = ret;
		index = 1;
	}

	pr_debug("[BATT] %s: adc=%d, sum=%d, max=%d, min=%d, ret=%d\n", __func__, chg_current_adc, sum, max, min, ret);

	if (count < AVERAGE_COUNT)
	{
		if (board_hw_revision >= 0x06)
			return (BATT_FULL_CHARGING_CURRENT_REV_5+50);	// do not check full charging before current sampling is stable...
		else
			return (BATT_FULL_CHARGING_CURRENT+50);	// do not check full charging before current sampling is stable...
	}
				

	return ret;
}

static int msm_batt_check_full_charging(int chg_current_adc)
{
	static unsigned int time_after_under_tsh = 0;
	int charging_current;

	// Check H/W rev.
	if (board_hw_revision >= 0x06)
		charging_current = BATT_FULL_CHARGING_CURRENT_REV_5;
	else
		charging_current = BATT_FULL_CHARGING_CURRENT;

	if (chg_current_adc == 0)
		return 0;	// not charging

	// check charging absolute time
	if (msm_batt_is_over_abs_time())
	{
		pr_info("[BATT] %s: Fully charged, over abs time! (recharging=%d)\n", __func__, msm_batt_info.batt_recharging);
		msm_batt_info.batt_full_check = 1;
		msm_batt_info.batt_recharging = 0;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		msm_batt_chg_en(STOP_CHARGING);
		return 1;
	}

	if (msm_batt_info.battery_voltage >= BATT_FULL_CHARGING_VOLTAGE)
	{
		// check charging current threshold
		if (chg_current_adc < charging_current)
		{
			if (time_after_under_tsh == 0)
				time_after_under_tsh = jiffies;
			else
			{
				if (time_after((unsigned long)jiffies, (unsigned long)(time_after_under_tsh + TOTAL_WATING_TIME)))
				{
					// fully charged !
					pr_info("[BATT] %s: Fully charged, cut off charging current! (voltage=%d, ICHG=%d)\n",
						__func__, msm_batt_info.battery_voltage, chg_current_adc);
					msm_batt_info.batt_full_check = 1;
					msm_batt_info.batt_recharging = 0;
					msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
					time_after_under_tsh = 0;
					msm_batt_chg_en(STOP_CHARGING);
					return 1;
				}
			}
		}
		else
		{
			time_after_under_tsh = 0;
		}
	}

	return 0;
}

static int msm_batt_check_recharging(void)
{
	static unsigned int time_after_vol1 = 0, time_after_vol2 = 0;


	if ( (msm_batt_info.batt_full_check == 0) ||
		(msm_batt_info.batt_recharging == 1) ||
		(msm_batt_info.batt_health != POWER_SUPPLY_HEALTH_GOOD) )
	{
		time_after_vol1 = 0;
		time_after_vol2 = 0;
		return 0;
	}

	/* check 1st voltage */
	if (msm_batt_info.battery_voltage <= BATT_RECHARGING_VOLTAGE_1)
	{
		if (time_after_vol1 == 0)
			time_after_vol1 = jiffies;

		if (time_after((unsigned long)jiffies, (unsigned long)(time_after_vol1 + TOTAL_WATING_TIME)))
		{
			pr_info("[BATT] %s: Recharging ! (voltage1 = %d)\n", __func__, msm_batt_info.battery_voltage);
			msm_batt_info.batt_recharging = 1;
			msm_batt_chg_en(START_CHARGING);
			return 1;
		}
	}
	else
		time_after_vol1 = 0;

	/* check 2nd voltage */
	if (msm_batt_info.battery_voltage <= BATT_RECHARGING_VOLTAGE_2)
	{
		if (time_after_vol2 == 0)
			time_after_vol2 = jiffies;

		if (time_after((unsigned long)jiffies, (unsigned long)(time_after_vol2 + TOTAL_WATING_TIME)))
		{
			pr_info("[BATT] %s: Recharging ! (voltage2 = %d)\n", __func__, msm_batt_info.battery_voltage);
			msm_batt_info.batt_recharging = 1;
			msm_batt_chg_en(START_CHARGING);
			return 1;
		}
	}
	else
		time_after_vol2 = 0;

	return 0;
}

static int msm_batt_check_level(int battery_level)
{
	/*
	if (msm_batt_info.batt_full_check)
	{
		battery_level = 100;
	}
	*/
	if ( (msm_batt_info.batt_full_check == 0) && (battery_level == 100) )
	{
		battery_level = 99;	// not yet fully charged
	}
/*
	else if ( (battery_level == 0)
#ifdef MAX17043_FUEL_GAUGE
		&& (is_alert == 0)
#endif
		)
	{
		battery_level = 1;	// not yet alerted low battery (do not power off yet)
	}

	if (msm_batt_info.battery_voltage< msm_batt_info.voltage_min_design)
	{
		battery_level = 0;
	}
*/
	if (msm_batt_info.batt_capacity != battery_level)
	{
		pr_info("[BATT] %s: Battery level changed ! (%d -> %d)\n", __func__, msm_batt_info.batt_capacity, battery_level);
		msm_batt_info.batt_capacity = battery_level;
		return 1;
	}

#ifdef MAX17043_FUEL_GAUGE
	if (is_alert)
		return 1;	// force update to power off !
#endif

	return 0;
}

static int msm_batt_average_temperature(int temp_adc)
{
	static int history[AVERAGE_COUNT] = {0};
	static int count = 0;
	static int index = 0;
	int i, sum, max, min, ret;

	if (temp_adc == 0)
		return 0;

	if (count == 0 && temp_adc == 150)
		return 0;

#ifdef __BATT_TEST_DEVICE__
		if (temp_test_adc)
			return temp_test_adc;
#endif

	if (count == 0)	// no data
	{
		for (i=0; i<AVERAGE_COUNT; i++)	history[i] = temp_adc;
	}

	if (index >= count)	count++;

	max = min = history[0];
	sum = 0;

	for (i=0; i<AVERAGE_COUNT; i++)
	{
		if (i == index)
		{
			history[i] = temp_adc;
		}

		if (max < history[i])	max = history[i];
		if (min > history[i])	min = history[i];

		sum += history[i];
	}

	ret = ((sum-max-min) / (AVERAGE_COUNT-2));

	index++;
	if (index == AVERAGE_COUNT)
	{
		history[0] = ret;
		index = 1;
	}

	pr_debug("[BATT] %s: adc=%d, sum=%d, max=%d, min=%d, ret=%d\n", __func__, temp_adc, sum, max, min, ret);
	return ret;
}

static int msm_batt_control_temperature(int temp_adc)
{
	int prev_health = msm_batt_info.batt_health;
	int new_health = prev_health;
	int array_size = 0;
	int i;
	int degree;

 
	static char *health_text[] = {
		"Unknown", "Good", "Overheat", "Dead", "Over voltage",
		"Unspecified failure", "Cold",
	};

	if (temp_adc == 0)
		return 0;

#ifdef __AUTO_TEMP_TEST__
	static unsigned int auto_test_start_time = 0;
	static unsigned int auto_test_interval = (2 * TIME_UNIT_MINUTE);
	static int auto_test_mode = 0;	// 0: normal (recover cold), 1: force overheat, 2: normal (recover overheat), 3: force cold
 		
	if (msm_batt_info.charging_source != NO_CHG)	// charging
	{
		if (auto_test_start_time == 0)
			auto_test_start_time = jiffies;

		if (time_after((unsigned long)jiffies, (unsigned long)(auto_test_start_time + auto_test_interval)))
		{
			auto_test_mode++;
			if (auto_test_mode > 3) auto_test_mode = 0;
			auto_test_start_time = jiffies;
		}
		pr_debug("[BATT] auto test mode = %d (0:normal,1:overheat,2:normal,3:cold)\n", auto_test_mode);

		if (auto_test_mode == 1)
		{
			temp_adc = BATT_TEMP_HIGH_BLOCK + 10;
			msm_batt_info.battery_temp_adc = temp_adc;
		}
		else if (auto_test_mode == 3)
		{
			temp_adc = BATT_TEMP_LOW_BLOCK - 10;
			msm_batt_info.battery_temp_adc = temp_adc;
		}
	}
	else	// not charging
	{
		auto_test_start_time = 0;
		auto_test_mode = 0;
	}
#endif

	// map in celcius degree
	array_size = ARRAY_SIZE(temp_table);
	for (i = 0; i <= (array_size - 1); i++)
	{		
		if (i == 0)
		{
			if (temp_adc >= temp_table[0][0])
			{
				degree = temp_table[0][1];
				break;
			}
			else if (temp_adc <= temp_table[array_size-1][0])
			{
				degree = temp_table[array_size-1][1];
				break;
			}
		}

		if (temp_table[i][0] < temp_adc && temp_table[i-1][0] >= temp_adc)
		{
			degree = temp_table[i-1][1];
		}
	}

	msm_batt_info.battery_temp = degree;	// celcius degree

	// TODO:  check application

	if (prev_health == POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
	{
		return 0;	// do not check temperature... (charging is already blocked!)
	}

	if (temp_adc <= BATT_TEMP_HIGH_BLOCK)
	{
		// over high block
		if (prev_health != POWER_SUPPLY_HEALTH_OVERHEAT)
			new_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	else if ((temp_adc >= BATT_TEMP_HIGH_RECOVER) && (temp_adc <= BATT_TEMP_LOW_RECOVER))
	{
		// low recover ~ high recover (normal)
		if ( (prev_health == POWER_SUPPLY_HEALTH_OVERHEAT) || 
			(prev_health == POWER_SUPPLY_HEALTH_COLD) )
			new_health = POWER_SUPPLY_HEALTH_GOOD;
	}
	else if (temp_adc >= BATT_TEMP_LOW_BLOCK)
	{
		// under low block
		if (prev_health != POWER_SUPPLY_HEALTH_COLD)
			new_health = POWER_SUPPLY_HEALTH_COLD;
	}

	if (msm_batt_info.charging_source == NO_CHG)
	{
		if ((BATT_TEMP_LOW_BLOCK > temp_adc) && (temp_adc > BATT_TEMP_HIGH_BLOCK))
		{
			if ( (prev_health == POWER_SUPPLY_HEALTH_OVERHEAT) ||
				(prev_health == POWER_SUPPLY_HEALTH_COLD) )
				new_health = POWER_SUPPLY_HEALTH_GOOD;
		}
	}

	if (msm_batt_info.chargingblock_clear != 0x0)
		new_health = POWER_SUPPLY_HEALTH_GOOD;

	if (prev_health != new_health)
	{
		if (msm_batt_info.charging_source == NO_CHG)	// not charging
		{
			pr_info("[BATT] %s: Health changed by temperature! (ADC = %d, %s-> %s)\n",
				__func__, temp_adc, health_text[prev_health], health_text[new_health]);
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		else	// in charging
		{
			if (new_health != POWER_SUPPLY_HEALTH_GOOD)	// block!
			{
				pr_info("[BATT] %s: Block charging! (ADC = %d, %s-> %s)\n",
					__func__, temp_adc, health_text[prev_health], health_text[new_health]);
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
				msm_batt_chg_en(STOP_CHARGING);
			}
			else										// recover!
			{
				pr_info("[BATT] %s: Recover charging! (ADC = %d, %s-> %s)\n",
					__func__, temp_adc, health_text[prev_health], health_text[new_health]);
				msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
				msm_batt_chg_en(START_CHARGING);
			}
		}

		msm_batt_info.batt_health = new_health;
	}

	return 1;
//	return 0;	// nothing is changed
}

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))
#define	be16_to_cpu_self(v)	(v = be16_to_cpu(v))

#ifdef BATTERY_CHECK_OVP
static int msm_batt_block_ovp_chg(void)
{
#ifdef CONFIG_CHARGER_SMB328A
	struct power_supply *psy = power_supply_get_by_name("smb328a-charger");
	union power_supply_propval val_status;
	int ret;

    /* check battery charging status */
    if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_CHARGING) {
        return 0;
    }

    /* check charger connected */
    if (msm_batt_info.charging_source == NO_CHG) {
        return 0;
    }

    /* check ovp status */
    ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS,
	            &val_status);
	if (ret) {
        printk("[BATT] %s fail to get ovp status from SMB328 charger! (%d)\n",
            __func__, ret);
        return 0;
    }

    if (val_status.intval == POWER_SUPPLY_STATUS_DISCHARGING) {
        msm_batt_info.batt_ovp = 1;
        printk("\n[BATT] ##### OVP detected!!! ####\n\n");
    } else {
        msm_batt_info.batt_ovp = 0;
    }
#else
    msm_batt_info.batt_ovp = 0;
#endif

    if (msm_batt_info.batt_ovp == 1)
        return 1;
    else
        return 0;
}

static int msm_batt_resume_ovp_chg(void)
{
#ifdef CONFIG_CHARGER_SMB328A
    struct power_supply *psy = power_supply_get_by_name("smb328a-charger");
	union power_supply_propval val_status;
	int ret;

    /* check charging blocked from ovp */
    if (msm_batt_info.batt_ovp_chg_block == 0)
        return 0;

    /* check charger connected */
    if (msm_batt_info.charging_source == NO_CHG) {
        return 0;
    }

    /* check battery status is not charging */
    if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING) {
        msm_batt_info.batt_ovp_chg_block = 0;
        return 0;
    }

    /* check ovp status */
    ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS,
                &val_status);
	if (ret) {
        printk("[BATT] %s fail to get ovp status from SMB328 charger! (%d)\n",
            __func__, ret);
        return 0;
    }

    if (val_status.intval == POWER_SUPPLY_STATUS_CHARGING) {
        printk("\n[BATT] ##### OVP relesed!!! ####\n\n");
        msm_batt_info.batt_ovp = 0;
        return 1;
    }
    return 0;
#else
    return 0;
#endif
}
#endif

static int msm_batt_get_batt_chg_status(void)
{
	int rc ;
	struct rpc_req_batt_chg {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_batt_chg;
	struct rpc_reply_batt_chg_v1 *v1p;


	req_batt_chg.more_data = cpu_to_be32(1);

	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	v1p = &rep_batt_chg.v1;
	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_batt_chg, sizeof(rep_batt_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(v1p->more_data)) {
		be32_to_cpu_self(v1p->charger_status);
		be32_to_cpu_self(v1p->charger_type);
		be32_to_cpu_self(v1p->battery_status);
		be32_to_cpu_self(v1p->battery_level);
		be32_to_cpu_self(v1p->battery_voltage);
		be32_to_cpu_self(v1p->battery_temp);
		be32_to_cpu_self(v1p->chg_current);
#ifdef CONFIG_WIRELESS_CHARGING
		be32_to_cpu_self(v1p->wc_adc);
#endif
	} else {
		pr_err("%s: No battery/charger data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;
}

static void msm_batt_update_psy_status(void)
{
	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32	fuel_battery_voltage;
	u32 battery_voltage_adc;
	u32	battery_temp_adc;
	u32 chg_current_adc;
#ifdef CONFIG_WIRELESS_CHARGING
	u32 wc_adc;
#endif

#ifdef CONFIG_CHARGER_SMB328A	
	struct power_supply *psy = power_supply_get_by_name("smb328a-charger");
	union power_supply_propval val_status;
	int ret;
#endif

	u32 status_changed = 0;

	/* Get general status from CP by RPC */
	if (msm_batt_get_batt_chg_status())
		return;

	charger_status = rep_batt_chg.v1.charger_status;
	charger_type = rep_batt_chg.v1.charger_type;
	battery_status = rep_batt_chg.v1.battery_status;
	battery_temp_adc = rep_batt_chg.v1.battery_temp;
//	chg_current_adc = rep_batt_chg.v1.chg_current;
	chg_current_adc = battery_voltage_adc = rep_batt_chg.v1.battery_voltage;  // Use voltage for current adc

	msm_batt_info.battery_voltage_adc = battery_voltage_adc;
	msm_batt_info.battery_temp_adc = battery_temp_adc;
	batt_temp_adc_info = battery_temp_adc;
#ifdef CONFIG_WIRELESS_CHARGING
	wc_adc = rep_batt_chg.v1.wc_adc;
	wc_adc = wc_adc * 46 / 160;	// HW req.
#endif

//	printk("[SSAM] battery status from cp : %d\n", battery_status);

	if ( (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING) ||
		(msm_batt_info.batt_recharging == 1) )
	{
		pr_debug("[BATT] %s: chg_current_adc from CP = %d\n", __func__, chg_current_adc);
		if (chg_current_adc < 30)
			chg_current_adc = 0;
	}
	else
		chg_current_adc = 0;	// not charging

#ifdef CONFIG_MAX17043_FUEL_GAUGE
	battery_level = get_level_from_fuelgauge();
	fuel_battery_voltage = get_voltage_from_fuelgauge();
#endif

	msm_batt_info.battery_voltage = fuel_battery_voltage;
	msm_batt_info.batt_voltage_now = fuel_battery_voltage;

#ifdef CONFIG_WIRELESS_CHARGING
	msm_batt_info.wc_adc = wc_adc;
#endif

	if(board_hw_revision >= 0x06 && psy != NULL)
	{
		/* Check battery status from CP */
		if(boot_check_wrong_battery == 1 && battery_status == BATTERY_STATUS__INVALID)
		{
            printk("[BATT] wrong battery detected!!\n");
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			
			return;
		}
		
#ifdef BATTERY_CHECK_OVP
	    /* resume charging scenario */

	    if (msm_batt_resume_ovp_chg() == 1){

	        msm_batt_info.batt_ovp_chg_block = 0;

	        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;

	        msm_batt_chg_en(START_CHARGING);

	        return;

	    }

	    /* block charging scenario */

	    if (msm_batt_block_ovp_chg() == 1) {

	        msm_batt_info.batt_ovp_chg_block = 1;

	        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		    msm_batt_chg_en(STOP_CHARGING);

	        return;

	    }

#endif
	}
	/**************************/
	/* Check what is changed */

#ifdef USE_SMB_VF_CHECK // VF Check is Modem side so this code delete

	if(board_hw_revision >= 0x06)
	{
		/* check vf */
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT,
  						&val_status);
	   	if (ret) {
   				printk("[BATT] %s fail to get vf status (%d)\n",
   					__func__, ret);
	   			return ret;
	   	}

//		printk("[SSAM] batt status %d\n", val_status.intval);

		if(!val_status.intval)
		{
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

			/* Check LPM */
			if(charging_boot)
			{
				msm_batt_chg_en(STOP_CHARGING);
				return;
			}
			else
				pm_power_off();
		}
	}
#endif

 	/* check temperature */
//	msm_batt_info.battery_temp_adc = msm_batt_average_temperature(battery_temp_adc);
	

	status_changed += msm_batt_control_temperature(msm_batt_info.battery_temp_adc);

	/* check full charging */
	msm_batt_info.chg_current_adc = msm_batt_average_chg_current(chg_current_adc);
	status_changed += msm_batt_check_full_charging(msm_batt_info.chg_current_adc);

	/* check recharging */
	status_changed += msm_batt_check_recharging();

	/* battery level, capacity (%) */
	status_changed += msm_batt_check_level(battery_level);

	/* temperature health for power off charging */
	if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_GOOD)
		msm_batt_info.batt_temp_check = 1;
	else
		msm_batt_info.batt_temp_check = 0;

#ifndef DEBUG
	if (msm_batt_info.charging_source != NO_CHG)
#endif
	{
		if(!power_down)
		{
#ifdef BATTERY_CHECK_OVP
			pr_info("[BATT] %s: chg_type=%d, bat_status=%d, bat_adc=%d, chg_current=%d, bat_full=%d, bat_recharging=%d bat_ovp=%d,%d\n",
				__func__, msm_batt_info.charger_type, msm_batt_info.batt_health, msm_batt_info.battery_temp_adc, msm_batt_info.chg_current_adc, msm_batt_info.batt_full_check, msm_batt_info.batt_recharging, msm_batt_info.batt_ovp, msm_batt_info.batt_ovp_chg_block);
#else
			pr_info("[BATT] %s: charger_type=%d, battery_status=%d, battery_temp_adc=%d, chg_current=%d, battery_full=%d, battery_recharging=%d\n",
				__func__, msm_batt_info.charger_type, msm_batt_info.batt_health, msm_batt_info.battery_temp_adc, msm_batt_info.chg_current_adc, msm_batt_info.batt_full_check, msm_batt_info.batt_recharging);
#endif
		}
	}

	if (status_changed)
	{
		if(!power_down)
		{
			pr_info("[BATT] %s: power_supply_changed !\n", __func__);
		}
		power_supply_changed(&msm_psy_batt);
	}

	if (msm_batt_unhandled_interrupt)
	{
		msm_batt_cable_status_update();
		msm_batt_unhandled_interrupt = 0;
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct  batt_modify_client_rep *data_ptr, *buf_ptr;

	

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
	     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req  req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;


	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n",
				__func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
				__func__, rep.result);
		return -EIO;
	}

	return 0;
}

void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;


	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL, BATTERY_LOW);

		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client. rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;


	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client FAIL rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	msm_batt_update_psy_status();
	pr_debug("%s: exit\n", __func__);
}
#endif

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,

		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;


	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{

	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;


	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct  msm_batt_vbatt_filter_req  vbatt_filter_req;
	struct  msm_batt_vbatt_filter_rep  vbatt_filter_rep;


	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
		       __func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;



	*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->cb_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->more_data);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_reg_req->batt_error);
	size += sizeof(u32);

	return size;
}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;


	data_ptr = (struct batt_client_registration_rep *)data;
	buf_ptr = (struct batt_client_registration_rep *)buf;

	data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);

	return 0;
}

static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_rep batt_reg_rep;
	int rc;


	batt_reg_req.desired_batt_voltage = desired_batt_voltage;
	batt_reg_req.voltage_direction = voltage_direction;
	batt_reg_req.batt_cb_id = batt_cb_id;
	batt_reg_req.cb_data = cb_data;
	batt_reg_req.more_data = 1;
	batt_reg_req.batt_error = 0;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, &batt_reg_req,
			msm_batt_register_ret_func, &batt_reg_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	pr_debug("%s: got handle = %d\n", __func__, msm_batt_info.batt_handle);

	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct  batt_client_deregister_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;


	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;


	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
		       __func__, rep.batt_error, batt_handle);
		return -EIO;
	}

	return 0;
}

static int msm_batt_get_charger_type(void)
{
	int charger_type = CHARGER_TYPE_NONE;

	charger_type = fsa9480_get_charger_status();

	return charger_type;
}

#ifdef CONFIG_WIRELESS_CHARGING
static int msm_batt_get_wireless_status(void)
{
	int wc_detect = 0;
	int charger_type = msm_batt_get_charger_type();


	if (charger_type == CHARGER_TYPE_NONE)	// no cable inserted
	{
		wc_detect = gpio_get_value_cansleep(GPIO_WC_DETECT);
		return wc_detect;
	}

	return 0;	// plugged
}
#endif

static void msm_batt_cable_status_update(void)
{
	/* Check charger type and update if changed */

	int charger_type = CHARGER_TYPE_NONE;

	static char *health_text[] = {
		"Unknown", "Good", "Overheat", "Dead", "Over voltage",
		"Unspecified failure", "Cold",
	};

	charger_type = msm_batt_get_charger_type();

#ifdef CONFIG_WIRELESS_CHARGING
	msm_batt_info.batt_wireless = msm_batt_get_wireless_status();

	if (msm_batt_info.batt_wireless)
	{
		charger_type = CHARGER_TYPE_WALL;
	}
#endif

	msm_batt_info.charger_type = charger_type;
	pr_info("[BATT] %s: charger_type = %d (0:none, 1:TA, 2:USB) \n", __func__, charger_type);

	msm_batt_info.batt_full_check = 0;
	msm_batt_info.batt_recharging = 0;

	if (charger_type != CHARGER_TYPE_NONE)	// USB, TA, Wireless
	{
		if (charger_type == CHARGER_TYPE_USB_PC)
		{
			msm_batt_info.charging_source = USB_CHG;
			hsusb_chg_connected_ext(USB_CHG_TYPE__SDP);
			power_supply_changed(&msm_psy_usb);
		}
		else	// TA and Wireless
		{
			msm_batt_info.charging_source = AC_CHG;
			hsusb_chg_connected_ext(USB_CHG_TYPE__WALLCHARGER);
			power_supply_changed(&msm_psy_ac);
		}

		if (msm_batt_info.batt_health != POWER_SUPPLY_HEALTH_GOOD)
		{
			pr_info("[BATT] %s: Battery health is %s, stop charging! \n", __func__, health_text[msm_batt_info.batt_health]);
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			msm_batt_chg_en(STOP_CHARGING);
		}
		else
		{
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
			msm_batt_chg_en(START_CHARGING);
		}

		wake_lock(&vbus_wake_lock);
	}
	else	// No charger
	{
		msm_batt_info.charging_source = NO_CHG;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		msm_batt_chg_en(STOP_CHARGING);
		wake_lock_timeout(&vbus_wake_lock, 5 * TIME_UNIT_SECOND);
	}

	pr_info("[BATT] %s: power_supply_changed !\n", __func__);
	power_supply_changed(&msm_psy_batt);

	if (msm_batt_unhandled_interrupt)
		msm_batt_unhandled_interrupt = 0;
}

static int msm_batt_suspend(struct platform_device *pdev, 
		pm_message_t state)
{
	pr_debug("[BATT] %s\n",__func__);
	del_timer_sync(&msm_batt_info.timer);

	return 0;
}

static int msm_batt_resume(struct platform_device *pdev)
{
	pr_debug("[BATT] %s\n",__func__);
	queue_work(msm_batt_info.msm_batt_wq, &msm_batt_work);	
	mod_timer(&msm_batt_info.timer, (jiffies + BATT_CHECK_INTERVAL));

	return 0;
}

int batt_restart(void)
{
	
	if (msm_batt_driver_init)
	{
		msm_batt_cable_status_update();

		del_timer_sync(&msm_batt_info.timer);
		queue_work(msm_batt_info.msm_batt_wq, &msm_batt_work);
		mod_timer(&msm_batt_info.timer, (jiffies + BATT_CHECK_INTERVAL));
	}
	else
	{
		pr_err("[BATT] %s: Battery driver is not ready !!\n", __func__);
		msm_batt_unhandled_interrupt = 1;
	}

	return 0;
}
EXPORT_SYMBOL(batt_restart);

static int msm_batt_cleanup(void)
{
	int rc = 0;

	pr_info("[BATT] %s\n", __func__);

	del_timer_sync(&msm_batt_info.timer);
	msm_batt_remove_attrs(msm_psy_batt.dev);
	i2c_del_driver(&fg_i2c_driver);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
			       __func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);

	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(chg_ep). rc=%d\n",
			       __func__, rc);
		}
	}

#ifdef CONFIG_WIRELESS_CHARGING
	free_irq(IRQ_WC_DETECT, NULL);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
	return rc;
}

int msm_batt_get_charger_api_version(void)
{
	int rc ;
	struct rpc_reply_hdr *reply;


	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;


	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VER_1_1,
			 ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
			sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err("%s: FAIL: msm_rpc_write. proc=0x%08x, rc=%d\n",
		       __func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *) &reply, -1,
				BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s: LENGTH ERR: msm_rpc_read. rc=%d (<%d)\n",
			       __func__, rc, RPC_REQ_REPLY_COMMON_HEADER_SIZE);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {
			pr_err("%s: TYPE ERR: type=%d (!=%d)\n",
			       __func__, reply->type, RPC_TYPE_REQ);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {
			pr_err("%s: XID ERR: xid=%d (!=%d)\n", __func__,
			       reply->xid, req_chg_api_ver.hdr.xid);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat !=
				RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =
			be32_to_cpu(rep_chg_api_ver->num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions =  (u32 *)
			((u8 *) reply + sizeof(struct rpc_reply_hdr) +
			sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(
			rep_chg_api_ver->chg_api_versions[num_of_versions - 1]);

		pr_debug("%s: num_of_chg_api_versions = %u. "
			"  The chg api version = 0x%08x\n", __func__,
			num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;


	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
	{
		del_timer_sync(&msm_batt_info.timer);
		queue_work(msm_batt_info.msm_batt_wq, &msm_batt_work);
		mod_timer(&msm_batt_info.timer, (jiffies + BATT_CHECK_INTERVAL));
	}

	return rc;
}

#ifdef CONFIG_WIRELESS_CHARGING
static irqreturn_t wc_detect_irq_handler(int irq, void *data)
{
	int wc_detect = msm_batt_get_wireless_status();

	
	pr_info("[BATT] %s: WC_DETECT = (%d)\n", __func__, wc_detect);

	if (wc_detect == msm_batt_info.batt_wireless)	// wireless status is not changed...
	{
		return IRQ_HANDLED;
	}

	batt_restart();	// charger status changed...
	return IRQ_HANDLED;
}
#endif

static void msm_batt_set_charging_start_time(chg_enable_type enable)
{
	
	if (enable == START_CHARGING)
	{
		charging_start_time = jiffies;
	}
	else	// STOP_CHARGING
	{
		charging_start_time = 0;
	}
}

static int msm_batt_is_over_abs_time(void)
{
	unsigned int total_time;


	if (charging_start_time == 0)
	{
		return 0;	// not charging
	}

	if (msm_batt_info.batt_full_check == 1 )
	{
		total_time = TOTAL_RECHARGING_TIME;	// already fully charged... (recharging)
	}
	else
	{
		total_time = TOTAL_CHARGING_TIME;
	}

	if (time_after((unsigned long)jiffies, (unsigned long)(charging_start_time + total_time))) 
	{ 	 
		pr_debug("[BATT] %s: abs time is over !! \n", __func__);
		return 1;
	} 
	else
	{
		return 0;
	}
}

#ifdef MAX17043_FUEL_GAUGE
/* Quick start condition check. */
static struct fuelgauge_linear_data {
	u32 min_vcell;
	u32 slope;
	u32 y_interception;
} qstrt_table[2][12] = {
	{	// w/o charger
		{ 450000000,       0,         0 },
		{ 407900000, 2171993,	193731125 },
		{ 389790000,  847027,	324374902 },
		{ 378060000,  602617,	343245771 },
		{ 372020000,  293109,	361124348 },
		{ 368220000,  209554,	364231282 },
		{ 362530000,  596997,	356856383 },
		{ 359070000,  604297,	356792124 },
		{ 354500000, 2679480,	348980813 },
		{ 344820000, 6365513, 341425848 },
		{ 339970000, 9109696, 339974670 },
		{ 100000000,       0,         0 }
	},

	{	// w/charger
		{ 450000000,       0,         0 },
		{ 419270000,   12835,	406438276 },
		{ 418480000,   73645,	349402654 },
		{ 404650000,   45824,	370277069 },
		{ 392800000,   20460,	382744637 },
		{ 387510000,   51008,	375639409 },
		{ 377390000,  298446,	367071455 },
		{ 373320000,  630069,	360004053 },
		{ 363720000, 1231165,	356301531 },
		{ 100000000,       0,         0 },
		{ 100000000,       0,         0 },
		{ 100000000,       0,         0 }
	},
};

#define FG_SOC_TOLERANCE	20	// 15

static int check_quick_start(void)
{
	unsigned int vcell_raw = 0;
	int soc_raw = 0, soc_cal = 0;
	int i, curr_idx = 0;
	int status = 0;
	int array_size = 0;


	if (msm_batt_get_charger_type() == CHARGER_TYPE_NONE)
	{
		status = 0;
		array_size = 12;
		pr_debug("[BATT] %s: No charger !\n", __func__);
	}
	else
	{
		status = 1;
		array_size = 10;
		pr_debug("[BATT] %s: charger detected !\n", __func__);
	}

	/* get vcell. */
// hyekoseon.yu
	vcell_raw = fg_read_raw_vcell();



	/* get soc. */
	soc_raw = fg_read_raw_soc(); 



	/* find range */
	for (i = 0; i < array_size-1; i++) {
		if (vcell_raw < qstrt_table[status][i].min_vcell &&
				vcell_raw >= qstrt_table[status][i+1].min_vcell) {
			curr_idx = i+1;
			break;
		}
	}

	pr_debug("[BATT] %s: curr_idx = %d (vol=%d)\n", __func__, curr_idx, qstrt_table[status][curr_idx].min_vcell);

	/* calculate assumed soc and compare */
	if ( (status == 0 && curr_idx > 0 && curr_idx < 11) ||
		(status == 1 && curr_idx > 0 && curr_idx < 9) ) {
		int limit_min, limit_max;
		soc_cal = (int) ((vcell_raw - qstrt_table[status][curr_idx].y_interception) / qstrt_table[status][curr_idx].slope);

		pr_debug("[BATT] %s: soc_cal = %d\n", __func__, soc_cal);

		limit_min = soc_cal - FG_SOC_TOLERANCE;
		limit_max = soc_cal + FG_SOC_TOLERANCE;
		if (limit_min < 0)
			limit_min = 0;

		if (soc_raw > limit_max || soc_raw < limit_min) {
//			hsusb_chg_vbus_draw_ext(0);
			fg_reset_soc();
			pr_info("\n[BATT] %s: QUICK START (reset_soc)!!! \n\n", __func__);
		}
	}

	return 0;
}
#endif

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;	

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

	if (pdata->avail_chg_sources & AC_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {

		dev_err(&pdev->dev,
			"%s: No external Power supply(AC or USB)"
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;
	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

    if (boot_check_wrong_battery == 1) {
        printk("[BATT] wrong battery detected set battery health Unspecified\n");
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    } else {
        printk("[BATT] battery normal!\n");
    }

	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc =  msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	msm_batt_create_attrs(msm_psy_batt.dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif

	setup_timer(&msm_batt_info.timer, batt_timeover, 0);
	mod_timer(&msm_batt_info.timer, (jiffies + BATT_CHECK_INTERVAL));

	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_wake_lock");

#ifdef CONFIG_WIRELESS_CHARGING
	set_irq_type(IRQ_WC_DETECT, IRQ_TYPE_EDGE_BOTH);

	if (request_threaded_irq(IRQ_WC_DETECT, NULL,
		wc_detect_irq_handler,
		IRQF_ONESHOT|(IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING),
		"WC_DETECT", NULL)) {
		free_irq(IRQ_WC_DETECT, NULL);
		pr_err("BATT: irq handler registration failed....\n");
	}
#endif

#ifdef CONFIG_MAX17043_FUEL_GAUGE

	if (i2c_add_driver(&fg_i2c_driver))
	{
		printk("%s: Can't add max17043 fuel gauge i2c drv\n", __func__);
		pr_err("%s: Can't add max17043 fuel gauge i2c drv\n", __func__);
	}

	//check_quick_start();

	if (is_attached)
	{
		msm_batt_info.batt_capacity = get_level_from_fuelgauge();
		msm_batt_info.battery_voltage = get_voltage_from_fuelgauge();
		// Write value on voltage_now because msm batt loading delayed.
		msm_batt_info.batt_voltage_now = get_voltage_from_fuelgauge();
	}

#ifdef MAX17043_FUEL_GAUGE
	setup_timer(&fg_alert_timer, fg_set_alert_ext, 0);
	fg_alert_handler = fg_set_alert;
#endif

#endif 

	/* sys fs */
	jig_class = class_create(THIS_MODULE, "jig");
	if (IS_ERR(jig_class))
		pr_err("Failed to create class(jig)!\n");

	jig_dev = device_create(jig_class, NULL, 0, NULL, "jig");
	if (IS_ERR(jig_dev))
		pr_err("Failed to create device(jig)!\n");

	if (device_create_file(jig_dev, &dev_attr_jig) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_jig.attr.name); 
	/* sys fs */

	msm_batt_driver_init = 1;

	//Need to check init connect check!
	msm_batt_cable_status_update();

	pr_debug("[BATT] %s : success!\n", __func__);
	
	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	wake_lock_destroy(&vbus_wake_lock);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}

	return 0;
}

static void msm_batt_shutdown(struct platform_device *pdev)
{
#if 0
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
	}
#endif
	del_timer_sync(&msm_batt_info.timer);
}

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.suspend = msm_batt_suspend,
	.resume = msm_batt_resume,		
	.remove = __devexit_p(msm_batt_remove),
	.shutdown = msm_batt_shutdown,
	.driver = {
		   .name = "ancora-battery",
		   .owner = THIS_MODULE,
		   },
};

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	if (board_hw_revision < 0x06)
	{
		msm_batt_info.msm_batt_wq =
				create_singlethread_workqueue("msm_battery");	
		if (!msm_batt_info.msm_batt_wq) {
			printk(KERN_ERR "%s: create workque failed \n", __func__);
			return -ENOMEM;
		}
	}

	msm_batt_info.chg_ep =
		msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VER_4_1, 0);
	msm_batt_info.chg_api_version =  CHG_RPC_VER_4_1;
	if (msm_batt_info.chg_ep == NULL) {
		pr_err("%s: rpc connect CHG_RPC_PROG = NULL\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_3_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_3_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_3, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_3;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_2_2, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_2_2;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: FAIL: rpc connect for CHG_RPC_PROG. rc=%d\n",
		       __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	/* Get the real 1.x version */
	if (msm_batt_info.chg_api_version == CHG_RPC_VER_1_1)
		msm_batt_info.chg_api_version =
			msm_batt_get_charger_api_version();

	/* Fall back to 1.1 for default */
	if (msm_batt_info.chg_api_version < 0)
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;
	msm_batt_info.batt_api_version =  BATTERY_RPC_VER_4_1;

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
					BATTERY_RPC_VER_4_1,
					1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
		       __func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_2_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_2_1;
	}

	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_5_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);

		return rc;
}

static int __init msm_batt_init(void)
{
//	printk("[SSAM] %s called. version : 0x%x \n", __func__, board_hw_revision);
	
	if (board_hw_revision >= 0x06)
	{		
		msm_batt_info.msm_batt_wq =
			create_singlethread_workqueue("msm_battery");	
		if (!msm_batt_info.msm_batt_wq) {
			printk(KERN_ERR "%s: create workque failed \n", __func__);
			return -ENOMEM;
		}

//		printk("[SSAM] Enter delayed work. \n");
		
		schedule_delayed_work(&msm_batt_work_init, msecs_to_jiffies(5000));
		p_batt_init = &msm_batt_work_init;
	}
	else
	{
		int rc;
		
		rc = msm_batt_init_rpc();
		
		if (rc < 0) {
			pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		
		pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
			__func__, msm_batt_info.chg_api_version,
			msm_batt_info.batt_api_version);
	}
		
	//Check jig status
	if(fsa9480_get_jig_status())
		batt_jig_on_status = 1;
	else
		batt_jig_on_status = 0;	
	
	return 0;
}

static void __exit msm_batt_exit(void)
{
	
	platform_driver_unregister(&msm_batt_driver);
}

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:ancora_battery");

