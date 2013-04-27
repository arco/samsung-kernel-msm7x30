/*
 * Copyright (c) 2010 SAMSUNG
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/gp2a.h>
#include <mach/vreg.h>
#include <linux/slab.h>
#include <linux/gpio.h>


/* for debugging */
#define DEBUG 0

/*********** for debug **********************************************************/
#if 1 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/


#ifdef CONFIG_MACH_ARIESVE
#define SENSOR_NAME "light"
#else
#define SENSOR_NAME "light_sensor"
#endif

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
#define MSM_LIGHTSENSOR_ADC_READ
#endif
//#define PMIC_GPIO_PROX_EN	15 /* PMIC GPIO 16 */
/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

struct sensor_data {
    struct mutex mutex;
	struct delayed_work work;

    int enabled;
    int delay;

	state_type light_data;
#if DEBUG
    int suspend;
#endif
    int testmode;
};

#ifdef MSM_LIGHTSENSOR_ADC_READ
extern int __devinit msm_lightsensor_init_rpc(void);
extern u32 lightsensor_get_adc(void);
extern void msm_lightsensor_cleanup(void);
#endif

/* global var */
static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

static state_type cur_state = LIGHT_INIT;
static int cur_adc_value = 0;

static int buffering = 2;
static int adc_value_buf[ADC_BUFFER_NUM] = {0};

int autobrightness_mode = OFF;

static int StateToLux(state_type state);
static int AdcToLux(int adc_value);

//extern int backlight_level;
#ifdef CONFIG_FB_S3C_MDNIE_TUNINGMODE_FOR_BACKLIGHT
int pre_val = -1;
extern int current_gamma_value;
extern u16 *pmDNIe_Gamma_set[];

typedef enum
{
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE,
	mDNIe_VIDEO_COLD_MODE,
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI
}Lcd_mDNIe_UI;

extern Lcd_mDNIe_UI current_mDNIe_UI;

extern void mDNIe_Mode_set_for_backlight(u16 *buf);

int value_buf[4] = {0};


int IsChangedADC(int val)
{
	int j = 0;
	int ret = 0;

	int adc_index = 0;
	static int adc_index_count = 0;


	adc_index = (adc_index_count)%4;		
	adc_index_count++;

	if(pre_val == -1) //ADC buffer initialize 
	{
		for(j = 0; j<4; j++)
			value_buf[j] = val;

		pre_val = 0;
	}
    else
    {
    	value_buf[adc_index] = val;
	}

	ret = ((value_buf[0] == value_buf[1])&& \
			(value_buf[1] == value_buf[2])&& \
			(value_buf[2] == value_buf[3]))? 1 : 0;

	
	if(adc_index_count == 4)
		adc_index_count = 0;

	return ret;
}
#endif

static int StateToLux(state_type state)
{
	int lux = 0;

	if(state== LIGHT_LEVEL5){
		lux = 15000;
	}
	else if(state == LIGHT_LEVEL4){
		lux = 9000;
	}
	else if(state == LIGHT_LEVEL3){
		lux = 5000;
	}
	else if(state == LIGHT_LEVEL2){
		lux = 1000;
	}
	else if(state == LIGHT_LEVEL1){
		lux = 6;
	}

	else {
		lux = 5000;
	}
	return lux;
}

static int AdcToLux(int adc_value)
{
  int lux=0;
  adc_value *= 1000;
  if (adc_value < 25000)
    lux = 0;
  else if(adc_value >= 25000 && adc_value < 91000)
    lux = ((adc_value-25000)/66)*15;
  else if(adc_value >= 91000 && adc_value < 457000)
    lux = ((adc_value-91000)/366)*135+15000;
  else if(adc_value >= 457000 && adc_value < 1156000)
    lux = (((adc_value-457000)/699)*1350)+150000;
  else if(adc_value >= 1156000)
    lux = (((adc_value-1156000)/454)*13500)+1500000;

  return lux/1000;  
}

/* Light Sysfs interface */
static ssize_t
light_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay;

    delay = data->delay;

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
light_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay = simple_strtoul(buf, NULL, 10);

    if (delay < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < delay) {
        delay = SENSOR_MAX_DELAY;
    }

    data->delay = delay;

	mutex_lock(&data->mutex);

	if( data->enabled)
	{
		cancel_delayed_work_sync(&data->work);
		schedule_delayed_work(&data->work, msecs_to_jiffies(delay));
	}

    input_report_abs(input_data, ABS_CONTROL_REPORT, (data->delay<<16) | delay);

	mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
light_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int enabled;

    enabled = data->enabled;

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
light_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

    if (value != 0 && value != 1) {
        return count;
    }

    mutex_lock(&data->mutex);

    if (data->enabled && !value) {
		cancel_delayed_work_sync(&data->work);
        gprintk("timer canceled.\n");
    }
    if (!data->enabled && value) {
		schedule_delayed_work(&data->work, 0);
        gprintk("timer started.\n");
    }
    data->enabled= value;

        printk("[HSS] [%s] enable = %d\n", __FUNCTION__, value);

    input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | data->delay);

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
light_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;
 
    input_report_abs(input_data, ABS_WAKE, cnt++);

    return count;
}

static ssize_t
light_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
//    unsigned long flags;
    int light_lux;

	mutex_lock(&data->mutex);
	light_lux = AdcToLux(data->light_data);
	mutex_unlock(&data->mutex);	

	return sprintf(buf, "%d\n", light_lux);
}

static ssize_t
light_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

    printk(KERN_INFO "%s : cur_state(%d)\n", __func__, cur_state);

    return sprintf(buf, "%d\n", cur_state);
}

static ssize_t light_autobrightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

	gprintk("called %s \n",__func__);

	int adc = 0;
	int sum = 0;
	int i = 0;
	if(data->enabled)
	{
		for(i = 0; i < 10; i++)
		{
			adc = lightsensor_get_adcvalue();
			msleep(20);
			sum += adc;
		}
		adc = sum/10;
		gprintk("called %s  - subdued alarm(adc : %d)\n",__func__,adc);
		return sprintf(buf,"%d\n",adc);
	}
	else
	{
		gprintk("called %s  - *#0589#\n",__func__);
	    return sprintf(buf,"%d\n",cur_adc_value);
	}
}

static ssize_t light_autobrightness_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{

    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
	int value;
    sscanf(buf, "%d", &value);

	gprintk("called %s \n",__func__);
	

	if(value==1)
	{
		autobrightness_mode = ON;
		printk(KERN_DEBUG "[brightness_mode] BRIGHTNESS_MODE_SENSOR\n");
	}
	else if(value==0) 
	{
		autobrightness_mode = OFF;
		printk(KERN_DEBUG "[brightness_mode] BRIGHTNESS_MODE_USER\n");
#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
		if(pre_val==1)
		{
			mDNIe_Mode_set_for_backlight(pmDNIe_Gamma_set[2]);
		}	
		pre_val = -1;
#endif
	}

	return size;
}

static ssize_t light_testmode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{

    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
	int value;
    sscanf(buf, "%d", &value);


	if(value==1)
	{
		data->testmode= 1;
		gprintk("lightsensor testmode ON.\n");
	}
	else if(value==0) 
	{
		data->testmode  = 0;
		gprintk("lightsensor testmode OFF.\n");
	}

	return size;
}

static ssize_t light_testmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

    int testmode;

	testmode = data->testmode;

  	gprintk(" : %d \n", testmode);

	return sprintf(buf, "%d\n", testmode);
}


static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP, light_delay_show, light_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, light_enable_show, light_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, light_wake_store);
static DEVICE_ATTR(data, S_IRUGO, light_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, light_status_show, NULL);
static DEVICE_ATTR(autobrightness, S_IRUGO|S_IWUSR|S_IWGRP, light_autobrightness_show, light_autobrightness_store);
static DEVICE_ATTR(testmode, S_IRUGO|S_IWUSR|S_IWGRP, light_testmode_show, light_testmode_store);


static struct attribute *lightsensor_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    &dev_attr_autobrightness.attr,
    &dev_attr_testmode.attr,
    NULL
};

static struct attribute_group lightsensor_attribute_group = {
    .attrs = lightsensor_attributes
};

static int
lightsensor_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct sensor_data *data = input_get_drvdata(this_data);
    struct vreg* vreg_L11;
    int rt = 0;

    mutex_lock(&data->mutex);

    if (data->enabled) {
        rt = cancel_delayed_work_sync(&data->work);
        gprintk(": The timer is cancled.\n");
    }

    mutex_unlock(&data->mutex);


    /*[HSS] Use Prox_EN*/   
     //gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_PROX_EN), 0);	

    gprintk("[HSS] PROX_EN is OFF\n");

    return rt;
}

static int
lightsensor_resume(struct platform_device *pdev)
{
    struct sensor_data *data = input_get_drvdata(this_data);
    struct vreg* vreg_L11;
    int rt = 0;

    mutex_lock(&data->mutex);

    if (data->enabled) {
        rt = schedule_delayed_work(&data->work, 0);
        gprintk(": The timer is started.\n");
    }

    mutex_unlock(&data->mutex);

      /*[HSS] Use Prox_EN*/   
     //gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_PROX_EN), 1);	

    gprintk("[HSS] PROX_EN is ON\n");

    return rt;
}

int lightsensor_get_adcvalue(void)
{
	int i = 0;
	int j = 0;
	unsigned int adc_total = 0;
	static int adc_avr_value = 0;
	unsigned int adc_index = 0;
	static unsigned int adc_index_count = 0;
	unsigned int adc_max = 0;
	unsigned int adc_min = 0;
	int value =0;

	//get ADC
#ifdef MSM_LIGHTSENSOR_ADC_READ	
	value = lightsensor_get_adc();
#endif
	cur_adc_value = value;
	
	adc_index = (adc_index_count++)%ADC_BUFFER_NUM;		

	if(cur_state == LIGHT_INIT) //ADC buffer initialize (light sensor off ---> light sensor on)
	{
		for(j = 0; j<ADC_BUFFER_NUM; j++)
			adc_value_buf[j] = value;
	}
    else
    {
    	adc_value_buf[adc_index] = value;
	}
	
	adc_max = adc_value_buf[0];
	adc_min = adc_value_buf[0];

	for(i = 0; i <ADC_BUFFER_NUM; i++)
	{
		adc_total += adc_value_buf[i];

		if(adc_max < adc_value_buf[i])
			adc_max = adc_value_buf[i];
					
		if(adc_min > adc_value_buf[i])
			adc_min = adc_value_buf[i];
	}
	adc_avr_value = (adc_total-(adc_max+adc_min))/(ADC_BUFFER_NUM-2);
	
	if(adc_index_count == ADC_BUFFER_NUM-1)
		adc_index_count = 0;

	return adc_avr_value;
}

static void gp2a_work_func_light(struct work_struct *work)
{
	struct sensor_data *data = container_of((struct delayed_work *)work,
							struct sensor_data, work);
	
	int adc=0;
	state_type level_state = LIGHT_INIT;
    int lux = 0;
    static int old_lux = 0;

	/* read adc data from s5p110 */
	adc = lightsensor_get_adcvalue();

	if(adc >= 1643)//(adc >= 1050)
	{
		level_state = LIGHT_LEVEL5;
		buffering = 5;
	}

	else if(adc >= 1576 && adc < 1643)//(adc >= 800 && adc < 1050)
	{
		if(buffering == 5)
		{	
			level_state = LIGHT_LEVEL5;
			buffering = 5;
		}
		else if((buffering == 1)||(buffering == 2)||(buffering == 3)||(buffering == 4))
		{
			level_state = LIGHT_LEVEL4;
			buffering = 4;
		}
	}

	else if(adc >= 1159 && adc < 1576)//(adc >= 750 && adc < 800)
	{
		level_state = LIGHT_LEVEL4;
		buffering = 4;
	}

	else if(adc >= 1104 && adc < 1159)//(adc >= 560 && adc < 750)
	{
		if((buffering == 4)||(buffering == 5))
		{	
			level_state = LIGHT_LEVEL4;
			buffering = 4;
		}
		else if((buffering == 1)||(buffering == 2)||(buffering == 3))
		{
			level_state = LIGHT_LEVEL3;
			buffering = 3;
		}
	}
	
	else if(adc >= 462 && adc < 1104)//(adc >= 550 && adc < 560)
	{
		level_state = LIGHT_LEVEL3;
		buffering = 3;
	}

	else if(adc >= 430 && adc < 462)//(adc >= 370 && adc < 550)
	{
		if((buffering == 3)||(buffering == 4)||(buffering == 5))
		{	
			level_state = LIGHT_LEVEL3;
			buffering = 3;
		}
		else if((buffering == 1)||(buffering == 2))
		{
			level_state = LIGHT_LEVEL2;
			buffering = 2;
		}
	}

	else if(adc >= 94 && adc < 430)//(adc >= 270 && adc < 370)
	{
		level_state = LIGHT_LEVEL2;
		buffering = 2;
	}
	
	else if(adc >= 86 && adc < 94)//(adc >= 200 && adc < 270)
	{
		if((buffering == 2)||(buffering == 3)||(buffering == 4)||(buffering == 5))
		{	
			level_state = LIGHT_LEVEL2;
			buffering = 2;
		}
		else if(buffering == 1)
		{
			level_state = LIGHT_LEVEL1;
			buffering = 1;
		}
	}

	else if(adc < 86)//(adc < 200)
	{
		level_state = LIGHT_LEVEL1;
		buffering = 1;
	}

    cur_state = level_state;	

#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
	if(autobrightness_mode)
	{
		if((pre_val!=1)&&(current_gamma_value == 24)&&(level_state == LIGHT_LEVEL5)&&(current_mDNIe_UI == mDNIe_UI_MODE))
			{
			mDNIe_Mode_set_for_backlight(pmDNIe_Gamma_set[1]);
			pre_val = 1;
			gprintk("mDNIe_Mode_set_for_backlight - pmDNIe_Gamma_set[1]\n" );
		}	
	}
#endif

    lux = StateToLux(cur_state);

	data->light_data= cur_adc_value;

    if(data->testmode == 1)
    {
        input_report_abs(this_data, ABS_Y, adc);
    	input_sync(this_data);
    	//gprintk("testmode : adc(%d), lux(%d), cur_state(%d) \n", adc, lux, cur_state);
    }
    else
    {
      if(old_lux != lux)
      {
        old_lux = lux;
      	input_report_abs(this_data, ABS_X, lux);
    	input_sync(this_data);
        gprintk("realmode : adc(%d), lux(%d), cur_state(%d) \n", adc, lux, cur_state);
      }
    }

	schedule_delayed_work(&data->work, msecs_to_jiffies(data->delay));
}

#ifdef MSM_LIGHTSENSOR_ADC_READ  
void lightsensor_rpc_init(void)
{
  /* RPC initial sequence */
  int err = 1;

  gprintk("\n");

  err = msm_lightsensor_init_rpc();

  if (err < 0) {
    pr_err("%s: FAIL: msm_lightsensor_init_rpc.  err=%d\n", __func__, err);
    msm_lightsensor_cleanup();
  }
}
#endif

static int
lightsensor_probe(struct platform_device *pdev)
{
    struct sensor_data *data = NULL;
    struct input_dev *input_data = NULL;
    int input_registered = 0, sysfs_created = 0;
    int rt;

	gprintk("\n");

    data = (struct sensor_data *) kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
    if (!data) {
        rt = -ENOMEM;
        goto err;
    }
    data->enabled = 0;
    data->delay = SENSOR_DEFAULT_DELAY;
    data->testmode = 0;

	INIT_DELAYED_WORK(&data->work, gp2a_work_func_light);

    input_data = input_allocate_device();
    if (!input_data) {
        rt = -ENOMEM;
        printk(KERN_ERR
               "sensor_probe: Failed to allocate input_data device\n");
        goto err;
    }

    set_bit(EV_ABS, input_data->evbit);
    input_set_capability(input_data, EV_ABS, ABS_X);
    input_set_capability(input_data, EV_ABS, ABS_Y); 
    input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
    input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */

	input_set_abs_params(input_data, ABS_X, 0, 1, 0, 0);
	input_set_abs_params(input_data, ABS_Y, 0, 1, 0, 0);
	input_set_abs_params(input_data, ABS_WAKE, 0, (1<<31), 0, 0);
	input_set_abs_params(input_data, ABS_CONTROL_REPORT, 0, 1<<16, 0, 0);

    input_data->name = SENSOR_NAME;

    rt = input_register_device(input_data);
    if (rt) {
        printk(KERN_ERR
               "sensor_probe: Unable to register input_data device: %s\n",
               input_data->name);
        goto err;
    }
    input_set_drvdata(input_data, data);
    input_registered = 1;

    rt = sysfs_create_group(&input_data->dev.kobj,
            &lightsensor_attribute_group);
    if (rt) {
        printk(KERN_ERR
               "sensor_probe: sysfs_create_group failed[%s]\n",
               input_data->name);
        goto err;
    }
    sysfs_created = 1;
    mutex_init(&data->mutex);
    this_data = input_data;

#ifdef MSM_LIGHTSENSOR_ADC_READ  
    lightsensor_rpc_init();
#endif

    return 0;

err:
    if (data != NULL) {
        if (input_data != NULL) {
            if (sysfs_created) {
                sysfs_remove_group(&input_data->dev.kobj,
                        &lightsensor_attribute_group);
            }
            if (input_registered) {
                input_unregister_device(input_data);
            }
            else {
                input_free_device(input_data);
            }
            input_data = NULL;
        }
        kfree((void*) data);
    }

    return rt;
}

static int
lightsensor_remove(struct platform_device *pdev)
{
    struct sensor_data *data;

    if (this_data != NULL) {
        data = input_get_drvdata(this_data);
        sysfs_remove_group(&this_data->dev.kobj,
                &lightsensor_attribute_group);
        if (data != NULL) {
          cancel_delayed_work(&data->work);
          flush_scheduled_work();
          //kfree(data);
        }
        input_unregister_device(this_data);
    }

    //gprintk("\n");

    return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver lightsensor_driver = {
    .probe      = lightsensor_probe,
    .remove     = lightsensor_remove,
    .suspend    = lightsensor_suspend,
    .resume     = lightsensor_resume,
    .shutdown = lightsensor_remove,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    },
};

static int __init lightsensor_init(void)
{
    sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
    if (IS_ERR(sensor_pdev)) {
        return -1;
    }
    return platform_driver_register(&lightsensor_driver);
}
module_init(lightsensor_init);

static void __exit lightsensor_exit(void)
{
    platform_driver_unregister(&lightsensor_driver);
    platform_device_unregister(sensor_pdev);
}
module_exit(lightsensor_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP002A00F");
MODULE_LICENSE("GPL");
