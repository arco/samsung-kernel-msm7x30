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
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/msm_adc.h>

#include <mach/vreg.h>
#include <linux/workqueue.h>


/* for debugging */
#define DEBUG 0

/*********** for debug **********************************************************/
#if 1 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/


#define LIGHT_DEVICE_NAME             "light_sensor"
#define LIGHT_INPUT_NAME              "light_sensor"

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

#define REALMODE_BUFFER		/* use realmode buffer */

#ifdef REALMODE_BUFFER
#define LIGHT_BUFFER_NUM	14
#else
#define LIGHT_BUFFER_UP	5
#define LIGHT_BUFFER_DOWN	5
#endif

#define LIGHT_ADC_FUZZ 1

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
#define MSM_LIGHTSENSOR_ADC_READ
#endif

#define SENSOR_ZEROVALUE_FIX	/* probably for some interference with light of display, the value of the sensor does not come to zero, in case of total darkness */
//#define SENSOR_ENABLE_TESTMODE_ABS_Y

struct sensor_data {
	struct mutex mutex;
	struct delayed_work work;
	struct class *lightsensor_class;
	struct device *switch_cmd_dev;

	int enabled;
	int delay;

	state_type light_data;
#if DEBUG
	int suspend;
#endif
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
	int testmode;
#endif
	int light_buffer;
	int light_count;
#ifndef REALMODE_BUFFER
	int light_level_state;
	bool light_first_level;
#endif
#ifdef SENSOR_ZEROVALUE_FIX
	int zerovalue_fix_last_adc;
	int zerovalue_fix_light_count;
	int zerovalue_fix_light_shift;
	bool zerovalue_fix_enabled;
#endif
};

static const int adc_table[4] = {
	86,
	430,
	1104,
	1643,
};


#ifdef MSM_LIGHTSENSOR_ADC_READ
extern int __devinit msm_lightsensor_init_rpc(void);
extern u32 lightsensor_get_adc(void);
extern void msm_lightsensor_cleanup(void);
#endif

struct workqueue_struct *light_workqueue;

/* global var */
static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

static state_type cur_state = LIGHT_INIT;
static int cur_adc_value = 0;

static int adc_value_buf[ADC_BUFFER_NUM] = {0};

int autobrightness_mode = OFF;
int LightSensor_Log_Cnt =0;

//static int StateToLux(state_type state);

//extern int backlight_level;
#ifdef CONFIG_FB_S3C_MDNIE_TUNINGMODE_FOR_BACKLIGHT
static int pre_val = -1;
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

	if(pre_val == -1) { //ADC buffer initialize
		for(j = 0; j<4; j++)
			value_buf[j] = val;

		pre_val = 0;
	} else {
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

/*
static int StateToLux(state_type state)
{
	int lux = 0;

	if(state== LIGHT_LEVEL5) {
		lux = 15000;
	} else if(state == LIGHT_LEVEL4) {
		lux = 9000;
	} else if(state == LIGHT_LEVEL3) {
		lux = 5000;
	} else if(state == LIGHT_LEVEL2) {
		lux = 1000;
	} else if(state == LIGHT_LEVEL1) {
		lux = 6;
	} else {
		lux = 5000;
	}

	return lux;
}
*/

static ssize_t lightsensor_file_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int adc = 0;

	if (data->enabled) {
		adc = lightsensor_get_adcvalue();
		printk("%s : adc(%d)\n", __func__, adc);
	} else {
		adc = 0;
		printk("%s : lightsensor disabled\n", __func__);
	}

	return sprintf(buf, "%d\n", adc);
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

	#if 0
	int delay = simple_strtoul(buf, NULL, 10);

	if (delay < 0) {
		return count;
	}
	#else

	int delay;
	int err = 0;

	err = kstrtoint(buf, 10, &delay);
	if (err)
		pr_err("%s, kstrtoint failed.", __func__);

	if (delay < 0) {
		return count;
	}

	delay = delay / 1000000;	//ns to msec
	#endif
	
	pr_info("%s, new_delay = %d, old_delay = %d", __func__, delay,
	       data->delay);


	if (SENSOR_MAX_DELAY < delay) {
		delay = SENSOR_MAX_DELAY;
	}

	data->delay = delay;

	mutex_lock(&data->mutex);

	if( data->enabled)
	{
		cancel_delayed_work_sync(&data->work);
		queue_delayed_work(light_workqueue,&data->work,msecs_to_jiffies(delay));
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
		data->enabled= value; // sync with gp2a_work_func_light function
		cancel_delayed_work_sync(&data->work);
		gprintk("timer canceled.\n");
	}
	if (!data->enabled && value) {
		data->enabled= value; // sync with gp2a_work_func_light function
		queue_delayed_work(light_workqueue,&data->work,0);
		gprintk("timer started.\n");
	}

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

static ssize_t lightsensor_raw_data_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int adc = 0;

	adc = lightsensor_get_adcvalue();

	return sprintf(buf, "%d\n", adc);
}

static ssize_t
light_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	//struct input_dev *input_data = to_input_dev(dev);
	//struct sensor_data *data = input_get_drvdata(input_data);

	printk(KERN_INFO "%s : cur_state(%d)\n", __func__, cur_state);

	return sprintf(buf, "%d\n", cur_state);
}

static ssize_t light_autobrightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);

	int adc = 0;
	int sum = 0;
	int i = 0;

	gprintk("called %s \n", __func__);

	if(data->enabled) {
		for(i = 0; i < 10; i++) {
			adc = lightsensor_get_adcvalue();
			msleep(20);
			sum += adc;
		}
		adc = sum/10;
		return sprintf(buf,"%d\n", adc);
	} else {
		return sprintf(buf,"%d\n", cur_adc_value);
	}
}

static ssize_t light_autobrightness_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	gprintk("called %s \n", __func__);

	if(value == 1) {
		autobrightness_mode = ON;
		printk(KERN_DEBUG "[brightness_mode] BRIGHTNESS_MODE_SENSOR\n");
	} else if(value == 0) {
		autobrightness_mode = OFF;
		printk(KERN_DEBUG "[brightness_mode] BRIGHTNESS_MODE_USER\n");
#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
		if(pre_val == 1) {
			mDNIe_Mode_set_for_backlight(pmDNIe_Gamma_set[2]);
		}
		pre_val = -1;
#endif
	}

	return size;
}

#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
static ssize_t light_testmode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int value;

	sscanf(buf, "%d", &value);

	if(value == 1) {
		data->testmode = 1;
		gprintk("lightsensor testmode ON.\n");
	} else if(value == 0) {
		data->testmode = 0;
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
#endif

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP, light_delay_show, light_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, light_enable_show, light_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, light_wake_store);
static DEVICE_ATTR(raw_data, S_IRUGO, lightsensor_raw_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, light_status_show, NULL);
static DEVICE_ATTR(autobrightness, S_IRUGO|S_IWUSR|S_IWGRP, light_autobrightness_show, light_autobrightness_store);
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
static DEVICE_ATTR(testmode, S_IRUGO|S_IWUSR|S_IWGRP, light_testmode_show, light_testmode_store);
#endif
static DEVICE_ATTR(lightsensor_file_state, 0644, lightsensor_file_state_show, NULL);

static struct attribute *lightsensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_status.attr,
	&dev_attr_autobrightness.attr,
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
	&dev_attr_testmode.attr,
#endif
	&dev_attr_lightsensor_file_state.attr,
	NULL
};

static struct attribute_group lightsensor_attribute_group = {
	.attrs = lightsensor_attributes
};

static int
lightsensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sensor_data *data = input_get_drvdata(this_data);

	int rt = 0;

	mutex_lock(&data->mutex);

	if (data->enabled) {
		rt = cancel_delayed_work_sync(&data->work);
		gprintk(": The timer is cancled.\n");
	}

	mutex_unlock(&data->mutex);

	return rt;
}

static int
lightsensor_resume(struct platform_device *pdev)
{
	struct sensor_data *data = input_get_drvdata(this_data);
	int rt = 0;
	data->light_count = 0;
	data->light_buffer = 0;
#ifndef REALMODE_BUFFER
	data->light_first_level = true;
#endif
#ifdef SENSOR_ZEROVALUE_FIX
	data->zerovalue_fix_last_adc = 0;
	data->zerovalue_fix_light_count = 0;
	data->zerovalue_fix_enabled = false;
#endif
	mutex_lock(&data->mutex);

	if (data->enabled) {
		rt = queue_delayed_work(light_workqueue,&data->work,0);
		gprintk(": The timer is started.\n");
	}

	mutex_unlock(&data->mutex);

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

	if(cur_state == LIGHT_INIT) { //ADC buffer initialize (light sensor off ---> light sensor on)
		for(j = 0; j < ADC_BUFFER_NUM; j++)
			adc_value_buf[j] = value;
	} else {
		adc_value_buf[adc_index] = value;
	}

	adc_max = adc_value_buf[0];
	adc_min = adc_value_buf[0];

	for(i = 0; i < ADC_BUFFER_NUM; i++) {
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

	int i;
	int adc = 0;

	adc = lightsensor_get_adcvalue();

#ifdef SENSOR_ZEROVALUE_FIX
	if (data->zerovalue_fix_last_adc == adc && adc <= 26) {
		if (data->zerovalue_fix_light_count++ == (LIGHT_BUFFER_NUM * 2) && !data->zerovalue_fix_enabled) {
			data->zerovalue_fix_light_shift = adc;
			data->zerovalue_fix_enabled = true;
			data->zerovalue_fix_light_count = 0;
		}
	} else {
		if (adc > 26)
			data->zerovalue_fix_enabled = false;
	}
	
	if (data->zerovalue_fix_enabled) {
		adc = data->zerovalue_fix_light_shift;
		
		if (data->zerovalue_fix_light_shift > 0)
			data->zerovalue_fix_light_shift = data->zerovalue_fix_light_shift - 1;
	}

	data->zerovalue_fix_last_adc = adc;
#endif

	for (i = 0; ARRAY_SIZE(adc_table); i++)
		if (adc <= adc_table[i])
			break;

	if (data->light_buffer == i) {

#ifdef REALMODE_BUFFER
		if (data->light_count++ == LIGHT_BUFFER_NUM) {

#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
			if(data->testmode == 1) {
				input_report_abs(this_data, ABS_Y, adc);
				input_sync(this_data);
				data->light_count = 0;
				gprintk("[LIGHT SENSOR] testmode : adc(%d), cur_state(%d)\n", adc, data->light_data);
			} else {
#endif
				input_report_abs(this_data, ABS_MISC, adc);
				input_sync(this_data);
				data->light_count = 0;
				//gprintk("[LIGHT SENSOR] realmode : adc(%d)\n", adc);
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
			}
#endif
		}
#else
		if(data->light_level_state <= i || data->light_first_level == true){
			if (data->light_count++ == LIGHT_BUFFER_UP) {
                if (LightSensor_Log_Cnt == 10) {
                    printk("[LIGHT SENSOR] lux up 0x%0X (%d)\n", adc, adc);
                    LightSensor_Log_Cnt = 0;
                }

                LightSensor_Log_Cnt = LightSensor_Log_Cnt + 1;       
				input_report_abs(this_data,	ABS_MISC, adc);
				input_sync(this_data);
				data->light_count = 0;
				data->light_first_level = false;
				data->light_level_state = data->light_buffer;
			}
		} else {
			if (data->light_count++ == LIGHT_BUFFER_DOWN) {
                if (LightSensor_Log_Cnt == 10) {
                    printk("[LIGHT SENSOR] lux down 0x%0X (%d)\n", adc, adc);
                    LightSensor_Log_Cnt = 0;
				}

                LightSensor_Log_Cnt = LightSensor_Log_Cnt + 1;         
				input_report_abs(this_data,	ABS_MISC, adc);
				input_sync(this_data);
				data->light_count = 0;
				data->light_level_state = data->light_buffer;
			}
		}
#endif
	} else {
		data->light_buffer = i;
		data->light_count = 0;
	}

#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
	if(data->testmode == 1)
		data->light_data = adc;
#endif

	if(data->enabled)
		queue_delayed_work(light_workqueue,&data->work, msecs_to_jiffies(data->delay));
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

	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
	data->testmode = 1;
#endif
#ifndef REALMODE_BUFFER
	data->light_level_state = 0;
#endif

	light_workqueue = create_singlethread_workqueue("klightd");
	if (!light_workqueue) {
		rt = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate work queue\n", __func__);
		goto err;
	}

	INIT_DELAYED_WORK(&data->work, gp2a_work_func_light);

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		printk(KERN_ERR
			"sensor_probe: Failed to allocate input_data device\n");
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
    input_set_capability(input_data, EV_ABS, ABS_MISC);
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
    input_set_capability(input_data, EV_ABS, ABS_Y);
#endif
	input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
	input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */
	input_set_abs_params(input_data, ABS_MISC, 0, 1, LIGHT_ADC_FUZZ, 0);
#ifdef SENSOR_ENABLE_TESTMODE_ABS_Y
	input_set_abs_params(input_data, ABS_Y, 0, 1, 0, 0);
#endif
	input_set_abs_params(input_data, ABS_WAKE, 0, (1<<31), 0, 0);
	input_set_abs_params(input_data, ABS_CONTROL_REPORT, 0, 1<<16, 0, 0);
	input_data->name = LIGHT_INPUT_NAME;

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

	data->lightsensor_class = class_create(THIS_MODULE, LIGHT_DEVICE_NAME);
	if (IS_ERR(data->lightsensor_class)) {
		pr_err("%s: could not create lightsensor_class\n", __func__);
		goto err;
	}

	data->switch_cmd_dev = device_create(data->lightsensor_class,
						NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(data->switch_cmd_dev)) {
		pr_err("%s: could not create switch_cmd_dev\n", __func__);
		goto err_light_device_create;
	}

	if (device_create_file(data->switch_cmd_dev,
		&dev_attr_lightsensor_file_state) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_lightsensor_file_state.attr.name);
		goto err_light_device_create_file;
	}
	dev_set_drvdata(data->switch_cmd_dev, data);

	return 0;

err_light_device_create_file:
	device_destroy(data->lightsensor_class, 0);
err_light_device_create:
	class_destroy(data->lightsensor_class);
err:
	if (data != NULL) {
		if (input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&lightsensor_attribute_group);
			}
			if (input_registered)
				input_unregister_device(input_data);
			else
				input_free_device(input_data);
			input_data = NULL;
		}
		kfree(data);
	}
	return rt;
}

static int lightsensor_remove(struct platform_device *pdev)
{
	struct sensor_data *data;
	int rt;

	rt = 0;
	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		if (data != NULL)
			data->enabled = 0;
		sysfs_remove_group(&this_data->dev.kobj,
				&lightsensor_attribute_group);
		if (data != NULL) {
			cancel_delayed_work(&data->work);
			flush_workqueue(light_workqueue);
			destroy_workqueue(light_workqueue);
			kfree(data);
		}
		input_unregister_device(this_data);
	}

	//gprintk("\n");

	return rt;
}

static void
lightsensor_shutdown(struct platform_device *pdev)
{
	struct sensor_data *data;
	int rt;

	rt = 0;
	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		if (data != NULL)
			data->enabled = 0;
		sysfs_remove_group(&this_data->dev.kobj,
				&lightsensor_attribute_group);
		if (data != NULL) {
			cancel_delayed_work(&data->work);
			flush_workqueue(light_workqueue);
			kfree(data);
		}
		input_unregister_device(this_data);
	}

	//gprintk("\n");
}


/*
 * Module init and exit
 */
static struct platform_driver lightsensor_driver = {
	.probe      = lightsensor_probe,
	.remove     = lightsensor_remove,
	.suspend    = lightsensor_suspend,
	.resume     = lightsensor_resume,
	.shutdown   = lightsensor_shutdown,
	.driver = {
		.name   = LIGHT_DEVICE_NAME,
		.owner  = THIS_MODULE,
	},
};


static int __init lightsensor_init(void)
{
	sensor_pdev = platform_device_register_simple(LIGHT_DEVICE_NAME, 0, NULL, 0);
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
