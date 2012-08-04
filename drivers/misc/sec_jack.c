/*
 *  headset/ear-jack device detection driver.
 *
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/sec_jack.h>
#include <linux/gpio.h>

#define MODULE_NAME "sec_jack:"
#define MAX_ZONE_LIMIT		10
#define SEND_KEY_CHECK_TIME_MS	120		/* 120ms */
#define DET_CHECK_TIME_MS	100			/* 100ms */
#define WAKE_LOCK_TIME		(HZ * 5)	/* 5 sec */
#define SUPPORT_PBA

#define MSM_GPIO_SHORT_SENDEND	44

#ifdef SUPPORT_PBA
struct class *jack_class;
EXPORT_SYMBOL(jack_class);

/* Sysfs device, this is used for communication with Cal App. */
static struct device *jack_selector_fs;
EXPORT_SYMBOL(jack_selector_fs);
#endif

extern int on_call_flag;

struct sec_jack_info {
	struct sec_jack_platform_data *pdata;
	struct delayed_work jack_detect_work;
	struct input_dev *input;
	struct wake_lock det_wake_lock;
	struct sec_jack_zone *zone;
#ifdef JACK_REMOTE_KEY
	int pressed_code;
#endif
	bool send_key_pressed;
	bool send_key_irq_enabled;
	unsigned int cur_jack_type;

};

/* sysfs name HeadsetObserver.java looks for to track headset state
 */
struct switch_dev switch_jack_detection = {
	.name = "h2w",
};

/* To support samsung factory test */
struct switch_dev switch_sendend = {
	.name = "send_end",
};

static void set_send_key_state(struct sec_jack_info *hi, int state)
{
#ifdef JACK_REMOTE_KEY
	struct sec_jack_platform_data *pdata = hi->pdata;
	struct sec_jack_buttons_zone *btn_zones = pdata->buttons_zones;
	int adc;
	int i;

	/* when button is released */
	if (state == 0) {
		input_report_key(hi->input, hi->pressed_code, 0);
		switch_set_state(&switch_sendend, 0);
		input_sync(hi->input);
		pr_info("%s: keycode=%d, is released\n", __func__,
			hi->pressed_code);
      	hi->send_key_pressed = state;
		return;
	}

	/* when button is pressed */
	adc = pdata->get_adc_value();

	pr_err("[HSS][%s]: adc=%d\n", __func__, adc);
      
	msleep(40);
  
	if(!pdata->get_send_key_state() || !pdata->get_det_jack_state())
	{
		pr_warn("%s: key is skipped. ADC value is %d, send_key_state is %d, jack_state is %d.\n", __func__, adc, pdata->get_send_key_state(), pdata->get_det_jack_state());
		hi->send_key_pressed = 0;
     	
		return;
 	}

	for (i = 0; i < pdata->num_buttons_zones; i++)
		if (adc >= btn_zones[i].adc_low &&
		    adc <= btn_zones[i].adc_high) {
			hi->pressed_code = btn_zones[i].code;
			input_report_key(hi->input, btn_zones[i].code, 1);
			switch_set_state(&switch_sendend, 1);
			input_sync(hi->input);
			pr_info("%s: keycode=%d, is pressed\n", __func__,
				btn_zones[i].code);
            	hi->send_key_pressed = state;
			return;
		}

	pr_warn("%s: key is skipped. ADC value is %d.\n", __func__, adc);
     	hi->send_key_pressed = 0;

    return;
#else   
	input_report_key(hi->input, KEY_MEDIA, state);
	input_sync(hi->input);
	switch_set_state(&switch_sendend, state);
	hi->send_key_pressed = state;
#endif   
}

static void sec_jack_set_type(struct sec_jack_info *hi, int jack_type)
{
	struct sec_jack_platform_data *pdata = hi->pdata;

	/* this can happen during slow inserts where we think we identified
	 * the type but then we get another interrupt and do it again
	 */
	if (jack_type == hi->cur_jack_type)
	{
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)// [HSS] Fix Ancora HW Issue      
		if (jack_type  != SEC_HEADSET_4POLE)
			pdata->set_micbias_state(false);
#endif      
		return;
	}

	if (jack_type == SEC_HEADSET_4POLE) {
		/* for a 4 pole headset, enable irq
		   for detecting send/end key presses */
		if (!hi->send_key_irq_enabled) {
			enable_irq(pdata->send_int);
			enable_irq_wake(pdata->send_int);
			hi->send_key_irq_enabled = 1;
		}
	} else {
		/* for all other jacks, disable send/end irq */
		if (hi->send_key_irq_enabled) {
			disable_irq(pdata->send_int);
			disable_irq_wake(pdata->send_int);
			hi->send_key_irq_enabled = 0;
		}
		if (hi->send_key_pressed) {
			set_send_key_state(hi, 0);
			pr_info(MODULE_NAME "%s : BTN set released by jack switch to %d\n",
				__func__, jack_type);
		}
	}

	pr_info(MODULE_NAME "%s : jack_type = %d\n", __func__, jack_type);
	/* prevent suspend to allow user space to respond to switch */
	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);

	hi->cur_jack_type = jack_type;
	switch_set_state(&switch_jack_detection, jack_type);
	
	/* micbias is left enabled for 4pole and disabled otherwise */
	pdata->set_micbias_state(hi->send_key_irq_enabled);
}

static void handle_jack_not_inserted(struct sec_jack_info *hi)
{
	sec_jack_set_type(hi, SEC_JACK_NO_DEVICE);
	/* [HSS] ESD Defect : do not turn off mic bias in call */
	if(!on_call_flag)
		hi->pdata->set_micbias_state(false);
}

static void determine_jack_type(struct sec_jack_info *hi)
{
	struct sec_jack_zone *zones = hi->pdata->zones;
	int size = hi->pdata->num_zones;
	int count[MAX_ZONE_LIMIT] = {0};
	int adc;
	int i;

	while (hi->pdata->get_det_jack_state()) {
		adc = hi->pdata->get_adc_value();
		pr_err("[%s]: adc=%d\n", __func__, adc);

		/* determine the type of headset based on the
		 * adc value.  An adc value can fall in various
		 * ranges or zones.  Within some ranges, the type
		 * can be returned immediately.  Within others, the
		 * value is considered unstable and we need to sample
		 * a few more types (up to the limit determined by
		 * the range) before we return the type for that range.
		 */
		for (i = 0; i < size; i++) {
			if (adc <= zones[i].adc_high) {
				if (++count[i] > zones[i].check_count) {
					sec_jack_set_type(hi,
							  zones[i].jack_type);
						return;
				}
				msleep(zones[i].delay_ms);
				break;
			}
		}
	}
	/* jack removed before detection complete */
	handle_jack_not_inserted(hi);
}

/* thread run whenever the headset detect state changes (either insertion
 * or removal).
 */
static irqreturn_t sec_jack_detect_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = DET_CHECK_TIME_MS;

	/* debounce headset jack.  don't try to determine the type of
	 * headset until the detect state is true for a while.
	 */
	while (time_left_ms > 0) {
		if (!pdata->get_det_jack_state()) {
			/* jack not detected. */
			handle_jack_not_inserted(hi);
			return IRQ_HANDLED;
		}
		msleep(10);
		time_left_ms -= 10;
	}
	/* set mic bias to enable adc */
	pdata->set_micbias_state(true);

	/* to reduce noise in earjack when attaching */
	msleep(200);

	/* jack presence was detected the whole time, figure out which type */
	determine_jack_type(hi);
	return IRQ_HANDLED;
}

#ifdef SUPPORT_PBA
static ssize_t select_jack_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);

	return 0;
}       
static ssize_t select_jack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	int value = 0;


	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);
	if (value == SEC_HEADSET_4POLE) {
		pdata->set_micbias_state(true);
		msleep(100);
	}

	sec_jack_set_type(hi, value);

	return size;
}
static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IXOTH, select_jack_show, select_jack_store);
#endif

/* thread run whenever the send/end key state changes. irq thread
 * handles don't need wake locks and since this one reports using
 * input_dev, input_dev guarantees that user space gets event
 * without needing a wake_lock.
 */
static irqreturn_t sec_jack_send_key_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = SEND_KEY_CHECK_TIME_MS;
	int send_key_state=0;

	/* debounce send/end key */
	while (time_left_ms > 0 && !hi->send_key_pressed) {
		send_key_state = pdata->get_send_key_state();
		if (!send_key_state || !pdata->get_det_jack_state() ||
		    hi->cur_jack_type != SEC_HEADSET_4POLE) {
			/* button released or jack removed or more
			 * strangely a non-4pole headset
			 */
			pr_info(MODULE_NAME "%s : ignored button (%d %d %d)\n", __func__,
				!send_key_state, !pdata->get_det_jack_state(),
				hi->cur_jack_type != SEC_HEADSET_4POLE );
			return IRQ_HANDLED;
		}
		msleep(10);
		time_left_ms -= 10;
	}

	/* report state change of the send_end_key */
	if (hi->send_key_pressed != send_key_state) {
		set_send_key_state(hi, send_key_state);
		pr_info(MODULE_NAME "%s : BTN is %s.\n",
				__func__, send_key_state ? "pressed" : "released");
	}
	return IRQ_HANDLED;
}

static int sec_jack_probe(struct platform_device *pdev)
{
	struct sec_jack_info *hi;
	struct sec_jack_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	pr_info(MODULE_NAME "%s : Registering jack driver\n", __func__);
	if (!pdata) {
		pr_err("%s : pdata is NULL.\n", __func__);
		return -ENODEV;
	}
	if (!pdata->get_adc_value || !pdata->get_det_jack_state	||
	    !pdata->get_send_key_state || !pdata->zones ||
	    !pdata->set_micbias_state || pdata->num_zones > MAX_ZONE_LIMIT) {
		pr_err("%s : need to check pdata\n", __func__);
		return -ENODEV;
	}
#ifdef GET_JACK_ADC
   pdata->rpc_init();
#endif

	hi = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (hi == NULL) {
		pr_err("%s : Failed to allocate memory.\n", __func__);
		return -ENOMEM;
	}

	hi->pdata = pdata;
	hi->input = input_allocate_device();
	if (hi->input == NULL) {
		ret = -ENOMEM;
		pr_err("%s : Failed to allocate input device.\n", __func__);
		goto err_request_input_dev;
	}

	hi->input->name = "sec_jack";
	input_set_capability(hi->input, EV_KEY, KEY_MEDIA);
#ifdef JACK_REMOTE_KEY
	input_set_capability(hi->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(hi->input, EV_KEY, KEY_VOLUMEDOWN);
#endif
	ret = input_register_device(hi->input);
	if (ret) {
		pr_err("%s : Failed to register driver\n", __func__);
		goto err_register_input_dev;
	}

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}

	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}

	wake_lock_init(&hi->det_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack_det");

#ifdef SUPPORT_PBA
	/* Create JACK Device file in Sysfs */
	jack_class = class_create(THIS_MODULE, "h2w");
	if(IS_ERR(jack_class))
	{
		printk(KERN_ERR "Failed to create class(sec_jack)\n");
	}

	jack_selector_fs = device_create(jack_class, NULL, 0, hi, "jack_selector");
	if (IS_ERR(jack_selector_fs))
		printk(KERN_ERR "Failed to create device(sec_jack)!= %ld\n", IS_ERR(jack_selector_fs));

	if (device_create_file(jack_selector_fs, &dev_attr_select_jack) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_select_jack.attr.name);
#endif
	ret = request_threaded_irq(pdata->det_int, NULL,
				   sec_jack_detect_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT, "sec_headset_detect", hi);
	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);
		goto err_request_detect_irq;
	}

	/* to handle insert/removal when we're sleeping in a call */
	ret = enable_irq_wake(pdata->det_int);
	if (ret) {
		pr_err("%s : Failed to enable_irq_wake.\n", __func__);
		goto err_enable_irq_wake;
	}

	ret = request_threaded_irq(pdata->send_int, NULL,
				   sec_jack_send_key_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT,
				   "sec_headset_send_key", hi);
	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);

		goto err_request_send_key_irq;
	}

	/* start with send/end interrupt disable. we only enable it
	 * when we detect a 4 pole headset
	 */
	disable_irq(pdata->send_int);
	dev_set_drvdata(&pdev->dev, hi);

#if (defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE))
	/* call irq_thread forcely because of missing interrupt when booting. */
	if(pdata->get_det_jack_state())
	{
		sec_jack_detect_irq_thread(pdata->det_int, hi);
	}
#endif
	
	return 0;

err_request_send_key_irq:
	disable_irq_wake(pdata->det_int);
err_enable_irq_wake:
	free_irq(pdata->det_int, hi);
err_request_detect_irq:
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_sendend);
err_switch_dev_register:
	input_unregister_device(hi->input);
	goto err_request_input_dev;
err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
	kfree(hi);

	return ret;
}

static int sec_jack_remove(struct platform_device *pdev)
{

	struct sec_jack_info *hi = dev_get_drvdata(&pdev->dev);

	pr_info(MODULE_NAME "%s :\n", __func__);
	/* rebalance before free */
	if (hi->send_key_irq_enabled)
		disable_irq_wake(hi->pdata->send_int);
	else
		enable_irq(hi->pdata->send_int);
	free_irq(hi->pdata->send_int, hi);
	disable_irq_wake(hi->pdata->det_int);
	free_irq(hi->pdata->det_int, hi);
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_sendend);
	input_unregister_device(hi->input);
	kfree(hi);

	return 0;
}

static int sec_jack_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sec_jack_info *hi = dev_get_drvdata(&pdev->dev);
	struct sec_jack_platform_data *pdata = hi->pdata;

	printk("[HSS][%s] det status = %d\n", __func__, (hi->pdata->get_det_jack_state() ));

	if( (hi->pdata->get_det_jack_state() ) )
	{
		gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SHORT_SENDEND , 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	}
	else
	{
		gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SHORT_SENDEND , 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	}
	return 0;
}

static int sec_jack_resume(struct platform_device *pdev)
{
	printk("[HSS][%s] \n", __func__);
   
	gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SHORT_SENDEND , 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);     
   
	return 0;
}

static struct platform_driver sec_jack_driver = {
	.probe = sec_jack_probe,
	.remove = sec_jack_remove,
	.suspend = sec_jack_suspend,
	.resume = sec_jack_resume,
	.driver = {
			.name = "sec_jack",
			.owner = THIS_MODULE,
		   },
};
static int __init sec_jack_init(void)
{
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
	platform_driver_unregister(&sec_jack_driver);
}

module_init(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("ms17.kim@samsung.com");
MODULE_DESCRIPTION("Samsung Electronics Corp Ear-Jack detection driver");
MODULE_LICENSE("GPL");
