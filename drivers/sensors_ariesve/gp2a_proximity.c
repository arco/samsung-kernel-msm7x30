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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/mfd/pmic8058.h>
#include <linux/gp2a.h>
#include <linux/slab.h>


#define gprintk(fmt, x...) \
	printk(KERN_ERR"%s(%d): " fmt, __func__, __LINE__, ## x)


#define PROX_MODE_B

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

static int opt_i2c_init(void);
static struct i2c_driver opt_i2c_driver;

/* driver data */
struct opt_gp2a_platform_data {
    int gp2a_irq;
    int gp2a_gpio;
};

/* global var */

static struct i2c_client *opt_i2c_client;

struct gp2a_data {
	struct input_dev *input_dev;
	struct work_struct work;  /* for proximity sensor */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct class *proximity_class;
	struct device *proximity_dev;

	int	wakeup;
	/* configure the button as a wake-up source */
	int   enabled;
	int   delay;
	int   prox_data;
	int   irq;

	spinlock_t prox_lock;
	struct kobject *uevent_kobj;

    int gp2a_irq;
    int gp2a_gpio;
};


struct opt_state {
	struct i2c_client *client;
};


/* initial value for sensor register */
static u8 gp2a_original_image[8] = {
#ifdef PROX_MODE_A
	0x00,
	0x08,
	0xC2,
	0x04,
	0x01,
#else
	0x00,
	0x08,
	0x40,
	0x04,
	0x03,
#endif
};

#ifdef PROX_MODE_B
/*INT clear bit*/
#define INT_CLEAR    (1 << 7)

static inline u8 INT_CLR(u8 reg)
{
	return reg | INT_CLEAR;
}

static inline u8 NOT_INT_CLR(u8 reg)
{
	return reg & ~(INT_CLEAR);
}
#endif

static int proximity_onoff(u8 onoff);


/* Proximity Sysfs interface */
static ssize_t
proximity_delay_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->delay);
}

static ssize_t
proximity_delay_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int delay = simple_strtoul(buf, NULL, 10);

	if (delay < 0)
		return count;

	if (SENSOR_MAX_DELAY < delay)
		delay = SENSOR_MAX_DELAY;

	data->delay = delay;

	input_report_abs(input_data, ABS_CONTROL_REPORT,
		(data->enabled<<16) | delay);

	return count;
}

static ssize_t
proximity_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->enabled);
}

static ssize_t
proximity_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);
	char input;
	unsigned long flags;

	if (value != 0 && value != 1)
		return count;

	/* Proximity power off */
	if (data->enabled && !value) {
		disable_irq(data->irq);

		proximity_onoff(0);
		disable_irq_wake(data->gp2a_irq);
	}
	/* proximity power on */
	if (!data->enabled && value) {
		proximity_onoff(1);

		input = 0x01;
		opt_i2c_write((u8)(REGS_OPMOD), &input);

		msleep(50);
		enable_irq_wake(data->gp2a_irq);
		input = gpio_get_value_cansleep(data->gp2a_gpio);
		input_report_abs(data->input_dev, ABS_DISTANCE,  input);

		input_sync(data->input_dev);
		gprintk("[PROX] Start proximity = %d\n", input);
		spin_lock_irqsave(&data->prox_lock, flags);
		input = 0x03;
		opt_i2c_write((u8)(REGS_OPMOD), &input);

		enable_irq(data->gp2a_irq);

		spin_unlock_irqrestore(&data->prox_lock, flags);
	}

	data->enabled = value;
	input_report_abs(input_data, ABS_CONTROL_REPORT,
		(value<<16) | data->delay);
	return count;
}

static ssize_t proximity_wake_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

static ssize_t
proximity_data_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{

	struct gp2a_data *data = dev_get_drvdata(dev);

	int x;

	mutex_lock(&data->data_mutex);
	x = data->prox_data;
	mutex_unlock(&data->data_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", x);
}

static ssize_t proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 proximity_value = 0;

	pr_err("[PROX] %s(%d)\n", __func__, __LINE__);
	proximity_data_show(dev, attr, buf);

	if (*buf == '1')
		proximity_value = snprintf(buf, PAGE_SIZE, "1.0\n");
	else
		proximity_value = snprintf(buf, PAGE_SIZE, "0.0\n");

	pr_err("[PROX] result=%s\n", buf);
	return proximity_value;
}

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "11,12,13\n");
}

static ssize_t proximity_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return proximity_enable_store(dev, attr, buf, size);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
	proximity_delay_show, proximity_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
	proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(wake, S_IRUGO|S_IWUSR, NULL, proximity_wake_store);
static DEVICE_ATTR(data, S_IRUGO|S_IWUSR, proximity_data_show, NULL);
static DEVICE_ATTR(avg, S_IRUGO|S_IWUSR,
	proximity_avg_show, proximity_avg_store);
static DEVICE_ATTR(state, S_IRUGO|S_IWUSR, proximity_state_show, NULL);

static struct attribute *proximity_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_state.attr,
	&dev_attr_avg.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
    .attrs = proximity_attributes
};

static char get_ps_vout_value(int gp2a_gpio)
{
  char value = 0;

#ifdef PROX_MODE_A
  value = gpio_get_value_cansleep(GPIO_PS_VOUT);
#else
  opt_i2c_read(0x00, &value, 2);
  value &= 0x01;
  value ^= 0x01;
#endif

  return value;
}

irqreturn_t gp2a_irq_handler(int irq, void *ptr)
{
	struct gp2a_data *gp2a = ptr;
	char value;
#ifdef PROX_MODE_B
	u8 reg = 0;
#endif

	value = get_ps_vout_value(gp2a->gp2a_gpio);

	input_report_abs(gp2a->input_dev, ABS_DISTANCE,  value);
	input_sync(gp2a->input_dev);

	gp2a->prox_data = value;

#ifdef PROX_MODE_B
	if (value == 1)
		reg = 0x40;
	else
		reg = 0x20;

	opt_i2c_write(NOT_INT_CLR(REGS_HYS), &reg);

	reg = 0x18;
	opt_i2c_write(NOT_INT_CLR(REGS_CON), &reg);
#endif

#ifdef PROX_MODE_B
	reg = 0x00;
	opt_i2c_write(INT_CLR(REGS_CON), &reg);
#endif


	return IRQ_HANDLED;
}

int opt_i2c_read(u8 reg, u8 *val, unsigned int len )
{

	int err;
	u8 buf[2];
	struct i2c_msg msg[2];

	buf[0] = reg;

	msg[0].addr = opt_i2c_client->addr;
	msg[0].flags = 1;

	msg[0].len = len;
	msg[0].buf = buf;
	err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

	*val = buf[1];

	gprintk(": 0x%x, 0x%x\n", reg, *val);
	if (err >= 0)
		return 0;

	printk(KERN_ERR "%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

int opt_i2c_write( u8 reg, u8 *val )
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 10;

	if ((opt_i2c_client == NULL) || (!opt_i2c_client->adapter))
		return -ENODEV;

	while (retry--) {
		data[0] = reg;
		data[1] = *val;

		msg->addr = opt_i2c_client->addr;
		msg->flags = I2C_M_WR;
		msg->len = 2;
		msg->buf = data;

		err = i2c_transfer(opt_i2c_client->adapter, msg, 1);
		gprintk(": 0x%x, 0x%x\n", reg, *val);

		if (err >= 0)
			return 0;
	}
	printk(KERN_ERR "%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

void gp2a_chip_init(void)
{
  /* Power On */
  gprintk("\n");
}

static int proximity_input_init(struct gp2a_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_capability(dev, EV_ABS, ABS_STATUS);
	/* status */
	input_set_capability(dev, EV_ABS, ABS_WAKE);
	/* wake */
	input_set_capability(dev, EV_ABS, ABS_CONTROL_REPORT);
	/* enabled/delay */
	input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);


	dev->name = "proximity_sensor";
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input_dev = dev;

	return 0;
}

static int gp2a_opt_probe(struct platform_device *pdev)
{
	struct gp2a_data *gp2a;
	struct opt_gp2a_platform_data *pdata = pdev->dev.platform_data;
	u8 value;
	int err = 0;
	int wakeup = 0;

	/* allocate driver_data */
	gp2a = (struct gp2a_data*) kzalloc(sizeof(struct gp2a_data), GFP_KERNEL);
	if (!gp2a) {
		pr_err("kzalloc error\n");
		return -ENOMEM;
	}

	gp2a->enabled = 0;
	gp2a->delay = SENSOR_DEFAULT_DELAY;


	if (pdata) {
		if (pdata->gp2a_irq)
			gp2a->gp2a_irq = pdata->gp2a_irq;
		if (pdata->gp2a_gpio)
			gp2a->gp2a_gpio = pdata->gp2a_gpio;
	}

	mutex_init(&gp2a->enable_mutex);
	mutex_init(&gp2a->data_mutex);

	wakeup = 1;

	err = proximity_input_init(gp2a);
	if (err < 0)
		goto error_setup_reg;

	err = sysfs_create_group(&gp2a->input_dev->dev.kobj,
				&proximity_attribute_group);
	if (err < 0)
		goto err_sysfs_create_group_proximity;


	/* set platdata */
	platform_set_drvdata(pdev, gp2a);

	gp2a->uevent_kobj = &pdev->dev.kobj;

	spin_lock_init(&gp2a->prox_lock);
	/* init i2c */
	opt_i2c_init();

	if (opt_i2c_client == NULL) {
		pr_err("opt_probe failed : i2c_client is NULL\n");
		err = -ENODEV;
		goto err_i2c_add;
	} else
		printk(KERN_ERR "opt_i2c_client : (0x%p)\n", opt_i2c_client);


	/* GP2A Regs INIT SETTINGS */
#ifdef PROX_MODE_A
	value = 0x00;
#else
	value = 0x02;
#endif
	opt_i2c_write((u8)(REGS_OPMOD), &value);

	/* INT Settings */
	err = request_threaded_irq(gp2a->gp2a_irq,
		NULL, gp2a_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"proximity_int", gp2a);

	if (err < 0) {
		printk(KERN_ERR "failed to request proximity_irq\n");
		goto err_request_irq;
	}

	disable_irq(gp2a->gp2a_irq);

	/* set sysfs for proximity sensor */
	gp2a->proximity_class = class_create(THIS_MODULE, "proximity");
	if (IS_ERR(gp2a->proximity_class)) {
		pr_err("%s: could not create proximity_class\n", __func__);
		err = PTR_ERR(gp2a->proximity_class);
		goto err_proximity_class_create;
	}

	gp2a->proximity_dev = device_create(gp2a->proximity_class,
						NULL, 0, NULL, "proximity");
	if (IS_ERR(gp2a->proximity_dev)) {
		pr_err("%s: could not create proximity_dev\n", __func__);
		err = PTR_ERR(gp2a->proximity_dev);
		goto err_proximity_device_create;
	}

	if ((err = device_create_file(gp2a->proximity_dev,
		&dev_attr_state)) < 0) {
		pr_err("%s: could not create device file(%s)!\n",__func__,
			dev_attr_state.attr.name);
		goto err_proximity_device_create_file1;
	}

	if ((err = device_create_file(gp2a->proximity_dev,
		&dev_attr_avg)) < 0) {
		pr_err("%s: could not create device file(%s)!\n", __func__,
			dev_attr_avg.attr.name);
		goto err_proximity_device_create_file2;
	}
	dev_set_drvdata(gp2a->proximity_dev, gp2a);

	device_init_wakeup(&pdev->dev, wakeup);
	return 0;

err_proximity_device_create_file2:
	device_remove_file(gp2a->proximity_dev, &dev_attr_state);
err_proximity_device_create_file1:
	device_destroy(gp2a->proximity_class, 0);
err_proximity_device_create:
	class_destroy(gp2a->proximity_class);
err_proximity_class_create:
	free_irq(gp2a->gp2a_irq, gp2a);
err_request_irq:
	i2c_del_driver(&opt_i2c_driver);
err_i2c_add:
	sysfs_remove_group(&gp2a->input_dev->dev.kobj,
			   &proximity_attribute_group);
err_sysfs_create_group_proximity:
	input_unregister_device(gp2a->input_dev);
	input_free_device(gp2a->input_dev);
error_setup_reg:
	destroy_work_on_stack(&gp2a->work);
	mutex_destroy(&gp2a->enable_mutex);
	mutex_destroy(&gp2a->data_mutex);

	kfree(gp2a);
	return err;
}

static int gp2a_opt_remove(struct platform_device *pdev)
{
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);

	if (gp2a->proximity_class != NULL) {
		device_remove_file(gp2a->proximity_dev, &dev_attr_avg);
		device_remove_file(gp2a->proximity_dev, &dev_attr_state);
		device_destroy(gp2a->proximity_class, 0);
		class_destroy(gp2a->proximity_class);
	}

	disable_irq(gp2a->gp2a_irq);
	free_irq(gp2a->gp2a_irq, gp2a);

	if (gp2a->input_dev != NULL) {
		sysfs_remove_group(&gp2a->input_dev->dev.kobj,
			&proximity_attribute_group);
		input_unregister_device(gp2a->input_dev);
		if (gp2a->input_dev != NULL)
			kfree(gp2a->input_dev);
	}

	mutex_destroy(&gp2a->enable_mutex);
	mutex_destroy(&gp2a->data_mutex);

	device_init_wakeup(&pdev->dev, 0);

	kfree(gp2a);

	return 0;
}

static int gp2a_opt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);

	if (gp2a->enabled) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(gp2a->gp2a_irq);

	}

	return 0;
}

static int gp2a_opt_resume(struct platform_device *pdev)
{
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);

	if (gp2a->enabled) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(gp2a->gp2a_irq);

	}

	return 0;
}

static int proximity_onoff(u8 onoff)
{
	u8 value;
	int i;

	if (onoff) {
		for (i = 1 ; i < 5 ; i++)
			opt_i2c_write((u8)(i), &gp2a_original_image[i]);
	} else {
#ifdef PROX_MODE_A
		value = 0x00;
#else
		value = 0x02;
#endif
		opt_i2c_write((u8)(REGS_OPMOD), &value);
	}

	return 0;
}
static int opt_i2c_remove(struct i2c_client *client)
{
	struct opt_state *data = i2c_get_clientdata(client);

	kfree(data);
	opt_i2c_client = NULL;

	return 0;
}

static int opt_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct opt_state *opt;

	gprintk("\n");
	pr_err("%s, %d : start!!!\n", __func__, __LINE__);

	opt = kzalloc(sizeof(struct opt_state), GFP_KERNEL);
	if (opt == NULL) {
		printk(KERN_ERR "failed to allocate memory\n");
		pr_err("%s, %d : error!!!\n", __func__, __LINE__);
		return -ENOMEM;
	}

	opt->client = client;
	i2c_set_clientdata(client, opt);

	/* rest of the initialisation goes here. */

	pr_err("GP2A opt i2c attach success!!!\n");

	opt_i2c_client = client;
	pr_err("%s, %d : end!!!\n", __func__, __LINE__);

	return 0;
}


static const struct i2c_device_id opt_device_id[] = {
	{"gp2a", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, opt_device_id);

static struct i2c_driver opt_i2c_driver = {
	.driver = {
		.name = "gp2a",
		.owner = THIS_MODULE,
	},
	.probe		= opt_i2c_probe,
	.remove		= opt_i2c_remove,
	.id_table	= opt_device_id,
};


static struct platform_driver gp2a_opt_driver = {
	.probe = gp2a_opt_probe,
	.remove = gp2a_opt_remove,
	.suspend = gp2a_opt_suspend,
	.resume  = gp2a_opt_resume,
	.driver  = {
		.name = "gp2a-opt",
		.owner = THIS_MODULE,
	},
};

static int opt_i2c_init(void)
{
	if (i2c_add_driver(&opt_i2c_driver)) {
		printk(KERN_ERR "i2c_add_driver failed\n");
		return -ENODEV;
	}
	return 0;
}

static int __init gp2a_opt_init(void)
{
	int ret;

	ret = platform_driver_register(&gp2a_opt_driver);
	return ret;
}

static void __exit gp2a_opt_exit(void)
{
	platform_driver_unregister(&gp2a_opt_driver);
}

module_init(gp2a_opt_init);
module_exit(gp2a_opt_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP002A00F");
MODULE_LICENSE("GPL");
