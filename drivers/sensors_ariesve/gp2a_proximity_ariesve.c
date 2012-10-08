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


/*********** for debug **********************************************************/
#if 1 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

#define IRQ_GP2A_INT MSM_GPIO_TO_INT(118)  
#define GPIO_PS_VOUT 118
#define PMIC_GPIO_PROX_EN	15 /* PMIC GPIO 16 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio) (pm_gpio + NR_GPIO_IRQS)
//#define PROX_MODE_A
#define PROX_MODE_B

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

/* sdcard detect */
static unsigned int external_sd_status_read = 0;

/* global var */
static struct wake_lock prx_wake_lock;

static struct i2c_driver opt_i2c_driver;
static struct i2c_client *opt_i2c_client = NULL;

/* driver data */
struct gp2a_data {
	struct input_dev *input_dev;
	struct work_struct work;  /* for proximity sensor */
	struct mutex enable_mutex;
	struct mutex data_mutex;

	int   enabled;
	int   delay;
	int   prox_data;
	int   irq;

  	struct kobject *uevent_kobj;
};


static struct gp2a_data *prox_data = NULL;

struct opt_state{
	struct i2c_client	*client;	
};

struct opt_state *opt_state;

/* initial value for sensor register */
static u8 gp2a_original_image[8] =
{
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
#endif //PROX_MODE_A	
};

#ifdef PROX_MODE_B
/*INT clear bit*/
#define INT_CLEAR    (1 << 7)

static inline u8 INT_CLR(u8 reg)
{
	return (reg | INT_CLEAR);
}

static inline u8 NOT_INT_CLR(u8 reg)
{
	return (reg & ~(INT_CLEAR));
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
    int delay;

    delay = data->delay;

    return sprintf(buf, "%d\n", delay);
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

    if (delay < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < delay) {
        delay = SENSOR_MAX_DELAY;
    }

    data->delay = delay;

    input_report_abs(input_data, ABS_CONTROL_REPORT, (data->enabled<<16) | delay);

    return count;
}

static ssize_t
proximity_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2a_data *data = input_get_drvdata(input_data);
    int enabled;

    enabled = data->enabled;

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
proximity_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2a_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);
    int err = 0;
    
    struct file *fp_sd 	 = NULL;
	mm_segment_t old_fs;
	
	if( external_sd_status_read==0 )
	{
	//For Hardware detected	
	printk(KERN_INFO "SD card Hardware detected\n");
	fp_sd = filp_open( "/persist/sd_det.bin", O_RDWR , 0666);

	if(IS_ERR(fp_sd)||(fp_sd==NULL))
	{
		printk(KERN_ERR "[SDCard] %s: File open error\n", "persist/sd_det.bin");
	}
	else
	{
		
		old_fs = get_fs();
       	set_fs(KERNEL_DS);
        	
		char buffer[2]	 = {1};
		if(fp_sd->f_mode & FMODE_WRITE)
		{
			sprintf(buffer,"0\n");
			printk(KERN_INFO "[SDCard] external_sd_status = 0\n");
			fp_sd->f_op->write(fp_sd, (const char *)buffer, sizeof(buffer), &fp_sd->f_pos);
		}

		if( fp_sd != NULL ) {
 		filp_close(fp_sd, NULL);
 		fp_sd = NULL;
 		}
		external_sd_status_read = 1;
 		set_fs(old_fs);
 	
		}
	}

    gprintk("\n");



    if (value != 0 && value != 1) {
        return count;
    }

    if (data->enabled && !value) { 			/* Proximity power off */

	//register irq to wakeup source
	err = irq_set_irq_wake(IRQ_GP2A_INT, 0);	// enable : 1, disable : 0
	printk("[TAEKS] register wakeup source = %d\n",err);
	if (err) 
		printk("[TAEKS] register wakeup source failed\n");
		
       disable_irq(IRQ_GP2A_INT);
	proximity_onoff(0);
    }
    if (!data->enabled && value) {			/* proximity power on */
	proximity_onoff(1);

	//register irq to wakeup source
	err = irq_set_irq_wake(IRQ_GP2A_INT, 1);	// enable : 1, disable : 0
	printk("[TAEKS] register wakeup source = %d\n",err);
	if (err) 
		printk("[TAEKS] register wakeup source failed\n");
		
        enable_irq(IRQ_GP2A_INT);
    }
    data->enabled = value;

        printk("[HSS] [%s] enable = %d\n", __FUNCTION__, value);

    input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | data->delay);

    return count;
}

static ssize_t
proximity_wake_store(struct device *dev,
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
proximity_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
    int x;

	mutex_lock(&data->data_mutex);
	x = data->prox_data;
	mutex_unlock(&data->data_mutex);
	
    return sprintf(buf, "%d\n", x);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
        proximity_delay_show, proximity_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
        proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
        NULL, proximity_wake_store);
static DEVICE_ATTR(data, S_IRUGO, proximity_data_show, NULL);


static struct attribute *proximity_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    NULL
};

static struct attribute_group proximity_attribute_group = {
    .attrs = proximity_attributes
};


static char get_ps_vout_value(void)
{
  char value = 0;


#ifdef PROX_MODE_A
  value = gpio_get_value_cansleep(GPIO_PS_VOUT);
#else //PROX_MODE_A
  opt_i2c_read(0x00, &value, 2);
  value &= 0x01;
  value ^= 0x01;
#endif //PROX_MODE_A

  return value;
}
/*****************************************************************************************
 *  
 *  function    : gp2a_work_func_prox 
 *  description : This function is for proximity sensor (using B-1 Mode ). 
 *                when INT signal is occured , it gets value from VO register.   
 *
 *                 
 */
static void gp2a_work_func_prox(struct work_struct *work)
{
	struct gp2a_data *gp2a = container_of((struct work_struct *)work,
							struct gp2a_data, work);
	
	char value;
#ifdef PROX_MODE_B
    u8 reg = 0;
#endif

    disable_irq(IRQ_GP2A_INT);

    value = get_ps_vout_value(); 

    input_report_abs(gp2a->input_dev, ABS_X,  value);
    input_sync(gp2a->input_dev);

	gp2a->prox_data= value;
	gprintk("proximity = %d\n",value); //Temp

#ifdef PROX_MODE_B
    if(value == 1) //VO == 0
    {
      reg = 0x40;
      opt_i2c_write(NOT_INT_CLR(REGS_HYS), &reg);
    }
    else
    {
      reg = 0x20;
      opt_i2c_write(NOT_INT_CLR(REGS_HYS), &reg);
    }

    reg = 0x18;
    opt_i2c_write(NOT_INT_CLR(REGS_CON), &reg);
#endif //PROX_MODE_B

    enable_irq(IRQ_GP2A_INT);

#ifdef PROX_MODE_B
    reg = 0x00;
    opt_i2c_write(INT_CLR(REGS_CON), &reg);
#endif //PROX_MODE_B

}

irqreturn_t gp2a_irq_handler(int irq, void *dev_id)
{
    u8 reg = 0;
    
	wake_lock_timeout(&prx_wake_lock, 3*HZ);

    gprintk("\n");

	schedule_work(&prox_data->work);

	printk("[PROXIMITY] IRQ_HANDLED.\n");
	return IRQ_HANDLED;
}

static int opt_i2c_init(void) 
{
	if( i2c_add_driver(&opt_i2c_driver))
	{
		printk("i2c_add_driver failed \n");
		return -ENODEV;
	}
	return 0;
}


int opt_i2c_read(u8 reg, u8 *val, unsigned int len )
{

	int err;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg; 

	msg[0].addr = opt_i2c_client->addr;
	msg[0].flags = 1;
	
	msg[0].len = len;
	msg[0].buf = buf;
	err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

	*val = buf[1];

    gprintk(": 0x%x, 0x%x \n", reg, *val);
    if (err >= 0) return 0;

    printk("%s %d i2c transfer error\n", __func__, __LINE__);
    return err;
}

int opt_i2c_write( u8 reg, u8 *val )
{
    int err = 0;
    struct i2c_msg msg[1];
    unsigned char data[2];
    int retry = 10;

    if( (opt_i2c_client == NULL) || (!opt_i2c_client->adapter) ){
        return -ENODEV;
    }

    while(retry--)
    {
        data[0] = reg;
        data[1] = *val;

        msg->addr = opt_i2c_client->addr;
        msg->flags = I2C_M_WR;
        msg->len = 2;
        msg->buf = data;

        err = i2c_transfer(opt_i2c_client->adapter, msg, 1);
        gprintk(": 0x%x, 0x%x\n", reg, *val);

        if (err >= 0) return 0;
    }
    printk("%s %d i2c transfer error\n", __func__, __LINE__);
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
	if (!dev) {
		return -ENOMEM;
	}

	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_STATUS); /* status */
	input_set_capability(dev, EV_ABS, ABS_WAKE); /* wake */
	input_set_capability(dev, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */
	input_set_abs_params(dev, ABS_X, 0, 1, 0, 0);
	input_set_abs_params(dev, ABS_STATUS, 0, (1<<16), 0, 0);
	input_set_abs_params(dev, ABS_WAKE, 0, (1<<31), 0, 0);
	input_set_abs_params(dev, ABS_CONTROL_REPORT, 0, 1<<16, 0, 0);


	dev->name = "proximity";
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input_dev = dev;
	
	return 0;
}

static int gp2a_opt_probe( struct platform_device* pdev )
{
    struct gp2a_data *gp2a;
    u8 value;
    int err = 0;

	/* allocate driver_data */
	gp2a = (struct gp2a_data*) kzalloc(sizeof(struct gp2a_data),GFP_KERNEL);
	if(!gp2a)
	{
		pr_err("kzalloc error\n");
		return -ENOMEM;

	}

	gp2a->enabled = 0;
	gp2a->delay = SENSOR_DEFAULT_DELAY;

    prox_data = gp2a;

	mutex_init(&gp2a->enable_mutex);
	mutex_init(&gp2a->data_mutex);

	INIT_WORK(&gp2a->work, gp2a_work_func_prox);
	
	err = proximity_input_init(gp2a);
	if(err < 0) {
		goto error_1;
	}

	err = sysfs_create_group(&gp2a->input_dev->dev.kobj,
				&proximity_attribute_group);
	if(err < 0)
	{
		goto error_2;
	}

	/* set platdata */
	platform_set_drvdata(pdev, gp2a);

    gp2a->uevent_kobj = &pdev->dev.kobj;

	/* wake lock init */
	wake_lock_init(&prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");
	
	/* init i2c */
	opt_i2c_init();

	if(opt_i2c_client == NULL)
	{
		pr_err("opt_probe failed : i2c_client is NULL\n"); 
		return -ENODEV;
	}
	else
		printk("opt_i2c_client : (0x%p)\n",opt_i2c_client);
	

	/* GP2A Regs INIT SETTINGS */
#ifdef PROX_MODE_A
	value = 0x00;
#else
	value = 0x02;
#endif //PROX_MODE_A
	opt_i2c_write((u8)(REGS_OPMOD),&value);

	printk("gp2a_opt_probe is OK!!\n");
	
	/* INT Settings */	
  	err = request_threaded_irq( IRQ_GP2A_INT , 
		NULL, gp2a_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING	, "proximity_int", gp2a);

	if (err < 0) {
		printk(KERN_ERR "failed to request proximity_irq\n");
		goto error_2;
	}
    disable_irq(IRQ_GP2A_INT);
	
	return 0;

error_2:
	input_unregister_device(gp2a->input_dev);
	input_free_device(gp2a->input_dev);
error_1:
	kfree((void*)gp2a);
	return err;
}

static int gp2a_opt_remove( struct platform_device* pdev )
{
  struct gp2a_data *gp2a = platform_get_drvdata(pdev);

  if (gp2a->input_dev!= NULL) {
    sysfs_remove_group(&gp2a->input_dev->dev.kobj,
				&proximity_attribute_group);
    input_unregister_device(gp2a->input_dev);
    if (gp2a->input_dev != NULL) {
        kfree(gp2a->input_dev);
    }
  }

  kfree(gp2a);

  return 0;
}

static int gp2a_opt_suspend( struct platform_device* pdev, pm_message_t state )
{
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);
	
	gprintk("\n");

#if 0
	if(gp2a->enabled) // calling
	{

      if (device_may_wakeup(&pdev->dev))
        enable_irq_wake(IRQ_GP2A_INT);

	   gprintk("The timer is cancled.\n");
     	return 0;	   
	}
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_PROX_EN), 0);
#endif

	return 0;
}

static int gp2a_opt_resume( struct platform_device* pdev )
{

	struct gp2a_data *gp2a = platform_get_drvdata(pdev);

	gprintk("\n");

#if 0
	if(gp2a->enabled) //calling
	{
      if (device_may_wakeup(&pdev->dev))
        disable_irq_wake(IRQ_GP2A_INT);

      gprintk("The timer is cancled.\n");
	}
	gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_PROX_EN), 1);
	proximity_onoff(1);  // when proximity 0 , enter suspend and than resume proximity does not work issue

#endif
    return 0;
}

static int proximity_onoff(u8 onoff)
{
	u8 value;
    int i;
       
	if(onoff)
	{
       	for(i=1;i<5;i++)
       	{
       		opt_i2c_write((u8)(i),&gp2a_original_image[i]);
    	}
	}
	else
	{
#ifdef PROX_MODE_A
		value = 0x00;
#else
		value = 0x02;
#endif //PROX_MODE_A	
		opt_i2c_write((u8)(REGS_OPMOD),&value);
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

static int opt_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct opt_state *opt;

    gprintk("\n");
	opt = kzalloc(sizeof(struct opt_state), GFP_KERNEL);
	if (opt == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	opt->client = client;
	i2c_set_clientdata(client, opt);
	
	/* rest of the initialisation goes here. */
	
	printk("GP2A opt i2c attach success!!!\n");

	opt_i2c_client = client;

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
		.owner= THIS_MODULE,
	},
	.probe		= opt_i2c_probe,
	.remove		= opt_i2c_remove,
	.id_table	= opt_device_id,
};


static struct platform_driver gp2a_opt_driver = {
	.probe 	 = gp2a_opt_probe,
    .remove = gp2a_opt_remove,
	.suspend = gp2a_opt_suspend,
	.resume  = gp2a_opt_resume,
	.driver  = {
		.name = "gp2a-opt",
		.owner = THIS_MODULE,
	},
};

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

module_init( gp2a_opt_init );
module_exit( gp2a_opt_exit );

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP002A00F");
MODULE_LICENSE("GPL");

