/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

MODULE_LICENSE("GPL");

#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO)  || defined(CONFIG_MACH_APACHE)
#define CAM_FLASH_ENSET 57
#define CAM_FLASH_FLEN 56
#else
#define CAM_FLASH_ENSET 1
#define CAM_FLASH_FLEN 2
#endif

static dev_t ledflash_dev;
static struct cdev ledflash_cdev;
static int ledflash_major = 230, ledflash_minor = 0;
struct class *ledflash_class;

static int ledflash_open(struct inode *inode, struct file *file)
{
	/* Do nothing */
	return 0;
}

static int ledflash_close(struct inode *inode, struct file *file)
{
	/* Do nothing */
	return 0;
}

static ssize_t ledflash_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;  
	int i = 0;
	printk("[HCHYUN] %s\n", __func__);
	/* initailize falsh IC */
	gpio_set_value(CAM_FLASH_ENSET,0); 
	gpio_set_value(CAM_FLASH_FLEN,0);
	mdelay(1); // to enter a shutdown mode
	
	/* set to movie mode */
	for(i=0;i<8;i++)
	{
		udelay(1);
		gpio_set_value(CAM_FLASH_ENSET,1);
		udelay(1);
		gpio_set_value(CAM_FLASH_ENSET,0);
	}
	gpio_set_value(CAM_FLASH_ENSET,1); //value set
	count = sprintf(buf,"ledflash_read called\n");
	
	return count;
}

static ssize_t ledflash_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("[HCHYUN] %s\n", __func__);
	/* initailize falsh IC */
	gpio_set_value(CAM_FLASH_ENSET,0);
	gpio_set_value(CAM_FLASH_FLEN,0);
	mdelay(1); // to enter a shutdown mode
	
	return size;
}

static ssize_t samsung_factorytest_camflash_on (struct file *flip,  char __user *buf, size_t count, loff_t *offp)
{
	int i = 0;
	printk("[HCHYUN] %s\n", __func__);
	/* initailize falsh IC */
	gpio_set_value(CAM_FLASH_ENSET,0); 
	gpio_set_value(CAM_FLASH_FLEN,0);
	mdelay(1); // to enter a shutdown mode
	
	/* set to movie mode */
	for(i=0;i<8;i++)
	{
		udelay(1);
		gpio_set_value(CAM_FLASH_ENSET,1);
		udelay(1);
		gpio_set_value(CAM_FLASH_ENSET,0);
	}
	gpio_set_value(CAM_FLASH_ENSET,1); //value set
	
	return count;
}

static ssize_t samsung_factorytest_camflash_off(struct file *flip, const char __user *buf, size_t count, loff_t *offp)
{
	printk("[HCHYUN] %s\n", __func__);
	/* initailize falsh IC */
	gpio_set_value(CAM_FLASH_ENSET,0);
	gpio_set_value(CAM_FLASH_FLEN,0);
	mdelay(1); // to enter a shutdown mode
	
	return 0;
}

static const struct file_operations ledflash_fops = {
	.owner = THIS_MODULE,
	.read = samsung_factorytest_camflash_off,
	.write = samsung_factorytest_camflash_on,
	.open = ledflash_open,
	.release = ledflash_close,
};

static DEVICE_ATTR(ledflash_file, S_IRUGO, NULL, NULL);

static int ledflash_register_cdev(void)
{
	int err;

	if(ledflash_major){
		ledflash_dev = MKDEV(ledflash_major,ledflash_minor);
		err = register_chrdev_region(ledflash_dev,1,"ledflash");
	}else {
		err = alloc_chrdev_region(&ledflash_dev,ledflash_minor,1,"ledflash");
		ledflash_major =MAJOR(ledflash_dev);
	}

	printk("ledflash : major-%d, minor-%d\n",ledflash_major,ledflash_minor);
	if(err<0) {
		printk("ledflash : cat't get major %d\n",ledflash_major);
		return err;
	}

	cdev_init(&ledflash_cdev,&ledflash_fops);
	ledflash_cdev.owner = THIS_MODULE;
	ledflash_cdev.ops = & ledflash_fops;
	err = cdev_add(&ledflash_cdev,ledflash_dev,1);

	if(err)
		printk("ledflash : led flash register error %d\n",err);
	
	return 0;
}

static int ledflash_init(void)
{
	int err = 0;
	struct device *dev_t;
	
	printk("ledflash_init\n");

	if((err = ledflash_register_cdev()) < 0) {
		return err;
	}
	
	ledflash_class = class_create(THIS_MODULE, "ledflash");
	
	if (IS_ERR(ledflash_class)) 
		return PTR_ERR( ledflash_class );
	
	//dev_t = device_create( ledflash_class, NULL, MKDEV(247, 0), "%s", "ledflash");
	dev_t = device_create( ledflash_class, NULL, ledflash_dev, "%s", "ledflash");
	
	if (device_create_file(dev_t, &dev_attr_ledflash_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_ledflash_file.attr.name);

	if (IS_ERR(dev_t)) {
		return PTR_ERR(dev_t);
	}

	return 0;
}

static void ledflash_exit(void)
{
	printk("ledflash exit\n");
	class_destroy(ledflash_class);
	//cdev_del(&ledflash_cdev);
	//unregister_chrdev_region(ledflash_dev,1);
}

module_init(ledflash_init);
module_exit(ledflash_exit);


