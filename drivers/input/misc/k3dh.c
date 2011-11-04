/*
 *  STMicroelectronics k3dh acceleration sensor driver
 *
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/i2c/k3dh.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include "k3dh_reg.h"

#define k3dh_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)

/* The default settings when sensor is on is for all 3 axis to be enabled
 * and output data rate set to 400Hz.  Output is via a ioctl read call.
 * The ioctl blocks on data_ready completion.
 * The sensor generates an interrupt when the output is ready and the
 * irq handler atomically sets the completion and wakes any
 * blocked reader.
 */
#define DEFAULT_POWER_ON_SETTING (ODR400 | ENABLE_ALL_AXES)
#define ACC_DEV_NAME "accelerometer"
#define ACC_DEV_MAJOR 103 /* 241 */

static const struct odr_delay {
	u8 odr; /* odr reg setting */
	s64 delay_ns; /* odr in ns */
} odr_delay_table[] = {
	{ ODR1344,     744047LL }, /* 1344Hz */
	{  ODR400,    2500000LL }, /*  400Hz */
	{  ODR200,    5000000LL }, /*  200Hz */
	{  ODR100,   10000000LL }, /*  100Hz */
	{   ODR50,   20000000LL }, /*   50Hz */
	{   ODR25,   40000000LL }, /*   25Hz */
	{   ODR10,  100000000LL }, /*   10Hz */
	{    ODR1, 1000000000LL }, /*    1Hz */
};

/* K3DH acceleration data */
struct k3dh_acc {
	s16 x;
	s16 y;
	s16 z;
};

struct k3dh_data {
	struct i2c_client *client;
	struct miscdevice k3dh_device;
	struct k3dh_platform_data *pdata;
	struct mutex read_lock;
	struct mutex write_lock;
	struct completion data_ready;
	struct class *acc_class;
	int irq;
	u8 ctrl_reg1_shadow;
	atomic_t opened; /* opened implies enabled */
};

static void k3dh_disable_irq(struct k3dh_data *k3dh)
{
	disable_irq(k3dh->irq);
	if (try_wait_for_completion(&k3dh->data_ready)) {
		/* we actually got the interrupt before we could disable it
		 * so we need to enable again to undo our disable and the
		 * one done in the irq_handler
		 */
		enable_irq(k3dh->irq);
	}
}

static irqreturn_t k3dh_irq_handler(int irq, void *data)
{
	struct k3dh_data *k3dh = data;
	disable_irq_nosync(irq);
	complete(&k3dh->data_ready);
	return IRQ_HANDLED;
}

static int k3dh_wait_for_data_ready(struct k3dh_data *k3dh)
{
	int err;

	return 0; /* tempolary block for polling umfa.k3dh*/

	if (gpio_get_value(k3dh->pdata->gpio_acc_int))
		return 0;

	enable_irq(k3dh->irq);

	err = wait_for_completion_timeout(&k3dh->data_ready, 5*HZ);
	if (err > 0)
		return 0;

	k3dh_disable_irq(k3dh);

	if (err == 0) {
		pr_err("k3dh: wait timed out\n");
		return -ETIMEDOUT;
	}

	pr_err("k3dh: wait restart\n");
	return err;
}

/* Read X,Y and Z-axis acceleration data.  Blocks until there is
 * something to read, based on interrupt from chip.
 */
static int k3dh_read_accel_xyz(struct k3dh_data *k3dh,
				struct k3dh_acc *acc)
{
	int err;
	s8 reg = OUT_X_L | AC; /* read from OUT_X_L to OUT_Z_H by auto-inc */
	u8 acc_data[6];

	err = k3dh_wait_for_data_ready(k3dh);
	if (err)
		return err;

	err = i2c_smbus_read_i2c_block_data(k3dh->client, reg,
					    sizeof(acc_data), acc_data);
	if (err != sizeof(acc_data)) {
		pr_err("%s : failed to read 6 bytes for getting x/y/z\n",
		       __func__);
		return -EIO;
	}

	acc->x = (acc_data[1] << 8) | acc_data[0];
	acc->y = (acc_data[3] << 8) | acc_data[2];
	acc->z = (acc_data[5] << 8) | acc_data[4];

	acc->x = acc->x >> 4;
	acc->y = acc->y >> 4;
	acc->z = acc->z >> 4;

	return 0;
}

/*  open command for K3DH device file  */
static int k3dh_open(struct inode *inode, struct file *file)
{
	int err = 0;
	struct k3dh_data *k3dh = container_of(file->private_data,
						struct k3dh_data,
						k3dh_device);

	if (atomic_read(&k3dh->opened) == 0) {
		file->private_data = k3dh;
		k3dh->ctrl_reg1_shadow = DEFAULT_POWER_ON_SETTING;
		err = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1,
						DEFAULT_POWER_ON_SETTING);
		if (err)
			pr_err("k3dh_open() i2c write ctrl_reg1 failed\n");

		err = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG4,
						CTRL_REG4_HR);
		if (err)
			pr_err("k3dh_open() i2c write ctrl_reg4 failed\n");
	}
	atomic_add(1, &k3dh->opened);

	return err;
}

/*  release command for K3DH device file */
static int k3dh_close(struct inode *inode, struct file *file)
{
	int err = 0;
	struct k3dh_data *k3dh = file->private_data;

	atomic_sub(1, &k3dh->opened);
	if (atomic_read(&k3dh->opened) == 0) {
		err = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1,
								PM_OFF);
		k3dh->ctrl_reg1_shadow = PM_OFF;
	}

	return err;
}

static s64 k3dh_get_delay(struct k3dh_data *k3dh)
{
	int i;
	u8 odr;
	s64 delay = -1;

	odr = k3dh->ctrl_reg1_shadow & ODR_MASK;
	for (i = 0; i < ARRAY_SIZE(odr_delay_table); i++) {
		if (odr == odr_delay_table[i].odr) {
			delay = odr_delay_table[i].delay_ns;
			break;
		}
	}
	return delay;
}

static int k3dh_set_delay(struct k3dh_data *k3dh, s64 delay_ns)
{
	int odr_value = ODR1;
	int res = 0;
	int i;
	/* round to the nearest delay that is less than
	 * the requested value (next highest freq)
	 */
	k3dh_dbgmsg(" passed %lldns\n", delay_ns);
	for (i = 0; i < ARRAY_SIZE(odr_delay_table); i++) {
		if (delay_ns < odr_delay_table[i].delay_ns)
			break;
	}
	if (i > 0)
		i--;
	k3dh_dbgmsg("matched rate %lldns, odr = 0x%x\n",
			odr_delay_table[i].delay_ns,
			odr_delay_table[i].odr);
	odr_value = odr_delay_table[i].odr;
	delay_ns = odr_delay_table[i].delay_ns;
	mutex_lock(&k3dh->write_lock);
	k3dh_dbgmsg("old = %lldns, new = %lldns\n",
		     k3dh_get_delay(k3dh), delay_ns);
	if (odr_value != (k3dh->ctrl_reg1_shadow & ODR_MASK)) {
		u8 ctrl = (k3dh->ctrl_reg1_shadow & ~ODR_MASK);
		ctrl |= odr_value;
		k3dh->ctrl_reg1_shadow = ctrl;
		res = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1, ctrl);
		k3dh_dbgmsg("writing odr value 0x%x\n", odr_value);
	}
	mutex_unlock(&k3dh->write_lock);
	return res;
}

/*  ioctl command for K3DH device file */
static int k3dh_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct k3dh_data *k3dh = file->private_data;
	struct k3dh_acc data;
	s64 delay_ns;

	/* cmd mapping */
	switch (cmd) {
	case K3DH_IOCTL_SET_DELAY:
		if (copy_from_user(&delay_ns, (void __user *)arg,
					sizeof(delay_ns)))
			return -EFAULT;
		err = k3dh_set_delay(k3dh, delay_ns);
		break;
	case K3DH_IOCTL_GET_DELAY:
		delay_ns = k3dh_get_delay(k3dh);
		if (put_user(delay_ns, (s64 __user *)arg))
			return -EFAULT;
		break;
	case K3DH_IOCTL_READ_ACCEL_XYZ:
		mutex_lock(&k3dh->read_lock);
		err = k3dh_read_accel_xyz(k3dh, &data);
		if (err)
			break;
		mutex_unlock(&k3dh->read_lock);
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int k3dh_suspend(struct device *dev)
{
	int res = 0;
	struct k3dh_data *k3dh = dev_get_drvdata(dev);

	if (atomic_read(&k3dh->opened) > 0)
		res = i2c_smbus_write_byte_data(k3dh->client,
						CTRL_REG1, PM_OFF);

	return res;
}

static int k3dh_resume(struct device *dev)
{
	int res = 0;
	struct k3dh_data *k3dh = dev_get_drvdata(dev);

	if (atomic_read(&k3dh->opened) > 0)
		res = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1,
						k3dh->ctrl_reg1_shadow);

	return res;
}


static const struct dev_pm_ops k3dh_pm_ops = {
	.suspend = k3dh_suspend,
	.resume = k3dh_resume,
};

static const struct file_operations k3dh_fops = {
	.owner = THIS_MODULE,
	.open = k3dh_open,
	.release = k3dh_close,
	.ioctl = k3dh_ioctl,
};

static int k3dh_setup_irq(struct k3dh_data *k3dh)
{
	int rc = -EIO;
	struct k3dh_platform_data *pdata = k3dh->pdata;
	int irq;

	rc = gpio_request(pdata->gpio_acc_int, "gpio_acc_int");
	if (rc < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, pdata->gpio_acc_int, rc);
		return rc;
	}

	rc = gpio_direction_input(pdata->gpio_acc_int);
	if (rc < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, pdata->gpio_acc_int, rc);
		goto err_gpio_direction_input;
	}

	/* configure INT1 to deliver data ready interrupt */
	rc = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG3, I1_DRDY1);
	if (rc) {
		pr_err("%s: CTRL_REG3 write failed with error %d\n",
			__func__, rc);
		goto err_i2c_write_failed;
	}

	irq = gpio_to_irq(pdata->gpio_acc_int);

	/* trigger high so we don't miss initial interrupt if it
	 * is already pending
	 */
	rc = request_irq(irq, k3dh_irq_handler, IRQF_TRIGGER_HIGH,
			 "acc_int", k3dh);
	if (rc < 0) {
		pr_err("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			pdata->gpio_acc_int, rc);
		goto err_request_irq;
	}

	/* start with interrupt disabled until the driver is enabled */
	k3dh->irq = irq;
	k3dh_disable_irq(k3dh);

	goto done;

err_request_irq:
err_i2c_write_failed:
err_gpio_direction_input:
	gpio_free(pdata->gpio_acc_int);
done:
	return rc;
}

static ssize_t k3dh_fs_read(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct k3dh_data *k3dh = dev_get_drvdata(dev);
	struct k3dh_acc data = { 0, };
	int err;
	int on;

	mutex_lock(&k3dh->read_lock);
	on = atomic_read(&k3dh->opened);
	if (on == 0) {
		err = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1,
						DEFAULT_POWER_ON_SETTING);
	if (err) {
			mutex_unlock(&k3dh->read_lock);
			pr_err("%s: i2c write ctrl_reg1 failed\n", __func__);
			return err;
		}
	}

	err = k3dh_read_accel_xyz(k3dh, &data);
	if (err < 0) {
		mutex_unlock(&k3dh->read_lock);
		pr_err("%s: k3dh_read_accel_xyz failed\n", __func__);
		return err;
	}

	if (on == 0) {
		err = i2c_smbus_write_byte_data(k3dh->client, CTRL_REG1,
								PM_OFF);
		if (err)
			pr_err("%s: i2c write ctrl_reg1 failed\n", __func__);

	}
	mutex_unlock(&k3dh->read_lock);

	return sprintf(buf, "%d,%d,%d\n", data.x, data.y, data.z);
}

static DEVICE_ATTR(acc_file, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH,
							k3dh_fs_read, NULL);

static int k3dh_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct k3dh_data *k3dh;
	struct device *dev_t;
	int err;
	struct k3dh_platform_data *pdata = client->dev.platform_data;

	if (!pdata) {
		pr_err("%s: missing pdata!\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		err = -ENODEV;
		goto exit;
	}

	k3dh = kzalloc(sizeof(struct k3dh_data), GFP_KERNEL);
	if (k3dh == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	k3dh->client = client;
	k3dh->pdata = pdata;
	i2c_set_clientdata(client, k3dh);

	init_completion(&k3dh->data_ready);
	mutex_init(&k3dh->read_lock);
	mutex_init(&k3dh->write_lock);
	atomic_set(&k3dh->opened, 0);

#if 0 /* tempolary block for polling umfa.k3dh*/
	err = k3dh_setup_irq(k3dh);
	if (err) {
		pr_err("%s: could not setup irq\n", __func__);
		goto err_setup_irq;
	}
#endif

	/* sensor HAL expects to find /dev/accelerometer */
	k3dh->k3dh_device.minor = MISC_DYNAMIC_MINOR;
	k3dh->k3dh_device.name = "accelerometer";
	k3dh->k3dh_device.fops = &k3dh_fops;

	err = misc_register(&k3dh->k3dh_device);
	if (err) {
		pr_err("%s: misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* for test */
	k3dh->acc_class = class_create(THIS_MODULE, "accelerometer");
	if (IS_ERR(k3dh->acc_class)) {
		pr_err("%s: class create failed(accelerometer)\n", __func__);
		err = PTR_ERR(k3dh->acc_class);
		goto err_class_create;
	}

	dev_t = device_create(k3dh->acc_class, NULL,
				MKDEV(ACC_DEV_MAJOR, 0), "%s", "accelerometer");
	if (IS_ERR(dev_t)) {
		pr_err("%s: device create failed(accelerometer)\n", __func__);
		err = PTR_ERR(dev_t);
		goto err_device_create;
	}

	err = device_create_file(dev_t, &dev_attr_acc_file);
	if (err < 0) {
		pr_err("%s: Failed to create device file(%s)\n",
				__func__, dev_attr_acc_file.attr.name);
		goto err_device_create_file;
	}
	dev_set_drvdata(dev_t, k3dh);

	printk("k3dh probe success!\n");

	return 0;

err_device_create_file:
	device_destroy(k3dh->acc_class, MKDEV(ACC_DEV_MAJOR, 0));
err_device_create:
	class_destroy(k3dh->acc_class);
err_class_create:
	misc_deregister(&k3dh->k3dh_device);
err_misc_register:
	free_irq(k3dh->irq, k3dh);
	gpio_free(k3dh->pdata->gpio_acc_int);
err_setup_irq:
	mutex_destroy(&k3dh->read_lock);
	mutex_destroy(&k3dh->write_lock);
	kfree(k3dh);
exit:
	return err;
}

static int k3dh_remove(struct i2c_client *client)
{
	struct k3dh_data *k3dh = i2c_get_clientdata(client);

	device_destroy(k3dh->acc_class, MKDEV(ACC_DEV_MAJOR, 0));
	class_destroy(k3dh->acc_class);
	misc_deregister(&k3dh->k3dh_device);
	free_irq(k3dh->irq, k3dh);
	gpio_free(k3dh->pdata->gpio_acc_int);
	mutex_destroy(&k3dh->read_lock);
	mutex_destroy(&k3dh->write_lock);
	kfree(k3dh);

	return 0;
}

static const struct i2c_device_id k3dh_id[] = {
	{ "k3dh", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, k3dh_id);

static struct i2c_driver k3dh_driver = {
	.probe = k3dh_probe,
	.remove = __devexit_p(k3dh_remove),
	.id_table = k3dh_id,
	.driver = {
		.pm = &k3dh_pm_ops,
		.owner = THIS_MODULE,
		.name = "k3dh",
	},
};

static int __init k3dh_init(void)
{
	return i2c_add_driver(&k3dh_driver);
}

static void __exit k3dh_exit(void)
{
	i2c_del_driver(&k3dh_driver);
}

module_init(k3dh_init);
module_exit(k3dh_exit);

MODULE_DESCRIPTION("k3dh accelerometer driver");
MODULE_AUTHOR("tim.sk.lee@samsung.com");
MODULE_LICENSE("GPL");
