#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>

#include "Si4709_dev.h"
#include "Si4709_common.h"

struct si4709_data {
	struct i2c_client		*client;
};

static struct i2c_driver Si4709_i2c_driver;

static const struct i2c_device_id si4709_id[] = {
	{"Si4709", 0},
	{}
};

static int si4709_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;
	struct si4709_data *si4709_dev;

	printk("%s:\n", __func__);

	si4709_dev = kzalloc(sizeof(struct si4709_data), GFP_KERNEL);

	if (!si4709_dev) {
		err = -ENOMEM;
		return err;
	}

	i2c_set_clientdata(client, si4709_dev);

	err = Si4709_dev_init(client);
	if (err < 0)
		pr_err("%s: Si4709_dev_init failed\n", __func__);

	return 0;
}

static int si4709_i2c_remove(struct i2c_client *client)
{
	struct si4709_data *si4709_dev = i2c_get_clientdata(client);
	int ret = 0;

	printk("%s:\n", __func__);

	ret = Si4709_dev_exit();
	if (ret < 0)
		pr_err("%s: Si4709_dev_exit failed\n", __func__);

	kfree(si4709_dev);
	kfree(client);
	si4709_dev = NULL;

	return ret;
}

static int Si4709_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;

	printk("%s:\n", __func__);

	ret = Si4709_dev_suspend();
	if (ret < 0)
		pr_err("%s: Si4709_dev_disable failed\n", __func__);

	return 0;
}

static int Si4709_resume(struct i2c_client *client)
{
	int ret = 0;

	printk("%s:\n", __func__);

	ret = Si4709_dev_resume();
	if (ret < 0)
		pr_err("%s: Si4709_dev_enable failed\n", __func__);

	return 0;
}

MODULE_DEVICE_TABLE(i2c, si4709_id);

static struct i2c_driver Si4709_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "Si4709",
	},
	.id_table = si4709_id,
	.probe = si4709_i2c_probe,
	.remove = si4709_i2c_remove,
	.suspend = Si4709_suspend,
	.resume = Si4709_resume,
};

int Si4709_i2c_drv_init(void)
{
	int ret = 0;

	printk("%s:\n", __func__);

	ret = i2c_add_driver(&Si4709_i2c_driver);
	if (ret < 0)
		pr_err("%s:Si4709 i2c_add_driver failed\n", __func__);

	return ret;
}

void Si4709_i2c_drv_exit(void)
{
	printk("%s:\n", __func__);

	i2c_del_driver(&Si4709_i2c_driver);
}


