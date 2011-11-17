/*
 * drivers/input/sec-input-bridge.c
 *
 * Copyright 2011 Samsung electronics.
 *
 * Specific control input event bridge for samsung electronics
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <linux/workqueue.h>
#include <linux/mutex.h>

#include <linux/input/sec-input-bridge.h>

/* select ram dump feature */
//#define USE_BUG_RAMDUMP
#define USE_ARM11_RESET_RAMDUMP

#ifdef USE_ARM11_RESET_RAMDUMP
#include <asm/io.h>
#include <mach/msm_iomap.h>
#endif

/* led black feature for GT-I8150 */
#define USE_LED_BLANK_FOR_ANCORA

#ifdef USE_LED_BLANK_FOR_ANCORA
#include <mach/vreg.h>

static struct vreg *vreg_ldo2;
static int led_loop_count = 5;
#endif

/* dump enable flag */
extern int dump_enable_flag;

struct sec_input_bridge {
    struct sec_input_bridge_platform_data *pdata;
    struct work_struct work;
    struct mutex lock;
    struct platform_device *dev;

    u8 check_index;
};

static void input_bridge_set_ids(struct input_device_id *ids, unsigned int type, unsigned int code)
{
    switch (type) {
    case EV_KEY:
        ids->flags = INPUT_DEVICE_ID_MATCH_KEYBIT;
        __set_bit(code, ids->keybit);
        break;

    case EV_REL:
        ids->flags = INPUT_DEVICE_ID_MATCH_RELBIT;
        __set_bit(code, ids->relbit);
        break;

    case EV_ABS:
        ids->flags = INPUT_DEVICE_ID_MATCH_ABSBIT;

        __set_bit(code, ids->absbit);
        break;

    case EV_MSC:
        ids->flags = INPUT_DEVICE_ID_MATCH_MSCIT;
        __set_bit(code, ids->mscbit);
        break;

    case EV_SW:
        ids->flags = INPUT_DEVICE_ID_MATCH_SWBIT;
        __set_bit(code, ids->swbit);
        break;

    case EV_LED:
        ids->flags = INPUT_DEVICE_ID_MATCH_LEDBIT;
        __set_bit(code, ids->ledbit);
        break;

    case EV_SND:
        ids->flags = INPUT_DEVICE_ID_MATCH_SNDBIT;
        __set_bit(code, ids->sndbit);
        break;

    case EV_FF:
        ids->flags = INPUT_DEVICE_ID_MATCH_FFBIT;
        __set_bit(code, ids->ffbit);
        break;

    case EV_PWR:
        break;

    default:
        printk(KERN_ERR
                "input_bridge_set_ids: unknown type %u (code %u)\n",
                type, code);
        return;
    }

    ids->flags |= INPUT_DEVICE_ID_MATCH_EVBIT;
    __set_bit(type, ids->evbit);
}

static void input_bridge_work(struct work_struct *work) {
    struct sec_input_bridge *bridge = container_of(work, struct sec_input_bridge, work);

#ifdef USE_LED_BLANK_FOR_ANCORA
    int i;
#endif

#ifdef USE_ARM11_RESET_RAMDUMP
    void __iomem *reset_base;


#define RESET_ALL       0xab800200
#define RESET_ALL_VAL   0x1

#endif

    if (!dump_enable_flag) {
//        printk(KERN_INFO "[jjals] debug level low skip ramdump mode!!!\n");
        return;
    }

    mutex_lock(&bridge->lock);

//    printk("\n[jjals] ramdump gogosing!!!!!\n");

#ifdef USE_LED_BLANK_FOR_ANCORA
    for (i = 0; i < led_loop_count; i++) {
        vreg_enable(vreg_ldo2);
        msleep(300);
        vreg_disable(vreg_ldo2);
        msleep(300);
    }
#endif

#ifdef USE_BUG_RAMDUMP
    BUG();
#endif

#ifdef USE_ARM11_RESET_RAMDUMP

    /* force upload mode */
    writel(0xCCCC, MSM_SHARED_RAM_BASE + 0x30); //proc_comm[3].command
    writel(0, MSM_SHARED_RAM_BASE + 0x38); //proc_comm[3].data1
	writel(0xABCDABCD, MSM_SHARED_RAM_BASE + 0x3C); //proc_comm[3].data2

	reset_base = ioremap(RESET_ALL, PAGE_SIZE);

	writel(RESET_ALL_VAL, reset_base);

	while(1);
#endif

    mutex_unlock(&bridge->lock);
    printk(KERN_INFO "<sec-input-bridge> all process done!!\n");
}

static void input_bridge_event(struct input_handle *handle, unsigned int type,
                unsigned int code, int value)
{
    int rep_check;

    struct input_handler *sec_bridge_handler = handle->handler;
    struct sec_input_bridge *sec_bridge = sec_bridge_handler->private;

    rep_check = test_bit(EV_REP, sec_bridge_handler->id_table->evbit);
    rep_check = (rep_check << 1) | 1;

    switch (type) {
    case EV_KEY:
        if (value & rep_check) {
#ifdef KEY_LOG_TEST
            printk(KERN_INFO "sec-input-bridge: KEY input intercepted, type: %d, code: %d, value: %d\n", type, code, value);
#endif

            if (sec_bridge->pdata->mkey_map[sec_bridge->check_index].code == code) {
                sec_bridge->check_index++;
                if ((sec_bridge->check_index) >= (sec_bridge->pdata->num_mkey)) {
                    schedule_work(&sec_bridge->work);
                    sec_bridge->check_index = 0;
                }
            } else {
                sec_bridge->check_index = 0;
            }
        }

    default:
        break;
    }
}

static int input_bridge_connect(struct input_handler *handler,
                            struct input_dev *dev,
                            const struct input_device_id *id)
{
    struct input_handle *handle;
    int error;

    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle)
        return -ENOMEM;

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "sec-input-bridge";

    error = input_register_handle(handle);
    if (error) {
        printk(KERN_ERR
                "sec-input-bridge: Failed to register input bridge handler, "
                "error %d\n", error);
        kfree(handle);
        return error;
    }

    error = input_open_device(handle);
    if (error) {
        printk(KERN_ERR
                "sec-input-bridge: Failed to open input_bridge device, "
                "error %d\n", error);
        input_unregister_handle(handle);
        kfree(handle);
        return error;
    }

    return 0;
}

static void input_bridge_disconnect(struct input_handle *handle)
{
    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
}

static struct input_handler input_bridge_handler = {
    .event = input_bridge_event,
    .connect = input_bridge_connect,
    .disconnect = input_bridge_disconnect,
    .name = "sec-input-bridge",
};

static int __devinit sec_input_bridge_probe(struct platform_device *dev)
{
    struct sec_input_bridge_platform_data *pdata;
    struct sec_input_bridge *bridge;
    struct input_device_id *input_bridge_ids;

    int state;
    int i;

    pdata = dev->dev.platform_data;
    if (!pdata) {
        dev_err(&dev->dev, "No samsung input bridge platform data\n");
        return -EINVAL;
    }

    if (pdata->num_mkey == 0) {
        dev_err(&dev->dev, "No samsung input bridge platform data. num_mkey == 0\n");
        return -EINVAL;
    }

    bridge = kzalloc(sizeof(struct sec_input_bridge), GFP_KERNEL);
    if (!bridge) {
        dev_err(&dev->dev, "failed to allocate mem\n");
        return -ENOMEM;
    }

    input_bridge_ids = kzalloc(sizeof(struct input_device_id[(pdata->num_mkey + 1)]), GFP_KERNEL);
    if (!input_bridge_ids) {
        dev_err(&dev->dev, "failed to allocate mem!\n");
        kfree(bridge);
        return -ENOMEM;
    }
    memset(input_bridge_ids, 0x00, sizeof(input_bridge_ids));

    for (i = 0; i < pdata->num_mkey; i++) {
        input_bridge_set_ids(&input_bridge_ids[i], pdata->mkey_map[i].type, pdata->mkey_map[i].code);
    }

    input_bridge_handler.private = bridge;
    input_bridge_handler.id_table = input_bridge_ids;

    state = input_register_handler(&input_bridge_handler);
    if (state) {
        goto input_register_fail;
    }

    bridge->dev = dev;
    bridge->pdata = pdata;

    INIT_WORK(&bridge->work, input_bridge_work);
    mutex_init(&bridge->lock);

    platform_set_drvdata(dev, bridge);

#ifdef USE_LED_BLANK_FOR_ANCORA
    vreg_ldo2 = vreg_get(NULL, "xo_out");
    if (IS_ERR(vreg_ldo2)) {
        dev_err(&dev->dev, "failed to get ldo2 regulator\n");
        goto input_register_fail;
    }
#endif

    return 0;

input_register_fail:
    cancel_work_sync(&bridge->work);
    mutex_destroy(&bridge->lock);
    kfree(bridge);
    kfree(input_bridge_ids);

    return state;
}

static int __devexit sec_input_bridge_remove(struct platform_device *dev)
{
    struct sec_input_bridge *bridge = platform_get_drvdata(dev);

    cancel_work_sync(&bridge->work);
    mutex_destroy(&bridge->lock);
    kfree(input_bridge_handler.id_table);
    input_unregister_handler(&input_bridge_handler);
    kfree(bridge);
    platform_set_drvdata(dev, NULL);

    return 0;
}

#ifdef CONFIG_PM
static int sec_input_bridge_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

static int sec_input_bridge_resume(struct platform_device *dev)
{
    return 0;
}
#else
#define sec_input_bridge_suspend    NULL
#define sec_input_bridge_resume     NULL
#endif

static struct platform_driver sec_input_bridge_driver = {
    .probe = sec_input_bridge_probe,
    .remove = __devexit_p(sec_input_bridge_remove),
    .suspend = sec_input_bridge_suspend,
    .resume = sec_input_bridge_resume,
    .driver = {
        .name = "samsung_input_bridge",
    },
};

static int __init sec_input_bridge_init(void)
{
    return platform_driver_register(&sec_input_bridge_driver);
}

static void __exit sec_input_bridge_exit(void)
{
    platform_driver_unregister(&sec_input_bridge_driver);
}

module_init(sec_input_bridge_init);
module_exit(sec_input_bridge_exit);

MODULE_AUTHOR("jc22.kim@samsung.com");
MODULE_DESCRIPTION("Input event -> specific control bridge");
MODULE_LICENSE("GPL");
