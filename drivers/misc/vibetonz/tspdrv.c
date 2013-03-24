/*
** =========================================================================
** File:
**     tspdrv.c
**
** Description: 
**     TouchSense Kernel Module main entry-point.
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifndef __KERNEL__
#define __KERNEL__
#endif
/* Remove below three lines if this is not needed. */
#ifndef MODULE
//#define MODULE
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include "../../staging/android/timed_output.h"
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

#include "tspdrv.h"
#include "ImmVibeSPI.c"
#if defined(VIBE_DEBUG) && defined(VIBE_RECORD)
#include <tspdrvRecorder.c>
#endif

/* Device name and version information */
#define VERSION_STR " v3.4.55.5\n"                  /* DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[  (VIBE_MAX_DEVICE_NAME_LENGTH 
                            + VERSION_STR_LEN)
                            * NUM_ACTUATORS];       /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */

static struct wake_lock vib_wake_lock;

/* Flag indicating whether the driver is in use */
static char g_bIsPlaying = false;

/* Buffer to store data sent to SPI */
#define SPI_BUFFER_SIZE (NUM_ACTUATORS * (VIBE_OUTPUT_SAMPLE_SIZE + SPI_HEADER_SIZE))
static int g_bStopRequested = false;
static actuator_samples_buffer g_SamplesBuffer[NUM_ACTUATORS] = {{0}}; 
static char g_cWriteBuffer[SPI_BUFFER_SIZE];
#define VIBE_TUNING

#define VIBE_TUNING_IMMR
#ifdef VIBE_TUNING_IMMR
#define TSPDRV_TUNING_ARG1  _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 185)
#define TSPDRV_TUNING_ARG2  _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 186)
#endif

/* For QA purposes */
#ifdef QA_TEST
#define FORCE_LOG_BUFFER_SIZE   128
#define TIME_INCREMENT          5
static int g_nTime = 0;
static int g_nForceLogIndex = 0;
static VibeInt8 g_nForceLog[FORCE_LOG_BUFFER_SIZE];
#endif

#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(2,6,0))
#error Unsupported Kernel version
#endif

#ifdef IMPLEMENT_AS_CHAR_DRIVER
static int g_nMajor = 0;
#endif

/* Needs to be included after the global variables because it uses them */
#include "VibeOSKernelLinuxTime.c"
/* timed_output */
#define VIBRATOR_PERIOD	87084/2
#define VIBRATOR_DUTY	87000/2

static struct hrtimer timer;
static struct regulator *vreg_ldo19;

static int max_timeout = 5000;
static int vibrator_value = 0;
static int pwmvalue = 121;

struct pwm_device	*vib_pwm;

static int set_vibetonz(int timeout) {
	if(!timeout) {
		//printk("[VIBETONZ] FIXED DISABLE\n");
		ImmVibeSPI_ForceOut_AmpDisable(0);
		wake_unlock(&vib_wake_lock);
	} else {
		wake_lock(&vib_wake_lock);
		//printk("[VIBETONZ] FIXED ENABLE\n");
		ImmVibeSPI_ForceOut_AmpEnable(0);
		vibe_set_pwm_freq(pwmvalue);
	}

	vibrator_value = timeout;

	return 0;
}

static enum hrtimer_restart vibetonz_timer_func(struct hrtimer *timer)
{
	set_vibetonz(0);
	return HRTIMER_NORESTART;
}

static int get_time_for_vibetonz(struct timed_output_dev *dev)
{
	int remaining;

	if (hrtimer_active(&timer)) {
		ktime_t r = hrtimer_get_remaining(&timer);
		remaining = ktime_to_ms(r);
	} else
		remaining = 0;

	if (vibrator_value ==-1)
		remaining = -1;

	return remaining;

}

static void enable_vibetonz_from_user(struct timed_output_dev *dev,int value)
{
	//printk("[VIBETONZ] %s : time = %d msec \n",__func__,value);
	hrtimer_cancel(&timer);
	
	set_vibetonz(value);
	vibrator_value = value;

	if (value > 0) 
	{
		if (value > max_timeout)
			value = max_timeout;

		hrtimer_start(&timer,
						ktime_set(value / 1000, (value % 1000) * 1000000),
						HRTIMER_MODE_REL);
		vibrator_value = 0;
	}
}

static struct timed_output_dev timed_output_vt = {
	.name     = "vibrator",
	.get_time = get_time_for_vibetonz,
	.enable   = enable_vibetonz_from_user,
};

static void vibetonz_start(void)
{
	int ret = 0;

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = vibetonz_timer_func;

	ret = timed_output_dev_register(&timed_output_vt);
	if(ret)
		printk(KERN_ERR "[VIBETONZ] timed_output_dev_register is fail \n");	
}


/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);
static long ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static struct file_operations fops = 
{
    .owner =    THIS_MODULE,
    .read =     read,
    .write =    write,
    .unlocked_ioctl =    ioctl,
    .open =     open,
    .release =  release
};

#ifndef IMPLEMENT_AS_CHAR_DRIVER
static struct miscdevice miscdev = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     MODULE_NAME,
	.fops =     &fops
};
#endif

static int suspend(struct platform_device *pdev, pm_message_t state);
static int resume(struct platform_device *pdev);

static struct platform_driver platdrv = 
{
    .suspend =  suspend,	
    .resume =   resume,	
    .driver = 
    {		
        .name = MODULE_NAME,	
    },	
};

static void platform_release(struct device *dev);
static struct platform_device platdev = 
{	
	.name =     MODULE_NAME,	
	.id =       -1,                     /* means that there is only one device */
	.dev = 
    {
		.platform_data = NULL, 		
		.release = platform_release,    /* a warning is thrown during rmmod if this is absent */
	},
};

#ifdef VIBE_TUNING
struct class *vibetonz_class;
EXPORT_SYMBOL(vibetonz_class);

struct device *immTest_test;
EXPORT_SYMBOL(immTest_test);

extern long int freq_count;
static ssize_t immTest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[VIBETONZ] %s : operate nothing\n", __FUNCTION__);

	return 0;
}
static ssize_t immTest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);

   	if(value ==1000)
	{
		ImmVibeSPI_ForceOut_AmpEnable(0);
		vibe_set_pwm_freq(121);

		mdelay(1000);

		ImmVibeSPI_ForceOut_AmpDisable(0);
	}
	else
		ImmVibeSPI_ForceOut_AmpDisable(value);

	return size;
}
static DEVICE_ATTR(immTest, S_IRUGO | S_IWUSR, immTest_show, immTest_store);

struct device *immIntensity;

static ssize_t pwmvalue_intensity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	count = sprintf(buf,"%d\n", pwmvalue);
	return count;
}
static ssize_t pwmvalue_intensity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);

   	if(0 <= value && value <= 127) {
		pwmvalue = (int) value;
		vibe_set_pwm_freq((int) value);
	}
	return size;
}
static DEVICE_ATTR(pwmvalue_intensity, S_IRUGO | S_IWUSR, pwmvalue_intensity_show, pwmvalue_intensity_store);
#endif /* VIBE_TUNING */

int vibe_init(void)
{
    int nRet, i;   /* initialized below */
	int rc;

    DbgOut((KERN_INFO "tspdrv: init_module.\n"));

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    g_nMajor = register_chrdev(0, MODULE_NAME, &fops);
    if (g_nMajor < 0) 
    {
        DbgOut((KERN_ERR "tspdrv: can't get major number.\n"));
        return g_nMajor;
    }
#else
    nRet = misc_register(&miscdev);
	if (nRet) 
    {
        DbgOut((KERN_ERR "tspdrv: misc_register failed.\n"));
		return nRet;
	}
#endif

	nRet = platform_device_register(&platdev);
	if (nRet) 
    {
        DbgOut((KERN_ERR "tspdrv: platform_device_register failed.\n"));
    }

	nRet = platform_driver_register(&platdrv);
	if (nRet) 
    {
        DbgOut((KERN_ERR "tspdrv: platform_driver_register failed.\n"));
    }

    DbgRecorderInit(());

	if (gpio_tlmm_config(GPIO_CFG(GPIO_VIBTONE_EN1, 0, GPIO_CFG_OUTPUT,
				  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, GPIO_VIBTONE_EN1);

	if (gpio_tlmm_config(GPIO_CFG(GPIO_VIBTONE_PWM, 3, GPIO_CFG_OUTPUT,
					  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
			pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
			       __func__, GPIO_VIBTONE_PWM);

	if (gpio_is_valid(GPIO_VIBTONE_EN1)) {
		if (gpio_request(GPIO_VIBTONE_EN1, "GPIO_VIBTONE_EN1"))
			printk(KERN_ERR "Failed to request GPIO_VIBTONE_EN1! \n");
	}

	if (gpio_is_valid(GPIO_VIBTONE_PWM)) {
		if (gpio_request(GPIO_VIBTONE_PWM, "GPIO_VIBTONE_PWM"))
			printk(KERN_ERR "Failed to request GPIO_VIBTONE_PWM! \n");
	}

	// VDD_VIB_3.0V
	vreg_ldo19 = regulator_get(NULL, "wlan2");
	if (IS_ERR(vreg_ldo19)) {
		rc = PTR_ERR(vreg_ldo19);
		pr_err("%s: wlan2 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = regulator_set_voltage(vreg_ldo19,3000000,3000000);
	if (rc) {
		pr_err("%s: vreg LDO19 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = regulator_enable(vreg_ldo19);

	if (rc) {
		pr_err("%s: LDO19 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}

    ImmVibeSPI_ForceOut_Initialize();
    VibeOSKernelLinuxInitTimer();

    /* Get and concatenate device name and initialize data buffer */
    g_cchDeviceName = 0;
    for (i=0; i<NUM_ACTUATORS; i++)
    {
        char *szName = g_szDeviceName + g_cchDeviceName;
        ImmVibeSPI_Device_GetName(i, szName, VIBE_MAX_DEVICE_NAME_LENGTH);

        /* Append version information and get buffer length */
        strcat(szName, VERSION_STR);
        g_cchDeviceName += strlen(szName);

        g_SamplesBuffer[i].nIndexPlayingBuffer = -1; /* Not playing */
        g_SamplesBuffer[i].actuatorSamples[0].nBufferSize = 0;
        g_SamplesBuffer[i].actuatorSamples[1].nBufferSize = 0;
    }
    
    wake_lock_init(&vib_wake_lock, WAKE_LOCK_SUSPEND, "vib_present");

#ifdef VIBE_TUNING
	// ---------- file creation at '/sys/class/vibetonz/immTest'------------------------------
	vibetonz_class = class_create(THIS_MODULE, "vibetonz");
	if (IS_ERR(vibetonz_class))
		pr_err("Failed to create class(vibetonz)!\n");

	immTest_test = device_create(vibetonz_class, NULL, 0, NULL, "immTest");
	if (IS_ERR(immTest_test))
		pr_err("Failed to create device(switch)!\n");

	if (device_create_file(immTest_test, &dev_attr_immTest) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_immTest.attr.name);

    immIntensity = device_create(vibetonz_class, NULL, 0, NULL, "immDuty");
	if (IS_ERR(immIntensity))
		pr_err("Failed to create intensity device(switch)!\n");

	if (device_create_file(immIntensity, &dev_attr_pwmvalue_intensity) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_pwmvalue_intensity.attr.name);

#endif

    vibetonz_start();

    return 0;
}

void vibe_cleanup(void)
{
    DbgOut((KERN_INFO "tspdrv: cleanup_module.\n"));

    DbgRecorderTerminate(());

    VibeOSKernelLinuxTerminateTimer();
    ImmVibeSPI_ForceOut_Terminate();

	platform_driver_unregister(&platdrv);
	platform_device_unregister(&platdev);

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    unregister_chrdev(g_nMajor, MODULE_NAME);
#else
    misc_deregister(&miscdev);
#endif
    gpio_free(GPIO_VIBTONE_EN1);
}

static int open(struct inode *inode, struct file *file) 
{
    DbgOut((KERN_INFO "tspdrv: open.\n"));

    if (!try_module_get(THIS_MODULE)) return -ENODEV;

    return 0; 
}

static int release(struct inode *inode, struct file *file) 
{
    DbgOut((KERN_INFO "tspdrv: release.\n"));

    /* 
    ** Reset force and stop timer when the driver is closed, to make sure
    ** no dangling semaphore remains in the system, especially when the
    ** driver is run outside of immvibed for testing purposes.
    */
    VibeOSKernelLinuxStopTimer();

    /* 
    ** Clear the variable used to store the magic number to prevent 
    ** unauthorized caller to write data. TouchSense service is the only 
    ** valid caller.
    */
    file->private_data = (void*)NULL;

    module_put(THIS_MODULE);

    return 0; 
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    const size_t nBufSize = (g_cchDeviceName > (size_t)(*ppos)) ? min(count, g_cchDeviceName - (size_t)(*ppos)) : 0;

    /* End of buffer, exit */
    if (0 == nBufSize) return 0;

    if (0 != copy_to_user(buf, g_szDeviceName + (*ppos), nBufSize)) 
    {
        /* Failed to copy all the data, exit */
        DbgOut((KERN_ERR "tspdrv: copy_to_user failed.\n"));
        return 0;
    }

    /* Update file position and return copied buffer size */
    *ppos += nBufSize;
    return nBufSize;
}


static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    int i = 0;

    *ppos = 0;  /* file position not used, always set to 0 */

    /* 
    ** Prevent unauthorized caller to write data. 
    ** TouchSense service is the only valid caller.
    */
    if (file->private_data != (void*)TSPDRV_MAGIC_NUMBER) 
    {
        DbgOut((KERN_ERR "tspdrv: unauthorized write.\n"));
        return 0;
    }

    /* Copy immediately the input buffer */
    if (0 != copy_from_user(g_cWriteBuffer, buf, count))
    {
        /* Failed to copy all the data, exit */
        DbgOut((KERN_ERR "tspdrv: copy_from_user failed.\n"));
        return 0;
    }

    /* Check buffer size */
    if ((count <= SPI_HEADER_SIZE) || (count > SPI_BUFFER_SIZE))
    {
        DbgOut((KERN_ERR "tspdrv: invalid write buffer size.\n"));
        return 0;
    }

    while (i < count)
    {
        int nIndexFreeBuffer;   /* initialized below */

        samples_buffer* pInputBuffer = (samples_buffer*)(&g_cWriteBuffer[i]);

        if ((i + SPI_HEADER_SIZE) >= count)
        {
            /*
            ** Index is about to go beyond the buffer size.
            ** (Should never happen).
            */
            DbgOut((KERN_EMERG "tspdrv: invalid buffer index.\n"));
        }

        /* Check bit depth */
        if (8 != pInputBuffer->nBitDepth)
        {
            DbgOut((KERN_WARNING "tspdrv: invalid bit depth. Use default value (8).\n"));
        }

        /* The above code not valid if SPI header size is not 3 */
#if (SPI_HEADER_SIZE != 3)
#error "SPI_HEADER_SIZE expected to be 3"
#endif

        /* Check buffer size */
        if ((i + SPI_HEADER_SIZE + pInputBuffer->nBufferSize) > count)
        {
            /*
            ** Index is about to go beyond the buffer size.
            ** (Should never happen).
            */
            DbgOut((KERN_EMERG "tspdrv: invalid data size.\n"));
        }
        
        /* Check actuator index */
        if (NUM_ACTUATORS <= pInputBuffer->nActuatorIndex)
        {
            DbgOut((KERN_ERR "tspdrv: invalid actuator index.\n"));
            i += (SPI_HEADER_SIZE + pInputBuffer->nBufferSize);
            continue;
        }

        if (0 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[0].nBufferSize)
        {
            nIndexFreeBuffer = 0;
        }
        else if (0 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[1].nBufferSize)
        {
             nIndexFreeBuffer = 1;
        }
        else
        {
            /* No room to store new samples  */
            DbgOut((KERN_ERR "tspdrv: no room to store new samples.\n"));
            return 0;
        }

        /* Store the data in the free buffer of the given actuator */
        memcpy(&(g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[nIndexFreeBuffer]), &g_cWriteBuffer[i], (SPI_HEADER_SIZE + pInputBuffer->nBufferSize));

        /* If the no buffer is playing, prepare to play g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[nIndexFreeBuffer] */
        if ( -1 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexPlayingBuffer)
        {
           g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexPlayingBuffer = nIndexFreeBuffer;
           g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexOutputValue = 0;
        }

        /* Increment buffer index */
        i += (SPI_HEADER_SIZE + pInputBuffer->nBufferSize);
    }

#ifdef QA_TEST
    g_nForceLog[g_nForceLogIndex++] = g_cSPIBuffer[0];
    if (g_nForceLogIndex >= FORCE_LOG_BUFFER_SIZE)
    {
        for (i=0; i<FORCE_LOG_BUFFER_SIZE; i++)
        {
            printk("<6>%d\t%d\n", g_nTime, g_nForceLog[i]);
            g_nTime += TIME_INCREMENT;
        }
        g_nForceLogIndex = 0;
    }
#endif

    /* Start the timer after receiving new output force */
    g_bIsPlaying = true;
    VibeOSKernelLinuxStartTimer();

    return count;
}

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef QA_TEST
    int i;
#endif

	DbgOut((KERN_INFO "tspdrv: ioctl cmd[0x%x].\n", cmd));

    switch (cmd)
    {
#ifdef VIBE_TUNING1
		case 185:
			g_PWM_duty_max = arg; /* set value of g_PWM_duty_max in ImmVibeSPI */
			break;
		case 186:
			g_PWM_ctrl = arg; /* set value of g_PWM_ctrl in ImmVibeSPI */
			break;
		case 187:
			g_PWM_multiplier = arg; /* set value of g_PWM_multiplier in ImmVibeSPI */
			break;
#endif /* VIBE_TUNING */
#ifdef VIBE_TUNING_IMMR
		/*commands from the test app(ChangeFreq.apk)*/
		case TSPDRV_TUNING_ARG1:
			printk("[tspdrv] TSPDRV_TUNING_ARG1 arg : %ld\n", arg);
			g_nLRA_GP_CLK_M = arg;
			break;
		case TSPDRV_TUNING_ARG2:
			printk("[tspdrv] TSPDRV_TUNING_ARG2 arg : %ld\n", arg);
			g_nLRA_GP_CLK_N = arg; 
			g_nLRA_GP_CLK_D = (g_nLRA_GP_CLK_N >> 1);
			break;
#endif /* VIBE_TUNING_IMMR */
        case TSPDRV_STOP_KERNEL_TIMER:
            /* 
            ** As we send one sample ahead of time, we need to finish playing the last sample
            ** before stopping the timer. So we just set a flag here.
            */
            if (true == g_bIsPlaying) g_bStopRequested = true;

#ifdef VIBEOSKERNELPROCESSDATA
            /* Last data processing to disable amp and stop timer */
            VibeOSKernelProcessData(NULL);
#endif

#ifdef QA_TEST
            if (g_nForceLogIndex)
            {
                for (i=0; i<g_nForceLogIndex; i++)
                {
                    printk("<6>%d\t%d\n", g_nTime, g_nForceLog[i]);
                    g_nTime += TIME_INCREMENT;
                }
            }
            g_nTime = 0;
            g_nForceLogIndex = 0;
#endif
            break;

        case TSPDRV_MAGIC_NUMBER:
            file->private_data = (void*)TSPDRV_MAGIC_NUMBER;
            break;

        case TSPDRV_ENABLE_AMP:
            wake_lock(&vib_wake_lock);
            ImmVibeSPI_ForceOut_AmpEnable(arg);
            DbgRecorderReset((arg));
            DbgRecord((arg,";------- TSPDRV_ENABLE_AMP ---------\n"));
            break;

        case TSPDRV_DISABLE_AMP:
            /* Small fix for now to handle proper combination of TSPDRV_STOP_KERNEL_TIMER and TSPDRV_DISABLE_AMP together */
            /* If a stop was requested, ignore the request as the amp will be disabled by the timer proc when it's ready */
            if(!g_bStopRequested)
            {
                ImmVibeSPI_ForceOut_AmpDisable(arg);
            }
            wake_unlock(&vib_wake_lock);
            break;

        case TSPDRV_GET_NUM_ACTUATORS:
            return NUM_ACTUATORS;
    }

    return 0;
}

static int suspend(struct platform_device *pdev, pm_message_t state) 
{
    if (g_bIsPlaying)
    {
        DbgOut((KERN_INFO "tspdrv: can't suspend, still playing effects.\n"));
        return -EBUSY;
    }
    else
    {
        DbgOut((KERN_INFO "tspdrv: suspend.\n"));
        return 0;
    }
}

static int resume(struct platform_device *pdev) 
{	
	DbgOut((KERN_INFO "tspdrv: resume.\n"));

	return 0;   /* can resume */
}

static void platform_release(struct device *dev) 
{	
    DbgOut((KERN_INFO "tspdrv: platform_release.\n"));
}

module_init(vibe_init);
module_exit(vibe_cleanup);
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("TouchSense Kernel Module");
MODULE_LICENSE("GPL v2");
