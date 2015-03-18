#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

#include <mach/samsung_vibe.h>
#include <linux/gpio.h>
#include <mach/vreg.h>

static struct hrtimer vibe_timer;
static struct vreg *vreg_vib = NULL;

static int is_vibe_on = 0;


static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state);
static int msm_vibrator_resume(struct platform_device *pdev);
static int msm_vibrator_probe(struct platform_device *pdev);
static int msm_vibrator_exit(struct platform_device *pdev);

extern int board_hw_revision;

/* for the suspend/resume VIBRATOR Module */
static struct platform_driver msm_vibrator_platdrv = 
{
	.probe   = msm_vibrator_probe,
	.suspend = msm_vibrator_suspend,
	.resume  = msm_vibrator_resume,
	.remove  = __devexit_p(msm_vibrator_exit),
	.driver = 
	{
			.name = MODULE_NAME,
			.owner = THIS_MODULE,
	},
};

static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state) {
	if(is_vibe_on) {
		is_vibe_on = 0;
	}
	pr_info("[VIB] suspend\n");
	return VIBE_S_SUCCESS;
}

static int msm_vibrator_resume(struct platform_device *pdev) {
	return VIBE_S_SUCCESS;
}

static int __devexit msm_vibrator_exit(struct platform_device *pdev) {
		pr_info("[VIB] EXIT\n");
		return 0;
}

static void set_pmic_vibrator(int on) {

	int rc;

	if (on) {
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_APACHE)
		rc = vreg_enable(vreg_vib);
		if (rc) {
			pr_err("%s: vreg_enable failed \n", __func__);
		}
#elif defined(CONFIG_MACH_ANCORA_TMO)
        if(board_hw_revision == 1)
        {
         gpio_set_value(VIB_ON,VIBRATION_ON);
        }
        else if (board_hw_revision>= 2 )
        {
            rc = vreg_enable(vreg_vib);
	    	if (rc) {
			pr_err("%s: vreg_enable failed \n", __func__);
	    	}
        }
#endif
		is_vibe_on = 1;
	} else {
		if(is_vibe_on) {
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_APACHE)
			rc = vreg_disable(vreg_vib);
			if (rc) {
				pr_err("%s: vreg_disable failed \n", __func__);
			}
#elif defined(CONFIG_MACH_ANCORA_TMO)
            if(board_hw_revision == 1) {
		gpio_set_value(VIB_ON, VIBRATION_OFF);
            } else if ( board_hw_revision>= 2 ) {
                    rc = vreg_disable(vreg_vib);
        			if (rc) {
        				pr_err("%s: vreg_disable failed \n", __func__);
        			}
            }
#endif
			is_vibe_on = 0;
		}
	}

}

static void pmic_vibrator_on(void) {
	set_pmic_vibrator(VIBRATION_ON);
}

static void pmic_vibrator_off(void) {
	set_pmic_vibrator(VIBRATION_OFF);
}

static DEFINE_SPINLOCK(lock);

static void vibrator_enable(struct timed_output_dev *dev, int value) {
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	hrtimer_cancel(&vibe_timer);

	if (value == 0) {
		pmic_vibrator_off();
	} else {
		if(value < 0)
			value = ~value;

		value = (value > 15000 ? 15000 : value);

		pmic_vibrator_on();

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&lock, flags);
}

static int vibrator_get_time(struct timed_output_dev *dev) {
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return ktime_to_ms(r);
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer) {

		unsigned int remain;

		if(hrtimer_active(&vibe_timer)) {
				ktime_t r = hrtimer_get_remaining(&vibe_timer);
				remain = ktime_to_ms(r);
				remain = remain / 1000;
				if(ktime_to_ms(r) < 0) {
						remain = 0;
				}
				if(!remain) 
					pmic_vibrator_off();
		} else {
			pmic_vibrator_off();
		}
		return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __devinit msm_vibrator_probe(struct platform_device *pdev) {

	int rc;

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
	 
	pr_info("[VIB] msm_vibrator_probe \n");

#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_APACHE)
	vreg_vib = vreg_get(NULL, "wlan2");

	if (IS_ERR(vreg_vib)) {
		pr_err("%s: wlan2 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_vib));
	}

	rc = vreg_set_level(vreg_vib, 3300);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
	}
#elif defined(CONFIG_MACH_ANCORA_TMO)
	rc = vreg_set_level(vreg_vib, 3000);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
	}

        if(board_hw_revision>= 2) {
	vreg_vib = vreg_get(NULL, "gp4");
		if (IS_ERR(vreg_vib)) {
			pr_err("%s: gp4 vreg get failed (%ld)", __func__, PTR_ERR(vreg_vib));
		}
    	rc = vreg_set_level(vreg_vib, 3300);
		if (rc) {
			pr_err("%s: vreg_set_level failed \n", __func__);
		}
	}
#endif
	return 0;
}

static int __init msm_init_pmic_vibrator(void) {

	int nRet;

	nRet = platform_driver_register(&msm_vibrator_platdrv);

	pr_info("[VIB] platform driver register result : %d\n",nRet);
	if (nRet) { 
		pr_info("[VIB] platform_driver_register failed\n");
	}
	return nRet;
}

static void __exit msm_exit_pmic_vibrator(void) {
	platform_driver_unregister(&msm_vibrator_platdrv);
}

module_init(msm_init_pmic_vibrator);
module_exit(msm_exit_pmic_vibrator);

MODULE_DESCRIPTION("Samsung vibrator device");
MODULE_LICENSE("GPL");
