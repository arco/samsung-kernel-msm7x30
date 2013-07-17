/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
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
#include <linux/pwm.h>
//#include <plat/gpio-cfg.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/io.h>
//#include <mach/gpio-aries.h>
#include "tspdrv.h"
#include <linux/clk.h>

#include <mach/clk-provider.h>

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1

#define PWM_DUTY_MAX    579 /* 13MHz / (579 + 1) = 22.4kHz */

#define FREQ_COUNT		87084/2	/*89284*/

#define PWM_DEVICE	1

#define GPD0_TOUT_1		2 << 4

//struct pwm_device	*Immvib_pwm;

static bool g_bAmpEnabled = false;

long int freq_count = FREQ_COUNT;
struct clk *android_vib_clk; /* gp_clk */

#define GP_CLK_M_DEFAULT			2
#define GP_CLK_N_DEFAULT			1690
#define GP_CLK_D_DEFAULT			845	/* 50% duty cycle */ 
#define IMM_PWM_MULTIPLIER		    17778	/* Must be integer */

/* Variable for setting PWM in Force Out Set */
VibeInt32 g_nForce_32 = 0;

/*
 * ** Global variables for LRA PWM M,N and D values.
 * */
VibeInt32 g_nLRA_GP_CLK_M = GP_CLK_M_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_N = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_D = GP_CLK_D_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_PWM_MUL = IMM_PWM_MULTIPLIER;





static int vibe_set_pwm_freq(int nForce)
{

	if(nForce == 0) //Generate Clock with appropriate frequency(around 22KHz) and 50% duty cycle.
	{
		/* Put the MND counter in reset mode for programming */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 0 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT); /* P: 0 => Freq/1, 1 => Freq/2, 4 => Freq/4 */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT); /* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT); /* Dual-edge mode */
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, g_nLRA_GP_CLK_M << HWIO_GP_MD_REG_M_VAL_SHFT);
		//printk("%s, g_nForce_32 : %d\n",__FUNCTION__,g_nForce_32);
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nLRA_GP_CLK_D << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, ~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);                    /* Enable M/N counter */
		//printk("%x, %x, %x\n",g_nLRA_GP_CLK_M,~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M),g_nForce_32);
	}
	else //Clock is already running, so control only D register here.
	{
	
	g_nForce_32 = ((nForce * (g_nLRA_GP_CLK_D -2)) / 127 ) + g_nLRA_GP_CLK_D;
		//printk("%s, g_nForce_32 : %d\n",__FUNCTION__,g_nForce_32);
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);
	}
		return VIBE_S_SUCCESS;
}
/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
#if 0
#error Please review the code between the #if and #endif

    if (g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));

        g_bAmpEnabled = false;

#if 0
        mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_LOW);
	    mz_ops.bstat &= ~HN_BATTERY_MOTOR;
#endif
    }
#endif
    if (g_bAmpEnabled)
    {
		g_bAmpEnabled = false;
		
		//pwm_config(Immvib_pwm, 0, freq_count/2);
		//pwm_disable(Immvib_pwm);
		
		//gpio_request(GPIO_VIBTONE_EN1, "GPIO_VIBTONE_EN1");
		//gpio_direction_output(GPIO_VIBTONE_EN1, GPIO_LEVEL_LOW);
		//gpio_direction_output(GPIO_VIBTONE_PWM, GPIO_LEVEL_LOW);
		//gpio_free(GPIO_VIBTONE_EN1);
		gpio_set_value(GPIO_VIBTONE_EN1, GPIO_LEVEL_LOW);
		
		/* Init - fix for android_vib_clk is unbalanced error */
		if (android_vib_clk->count == 0) {
			printk("[VIBETONZ] %s: there are no android_vib_clk to disable \n",__func__);
		} else {
			clk_disable(android_vib_clk);
		}
		/* End - fix for android_vib_clk is unbalanced error */
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
#if 0
#error Please review the code between the #if and #endif

    if (!g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));

        g_bAmpEnabled = true;

#if 0
        /* 
        ** Ensure the PWM frequency is at the expected value. These 2 lines of code
        ** can be removed if no other application alters the PWM frequency.
        */
        PWM_CTRL  = 0;                  /* 13Mhz / (0 + 1) = 13MHz */
        PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

        /* Set duty cycle to 50% */
        PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */

        /* Enable amp */
        mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_HIGH);
        mz_ops.bstat |= HN_BATTERY_MOTOR;
#endif
    }
#endif

    if (!g_bAmpEnabled)
    {
    		g_bAmpEnabled = true;

		//pwm_enable(Immvib_pwm);
		
		//gpio_request(GPIO_VIBTONE_EN1, "GPIO_VIBTONE_EN1");
		//gpio_direction_output(GPIO_VIBTONE_EN1, GPIO_LEVEL_LOW);
#if 0
	if (gpio_tlmm_config(GPIO_CFG(GPIO_VIBTONE_PWM, 3, GPIO_CFG_OUTPUT,
					  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
			pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
			       __func__, GPIO_VIBTONE_PWM);
#endif

		mdelay(1);
		clk_enable(android_vib_clk);

		vibe_set_pwm_freq(0);
		gpio_set_value(GPIO_VIBTONE_EN1, GPIO_LEVEL_HIGH);
		//gpio_free(GPIO_VIBTONE_EN1);
    }

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
#if 0
#error Please review the code between the #if and #endif

    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize.\n"));

    g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */

    /* 
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call ImmVibeSPI_ForceOut_AmpDisable
    ** for each actuator (provide the actuator index as input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

#if 0
    /* 
    ** PWM frequency:
    ** The PWM frequency must be set to a fixed value and shouldn't change
    ** during the lifetime of the app. The ideal solution would be to use a
    ** frequency value between 20kHz and 50kHz. A frequency value slightly
    ** outside of the above limits should still work and be compliant with
    ** TSP requirements (please refer to the TSP integration guide for
    ** further information).
    */

    /* 22.4kHz PWM, duty cycle 50% */
    PWM_CTRL = 0;                   /* 13Mhz / (0 + 1) = 13MHz */
    PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */
    PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */
#endif
#endif
	g_bAmpEnabled = true; 

	//Immvib_pwm = pwm_request(PWM_DEVICE, "Immvibtonz");
	//pwm_config(Immvib_pwm, freq_count/2, freq_count);
	//printk("[TEMP] Immvib_pwm = %d PWM_DEVICE = %d\n",Immvib_pwm, PWM_DEVICE);
	android_vib_clk = clk_get(NULL,"gp_clk");
	if(IS_ERR(android_vib_clk)) {
		printk("android vib clk failed!!!\n");
	} else {
		printk("THNAK YOU!!\n");
	}

	ImmVibeSPI_ForceOut_AmpDisable(0);
//	vibe_set_pwm_freq(120);

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
//#error Please review the code between the #if and #endif

 //   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.\n"));

    /* 
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call
    ** ImmVibeSPI_ForceOut_AmpDisable for each actuator (provide the actuator index as
    ** input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    VibeInt8 nForce;

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1)
			{
				DbgOut((KERN_ERR "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
				return VIBE_E_FAIL;
            }
            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            return VIBE_E_FAIL;
    }
	//int pwm_duty=freq_count/2 + ((freq_count/2 - 2) * nForce)/127;

	//DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_SetSamples: nForce[%d], freq_count:[%d], pwm_duty:[%d]\n", nForce, freq_count, pwm_duty));

	if (nForce == 0)
	{
		ImmVibeSPI_ForceOut_AmpDisable(0);

	}
	else
	{
		//pwm_config(Immvib_pwm, pwm_duty, freq_count);
		ImmVibeSPI_ForceOut_AmpEnable(0);
		vibe_set_pwm_freq(nForce);
	}
    


    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    return VIBE_S_SUCCESS;
}
