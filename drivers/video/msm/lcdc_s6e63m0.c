/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <mach/vreg.h>
#include "lcdc_s6e63m0.h"

static int ldi_enable = 0;
static int current_gamma_lvl = -1;
static int lcd_type = 0;
static int spi_cs;
static int spi_sclk;
static int spi_sdi;
static int lcd_reset;
static int delayed_backlight_value = -1;
static boolean First_Disp_Power_On = FALSE;
static struct msm_panel_common_pdata *lcdc_s6e63m0_pdata;
static struct device *lcd_dev;
extern struct class *sec_class;
#ifdef CONFIG_USES_ACL
static int acl_enable = 0;
static int cur_acl = 0;
static struct class *acl_class;
static struct device *switch_aclset_dev;
#endif
#ifdef GAMMASET_CONTROL
struct class *gammaset_class;
struct device *switch_gammaset_dev;
#endif
static int on_19gamma = 0;

#if defined(CONFIG_MACH_APACHE)
extern int board_hw_revision;
#endif
#define DEFAULT_LCD_ON_BACKLIGHT_LEVEL 23

static DEFINE_SPINLOCK(lcd_ctrl_irq_lock);
static DEFINE_SPINLOCK(bl_ctrl_lock);
static DEVICE_ATTR(lcd_power , 0664, s6e63m0_show_power, s6e63m0_store_power);
static DEVICE_ATTR(lcdtype_file_cmd , 0664, s6e63m0_show_lcd_type, NULL);
#ifdef CONFIG_USES_ACL
static DEVICE_ATTR(aclset_file_cmd,0664, aclset_file_cmd_show, aclset_file_cmd_store);
#endif
#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
static DEVICE_ATTR(gammaset_file_cmd,0664, gammaset_file_cmd_show, gammaset_file_cmd_store);
#endif

static struct setting_table *p22Gamma_set[] = {
    NULL,// display off
    s6e63m0_22gamma_30cd,// 1
    s6e63m0_22gamma_40cd,
    s6e63m0_22gamma_70cd,
    s6e63m0_22gamma_90cd,
    s6e63m0_22gamma_100cd,// 5
    s6e63m0_22gamma_110cd,
    s6e63m0_22gamma_120cd,
    s6e63m0_22gamma_130cd,
    s6e63m0_22gamma_140cd,
    s6e63m0_22gamma_150cd,// 10
    s6e63m0_22gamma_170cd,
    s6e63m0_22gamma_180cd,
    s6e63m0_22gamma_190cd,
    s6e63m0_22gamma_200cd,
    s6e63m0_22gamma_210cd,// 15
    s6e63m0_22gamma_220cd,
    s6e63m0_22gamma_230cd,
    s6e63m0_22gamma_240cd,
    s6e63m0_22gamma_250cd,
    s6e63m0_22gamma_260cd,// 20
    s6e63m0_22gamma_270cd,
    s6e63m0_22gamma_280cd,
    s6e63m0_22gamma_290cd,
    s6e63m0_22gamma_300cd,// 24
};

struct setting_table *p19Gamma_set[] = {
    NULL,// display off
	s6e63m0_19gamma_30cd,// 1
	s6e63m0_19gamma_40cd,
	s6e63m0_19gamma_70cd,
	s6e63m0_19gamma_90cd,
	s6e63m0_19gamma_100cd,// 5
	s6e63m0_19gamma_110cd,
	s6e63m0_19gamma_120cd,
	s6e63m0_19gamma_130cd,
	s6e63m0_19gamma_140cd,
	s6e63m0_19gamma_150cd,// 10
	s6e63m0_19gamma_170cd,
	s6e63m0_19gamma_180cd,
	s6e63m0_19gamma_190cd,
	s6e63m0_19gamma_200cd,
	s6e63m0_19gamma_210cd,// 15
	s6e63m0_19gamma_220cd,
	s6e63m0_19gamma_230cd,
	s6e63m0_19gamma_240cd,
	s6e63m0_19gamma_250cd,
	s6e63m0_19gamma_260cd,// 20
	s6e63m0_19gamma_270cd,
	s6e63m0_19gamma_280cd,
	s6e63m0_19gamma_290cd,
	s6e63m0_19gamma_300cd,//24
};

#if defined(CONFIG_MACH_APACHE)

struct setting_table *p22Gamma_set_new_hw[] = {
    NULL,// display off
    s6e63m0_22gamma_30cd_new_hw,// 1
    s6e63m0_22gamma_40cd_new_hw,
    s6e63m0_22gamma_70cd_new_hw,
    s6e63m0_22gamma_90cd_new_hw,
    s6e63m0_22gamma_100cd_new_hw,// 5
    s6e63m0_22gamma_110cd_new_hw,
    s6e63m0_22gamma_120cd_new_hw,
    s6e63m0_22gamma_130cd_new_hw,
    s6e63m0_22gamma_140cd_new_hw,
    s6e63m0_22gamma_150cd_new_hw,// 10
    s6e63m0_22gamma_170cd_new_hw,
    s6e63m0_22gamma_180cd_new_hw,
    s6e63m0_22gamma_190cd_new_hw,
    s6e63m0_22gamma_200cd_new_hw,
    s6e63m0_22gamma_210cd_new_hw,// 15
    s6e63m0_22gamma_220cd_new_hw,
    s6e63m0_22gamma_230cd_new_hw,
    s6e63m0_22gamma_240cd_new_hw,
    s6e63m0_22gamma_250cd_new_hw,
    s6e63m0_22gamma_260cd_new_hw,// 20
    s6e63m0_22gamma_270cd_new_hw,
    s6e63m0_22gamma_280cd_new_hw,
    s6e63m0_22gamma_290cd_new_hw,
    s6e63m0_22gamma_300cd_new_hw,// 24
};

struct setting_table *p19Gamma_set_new_hw[] = {
    NULL,// display off
	s6e63m0_19gamma_30cd_new_hw,// 1
	s6e63m0_19gamma_40cd_new_hw,
	s6e63m0_19gamma_70cd_new_hw,
	s6e63m0_19gamma_90cd_new_hw,
	s6e63m0_19gamma_100cd_new_hw,// 5
	s6e63m0_19gamma_110cd_new_hw,
	s6e63m0_19gamma_120cd_new_hw,
	s6e63m0_19gamma_130cd_new_hw,
	s6e63m0_19gamma_140cd_new_hw,
	s6e63m0_19gamma_150cd_new_hw,// 10
	s6e63m0_19gamma_170cd_new_hw,
	s6e63m0_19gamma_180cd_new_hw,
	s6e63m0_19gamma_190cd_new_hw,
	s6e63m0_19gamma_200cd_new_hw,
	s6e63m0_19gamma_210cd_new_hw,// 15
	s6e63m0_19gamma_220cd_new_hw,
	s6e63m0_19gamma_230cd_new_hw,
	s6e63m0_19gamma_240cd_new_hw,
	s6e63m0_19gamma_250cd_new_hw,
	s6e63m0_19gamma_260cd_new_hw,// 20
	s6e63m0_19gamma_270cd_new_hw,
	s6e63m0_19gamma_280cd_new_hw,
	s6e63m0_19gamma_290cd_new_hw,
	s6e63m0_19gamma_300cd_new_hw,//24
};

#endif
#ifdef CONFIG_USES_ACL
static struct setting_table *ACL_cutoff_set[] = {
    acl_cutoff_off, //0
    acl_cutoff_8p,
    acl_cutoff_14p,
    acl_cutoff_20p,
    acl_cutoff_24p,
    acl_cutoff_28p, //5
    acl_cutoff_32p,
    acl_cutoff_35p,
    acl_cutoff_37p,
    acl_cutoff_40p, //9
    acl_cutoff_45p, //10
    acl_cutoff_47p, //11
    acl_cutoff_48p, //12
    acl_cutoff_50p, //13
    acl_cutoff_60p, //14
    acl_cutoff_75p, //15
    acl_cutoff_43p, //16
};
#endif /* CONFIG_USES_ACL */

struct brightness_level brt_table[] = {
    { 0, 5 },// Off
    { 20, 1 },// Dimming pulse
    { MIN_BRIGHTNESS_VALUE,1 },// Min
    { 39, 2},
    { 49, 3},
    { 59, 4},
    { 69, 5},
    { 78, 6},
    { 88, 7},
    { 105, 8},
    //{ 108,  8 },
    { 118, 9},// default
    { 127, 10 },
    { 137, 11 },
    { 147, 12 },
    { 157, 13 },
    { 166, 14 },
    { 176, 15 },
    { 186, 16 },
    { 196, 17 },
    { 206, 18 },
    { 215, 19 },
    { 225, 20 },
    { 235, 21 },
    { 245, 22 },
    { MAX_BRIGHTNESS_VALUE, 23 },// Max
};

static struct s6e63m0_state_type s6e63m0_state = {
    .disp_initialized = FALSE,
    .display_on = FALSE,
    .disp_powered_up = FALSE,
};

static int lcdc_s6e63m0_get_ldi_state(void)
{
    return ldi_enable;
}

static void lcdc_s6e63m0_set_ldi_state(int OnOff)
{
    ldi_enable = OnOff;
}

static void lcdc_s6e63m0_write_no_spinlock(struct setting_table *table)
{
    long i, j;
    
    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP);

    /* Write Command */
    LCD_CSX_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SCL_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SDI_LOW
    udelay(DEFAULT_USLEEP);
    
    LCD_SCL_HIGH
    udelay(DEFAULT_USLEEP);

    for (i = 7; i >= 0; i--) {
        LCD_SCL_LOW
        udelay(DEFAULT_USLEEP);
        if ((table->command >> i) & 0x1)
            LCD_SDI_HIGH
        else
            LCD_SDI_LOW
        udelay(DEFAULT_USLEEP);
        LCD_SCL_HIGH
        udelay(DEFAULT_USLEEP);
    }

    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);

    /* Write Parameter */
    if ((table->parameters) > 0) {
        for (j = 0; j < table->parameters; j++) {
            LCD_CSX_LOW 
            udelay(DEFAULT_USLEEP);
            
            LCD_SCL_LOW
            udelay(DEFAULT_USLEEP);
            LCD_SDI_HIGH
            udelay(DEFAULT_USLEEP);
            LCD_SCL_HIGH
            udelay(DEFAULT_USLEEP);

            for (i = 7; i >= 0; i--) {
                LCD_SCL_LOW
                udelay(DEFAULT_USLEEP);
                if ((table->parameter[j] >> i) & 0x1)
                    LCD_SDI_HIGH
                else
                    LCD_SDI_LOW
                udelay(DEFAULT_USLEEP);
                LCD_SCL_HIGH
                udelay(DEFAULT_USLEEP);
            }

            LCD_CSX_HIGH
            udelay(DEFAULT_USLEEP);
        }
    }
    mdelay(table->wait);
}

static void lcdc_s6e63m0_write(struct setting_table *table)
{
    long i, j;
    unsigned long irqflags;

    spin_lock_irqsave(&lcd_ctrl_irq_lock, irqflags);
    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP);

    /* Write Command */
    LCD_CSX_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SCL_LOW 
    udelay(DEFAULT_USLEEP);
    LCD_SDI_LOW 
    udelay(DEFAULT_USLEEP);
    
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP); 

       for (i = 7; i >= 0; i--) {
        LCD_SCL_LOW
        udelay(DEFAULT_USLEEP);
        if ((table->command >> i) & 0x1)
            LCD_SDI_HIGH
        else
            LCD_SDI_LOW
        udelay(DEFAULT_USLEEP);
        LCD_SCL_HIGH
        udelay(DEFAULT_USLEEP);
    }

    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);

    /* Write Parameter */
    if ((table->parameters) > 0) {
        for (j = 0; j < table->parameters; j++) {
            LCD_CSX_LOW 
            udelay(DEFAULT_USLEEP);
            
            LCD_SCL_LOW 
            udelay(DEFAULT_USLEEP);
            LCD_SDI_HIGH 
            udelay(DEFAULT_USLEEP);
            LCD_SCL_HIGH 
            udelay(DEFAULT_USLEEP);

            for (i = 7; i >= 0; i--) {
                LCD_SCL_LOW
                udelay(DEFAULT_USLEEP);
                if ((table->parameter[j] >> i) & 0x1)
                    LCD_SDI_HIGH
                else
                    LCD_SDI_LOW
                udelay(DEFAULT_USLEEP);
                LCD_SCL_HIGH
                udelay(DEFAULT_USLEEP);
            }

            LCD_CSX_HIGH
            udelay(DEFAULT_USLEEP);
        }
    }
    spin_unlock_irqrestore(&lcd_ctrl_irq_lock, irqflags);
    mdelay(table->wait);
}

static void lcdc_s6e63m0_spi_init(void)
{
    /* Setting the Default GPIO's */
    spi_sclk = *(lcdc_s6e63m0_pdata->gpio_num);
    spi_cs   = *(lcdc_s6e63m0_pdata->gpio_num + 1);
    spi_sdi  = *(lcdc_s6e63m0_pdata->gpio_num + 2);
    lcd_reset= *(lcdc_s6e63m0_pdata->gpio_num + 3);

    /* Set the output so that we dont disturb the slave device */
    gpio_set_value(spi_sclk, 0);
    gpio_set_value(spi_sdi, 0);

    /* Set the Chip Select De-asserted */
    gpio_set_value(spi_cs, 0);

}

static void lcdc_s6e63m0_vreg_config(int on)
{
#if 0
    int rc = 0;
    struct vreg *vreg_ldo15, *vreg_ldo17 = NULL;
    
    // VREG_LCD_2.8V
    vreg_ldo15 = vreg_get(NULL, "gp6");
    if (IS_ERR(vreg_ldo15)) {
        rc = PTR_ERR(vreg_ldo15);
        pr_err("%s: gp6 vreg get failed (%d)\n",
               __func__, rc);
        return rc;
    }
    
    // VREG_LCD_1.8V
    vreg_ldo17 = vreg_get(NULL, "gp11");
    if (IS_ERR(vreg_ldo17)) {
        rc = PTR_ERR(vreg_ldo17);
        pr_err("%s: gp9 vreg get failed (%d)\n",
               __func__, rc);
        return rc;
    }

    rc = vreg_set_level(vreg_ldo15, 3000);
    if (rc) {
        pr_err("%s: vreg LDO15 set level failed (%d)\n",
               __func__, rc);
        return rc;
    }

    rc = vreg_set_level(vreg_ldo17, 1800);
    if (rc) {
        pr_err("%s: vreg LDO17 set level failed (%d)\n",
               __func__, rc);
        return rc;
    }

    if (on)
        rc = vreg_enable(vreg_ldo17);
    else
        rc = vreg_disable(vreg_ldo17);

    if (rc) {
        pr_err("%s: LDO17 vreg enable failed (%d)\n",
               __func__, rc);
        return rc;
    }

    if (on)
        rc = vreg_enable(vreg_ldo15);
    else
        rc = vreg_disable(vreg_ldo15);

    if (rc) {
        pr_err("%s: LDO15 vreg enable failed (%d)\n",
               __func__, rc);
        return rc;
    }

    mdelay(5);        /* ensure power is stable */

    return rc;
#endif    
}

static void lcdc_s6e63m0_disp_powerup(void)
{
    DPRINT("start %s\n", __func__);

    if (!s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {
        /* Reset the hardware first */
        lcdc_s6e63m0_vreg_config(VREG_ENABLE);
        //TODO: turn on ldo
        gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);

        //LCD_RESET_N_HI
        gpio_set_value(lcd_reset, 1);
        mdelay(10);
        //LCD_RESET_N_LO
        gpio_set_value(lcd_reset, 0);
        mdelay(20);
        //LCD_RESET_N_HI
        gpio_set_value(lcd_reset, 1);
        mdelay(10);

        /* Include DAC power up implementation here */
        s6e63m0_state.disp_powered_up = TRUE;
    }
}

static void lcdc_s6e63m0_disp_powerdown(void)
{
    DPRINT("start %s\n", __func__);

    /* Reset Assert */
    gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
    gpio_set_value(lcd_reset, 0);        
    lcdc_s6e63m0_vreg_config(VREG_DISABLE);
    mdelay(10);        /* ensure power is stable */

    /* turn off LDO */
    //TODO: turn off LDO

    s6e63m0_state.disp_powered_up = FALSE;
}

static void lcdc_s6e63m0_disp_on(void)
{
    DPRINT("start %s\n", __func__);

    if (s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {
        //mdelay(20);
        S6E63M0_WRITE_LIST(power_on_sequence);
        s6e63m0_state.display_on = TRUE;
    }
}

static int lcdc_s6e63m0_get_gamma_value_from_bl(int bl)
{
    int gamma_value = 0;
    int gamma_val_x10 = 0;

    if(bl >= MIN_BL) {
        gamma_val_x10 = 10*(MAX_GAMMA_VALUE-1)*bl/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));
        gamma_value = (gamma_val_x10+5)/10;
    }else{
        gamma_value = 0;
    }

    return gamma_value;
}

static void lcdc_s6e63m0_set_brightness(int level)
{

#if defined(CONFIG_MACH_APACHE)

	if(board_hw_revision == 0 ) {
		
			if(level){
			        if(on_19gamma)
			            lcdc_s6e63m0_write(p19Gamma_set[level]);
			        else
			            lcdc_s6e63m0_write(p22Gamma_set[level]);

			   lcdc_s6e63m0_write(gamma_update);
		        }
	} else {
			if(level){
			        if(on_19gamma)
			            lcdc_s6e63m0_write(p19Gamma_set_new_hw[level]);
			        else
			            lcdc_s6e63m0_write(p22Gamma_set_new_hw[level]);

			   lcdc_s6e63m0_write(gamma_update);
		        }
	}
		
#else
    if(level){
        if(on_19gamma)
            lcdc_s6e63m0_write(p19Gamma_set[level]);
        else
            lcdc_s6e63m0_write(p22Gamma_set[level]);

	   lcdc_s6e63m0_write(gamma_update);
    }
#endif

    //lcdc_s6e63m0_write(display_on_seq);
    DPRINT("brightness: %d on_19gamma: %d\n",level,on_19gamma);
}

static ssize_t s6e63m0_show_power(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", s6e63m0_state.display_on );
}

static ssize_t s6e63m0_store_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#if 1
    char *endp;
    int power = simple_strtoul(buf, &endp, 0);    

    DPRINT("s6e63m0_store_power is called: %d", power);

    if (power == 1)
        lcdc_s6e63m0_panel_on((struct platform_device*) dev);
    else if(power == 0)
        lcdc_s6e63m0_panel_off((struct platform_device*) dev);
    else if(power == 2){
        lcdc_s6e63m0_write(disp_on_sequence);
#if defined(CONFIG_MACH_APACHE)
	if(board_hw_revision == 0)
		 lcdc_s6e63m0_write(p22Gamma_set[20]);
	else
		  lcdc_s6e63m0_write(p22Gamma_set_new_hw[20]);
		
#else
        lcdc_s6e63m0_write(p22Gamma_set[20]);
#endif
        lcdc_s6e63m0_write(gamma_update);
    }

#endif
    return 0;
}

static ssize_t s6e63m0_show_lcd_type(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    
    if(lcd_type == 0)
        count = sprintf(buf, "SMD_S6E63M0\n");
        
    return count;
}

#ifdef CONFIG_USES_ACL
static void lcdc_s6e63m0_set_acl_parameter(int gamma)
{
    DPRINT("lcdc_s6e63m0_set_acl_parameter (gamma:%d)\n",gamma);
#if 0  
    /* dimming of display off (ACL off)*/
    if(!gamma){
        if(cur_acl != 0){
            S6E63M0_WRITE_LIST(acl_cutoff_off); //set 0% ACL
            cur_acl = 0;
            DPRINT("ACL_cutoff_set Percentage : 0!!\n");
        }
        return;
    }
#endif
    /* ACL on*/
	if(acl_enable)
	{
	    if((cur_acl == 0) && (gamma != 1)){
	        S6E63M0_WRITE_LIST(acl_cutoff_init);
	        mdelay(20);
	    }
	    DPRINT("lcdc_s6e63m0_set_acl_parameter (cur_acl:%d)\n",cur_acl);

	    switch (gamma){
	        case 1:
	        case 2:
	            if (cur_acl != 0){
	                S6E63M0_WRITE_LIST(acl_cutoff_off); //set 0% ACL
	                cur_acl = 0;
	                DPRINT("ACL_cutoff_set Percentage : 0!!\n");
	            }
	            break;
	        case 3:
	        case 4:
	        case 5:
	        case 6:
	        case 7:
	        case 8:
	        case 9:
	        case 10:
	        case 11:
	        case 12:
	            if (cur_acl != 40){
	                S6E63M0_WRITE_LIST(acl_cutoff_40p);
	                cur_acl = 40;
	                DPRINT("ACL_cutoff_set Percentage : 40!!\n");
	            }
	            break;
	        case 13:
	            if (cur_acl != 43){
	                S6E63M0_WRITE_LIST(acl_cutoff_43p);
	                cur_acl = 43;
	                DPRINT("ACL_cutoff_set Percentage : 43!!\n");
	            }
	            break;
	        case 14:
	            if (cur_acl != 45){
	                S6E63M0_WRITE_LIST(acl_cutoff_45p);
	                cur_acl = 45;
	                DPRINT("ACL_cutoff_set Percentage : 45!!\n");
	            }
	            break;
	        case 15:
	            if (cur_acl != 47){
	                S6E63M0_WRITE_LIST(acl_cutoff_47p);
	                cur_acl = 47;
	                DPRINT("ACL_cutoff_set Percentage : 47!!\n");
	            }
	            break;
	        case 16:
	            if (cur_acl != 48){
	                S6E63M0_WRITE_LIST(acl_cutoff_48p);
	                cur_acl = 48;
	                DPRINT("ACL_cutoff_set Percentage : 48!!\n");
	            }
	            break;
	        default:
	            if (cur_acl !=50){
	                S6E63M0_WRITE_LIST(acl_cutoff_50p);
	                cur_acl = 50;
	                DPRINT("ACL_cutoff_set Percentage : 50!!\n");
	            }
	    }
	}
	else
	{
	       if(cur_acl != 0){
	            S6E63M0_WRITE_LIST(acl_cutoff_off); //set 0% ACL
	            cur_acl = 0;
	            DPRINT("ACL_cutoff_set Percentage : 0!!\n");
	        }
	}
	    return;
}

static ssize_t aclset_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    DPRINT("called %s \n",__func__);
    return sprintf(buf,"%u\n", acl_enable);
}

static ssize_t aclset_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value;
    sscanf(buf, "%d", &value);
    DPRINT("in aclset_file_cmd_store, input value = %d \n", value);

    if (!lcdc_s6e63m0_get_ldi_state()){
        DPRINT("return because LDI is disabled, input value = %d \n", value);
        return size;
    }

    if ((value != 0) && (value != 1)){
        DPRINT("aclset_file_cmd_store value is same : value(%d)\n", value);
        return size;
    }

    if (acl_enable != value){
        acl_enable = value;
#if 1
            lcdc_s6e63m0_set_acl_parameter(current_gamma_lvl);
#else
        if(acl_enable)
            lcdc_s6e63m0_set_acl_parameter(current_gamma_lvl);
        else 
            lcdc_s6e63m0_set_acl_parameter(0);

#endif

	}

    return size;
}
#endif

#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
static ssize_t gammaset_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	DPRINT("called %s \n",__func__);

	return sprintf(buf,"%u\n", current_gamma_lvl);
}
static ssize_t gammaset_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
    sscanf(buf, "%d", &value);

	//printk(KERN_INFO "[gamma set] in gammaset_file_cmd_store, input value = %d \n",value);
	if (!lcdc_s6e63m0_get_ldi_state())	{
		DPRINT("[gamma set] return because LDI is disabled, input value = %d \n", value);
		return size;
	}

	if ((value != 0) && (value != 1))	{
		DPRINT("\ngammaset_file_cmd_store value(%d) on_19gamma(%d) \n", value,on_19gamma);
		return size;
	}

	if (value != on_19gamma)	{
		on_19gamma = value;
		lcdc_s6e63m0_set_brightness(current_gamma_lvl);
	}

	return size;
}
#endif

static int lcdc_s6e63m0_panel_on(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__);

    if (!s6e63m0_state.disp_initialized) {
        /* Configure reset GPIO that drives DAC */
        lcdc_s6e63m0_pdata->panel_config_gpio(1);
        lcdc_s6e63m0_spi_init();    /* LCD needs SPI */
        lcdc_s6e63m0_disp_powerup();
        lcdc_s6e63m0_disp_on();
        s6e63m0_state.disp_initialized = TRUE;
    }

    if(!First_Disp_Power_On)
    {
    	First_Disp_Power_On = TRUE;
	lcdc_s6e63m0_set_brightness(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);	
    }

    if(delayed_backlight_value != -1) {
        lcdc_s6e63m0_set_brightness(delayed_backlight_value);
        DPRINT("delayed backlight on %d\n", delayed_backlight_value);
    }

    return 0;
}

static int lcdc_s6e63m0_panel_off(struct platform_device *pdev)
{
    int i;
    unsigned long irqflags;

    DPRINT("start %s\n", __func__);

    if (s6e63m0_state.disp_powered_up && s6e63m0_state.display_on) {
        if (!lcdc_s6e63m0_get_ldi_state()) {
            spin_lock_irqsave(&lcd_ctrl_irq_lock, irqflags);
            for (i = 0; i < POWER_OFF_SEQ; i++) {
                lcdc_s6e63m0_write_no_spinlock(&power_off_sequence[i]);
            }
            spin_unlock_irqrestore(&lcd_ctrl_irq_lock, irqflags);
        } else {
            for (i = 0; i < POWER_OFF_SEQ; i++) {
                lcdc_s6e63m0_write(&power_off_sequence[i]);
            }
        }
        lcdc_s6e63m0_pdata->panel_config_gpio(0);
        lcdc_s6e63m0_disp_powerdown();
        s6e63m0_state.display_on = FALSE;
        s6e63m0_state.disp_initialized = FALSE;
    }
    return 0;
}

static void lcdc_s6e63m0_set_backlight(struct msm_fb_data_type *mfd)
{    
    int bl_level = mfd->bl_level;
    int gamma_level = 0;
    int i;

#if 1
    if(bl_level > 0){
        lcdc_s6e63m0_write(disp_on_sequence);
        if(bl_level < MIN_BRIGHTNESS_VALUE) {
            /* dimming set */
            gamma_level = brt_table[1].driver_level;
        } else if (bl_level == MAX_BRIGHTNESS_VALUE) {
            /* max brightness set */
            gamma_level = brt_table[MAX_BRT_STAGE-1].driver_level;
        } else {
            for(i = 0; i < MAX_BRT_STAGE; i++) {
                if(bl_level <= brt_table[i].platform_level ) {
                    gamma_level = brt_table[i].driver_level;
                    break;
                }
            }
        }
    } else {
        lcdc_s6e63m0_write(disp_off_sequence);
        DPRINT("bl: %d \n",bl_level);
        return;
    }

#else
    gamma_level = lcdc_s6e63m0_get_gamma_value_from_bl(bl_level);
#endif

    // LCD should be turned on prior to backlight
    if(s6e63m0_state.disp_initialized == FALSE && gamma_level > 0){
        delayed_backlight_value = gamma_level;
        DPRINT("delayed_backlight_value = gamma_level\n");
        return;
    } else {
        delayed_backlight_value = -1;
    }

    DPRINT("bl: %d, gamma: %d\n",bl_level,gamma_level);
    lcdc_s6e63m0_set_brightness(gamma_level);
#ifdef CONFIG_USES_ACL
    if(acl_enable)lcdc_s6e63m0_set_acl_parameter(gamma_level);
        current_gamma_lvl = gamma_level;
#endif
}

static int __init lcdc_s6e63m0_probe(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__);

    if (pdev->id == 0) {
        lcdc_s6e63m0_pdata = pdev->dev.platform_data;
        
        lcdc_s6e63m0_spi_init();
        if( !spi_sclk || !spi_cs || !spi_sdi || !lcd_reset)
        {
            DPRINT("SPI Init Error. %d,%d,%d,%d\n",spi_sclk,spi_cs,spi_sdi,lcd_reset);
            spi_cs = 46;
            spi_sclk = 45;
            spi_sdi = 47;
            lcd_reset = 129;
        }    

        /* sys fs */
        if(IS_ERR(sec_class))
            pr_err("Failed to create class(sec)!\n");
     
        lcd_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
        if(IS_ERR(lcd_dev))
            pr_err("Failed to create device(lcd)!\n");
     
        if(device_create_file(lcd_dev, &dev_attr_lcdtype_file_cmd) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_lcdtype_file_cmd.attr.name);
        if(device_create_file(lcd_dev, &dev_attr_lcd_power) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_lcd_power.attr.name); 

        lcdc_s6e63m0_set_ldi_state(1);
#ifdef CONFIG_USES_ACL
        DPRINT("making aclset sysfile start\n");
        acl_class = class_create(THIS_MODULE, "aclset");
        if (IS_ERR(acl_class))
            DPRINT("Failed to create class(acl_class)!\n");
    
        switch_aclset_dev = device_create(acl_class, NULL, 0, NULL, "switch_aclset");
        if (IS_ERR(switch_aclset_dev))
            DPRINT("Failed to create device(switch_aclset_dev)!\n");
    
        if (device_create_file(switch_aclset_dev, &dev_attr_aclset_file_cmd) < 0)
            DPRINT("Failed to create device file(%s)!\n", dev_attr_aclset_file_cmd.attr.name);
#endif
#ifdef GAMMASET_CONTROL
	gammaset_class = class_create(THIS_MODULE, "gammaset");
	if (IS_ERR(gammaset_class))
		DPRINT("Failed to create class(gammaset_class)!\n");

	switch_gammaset_dev = device_create(gammaset_class, NULL, 0, NULL, "switch_gammaset");
	if (IS_ERR(switch_gammaset_dev))
		DPRINT("Failed to create device(switch_gammaset_dev)!\n");

	if (device_create_file(switch_gammaset_dev, &dev_attr_gammaset_file_cmd) < 0)
		DPRINT("Failed to create device file(%s)!\n", dev_attr_gammaset_file_cmd.attr.name);
#endif
            return 0;
    }
    msm_fb_add_device(pdev);

    return 0;
}

static void lcdc_s6e63m0_shutdown(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__);
    lcdc_s6e63m0_set_ldi_state(0);
    current_gamma_lvl = -1;
    //lcdc_s6e63m0_panel_off(pdev);
    gpio_set_value(lcd_reset, 0);    
}

static struct platform_driver this_driver = {
    .probe = lcdc_s6e63m0_probe,
    .shutdown = lcdc_s6e63m0_shutdown,
    .driver = {
        .name   = "lcdc_s6e63m0_wvga",
        .owner  = THIS_MODULE,
    },
};

static struct msm_fb_panel_data s6e63m0_panel_data = {
    .on = lcdc_s6e63m0_panel_on,
    .off = lcdc_s6e63m0_panel_off,
    .set_backlight = lcdc_s6e63m0_set_backlight,
};

static struct platform_device this_device = {
    .name = "lcdc_panel",
    .id = 1,
    .dev = {
        .platform_data = &s6e63m0_panel_data,
    }
};

static int __init lcdc_s6e63m0_panel_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
    if (msm_fb_detect_client("lcdc_s6e63m0_wvga"))
    {
        DPRINT("%s: msm_fb_detect_client failed!\n", __func__);
        return 0;
    }
#endif
    DPRINT("start %s\n", __func__);
    
    ret = platform_driver_register(&this_driver);
    if (ret)
    {
        DPRINT("%s: platform_driver_register failed! ret=%d\n", __func__, ret);
        return ret;
    }

    pinfo = &s6e63m0_panel_data.panel_info;
    pinfo->xres = LCDC_FB_XRES;
    pinfo->yres = LCDC_FB_YRES;
    pinfo->type = LCDC_PANEL;
    pinfo->pdest = DISPLAY_1;
    pinfo->wait_cycle = 0;
    pinfo->bpp = 24;
    pinfo->fb_num = 2;
    pinfo->clk_rate = 24576* 1000;
    pinfo->bl_max = 255;
    pinfo->bl_min = 1;

    pinfo->lcdc.h_back_porch = LCDC_HBP;
    pinfo->lcdc.h_front_porch = LCDC_HFP;
    pinfo->lcdc.h_pulse_width = LCDC_HPW;
    pinfo->lcdc.v_back_porch = LCDC_VBP;
    pinfo->lcdc.v_front_porch = LCDC_VFP;
    pinfo->lcdc.v_pulse_width = LCDC_VPW;
    pinfo->lcdc.border_clr = 0;     /* blk */
    pinfo->lcdc.underflow_clr = 0xff0000;       /* red */
    pinfo->lcdc.hsync_skew = 0;

    DPRINT("%s\n", __func__);

    ret = platform_device_register(&this_device);
    if (ret)
    {
        DPRINT("%s: platform_device_register failed! ret=%d\n", __func__, ret);
        platform_driver_unregister(&this_driver);
    }

    return ret;
}

module_init(lcdc_s6e63m0_panel_init);


