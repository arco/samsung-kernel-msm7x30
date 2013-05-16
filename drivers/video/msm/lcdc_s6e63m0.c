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
#include <linux/mfd/pmic8058.h>
#include <linux/gpio.h>
#include "msm_fb.h"
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/miscdevice.h>

#include "lcdc_s6e63m0_seq.h"
#include "mdp4_video_enhance.h"

#define LCDC_DEBUG

//#define SMART_MTP //Ariesve display doesn't support, removed all smart_mtp code
#define MAPPING_TBL_AUTO_BRIGHTNESS 1

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk("s6e63m0 " x)
#else
#define DPRINT(x...)	
#endif

/*
 * Serial Interface
 */
#define LCD_CSX_HIGH	gpio_set_value(spi_cs, 1);
#define LCD_CSX_LOW	gpio_set_value(spi_cs, 0);

#define LCD_SCL_HIGH	gpio_set_value(spi_sclk, 1);
#define LCD_SCL_LOW		gpio_set_value(spi_sclk, 0);

#define LCD_SDI_HIGH	gpio_set_value(spi_sdi, 1);
#define LCD_SDI_LOW		gpio_set_value(spi_sdi, 0);

#define DEFAULT_USLEEP	1	

// brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define LOW_BRIGHTNESS_LEVEL 20
#define DEFAULT_GAMMA_LEVEL GAMMA_160CD
// static DEFINE_SPINLOCK(bl_ctrl_lock);

extern int board_hw_revision;
static struct device *lcd_dev;
extern struct class *sec_class;

struct s6e63m0 {
	struct device			*dev;
	struct spi_device		*spi;
	unsigned int			power;
	unsigned int			gamma_mode;
	unsigned int			goal_brightness;
	unsigned int			gamma_table_count;
	unsigned int			bl;
	unsigned int			beforepower;
	unsigned int			ldi_enable;
	unsigned int 			acl_enable;
	unsigned int 			cur_acl;	
	struct mutex	lock;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	struct lcd_platform_data	*lcd_pd;
	struct early_suspend    early_suspend;

#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	unsigned int			auto_brightness;
#endif
};

static struct s6e63m0 lcd;

/* 
*	PANEL_TYPE = SMART_MTP_PANEL_ID -> M2 PANEL
*/
#define ELVSS_MAX    0x28
const int ELVSS_OFFSET[] = {0xD, 0x09, 0x07, 0x0};
#define SMART_MTP_PANEL_ID 0xa4

static int PANEL_TYPE = 0;
static int PANLE_ELVSS_VALUE = 0;
static int lcdc_s6e63m0_panel_off(struct platform_device *pdev);

static int spi_cs;
static int spi_sclk;
static int spi_sdi;
static int lcd_reset;

static void lcdc_s6e63m0_set_brightness(int level);
static void s6e63m0_set_acl(struct s6e63m0 *lcd);
static void s6e63m0_set_elvss(struct s6e63m0 *lcd);

struct s6e63m0_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

volatile struct s6e63m0_state_type s6e63m0_state = { 0 };
static struct msm_panel_common_pdata *lcdc_s6e63m0_pdata;

static void setting_table_write(struct setting_table *table)
{
	long i, j;

	mutex_lock(&lcd.lock);

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

	mutex_unlock(&lcd.lock);

	msleep(table->wait);

}

static void lcdtool_write_byte(boolean dc, u8 data)
{
	uint32 bit;
	int bnum;

	LCD_SCL_LOW
	if(dc)
		LCD_SDI_HIGH
	else
		LCD_SDI_LOW
	udelay(1);			/* at least 20 ns */
	LCD_SCL_HIGH	/* clk high */
	udelay(1);			/* at least 20 ns */

	bnum = 8;			/* 8 data bits */
	bit = 0x80;
	while (bnum--) {
		LCD_SCL_LOW /* clk low */
		if(data & bit)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(1);
		LCD_SCL_HIGH /* clk high */
		udelay(1);
		bit >>= 1;
	}
	LCD_SDI_LOW

}


static void spi_read_id(u8 cmd, u8 *data, int num)
{
	int bnum;

	/* Chip Select - low */
	LCD_CSX_LOW
	udelay(2);

	/* command byte first */
	lcdtool_write_byte(0, cmd);
	udelay(2);

	gpio_direction_input(spi_sdi);

	if (num > 1) {
		/* extra dummy clock */
		LCD_SCL_LOW
		udelay(1);
		LCD_SCL_HIGH
		udelay(1);
	}

	/* followed by data bytes */
	bnum = num * 8;	/* number of bits */
	*data = 0;
	while (bnum) {
		LCD_SCL_LOW /* clk low */
		udelay(1);
		*data <<= 1;
		*data |= gpio_get_value(spi_sdi) ? 1 : 0;
		LCD_SCL_HIGH /* clk high */
		udelay(1);
		--bnum;
	}
	
	gpio_direction_output(spi_sdi, 0);

	/* Chip Select - high */
	udelay(2);
	LCD_CSX_HIGH

}
static void spi_init(void)
{
        DPRINT("start %s\n", __func__);	
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_s6e63m0_pdata->gpio_num);
	spi_cs   = *(lcdc_s6e63m0_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_s6e63m0_pdata->gpio_num + 2);
	 lcd_reset = *(lcdc_s6e63m0_pdata->gpio_num + 3); 
	DPRINT("clk : %d, cs : %d, sdi : %d\n", spi_sclk,spi_cs , spi_sdi);
	  
	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);
	
	DPRINT("end %s\n", __func__);	

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

    rc = vreg_set_level(vreg_ldo15, 2850);
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

static void s6e63m0_disp_powerup(void)
{
	DPRINT("start %s\n", __func__);	

	if (!s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {
		 /* Reset the hardware first */
		lcdc_s6e63m0_vreg_config(1);
        
        gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        
		gpio_set_value(lcd_reset, 1);
		msleep(2);
		gpio_set_value(lcd_reset, 0);
		msleep(2);
		gpio_set_value(lcd_reset, 1);

		msleep(20);

		s6e63m0_state.disp_powered_up = TRUE;
	}
	
}

static void s6e63m0_disp_powerdown(void)
{
	DPRINT("start %s\n", __func__);	
    /* Reset Assert */
    gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(lcd_reset, 0);        
    lcdc_s6e63m0_vreg_config(0);
    mdelay(10);        /* ensure power is stable */

	LCD_CSX_LOW
	LCD_SCL_LOW

	s6e63m0_state.disp_powered_up = FALSE;
}

int s6e63m0_read_lcd_id(void)
{

	u8 data;
	u8 idcheck[3];

	static int init_lcd_id=0;

	if(init_lcd_id)
	{
		DPRINT("LCD_PANEL_TYPE : %d\n",PANEL_TYPE);
		return 0;
	}

	gpio_request(spi_sdi, "s6e63m0_read_lcd_id");
	udelay(2);
	spi_read_id(0xda, &data, 1);
	idcheck[0] = data;

	spi_read_id(0xdb, &data, 1);
	idcheck[1] = data;	

	spi_read_id(0xdc, &data, 1);
	idcheck[2] = data;	

	PANEL_TYPE = idcheck[1];
	PANLE_ELVSS_VALUE = idcheck[2];
	
	DPRINT("%s: %x %x %x\n", __func__, idcheck[0], idcheck[1], idcheck[2]);
	
	init_lcd_id = 1;

	return 0;
}

void s6e63m0_disp_on(void)
{
	int i;
	
	DPRINT("start %s - HW Rev: %d\n", __func__,board_hw_revision);	

	if (s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {
		s6e63m0_read_lcd_id();
			
		for (i = 0; i < POWER_ON_SEQ_S6E63M0; i++)
		setting_table_write(&power_on_sequence_S6E63M0[i]);	
		
		// Sleep Out
		for (i = 0; i < SLEEP_OUT_SEQ; i++)
		setting_table_write(&sleep_out_display[i]);

		/* force ACL on */
		setting_table_write(ACL_cutoff_set[0]);
		setting_table_write(&SEQ_ACL_ON[0]);
		
		//set brightness
		lcdc_s6e63m0_set_brightness(lcd.goal_brightness);
		
		msleep(120);
		// Display On
		for (i = 0; i < DISPLAY_ON_SEQ; i++)
		setting_table_write(&display_on[i]);

		s6e63m0_state.display_on = TRUE;
	}
}

void s6e63m0_sleep_off(void)
{
	int i;

	DPRINT("start %s\n", __func__);	

	for (i = 0; i < POWER_ON_SEQ_2; i++)
		setting_table_write(&power_on_sequence_2[i]);	
	
	msleep(1);
}

void s6e63m0_sleep_in(void)
{
	int i;

	DPRINT("start %s\n", __func__); 

	for (i = 0; i < POWER_OFF_SEQ; i++)
		setting_table_write(&power_off_sequence[i]);			

	msleep(1);	
}

static int lcdc_s6e63m0_panel_on(struct platform_device *pdev)
{
	DPRINT("%s  +  (%d,%d,%d)\n", __func__, s6e63m0_state.disp_initialized, s6e63m0_state.disp_powered_up, s6e63m0_state.display_on);	
	
	if (!s6e63m0_state.disp_initialized) {
		/* Configure reset GPIO that drives DAC */
		lcdc_s6e63m0_pdata->panel_config_gpio(1);
		spi_init();	/* LCD needs SPI */
		s6e63m0_disp_powerup();
		s6e63m0_disp_on();
		s6e63m0_state.disp_initialized = TRUE;

		if(lcd.goal_brightness != lcd.bl)
		{
		        lcdc_s6e63m0_set_brightness(lcd.goal_brightness);
		}		
	}

	DPRINT("%s  -  (%d,%d,%d)\n", __func__,s6e63m0_state.disp_initialized, s6e63m0_state.disp_powered_up, s6e63m0_state.display_on);	

	return 0;
}

static int lcdc_s6e63m0_panel_off(struct platform_device *pdev)
{
	int i;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct fb_info *info;
	unsigned short *bits;
	DPRINT("%s : Draw Black screen.\n", __func__);	
	info = registered_fb[0];
	bits = (unsigned short *)(info->screen_base);
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER	
	memset(bits, 0x00, 800*480*4*3); /*info->var.xres*info->var.yres*/
#else
	memset(bits, 0x00, 800*480*4*2);
#endif

#endif 


	DPRINT("%s +  (%d,%d,%d)\n", __func__,s6e63m0_state.disp_initialized, s6e63m0_state.disp_powered_up, s6e63m0_state.display_on);	

	if (s6e63m0_state.disp_powered_up && s6e63m0_state.display_on) {
		for (i = 0; i < POWER_OFF_SEQ; i++)
			setting_table_write(&power_off_sequence[i]);
		lcdc_s6e63m0_pdata->panel_config_gpio(0);
		s6e63m0_state.display_on = FALSE;
		s6e63m0_state.disp_initialized = FALSE;
		s6e63m0_disp_powerdown();
	}
	
	DPRINT("%s -\n", __func__);	
	return 0;
}

static void s6e63m0_gamma_ctl(struct s6e63m0 *lcd)
{
	int tune_level = lcd->bl;

	setting_table_write(lcd_s6e63m0_table_22gamma[tune_level]);
	
	setting_table_write(&s6e63m0_gamma_update_enable[0]);
}

static void lcdc_s6e63m0_set_brightness(int level)
{
	// unsigned long irqflags;
	int tune_level = level;

	//spin_lock_irqsave(&bl_ctrl_lock, irqflags);

	lcd.bl = tune_level;

	s6e63m0_set_elvss(&lcd);

	s6e63m0_set_acl(&lcd);

	s6e63m0_gamma_ctl(&lcd);	

	//spin_unlock_irqrestore(&bl_ctrl_lock, irqflags);

}

#define DIM_BL 20
#define MIN_BL 30
#define MAX_BL 255
#define MAX_GAMMA_VALUE 25

static int get_gamma_value_from_bl(int bl)
{
	int gamma_value =0;
	int gamma_val_x10 =0;
	
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	if (unlikely(!lcd.auto_brightness && bl > 250)) bl = 250;

	/* brightness setting from platform is from 0 to 255
	 * But in this driver, brightness is
	  only supported from 0 to 24 */

	switch (bl) {
	case 0 ... 29:
		gamma_value = GAMMA_20CD;
		break;
	case 30 ... 39:
		gamma_value = GAMMA_30CD;
		break;
	case 40 ... 49:
		gamma_value = GAMMA_40CD;
		break;
	case 50 ... 59:
		gamma_value = GAMMA_50CD;
		break;
	case 60 ... 69:
		gamma_value = GAMMA_60CD;
		break;
	case 70 ... 79:
		gamma_value = GAMMA_70CD;
		break;
	case 80 ... 89:
		gamma_value = GAMMA_80CD;
		break;
	case 90 ... 99:
		gamma_value = GAMMA_90CD;
		break;
	case 100 ... 109:
		gamma_value = GAMMA_100CD;
		break;
	case 110 ... 119:
		gamma_value = GAMMA_110CD;
		break;
	case 120 ... 129:
		gamma_value = GAMMA_120CD;
		break;
	case 130 ... 139:
		gamma_value = GAMMA_130CD;
		break;
	case 140 ... 149:
		gamma_value = GAMMA_140CD;
		break;
	case 150 ... 159:
		gamma_value = GAMMA_150CD;
		break;
	case 160 ... 169:
		gamma_value = GAMMA_160CD;
		break;
	case 170 ... 179:
		gamma_value = GAMMA_170CD;
		break;
	case 180 ... 189:
		gamma_value = GAMMA_180CD;
		break;
	case 190 ... 199:
		gamma_value = GAMMA_190CD;
		break;
	case 200 ... 209:
		gamma_value = GAMMA_200CD;
		break;
	case 210 ... 219:
		gamma_value = GAMMA_210CD;
		break;
	case 220 ... 229:
		gamma_value = GAMMA_220CD;
		break;
	case 230 ... 239:
		gamma_value = GAMMA_230CD;
		break;
	case 240 ... 249:
		gamma_value = GAMMA_240CD;
		break;
	case 250 ... 254:
		gamma_value = GAMMA_250CD;
		break;
	case 255:
		gamma_value = GAMMA_300CD;
		break;
	default:
		gamma_value = DEFAULT_GAMMA_LEVEL;
		break;
	}

	DPRINT("%s >>> bl_value:%d, gamma_value: %d. \n ",__func__,bl,gamma_value);	
#else

	if(bl >= MIN_BL){
		gamma_val_x10 = 10 *(MAX_GAMMA_VALUE-1)*bl/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));
		gamma_value=(gamma_val_x10 +5)/10;
	}else{
		gamma_value =0;
	}
#endif	
	return gamma_value;
}
static void lcdc_s6e63m0_set_backlight(struct msm_fb_data_type *mfd)
{	
	int bl_level = mfd->bl_level;
	int tune_level;

	tune_level = get_gamma_value_from_bl(bl_level);

	if(s6e63m0_state.disp_initialized)
		DPRINT("brightness!!! bl_level=%d, tune_level=%d goal=%d\n",bl_level,tune_level,lcd.goal_brightness);
	
 	if ((s6e63m0_state.disp_initialized) 
		&& (s6e63m0_state.disp_powered_up) 
		&& (s6e63m0_state.display_on))
	{
		if(lcd.bl != tune_level)
		{
			lcdc_s6e63m0_set_brightness(tune_level);
		}
	}
	else
	{
		lcd.goal_brightness = tune_level;
	}

}

/////////////////////// sysfs
struct class *sysfs_lcd_class;
struct device *sysfs_panel_dev;
#if 0
static int s6e63m0_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int s6e63m0_set_brightness(struct backlight_device *bd)
{
	int ret = 0, bl = bd->props.brightness;
//	struct s6e63m0 *lcd = bl_get_data(bd);

	if (bl < LOW_BRIGHTNESS_LEVEL ||
		bl > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d. now %d\n",
			LOW_BRIGHTNESS_LEVEL, MAX_BRIGHTNESS_LEVEL, bl);
		return -EINVAL;
	}
	return ret;
}
#endif
struct setting_table SEQ_DYNAMIC_ELVSS = {
	0xB2, 4, 
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	0
};

struct setting_table SEQ_DYNAMIC_ELVSS_ON = {
	0xB1, 1, 
	{0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	0
};


void dynamic_elvss_cal(int step)
{
	int data;

	data = PANLE_ELVSS_VALUE + ELVSS_OFFSET[step];
	
	if (data > ELVSS_MAX)
		data = ELVSS_MAX;

	printk("%s data:%d\n",__func__,data);
	
	SEQ_DYNAMIC_ELVSS.parameter[0] = data;
	SEQ_DYNAMIC_ELVSS.parameter[1] = data;
	SEQ_DYNAMIC_ELVSS.parameter[2] = data;
	SEQ_DYNAMIC_ELVSS.parameter[3] = data;
	
	setting_table_write(&SEQ_DYNAMIC_ELVSS);
	setting_table_write(&SEQ_DYNAMIC_ELVSS_ON);
}


static void s6e63m0_set_elvss(struct s6e63m0 *lcd)
{
	if( (PANEL_TYPE >= SMART_MTP_PANEL_ID) && (PANLE_ELVSS_VALUE)) {
		switch (lcd->bl) {
		case 0 ... 7: /* 30cd ~ 100cd */
			dynamic_elvss_cal(0);
			break;
		case 8 ... 14: /* 110cd ~ 160cd */
			dynamic_elvss_cal(1);
			break;
		case 15 ... 18: /* 170cd ~ 200cd */
			dynamic_elvss_cal(2);
			break;
		case 19 ... 27: /* 210cd ~ 300cd */
			dynamic_elvss_cal(3);
			break;
		default:
			break;
		}
	} else {
		switch (lcd->bl) {
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
			case GAMMA_30CD ... GAMMA_100CD: /* 30cd ~ 100cd */
				setting_table_write(SEQ_ELVSS_set[0]);
				break;
			case GAMMA_110CD ... GAMMA_160CD: /* 110cd ~ 160cd */
				setting_table_write(SEQ_ELVSS_set[1]);
				break;
			case GAMMA_170CD ... GAMMA_200CD: /* 170cd ~ 200cd */
				setting_table_write(SEQ_ELVSS_set[2]);
				break;
			case GAMMA_210CD ... GAMMA_300CD: /* 210cd ~ 300cd */
				setting_table_write(SEQ_ELVSS_set[3]);
				break;
			default:
				break;
#else
			case 0 ... 7: /* 30cd ~ 100cd */
				setting_table_write(SEQ_ELVSS_set[0]);
				break;
			case 8 ... 13: /* 110cd ~ 160cd */
				setting_table_write(SEQ_ELVSS_set[1]);
				break;
			case 14 ... 17: /* 170cd ~ 200cd */
				setting_table_write(SEQ_ELVSS_set[2]);
				break;
			case 18 ... 27: /* 210cd ~ 300cd */
				setting_table_write(SEQ_ELVSS_set[3]);
				break;
			default:
				break;
#endif
		}
	}
}


static void s6e63m0_set_acl(struct s6e63m0 *lcd)
{
	int ret = 0;

	if (lcd->acl_enable) {	
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
		switch (lcd->bl) {
		case GAMMA_20CD ... GAMMA_40CD: /* 30cd ~ 40cd */
			if (lcd->cur_acl != 0) {
			setting_table_write(ACL_cutoff_set[0]);
			DPRINT("ACL_cutoff_set Percentage : off!!\n");
			lcd->cur_acl = 0;
			}
			break;
			
		case GAMMA_50CD ... GAMMA_180CD: /* 50cd ~ 180cd */
			if (lcd->cur_acl != 40) {
			setting_table_write(ACL_cutoff_set[1]);
			DPRINT("ACL_cutoff_set Percentage : 40!!\n");
			lcd->cur_acl = 40;
			}
			break;

		case GAMMA_190CD: /* 190cd */
			if (lcd->cur_acl != 43) {
			setting_table_write(ACL_cutoff_set[2]);
			DPRINT("ACL_cutoff_set Percentage : 43!!\n");
			lcd->cur_acl = 43;
			}
			break;
		case GAMMA_200CD: /* 200cd */
			if (lcd->cur_acl != 45) {
			setting_table_write(ACL_cutoff_set[3]);
			DPRINT("ACL_cutoff_set Percentage : 45!!\n");
			lcd->cur_acl = 45;
			}
			break;
		case GAMMA_210CD: /* 210cd */
			if (lcd->cur_acl != 47) {
			setting_table_write(ACL_cutoff_set[4]);
			DPRINT("ACL_cutoff_set Percentage : 47!!\n");
			lcd->cur_acl = 47;
			}
			break;
		case GAMMA_220CD: /* 220cd */
			if (lcd->cur_acl != 48) {
			setting_table_write(ACL_cutoff_set[5]);
			DPRINT("ACL_cutoff_set Percentage : 48!!\n");
			lcd->cur_acl = 48;
			}
			break;
		default: /* 300cd */
			if (lcd->cur_acl != 50) {
			setting_table_write(ACL_cutoff_set[6]);
			DPRINT("ACL_cutoff_set Percentage : 50!!\n");
			lcd->cur_acl = 50;
			}
			break;
		}
#else
		switch (lcd->bl) {
		case 0 ... 3: /* 30cd ~ 60cd */
			if (lcd->cur_acl != 0) {
			setting_table_write(ACL_cutoff_set[0]);
			DPRINT("ACL_cutoff_set Percentage : off!!\n");
			lcd->cur_acl = 0;
			}
			break;
		case 4 ... 24: /* 70cd ~ 250 */
			if (lcd->cur_acl != 40) {
			setting_table_write(ACL_cutoff_set[1]);
			DPRINT("ACL_cutoff_set Percentage : 40!!\n");
			lcd->cur_acl = 40;
			}
			break;	
		default: /* 300cd */
			if (lcd->cur_acl != 50) {
			setting_table_write(ACL_cutoff_set[6]);
			DPRINT("ACL_cutoff_set Percentage : 50!!\n");
			lcd->cur_acl = 50;
			}
			break;
		}
#endif
	}
	else{
			setting_table_write(ACL_cutoff_set[0]);
			lcd->cur_acl = 0;
			DPRINT("ACL_cutoff_set Percentage : off!!\n");
	}

	if (ret) {
		DPRINT("failed to initialize ldi.\n");
	}
}

static ssize_t power_reduce_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
//	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", lcd.acl_enable);
	strcpy(buf, temp);

	return strlen(buf);
}

static ssize_t power_reduce_store(struct device *dev, struct 
device_attribute *attr, const char *buf, size_t size)
{
	//	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	DPRINT("power_reduce_store : %d\n", value);	
	if (rc < 0)
		return rc;
	else{
		if (lcd.acl_enable != value) {
			lcd.acl_enable = value;
			s6e63m0_set_acl(&lcd);
		}
	return size;
	}
}

static DEVICE_ATTR(power_reduce, 0664,
		power_reduce_show, power_reduce_store);


static ssize_t lcd_type_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{

	char temp[15];
	sprintf(temp, "SMD_S6E63M0\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(lcd_type, 0664,
		lcd_type_show, NULL);
#if 0
static ssize_t octa_lcd_type_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
	char temp[15];
	
	if(PANEL_TYPE)
	{
		sprintf(temp, "OCTA : M3\n");
	}
	else
	{
		sprintf(temp, "OCTA : M2\n");
	}
	strcat(buf, temp);
	return strlen(buf);

}

static DEVICE_ATTR(octa_lcdtype, 0664,
		octa_lcd_type_show, NULL);
#endif
#if 0
static ssize_t s6e63m0_sysfs_show_gamma_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
//	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	char temp[10];

	switch (lcd.gamma_mode) {
	case 0:
		sprintf(temp, "2.2 mode\n");
		strcat(buf, temp);
		break;
	case 1:
		sprintf(temp, "1.9 mode\n");
		strcat(buf, temp);
		break;
	default:
		dev_info(dev, "gamma mode could be 0:2.2, 1:1.9 or 2:1.7)n");
		break;
	}

	return strlen(buf);
}

static ssize_t s6e63m0_sysfs_store_gamma_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
//	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	int rc;

	dev_info(dev, "s6e63m0_sysfs_store_gamma_mode\n");

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd.gamma_mode);
	printk(KERN_ERR "store_gamma_mode (0:2.2, 1:1.9) %d\n", lcd.gamma_mode);
	if (rc < 0)
		return rc;
	
#ifdef LCDC_19GAMMA_ENABLE
	s6e63m0_gamma_ctl(&lcd);  
#endif
	return len;
}

static DEVICE_ATTR(gamma_mode, 0664,
		s6e63m0_sysfs_show_gamma_mode, s6e63m0_sysfs_store_gamma_mode);

static ssize_t s6e63m0_sysfs_show_gamma_table(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct s6e63m0 *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", lcd->gamma_table_count);
	strcpy(buf, temp);

	return strlen(buf);
}

static DEVICE_ATTR(gamma_table, 0664,
		s6e63m0_sysfs_show_gamma_table, NULL);
#endif

static int s6e63m0_power(struct s6e63m0 *lcd, int power)
{
	int ret = 0;
	int i;

	if(power == FB_BLANK_UNBLANK)
	{
		DPRINT("s6e63m0_power : UNBLANK\n");
		/* Configure reset GPIO that drives DAC */
		lcdc_s6e63m0_pdata->panel_config_gpio(1);
		spi_init();	/* LCD needs SPI */
		s6e63m0_disp_powerup();
		s6e63m0_disp_on();
		s6e63m0_state.disp_initialized = TRUE;
	}
	else if(power == FB_BLANK_POWERDOWN)
	{
		DPRINT("s6e63m0_power : POWERDOWN\n");    
		if (s6e63m0_state.disp_powered_up && s6e63m0_state.display_on) {
			for (i = 0; i < POWER_OFF_SEQ; i++)
				setting_table_write(&power_off_sequence[i]);	

			lcdc_s6e63m0_pdata->panel_config_gpio(0);
			s6e63m0_state.display_on = FALSE;
			s6e63m0_state.disp_initialized = FALSE;
			s6e63m0_disp_powerdown();
		}
	}

	return ret;
}


static ssize_t s6e63m0_sysfs_store_lcd_power(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	int rc;
	int lcd_enable;
	struct s6e63m0 *lcd = dev_get_drvdata(dev);

	dev_info(dev, "s6e63m0_sysfs_store_lcd_power\n");

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd_enable);
	if (rc < 0)
		return rc;

	if(lcd_enable) {
		s6e63m0_power(lcd, FB_BLANK_UNBLANK);
	}
	else {
		s6e63m0_power(lcd, FB_BLANK_POWERDOWN);
	}

	return len;
}

static DEVICE_ATTR(lcd_power, 0664,
		NULL, s6e63m0_sysfs_store_lcd_power);

#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
static ssize_t lcd_sysfs_store_auto_brightness(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	int value;
	int rc;
	
	dev_info(dev, "lcd_sysfs_store_auto_brightness\n");

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (lcd.auto_brightness != value) {
			dev_info(dev, "%s - %d, %d\n", __func__, lcd.auto_brightness, value);
			mutex_lock(&(lcd.lock));
			lcd.auto_brightness = value;
			mutex_unlock(&(lcd.lock));
		}
	}
	return len;
}

static DEVICE_ATTR(auto_brightness, 0664,
		NULL, lcd_sysfs_store_auto_brightness);

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void s6e63m0_early_suspend(struct early_suspend *h) {

	int i;
	
	DPRINT("panel off at early_suspend (%d,%d,%d)\n",
			s6e63m0_state.disp_initialized,
			s6e63m0_state.disp_powered_up, 
			s6e63m0_state.display_on);
	
	if (s6e63m0_state.disp_powered_up && s6e63m0_state.display_on) {
		for (i = 0; i < POWER_OFF_SEQ; i++)
			setting_table_write(&power_off_sequence[i]);	
		
		lcdc_s6e63m0_pdata->panel_config_gpio(0);
		s6e63m0_state.display_on = FALSE;
		s6e63m0_state.disp_initialized = FALSE;
		s6e63m0_disp_powerdown();
	}
	
	return;
}

static void s6e63m0_late_resume(struct early_suspend *h) {

	DPRINT("panel on at late_resume (%d,%d,%d)\n",
			s6e63m0_state.disp_initialized,
			s6e63m0_state.disp_powered_up,
			s6e63m0_state.display_on);	

	if (!s6e63m0_state.disp_initialized) {
		/* Configure reset GPIO that drives DAC */
		lcdc_s6e63m0_pdata->panel_config_gpio(1);
		spi_init();	/* LCD needs SPI */
		s6e63m0_disp_powerup();
		s6e63m0_disp_on();
		s6e63m0_state.disp_initialized = TRUE;
	}

	return;
}
#endif


static int __devinit s6e63m0_probe(struct platform_device *pdev)
{
	int ret = 0;
	
	//struct msm_fb_data_type *mfd;
	DPRINT("start %s: pdev->name:%s\n", __func__,pdev->name );	

	if (pdev->id == 0) {
		lcdc_s6e63m0_pdata = pdev->dev.platform_data;
	}

	mutex_init(&lcd.lock);
	
	DPRINT("msm_fb_add_device START\n");
	msm_fb_add_device(pdev);
	DPRINT("msm_fb_add_device end\n");

///////////// sysfs
/*
    sysfs_lcd_class = class_create(THIS_MODULE, "lcd");
	if (IS_ERR(sysfs_lcd_class))
		pr_err("Failed to create class(sysfs_lcd_class)!\n");

	sysfs_panel_dev = device_create(sysfs_lcd_class,
		NULL, 0, NULL, "panel");
	if (IS_ERR(sysfs_panel_dev))
		pr_err("Failed to create device(sysfs_panel_dev)!\n");
*/

	if(IS_ERR(sec_class))
            pr_err("Failed to create class(sec)!\n");

	lcd_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
	if(IS_ERR(lcd_dev))
		pr_err("Failed to create device(lcd)!\n");

	lcd.bl = DEFAULT_GAMMA_LEVEL;
	lcd.goal_brightness = lcd.bl;

	lcd.acl_enable = 1;
	lcd.cur_acl = 0;
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	lcd.auto_brightness = 1;
#endif
	ret = device_create_file(lcd_dev, &dev_attr_power_reduce);
	if (ret < 0)
		dev_err(&(pdev->dev), "failed to add sysfs entries\n");


	ret = device_create_file(lcd_dev, &dev_attr_lcd_type);  
	if (ret < 0)
		dev_err(&(pdev->dev), "failed to add sysfs entries\n");

//	ret = device_create_file(lcd_dev, &dev_attr_octa_lcdtype);  
//	if (ret < 0)
//		DPRINT("octa_lcdtype failed to add sysfs entries\n");
//		dev_err(&(pdev->dev), "failed to add sysfs entries\n");

	ret = device_create_file(lcd_dev, &dev_attr_lcd_power);  
	if (ret < 0)
		DPRINT("lcd_power failed to add sysfs entries\n");

	// mdnie sysfs create
	init_mdnie_class();

#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
    
      struct backlight_device *pbd = NULL;
      pbd = backlight_device_register("panel", NULL, NULL,NULL,NULL);
      if (IS_ERR(pbd)) {
        DPRINT("Could not register 'panel' backlight device\n");
      }
      else
      {
        ret = device_create_file(&pbd->dev, &dev_attr_auto_brightness);
        if (ret < 0)
          DPRINT("auto_brightness failed to add sysfs entries\n");
      }

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd.early_suspend.suspend = s6e63m0_early_suspend;
	lcd.early_suspend.resume = s6e63m0_late_resume;
	lcd.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB; //EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&lcd.early_suspend);
#endif

	return 0;
}

static void s6e63m0_shutdown(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);	

	lcdc_s6e63m0_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = s6e63m0_probe,
	.shutdown	= s6e63m0_shutdown,
	.driver = {
		.name   = "lcdc_s6e63m0_wvga",
	},
};

static struct msm_fb_panel_data s6e63m0_panel_data = {
	.on = lcdc_s6e63m0_panel_on,
	.off = lcdc_s6e63m0_panel_off,
	.set_backlight = lcdc_s6e63m0_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_panel",
	.id	= 1,
	.dev	= {
		.platform_data = &s6e63m0_panel_data,
	}
};

#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HPW		2
#define LCDC_HBP		16
#define LCDC_HFP		16
#define LCDC_VPW		2
#define LCDC_VBP		4//6
#define LCDC_VFP		10

#define LCDC_s6e63m0_HPW		2
#define LCDC_s6e63m0_HBP		14
#define LCDC_s6e63m0_HFP		16
#define LCDC_s6e63m0_VPW		2
#define LCDC_s6e63m0_VBP		1
#define LCDC_s6e63m0_VFP		28

 
static int __init lcdc_s6e63m0_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_S6E63M0_wvga"))
	{
		printk(KERN_ERR "%s: msm_fb_detect_client failed!\n", __func__); 
		return 0;
	}
#endif
	DPRINT("start %s\n", __func__);	
	
	ret = platform_driver_register(&this_driver);
	if (ret)
	{
		printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n", __func__, ret); 
		return ret;
	}
      DPRINT("platform_driver_register(&this_driver) is done \n");
		
	pinfo = &s6e63m0_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;

	DPRINT("LDI : s6e63m0, pixelclock 24576000\n");
	pinfo->clk_rate = 24576000;

	pinfo->lcdc.h_back_porch = LCDC_s6e63m0_HBP;
	pinfo->lcdc.h_front_porch = LCDC_s6e63m0_HFP;
	pinfo->lcdc.h_pulse_width = LCDC_s6e63m0_HPW;
	pinfo->lcdc.v_back_porch = LCDC_s6e63m0_VBP;
	pinfo->lcdc.v_front_porch = LCDC_s6e63m0_VFP;
	pinfo->lcdc.v_pulse_width = LCDC_s6e63m0_VPW;
	
	
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0x00; /* black */ //0xff0000; /* red */
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	
	DPRINT("platform_device_register(&this_device) is done \n");
	if (ret)
	{
		printk(KERN_ERR "%s: platform_device_register failed! ret=%d\n", __func__, ret); 
		platform_driver_unregister(&this_driver);
	}
	return ret;
}

module_init(lcdc_s6e63m0_panel_init);


