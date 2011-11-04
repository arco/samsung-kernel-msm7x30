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

#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk("s6d16a0x22 " x)
#else
#define DPRINT(x...)	
#endif

#define __LCD_CONTROL_BY_FILE__

#define GPIO_BL_CTRL	16
#define GPIO_LCD_DET	26

enum {
    GPIO_INPUT,
    GPIO_OUTPUT,
};

/* GPIO TLMM: Pullup/Pulldown */
enum {
    GPIO_NO_PULL,
    GPIO_PULL_DOWN,
    GPIO_KEEPER,
    GPIO_PULL_UP,
};

/* GPIO TLMM: Drive Strength */
enum {
    GPIO_2MA,
    GPIO_4MA,
    GPIO_6MA,
    GPIO_8MA,
    GPIO_10MA,
    GPIO_12MA,
    GPIO_14MA,
    GPIO_16MA,
};

enum {
    GPIO_ENABLE,
    GPIO_DISABLE,
};

/*
 * Serial Interface
 */
#define LCD_CSX_HIGH	gpio_set_value(spi_cs, 1);
#define LCD_CSX_LOW		gpio_set_value(spi_cs, 0);

#define LCD_SCL_HIGH	gpio_set_value(spi_sclk, 1);
#define LCD_SCL_LOW		gpio_set_value(spi_sclk, 0);

#define LCD_SDI_HIGH	gpio_set_value(spi_sdi, 1);
#define LCD_SDI_LOW		gpio_set_value(spi_sdi, 0);

#define DEFAULT_USLEEP	1	

// #define LCD_DET_ENABLE   // minhyodebug

#ifdef LCD_DET_ENABLE
#include <linux/irq.h>

struct s6d16a0x22_lcd_det_data {
	int             irq;
	struct work_struct work_lcd_det;  /* for lcd_det */
};

struct s6d16a0x22_lcd_det_data lcd_det_data;
struct workqueue_struct *lcd_det_wq;
struct platform_device *plcdc_s6d16a0x22_panel_device;
#endif

#if 0 // CALLISTO-REV00A, minhyo
#define WRPWD			0xF0
#define POWCTL			0xF3
#define EDSTEST			0xFF
#define VCMCTL			0xF4
#define SRCCTL			0xF5
#define SLPOUT			0x11
#define MADCTL			0x36
#define COLMOD			0x3A
#define DISCTL			0xF2
#define IFCTL			0xF6
#define GATECTL			0xFD
#define WRDISBV			0x51
#define WRCABCMB		0x5E
#define MIECTL1			0xCA
#define BCMODE			0xCB
#define MIECTL2			0xCC
#define MIDCTL3			0xCD
#define RPGAMCTL		0xF7
#define RNGAMCTL		0xF8
#define GPGAMCTL		0xF9
#define GNGAMCTL		0xFA
#define BPGAMCTL		0xFB
#define BNGAMCTL		0xFC
#define CASET			0x2A
#define PASET			0x2B
#define RAMWR           0x2C
#define WRCTRLD			0x53
#define WRCABC			0x55
#define DISPON			0x29
#define DISPOFF			0x28
#define SLPIN			0x10
#endif

struct setting_table {
	unsigned char command;	
	unsigned char parameters;
	unsigned char parameter[48]; // 31 > 90 > 45
	long wait;
};


static struct setting_table power_on_setting_table[] = {
	{ 0xF1,	2, 
		{ 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	// Rotation
	{ 0x36,	1, 
		{ 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	
	
	{ 0xB7,	3, 
		{ 0x00, 0x11, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xB8,	2, 
		{ 0x0C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xB9,	2, 
		{ 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xBB,	1, 
		{ 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xC0,	2, 
		{ 0x80, 0x80, 0x010, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xC1,	1, 
		{ 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xEE,	1, 
		{ 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF2,	9, 
		{ 0x00, 0x30, 0x94, 0x94, 0x57, 0x57, 0x10, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF3,	12, 
		{ 0x00, 0x10, 0x25, 0x10, 0x2D, 0x2D, 0x24, 0x2D, 0x10, 0x10, 0x0A, 0x37, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF4,	10, 
		{ 0x00, 0x20, 0x00, 0xAF, 0x64, 0x00, 0xAF, 0x64, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF5,	10, 
		{ 0x00, 0x00, 0x57, 0x11, 0x05, 0x01, 0x01, 0x01, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF6,	5, 
		{ 0xA1, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF7,	17, 
		{ 0x04, 0x6E, 0x00, 0x12, 0x03, 0x0D, 0x0A, 0x16, 0x05, 0x04, 0x0E, 0x04, 0x04, 0x00, 0x96, 0x01,
	  	    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF8,	17, 
		{ 0x04, 0x6E, 0x00, 0x00, 0x03, 0x0D, 0x0A, 0x16, 0x05, 0x04, 0x0E, 0x04, 0x04, 0x00, 0x96, 0x01,
	  	    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xF9,	22, 
		{ 0x01, 0x02, 0x05, 0x04, 0x0A, 0x0B, 0x00, 0x06, 0x11, 0x10, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0xCF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0xFA,	45, 
		{ 0x00, 0x2C, 0x2B, 0x25, 0x39, 0x37, 0x34, 0x26, 0x20, 0x34, 0x1F, 0x1D, 0x1D, 0x0E, 0x04, 0x1C, 
	  	    0x2E, 0x2C, 0x24, 0x36, 0x36, 0x37, 0x28, 0x23, 0x38, 0x25, 0x25, 0x29, 0x21, 0x1D, 0x1D, 0x2E,
   	  	    0x2D, 0x25, 0x39, 0x37, 0x37, 0x29, 0x24, 0x38, 0x25, 0x24, 0x28, 0x21, 0x1E, 0x00, 0x00, 0x00, },
	0 },			

	{ 0xFB,	45, 
		{ 0x00, 0x3F, 0x3A, 0x2F, 0x40, 0x3E, 0x3A, 0x23, 0x16, 0x2D, 0x1D, 0x19, 0x16, 0x08, 0x01, 0x00, 
	  	    0x23, 0x21, 0x1D, 0x35, 0x38, 0x36, 0x21, 0x16, 0x2D, 0x1D, 0x1D, 0x1B, 0x0C, 0x0c, 0x00, 0x22,
	  	    0x21, 0x1D, 0x35, 0x39, 0x36, 0x21, 0x14, 0x2C, 0x1C, 0x1B, 0x19, 0x0A, 0x02, 0x00, 0x00, 0x00, },
	0 },		

	{ 0xF1,	2, 
		{ 0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0x2A,	4, 
		{ 0x00, 0x00, 0x01, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	{ 0x2B,	4, 
		{ 0x00, 0x00, 0x03, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	0 },	

	// Sleep Out Command
	{ 0x11,	0, 
	  	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	5 },	

	// COLMOD command
	{ 0x3A,	1, 
		{ 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	130 },	

	// Display On Command
	{ 0x29,	0, 
	  	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,},
	10 },
};

static struct setting_table power_off_setting_table[] = {
//	Sleep In
	{ 0x10, 0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 },
};

/* 20100824 */
static struct setting_table backlight_setting_table[] = {
	{    0x51, 1, { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
};
/***/

#define POWER_ON_SETTINGS	(int)(sizeof(power_on_setting_table)/sizeof(struct setting_table))
#define POWER_OFF_SETTINGS	(int)(sizeof(power_off_setting_table)/sizeof(struct setting_table))
#define DEFAULT_LCD_ON_BACKLIGHT_LEVEL 23

static int lcdc_s6d16a0x22_panel_off(struct platform_device *pdev);

static int lcd_brightness = -1;

static int spi_cs;
static int spi_sclk;
//static int spi_sdo;
static int spi_sdi;
//static int spi_dac;
//static int lcd_en;
static int lcd_reset;

#if 0
static unsigned char bit_shift[8] = { (1 << 7),	/* MSB */
	(1 << 6),
	(1 << 5),
	(1 << 4),
	(1 << 3),
	(1 << 2),
	(1 << 1),
	(1 << 0)		               /* LSB */
};
#endif

static int delayed_backlight_value = -1;

static boolean First_Disp_Power_On = FALSE;

static void lcdc_s6d16a0x22_set_brightness_in_blu(int level);

struct s6d16a0x22_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static struct s6d16a0x22_state_type s6d16a0x22_state = { 
	.disp_initialized = FALSE,
	.display_on = FALSE,
	.disp_powered_up = FALSE,
};

static struct msm_panel_common_pdata *lcdc_s6d16a0x22_pdata;

static void setting_table_write(struct setting_table *table)
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

static void spi_init(void)
{
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_s6d16a0x22_pdata->gpio_num);
	spi_cs   = *(lcdc_s6d16a0x22_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_s6d16a0x22_pdata->gpio_num + 2);
//	lcd_en   = *(lcdc_s6d16a0x22_pdata->gpio_num + 3);
	lcd_reset= *(lcdc_s6d16a0x22_pdata->gpio_num + 3);
//	spi_sdo  = *(lcdc_s6d16a0x22_pdata->gpio_num + 3);

	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);

}

static void s6d16a0x22_disp_powerup(void)
{
	DPRINT("start %s\n", __func__);	

	if (!s6d16a0x22_state.disp_powered_up && !s6d16a0x22_state.display_on) {
		/* Reset the hardware first */
		gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		//LCD_RESET_N_HI
		gpio_set_value(lcd_reset, 1);

		mdelay(20);
		
		//LCD_RESET_N_LO
		gpio_set_value(lcd_reset, 0);
		mdelay(20);
		//LCD_RESET_N_HI
		gpio_set_value(lcd_reset, 1);
		mdelay(10);

		s6d16a0x22_state.disp_powered_up = TRUE;
	}
}

static void s6d16a0x22_disp_powerdown(void)
{
	DPRINT("start %s\n", __func__);	

	gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		/* Reset Assert */
	gpio_set_value(lcd_reset, 0);		
	/* turn off LDO */
	//TODO: turn off LDO
	//gpio_set_value(lcd_en, 0);

	s6d16a0x22_state.disp_powered_up = FALSE;
}

static void s6d16a0x22_disp_on(void)
{
	int i;
	DPRINT("start %s\n", __func__);	

	if (s6d16a0x22_state.disp_powered_up && !s6d16a0x22_state.display_on) {
		mdelay(20);
		for (i = 0; i < POWER_ON_SETTINGS; i++)
			setting_table_write(&power_on_setting_table[i]);
		mdelay(1);
		s6d16a0x22_state.display_on = TRUE;
	}
}

static int lcdc_s6d16a0x22_panel_on(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);	

	if (!s6d16a0x22_state.disp_initialized) {
		/* Configure reset GPIO that drives DAC */
		lcdc_s6d16a0x22_pdata->panel_config_gpio(1);
		spi_init();	/* LCD needs SPI */
		s6d16a0x22_disp_powerup();
		s6d16a0x22_disp_on();

		mdelay(50);
		s6d16a0x22_state.disp_initialized = TRUE;
	}

#ifdef LCD_DET_ENABLE
	/* enable INT */
	enable_irq(lcd_det_data.irq);	
#endif

    if(!First_Disp_Power_On)
    {
    	First_Disp_Power_On = TRUE;
       lcdc_s6d16a0x22_set_brightness_in_blu(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);	
     }
	if(delayed_backlight_value != -1) {
		lcdc_s6d16a0x22_set_brightness_in_blu(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);
	//	DPRINT("delayed backlight on %d", delayed_backlight_value);
	}

	return 0;
}

static int lcdc_s6d16a0x22_panel_off(struct platform_device *pdev)
{
	int i;

	DPRINT("start %s\n", __func__);	

#ifdef LCD_DET_ENABLE
	disable_irq(lcd_det_data.irq);
#endif

	if (s6d16a0x22_state.disp_powered_up && s6d16a0x22_state.display_on) {
		
		for (i = 0; i < POWER_OFF_SETTINGS; i++)
			setting_table_write(&power_off_setting_table[i]);	
		
		lcdc_s6d16a0x22_pdata->panel_config_gpio(0);
		s6d16a0x22_state.display_on = FALSE;
		s6d16a0x22_state.disp_initialized = FALSE;
		mdelay(120);
		s6d16a0x22_disp_powerdown();
	}
	return 0;
}

#if 1

// brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define DFT_BRIGHTNESS_LEVEL 		141//151//129//170//158//120  // When we are booting, Application set brightness-leverl 140, Brightness-level 140 is mapped Backlight-level 34. 
#define LOW_BRIGHTNESS_LEVEL 30
#define MAX_BACKLIGHT_VALUE2	25//27//30
#define MAX_BACKLIGHT_VALUE	25//27//30
#define LOW_BACKLIGHT_VALUE	6//5//6//5
#define DIM_BACKLIGHT_VALUE		3//4//2
#define DFT_BACKLIGHT_VALUE		( (DFT_BRIGHTNESS_LEVEL - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE) / (MAX_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE )
#define MAX_BRIGHTNESS_IN_BLU	32 // backlight-IC MAX VALUE

static DEFINE_SPINLOCK(bl_ctrl_lock);
static DEFINE_SPINLOCK(bl_ctrl_irq_lock);


static void lcdc_s6d16a0x22_set_brightness_in_blu(int level)
{
	unsigned long irqflags;
	int tune_level = level;
	int pulse;

	// LCD should be turned on prior to backlight
	if(s6d16a0x22_state.disp_initialized == FALSE && tune_level > 0)
	{
		//DPRINT("skip bl:%d\n", tune_level);	
		delayed_backlight_value = tune_level;
		return;
	}
	else
	{
		//DPRINT("set bl:%d\n", tune_level);	
		delayed_backlight_value = -1;
	}

	//TODO: lock
	spin_lock(&bl_ctrl_lock);

	if (tune_level <= 0) 
	{
		/* keep back light OFF */
		gpio_set_value(GPIO_BL_CTRL, 0);
		mdelay(3);		//guaranteed shutdown
		lcd_brightness = tune_level;
	} 
	else 
	{
		/* keep back light ON */
		if(unlikely(lcd_brightness < 0))
		{
			// first time
			int val = gpio_get_value(GPIO_BL_CTRL);
			if(val)
			{
				lcd_brightness = 0;
				gpio_set_value(GPIO_BL_CTRL, 0);
				mdelay(3);		//guaranteed shutdown
			}
		}

		if(!lcd_brightness)
		{
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3); 
			lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
		}
			
		pulse = (lcd_brightness - tune_level + MAX_BRIGHTNESS_IN_BLU) % MAX_BRIGHTNESS_IN_BLU;

		spin_lock_irqsave(&bl_ctrl_irq_lock, irqflags);

		for(; pulse>0; pulse--)
		{
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(3);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3);
		}
				
		spin_unlock_irqrestore(&bl_ctrl_irq_lock, irqflags);
		lcd_brightness = tune_level;
	
	}

	
	//TODO: unlock
	spin_unlock(&bl_ctrl_lock);

}

static void lcdc_s6d16a0x22_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;
	int tune_level;

		// brightness tuning
	if(bl_level >= LOW_BRIGHTNESS_LEVEL)
		tune_level = (bl_level - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE) / (MAX_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE;
	else if(bl_level > 0)
		tune_level = DIM_BACKLIGHT_VALUE;
	else
		tune_level = bl_level;

	DPRINT("%s:%d,%d\n", __func__, bl_level, tune_level);	

		// turn on lcd if needed
#ifdef __LCD_ON_EARLY__
	if(tune_level > 0)
	{
		if(!s6d16a0x22_state.disp_powered_up)
			s6d16a0x22_disp_powerup();
		if(!s6d16a0x22_state.display_on)
			s6d16a0x22_disp_on();
	}
#endif


		lcdc_s6d16a0x22_set_brightness_in_blu(tune_level);
}
#endif


#ifdef __LCD_CONTROL_BY_FILE__
static int s3cfb_sysfs_show_lcd_power(struct device *dev, struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", s6d16a0x22_state.disp_initialized);
}

static int s3cfb_sysfs_store_lcd_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct platform_device* pdev;
	
    if (len < 1)
        return -EINVAL;

	pdev = container_of(dev, struct platform_device, dev);

    if (strnicmp(buf, "on", 2) == 0 || strnicmp(buf, "1", 1) == 0)
    	{
		lcdc_s6d16a0x22_panel_on(pdev);
    	lcdc_s6d16a0x22_set_brightness_in_blu(15);
    	}
    else if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0)
    	{
    	lcdc_s6d16a0x22_set_brightness_in_blu(0);
		lcdc_s6d16a0x22_panel_off(pdev);
    	}
    else
            return -EINVAL;

    return len;
}

static DEVICE_ATTR(lcd_power, 0666,						//sm.kim: give write permission for application
                        s3cfb_sysfs_show_lcd_power,
                        s3cfb_sysfs_store_lcd_power);
#endif

#ifdef LCD_DET_ENABLE // minhyo0614

static irqreturn_t lcd_det_irq_handler(int irq, void *dev_id)
{
	struct s6d16a0x22_lcd_det_data *lcd_det = dev_id;

	printk("[LCD] lcd_det->irq = %d\n",lcd_det->irq);

	if(lcd_det->irq !=-1)
	{
		disable_irq(lcd_det->irq);
		printk("[LCD] disable_irq \n");

		queue_work(lcd_det_wq, &lcd_det->work_lcd_det);

	}
	
	printk("[LCD] IRQ_HANDLED \n");

	return IRQ_HANDLED;
}

static void lcd_det_work_func(struct work_struct *work)
{
	/* LCD init */
	lcdc_s6d16a0x22_panel_off(plcdc_s6d16a0x22_panel_device);
	mdelay(2);
	lcdc_s6d16a0x22_panel_on(plcdc_s6d16a0x22_panel_device);

	/* enable INT */
//	enable_irq(lcd_det->irq);
}

#endif

static int __init s6d16a0x22_probe(struct platform_device *pdev)
{
	int ret;
	
	printk("++++++%s\n", __func__);
	printk("++++++pdev->id=%d\n", pdev->id);


#ifdef __LCD_CONTROL_BY_FILE__
    ret = device_create_file(&(pdev->dev), &dev_attr_lcd_power);
    if (ret < 0)
            printk(KERN_WARNING "s6d16a0x22: failed to add entries\n");
#endif

	if (pdev->id == 0)
	{
		lcdc_s6d16a0x22_pdata = pdev->dev.platform_data;    // lcdc_s6d16a0x22_panel_data in board-msm7x27.c

		#ifdef LCD_DET_ENABLE
		{ 
			int config, irq, err;

			plcdc_s6d16a0x22_panel_device = pdev;

			/* work queue setting */
			lcd_det_wq= create_singlethread_workqueue("lcd_det_wq");
			if (!lcd_det_wq)
				return -ENOMEM;
			INIT_WORK(&lcd_det_data.work_lcd_det, lcd_det_work_func);
			printk("Workqueue Settings complete\n");

			/* gpio config */
			config = GPIO_CFG(GPIO_LCD_DET, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
			err = gpio_tlmm_config(config, GPIO_ENABLE);
			if (err) 
				printk(KERN_ERR "%s: gpio_tlmm_config(%d)=%d\n", __func__, (int)GPIO_LCD_DET, err);
			
			/* INT setting */
			irq = gpio_to_irq(GPIO_LCD_DET);
			lcd_det_data.irq = -1;
			set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
			err = request_irq(irq, lcd_det_irq_handler, IRQF_DISABLED, "lcd_det_int", &lcd_det_data);
			if (err)
			{
				printk("%s: request_irq failed for lcd_det\n", __func__);
				return 0;
			}
			lcd_det_data.irq = irq;
			printk("INT Settings complete\n");
		}
		#endif
		
		return 0;
	}

	printk("------%s\n", __func__);

	
	msm_fb_add_device(pdev);

	return 0;
}

static void s6d16a0x22_shutdown(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);	

   	lcdc_s6d16a0x22_set_brightness_in_blu(0);
	lcdc_s6d16a0x22_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = s6d16a0x22_probe,
	.shutdown	= s6d16a0x22_shutdown,
	.driver = {
		.name   = "lcdc_s6d16a0x22_wvga",
	},
};

static struct msm_fb_panel_data s6d16a0x22_panel_data = {
	.on = lcdc_s6d16a0x22_panel_on,
	.off = lcdc_s6d16a0x22_panel_off,
	.set_backlight = lcdc_s6d16a0x22_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_panel",
	.id	= 1,
	.dev	= {
		.platform_data = &s6d16a0x22_panel_data,
	}
};

#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HPW	2
#define LCDC_HBP		20
#define LCDC_HFP		20
#define LCDC_VPW		4
#define LCDC_VBP		7
#define LCDC_VFP		11

#define LCDC_PCLK		(LCDC_FB_XRES + LCDC_HBP + LCDC_HPW + LCDC_HFP) * (LCDC_FB_YRES + LCDC_VBP + LCDC_VPW + LCDC_VFP) * 2

static int __init lcdc_s6d16a0x22_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_s6d16a0x22_wvga"))
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

	pinfo = &s6d16a0x22_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 24576 * 1000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = LCDC_HBP;
	pinfo->lcdc.h_front_porch = LCDC_HFP;
	pinfo->lcdc.h_pulse_width = LCDC_HPW;
	pinfo->lcdc.v_back_porch = LCDC_VBP;
	pinfo->lcdc.v_front_porch = LCDC_VFP;
	pinfo->lcdc.v_pulse_width = LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;

	printk("%s\n", __func__);

	ret = platform_device_register(&this_device);
	if (ret)
	{
		printk(KERN_ERR "%s: platform_device_register failed! ret=%d\n", __func__, ret); 
		platform_driver_unregister(&this_driver);
	}

	return ret;
}

module_init(lcdc_s6d16a0x22_panel_init);


