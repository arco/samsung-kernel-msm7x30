/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/delay.h>
#include <linux/pwm.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "lcdc_samsung_ancora.h"
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <linux/mfd/pmic8058.h>
#include <linux/clk.h>
#include <linux/module.h>

static int spi_cs;
static int spi_sclk;
static int spi_mosi;
static int lcd_reset;
static int delayed_backlight_value = -1;
static boolean First_Disp_Power_On = FALSE;
static int lcd_brightness = -1;

extern unsigned long acpuclk_usr_set_max(void);


static struct samsung_state_type samsung_state = { .brightness = 180 };
static struct msm_panel_common_pdata *lcdc_samsung_pdata;
extern unsigned int board_lcd_hw_revision;
#define LCD_RESET_N_HI	gpio_set_value(lcd_reset, 1);			
#define LCD_RESET_N_LO	gpio_set_value(lcd_reset, 0);
#define LCD_CS_N_HIGH	gpio_set_value(spi_cs, 1);
#define LCD_CS_N_LOW	gpio_set_value(spi_cs, 0);
#define LCD_SCLK_HIGH	gpio_set_value(spi_sclk, 1);
#define LCD_SCLK_LOW	gpio_set_value(spi_sclk, 0);
#define LCD_SDI_HIGH	gpio_set_value(spi_mosi, 1);
#define LCD_SDI_LOW		gpio_set_value(spi_mosi, 0);
#if defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_ANCORA)
#define LCD_DET_ENABLE
#endif
#ifdef LCD_DET_ENABLE
#define LCD_DET_ENABLE_IRQ PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 25)
boolean irq_disabled = FALSE;
boolean panel_initialized = FALSE;
static int ESD_count = 0;
static irqreturn_t s6d16a0x_esd_irq_handler(int irq, void *handle);
static struct timer_list lcd_esd_timer;
static void lcd_esd_timer_handler(unsigned long data);
static struct timer_list lcd_esd_timer2;
static void lcd_esd_timer_handler2(unsigned long data);
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)

#endif

#ifdef CONFIG_MACH_ANCORA
#define SYSFS_LCD_CONNECT_CHECK
static int smd_lcd_connected = 0;
extern bool MicroJigUARTOffStatus;
#endif

typedef	unsigned char		UINT8;
static void HYDIS_LG4573B_write(UINT8 cmd, UINT8 param);
static void HYDIS_LG4573B_Index(UINT8 cmd);
static void HYDIS_LG4573B_Data(UINT8 param);

 void HYDIS_LG4573B_Index(UINT8 cmd)
{
	int j;
	if(cmd != 0x0000){
	LCD_SDI_HIGH
	LCD_CS_N_HIGH
	LCD_SCLK_HIGH
	LCD_CS_N_LOW
	
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH		
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH	
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH			
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH	
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH	
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH			
	// Command
	for (j = 7 ; j >= 0; j--){
		LCD_SCLK_LOW   
		if ((cmd >> j) & 0x01)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		LCD_SCLK_HIGH       
	}
	LCD_CS_N_HIGH
	}		
}
void HYDIS_LG4573B_Data(UINT8 param)
{
	int j;
	LCD_SDI_HIGH
	LCD_CS_N_HIGH
	LCD_SCLK_HIGH
	LCD_CS_N_LOW
	
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH		
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH	
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH			
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	// ID
	LCD_SCLK_HIGH	
	LCD_SCLK_LOW   
	LCD_SDI_HIGH	
	LCD_SCLK_HIGH	
	LCD_SCLK_LOW   
	LCD_SDI_LOW	
	LCD_SCLK_HIGH
	// parameter
	for (j = 7 ; j >= 0; j--){
		LCD_SCLK_LOW   
		if ((param >> j) & 0x01)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		LCD_SCLK_HIGH
	}
	LCD_CS_N_HIGH					
}
void HYDIS_LG4573B_write(UINT8 cmd, UINT8 param)
{
	HYDIS_LG4573B_Index(cmd);
	HYDIS_LG4573B_Data(param);
}
void Setting_Table_HYDIS(void)
{
	HYDIS_LG4573B_write(0xC1, 0x00);
	HYDIS_LG4573B_Index(0x11);  // Exit Sleep Mode
	HYDIS_LG4573B_write(0x3A, 0x70); // 18 Bit : 0060h
	HYDIS_LG4573B_write(0xB1, 0x00);
	HYDIS_LG4573B_write(0x00, 0x14);   // 11, 10, 11, 13, 18, 1A, 14 Inc : L, Dec : R
	HYDIS_LG4573B_write(0x00, 0x06); // 3, 5, 7, 1C, 10, 0C  Inc : U, Dec : D
	HYDIS_LG4573B_write(0xB2, 0x10); 
	HYDIS_LG4573B_write(0x00, 0xC8);
	HYDIS_LG4573B_write(0xB3, 0x00); // 0x01 : 1-Dot Inversion, 0x00 : Column Inversion
	HYDIS_LG4573B_write(0xB4, 0x04);
	HYDIS_LG4573B_write(0xB5, 0x05); // 5, 10, FF
	HYDIS_LG4573B_write(0x00, 0x10);
	HYDIS_LG4573B_write(0x00, 0x10);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0xB6, 0x01); // Overlapping
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x02);
	HYDIS_LG4573B_write(0x00, 0x40);
	HYDIS_LG4573B_write(0x00, 0x02);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0xC0, 0x00);
	HYDIS_LG4573B_write(0x00, 0x1F);

	// Power On Sequence 
	HYDIS_LG4573B_write(0xC2, 0x00);
			
	HYDIS_LG4573B_write(0xC3, 0x03);
	HYDIS_LG4573B_write(0x00, 0x04);
	HYDIS_LG4573B_write(0x00, 0x05);
	HYDIS_LG4573B_write(0x00, 0x06);
	HYDIS_LG4573B_write(0x00, 0x01)
		;
	HYDIS_LG4573B_write(0xC4, 0x02);
	HYDIS_LG4573B_write(0x00, 0x23);
	HYDIS_LG4573B_write(0x00, 0x16);
	HYDIS_LG4573B_write(0x00, 0x16);
	HYDIS_LG4573B_write(0x00, 0x02);
	HYDIS_LG4573B_write(0x00, 0x7A);
	
	HYDIS_LG4573B_write(0xC5, 0x77);
	
	HYDIS_LG4573B_write(0xC6, 0x24);
	HYDIS_LG4573B_write(0x00, 0x60);
	HYDIS_LG4573B_write(0x00, 0x00);

	// Gamma Control
	HYDIS_LG4573B_write(0xD0, 0x00); // Gama Set_1
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x26);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x31);
	HYDIS_LG4573B_write(0x00, 0x03);

	HYDIS_LG4573B_write(0xD1, 0x00); // Gama Set_2
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x26);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x31);
	HYDIS_LG4573B_write(0x00, 0x03);

	HYDIS_LG4573B_write(0xD2, 0x00); // Gama Set_1
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x26);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x31);
	HYDIS_LG4573B_write(0x00, 0x03);

	HYDIS_LG4573B_write(0xD3, 0x00); // Gama Set_2
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x26);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x31);
	HYDIS_LG4573B_write(0x00, 0x03);

	HYDIS_LG4573B_write(0xD4, 0x00); // Gama Set_1
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x26);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x31);
	HYDIS_LG4573B_write(0x00, 0x03);

	HYDIS_LG4573B_write(0xD5, 0x00); // Gama Set_2
	HYDIS_LG4573B_write(0x00, 0x01);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x26);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x00);
	HYDIS_LG4573B_write(0x00, 0x66);
	HYDIS_LG4573B_write(0x00, 0x31);
	HYDIS_LG4573B_write(0x00, 0x03);

	mdelay(100);
	HYDIS_LG4573B_Index(0x29);  // Display On		
}

struct brt_value{
	int level;				// Platform setting values
	int tune_level;			// Chip Setting values
};

struct brt_value brt_table_aat[] = {
		{ 255, 	6	}, // Max
		{ 244, 	8	}, 
		{ 232, 	9	}, 
		{ 221, 	10	}, 
		{ 210, 	11	}, 
		{ 199, 	12	}, 	
		{ 187, 	13	}, 
		{ 176, 	14	}, 
		{ 165, 	15	}, 
		{ 153, 	16	}, 
		{ 142, 	17	}, // Bennet Default	
		{ 127, 	18	}, 
		{ 120, 	19	}, 
		{ 112, 	20	}, 
		{ 105, 	21	},	
		{ 97, 	22	}, 
		{ 90, 	23	}, 
		{ 82, 	24	},  
		{ 75,   25  }, 
		{ 67, 	26	}, 
		{ 60, 	27	}, 
		{ 52,   28  }, 
		{ 45,   29  }, 
		{ 37,   30  }, 
		{ 30, 	31	}, // min
		{ 20, 	31	}, // dimming
		{ 0, 	32	  }, // Off
};

struct brt_value brt_table_aat_hysys[] = {
		{ 255, 	11	}, // Max
		{ 245, 	11	}, 
		{ 235, 	12	}, 
		{ 226, 	13	}, 
		{ 216, 	13	}, 
		{ 206, 	14	}, 	
		{ 196, 	15	}, 
		{ 187, 	16	}, 
		{ 177, 	17	}, 
		{ 167, 	18	}, 
		{ 157, 	18	}, 
		{ 147, 	19	}, 
		{ 138, 	19	}, // Ancro Hysys Default 
		{ 128, 	20	}, 
		{ 118, 	21	},	
		{ 108, 	22	}, 
		{ 98, 	23	}, 
		{ 89, 	24	},  
		{ 79,   26  }, 
		{ 69, 	27	}, 
		{ 59, 	28	}, 
		{ 50,   29  }, 
		{ 40,   30  }, 
		{ 30, 	31	}, // min
		{ 20, 	31	}, // dimming
		{ 0, 	32	}, // Off
};

struct brt_value brt_table_aat_smd[] = {
		{ 255, 	10	}, // Max
		{ 242, 	11	}, 
		{ 230, 	12	}, 
		{ 217, 	13	}, 
		{ 205, 	14	}, 
		{ 192, 	15	}, 
		{ 180, 	16	},
		{ 167, 	17	}, 
		{ 155, 	18	}, 
		{ 142, 	19	},  // Ancro smd Default   
		{ 132, 	20	}, 
		{ 123, 	21	}, 
		{ 114, 	22	},
		{ 104, 	23	}, 
		{ 95, 	24	},
		{ 86, 	25	},  
		{ 76, 	26	}, 
		{ 67, 	27	}, 
		{ 58, 	28	}, 
		{ 48,	29	}, 
		{ 39, 	30	}, 
		{ 30,	31	}, 
		{ 20, 	31	},
		{ 0, 	32	}, // Off
};

#define MAX_BRT_STAGE_AAT (int)(sizeof(brt_table_aat)/sizeof(struct brt_value))
#define MAX_BRT_STAGE_AAT_HYSYS (int)(sizeof(brt_table_aat_hysys)/sizeof(struct brt_value))
#define MAX_BRT_STAGE_AAT_SMD (int)(sizeof(brt_table_aat_smd)/sizeof(struct brt_value))

#define MIN_BRIGHTNESS_VALUE	        30
#define AAT_DIMMING_VALUE		31   //TA

static void samsung_spi_write_byte(boolean dc, u8 data)
{
	uint32 bit;
	int bnum;

	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_mosi, dc ? 1 : 0);
	udelay(1);			/* at least 20 ns */
	gpio_set_value(spi_sclk, 1);	/* clk high */
	udelay(1);			/* at least 20 ns */

	bnum = 8;			/* 8 data bits */
	bit = 0x80;
	while (bnum--) {
		gpio_set_value(spi_sclk, 0); /* clk low */
		gpio_set_value(spi_mosi, (data & bit) ? 1 : 0);
		udelay(1);
		gpio_set_value(spi_sclk, 1); /* clk high */
		udelay(1);
		bit >>= 1;
	}
	gpio_set_value(spi_mosi, 0);

}

static void samsung_spi_read_bytes(u8 cmd, u8 *data, int num)
{
	int bnum;

	/* Chip Select - low */
	gpio_set_value(spi_cs, 0);
	udelay(2);

	/* command byte first */
	samsung_spi_write_byte(0, cmd);
	udelay(2);

	gpio_direction_input(spi_mosi);

	if (num > 1) {
		/* extra dummy clock */
		gpio_set_value(spi_sclk, 0);
		udelay(1);
		gpio_set_value(spi_sclk, 1);
		udelay(1);
	}

	/* followed by data bytes */
	bnum = num * 8;	/* number of bits */
	*data = 0;
	while (bnum) {
		gpio_set_value(spi_sclk, 0); /* clk low */
		udelay(1);
		*data <<= 1;
		*data |= gpio_get_value(spi_mosi) ? 1 : 0;
		gpio_set_value(spi_sclk, 1); /* clk high */
		udelay(1);
		--bnum;
		if ((bnum % 8) == 0)
			++data;
	}

	gpio_direction_output(spi_mosi, 0);

	/* Chip Select - high */
	udelay(2);
	gpio_set_value(spi_cs, 1);
}

static int samsung_serigo(struct samsung_spi_data data)
{
	int i;

	/* Chip Select - low */
	gpio_set_value(spi_cs, 0);
	udelay(2);

	samsung_spi_write_byte(FALSE, data.addr);
	udelay(2);

	for (i = 0; i < data.len; ++i) {
		samsung_spi_write_byte(TRUE, data.data[i]);
		udelay(2);
	}

	/* Chip Select - high */
	gpio_set_value(spi_cs, 1);
#ifdef DEBUG
	pr_info("%s: cmd=0x%02x, #args=%d\n", __func__, data.addr, data.len);
#endif
	return 0;
}

static int samsung_write_cmd(u8 cmd)
{
	/* Chip Select - low */
	gpio_set_value(spi_cs, 0);
	udelay(2);

	samsung_spi_write_byte(FALSE, cmd);

	/* Chip Select - high */
	udelay(2);
	gpio_set_value(spi_cs, 1);
#ifdef DEBUG
	pr_info("%s: cmd=0x%02x\n", __func__, cmd);
#endif
	return 0;
}

static int samsung_serigo_list(struct samsung_spi_data *data, int count)
{
	int i, rc;
	for (i = 0; i < count; ++i, ++data) {
		rc = samsung_serigo(*data);
		if(data->delay)mdelay(data->delay);
		if (rc)
			return rc;
	}
	return 0;
}

static void samsung_spi_init(void)
{
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_samsung_pdata->gpio_num);
	spi_cs   = *(lcdc_samsung_pdata->gpio_num + 1);
	spi_mosi  = *(lcdc_samsung_pdata->gpio_num + 2);
	lcd_reset= *(lcdc_samsung_pdata->gpio_num + 3);

	/* Set the output so that we don't disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_mosi, 0);

	/* Set the Chip Select deasserted (active low) */
	gpio_set_value(spi_cs, 1);
	
}

static DEFINE_SPINLOCK(bl_ctrl_lock);
static DEFINE_SPINLOCK(bl_ctrl_irq_lock);

static void lcdc_samsung_set_brightness_in_blu(int level)
{
	unsigned long irqflags;
	int tune_level = level;
	int pulse;

	/* LCD should be turned on prior to control backlight */
	if(samsung_state.disp_initialized == FALSE && tune_level > 0) {
		delayed_backlight_value = tune_level;
		return;
	} else {
		delayed_backlight_value = -1;
	}

	spin_lock(&bl_ctrl_lock);
	if (tune_level <= 0) {
		gpio_set_value(GPIO_BL_CTRL, 0);
		mdelay(3);
		lcd_brightness = tune_level;
	} else {
		if(unlikely(lcd_brightness < 0)) {
			int val = gpio_get_value(GPIO_BL_CTRL);
			if(val) {
				lcd_brightness = 0;
				gpio_set_value(GPIO_BL_CTRL, 0);
				mdelay(3);
			}
		}
		
		if(!lcd_brightness) {
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3);
			lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
		}

		pulse = (lcd_brightness - tune_level + MAX_BRIGHTNESS_IN_BLU) % MAX_BRIGHTNESS_IN_BLU;

		spin_lock_irqsave(&bl_ctrl_irq_lock, irqflags);
		for(; pulse>0; pulse--) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(3);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3);
		}
		spin_unlock_irqrestore(&bl_ctrl_irq_lock, irqflags);
		lcd_brightness = tune_level;
	}
	spin_unlock(&bl_ctrl_lock);
}

static void lcdc_samsung_set_backlight(struct msm_fb_data_type *mfd)
{
	int level = mfd->bl_level;
	int tune_level = 0;
	int i;

#ifdef __LCD_ON_EARLY__
	if(tune_level > 0)
	{
		if(!samsung_state.disp_powered_up)
			samsung_disp_powerup();
		if(!samsung_state.display_on)
			samsung_disp_on();
	}
#endif

	spin_lock(&bl_ctrl_lock);

	if ( board_lcd_hw_revision == 1 ){ //SMD
		if(level > 0) {
			if(level < MIN_BRIGHTNESS_VALUE) {
				tune_level = AAT_DIMMING_VALUE; //DIMMING
				} else {
					
					for(i = 0; i < MAX_BRT_STAGE_AAT_SMD; i++) {
						if(level <= brt_table_aat_smd[i].level && level > brt_table_aat_smd[i+1].level) {
							tune_level = brt_table_aat_smd[i].tune_level;
							break;
						}
					}
				}
			} /*  BACKLIGHT is AAT1402 model */
	}else if ( board_lcd_hw_revision == 2 ){ //SONY
		if(level > 0) {
			if(level < MIN_BRIGHTNESS_VALUE) {
				tune_level = AAT_DIMMING_VALUE; //DIMMING
				} else {
					
					for(i = 0; i < MAX_BRT_STAGE_AAT; i++) {
						if(level <= brt_table_aat[i].level && level > brt_table_aat[i+1].level) {
							tune_level = brt_table_aat[i].tune_level;
							break;
						}
					}
				}
			} /*  BACKLIGHT is AAT1402 model */
	}else{ //HYSYS, 3
		if(level > 0) {
			if(level < MIN_BRIGHTNESS_VALUE) {
				tune_level = AAT_DIMMING_VALUE; //DIMMING
				} else {
					for(i = 0; i < MAX_BRT_STAGE_AAT_HYSYS; i++) {
						if(level <= brt_table_aat_hysys[i].level && level > brt_table_aat_hysys[i+1].level) {
							tune_level = brt_table_aat_hysys[i].tune_level;
							break;
						}
					}
				}
			} /*  BACKLIGHT is AAT1402 model */
	}		

	pr_info("%s:%d,%d\n", __func__, level, tune_level);//20 31

	if(!tune_level) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			mdelay(3);
	} else {
		for(;tune_level>0;tune_level--) 
		{
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(3);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3);

		}
	}
		mdelay(1);

	spin_unlock(&bl_ctrl_lock);

}

static void samsung_disp_powerup(void)
{
	if (!samsung_state.disp_powered_up && !samsung_state.display_on) {
		
		gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		
		/* Reset the hardware first */
		gpio_set_value(lcd_reset, 1);
		mdelay(10);
		gpio_set_value(lcd_reset, 0);
		mdelay(20);
		gpio_set_value(lcd_reset, 1);
		mdelay(10);
//		mdelay(20);

		
		samsung_state.disp_powered_up = TRUE;
	}
}

static void samsung_disp_powerdown(void)
{
	gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	
	/* Reset Assert */
	gpio_set_value(lcd_reset, 0);
	
	mdelay(1);
	gpio_set_value(spi_cs, 0);

	samsung_state.disp_powered_up = FALSE;
}

static struct work_struct disp_on_delayed_work;
static void samsung_disp_on_delayed_work(struct work_struct *work_ptr)
{
	if(board_lcd_hw_revision==3) //for HYDIS
		Setting_Table_HYDIS();
	
	else{
		samsung_serigo_list(panel_on_sequence,
			sizeof(panel_on_sequence)/sizeof(*panel_on_sequence));
	}
	if(!First_Disp_Power_On) {
		First_Disp_Power_On = TRUE;
		lcdc_samsung_set_brightness_in_blu(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);
	}

	if(delayed_backlight_value != -1) {
		lcdc_samsung_set_brightness_in_blu(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);
	}
}

static void samsung_disp_on(void)
{
#ifdef USE_DELAYED_WORK_DISP_ON
	if (samsung_state.disp_powered_up && !samsung_state.display_on) {

		INIT_WORK(&disp_on_delayed_work, samsung_disp_on_delayed_work);
		schedule_work(&disp_on_delayed_work);
		samsung_state.display_on = TRUE;
#else
	if(board_lcd_hw_revision==1) //for SMD
	{
		if (samsung_state.disp_powered_up && !samsung_state.display_on) {
			samsung_serigo_list(panel_on_sequence_smd,
				sizeof(panel_on_sequence_smd)/sizeof(*panel_on_sequence_smd));
				samsung_state.display_on = TRUE;
		}
	}
	else if(board_lcd_hw_revision==3) //for HYDIS
	{
		if (samsung_state.disp_powered_up && !samsung_state.display_on) {
			Setting_Table_HYDIS();
			samsung_state.display_on = TRUE;
		}
	}
	else
	{
		if (samsung_state.disp_powered_up && !samsung_state.display_on) {
			//mdelay(20);
			samsung_serigo_list(panel_on_sequence,
				sizeof(panel_on_sequence)/sizeof(*panel_on_sequence));
			samsung_state.display_on = TRUE;
		}
	}
	if(!First_Disp_Power_On) {
		First_Disp_Power_On = TRUE;
		lcdc_samsung_set_brightness_in_blu(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);
	}
	if(delayed_backlight_value != -1) {
		lcdc_samsung_set_brightness_in_blu(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);
	}
#endif

}

static int lcdc_samsung_panel_on(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	if (!samsung_state.disp_initialized) {

		acpuclk_usr_set_max();
		lcdc_samsung_pdata->panel_config_gpio(1);
		samsung_spi_init();
//		mdelay(50);	
		samsung_disp_powerup();
		samsung_disp_on();
		samsung_state.disp_initialized = TRUE;
#ifdef LCD_DET_ENABLE
		if((board_lcd_hw_revision==3)||(board_lcd_hw_revision==1))  //for HYDIS and SMD
		{
			if (irq_disabled) 
			{      
				enable_irq ( LCD_DET_ENABLE_IRQ );
				irq_disabled = FALSE;
				pr_info("%s - enable_irq, ESD_count is %d\n", __func__, ESD_count );
			}
		
			init_timer(&lcd_esd_timer);
			lcd_esd_timer.function = lcd_esd_timer_handler;
			lcd_esd_timer.expires = jiffies + 2*HZ;  // make panel_initialized true after 2 sec
			add_timer(&lcd_esd_timer);
		}
#endif
	}


	return 0;
}

static int lcdc_samsung_panel_off(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);

#ifdef LCD_DET_ENABLE
	if((board_lcd_hw_revision==3)||(board_lcd_hw_revision==1))  //for HYDIS and SMD
	{
  		disable_irq_nosync ( LCD_DET_ENABLE_IRQ);
  		irq_disabled = TRUE;
		pr_info("%s - disable_irq_nosync\n", __func__ );
	}
#endif

	if (samsung_state.disp_powered_up && samsung_state.display_on) {
#ifdef LCD_DET_ENABLE
		if((board_lcd_hw_revision==3)||(board_lcd_hw_revision==1))  //for HYDIS and SMD
		{
			del_timer(&lcd_esd_timer);
      	          panel_initialized = FALSE;
		}
#endif
		/* 0x10: Sleep In */
		samsung_write_cmd(0x10);
		mdelay(120);
		samsung_disp_powerdown();
		lcdc_samsung_pdata->panel_config_gpio(0);
		samsung_state.display_on = FALSE;
		samsung_state.disp_initialized = FALSE;
	}
	return 0;
}

#ifdef SYSFS_LCD_CONNECT_CHECK
static ssize_t smd_lcd_connect_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    if (MicroJigUARTOffStatus && board_lcd_hw_revision == 0)
        smd_lcd_connected = 1;

    pr_info("%s : smd_lcd_connected : %d [JIG:%d, LCD:%d]\n", __func__, smd_lcd_connected, MicroJigUARTOffStatus, board_lcd_hw_revision);
    return snprintf(buf, PAGE_SIZE, "%d\n", smd_lcd_connected);
}
static DEVICE_ATTR(lcd_connected, S_IRUGO | S_IWUGO, smd_lcd_connect_show, NULL);
#endif

#ifdef SYSFS_DEBUG_CMD
static ssize_t samsung_rda_cmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = snprintf(buf, PAGE_SIZE, "n/a\n");
	pr_info("%s: 'n/a'\n", __func__);
	return ret;
}

static ssize_t samsung_wta_cmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	uint32 cmd;

	sscanf(buf, "%x", &cmd);
	samsung_write_cmd((u8)cmd);

	return ret;
}

static DEVICE_ATTR(cmd, S_IRUGO | S_IWUGO, samsung_rda_cmd, samsung_wta_cmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};
#endif

#ifdef SYSFS_POWER_CONTROL
static int samsung_lcd_power_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", samsung_state.disp_initialized);
}

static int samsung_lcd_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct platform_device* pdev;

	if (len < 1)
		return -EINVAL;

	pdev = container_of(dev, struct platform_device, dev);

	if (strnicmp(buf, "on", 2) == 0 || strnicmp(buf, "1", 1) == 0) {
		lcdc_samsung_panel_on(pdev);
		lcdc_samsung_set_brightness_in_blu(15);
	} else if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0) {
		lcdc_samsung_set_brightness_in_blu(0);
		lcdc_samsung_panel_off(pdev);
	} else
		return -EINVAL;

	return len;
}

static DEVICE_ATTR(lcd_power, 0664, samsung_lcd_power_show, samsung_lcd_power_store);
#endif

static int samsung_lcd_read_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", board_lcd_hw_revision);
}

static DEVICE_ATTR(lcd_read_type, 0664, samsung_lcd_read_type_show, NULL);
static int samsung_lcdtype_file_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    
	if(board_lcd_hw_revision == 1)  //for SMD
		count = sprintf(buf, "SMD_LMS369KF01\n");
	
	else if(board_lcd_hw_revision == 2)  //for SONY
		count = sprintf(buf, "SON_S6D16A0x22\n");
	
	else if(board_lcd_hw_revision == 3) //for HYDIS
		count = sprintf(buf, "HYD_HVA37WV1\n");
	
	else
	      count = sprintf(buf, "lcdtype error\n");

	return count;
}

static DEVICE_ATTR(lcdtype_file, 0664, samsung_lcdtype_file_show, NULL);

static int samsung_probe(struct platform_device *pdev)
{
#ifdef SYSFS_DEBUG_CMD
	struct platform_device *fb_dev;
	struct msm_fb_data_type *mfd;
	int rc;
#endif

#ifdef SYSFS_LCD_CONNECT_CHECK
    int dcferr;
#endif

#ifdef SYSFS_POWER_CONTROL
	int err;
	err = device_create_file(&(pdev->dev), &dev_attr_lcd_power);
	if (err < 0)
		pr_err("%s: failed to add entries\n", __func__);
#endif

#ifdef SYSFS_LCD_CONNECT_CHECK
    dcferr = device_create_file(&(pdev->dev), &dev_attr_lcd_connected);
    if (dcferr <0) {
        pr_err("%s: failed to add 'lcd_connected' entry\n", __func__);
    }
#endif

	err = device_create_file(&(pdev->dev), &dev_attr_lcd_read_type);
	if (err < 0)
		pr_err("%s: failed to add entries\n", __func__);
		
	err = device_create_file(&(pdev->dev), &dev_attr_lcdtype_file);
	if (err < 0)
		pr_err("%s: failed to add entries\n", __func__);

	pr_info("%s: id=%d\n", __func__, pdev->id);
	if (pdev->id == 0) {
		lcdc_samsung_pdata = pdev->dev.platform_data;



#ifdef LCD_DET_ENABLE
	if((board_lcd_hw_revision==3)||(board_lcd_hw_revision==1))  //for HYDIS and SMD
	{
		irq_set_irq_type(LCD_DET_ENABLE_IRQ, IRQ_TYPE_EDGE_FALLING);
		err = request_threaded_irq(LCD_DET_ENABLE_IRQ, NULL,s6d16a0x_esd_irq_handler, IRQF_TRIGGER_FALLING, "LCD_ESD_DET", (void*)pdev->dev.platform_data);

		if (err)
		{
		pr_info ( "%s, request_irq failed ESD_DET, ret= %d\n", __func__, err);
		}
		else{
		pr_info ( "%s - irq is requested normally\n", __func__ );
		}
	}
#endif
		return 0;
	}

#ifndef SYSFS_DEBUG_CMD
	msm_fb_add_device(pdev);
#else
	fb_dev = msm_fb_add_device(pdev);
	mfd = platform_get_drvdata(fb_dev);
	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &fs_attr_group);
	if (rc) {
		pr_err("%s: sysfs group creation failed, rc=%d\n", __func__,
			rc);
		return rc;
	}
#endif
	return 0;
}

static void lcdc_samsung_shutdown(struct platform_device *pdev)
{
	pr_info("start %s\n", __func__);
	gpio_set_value(GPIO_BL_CTRL, 0);
	gpio_set_value(lcd_reset, 0);
}

static struct platform_driver this_driver = {
	.probe		= samsung_probe,
	.shutdown = lcdc_samsung_shutdown,
	.driver.name	= "lcdc_samsung_ancora",
};

static struct msm_fb_panel_data samsung_panel_data = {
	.on = lcdc_samsung_panel_on,
	.off = lcdc_samsung_panel_off,
	.set_backlight = lcdc_samsung_set_backlight,
};

static struct platform_device this_device = {
	.name	= "lcdc_panel",
	.id	= 1,
	.dev.platform_data = &samsung_panel_data,
};

static int __init lcdc_samsung_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#if defined(CONFIG_MACH_ANCORA_TMO)
 	board_lcd_hw_revision =	 ( ( gpio_get_value(177) << 1 ) | gpio_get_value(178));
 	pr_info ( "%s, For Recovery Mode lcd_hw_revision = %d\n", __func__, board_lcd_hw_revision);
#endif


#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_samsung_ancora")) {
		pr_err("%s: detect failed\n", __func__);
		return 0;
	}
#endif
if (board_lcd_hw_revision==1)  //for  smd 
{
	ret = platform_driver_register(&this_driver);
	if (ret)
	{
		pr_err("%s: driver register failed, rc=%d\n", __func__, ret);
		return ret;
	}

	pinfo = &samsung_panel_data.panel_info;
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

	pinfo->lcdc.h_back_porch = SMD_LCDC_HBP;
	pinfo->lcdc.h_front_porch = SMD_LCDC_HFP;
	pinfo->lcdc.h_pulse_width = SMD_LCDC_HPW;
	pinfo->lcdc.v_back_porch = SMD_LCDC_VBP;
	pinfo->lcdc.v_front_porch = SMD_LCDC_VFP;
	pinfo->lcdc.v_pulse_width = SMD_LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;

}

else if(board_lcd_hw_revision==3) //for HYDIS
{
	
	ret = platform_driver_register(&this_driver);
	if (ret)
	{
		pr_err("%s: driver register failed, rc=%d\n", __func__, ret);
		return ret;
	}
	pinfo = &samsung_panel_data.panel_info;
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

	pinfo->lcdc.h_back_porch = HYDIS_LCDC_HBP;
	pinfo->lcdc.h_front_porch = HYDIS_LCDC_HFP;
	pinfo->lcdc.h_pulse_width = HYDIS_LCDC_HPW;
	pinfo->lcdc.v_back_porch = HYDIS_LCDC_VBP;
	pinfo->lcdc.v_front_porch = HYDIS_LCDC_VFP;
	pinfo->lcdc.v_pulse_width = HYDIS_LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;
}	
else //for etc...
{
	ret = platform_driver_register(&this_driver);
	if (ret)
	{
		pr_err("%s: driver register failed, rc=%d\n", __func__, ret);
		return ret;
	}
	pinfo = &samsung_panel_data.panel_info;
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
}



	ret = platform_device_register(&this_device);
	if (ret) {
		pr_err("%s: device register failed, rc=%d\n", __func__, ret);
		goto fail_driver;
	}

	pr_info("%s: SUCCESS (BitBang)\n", __func__);

	return ret;

fail_driver:
	platform_driver_unregister(&this_driver);

	return ret;
}
#ifdef LCD_DET_ENABLE

static int lcdc_samsung_panel_on_esd(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	if (!samsung_state.disp_initialized) {

		acpuclk_usr_set_max();
		lcdc_samsung_pdata->panel_config_gpio(1);
		samsung_spi_init();
//		mdelay(50);	
		samsung_disp_powerup();
		samsung_disp_on();
		samsung_state.disp_initialized = TRUE;
#ifdef LCD_DET_ENABLE
		if (irq_disabled) 
		{      
			enable_irq ( LCD_DET_ENABLE_IRQ );
			irq_disabled = FALSE;
			pr_info("%s - enable_irq, ESD_count is %d\n", __func__, ESD_count );
		}

#endif
		panel_initialized=TRUE;

	}


	return 0;
}

void s6d16a0x_esd ( void )
{
	if ( panel_initialized )
	{
		++ESD_count;
		pr_info("%s - ESD count - %d.\n", __func__, ESD_count );
		lcdc_samsung_panel_off ( NULL );
		msleep(20);
		lcdc_samsung_panel_on_esd ( NULL );
		del_timer(&lcd_esd_timer2);
		init_timer(&lcd_esd_timer2);
	       lcd_esd_timer2.function = lcd_esd_timer_handler2;
	       lcd_esd_timer2.expires = jiffies + 4*HZ;  // make panel_initialized true after 2 sec
	       add_timer(&lcd_esd_timer2);
	}
}

static DECLARE_WORK ( lcd_esd_work, s6d16a0x_esd );

static irqreturn_t s6d16a0x_esd_irq_handler(int irq, void *handle)
{
	//pr_info("%s - IRQ\n", __func__ );

	if( samsung_state.disp_initialized )
	{
		schedule_work ( &lcd_esd_work );
	}

	return IRQ_HANDLED;
}

static void lcd_esd_timer_handler(unsigned long data)
{
       if (samsung_state.disp_initialized) 
	{      
		panel_initialized = TRUE;
       	}
}

static void lcd_esd_timer_handler2(unsigned long data)
{
       if (samsung_state.disp_initialized) 
	{      
		if(gpio_get_value(PM8058_GPIO_PM_TO_SYS(25)) == 0)
		{
			
			pr_info("%s - ESD 2 count - %d.\n", __func__, gpio_get_value(PM8058_GPIO_PM_TO_SYS(25)) );
			schedule_work ( &lcd_esd_work );

		}
       }
}

#endif
module_init(lcdc_samsung_panel_init);
