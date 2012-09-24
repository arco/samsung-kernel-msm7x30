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
 
struct samsung_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	int brightness;
};

struct samsung_spi_data {
	u8 addr;
	u8 len;
	u8 data[50];
	u8 delay;
};

enum {
    GPIO_INPUT,
    GPIO_OUTPUT,
};

enum {
    GPIO_NO_PULL,
    GPIO_PULL_DOWN,
    GPIO_KEEPER,
    GPIO_PULL_UP,
};

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

//#define DEBUG
//#define SYSFS_DEBUG_CMD
#define SYSFS_POWER_CONTROL

#define GPIO_BL_CTRL	16
#define GPIO_LCD_DET	26

#define MAX_BRIGHTNESS_LEVEL 255
#define DFT_BRIGHTNESS_LEVEL 		141//151//129//170//158//120  // When we are booting, Application set brightness-leverl 140, Brightness-level 140 is mapped Backlight-level 34. 
#define LOW_BRIGHTNESS_LEVEL 30
#define MAX_BACKLIGHT_VALUE2	25//27//30
#define MAX_BACKLIGHT_VALUE	25//27//30
#define LOW_BACKLIGHT_VALUE	6//5//6//5
#define DIM_BACKLIGHT_VALUE		3//4//2
#define DFT_BACKLIGHT_VALUE		( (DFT_BRIGHTNESS_LEVEL - LOW_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE) / (MAX_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE )
#define MAX_BRIGHTNESS_IN_BLU	32 // backlight-IC MAX VALUE
#define DEFAULT_LCD_ON_BACKLIGHT_LEVEL 23

#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HPW	2
#define LCDC_HBP		20
#define LCDC_HFP		20
#define LCDC_VPW		2
#define LCDC_VBP		5
#define LCDC_VFP		5
#define LCDC_PCLK		(LCDC_FB_XRES + LCDC_HBP + LCDC_HPW + LCDC_HFP) * (LCDC_FB_YRES + LCDC_VBP + LCDC_VPW + LCDC_VFP) * 2

#define SMD_LCDC_HPW		4
#define SMD_LCDC_HBP		40
#define SMD_LCDC_HFP		10
#define SMD_LCDC_VPW		1
#define SMD_LCDC_VBP		7
#define SMD_LCDC_VFP		6
#define HYDIS_LCDC_HPW		23
#define HYDIS_LCDC_HBP		10
#define HYDIS_LCDC_HFP		5
#define HYDIS_LCDC_VPW		12
#define HYDIS_LCDC_VBP		5
#define HYDIS_LCDC_VFP		5


static struct samsung_spi_data panel_on_sequence[] = {
    { .addr = 0xF0, .len = 2, .data = { 0x5A, 0x5A }, .delay = 0 },
    { .addr = 0xF1, .len = 2, .data = { 0x5A, 0x5A }, .delay = 0 },
    { .addr = 0x36, .len = 1, .data = { 0xD0 }, .delay = 0 },
    { .addr = 0xD0, .len = 2, .data = { 0x5A, 0x5A }, .delay = 0 },
 
    { .addr = 0xB7, .len = 3, .data = { 0x00, 0x11, 0x11 }, .delay = 0 },
    { .addr = 0xB8, .len = 2, .data = { 0x0C, 0x10 }, .delay = 0 },
    { .addr = 0xB9, .len = 2, .data = { 0x00, 0x06 }, .delay = 0 },
 
    { .addr = 0xBB, .len = 1, .data = { 0x00 }, .delay = 0 },
    { .addr = 0xC0, .len = 3, .data = { 0x80, 0x80, 0x00 }, .delay = 0 },
    { .addr = 0xC1, .len = 1, .data = { 0x08 }, .delay = 0 },
    { .addr = 0xD2, .len = 1, .data = { 0x80 }, .delay = 0 },
    { .addr = 0xEE, .len = 1, .data = { 0x12 }, .delay = 0 },
 
    { .addr = 0xF2, .len = 9, .data = { 0x00, 0x30, 0x84, 0x84, 0x57, 0x57, 0x10, 0x41, 0x00 }, .delay = 0 },
 
    { .addr = 0xF3, .len = 12, .data = { 0x00, 0x10, 0x25, 0x10, 0x2D, 0x2D, 0x24, 0x2D, 0x10, 0x10,
                                        0x0A, 0x37 }, .delay = 0 },
    { .addr = 0xF4, .len = 10, .data = { 0x00, 0x20, 0x00, 0xAF, 0x64, 0x00, 0xAF, 0x64, 0x01, 0x01 }, .delay = 0 },
    { .addr = 0xF5, .len = 10, .data = { 0x00, 0x00, 0x57, 0x11, 0x05, 0x01, 0x01, 0x01, 0x02, 0x02 }, .delay = 0 },
    { .addr = 0xF6, .len = 5, .data = { 0xA1, 0x00, 0xC0, 0x00, 0x00 }, .delay = 0 },
    { .addr = 0xF7, .len = 17, .data = { 0x04, 0x6E, 0x00, 0x12, 0x03, 0x0D, 0x0A, 0x16, 0x05, 0x04,
                                        0x0E, 0x04, 0x04, 0x00, 0x96, 0x01, 0x18 }, .delay = 0 },
    { .addr = 0xF8, .len = 17, .data = { 0x04, 0x6E, 0x00, 0x00, 0x03, 0x0D, 0x0A, 0x16, 0x05, 0x04,
                                         0x0E, 0x04, 0x04, 0x00, 0x96, 0x01, 0x18 }, .delay = 0 },
    { .addr = 0xF9, .len = 22, .data = { 0x01, 0x02, 0x05, 0x04, 0x0A, 0x0B, 0x00, 0x06, 0x11, 0x10,
                                        0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF,
                                        0xFF, 0xF0 }, .delay = 0 },
 
    { .addr = 0xFA, .len = 45, .data = { 0x30, 0x3F, 0x38, 0x29, 0x38, 0x3D, 0x3C, 0x2D, 0x27, 0x3E,
                                        0x2E, 0x31, 0x37, 0x32, 0x30, 0x00, 0x3F, 0x38, 0x29, 0x38,
                                        0x3C, 0x39, 0x2B, 0x24, 0x37, 0x21, 0x1C, 0x1D, 0x0F, 0x04,
                                        0x00, 0x3F, 0x3A, 0x2C, 0x3B, 0x3B, 0x3B, 0x2C, 0x25, 0x36,
                                        0x1E, 0x17, 0x18, 0x0B, 0x03 }, .delay = 0 },
 
    { .addr = 0xFB, .len = 45, .data = { 0x00, 0x0F, 0x0F, 0x0D, 0x28, 0x2E, 0x31, 0x21, 0x19, 0x32,
                                        0x23, 0x22, 0x27, 0x16, 0x07, 0x00, 0x3F, 0x3B, 0x30, 0x42,
                                        0x43, 0x3E, 0x28, 0x1C, 0x34, 0x26, 0x23, 0x27, 0x16, 0x07,
                                        0x00, 0x3F, 0x3C, 0x34, 0x47, 0x48, 0x41, 0x29, 0x1B, 0x33,
                                        0x24, 0x24, 0x24, 0x13, 0x05}, .delay = 0 },
    { .addr = 0x11, .len = 0, .data = { 0x00 }, .delay = 5 },
 
 
 
    { .addr = 0x3A, .len = 1, .data = { 0x77 }, .delay = 130 },
    { .addr = 0x29, .len = 0, .data = { 0x00, 0x00, 0x03, 0xDF }, .delay = 10 },
};
 

static struct samsung_spi_data panel_on_sequence_smd[] = {
	{ .addr = 0x36, .len = 1, .data = { 0x09 }, .delay = 0 },
	{ .addr = 0xB0, .len = 1, .data = { 0x00 }, .delay = 0 },
	{ .addr = 0xC0, .len = 2, .data = { 0x28, 0x08 }, .delay = 0 },
	{ .addr = 0xC1, .len = 5, .data = { 0x01, 0x30, 0x15, 0x05, 0x22 }, .delay = 0 },
	{ .addr = 0xC4, .len = 3, .data = { 0x10, 0x01, 0x00 }, .delay = 0 },
	{ .addr = 0xC5, .len = 9, .data = {0x06, 0x55, 0x03, 0x07, 0x07, 0x33, 0x00, 0x01, 0x03 }, .delay = 0 },
	{ .addr = 0xC6, .len = 1, .data = { 0x01 }, .delay = 0 },
	{ .addr = 0xC8, .len = 38, .data = {0x00, 0x7A, 0x09, 0x11, 0x1F, 0x27, 0x27, 0x2E, 0x2E, 0x38, 
										0x45, 0x46, 0x48, 0x46, 0x4F, 0x5A, 0x5A, 0x5D, 0x3C, 0x00, 
										0x7A, 0x09, 0x11, 0x1F, 0x27, 0x27, 0x2E, 0x2E, 0x38, 0x45, 
										0x46, 0x48, 0x46, 0x4F, 0x5A, 0x5A, 0x5D, 0x3C }, .delay = 0 },
	{ .addr = 0xC9, .len = 38, .data = {0x00, 0x41, 0x13, 0x20, 0x34, 0x41, 0x44, 0x46, 0x4A, 0x54, 
										0x5C, 0x5D, 0x5D, 0x5A, 0x5E, 0x65, 0x64, 0x65, 0x3E, 0x00, 
										0x41, 0x13, 0x20, 0x34, 0x41, 0x44, 0x46, 0x4A, 0x54, 0x5C, 
										0x5D, 0x5D, 0x5A, 0x5E, 0x65, 0x64, 0x65, 0x3E }, .delay = 0 },
	{ .addr = 0xCA, .len = 38, .data = {0x00, 0x28, 0x15, 0x25, 0x3B, 0x4A, 0x4D, 0x50, 0x54, 0x5E, 
										0x66, 0x66, 0x66, 0x62, 0x67, 0x6B, 0x69, 0x68, 0x40, 0x00, 
										0x28, 0x15, 0x25, 0x3B, 0x4A, 0x4D, 0x50, 0x54, 0x5E, 0x66, 
										0x66, 0x66, 0x62, 0x67, 0x6B, 0x69, 0x68, 0x40 }, .delay = 0 },
	{ .addr = 0xD1, .len = 2, .data = {  0x33, 0x13 }, .delay = 0 },
	
	{ .addr = 0xD2, .len = 3, .data = { 0x11, 0x00, 0x00 }, .delay = 0 },
	{ .addr = 0xD3, .len = 2, .data = {0x4F, 0x4E }, .delay = 0 },
	
	{ .addr = 0xD5, .len = 4, .data = { 0x2F, 0x11, 0x1E, 0x46 }, .delay = 0 },
	{ .addr = 0xD6, .len = 2, .data = {0x11, 0x0A }, .delay = 0 },
	
	{ .addr = 0x11, .len = 0, .data = { 0x00 }, .delay = 10 },
	
	{ .addr = 0x00, .len = 0, .data = { 0x00 }, .delay = 120 },
	
	{ .addr = 0x29, .len = 0, .data = {0x00 }, .delay = 0 },
};

static struct samsung_spi_data colmod_sequence[] = {
	{ .addr = 0x3A, .len = 1, .data = { 0x77 } },
	/* It needs 130ms delay */
};

