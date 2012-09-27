/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, SAMSUNG. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */


#ifndef _LSI_MC7_H_ 
#define _LSI_MC7_H_
#include <linux/types.h>

#define FUNC_ENTRY_DEBUG
#ifdef FUNC_ENTRY_DEBUG
#define FUNC_ENTER() printk(KERN_INFO "LSI-MC7:%05d: %s\n", __LINE__, __func__)
#else
#define FUNC_ENTER()
#endif

#define MC7_DEBUG
#ifdef MC7_DEBUG
#define MC7DBG(fmt, args...) printk(KERN_INFO "LSI-MC7: " fmt, ##args)
#else
#define MC7DBG(fmt, args...) do { } while (0)
#endif

struct packet {
	unsigned char *firmware;
	unsigned short length;
};

/* Sensor related informations are saved here */
struct mc7_ctrls_t {
	const struct msm_camera_sensor_info *sensordata;
};

#endif /* _LSI_MC7_H_ */

