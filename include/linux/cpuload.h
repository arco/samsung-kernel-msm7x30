/*
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *      Global GSM S/W 'jaecheol kim'
 *      email : jc22.kim@samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __LINUX_CPULOAD_H__
#define __LINUX_CPULOAD_H__

extern int cpuload_log(int load, int target_clk, int newclk,
        const char *buf, size_t count);

#endif
