/*
 *  Copyright (C) 2011 Samsung Electronics
 *  jongmyeong ko <jongmyeong.ko@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SMB328A_CHARGER_H_
#define __SMB328A_CHARGER_H_

struct smb328a_platform_data {
	int		(*set_charger)(int);
	int		(*topoff_cb) (void);
	void 	(*hw_init)(void);
};

#endif

