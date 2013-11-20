/* include/linux/phantom_keypress_filter.h
 *
 * Copyright (C) 2013, Cristoforo Cataldo <cristoforo.cataldo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_PHANTOM_KEYPRESS_FILTER_H
#define _LINUX_PHANTOM_KEYPRESS_FILTER_H

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>

/* Module version */
#define PKF_VERSION							2

/* Default filtering parameter values for MENU and BACK keys */
#define DEFAULT_MENUBACK_ENABLED			true
#define DEFAULT_MENUBACK_INTERRUPT_CHECKS	10
#define DEFAULT_MENUBACK_FIRST_ERROR_WAIT	500
#define DEFAULT_MENUBACK_LAST_ERROR_WAIT	100

/* Default filtering parameter values for HOME key */
#define DEFAULT_HOME_ENABLED				false
#define DEFAULT_HOME_ALLOWED_IRQS	 		4
#define DEFAULT_HOME_REPORT_WAIT			12

/* Min and max range constraint for sysfs editable values */
#define MIN_MENUBACK_INTERRUPT_CHECKS		1
#define MAX_MENUBACK_INTERRUPT_CHECKS		10
#define MIN_MENUBACK_FIRST_ERROR_WAIT		50
#define MAX_MENUBACK_FIRST_ERROR_WAIT		1000
#define MIN_MENUBACK_LAST_ERROR_WAIT		50
#define MAX_MENUBACK_LAST_ERROR_WAIT		1000
#define MIN_HOME_ALLOWED_IRQS				1
#define MAX_HOME_ALLOWED_IRQS				32
#define MIN_HOME_REPORT_WAIT				5
#define MAX_HOME_REPORT_WAIT				25

/* Struct for MENU and BACK keys filtering data */
struct pkf_menuback_data {
	bool enabled;
	u8 interrupt_checks;
	u16 first_err_wait;
	u16 last_err_wait;
	unsigned long ignored_kp;
};

/* Struct for HOME key filtering data */
struct pkf_home_data {
	bool enabled;
	u8 allowed_irqs;
	u8 report_wait;
	unsigned long ignored_kp;
};

/* Struct for the incoming HOME key presses */
struct pkf_home_incoming_kp {
	u32 states;			/* Collected key states (bits array) */
	u8 states_count;	/* Number of the collected key states */
	u32 irqs_count;		/* Number of incoming irqs */
};

/* Mutex lock for HOME key filtering */
static DEFINE_MUTEX(pkf_home_mutex);

/* Structs inizialized inside the module body */
extern struct pkf_menuback_data *pkf_menuback;
extern struct pkf_home_data *pkf_home;
extern struct pkf_home_incoming_kp *pkf_home_inc_kp;

/*
 * Function to count MENU or BACK ignored key press (possible phantom key press
 * If ignored_kp reachs the max unsigned long value, avoid the reset to 0
 */
static inline void count_ignored_menuback_kp(void)
{
	if (pkf_menuback->ignored_kp < ULONG_MAX)
		pkf_menuback->ignored_kp++;
}

/*
 * Function to count HOME key ignored key press (possible phantom key press
 * If ignored_kp reachs the max unsigned long value, avoid the reset to 0
 */
static inline void count_ignored_home_kp(void)
{
	if (pkf_home->ignored_kp < ULONG_MAX)
		pkf_home->ignored_kp++;
}

/*
 * Function to clear the collected key states bits array
 */
static inline void clear_homekey_states(void)
{
	if (pkf_home_inc_kp->states_count > 0) {
		pkf_home_inc_kp->states = 0;
		pkf_home_inc_kp->states_count = 0;
	}
}

/*
 * Function to store the home key status to be collected
 */
static inline void collect_homekey_status(int status)
{
	/* If status != 0, set the bit to 1 at last position */
	if (status)
		__set_bit(pkf_home_inc_kp->states_count, (unsigned long *)&pkf_home_inc_kp->states);
	/* Else, set the bit to 0 at last position */
	else
		__clear_bit(pkf_home_inc_kp->states_count, (unsigned long *)&pkf_home_inc_kp->states);

	/* Increment the number of collected status */
	pkf_home_inc_kp->states_count++;
}

/*
 * Function to get the home key status collected at the desired position
 */
static inline int get_homekey_status(int pos)
{
	return test_bit(pos, (unsigned long *)&pkf_home_inc_kp->states);
}
#endif
