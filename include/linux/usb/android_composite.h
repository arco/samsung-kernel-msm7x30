/*
 * Platform data for Android USB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H

#include <linux/usb/composite.h>
#include <linux/if_ether.h>

#define USB_MODE_RNDIS      (0X1<<0)
#define USB_MODE_DIAG       (0X1<<1)
#define USB_MODE_ADB        (0X1<<2)
#define USB_MODE_MODEM      (0X1<<3)
#define USB_MODE_NMEA       (0X1<<4)
#define USB_MODE_RMNET      (0X1<<5)
#define USB_MODE_UMS        (0X1<<6)
#define USB_MODE_ACM        (0X1<<7)
#define USB_MODE_MTP        (0X1<<8)
#define USB_MODE_ASKON      (0x1<<9)
#define USB_MODE_UMS_CDFS   (0x1<<10) //LnT added
#define USB_MODE_KIES   (0x1<<11) //KIES

struct android_usb_function {
	struct list_head	list;
	char			*name;
	int 			(*bind_config)(struct usb_configuration *c);
};

struct android_usb_product {
	/* Default product ID. */
	__u16 product_id;

	/* List of function names associated with this product.
	 * This is used to compute the USB product ID dynamically
	 * based on which functions are enabled.
	 */
	int num_functions;
	char **functions;
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Below variables are used for Samsung composite framework. */
        __u8 bDeviceClass;
	__u8 bDeviceSubClass;
	__u8 bDeviceProtocol;
	int  mode; /* if product id is same, you have to refer this mode value. */
	char *s;
#endif
};

struct android_usb_platform_data {
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	__u16 version;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;

	/* List of available USB products.
	 * This is used to compute the USB product ID dynamically
	 * based on which functions are enabled.
	 * if num_products is zero or no match can be found,
	 * we use the default product ID
	 */
	int num_products;
	struct android_usb_product *products;

	/* List of all supported USB functions.
	 * This list is used to define the order in which
	 * the functions appear in the configuration's list of USB interfaces.
	 * This is necessary to avoid depending upon the order in which
	 * the individual function drivers are initialized.
	 */
	int num_functions;
	char **functions;
};

/* Platform data for "usb_mass_storage" driver. */
struct usb_mass_storage_platform_data {
	/* Contains values for the SC_INQUIRY SCSI command. */
	char *vendor;
	char *product;
	int release;

	char can_stall;
	/* number of LUNS */
	int nluns;
};

/* Platform data for USB ethernet driver. */
struct usb_ether_platform_data {
	u8	ethaddr[ETH_ALEN];
	u32	vendorID;
	const char *vendorDescr;
};

extern void android_register_function(struct android_usb_function *f);

extern void android_enable_function(struct usb_function *f, int enable);


extern void samsung_enable_function(int mode);

#endif	/* __LINUX_USB_ANDROID_H */
