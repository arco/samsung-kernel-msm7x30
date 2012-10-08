/*
 * driver/misc/fsa9480.c - FSA9480 micro USB switch device driver
 *
 * Copyright (C) 2010 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c/fsa9480.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <mach/vreg.h>
#include <asm/io.h>
#include <linux/uaccess.h>

// #define DEBUG 1
#include <linux/device.h>

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
#include <linux/switch.h>
#include <mach/parameters.h>
extern struct device *switch_dev;
static int usb_state = 0;
#include <linux/usb/android_composite.h>
#include <mach/devs.h>// MSM8255

extern int android_usb_get_current_mode(void);
extern void android_usb_switch(int mode);

#include <linux/pm.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <mach/msm_iomap.h>
#include <linux/fs.h>
#include <linux/syscalls.h>

extern int charging_boot;
#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
extern int power_off_done;  // For Device Reset/Off
extern bool power_down;
#endif

extern int android_probe_done; // for check  android probe done 

#if defined(CONFIG_MACH_APACHE)
struct fsa9480_platform_data *fsa9480_pdata;
#endif

/* USB SWITCH CONTROL */
// 0: MSM , 
#define SWITCH_MSM		0

void usb_switch_state(void);
static int fsa9480_probe_done = 0;
int disable_vbus_flag =0 ;
#endif

#define _SUPPORT_SAMSUNG_AUTOINSTALLER_
#define dmsg(arg,...) printk("[USB_SWITCH] %s(%d): "arg,__FUNCTION__,__LINE__,##__VA_ARGS__)

#define DRIVER_NAME  "usb_mass_storage"   
void UsbIndicator(u8 state);
struct switch_dev indicator_dev;
u8 MicroUSBStatus=0;
static u8 MicroJigUSBOnStatus=0;
static u8 MicroJigUSBOffStatus=0;
bool MicroJigUARTOffStatus=0;
int askonstatus;
EXPORT_SYMBOL(MicroUSBStatus);
EXPORT_SYMBOL(UsbIndicator);
u8 FSA9480_Get_USB_Status(void)
{
      
	if( MicroUSBStatus | MicroJigUSBOnStatus | MicroJigUSBOffStatus |MicroJigUARTOffStatus)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(FSA9480_Get_USB_Status);

static ssize_t print_indicatorswitch_name(struct switch_dev *sdev, char *buf)
{  

    	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_indicatorswitch_state(struct switch_dev *sdev, char *buf)
{

return sprintf(buf, "%s\n", "online");
}

void UsbIndicator(u8 state)
{
	switch_set_state(&indicator_dev, state);
}

static struct i2c_client *pclient;

struct fsa9480_data {
	struct work_struct work;
};

int curr_usb_status = 0;
int curr_ta_status = 0;
int curr_uart_status = 0;
EXPORT_SYMBOL(curr_uart_status);

static u8 fsa9480_device1 = 0x0;
static u8 fsa9480_device2 = 0x0; 

// for debugging USB state
static u8 deb_dev1 = 0x0;
static u8 deb_dev2 = 0x0; 
static u8 deb_intr1 = 0x0;
static u8 deb_intr2 = 0x0;
static int deb_intb = 0;


#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static int g_dock;
static int g_default_ESN_status = 1;
static int curr_usb_path = 0;
static int curr_uart_path = 0;

int get_usb_cable_state(void)
{
	return usb_state;
}

/**********************************************************************
*    Name         : usb_state_show()
*    Description : for sysfs control (/sys/class/sec/switch/usb_state)
*                        return usb state using fsa9480's device1 and device2 register
*                        this function is used only when NPS want to check the usb cable's state.
*    Parameter   :
*
*
*    Return        : USB cable state's string
*                        USB_STATE_CONFIGURED is returned if usb cable is connected
***********************************************************************/
static ssize_t usb_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int cable_state = get_usb_cable_state();
	sprintf(buf, "%s\n", (cable_state & (CRB_JIG_USB<<8 | CRA_USB<<0 ))?"USB_STATE_CONFIGURED":"USB_STATE_NOTCONFIGURED");
	return sprintf(buf, "%s\n", buf);
}


/**********************************************************************
*    Name         : usb_state_store()
*    Description : for sysfs control (/sys/class/sec/switch/usb_state)
*                        noting to do.
*    Parameter   :
*
*
*    Return        : None
*
***********************************************************************/
static ssize_t usb_state_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

/*sysfs for usb cable's state.*/
static DEVICE_ATTR(usb_state, 0664, usb_state_show, usb_state_store);

#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
static int kies_status = 0;
static ssize_t KiesStatus_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(kies_status == 1)
		return sprintf(buf, "%s\n", "START");
	else if( kies_status == 2)
		return sprintf(buf, "%s\n", "STOP");
	else
		return sprintf(buf, "%s\n", "INIT");
}

static ssize_t KiesStatus_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	dmsg("buf=%s\n", buf);

	if (strncmp(buf, "START", 5) == 0)
	{
		kies_status = 1;
  	}
	else if (strncmp(buf, "STOP", 4) == 0)
	{
		kies_status = 2;
		UsbIndicator(2);
	}
	else if (strncmp(buf, "INIT", 4) == 0 )
	{
		kies_status = 0;
	}

	return size;
}

static DEVICE_ATTR(KiesStatus, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, KiesStatus_switch_show, KiesStatus_switch_store);
#endif /* _SUPPORT_SAMSUNG_AUTOINSTALLER_ */


struct switch_dev switch_dock_detection = {
		.name = "dock",	
};

#define DOCK_KEY_MASK	0x18
#define DOCK_KEY_SHIFT  3

typedef enum
{
	DOCK_KEY_VOLUMEUP = 0,
	DOCK_KEY_VOLUMEDOWN,
	DOCK_KEY_MAX,
} dock_key_type;

static unsigned int dock_keys[DOCK_KEY_MAX] = 
{
	KEY_VOLUMEDOWN,
	KEY_VOLUMEUP,
};

static const char * dock_keys_string[DOCK_KEY_MAX] = 
{
	"KEY_VOLUMEDOWN",
	"KEY_VOLUMEUP",
};

static struct input_dev * dock_key_input_dev;

static void dock_keys_input(dock_key_type key, int press) 
{
	if( key >= DOCK_KEY_MAX )
		return;

	input_report_key(dock_key_input_dev, dock_keys[key], press);
	input_sync(dock_key_input_dev);

	DEBUG_FSA9480("key pressed(%d) [%s] \n", press, dock_keys_string[key]);
}
#endif

static DECLARE_WAIT_QUEUE_HEAD(g_data_ready_wait_queue);

int fsa9480_i2c_write(unsigned char u_addr, unsigned char u_data);
int fsa9480_i2c_read(unsigned char u_addr, unsigned char *pu_data);

extern int batt_restart(void);

int fsa9480_i2c_tx_data(char* txData, int length)
{
	int rc; 

	if (!pclient) {
		printk(KERN_ERR "[FSA9480]: fsa9480_i2c_tx_data error pclient is NULL pointer\n");
		return -EINVAL;
	}

	struct i2c_msg msg[] = {
		{
			.addr = pclient->addr,
			.flags = 0,
			.len = length,
			.buf = txData,		
		},
	};

	rc = i2c_transfer(pclient->adapter, msg, 1);
	if (rc < 0) {
		printk(KERN_ERR "[FSA9480]: fsa9480_i2c_tx_data error %d\n", rc);
		return rc;
	}

	return 0;
}


int fsa9480_i2c_write(unsigned char u_addr, unsigned char u_data)
{
	int rc;
	unsigned char buf[2];

	buf[0] = u_addr;
	buf[1] = u_data;
    
	rc = fsa9480_i2c_tx_data(buf, 2);
	if(rc < 0)
		printk(KERN_ERR "[FSA9480]: txdata error %d add:0x%02x data:0x%02x\n",
			rc, u_addr, u_data);

	return rc;	
}

static int fsa9480_i2c_rx_data(char* rxData, int length)
{
	int rc;
	
	if (!pclient) {
		printk(KERN_ERR "[FSA9480]: fsa9480_i2c_rx_data error pclient is NULL pointer\n");
		return -EINVAL;
	}
	
	struct i2c_msg msgs[] = {
		{
			.addr = pclient->addr,
			.flags = 0,      
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = pclient->addr,
			.flags = I2C_M_RD|I2C_M_NO_RD_ACK,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(pclient->adapter, msgs, 2);
      
	if (rc < 0) {
		printk(KERN_ERR "[FSA9480]: fsa9480_i2c_rx_data error %d\n", rc);
		return rc;
	}
      
	return 0;
}

int fsa9480_i2c_read(unsigned char u_addr, unsigned char *pu_data)
{
	int rc;
	unsigned char buf;

	if (!pu_data) {
		printk(KERN_ERR "[FSA9480]: fsa9480_i2c_read error pu_data is NULL pointer\n");
		return -EINVAL;
	}
	
	buf = u_addr;
	rc = fsa9480_i2c_rx_data(&buf, 1);
	if (!rc)
		*pu_data = buf;
	else 
		printk(KERN_ERR "[FSA9480]: i2c read failed\n");
	return rc;	
}

void fsa9480_select_mode(int mode)
{
	unsigned char cont_reg = 0;
	
	printk("[fsa9480_select_mode]: mode = %x\n", mode);
	if (!pclient) 
		return;

    /* fsa9480 init sequence */
	fsa9480_i2c_write(REGISTER_CONTROL, mode); // FSA9480 Set AutoMode
	fsa9480_i2c_read(REGISTER_CONTROL, &cont_reg); // FSA9480 initilaization check
	printk("[fsa9480_select_mode]: Changed control reg 0x02 : 0x%x\n", cont_reg);
	
	/* delay 2 ms */
	msleep(2);	
}

EXPORT_SYMBOL(fsa9480_select_mode);

static void fsa9480_chip_init(void)
{
	unsigned char cont_reg = 0;
	int ret;
	
	printk("[FSA9480] fsa9480_chip_init \n");	
	if (!pclient) 
		return;

	fsa9480_i2c_write(REGISTER_INTERRUPTMASK1, 0x1ffc);
	if (ret < 0)
		printk("[FSA9480] %s : REGISTER_INTERRUPTMASK1 err %d\n", __func__, ret);

	fsa9480_i2c_write(REGISTER_CARKITMASK1, 0x07ff);
	if (ret < 0)
		printk("[FSA9480] %s : REGISTER_CARKITMASK1 err %d\n", __func__, ret);

	fsa9480_i2c_write(REGISTER_TIMINGSET1, 0x6);
	if (ret < 0)
		printk("[FSA9480] %s : REGISTER_TIMINGSET1 err %d\n", __func__, ret);
	
	/* fsa9480 init sequence */
	fsa9480_i2c_write(REGISTER_CONTROL, 0x1E); // FSA9480 Set AutoMode
	fsa9480_i2c_read(REGISTER_CONTROL, &cont_reg); // FSA9480 initilaization check
	printk("[FSA9480]: Initial control reg 0x02 : 0x%x\n", cont_reg);
	
	/* delay 2 ms */
	msleep(2);
	printk(KERN_INFO "[FSA9480]: fsa9480 sensor init sequence done\n");

	/* Remember attached devices */
	fsa9480_i2c_read(REGISTER_DEVICETYPE1, &fsa9480_device1);
	fsa9480_i2c_read(REGISTER_DEVICETYPE2, &fsa9480_device2);


	printk("[FSA9480] fsa9480_device1=0x%x, fsa9480_device2=0x%x\n",fsa9480_device1,fsa9480_device2);

	usb_state = (fsa9480_device2 << 8) | (fsa9480_device1 << 0);

	printk("[FSA9480] usb_state = %d\n",usb_state);


	if (fsa9480_device1 & CRA_USB)
	{
		curr_usb_status = 1;
	}
	else if (fsa9480_device1 & CRA_DEDICATED_CHG)
	{
		curr_ta_status = 1;
	}
	else if (fsa9480_device1 & CRA_CARKIT)
	{
		curr_ta_status = 1;
	}
	else if (fsa9480_device2 & ( CRB_JIG_UART_OFF | CRB_JIG_UART_ON) )
	{
	    MicroJigUARTOffStatus = 1;
	}
#if defined(CONFIG_MACH_APACHE)
	else if ( (fsa9480_device1==0) && (fsa9480_device2 & CRB_JIG_USB_OFF) ){
		curr_usb_status = 1;
	}
#endif
	
}

static int fsa9480_client(struct i2c_client *client)
{
	/* Initialize the fsa9480 Chip */
	init_waitqueue_head(&g_data_ready_wait_queue);
	return 0;
}

int fsa9480_get_jig_status(void)
{
#if defined(CONFIG_MACH_APACHE)
	u8 jig_devices = CRB_JIG_USB_ON | CRB_JIG_UART_ON | CRB_JIG_UART_OFF;
#else
	u8 jig_devices = CRB_JIG_USB_ON | CRB_JIG_USB_OFF | CRB_JIG_UART_ON | CRB_JIG_UART_OFF;
#endif

	if (fsa9480_device2 & jig_devices)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(fsa9480_get_jig_status);

int fsa9480_get_charger_status(void)
{
	if (curr_usb_status)
		return 2;	// (CHARGER_TYPE_USB_PC)
	else if (curr_ta_status)
		return 1;	// (CHARGER_TYPE_WALL)
	else
		return 0;	// no charger (CHARGER_TYPE_NONE)
}
EXPORT_SYMBOL(fsa9480_get_charger_status);

void fsa9480_connect_charger(void)	// vbus connected
{
	u8 dev1, dev2;

	fsa9480_i2c_read(REGISTER_DEVICETYPE1, &dev1);
	msleep(5);

	fsa9480_i2c_read(REGISTER_DEVICETYPE2, &dev2);

#if defined(CONFIG_MACH_APACHE)
	if((dev1 == CRA_USB)||(dev1==0 && dev2==CRB_JIG_USB_OFF))
#else
	if (dev1 == CRA_USB)
#endif
	{
		curr_usb_status = 1;

		usb_switch_state(); // re-enable HUB_EN if it should be on
			
		pr_info("[FSA9480] %s: USB connected...\n", __func__);
	}
	else	// consider as dedicated charger any device with vbus power
	{
		curr_ta_status = 1;
		pr_info("[FSA9480] %s: TA connected...\n", __func__);
	}

	batt_restart();
}
EXPORT_SYMBOL(fsa9480_connect_charger);

void fsa9480_disconnect_charger(void)	// vbus disconnected
{
	u8 dev1, dev2;

	fsa9480_i2c_read(REGISTER_DEVICETYPE1, &dev1);
	msleep(5);

	fsa9480_i2c_read(REGISTER_DEVICETYPE2, &dev2);

	curr_usb_status = 0;
	curr_ta_status = 0;

	pr_info("[FSA9480] %s\n", __func__);
	batt_restart();
}
EXPORT_SYMBOL(fsa9480_disconnect_charger);

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
static void fsa9480_read_interrupt_register(void);
extern void usb_force_reset(void);

static void usb_switch_mode(int mode)
{
	curr_usb_path = SWITCH_MSM;
	fsa9480_i2c_write(REGISTER_CONTROL, 0x1E);
#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	if(fsa9480_probe_done && curr_usb_status && !power_off_done) {
#else
	if(fsa9480_probe_done && curr_usb_status) {
#endif
		usb_force_reset();
		fsa9480_read_interrupt_register();
	}
}

static int get_current_mode(void)
{
	int mode = 0;
	
	mode = SWITCH_MSM;
	
	return mode;
}
#include <linux/vmalloc.h>
#include <linux/mm.h>

/* for sysfs control (/sys/class/sec/switch/usb_sel) */
static ssize_t usb_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int fd;
	char buffer[100]={0};
	int ret;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());


	if ((fd = sys_open("/persist/usb_sel.bin", O_RDONLY,0)) < 0){	

	printk("%s :: open failed %s ,fd=0x%x\n",__func__,"/persist/usb_sel.bin",fd);
	return;
	}

	ret = sys_read(fd,buffer,1);

	if(ret<0) {
		if(!power_down) {
			printk("usb_switch_show READ FAIL!\n");
		}
		return;
	}	

	sys_close(fd);
	set_fs(fs);

	if(!power_down)
		printk("[FSA9480] usb_sel.bin = %s, deb_dev1=0x%x, deb_dev2=0x%x, deb_intr1=0x%x, deb_intr2=0x%x, deb_intb=%d\n",buffer, deb_dev1, deb_dev2, deb_intr1,deb_intr2,deb_intb);


	if (!strcmp(buffer, "1")){
		sprintf(buf, "USB Switch : PDA_AP");
	}
	else if (!strcmp(buffer, "2")){
		sprintf(buf, "USB Switch : PDA_CP");
	}
	else if (!strcmp(buffer, "0")){
		sprintf(buf, "USB Switch : PDA_AP");
	}
		
	return sprintf(buf, "%s\n", buf);
}

static ssize_t usb_switch_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int fd;

	char buffer[1]={0};

	printk("usb_switch_store ENTRY %s!! \n",buf);

	mm_segment_t fs = get_fs();
	set_fs(get_ds());
	
	printk("usb_switch_store ENTRY !! \n");

	if ((fd = sys_open("/persist/usb_sel.bin", O_CREAT|O_WRONLY  ,0)) < 0){ 

		printk("%s :: open failed %s ,fd=0x%x\n",__func__,"/persist/usb_sel.bin",fd);
	}
	else{
		printk("%s :: open success %s ,fd=0x%x\n",__func__,"/persist/usb_sel.bin",fd);
	}

	if(strncmp(buf, "PDA_AP", 6) == 0 || strncmp(buf, "pda_ap", 6) == 0) {
		sprintf(buffer, "1");
		//samsung_enable_function( USBSTATUS_SAMSUNG_KIES);
		printk("usb_switch_store1 %s !! \n",buf);		
	}
    else
    {
		sprintf(buffer, "2");
		//samsung_enable_function( USBSTATUS_DM);
		printk("usb_switch_store2 %s !! \n",buf);		
    }
	sys_write(fd,buffer,strlen(buffer));

	sys_close(fd); 
	set_fs(fs);


}
static DEVICE_ATTR(usb_sel, 0664, usb_switch_show, usb_switch_store);

/* for sysfs control (/sys/class/sec/switch/disable_vbus) */
extern void msm_hsusb_set_vbus_state(int online);

static ssize_t disable_vbus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t disable_vbus_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	
	printk("disable_vbus_store ENTRY \n");
	disable_vbus_flag = 1;	
	msm_hsusb_set_vbus_state(0);


}
static DEVICE_ATTR(disable_vbus, 0664, disable_vbus_show, disable_vbus_store);
#endif


static void uart_switch_mode(int mode)
{
                curr_uart_path = SWITCH_MSM;
                fsa9480_select_mode(0x1E);        
                DEBUG_FSA9480("UART path changed - MSM");
}

void uart_switch_mode_select(int mode) {
	struct ariesve_parameter param_data;
	memset(&param_data,0,sizeof(struct ariesve_parameter));

	//msm_read_param(&param_data);

	uart_switch_mode(mode);
	param_data.uart_sel = mode;

	//msm_write_param(&param_data);
}
EXPORT_SYMBOL(uart_switch_mode_select);

static ssize_t DefaultESNStatus_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (g_default_ESN_status) {    
        return sprintf(buf, "DefaultESNStatus : TRUE\n");
    }
    else{
        return sprintf(buf, "DefaultESNStatus : FALSE\n");        
    }
}

static ssize_t DefaultESNStatus_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{		
    if ((strncmp(buf, "TRUE", 4) == 0) ||(strncmp(buf, "true", 4) == 0)) {
        g_default_ESN_status = 1;
    }
    if ((strncmp(buf, "FALSE", 5) == 0) ||(strncmp(buf, "false", 5) == 0)) {
        g_default_ESN_status = 0;
    }
}

static DEVICE_ATTR(DefaultESNStatus, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, DefaultESNStatus_switch_show, DefaultESNStatus_switch_store);

static ssize_t dock_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (g_dock == DOCK_REMOVED)
		return sprintf(buf, "0\n");
	else if (g_dock == DESK_DOCK_CONNECTED)
		return sprintf(buf, "1\n");
	else if (g_dock == CAR_DOCK_CONNECTED)
		return sprintf(buf, "2\n");
}

static ssize_t dock_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (strncmp(buf, "0", 1) == 0)
	{
		printk("remove dock\n");
		g_dock = DOCK_REMOVED;
	}
	else if (strncmp(buf, "1", 1) == 0)
	{
		printk("dsek dock inserted\n");
		g_dock = DESK_DOCK_CONNECTED;
	}
	else if (strncmp(buf, "2", 1) == 0)
	{
		printk("car dock inserted\n");
		g_dock = CAR_DOCK_CONNECTED;
	}

	return size;
}

static DEVICE_ATTR(dock, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, dock_switch_show, dock_switch_store);

void usb_switch_state(void)
{
                usb_switch_mode(SWITCH_MSM);
}

extern int set_tsp_for_ta_detect(int state);
static void fsa9480_process_device(u8 dev1, u8 dev2, u8 attach)
{
#if defined(CONFIG_MACH_APACHE) || defined(CONFIG_MACH_ANCORA_TMO)
	u8 is_uart_jig_on_charger = 0;
	u8 vbus_stat = 0;
#endif
	u8 vdev1 = dev1;
	u8 vdev2 = dev2;
#if defined(CONFIG_MACH_APACHE)
	if(vdev1==0 && vdev2==CRB_JIG_USB_OFF){
		vdev1 = CRA_USB;
		vdev2 = 0;
	}
#endif
	
	if (vdev1)
	{
		switch (vdev1)
		{		
			case CRA_AUDIO_TYPE1:
				if(attach & ATTACH){                
				    DEBUG_FSA9480("AUDIO_TYPE1(DESK_DOCK) --- ATTACH\n");
				    switch_set_state(&switch_dock_detection, (int)DESK_DOCK_CONNECTED);
				    g_dock = DESK_DOCK_CONNECTED;                   
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("AUDIO_TYPE1(DESK_DOCK) --- DETACH\n");
				    switch_set_state(&switch_dock_detection, (int)DOCK_REMOVED);
				    g_dock = DOCK_REMOVED;                    
				}                      
				break;
			case CRA_AUDIO_TYPE2:
				DEBUG_FSA9480("AUDIO_TYPE2 \n");                
				break;              
			case CRA_USB:                
				if(attach & ATTACH){
					DEBUG_FSA9480("USB --- ATTACH\n");
					if( disable_vbus_flag )
					{
						msm_hsusb_set_vbus_state(1);  // if MTP blocked before. disable_vbus_store
						disable_vbus_flag = 0;
					}
					if(!charging_boot)
					{
					DEBUG_FSA9480("USB ---!charging_boot && android_probe_done\n");					
						usb_switch_state();
					} 					
					curr_usb_status = 1;                    
					MicroUSBStatus=1;
					if((!askonstatus))
						UsbIndicator(1);
				}
				else if(attach & DETACH){
					DEBUG_FSA9480("USB --- DETACH\n");
					msleep(200);
					DEBUG_FSA9480("USB --- DETACH work around \n");					
					curr_usb_status = 0;
					MicroUSBStatus=0;
					UsbIndicator(0);
				}
                
				if (attach & (ATTACH|DETACH))
					batt_restart();                

				if (charging_boot && !curr_usb_status)
					pm_power_off();

				set_tsp_for_ta_detect(curr_usb_status);
#if defined(CONFIG_MACH_APACHE)
				if(fsa9480_pdata->charger_cb)
					fsa9480_pdata->charger_cb(curr_usb_status);
#endif
				break;
			case CRA_UART:
				DEBUG_FSA9480("UART \n");                
				if(attach & ATTACH){                
					curr_uart_status = 1;
				}
				else if(attach & DETACH){                
					curr_uart_status = 0;
				}                      

				break;
			case CRA_CARKIT:
				DEBUG_FSA9480("CARKIT \n");
//				printk("[ssam] attach value : 0x%x\n", attach);
				
				if(attach & ATTACH){
				    DEBUG_FSA9480("CARKIT_CHARGER --- ATTACH\n");
				    curr_ta_status = 1;                    
				}
				else if(attach & DETACH){
				    DEBUG_FSA9480("CARKIT_CHARGER --- DETACH\n");
				    curr_ta_status = 0;                    
				}

				if (attach & (ATTACH|DETACH))
					batt_restart();         

				if (charging_boot && !curr_ta_status)
					pm_power_off();

				set_tsp_for_ta_detect(curr_ta_status);
#if defined(CONFIG_MACH_APACHE)
				if(fsa9480_pdata->charger_cb)
					fsa9480_pdata->charger_cb(curr_ta_status);
#endif
				break;                
			case CRA_USB_CHARGER:
				DEBUG_FSA9480("USB CHARGER \n");
				break;
			case CRA_DEDICATED_CHG:
				if(attach & ATTACH){
				    DEBUG_FSA9480("DEDICATED_CHARGER --- ATTACH\n");
				    curr_ta_status = 1;                    
				}
				else if(attach & DETACH){
				    DEBUG_FSA9480("DEDICATED_CHARGER --- DETACH\n");
				    curr_ta_status = 0;                    
				}

				if (attach & (ATTACH|DETACH))
					batt_restart();         

				if (charging_boot && !curr_ta_status)
					pm_power_off();

				set_tsp_for_ta_detect(curr_ta_status);
#if defined(CONFIG_MACH_APACHE)
				 if(fsa9480_pdata->charger_cb)
				      fsa9480_pdata->charger_cb(curr_ta_status);
#endif
				break;                
			case CRA_USB_OTG:
				DEBUG_FSA9480("USB_OTG \n");                
				break;                	
			default:
				break;
		}
	}

	if (vdev2)
	{
		switch (vdev2)
		{
			case CRB_JIG_USB_ON:
				if(attach & ATTACH){                
				    DEBUG_FSA9480("JIG_USB_ON --- ATTACH\n");
				    usb_switch_state();  
                                    MicroJigUSBOnStatus=1;
                                    UsbIndicator(1);
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("JIG_USB_ON --- DETACH\n");
                                    MicroJigUSBOnStatus=0;
                                    UsbIndicator(0);
				}               
				break;
			case CRB_JIG_USB_OFF:
				if(attach & ATTACH){                
				    DEBUG_FSA9480("JIG_USB_OFF --- ATTACH\n");
				    usb_switch_state();                    
                                    MicroJigUSBOffStatus=1;
                                    UsbIndicator(1);
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("JIG_USB_OFF --- DETACH\n");                    
                                    MicroJigUSBOffStatus=0;
                                    UsbIndicator(0);
				}
				break;
			case CRB_JIG_UART_ON:
#if defined(CONFIG_MACH_APACHE) || defined(CONFIG_MACH_ANCORA_TMO)
				fsa9480_i2c_read(0x1D, &vbus_stat);
				if ( vbus_stat & 0x02 )
				{
					is_uart_jig_on_charger = 1;	
				}

				if(attach & ATTACH)
				{                
					MicroJigUARTOffStatus=1;
					if (is_uart_jig_on_charger)
					{
						curr_ta_status = 1; // FUEL GAUGE AUTO TEST					
						DEBUG_FSA9480("JIG_UART_ON --- ATTACH with  VBUS 5V INPUT! FORCE TA MODE!!! (reg = 0x%x)\n", vbus_stat);
					}
					else
					{
					    DEBUG_FSA9480("JIG_UART_ON --- ATTACH without  VBUS 5V INPUT! (reg = 0x%x\n");					
					}
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("JIG_UART_ON --- DETACH\n");
					MicroJigUARTOffStatus=0;
					curr_ta_status = 0; // FUEL GAUGE AUTO TEST					
				}  	
				
				curr_uart_status = 1;                   
				if ( ( attach & (ATTACH|DETACH) ) )// FUEL GAUGE AUTO TEST
					batt_restart();				
				break;				
#else				
				if(attach & ATTACH){                
				    DEBUG_FSA9480("JIG_UART_ON --- ATTACH\n");
                                    MicroJigUARTOffStatus=1;
					curr_ta_status = 1; // FUEL GAUGE AUTO TEST
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("JIG_UART_ON --- DETACH\n");
                                    MicroJigUARTOffStatus=0;
					curr_ta_status = 0; // FUEL GAUGE AUTO TEST					
				}  	
				curr_uart_status = 1;                   
				if (attach & (ATTACH|DETACH))// FUEL GAUGE AUTO TEST
					batt_restart();				
				break;
#endif
			case CRB_JIG_UART_OFF:
				if(attach & ATTACH){                
				    DEBUG_FSA9480("JIG_UART_OFF --- ATTACH\n");
                                    MicroJigUARTOffStatus=1;
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("JIG_UART_OFF --- DETACH\n");
                                    MicroJigUARTOffStatus=0;
				}  				
				curr_uart_status = 0;
				DEBUG_FSA9480("JIG_UART_OFF \n");                
				break;                
			case CRB_PPD:
				DEBUG_FSA9480("PPD \n");
				break;
			case CRB_TTY:
				DEBUG_FSA9480("TTY \n");                
				break;                
			case CRB_AV:
				DEBUG_FSA9480("AUDIO/VIDEO \n");
				if(attach & ATTACH){                
				    DEBUG_FSA9480("AUDIO/VIDEO(CAR_DOCK) --- ATTACH\n");
                                         if(!g_default_ESN_status){
				        switch_set_state(&switch_dock_detection, (int)CAR_DOCK_CONNECTED);
				        g_dock = CAR_DOCK_CONNECTED;
                                         }
				}
				else if(attach & DETACH){                
				    DEBUG_FSA9480("AUDIO/VIDEO(CAR_DOCK) --- DETACH\n");
				    switch_set_state(&switch_dock_detection, (int)DOCK_REMOVED);
				    g_dock = DOCK_REMOVED;                    
				}                 
				break;                		
			default:
				break;
		}
	}

	if ( curr_uart_status && ( attach & DETACH ) ) {
		curr_uart_status = 0;
		DEBUG_FSA9480("UART_OFF \n");                
	}

         if( attach & (KEY_PRESS|LONG_KEY_PRESS|LONG_KEY_RELEASE) )
         {
		u8 button2;
		bool is_press = 0;
		bool is_longkey = 0;

		/* read button register */ 
		fsa9480_i2c_read(REGISTER_BUTTON2, &button2);        

		if( attach & KEY_PRESS ) 
		{
			is_longkey = 0;

			DEBUG_FSA9480("KEY PRESS\n");
		}
		else
		{
			is_longkey = 1;

			if( attach & LONG_KEY_PRESS )
			{
				is_press = 1;

				DEBUG_FSA9480("LONG KEY PRESS\n");
			}
			else
			{
				is_press = 0;

				DEBUG_FSA9480("LONG KEY RELEASE\n");
			}
		}

    	        if( g_dock == DESK_DOCK_CONNECTED )
    	        {
		        u8 key_mask;
		        int index = 0;
			
		        key_mask = (button2 & DOCK_KEY_MASK) >> DOCK_KEY_SHIFT;

		        DEBUG_FSA9480("key_mask 0x%x \n", key_mask);

    		        while(key_mask) 
    		        {
    			    if( key_mask & 0x1 )
			    {
				if( is_longkey )
				{
    				    dock_keys_input(index, is_press);
				}
				else
				{
				    dock_keys_input(index, 1);
				    dock_keys_input(index, 0);
				}
			    }

    			    key_mask >>= 1;
    			    index++;
    		        }
    	        }    
         }
    
}

static void fsa9480_read_interrupt_register(void)
{
	int i, intb_val;
	u8 dev1, dev2, intr1, intr2;

	fsa9480_i2c_read(REGISTER_INTERRUPT1, &intr1);
	deb_intr1 = intr1;
	msleep(5);

	fsa9480_i2c_read(REGISTER_INTERRUPT2, &intr2);
	deb_intr2 = intr2;
	msleep(5);     
	
	intb_val = gpio_get_value(142);
	deb_intb = intb_val;
	msleep(5);     
	// Check interrupt fired.
	if(((int)intr1 <= 0)||(intr1 == 3)||(intr1 == 7)||(intb_val == 0))
	{
		for(i=0; i<3; i++)
		{			
			printk("[FSA9480] Recheck intr1. intr1=0x%x,intb=%d\n",intr1,intb_val);
			fsa9480_i2c_read(REGISTER_INTERRUPT1, &intr1);
			deb_intr1 = intr1;	
			msleep(5);
			intb_val = gpio_get_value(142);	
			deb_intb = intb_val;			
			msleep(5);
			if((intr1 > 0)&&(intr1 != 3)&&(intr1 != 7)&&(intb_val == 1))
				break;
		}
		if(((int)intr1 <= 0)||(intr1 == 3)||(intr1 == 7)||(intb_val == 0))
		{			
			printk("[FSA9480] %s : interrupt was fired. intr1= 0x%x,intb=%d\n", __func__, intr1,intb_val);
			fsa9480_chip_init();
			fsa9480_i2c_read(REGISTER_INTERRUPT1, &intr1);
		}
	}


	fsa9480_i2c_read(REGISTER_DEVICETYPE1, &dev1);
	deb_dev1 = dev1;
	msleep(5);

	fsa9480_i2c_read(REGISTER_DEVICETYPE2, &dev2);
	deb_dev2 = dev2;

	printk("[FSA9480] dev1=0x%x, dev2=0x%x, intr1=0x%x, intr2=0x%x\n",dev1,dev2,intr1,intr2);
	
#if defined(CONFIG_USA_FEATURE)//++ jwjang@tecace : check chip reset 
/* wrong recognition of cable remove to key press  */
/* because of surge when unplug cable */
/* this code is for forced unplug */
	if ( intr1 == 0xFF && dev1 == 0x00 && dev2 == 0x00 )
	{
		printk("[FSA9480] FSA9420 was reset!!!\n");
		fsa9480_i2c_write(REGISTER_CONTROL, 0x1E);
		intr1 = 0x2;
	}
#endif	//-- jwjang@tecace : check chip reset 
	usb_state = (dev2 << 8) | (dev1 << 0);

#if defined(CONFIG_MACH_APACHE)
	if ( !fsa9480_probe_done && 
		(( dev1 == CRA_USB)||(dev1==0 && dev2 == CRB_JIG_USB_OFF))) {
#else
	if ( !fsa9480_probe_done && ( dev1 == CRA_USB)) {
#endif
		printk("[FSA9480] target booting with USB cable attached\n");
		intr1 = intr1 | ATTACH;
	}

	/* Device attached */
	if (intr1 & ATTACH)
	{
		fsa9480_device1 = dev1;
		fsa9480_device2 = dev2;
	}
	
	if(!dev1 && !dev2 && !intr1 && !intr2)
	{
		/* Force update USB status when reading intr failed. */
		printk("[FSA9480] Force update!\n");
		printk("[FSA9480] dev1=0x%x, dev2=0x%x, intr1=0x%x, intr2=0x%x\n",dev1,dev2,intr1,intr2);

		// USB
		if(curr_usb_status == 1)
		{
		    curr_usb_status = 0;
			MicroUSBStatus=0;
			
			UsbIndicator(0);
			batt_restart(); 			   
			
			if (charging_boot && !curr_usb_status)
				pm_power_off();

			set_tsp_for_ta_detect(curr_usb_status);
		}

		// TA
		if(curr_ta_status == 1)
		{
			curr_ta_status =0;
			batt_restart();         

			if (charging_boot && !curr_ta_status)
				pm_power_off();

			set_tsp_for_ta_detect(curr_ta_status);
		}
	}

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	fsa9480_process_device(fsa9480_device1, fsa9480_device2, intr1);
#else
	if (fsa9480_device1)
	{
		switch (fsa9480_device1)
		{
			case CRA_USB:
                
#ifdef CONFIG_USB_GADGET_WESTBRIDGE
    //Vova: Detection of USB attach/detach for WestBridge chip
     //TBD - probaby need to be moved to gadget controller

				if (intr1 & ATTACH)
					curr_usb_status = 1;
				else if (intr1 & DETACH)
					curr_usb_status = 0;
				if (intr1 & (ATTACH|DETACH))
					batt_restart();			
				break;
			case CRA_DEDICATED_CHG:
				if (intr1 & ATTACH)
					curr_ta_status = 1;
				else if (intr1 & DETACH)
					curr_ta_status = 0;
				if (intr1 & (ATTACH|DETACH))
					batt_restart();
#endif//Vova: End of detection block:
				break;
			default:
				break;
		}
	}

	if (fsa9480_device2)
	{
		switch (fsa9480_device2)
		{
			default:
				break;
		}
	}	 
#endif

	/* Device detached */
	if ( (intr1 & DETACH) && !(intr1 & ATTACH) ) // workaround for intr1 = 0x03(ATTACH|DETACH)
	{
	 fsa9480_device1 = 0x0;
	 fsa9480_device2 = 0x0;
	}
}

static irqreturn_t fsa9480_interrupt_handler(int irq, void *data)
{
	printk("[FSA9480]: interrupt called\n");

#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)
	if(!power_off_done)
#endif
	{
		fsa9480_read_interrupt_register();
	}

	return IRQ_HANDLED;
}

static int fsa9480_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fsa9480_data *mt;
#if defined(CONFIG_MACH_ARIESVE) || defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_GODART) || defined(CONFIG_MACH_ANCORA_TMO) || defined(CONFIG_MACH_APACHE)	
	int i;
#endif
	int err = -1;
	printk("[FSA9480] fsa9480_probe \n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		goto exit_check_functionality_failed;		
	
	if(!(mt = kzalloc( sizeof(struct fsa9480_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, mt);
	fsa9480_client(client);
	pclient = client;
#if defined(CONFIG_MACH_APACHE)
	fsa9480_pdata=client->dev.platform_data;
#endif
	fsa9480_chip_init();

	if(request_threaded_irq(pclient->irq, NULL, fsa9480_interrupt_handler, IRQF_ONESHOT |IRQF_TRIGGER_FALLING, "MICROUSB", pclient)) {
		free_irq(pclient->irq, NULL);
		printk("[FSA9480] fsa9480_interrupt_handler can't register the handler! and passing....\n");
	}

	enable_irq_wake(pclient->irq);
	
	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		printk(KERN_ERR "[FSA9480]: Failed to create device file(%s)!\n", dev_attr_usb_sel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_disable_vbus) < 0)
		printk(KERN_ERR "[FSA9480]: Failed to create device file(%s)!\n", dev_attr_usb_sel.attr.name);


	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		printk(KERN_ERR "[FSA9480]Failed to create device file(%s)!\n", 		
					dev_attr_usb_state.attr.name);

    indicator_dev.name = DRIVER_NAME;
	indicator_dev.print_name = print_indicatorswitch_name;
	indicator_dev.print_state = print_indicatorswitch_state;
	switch_dev_register(&indicator_dev);

	dock_key_input_dev = input_allocate_device();
	if( !dock_key_input_dev )
		return -ENOMEM;

	dock_key_input_dev->name = switch_dock_detection.name;
	set_bit(EV_SYN, dock_key_input_dev->evbit);
	set_bit(EV_KEY, dock_key_input_dev->evbit);
	
	for(i=0; i < DOCK_KEY_MAX; i++) 
	{
		set_bit(dock_keys[i], dock_key_input_dev->keybit);
	}
    
	err = input_register_device(dock_key_input_dev);
	if( err ) {
		input_free_device(dock_key_input_dev);
		return err;
	}

	 switch_dev_register(&switch_dock_detection);

	 usb_switch_mode(SWITCH_MSM);
	 
	fsa9480_probe_done = 1;


	printk("[FSA9480]: probe complete\n");
	return 0;
	
exit_alloc_data_failed:
exit_check_functionality_failed:
	
	return err;
}

	
static int fsa9480_remove(struct i2c_client *client)
{
	struct fsa9480_data *mt;

	mt = i2c_get_clientdata(client);
	free_irq(client->irq, mt);
	pclient = NULL;
	kfree(mt);
	return 0;
}


static const struct i2c_device_id fsa9480_id[] = {
	{ "fsa9480", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static int fsa9480_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int fsa9480_resume(struct i2c_client *client)
{
	if ( system_rev >= 12) {
		if ( curr_usb_status )
			usb_switch_state(); // re-enable HUB_EN if it should be on
	}
	return 0;
}

static struct i2c_driver fsa9480_driver = {
	.probe 		= fsa9480_probe,
	.remove 	= fsa9480_remove,
	.suspend	= fsa9480_suspend,
	.resume	= fsa9480_resume,
	.shutdown = fsa9480_remove,
	.id_table	= fsa9480_id,
	.driver = {		
		.name   = "fsa9480",
	},
};

static int __init fsa9480_init(void)
{
	printk("[FSA9480] fsa9480_init \n");

	return i2c_add_driver(&fsa9480_driver);
}

static void __exit fsa9480_exit(void)
{
	i2c_del_driver(&fsa9480_driver);
}


EXPORT_SYMBOL(fsa9480_i2c_read);
EXPORT_SYMBOL(fsa9480_i2c_write);

module_init(fsa9480_init);
module_exit(fsa9480_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("fsa9480 Driver");
MODULE_LICENSE("GPL");

