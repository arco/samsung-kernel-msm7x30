/*
* ==============================================================================
*  Name          : fsa9480.h
*  Part of         : MicroUsbDetector Driver
*  Description :  Definitions of FSA9480
* ==============================================================================
*/

#define FSA9480_DBG_ENABLE	1

#ifdef  FSA9480_DBG_ENABLE
#define DEBUG_FSA9480(fmt,args...) printk("[FSA9480]" fmt, ##args)
#else
#define DEBUG_FSA9480(fmt,args...) do {} while(0)
#endif

void fsa9480_change_path_to_audio(u8 enable);

#if defined(CONFIG_MACH_APACHE)
struct fsa9480_platform_data {
	void (*charger_cb) (bool attached);
};
#endif
/*
  * FSA9480 Register definition                                                                           
*/
/* DEVICE ID Register*/
#define REGISTER_DEVICEID			0x01     
/* CONTROL Register*/
#define REGISTER_CONTROL			0x02   
#define REGISTER_INTERRUPT1		0x03  
#define REGISTER_INTERRUPT2		0x04   
#define REGISTER_INTERRUPTMASK1	0x05
#define REGISTER_INTERRUPTMASK2	0x06
#define REGISTER_ADC				0x07
#define REGISTER_TIMINGSET1		0x08
#define REGISTER_TIMINGSET2		0x09
#define REGISTER_DEVICETYPE1		0x0A
#define REGISTER_DEVICETYPE2		0x0B
#define REGISTER_BUTTON1			0x0C
#define REGISTER_BUTTON2			0x0D
#define REGISTER_CARKITSTATUS		0x0E
#define REGISTER_CARKITINT1		0x0F
#define REGISTER_CARKITINT2		0x10
#define REGISTER_CARKITMASK1		0x11
#define REGISTER_CARKITMASK2		0x12
/* Manual SW1 Register*/
#define REGISTER_MANUALSW1		0x13                       
/* Manual SW2 Register */
#define REGISTER_MANUALSW2		0x14                   
/* Hidden Register*/
#define HIDDEN_REGISTER_MANUAL_OVERRDES1	0x1B     

//CR2 : Control Register
#define INT_MASK				(0x1 << 0)
#define SW_WAIT		 			(0x1 << 1)
#define MANUAL_SW 				(0x1 << 2)
#define RAW_DATA				(0x1 << 3)
#define SW_OPEN					(0x1 << 4)

//CR3 : Interrupt 1 Register
#define ATTACH					(0x1 << 0)
#define DETACH					(0x1 << 1)
#define KEY_PRESS				(0x1 << 2)
#define LONG_KEY_PRESS			(0x1 << 3)
#define LONG_KEY_RELEASE		(0x1 << 4)
#define OVP_EN					(0x1 << 5)
#define OCP_EN					(0x1 << 6)
#define OVP_OCP_DIS				(0x1 << 7)

//CR5 : Interrupt 1 Mask Register
#define ATTACH_INT_MASK			(0x1 <<0)
#define DETACH_INT_MASK 			(0x1 <<1)
#define KEY_PRESS_INT_MASK			(0x1 <<2)
#define LONGKEY_PRESS_INT_MASK 	(0x1 <<3)
#define LONGKEY_RELEASE_INT_MASK	(0x1 <<4)
#define OVP_INT_MASK 				(0x1 <<5)
#define OCP_INT_MASK 				(0x1 <<6)
#define OVP_OCP_DIS_INT_MASK 		(0x1 <<7)

//CR7 : ADC Register
#define USB_OTG							0x00
#define SEND_END						0x01
#define AUDIO_REMOTE_S1_BUTTON		0x02
#define AUDIO_REMOTE_S2_BUTTON		0x03
#define AUDIO_REMOTE_S3_BUTTON		0x04
#define AUDIO_REMOTE_S4_BUTTON		0x05
#define AUDIO_REMOTE_S5_BUTTON		0x06
#define AUDIO_REMOTE_S6_BUTTON		0x07
#define AUDIO_REMOTE_S7_BUTTON		0x08
#define AUDIO_REMOTE_S8_BUTTON		0x09
#define AUDIO_REMOTE_S9_BUTTON		0x0A
#define AUDIO_REMOTE_S10_BUTTON		0x0B
#define AUDIO_REMOTE_S11_BUTTON		0x0C
#define AUDIO_REMOTE_S12_BUTTON		0x0D
#define RESERVED_ACCESSORY_1			0x0E
#define RESERVED_ACCESSORY_2			0x0F
#define RESERVED_ACCESSORY_3			0x10
#define RESERVED_ACCESSORY_4			0x11
#define RESERVED_ACCESSORY_5			0x12
#define AUDIO_DEICE_TYPE_2				0x13
#define PHONE_POWERED_DEVICE			0X14
#define TTY_CONVERTER					0x15
#define UART_CABLE						0x16
#define CEA936A_TYPE_1_CHARGER		0x17
#define FACTORY_MODE_BOOT_OFF_USB	0x18
#define FACTORY_MODE_BOOT_ON_USB		0x19
#define AUDIO_VEDIO_CABLE				0x1A
#define CEA936A_TYPE_2_CHARGER		0x1B
#define FACTORY_MODE_BOOT_OFF_UART	0x1C
#define FACTORY_MODE_BOOT_ON_UART	0x1D
#define AUDIO_DEVICE_TYPE_1			0x1E
#define USB_OR_ACCESSORY_DETACH		0x1F

//CR8 : Timing Set 1 Register
#define DEVICE_WAKE_UP_TIME_MASK		0x0F
#define KEY_PRESS_TIME_MASK			0xF0

#define KEY_PRESS_TIME_300MS			0x20
#define KEY_PRESS_TIME_700MS			0x60
#define KEY_PRESS_TIME_1S				0x90

//CR9 : Timing Set 2 Register
#define LONGKEY_PRESS_TIME_MASK		0x0F
#define SWITCHING_TIME_MASK			0xF0

#define LONGKEY_PRESS_TIME_1S			0x07
#define LONGKEY_PRESS_TIME_1_5S		0x0C

//CRA : Device Type 1 Register
#define CRA_AUDIO_TYPE1             (0x1 <<0)
#define CRA_AUDIO_TYPE2             (0x1 <<1)
#define CRA_USB					(0x1 <<2)
#define CRA_UART				(0x1 <<3)
#define CRA_CARKIT				(0x1 <<4)

#define CRA_USB_CHARGER		(0x1 <<5)
#define CRA_DEDICATED_CHG		(0x1 <<6)

#define CRA_USB_OTG			(0x1 <<7)

//CRB : Device Type 2 Register
#define CRB_JIG_USB_ON		(0x1 <<0)
#define CRB_JIG_USB_OFF		(0x1 <<1)
#define CRB_JIG_UART_ON	(0x1 <<2)
#define CRB_JIG_UART_OFF	(0x1 <<3)
#define CRB_PPD				(0x1 <<4)
#define CRB_TTY				(0x1 <<5)
#define CRB_AV				(0x1 <<6)
//Factory mode cable detected
#define CRB_JIG_USB			(0x3 <<0)
#define CRB_JIG_UART		(0x3 <<2)

//Device1, 2 Register's Device Not Connected value
#define DEVICE_TYPE_NC		0x00

//CRC : Button 1 Register
#define BUTTON_SEND_END			(0x1 <<0)
#define BUTTON_1	 				(0x1 <<1)
#define BUTTON_2 					(0x1 <<2)
#define BUTTON_3 					(0x1 <<3)
#define BUTTON_4 					(0x1 <<4)
#define BUTTON_5 					(0x1 <<5)
#define BUTTON_6 					(0x1 <<6)
#define BUTTON_7 					(0x1 <<7)

//CRD : Button 2 Register
#define BUTTON_8		 			(0x1 <<0)
#define BUTTON_9					(0x1 <<1)
#define BUTTON_10					(0x1 <<2)
#define BUTTON_11					(0x1 <<3)
#define BUTTON_12					(0x1 <<4)
#define BUTTON_ERROR				(0x1 <<5)
#define BUTTON_UNKNOW				(0x1 <<6)

#if 1 //20101125_inchul
typedef enum
{
	DOCK_REMOVED			         = 0x0,
	DESK_DOCK_CONNECTED		= 0x01 << 0,	
	CAR_DOCK_CONNECTED		= 0x01 << 1,
}USB_DOCK_TYPE;
#endif

