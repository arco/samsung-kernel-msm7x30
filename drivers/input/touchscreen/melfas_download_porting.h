//--------------------------------------------------------
//
//    MELFAS Firmware download base code for MCS6000
//    Version : v01
//    Date    : 2009.01.20
//
//--------------------------------------------------------

#ifndef __MELFAS_DOWNLOAD_PORTING_H_INCLUDED__
#define __MELFAS_DOWNLOAD_PORTING_H_INCLUDED__

//============================================================
//
//    Porting order
//
//============================================================
/*

1. melfas_download_porting.h
   1. Check typedef
   2. Check download options
   3. Port control setting  ( CE, I2C,... )
   4. Delay function pharameter constants ( with mcsdl_delay() )
   5. Check Watchdog timer, Interrupt factor

2. melfas_download.c
   1. Including Melfas binary .c file
   2. Baseband I2C functions.
      fill up _i2c_read() and _i2c_write()
   3. Basenad dealy function
      fill up mcsdl_delay()
   4. Implement Melfas binary .bin file reading

*/

//============================================================
//
//    Porting section 0. Type define
//
//============================================================

typedef char				INT8;
typedef unsigned char		UINT8;
//typedef unsigned char		uint8_t;
typedef short				INT16;
typedef unsigned short		UINT16;
//typedef unsigned short	uint16_t;
typedef int					INT32;
typedef unsigned int		UINT32;
typedef unsigned char		BOOLEAN;


#ifndef TRUE
#define TRUE				(1==1)
#endif

#ifndef FALSE
#define FALSE				(1==0)
#endif

#ifndef NULL
#define NULL				0
#endif

#ifndef GPIO_TOUCH_I2C_SDA
#define GPIO_TOUCH_I2C_SDA GPIO_I2C0_SDA
#define GPIO_TOUCH_I2C_SCL GPIO_I2C0_SCL
#endif



//============================================================
//
//    Porting section 1. Download Options
//
//============================================================

// Disable downlaoding, if module version does not match.
#define MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_DOES_NOT_MATCH        0

// For printing debug information. ( Please check 'printing function' )
#define MELFAS_ENABLE_DBG_PRINT                                            1
#define MELFAS_ENABLE_DBG_PROGRESS_PRINT                                1

// For delay function test. ( Disable after Porting is finished )
#define MELFAS_ENABLE_DELAY_TEST                                        1


//============================================================
//
//    Porting section 2. IO Control poting.
//
//    Fill 'Using signal' up only.
//     See MCSDL_USE_VDD_CONTROL,
//        MCSDL_USE_CE_CONTROL,
//        MCSDL_USE_INTR_CONTROL,
//        MCSDL_USE_RESETB_CONTROL is '1' or '0'
//============================================================

//----------------
// VDD
//----------------
#if MCSDL_USE_VDD_CONTROL
#define TKEY_VDD_SET_HIGH()             				mcsdl_vdd_on()
#define TKEY_VDD_SET_LOW()              				mcsdl_vdd_off()
#else
#define TKEY_VDD_SET_HIGH()			// Nothing
#define TKEY_VDD_SET_LOW()			// Nothing
#endif

//----------------
// CE
//----------------
#if MCSDL_USE_CE_CONTROL
#define TKEY_CE_SET_HIGH()   	          			 gpio_set_value(TOUCH_EN, 1)
#define TKEY_CE_SET_LOW()      	        			 gpio_set_value(TOUCH_EN, 0)
#define TKEY_CE_SET_OUTPUT()   	        			 gpio_tlmm_config(GPIO_CFG(TOUCH_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE)
#else
#define TKEY_CE_SET_HIGH()			// Nothing
#define TKEY_CE_SET_LOW()			// Nothing
#define TKEY_CE_SET_OUTPUT()		// Nothing
#endif

//----------------
// INTR
//----------------
#if MCSDL_USE_INTR_CONTROL
#define TKEY_INTR_SET_HIGH()             			gpio_set_value(GPIO_TOUCH_INT, 1)
#define TKEY_INTR_SET_LOW()              			gpio_set_value(GPIO_TOUCH_INT, 0)
#define TKEY_INTR_SET_OUTPUT()             			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,GPIO_CFG_16MA), GPIO_CFG_ENABLE)
#define TKEY_INTR_SET_INPUT()             			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,GPIO_CFG_2MA), GPIO_CFG_ENABLE)
#else
#define TKEY_INTR_SET_HIGH()		// Nothing
#define TKEY_INTR_SET_LOW()			// Nothing
#define TKEY_TINR_SET_OUTPUT()		// Nothing
#define TKEY_INTR_SET_INPUT()		// Nothing
#endif

//----------------
// RESETB
//----------------
#if MCSDL_USE_RESETB_CONTROL
#define TKEY_RESETB_SET_HIGH()		____HERE!_____
#define TKEY_RESETB_SET_LOW()		____HERE!_____
#define TKEY_RESETB_SET_OUTPUT()	____HERE!_____
#define TKEY_RESETB_SET_INPUT()		____HERE!_____
#else
#define TKEY_RESETB_SET_HIGH()		// Nothing
#define TKEY_RESETB_SET_LOW()		// Nothing
#define TKEY_RESETB_SET_OUTPUT()	// Nothing
#define TKEY_RESETB_SET_INPUT()		// Nothing
#endif


//------------------
// I2C SCL & SDA
//------------------

#define TKEY_I2C_SCL_SET_HIGH()		gpio_set_value(GPIO_TOUCH_I2C_SCL, 1)
#define TKEY_I2C_SCL_SET_LOW()		gpio_set_value(GPIO_TOUCH_I2C_SCL, 0)

#define TKEY_I2C_SDA_SET_HIGH()		gpio_set_value(GPIO_TOUCH_I2C_SDA, 1)
#define TKEY_I2C_SDA_SET_LOW()		gpio_set_value(GPIO_TOUCH_I2C_SDA, 0)

#define TKEY_I2C_SCL_SET_OUTPUT()					gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE)
#define TKEY_I2C_SDA_SET_OUTPUT()					gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE)

#define TKEY_I2C_SET_HIGH()			TKEY_I2C_SCL_SET_HIGH();	\
									TKEY_I2C_SDA_SET_HIGH()

#define TKEY_I2C_SET_LOW()			TKEY_I2C_SCL_SET_LOW();	\
									TKEY_I2C_SDA_SET_LOW()


#define TKEY_I2C_SET_OUTPUT()		TKEY_I2C_SCL_SET_OUTPUT();	\
									TKEY_I2C_SDA_SET_OUTPUT()

#define TKEY_I2C_INIT()				TKEY_I2C_SET_HIGH();	\
									TKEY_I2C_SET_OUTPUT()

#define TKEY_I2C_CLOSE()			TKEY_I2C_SET_LOW();		\
									TKEY_I2C_SET_OUTPUT()



//============================================================
//
//    Porting section 3. Delay parameter setting
//
//    These are used on 'mcsdl_delay()'
//
//============================================================

#define MCSDL_DELAY_15US			15
#define MCSDL_DELAY_100US			100
#define MCSDL_DELAY_150US			150
#define MCSDL_DELAY_500US			500
#define MCSDL_DELAY_1MS				1000
#define MCSDL_DELAY_25MS			25000
#define MCSDL_DELAY_45MS			45000


//============================================================
//
//    Porting section 4. Defence External Effect
//
//============================================================
#if 0

#define MELFAS_DISABLE_BASEBAND_ISR()				____HERE!_____
#define MELFAS_DISABLE_WATCHDOG_TIMER_RESET()		____HERE!_____

#define MELFAS_ROLLBACK_BASEBAND_ISR()				____HERE!_____
#define MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET()		____HERE!_____

#else

#define MELFAS_DISABLE_BASEBAND_ISR()				//Nothing
#define MELFAS_DISABLE_WATCHDOG_TIMER_RESET()		//Nothing

#define MELFAS_ROLLBACK_BASEBAND_ISR()				//Nothing
#define MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET()		//Nothing

#endif



#endif

