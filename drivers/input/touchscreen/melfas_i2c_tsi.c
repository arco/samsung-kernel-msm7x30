/* drivers/input/keyboard/melfas_i2c_mth_st909.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/miscdevice.h>
#include <mach/vreg.h>
#include "melfas_i2c_tsi.h"

#define INPUT_TOUCH_DEBUG

#ifdef INPUT_TOUCH_DEBUG 
#define DPRINTK(X...) do { printk("%s(): ", __FUNCTION__); printk(X); } while(0)
#else
#define DPRINTK(x...)        /* !!!! */
#endif

static struct vreg *vreg_touch;	
static struct workqueue_struct *melfas_wq;

struct melfas_ts_pdev {
	unsigned short force[3];
	unsigned short *forces[2];
	struct i2c_client_address_data addr_data;
	int irq;
};

struct melfas_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

//hojun_kim static int melfas_ts_download_firmware(struct i2c_client *client);
static struct melfas_ts_data *ts;

static int ts_irq_num;

#define TOUCH_HOME  KEY_HOME
#define TOUCH_MENU  KEY_MENU
#define TOUCH_BACK  KEY_BACK
#define TOUCH_SEARCH  KEY_SEARCH

int melfas_ts_tk_keycode[] =

{ TOUCH_HOME, TOUCH_MENU, TOUCH_BACK, TOUCH_SEARCH, };


//hojun_kim extern int bridge_on;
extern const uint8_t MELFAS_binary[]; //hojun_kim
extern const uint16_t MELFAS_binary_nLength; //hojun_kim


#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif


static int tsp_i2c_read(struct i2c_client* client, u8 reg, unsigned char *rbuf, int buf_size);

#define CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE //hojun_kim
#undef USE_BASEBAND_I2C_FUNCTION//hojun_kim

void mcsdl_vdd_on(void)
{ 
  vreg_set_level(vreg_touch, OUT2800mV);
  vreg_enable(vreg_touch);
}

void mcsdl_vdd_off(void)
{
  vreg_disable(vreg_touch);
}

#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
static int melfas_ts_enter_download_mode(void);
static int melfas_ts_i2c_erase_flash(void);
static int melfas_ts_i2c_read_flash(uint8_t *pBuffer, uint16_t nAddr_start, uint8_t cLength);
static int mcsdl_i2c_program_info(void);
static int melfas_ts_i2c_program_flash( uint8_t *pData, uint16_t nAddr_start, uint8_t cLength );

static void melfas_ts_hw_i2c_mode(int on) // 0: OFF, 1: ON
{
  if(on)
  {
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
  }
  else
  {
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
    gpio_set_value(TOUCH_I2C_SCL_F, 0);
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
    gpio_set_value(TOUCH_I2C_SDA_F, 0);
  }
}

static void melfas_ts_int_mode(int on)  // 0: OFF, 1: ON
{
  if(on)
  {
    gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_OUTPUT/*hojun_kim GPIO_INPUT*/, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);		
  }
  else
  {
    gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);		
    gpio_set_value(TOUCH_INT, 0);
  }
}

//============================================================
//
//	Porting section 6.	I2C function calling
//
//	Connect baseband i2c function
//
//	Warning 1. !!!!  Burst mode is not supported. Transfer 1 byte Only.
//
//    	Every i2c packet has to
//            " START > Slave address > One byte > STOP " at download mode.
//
//	Warning 2. !!!!  Check return value of i2c function.
//
//    	_i2c_read_(), _i2c_write_() must return
//        	TRUE (1) if success,
//        	FALSE(0) if failed.
//
//    	If baseband i2c function returns different value, convert return value.
//        	ex> baseband_return = baseband_i2c_read( slave_addr, pData, cLength );
//            	return ( baseband_return == BASEBAND_RETURN_VALUE_SUCCESS );
//
//
//	Warning 3. !!!!  Check Slave address
//
//    	Slave address is '0x7F' at download mode. ( Diffrent with Normal touch working mode )
//        '0x7F' is original address,
//        	If shift << 1 bit, It becomes '0xFE'
//
//============================================================
static void i2c_write_byte(uint8_t cData)
{
  int i;
  
  for(i=7; i>=0; i--)
  {
    if( (cData>>i) & 0x01){
	    gpio_set_value(TOUCH_I2C_SDA_F, 1);
    }else{
	    gpio_set_value(TOUCH_I2C_SDA_F, 0);
    }

    udelay(1);

    gpio_set_value(TOUCH_I2C_SCL_F, 1); 
    udelay(1);
    gpio_set_value(TOUCH_I2C_SCL_F, 0); 
    udelay(1);
  }
}

static void i2c_read_byte(uint8_t *pData)
{
  int i;
  
  *pData  = 0;
  for(i=7; i>=0; i--)
  {  
  	gpio_set_value(TOUCH_I2C_SCL_F, 1);
  
  	if( gpio_get_value(TOUCH_I2C_SDA_F) ){
      *pData |= 0x1<<i;
    }

  	gpio_set_value(TOUCH_I2C_SCL_F, 0);
  }
}

#if 0//hojun_kim
static void melfas_i2c_start(void)
{
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

    gpio_set_value(TOUCH_I2C_SDA_F, 0); 
    udelay(15);
    gpio_set_value(TOUCH_I2C_SCL_F, 0); 
    udelay(15);
}

static void melfas_i2c_close(void)
{
//hojun_kim    int nCount_limit=0;

    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

    gpio_set_value(TOUCH_I2C_SDA_F, 0); 
    gpio_set_value(TOUCH_I2C_SCL_F, 0); 

    udelay(15);

    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

    while( gpio_get_value(TOUCH_I2C_SCL_F) == 0 )
    {
        ; 
    }
}

static int melfas_i2c_read_burst( uint8_t slave_addr, uint8_t * pData, uint8_t nCount )
{
    int i;

    melfas_i2c_start();

    slave_addr |= 1;

    i2c_write_byte( slave_addr );
//hojun_kim removed something
    for( i=0; i<nCount; i++ )
    {
        i2c_read_byte(pData++);

        i2c_set_ack( i != (nCount - 1) );
    }

    ret = 1;

    melfas_i2c_close();

    return 1;
}

static int melfas_i2c_write_burst( uint8_t slave_addr, uint8_t * pData, uint8_t nCount )
{
    int i;

    melfas_i2c_start();

    i2c_write_byte( slave_addr );
    //hojun_kim removed something

    udelay(15);

    for( i=0; i<nCount; i++ )
    {
        ret = i2c_read_byte( *pData++ );

        udelay(15);

        if( ret == 0 && i != (nCount-1) )
        {
            melfas_i2c_close();

            return 0;
        }
    }

    melfas_i2c_close();

    return 1;
}
#endif

static int _i2c_read_( uint8_t slave_addr, uint8_t *pData)
{
  int bRet = 0;

  bRet = tsp_i2c_read(ts->client, slave_addr, pData, 1);

  printk("\n@@@bRet=%d, slave-addr=%x", bRet, (slave_addr));
  return 1;

#ifdef USE_BASEBAND_I2C_FUNCTION //hojun_kim
  bRet = baseband_i2c_read_a_byte(slave_addr, pData);

#else

  // START
  gpio_set_value(TOUCH_I2C_SDA_F, 0);
  udelay(1);
  gpio_set_value(TOUCH_I2C_SCL_F, 0);     
  
  //Write slave addr with read bit.
  i2c_write_byte( (slave_addr<<1)|1 );
  
  // CHKECK ACK
  gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 1/*hojun_kim */, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
  gpio_set_value(TOUCH_I2C_SCL_F, 1);  
  udelay(1);
  bRet = gpio_get_value(TOUCH_I2C_SDA_F);	
  gpio_set_value(TOUCH_I2C_SCL_F, 0); 
  
  if( bRet )
    return 0;
  
  udelay(15);
  
  i2c_read_byte( pData );
  
  // SEND NAK
  gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
  gpio_set_value(TOUCH_I2C_SDA_F, 1);
  udelay(1);
  gpio_set_value(TOUCH_I2C_SCL_F, 1);     
  udelay(1);
  gpio_set_value(TOUCH_I2C_SCL_F, 0);     
  
  udelay(15);
  
  // STOP
  gpio_set_value(TOUCH_I2C_SDA_F, 0);
  udelay(1);
  gpio_set_value(TOUCH_I2C_SCL_F, 1);     
  udelay(1);
  gpio_set_value(TOUCH_I2C_SDA_F, 1);
  
  return 1;
#endif //USE_BASEBAND_I2C_FUNCTION
}

static int _i2c_write_(uint8_t slave_addr, uint8_t data)
{
  int bRet = 0;

#ifdef USE_BASEBAND_I2C_FUNCTION
  bRet = baseband_i2c_write_a_byte( slave_addr, data);

#else

  // START
  gpio_set_value(TOUCH_I2C_SDA_F, 0);
  gpio_set_value(TOUCH_I2C_SCL_F, 0);     
  
  //Write Slave Addr with write bit
  i2c_write_byte( (slave_addr<<1)|0 );
  
  // CHKECK ACK
  gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
  gpio_set_value(TOUCH_I2C_SCL_F, 1);  
  bRet = gpio_get_value(TOUCH_I2C_SDA_F);	
  gpio_set_value(TOUCH_I2C_SCL_F, 0); 
  gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);	
  
  if( bRet )
    return 0;
  
  i2c_write_byte(data);
  
  // CHKECK ACK
  gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
  gpio_set_value(TOUCH_I2C_SCL_F, 1);  
  bRet = gpio_get_value(TOUCH_I2C_SDA_F);	
  gpio_set_value(TOUCH_I2C_SCL_F, 0); 
  gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);	
  
  // STOP
  gpio_set_value(TOUCH_I2C_SDA_F, 0);  
  gpio_set_value(TOUCH_I2C_SCL_F, 1);
  gpio_set_value(TOUCH_I2C_SDA_F, 1);  
  
  return 1;    
#endif //USE_BASEBAND_I2C_FUNCTION
}

#if 0 //hojun_kim
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
//============================================================
//
//	Debugging print functions.
//
//	Change MCSDL_PRINT() to Baseband printing function
//
//============================================================
static void melfas_ts_print_result(int nRet)
{
    if( nRet == MCSDL_RET_SAME_FIRMWARE_VERSION ){
		printk("[MELFAS] Firmware Version is Same, Not download.\n");
    }	
	else if( nRet == MCSDL_RET_SUCCESS ){

		printk("[MELFAS] Firmware downloading SUCCESS.\n");

	}else{

		printk("[MELFAS] Firmware downloading FAILED  :  ");

		switch( nRet ){

			case MCSDL_RET_SUCCESS                  		:   printk("[MELFAS] MCSDL_RET_SUCCESS\n" );                 	    break;
			case MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED   	:   printk("[MELFAS] MCSDL_RET_ENTER_ISP_MODE_FAILED\n" );      	break;
			case MCSDL_RET_ERASE_FLASH_FAILED           	:   printk("[MELFAS] MCSDL_RET_ERASE_FLASH_FAILED\n" );         	break;
			case MCSDL_RET_READ_FLASH_FAILED				:   printk("[MELFAS] MCSDL_RET_READ_FLASH_FAILED\n" );         	    break;
			case MCSDL_RET_READ_EEPROM_FAILED           	:   printk("[MELFAS] MCSDL_RET_READ_EEPROM_FAILED\n" );         	break;
			case MCSDL_RET_READ_INFORMAION_FAILED        	:   printk("[MELFAS] MCSDL_RET_READ_INFORMAION_FAILED\n" );     	break;
			case MCSDL_RET_PROGRAM_FLASH_FAILED				:   printk("[MELFAS] MCSDL_RET_PROGRAM_FLASH_FAILED\n" );        	break;
			case MCSDL_RET_PROGRAM_EEPROM_FAILED        	:   printk("[MELFAS] MCSDL_RET_PROGRAM_EEPROM_FAILED\n" );      	break;
			case MCSDL_RET_PROGRAM_INFORMAION_FAILED    	:   printk("[MELFAS] MCSDL_RET_PROGRAM_INFORMAION_FAILED\n" );      break;
			case MCSDL_RET_PROGRAM_VERIFY_FAILED			:   printk("[MELFAS] MCSDL_RET_PROGRAM_VERIFY_FAILED\n" );      	break;

			case MCSDL_RET_WRONG_MODE_ERROR             	:   printk("[MELFAS] MCSDL_RET_WRONG_MODE_ERROR\n" );         	    break;
			case MCSDL_RET_WRONG_SLAVE_SELECTION_ERROR		:   printk("[MELFAS] MCSDL_RET_WRONG_SLAVE_SELECTION_ERROR\n" );    break;
			case MCSDL_RET_COMMUNICATION_FAILED				:   printk("[MELFAS] MCSDL_RET_COMMUNICATION_FAILED\n" );      	    break;
			case MCSDL_RET_READING_HEXFILE_FAILED       	:   printk("[MELFAS] MCSDL_RET_READING_HEXFILE_FAILED\n" );         break;
			case MCSDL_RET_WRONG_PARAMETER       			:   printk("[MELFAS] MCSDL_RET_WRONG_PARAMETER\n" );      		    break;
			case MCSDL_RET_FILE_ACCESS_FAILED       		:   printk("[MELFAS] MCSDL_RET_FILE_ACCESS_FAILED\n" );      	    break;
			case MCSDL_RET_MELLOC_FAILED     		  		:   printk("[MELFAS] MCSDL_RET_MELLOC_FAILED\n" );      			break;
			case MCSDL_RET_WRONG_MODULE_REVISION     		:   printk("[MELFAS] MCSDL_RET_WRONG_MODULE_REVISION\n" );      	break;

			default                             			:	printk("[MELFAS] UNKNOWN ERROR. [0x%02X].\n", nRet );        	break;
		}

		printk("\n");
	}

}
#endif
#endif

#if 1 // KHG_0619
int mcsdl_i2c_mark_finished( void )
{
    int i, nRet, bRet;
    uint8_t i2c_buffer[4];


    nRet = MCSDL_RET_PROGRAM_FLASH_FAILED;
    //-----------------------------
    // Send Program Command
    //-----------------------------
    i2c_buffer[0] = MCSDL_ISP_CMD_PROGRAM_FLASH;
    i2c_buffer[1] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK >> 8 ) & 0xFF);
    i2c_buffer[2] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK      ) & 0xFF);
    i2c_buffer[3] = 1;
 
    for(i=0; i<4; i++){

        bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[i]);

        udelay(15);
		
        if( bRet == 0 )
            goto MCSDL_I2C_MARK_FINISHED_END;

    }

    //-----------------------------
    // Program Finish-Mark
    //-----------------------------

    bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, MCSDL_I2C_DATA_FINISH_MARK);

    udelay(50);

    if( bRet == 0 )
        goto MCSDL_I2C_MARK_FINISHED_END;

    //-----------------------------
    // Get result
    //-----------------------------

    bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &i2c_buffer[0]);

    if( bRet == 0 || i2c_buffer[0] != MCSDL_MDS_ACK_PROGRAM_FLASH )
        goto MCSDL_I2C_MARK_FINISHED_END;

    nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_MARK_FINISHED_END :

   return nRet;
}


int mcsdl_i2c_check_finished( void )
{
    int i, nRet, bRet;
    uint8_t i2c_buffer[4];

    nRet = MCSDL_RET_READ_INFORMAION_FAILED;
    //-----------------------------
    // Send Read Command
    //-----------------------------
    i2c_buffer[0] = MCSDL_ISP_CMD_READ_FLASH;
    i2c_buffer[1] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK >> 8 ) & 0xFF);
    i2c_buffer[2] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK      ) & 0xFF);
    i2c_buffer[3] = 1;
 
    for(i=0; i<4; i++){

        bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[i]);

        udelay(15);
		
        if( bRet == 0 )
            goto MCSDL_I2C_CHECK_FINISHED_END;

    }

    //-----------------------------
    // Read Finish-Mark
    //-----------------------------

    bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &i2c_buffer[0]);

    udelay(50);

    if( bRet == 0 )
        goto MCSDL_I2C_CHECK_FINISHED_END;

    if(i2c_buffer[0] != MCSDL_I2C_DATA_FINISH_MARK){
        goto MCSDL_I2C_CHECK_FINISHED_END;
    }

    nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_CHECK_FINISHED_END :

   return nRet;
}
#endif

//--------------------------------------------
//
//   Write ISP Mode entering signal
//
//--------------------------------------------
static void melfas_ts_write_download_mode_signal(void)
{
	int    i;

	uint8_t enter_code[14] = { 0, 1, 0, 1, 0, 1, 0, 1,   1, 0, 0, 0, 1, 1 };//{ 0, 1, 0, 1, 0, 1, 0, 1,   1, 0, 0, 1, 0, 1 };

	//---------------------------
	// ISP mode signal 0
	//---------------------------

	for(i=0; i<14; i++){

		if( enter_code[i] )	{
			gpio_set_value(TOUCH_INT, 1);

		}else{
			gpio_set_value(TOUCH_INT, 0);
		}

		gpio_set_value(TOUCH_I2C_SCL_F, 1);
	
		udelay(15);
		gpio_set_value(TOUCH_I2C_SCL_F, 0);

		gpio_set_value(TOUCH_INT, 0);

		udelay(100);
   }

	gpio_set_value(TOUCH_I2C_SCL_F, 1);

	udelay(100);

	gpio_set_value(TOUCH_INT, 1);
}

//------------------------------------------------------------------
//
//   Enter Download mode ( MDS ISP or I2C ISP )
//
//------------------------------------------------------------------
static int melfas_ts_enter_download_mode(void)
{

	int	nRet = MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED;
	int bRet;

	uint8_t cData=0;

	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------
	vreg_disable(vreg_touch); //gpio_set_value(TOUCH_EN, 0);

	melfas_ts_hw_i2c_mode(0);

	melfas_ts_int_mode(0);

	msleep(45);

	msleep(45);

	vreg_enable(vreg_touch); //gpio_set_value(TOUCH_EN, 1);

	gpio_set_value(TOUCH_I2C_SDA_F, 1);	
    
	msleep(25);

	//-------------------------------
	// Write 1st signal
	//-------------------------------
	melfas_ts_write_download_mode_signal();

	msleep(2);//msleep(25);

//	melfas_ts_hw_i2c_mode(1);

	//-------------------------------
	// Check response
	//-------------------------------

	bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &cData);

#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("\n[MELFAS12] melfas_ts_enter_download_mode() returns - ret : 0x%x,data=%x \n", bRet, cData);
#endif
	if( bRet != 1 || cData != MCSDL_I2C_SLAVE_READY_STATUS )
		goto MCSDL_ENTER_DOWNLOAD_MODE_FINISH;

	nRet = MCSDL_RET_SUCCESS;

	//-----------------------------------
	// Entering MDS ISP mode finished.
	//-----------------------------------

MCSDL_ENTER_DOWNLOAD_MODE_FINISH:

   return nRet;
}

//------------------------------------------------------------------
//
//	Download function
//
//------------------------------------------------------------------

int melfas_ts_download(const uint8_t *pData, const uint16_t nLength )
{
	int		nRet;
	uint16_t  nCurrent=0;
	uint8_t   cLength;

	uint8_t	buffer[MELFAS_TRANSFER_LENGTH];
//hojun_kim	uint8_t	buffer2[MELFAS_TRANSFER_LENGTH];

	uint8_t	*pBuffer;

#ifdef FEATURE_MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_NOT_MATCH
	uint8_t	melfas_module_revision;
	uint8_t	melfas_module_revision_of_new_firmware;
//	uint8_t	melfas_module_hw_revision=0;
#endif

#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Starting download...\n");
#endif

	//--------------------------------------------------------------
	//
	// Enter Download mode
	//
	//--------------------------------------------------------------
	nRet = melfas_ts_enter_download_mode();

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;

	msleep(1);					// Delay '1 msec'

	//--------------------------------------------------------------
	//
	// Check H/W Revision
	//
	// Don't download firmware, if Module H/W revision does not match.
	//
	//--------------------------------------------------------------
#ifdef FEATURE_MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_NOT_MATCH

		pBuffer  = (uint8_t *)pData;

		melfas_module_revision_of_new_firmware =
				((( pBuffer[MCSDL_ADDR_FIRMWARE_VERSION+1] - '0' ) & 0x0F) << 4)
			+ 	( pBuffer[MCSDL_ADDR_FIRMWARE_VERSION+2] - '0' );


		nRet = melfas_ts_i2c_read_flash( buffer, MCSDL_ADDR_FIRMWARE_VERSION, 8 );

		if( nRet != MCSDL_RET_SUCCESS )
			goto MCSDL_DOWNLOAD_FINISH;

		melfas_module_revision = ((( buffer[1]-'0' )&0x0F) << 4 ) + ( buffer[2] - '0' );

#if 0
		nRet = melfas_ts_i2c_read_flash( buffer, MCSDL_ADDR_MODULE_REVISION, 8 );
        if( nRet != MCSDL_RET_SUCCESS )
			goto MCSDL_DOWNLOAD_FINISH;

		melfas_module_hw_revision = ((( buffer2[1]-'0' )&0x0F) << 4 ) + ( buffer2[2] - '0' );
#endif
		
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
#if 1
		printk("\n[MELFAS] melfas_revision_of_new_firmware = 0x%x,  buffer[1]=%x,buffer[2]=%x,\n",
			melfas_module_revision_of_new_firmware,buffer[1],buffer[2]);//hojun_kim
	  return MCSDL_RET_SUCCESS;//hojun_kim
#else
		printk("[MELFAS] melfas_revision_of_new_firmware = 0x%x, melfas_firmware_revision = 0x%x\n",
			melfas_module_revision_of_new_firmware,melfas_module_revision);
#endif			
#endif			
		if( mcsdl_i2c_check_finished() == MCSDL_RET_SUCCESS)   // KGH_0619
		{
#if 0			
			if( melfas_module_hw_revision != MELFAS_LATEST_HW_MODULE_REVSION)
			{
				nRet = MCSDL_RET_NOT_SUPPORT_HW_VERSION;
				goto MCSDL_DOWNLOAD_FINISH;
			}
#endif
			if( melfas_module_revision == melfas_module_revision_of_new_firmware)
			{
				nRet = MCSDL_RET_SAME_FIRMWARE_VERSION;
				goto MCSDL_DOWNLOAD_FINISH;
			}
			msleep(1);					// Delay '1 msec'
		}
#endif

	//--------------------------------------------------------------
	//
	// Erase Flash
	//
	//--------------------------------------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Erasing...\n");
#endif

	nRet = melfas_ts_i2c_erase_flash();

	if( nRet != MCSDL_RET_SUCCESS ){
		goto MCSDL_DOWNLOAD_FINISH;
	}

	msleep(1);					// Delay '1 msec'

#if 0
	//---------------------------
	//
	// Verify erase
	//
	//---------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT	
	printk("[MELFAS] Verify Erasing...\n");
#endif

	nRet = melfas_ts_i2c_read_flash( buffer, 0x00, 16 );

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;

	for(i=0; i<16; i++){

		if( buffer[i] != 0xFF ){

			nRet = MCSDL_RET_ERASE_VERIFY_FAILED;
			goto MCSDL_DOWNLOAD_FINISH;
		}
	}

	msleep(1);					// Delay '1 msec'
#endif

	//-------------------------------
	//
	// Program flash information
	//
	//-------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Program information...\n");
#endif

	nRet = mcsdl_i2c_program_info();

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;


	msleep(1);					// Delay '1 msec'


   //-------------------------------
   // Program flash
   //-------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Program flash...  ");
#endif

	pBuffer  = (uint8_t *)pData;
	nCurrent = 0;
	cLength  = MELFAS_TRANSFER_LENGTH;

	while( nCurrent < nLength ){
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
		printk("#");
#endif
		if( ( nLength - nCurrent ) < MELFAS_TRANSFER_LENGTH ){
			cLength = (uint8_t)(nLength - nCurrent);
		}

		nRet = melfas_ts_i2c_program_flash( pBuffer, nCurrent, cLength );

        if( nRet != MCSDL_RET_SUCCESS ){

			printk("[MELFAS] Program flash failed position : 0x%x / nRet : 0x%x ", nCurrent, nRet);
            goto MCSDL_DOWNLOAD_FINISH;
		}

		pBuffer  += cLength;
		nCurrent += (uint16_t)cLength;

		msleep(1);					// Delay '1 msec'

	}
#ifdef CONFIG_MACH_BEHOLD2 // KGH_0619
	nRet = mcsdl_i2c_mark_finished();

	if(nRet != MCSDL_RET_SUCCESS){
	         goto MCSDL_DOWNLOAD_FINISH;
	}
#endif

#if 0
	//-------------------------------
	//
	// Verify flash
	//
	//-------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("\n");
	printk("[MELFAS] Verify flash...   ");
#endif

	pBuffer  = (uint8_t *) pData;

	nCurrent = 0;

	cLength  = MELFAS_TRANSFER_LENGTH;

	while( nCurrent < nLength ){
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
		printk("#");
#endif
		if( ( nLength - nCurrent ) < MELFAS_TRANSFER_LENGTH ){
			cLength = (uint8_t)(nLength - nCurrent);
		}

		//--------------------
		// Read flash
		//--------------------
		nRet = melfas_ts_i2c_read_flash( buffer, nCurrent, cLength );

		//--------------------
		// Comparing
		//--------------------
		for(i=0; i<(int)cLength; i++){

			if( buffer[i] != pBuffer[i] ){

				printk("0x%04X : 0x%02X - 0x%02X\n", nCurrent, pBuffer[i], buffer[i] );
				nRet = MCSDL_RET_PROGRAM_VERIFY_FAILED;
				goto MCSDL_DOWNLOAD_FINISH;

			}
		}

		pBuffer  += cLength;
		nCurrent += (uint16_t)cLength;

		msleep(1);					// Delay '1 msec'
	}
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("\n");
#endif
#endif

	nRet = MCSDL_RET_SUCCESS;


MCSDL_DOWNLOAD_FINISH :

	msleep(1);					// Delay '1 msec'

	//---------------------------
	//	Reset command
	//---------------------------
	buffer[0] = MCSDL_ISP_CMD_RESET;

	_i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, buffer[0]);

    msleep(300);
#if defined(CONFIG_SAMSUNG_BIGFOOT)    
    melfas_ts_hw_i2c_mode(1);
#else
    melfas_ts_hw_i2c_mode(0);
#endif
    melfas_ts_int_mode(1);
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
    printk("[MELFAS] Reset!!! \n");
#endif
	return nRet;
}

//--------------------------------------------
//
//   Erase flash
//
//--------------------------------------------
static int melfas_ts_i2c_erase_flash(void)
{
	int   nRet = MCSDL_RET_ERASE_FLASH_FAILED;

	uint8_t i;
	int   bRet;

	uint8_t i2c_buffer[4] = {	MCSDL_ISP_CMD_ERASE,
	                        MCSDL_ISP_PROGRAM_TIMING_VALUE_2,
	                        MCSDL_ISP_PROGRAM_TIMING_VALUE_1,
	                        MCSDL_ISP_PROGRAM_TIMING_VALUE_0   };

   //-----------------------------
   // Send Erase code
   //-----------------------------

   for(i=0; i<4; i++){

		bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[i]);

		if( !bRet )
			goto MCSDL_I2C_ERASE_FLASH_FINISH;

		udelay(15);
   }

   //-----------------------------
   // Read Result
   //-----------------------------

	msleep(45);                  // Delay 45ms


	bRet = _i2c_read_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer);

	if( bRet && i2c_buffer[0] == MCSDL_ISP_ACK_ERASE_DONE ){

		nRet = MCSDL_RET_SUCCESS;

	}


MCSDL_I2C_ERASE_FLASH_FINISH :

   return nRet;

}

//--------------------------------------------
//
//   Read flash
//
//--------------------------------------------
static int melfas_ts_i2c_read_flash(uint8_t *pBuffer, uint16_t nAddr_start, uint8_t cLength)
{
	int nRet = MCSDL_RET_READ_FLASH_FAILED;

	int     i;
	int   bRet;
	uint8_t   cmd[4];

	//-----------------------------------------------------------------------------
	// Send Read Flash command   [ Read code - address high - address low - size ]
	//-----------------------------------------------------------------------------

	cmd[0] = MCSDL_ISP_CMD_READ_FLASH;
	cmd[1] = (uint8_t)((nAddr_start >> 8 ) & 0xFF);
	cmd[2] = (uint8_t)((nAddr_start      ) & 0xFF);
	cmd[3] = cLength;

	for(i=0; i<4; i++){

		bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, cmd[i]);

		udelay(15);

		if( bRet == 0 )
			goto MCSDL_I2C_READ_FLASH_FINISH;

   }

	//----------------------------------
	// Read Data  [ pCmd[3] == Size ]
	//----------------------------------
	for(i=0; i<(int)cmd[3]; i++){

		udelay(100);                  // Delay about 100us

		bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, pBuffer++);

		if( bRet == 0 )
			goto MCSDL_I2C_READ_FLASH_FINISH;
	}

	nRet = MCSDL_RET_SUCCESS;


MCSDL_I2C_READ_FLASH_FINISH :

	return nRet;
}

//--------------------------------------------
//
//   Program information
//
//--------------------------------------------
static int mcsdl_i2c_program_info(void)
{

	int nRet = MCSDL_RET_PROGRAM_INFORMAION_FAILED;

	int i;
	int j;
	int bRet;

	uint8_t i2c_buffer[5] = { MCSDL_ISP_CMD_PROGRAM_INFORMATION,
		                    MCSDL_ISP_PROGRAM_TIMING_VALUE,
		                    0x00,                           // High addr
		                    0x00,                           // Low  addr
		                    0x00 };                         // Data

	uint8_t info_data[] = { 0x78, 0x00, 0xC0, 0xD4, 0x01 };

	//------------------------------------------------------
	//   Send information signal for programming flash
	//------------------------------------------------------
	for(i=0; i<5; i++){

		i2c_buffer[3] = 0x08 + i;            // Low addr
		i2c_buffer[4] = info_data[i];         // Program data

		for(j=0; j<5; j++){

			bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[j]);

			if( bRet == 0 )
				goto MCSDL_I2C_PROGRAM_INFO_FINISH;

			udelay(15);
		}

		udelay(500);                     // delay about  500us

		bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &i2c_buffer[4]);

		if( bRet == 0 || i2c_buffer[4] != MCSDL_I2C_ACK_PROGRAM_INFORMATION )
			goto MCSDL_I2C_PROGRAM_INFO_FINISH;

		udelay(100);                     // delay about  100us

   }

   nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_PROGRAM_INFO_FINISH :

   return nRet;

}

//--------------------------------------------
//
//   Program Flash
//
//--------------------------------------------

static int melfas_ts_i2c_program_flash( uint8_t *pData, uint16_t nAddr_start, uint8_t cLength )
{
	int nRet = MCSDL_RET_PROGRAM_FLASH_FAILED;

	int     i;
	int   bRet;
	uint8_t    cData;

	uint8_t cmd[4];

	//-----------------------------
	// Send Read code
	//-----------------------------

	cmd[0] = MCSDL_ISP_CMD_PROGRAM_FLASH;
	cmd[1] = (uint8_t)((nAddr_start >> 8 ) & 0xFF);
	cmd[2] = (uint8_t)((nAddr_start      ) & 0xFF);
	cmd[3] = cLength;

	for(i=0; i<4; i++){

		bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, cmd[i]);

		udelay(15);

		if( bRet == 0 )
			goto MCSDL_I2C_PROGRAM_FLASH_FINISH;

	}

	//-----------------------------
	// Program Data
	//-----------------------------

	udelay(500);                  // Delay about 500us

	for(i=0; i<(int)(cmd[3]); i++){


		bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, pData[i]);

		udelay(500);                  // Delay about 500us

		if( bRet == 0 )
			goto MCSDL_I2C_PROGRAM_FLASH_FINISH;
	}

	//-----------------------------
	// Get result
	//-----------------------------

	bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &cData);

	if( bRet == 0 || cData != MCSDL_MDS_ACK_PROGRAM_FLASH )
		goto MCSDL_I2C_PROGRAM_FLASH_FINISH;

	nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_PROGRAM_FLASH_FINISH :

   return nRet;
}

#if 0//hojun_kim
#if 1
static int melfas_ts_download_firmware(struct i2c_client *);

static int melfas_ts_download_firmware(struct i2c_client *client)
{
#else
static int melfas_ts_download_firmware()
{
#endif
	int ret;

	udelay(15);

	disable_irq(ts_irq_num);					// Disable Baseband touch interrupt ISR.

	//------------------------
	// Run Download
	//------------------------
	printk(KERN_INFO "\n bin_length=%x\n", MELFAS_binary_nLength);
//	ret = melfas_ts_download(MELFAS_binary, MELFAS_binary_nLength);
  mcsdl_download_binary_data();

	enable_irq(ts_irq_num);					// Roll-back Baseband touch interrupt ISR.

	#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT

		//------------------------
		// Show result
		//------------------------

		melfas_ts_print_result( ret );

	#endif


	return ( ret == MCSDL_RET_SUCCESS );
}
#endif
#endif

int tsp_i2c_read(struct i2c_client* client, u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	rmsg.addr = client->addr;
	rmsg.flags = 0;//I2C_M_WR;
	rmsg.len = 1;
	rmsg.buf = &start_reg;
	start_reg = reg;
	ret = i2c_transfer(client->adapter, &rmsg, 1);

	if(ret>=0) {
		rmsg.flags = I2C_M_RD;
		rmsg.len = buf_size;
		rmsg.buf = rbuf;
		ret = i2c_transfer(client->adapter, &rmsg, 1 );
	}

	if( ret < 0 )
	{
		printk("[TSP] Error code : L%d\n", __LINE__ );
	}

	return ret;
}


static void melfas_tsp_init(void)
{
	int rc;
	printk(KERN_INFO "melfas_tsp_esd_init because of esd recovery\n");

	// TOUCH OFF
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);	
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);	

	gpio_set_value(TOUCH_I2C_SCL_F, 0);
	gpio_set_value(TOUCH_I2C_SDA_F, 0);
	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");

	msleep(100);

	// TOUCH ON
	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");

	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

 	gpio_set_value(TOUCH_I2C_SCL_F, 1);	
	gpio_set_value(TOUCH_I2C_SDA_F, 1);	

	msleep(30);
}

static void melfas_ts_work_func(struct work_struct *work)
{
	int ret;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf1[7];
	uint8_t buf2[1];
	int button;  
	unsigned int keycode, keypress;

	//hojun_kim	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf1);
	msg[1].buf = buf1;

	ret = i2c_transfer(ts->client->adapter, msg, 2);
  
	if (ret < 0) {
		printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
		melfas_tsp_init();
	} 
	else 
	{
		if(1/*hojun_kim bridge_on*/)  // lcd on status
		{
			int x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8; 
			int y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8; 
			int z = buf1[5];
			int finger = buf1[0] & 0x01;
			int width = buf1[6];

			printk("\n[TOUCH] x:%d, y:%d, z:%d, finger=%d\n", x, y, z, finger);

			if(buf1[0] == 0x80) // ESD! turn TSP power off => (100ms) => turn TSP power back on
			{
				printk(KERN_ERR "\n\n[TOUCH] ESD detected resetting TSP!!!\n");				
				melfas_tsp_init();				
				return;
			}
			else if((buf1[0] == 0)||(buf1[0] == 1)) // Only Finger Touch, Palm Touch Disable
			{
				if ( x < MAX_ABS_X && x > 0 && y < MAX_ABS_Y && y > 0 ) {
					printk("\n[TOUCH] x:%d, y:%d, z:%d, finger=%d\n", x, y, z, finger); 
					if(finger == 1)
					{
						input_report_abs(ts->input_dev, ABS_X, x);
						input_report_abs(ts->input_dev, ABS_Y, y);
						input_report_abs(ts->input_dev, ABS_PRESSURE, z);
						input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
					}
					input_report_key(ts->input_dev, BTN_TOUCH, finger);
					input_sync(ts->input_dev);
			}
#if 0
#if defined(CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_DEVICE_CHECK)
					
				if (finger == 0)
					printk("T%3d%3d\n", x, y);
#endif
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				input_sync(ts->input_dev);
#endif
			}
      else if(buf1[0] & 0xC0) 
      {
        msg[0].addr = ts->client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = &start_reg;
        start_reg = 0x25;
        msg[1].addr = ts->client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = sizeof(buf2);
        msg[1].buf = buf2;
        
        ret = i2c_transfer(ts->client->adapter, msg, 2);
        
        button = buf2[0]; //key:1 home key:2 menu key:3 back
        //printk("\n[TOUCH_KEY] defined button: 0x%02x \n", button);
        
        switch(button) 
        {
          case 0x01 : 
          case 0x09 :
            keycode = TOUCH_MENU;
            printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
            break;
          
          case 0x02 : 
          case 0x0A :
            keycode = TOUCH_HOME;
            printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
            break;
          
          case 0x03 : 
          case 0x0B :
            keycode = TOUCH_BACK;
            printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
            break;
          

          case 0x04 : 
          case 0x0C :
            keycode = TOUCH_SEARCH;
            printk("[TOUCH_KEY] defined button: 0x%02x., keycode: %4d.\n", button, keycode);
            break;

          
          default :
            printk("[TOUCH_KEY] undefined button: 0x%02x.\n", button);
            enable_irq(ts->client->irq);
            return;
        }

        if(button & 0x08)
          keypress = 0;
        else 
          keypress = 1;
        
        printk("[TOUCH_KEY] keycode: %4d, keypress: %4d, use_irq: %d\n", keycode, keypress, ts->use_irq); 

        input_report_key(ts->input_dev, keycode, keypress);
      }

	    input_sync(ts->input_dev);
	  }
	}

	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer, struct melfas_ts_data, timer);
	/* printk("melfas_ts_timer_func\n"); */

	queue_work(melfas_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;

	/* printk("melfas_ts_irq_handler\n"); */
	disable_irq(ts->client->irq);
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
static int melfas_fw_open(struct inode *inode, struct file *file)
{
#if 1
	printk(KERN_INFO "[melfas_fw_open] %s\n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int melfas_fw_release(struct inode *inode, struct file *file)
{
#if 1
	printk(KERN_INFO "[melfas_fw_release] %s\n", __FUNCTION__);
#endif
	return 0;
}


#define MELFAS_TSP_IOCTL_MAGIC 78

#define TSP_FW_UPDATE _IO(MELFAS_TSP_IOCTL_MAGIC, 1)

static int melfas_fw_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
       int ret;
	   
		switch (cmd) {
			case TSP_FW_UPDATE:
#ifdef DEBUG
				printk("[melfas_fw_ioctl] TSP_FW_UPDATE %x\n", cmd);
#endif
				//ret = melfas_ts_download_firmware();
				if (ret < 0)
					return ret;
				break;
			default:
#ifdef DEBUG
				printk("Unknown cmd %x\n", cmd);
#endif
				return -ENOTTY;
		}

		return 0;
}
static struct file_operations melfas_fw_fops = {
	.owner = THIS_MODULE,
	.open = melfas_fw_open,
	.release = melfas_fw_release,
	.ioctl = melfas_fw_ioctl,
};

static struct miscdevice melfas_fw_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_fw_download",
	.fops = &melfas_fw_fops,
};
#endif



static int __init melfas_ts_probe(struct i2c_client *client,  const struct i2c_device_id *devid)
{
	char rbuf;
	int ret = 0;
	uint16_t max_x, max_y;
  printk("\n====================================================");
  printk("\n=======         [TOUCH SCREEN] probe       =========");
  printk("\n====================================================\n");


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	INIT_WORK(&ts->work, melfas_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	
//#ifdef CONFIG_MACH_BEHOLD2
//#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
//    ret = melfas_ts_download_firmware(ts->client);
//  	msleep(50);
//#endif
//#endif



      rbuf=0x0F;

	//ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	ret = tsp_i2c_read(ts->client, 0x20, &rbuf, 1);
	if (ret < 0) {
		printk(KERN_ERR "[1] i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "melfas_ts_probe: Current Firmware Version %x, length=%x\n", rbuf,MELFAS_binary_nLength);
	
	rbuf=0x0F;
//  melfas_ts_download_firmware(ts->client);
//  while(vreg_disable(vreg_touch));

//  mcsdl_download_binary_data();


	//ret = i2c_smbus_read_byte_data(ts->client, 0x21);
	ret = tsp_i2c_read(ts->client, 0x1E, &rbuf, 1);	
	if (ret < 0) {
		printk(KERN_ERR "[2] i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "melfas_ts_probe: Module Revision %x\n", ret);

	rbuf=0x0F;

	//ret = i2c_smbus_read_word_data(ts->client, 0x08);
	ret = tsp_i2c_read(ts->client, 0x08, &rbuf, 1);		
	if (ret < 0) {
		printk(KERN_ERR "[3] i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[0] = max_x = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);

	rbuf=0x0F;

	//ret = i2c_smbus_read_word_data(ts->client, 0x0a);
	ret = tsp_i2c_read(ts->client, 0x0a, &rbuf, 1);	
	if (ret < 0) {
		printk(KERN_ERR "[4] i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "[5] melfas_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "melfas-tsi-touchscreen";
	
	rbuf=0x0F;

	//ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	ret = tsp_i2c_read(ts->client, 0x20, &rbuf, 1);	
	if (ret < 0) {
		printk(KERN_ERR "[6] i2c_smbus_read_byte_data failed\n");
	}
	ts->input_dev->id.version  = ret; //version_id;

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(TOUCH_HOME, ts->input_dev->keybit);
	set_bit(TOUCH_MENU, ts->input_dev->keybit);
	set_bit(TOUCH_BACK, ts->input_dev->keybit);
	set_bit(TOUCH_SEARCH, ts->input_dev->keybit);
  
	ts->input_dev->keycode = melfas_ts_tk_keycode; 
	
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	max_x = MAX_ABS_X;
	max_y = MAX_ABS_Y;
	
	input_set_abs_params(ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	printk("melfas_ts_probe: max_x %d, max_y %d\n", max_x, max_y);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
    ret = misc_register(&melfas_fw_device);
	if (ret) {
		printk(KERN_ERR "melfas_fw_device: melfas_fw_device register failed\n");
		goto err_misc_register_device_failed;
	}
#endif

    ts_irq_num = client->irq;

	if (client->irq) {
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);

		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 4;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "melfas_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;
err_misc_register_device_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret,rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	vreg_touch = vreg_get(NULL, "ldo19"); // quattro_jiny46kim
	if (IS_ERR(vreg_touch)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed(%ld)\n", __func__, "ldo19", PTR_ERR(vreg_touch));
		return;
	}
    
	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");
	msleep(200);

// quattro_jiny46kim gpio off
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);

	printk(KERN_INFO "Melfas Touchscreen : suspend.\n");

	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);


	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);



    printk(KERN_INFO "Melfas Touchscreen : resume.\n");
	
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
#if 0
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
#endif
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
#if 0
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);
#endif
}
#endif


static const struct i2c_device_id melfas_ts_id[] = {
	{ "melfas-tsi-ts", 1 }, //hojun_kim 0->5
	{ }
};

MODULE_DEVICE_TABLE(i2c, melfas_ts_id);

static struct i2c_driver melfas_ts_driver = {
	.id_table	= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
#if 1 //ndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
	.driver = {
		.name	= "melfas-tsi-ts",
	},
};
extern int mcsdl_download_binary_data(void); //hojun_kim 

static int __devinit melfas_ts_init(void)
{
	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] init        =========");
	printk("\n====================================================\n");

	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq)
		return -ENOMEM;

	vreg_touch = vreg_get(0, "ldo19"); /* VTOUCH_3.0V */
	if (IS_ERR(vreg_touch))
		return PTR_ERR(vreg_touch);

	mcsdl_download_binary_data(); //hojun_kim melfas_ts_download_firmware(NULL);  

	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_UP,GPIO_16MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_UP,GPIO_16MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_UP,GPIO_16MA), GPIO_ENABLE);

	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}
module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");

