//--------------------------------------------------------
//
//
//	Melfas MCS8000 Series Download base v1.0 2011.03.17
//
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>
//
#include <asm/gpio.h>
#include <asm/io.h>

#include "mcs8000_download.h"


//============================================================
//
//	Include MELFAS Binary code File ( ex> MELFAS_FIRM_bin.c)
//
//	Warning!!!!
//		Please, don't add binary.c file into project
//		Just #include here !!
//
//============================================================

//#include "ESCAPE_R01_V02.c" // Master Binary data
//#include "MCS8000_R10_V05_bin.c"
//#include "MCS8000_V98_bin.c"
//#include "mcs8000_bin.c"
//#include "MCS8000_bin_V73.c"
//#include "MCS8000_bin_R11_V08.c"
#if defined(CONFIG_MACH_ANCORA_TMO)
#include "ANCORA_TMO_FW_R04_V06.c"
#include "ANCORA_TMO_FW_R06_V13.c"
#else
#include "ANCORA_FW_R03_V14.c" 
#include "ANCORA_FW_R50_V15.c"
#endif
 
UINT8  ucVerifyBuffer[MELFAS_TRANSFER_LENGTH];		//	You may melloc *ucVerifyBuffer instead of this


//---------------------------------
//	Downloading functions
//---------------------------------
static int  mcsdl_download(const UINT8 *pData, const UINT16 nLength,INT8 IdxNum );

static void mcsdl_set_ready(void);
static void mcsdl_reboot_mcs(void);

static int  mcsdl_erase_flash(INT8 IdxNum);
static int  mcsdl_program_flash( UINT8 *pDataOriginal, UINT16 unLength,INT8 IdxNum );
static void mcsdl_program_flash_part( UINT8 *pData );

static int  mcsdl_verify_flash( UINT8 *pData, UINT16 nLength, INT8 IdxNum );

static void mcsdl_read_flash( UINT8 *pBuffer);
static int  mcsdl_read_flash_from( UINT8 *pBuffer, UINT16 unStart_addr, UINT16 unLength, INT8 IdxNum);

static void mcsdl_select_isp_mode(UINT8 ucMode);
static void mcsdl_unselect_isp_mode(void);

static void mcsdl_read_32bits( UINT8 *pData );
static void mcsdl_write_bits(UINT32 wordData, int nBits);
static void mcsdl_scl_toggle_twice(void);

//---------------------------------
//	ISC Downloading functions
//---------------------------------
static UINT8 mcsdl_read_ack(void);
static UINT8 mcsdl_read_byte(void);
static void mcsdl_i2c_start(void);
static void mcsdl_i2c_stop(void);
static void mcsdl_ISC_write_bits(UINT32 wordData, int nBits);
static void mcsdl_ISC_enter_download_mode();
static int  mcsdl_ISC_download(const UINT8 *pData, const UINT16 nLength,INT8 IdxNum );
static void mcsdl_ISC_set_ready(void);
static UINT8 mcsdl_ISC_read_data(UINT8 addr);
static void mcsdl_ISC_firmware_update_mode_enter();
static UINT8 mcsdl_ISC_read_firmware_status();
static UINT8 mcsdl_ISC_firmware_update(UINT8 *_pBinary_reordered, UINT16 _unDownload_size, UINT8 flash_start, UINT8 flash_end);
static void mcsdl_ISC_reboot_mcs(void);
static void mcsdl_ISC_print_result(int nRet);
//---------------------------------
//	Delay functions
//---------------------------------
static void mcsdl_delay(UINT32 nCount);


//---------------------------------
//	For debugging display
//---------------------------------
#if MELFAS_ENABLE_DBG_PRINT
static void mcsdl_print_result(int nRet);
#endif


//----------------------------------
// Download enable command
//----------------------------------
#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD

void melfas_send_download_enable_command(void)
{
	// TO DO : Fill this up

}

#endif


//============================================================
//
//	Main Download furnction
//
//   1. Run mcsdl_download( pBinary[IdxNum], nBinary_length[IdxNum], IdxNum);
//       IdxNum : 0 (Master Chip Download)
//       IdxNum : 1 (2Chip Download)
//
//
//============================================================
int mcsdl_ISC_download_binary_data(void)
{
	int nRet;
	#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_delay(MCSDL_DELAY_100US);
	#endif
    
	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	nRet = mcsdl_ISC_download( (const UINT8*) MELFAS_binary, (const UINT16)MELFAS_binary_nLength , 0);

	MELFAS_ROLLBACK_BASEBAND_ISR(); 				// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET(); 		// Roll-back Baseband watchdog timer
	
	return ( nRet == MCSDL_RET_SUCCESS );    
}

#if defined(CONFIG_MACH_ANCORA)
int mcsdl_ISC_download_binary_data_G2(void)
{
	int nRet;
	#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_delay(MCSDL_DELAY_100US);
	#endif
    
	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	nRet = mcsdl_ISC_download( (const UINT8*) MELFAS_binary_G2, (const UINT16)MELFAS_binary_nLength_G2 , 0);

	MELFAS_ROLLBACK_BASEBAND_ISR(); 				// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET(); 		// Roll-back Baseband watchdog timer
	
	return ( nRet == MCSDL_RET_SUCCESS );    
}
#endif 

int mcsdl_download_binary_data(void)
{
	int nRet;
	#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_delay(MCSDL_DELAY_100US);
	#endif

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	//------------------------
	// Run Download
	//------------------------
	nRet = mcsdl_download( (const UINT8*) MELFAS_binary, (const UINT16)MELFAS_binary_nLength , 0);
	#if MELFAS_2CHIP_DOWNLOAD_ENABLE
    	nRet = mcsdl_download( (const UINT8*) MELFAS_binary_2, (const UINT16)MELFAS_binary_nLength_2, 1); // Slave Binary data download
    #endif
	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer

	return ( nRet == MCSDL_RET_SUCCESS );
}

#if defined(CONFIG_MACH_ANCORA_TMO)
int mcsdl_download_binary_data_55T(void)
{
	int nRet;
	#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_delay(MCSDL_DELAY_100US);
	#endif

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	//------------------------
	// Run Download
	//------------------------
	nRet = mcsdl_download( (const UINT8*) MELFAS_binary_55T, (const UINT16)MELFAS_binary_nLength_55T , 0);
	
	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer

	return ( nRet == MCSDL_RET_SUCCESS );
}
#endif

#if defined(CONFIG_MACH_ANCORA)   
int mcsdl_download_binary_data_G2(void)
{
	int nRet;
	#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_delay(MCSDL_DELAY_100US);
	#endif

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	//------------------------
	// Run Download
	//------------------------
	nRet = mcsdl_download( (const UINT8*) MELFAS_binary_G2, (const UINT16)MELFAS_binary_nLength_G2 , 0);
	
	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer

	return ( nRet == MCSDL_RET_SUCCESS );
}
#endif

int mcsdl_download_binary_file(void)
{
	int nRet;
    int i;
    
	UINT8  *pBinary[2] = {NULL,NULL};
	UINT16 nBinary_length[2] ={0,0};
    UINT8 IdxNum = MELFAS_2CHIP_DOWNLOAD_ENABLE;
	//==================================================
	//
	//	1. Read '.bin file'
	//   2. *pBinary[0]       : Binary data(Master)
	//       *pBinary[1]       : Binary data(Slave)
	//	   nBinary_length[0] : Firmware size(Master)
	//	   nBinary_length[1] : Firmware size(Slave)
	//	3. Run mcsdl_download( pBinary[IdxNum], nBinary_length[IdxNum], IdxNum);
    //       IdxNum : 0 (Master Chip Download)
    //       IdxNum : 1 (2Chip Download)
	//
	//==================================================

#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
    melfas_send_download_enable_command();
    mcsdl_delay(MCSDL_DELAY_100US);
#endif
    
    MELFAS_DISABLE_BASEBAND_ISR();                  // Disable Baseband touch interrupt ISR.
    MELFAS_DISABLE_WATCHDOG_TIMER_RESET();          // Disable Baseband watchdog timer

    for(i=0;i<=IdxNum;i++){
    	if( pBinary[i] != NULL && nBinary_length[i] > 0 && nBinary_length[i] < 32*1024 ){

    		//------------------------
    		// Run Download
    		//------------------------
    		nRet = mcsdl_download( (const UINT8 *)pBinary[i], (const UINT16)nBinary_length[i], i );
    	}else{

    		nRet = MCSDL_RET_WRONG_BINARY;
    	}
    }

    MELFAS_ROLLBACK_BASEBAND_ISR();                 // Roll-back Baseband touch interrupt ISR.
    MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();         // Roll-back Baseband watchdog timer
    
	#if MELFAS_ENABLE_DBG_PRINT
	mcsdl_print_result( nRet );
	#endif

	#if 0
		if( pData != NULL )										// free memory alloced.
			free(pData);
	#endif

	return ( nRet == MCSDL_RET_SUCCESS );

}

//------------------------------------------------------------------
//
//	Download function
//
//------------------------------------------------------------------

static int mcsdl_download(const UINT8 *pBianry, const UINT16 unLength, INT8 IdxNum )
{
	int nRet;

	//---------------------------------
	// Check Binary Size
	//---------------------------------
	if( unLength >= MELFAS_FIRMWARE_MAX_SIZE ){

		nRet = MCSDL_RET_PROGRAM_SIZE_IS_WRONG;
		goto MCSDL_DOWNLOAD_FINISH;
	}


	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" - Starting download...\n");
	#endif


	//---------------------------------
	// Make it ready
	//---------------------------------
	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Ready\n");
	#endif

	mcsdl_set_ready();

	//---------------------------------
	// Erase Flash
	//---------------------------------
	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Erase\n");
	#endif

	nRet = mcsdl_erase_flash(IdxNum);

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;

	//---------------------------------
	// Program Flash
	//---------------------------------
	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Program   ");
	#endif

	nRet = mcsdl_program_flash( (UINT8*)pBianry, (UINT16)unLength, IdxNum );
	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;
	//---------------------------------
	// Verify flash
	//---------------------------------

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Verify    ");
	#endif

	nRet = mcsdl_verify_flash( (UINT8*)pBianry, (UINT16)unLength, IdxNum );

	if( nRet != MCSDL_RET_SUCCESS ) {
		nRet = mcsdl_verify_flash( (UINT8*)pBianry, (UINT16)unLength, IdxNum );

		if( nRet != MCSDL_RET_SUCCESS ) {
			nRet = mcsdl_verify_flash( (UINT8*)pBianry, (UINT16)unLength, IdxNum );

			if( nRet != MCSDL_RET_SUCCESS )
				goto MCSDL_DOWNLOAD_FINISH;
		}
	}


	nRet = MCSDL_RET_SUCCESS;


MCSDL_DOWNLOAD_FINISH :

	#if MELFAS_ENABLE_DBG_PRINT
	mcsdl_print_result( nRet );								// Show result
	#endif

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Rebooting\n");
	printk(" - Fin.\n\n");
	#endif

	mcsdl_reboot_mcs();

	return nRet;
}

//------------------------------------------------------------------
//
//	Sub functions
//
//------------------------------------------------------------------

static int mcsdl_erase_flash(INT8 IdxNum)
{
	int	  i;
	UINT8 readBuffer[32];

	//----------------------------------------
	//	Do erase
	//----------------------------------------
	if(IdxNum > 0) mcsdl_select_isp_mode(ISP_MODE_NEXT_CHIP_BYPASS);
	
	mcsdl_select_isp_mode(ISP_MODE_ERASE_FLASH);
	mcsdl_unselect_isp_mode();


	//----------------------------------------
	//	Check 'erased well'
	//----------------------------------------
	mcsdl_read_flash_from(  readBuffer	  , 0x0000, 16, IdxNum );
	mcsdl_read_flash_from( &readBuffer[16], 0x7FF0, 16, IdxNum);

	// Compare with '0xFF'
	for(i=0; i<32; i++){
		if( readBuffer[i] != 0xFF )
			return MCSDL_RET_ERASE_FLASH_VERIFY_FAILED;
	}

	return MCSDL_RET_SUCCESS;
}


static int mcsdl_program_flash( UINT8 *pDataOriginal, UINT16 unLength, INT8 IdxNum )
{
	int		i;

	UINT8	*pData;
	UINT8   ucLength;

	UINT16  addr;
	UINT32  header;

	addr   = 0;
	pData  = pDataOriginal;

    ucLength = MELFAS_TRANSFER_LENGTH;
	
//kang

	while( (addr*4) < (int)unLength){

        if( ( unLength - (addr*4) ) < MELFAS_TRANSFER_LENGTH ){
            ucLength  = (UINT8)(unLength - (addr*4) );
        }

    	//--------------------------------------
    	//	Select ISP Mode
    	//--------------------------------------

        mcsdl_delay(MCSDL_DELAY_40US);
		
        if(IdxNum > 0) mcsdl_select_isp_mode(ISP_MODE_NEXT_CHIP_BYPASS);
    	mcsdl_select_isp_mode( ISP_MODE_SERIAL_WRITE );

    	//---------------------------------------------
    	//	Header
    	//	Address[13ibts] <<1
    	//---------------------------------------------
    	header = ((addr&0x1FFF) << 1) | 0x0 ;
 		header = header << 14;

    	 //Write 18bits
    	mcsdl_write_bits( header, 18 );
		 
    	//---------------------------------
    	//	Writing
    	//---------------------------------
            addr +=1;

		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
		printk("#");
		#endif


		mcsdl_program_flash_part(pData);

		pData  += ucLength;

    	//---------------------------------------------
    	//	Tail
    	//---------------------------------------------
        MCSDL_GPIO_SDA_SET_HIGH();
		
	    mcsdl_delay(MCSDL_DELAY_40US);

        for(i=0; i<6; i++){
        
             if( i==2 ) mcsdl_delay(MCSDL_DELAY_20US);
            else if( i==3 ) mcsdl_delay(MCSDL_DELAY_40US);
        
            MCSDL_GPIO_SCL_SET_HIGH();  mcsdl_delay(MCSDL_DELAY_10US);
            MCSDL_GPIO_SCL_SET_LOW();   mcsdl_delay(MCSDL_DELAY_10US);
        }
		
    	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
    	//printk("\n");
    	#endif

    	mcsdl_unselect_isp_mode();
		
        mcsdl_delay(MCSDL_DELAY_300US);

	}

	return MCSDL_RET_SUCCESS;
}

static void mcsdl_program_flash_part( UINT8 *pData)
{
	int     i;
	UINT32	data;


		//---------------------------------
		//	Body
		//---------------------------------

		data  = (UINT32)pData[0] <<  0;
		data |= (UINT32)pData[1] <<  8;
		data |= (UINT32)pData[2] << 16;
		data |= (UINT32)pData[3] << 24;
		mcsdl_write_bits(data, 32);


}

static int mcsdl_verify_flash( UINT8 *pDataOriginal, UINT16 unLength, INT8 IdxNum )
{
	int	  i, j;
	int	  nRet;

	UINT8 *pData;
	UINT8 ucLength;

	UINT16 addr;
	UINT32 wordData;

	addr  = 0;
	pData = (UINT8 *) pDataOriginal;

	ucLength  = MELFAS_TRANSFER_LENGTH;

    while(  (addr*4) < (int)unLength){

        if( ( unLength -  (addr*4) ) < MELFAS_TRANSFER_LENGTH ){
            ucLength = (UINT8)(unLength -  (addr*4) );
        }

        mcsdl_delay(MCSDL_DELAY_40US);

    	//--------------------------------------
    	//	Select ISP Mode
    	//--------------------------------------
        if(IdxNum > 0) mcsdl_select_isp_mode(ISP_MODE_NEXT_CHIP_BYPASS);
    	mcsdl_select_isp_mode(ISP_MODE_SERIAL_READ);


    	//---------------------------------------------
    	//	Header
    	//	Address[13ibts] <<1
    	//---------------------------------------------

    	wordData   = ( (addr&0x1FFF) << 1 ) | 0x0;
    	wordData <<= 14;

    	mcsdl_write_bits( wordData, 18 );

        addr+=1;

    		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
    		printk("#");
    		#endif



    		//--------------------
    		// Read flash
    		//--------------------
    		mcsdl_read_flash( ucVerifyBuffer);

  
            MCSDL_GPIO_SDA_SET_HIGH();

            for(i=0; i<6; i++){

                 if( i==2 ) mcsdl_delay(MCSDL_DELAY_3US);
                else if( i==3 ) mcsdl_delay(MCSDL_DELAY_40US);
            
                MCSDL_GPIO_SCL_SET_HIGH();  mcsdl_delay(MCSDL_DELAY_10US);
                MCSDL_GPIO_SCL_SET_LOW();   mcsdl_delay(MCSDL_DELAY_10US);
            }
			
    		//--------------------
    		// Comparing
    		//--------------------

    		for(j=0; j<(int)ucLength; j++){

    			#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
    			printk(" %02X", ucVerifyBuffer[j] );
                #endif
                
    			if( ucVerifyBuffer[j] != pData[j] ){

    				#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
    				printk("\n [Error] Address : 0x%04X : 0x%02X - 0x%02X\n", addr, pData[j], ucVerifyBuffer[j] );
                    #endif


    				nRet = MCSDL_RET_PROGRAM_VERIFY_FAILED;
    				goto MCSDL_VERIFY_FLASH_FINISH;

    			}               
    		}

    		pData += ucLength;

    		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
    		//printk("\n");
    		#endif
			 
			mcsdl_unselect_isp_mode();
    	}

	nRet = MCSDL_RET_SUCCESS;

MCSDL_VERIFY_FLASH_FINISH:

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk("\n");
	#endif

	mcsdl_unselect_isp_mode();

	return nRet;
}


static void mcsdl_read_flash( UINT8 *pBuffer)
{
	int i;
    
    MCSDL_GPIO_SDA_SET_LOW();

	mcsdl_delay(MCSDL_DELAY_40US);

    for (i=0; i< 5; i++){
        MCSDL_GPIO_SCL_SET_HIGH();  mcsdl_delay(MCSDL_DELAY_10US);
        MCSDL_GPIO_SCL_SET_LOW();  mcsdl_delay(MCSDL_DELAY_10US);
        }

		mcsdl_read_32bits( pBuffer );
}

static int mcsdl_read_flash_from( UINT8 *pBuffer, UINT16 unStart_addr, UINT16 unLength, INT8 IdxNum)
{
	int i;
	int j;

	UINT8  ucLength;

	UINT16 addr;
	UINT32 wordData;

	if( unLength >= MELFAS_FIRMWARE_MAX_SIZE ){
		return MCSDL_RET_PROGRAM_SIZE_IS_WRONG;
	}

	addr  = 0;
	ucLength  = MELFAS_TRANSFER_LENGTH;

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" %04X : ", unStart_addr );
	#endif

	for( i = 0; i < (int)unLength; i+=(int)ucLength ){

		addr = (UINT16)i;
     	if(IdxNum > 0) 
            mcsdl_select_isp_mode(ISP_MODE_NEXT_CHIP_BYPASS);   

        mcsdl_select_isp_mode(ISP_MODE_SERIAL_READ);
        wordData   = ( ((unStart_addr + addr)&0x1FFF) << 1 ) | 0x0;
        wordData <<= 14;

        mcsdl_write_bits( wordData, 18 );

		if( ( unLength - addr ) < MELFAS_TRANSFER_LENGTH ){

			ucLength = (UINT8)(unLength - addr);
		}

		//--------------------
		// Read flash
		//--------------------
		mcsdl_read_flash( &pBuffer[addr]);


		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
		for(j=0; j<(int)ucLength; j++){
			printk("%02X ", pBuffer[j] );
		}
		#endif
        
        mcsdl_unselect_isp_mode();

	}

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	//printk("\n");
	#endif

	return MCSDL_RET_SUCCESS;

}


static void mcsdl_set_ready(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------

	MCSDL_VDD_SET_LOW(); // power 

	//MCSDL_CE_SET_LOW();
	//MCSDL_CE_SET_OUTPUT();

	//MCSDL_SET_GPIO_I2C();

	MCSDL_GPIO_SDA_SET_LOW();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_GPIO_SCL_SET_LOW();
	MCSDL_GPIO_SCL_SET_OUTPUT();

	MCSDL_RESETB_SET_LOW();
	MCSDL_RESETB_SET_OUTPUT();

	mcsdl_delay(MCSDL_DELAY_25MS);						// Delay for Stable VDD

	MCSDL_VDD_SET_HIGH();
	//MCSDL_CE_SET_HIGH();

	MCSDL_GPIO_SDA_SET_HIGH();

	mcsdl_delay(MCSDL_DELAY_40MS); 						// Delay '30 msec'

}

static void mcsdl_reboot_mcs(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	MCSDL_RESETB_SET_LOW();
	MCSDL_RESETB_SET_OUTPUT();

	mcsdl_vdd_off();
	gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
	gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
	msleep(300);

	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	MCSDL_RESETB_SET_INPUT();
	MCSDL_RESETB_SET_HIGH();
	gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
	gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN   

	mcsdl_vdd_on();
 
	msleep(300);	
/*
	MCSDL_VDD_SET_LOW();

	//MCSDL_CE_SET_LOW();
	//MCSDL_CE_SET_OUTPUT();

	MCSDL_GPIO_SDA_SET_HIGH();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_GPIO_SCL_SET_HIGH();
	MCSDL_GPIO_SCL_SET_OUTPUT();

	//MCSDL_SET_HW_I2C();

	MCSDL_RESETB_SET_LOW();
	MCSDL_RESETB_SET_OUTPUT();

	mcsdl_delay(MCSDL_DELAY_25MS);						// Delay for Stable VDD

	MCSDL_RESETB_SET_INPUT();
	MCSDL_VDD_SET_HIGH();

	MCSDL_RESETB_SET_HIGH();
	//MCSDL_CE_SET_HIGH();

	mcsdl_delay(MCSDL_DELAY_30MS); 						// Delay '25 msec'
*/
}


//--------------------------------------------
//
//   Write ISP Mode entering signal
//
//--------------------------------------------

static void mcsdl_select_isp_mode(UINT8 ucMode)
{
	int    i;

	UINT8 enteringCodeMassErase[16]   = { 0,1,0,1,1,0,0,1,1,1,1,1,0,0,1,1 };
	UINT8 enteringCodeSerialWrite[16] = { 0,1,1,0,0,0,1,0,1,1,0,0,1,1,0,1 };
	UINT8 enteringCodeSerialRead[16]  = { 0,1,1,0,1,0,1,0,1,1,0,0,1,0,0,1 };
	UINT8 enteringCodeNextChipBypass[16]  = { 1,1,0,1,1,0,0,1,0,0,1,0,1,1,0,1 };

	UINT8 *pCode;


	//------------------------------------
	// Entering ISP mode : Part 1
	//------------------------------------

		 if( ucMode == ISP_MODE_ERASE_FLASH       ) pCode = enteringCodeMassErase;
	else if( ucMode == ISP_MODE_SERIAL_WRITE      ) pCode = enteringCodeSerialWrite;
	else if( ucMode == ISP_MODE_SERIAL_READ       ) pCode = enteringCodeSerialRead;
    else if( ucMode == ISP_MODE_NEXT_CHIP_BYPASS  ) pCode = enteringCodeNextChipBypass;

	for(i=0; i<16; i++){

		if( pCode[i] == 1 )	
            MCSDL_RESETB_SET_HIGH();
		else			
            MCSDL_RESETB_SET_LOW();

        mcsdl_delay(MCSDL_DELAY_3US);        

		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_3US);
   }

	MCSDL_RESETB_SET_HIGH();			// High
	
	//---------------------------------------------------
	// Entering ISP mode : Part 2	- Only Mass Erase
	//---------------------------------------------------

	 if( ucMode == ISP_MODE_ERASE_FLASH   ){
        mcsdl_delay(MCSDL_DELAY_7US);
		for(i=0; i<4; i++){

				 if( i==2 ) mcsdl_delay(MCSDL_DELAY_25MS);
			else if( i==3 ) mcsdl_delay(MCSDL_DELAY_150US);

			MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_3US);
			MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_7US);
		}
	}
}


static void mcsdl_unselect_isp_mode(void)
{
	int i;

	// MCSDL_GPIO_SDA_SET_HIGH();
	// MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_RESETB_SET_LOW();	

	mcsdl_delay(MCSDL_DELAY_3US);
	
	for(i=0; i<10; i++){

		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_3US);
	}

}



static void mcsdl_read_32bits( UINT8 *pData )
{
	int i, j;

	MCSDL_GPIO_SDA_SET_INPUT();

        for (i=3; i>=0; i--){

		pData[i] = 0;

		for (j=0; j<8; j++){

			pData[i] <<= 1;

                MCSDL_GPIO_SCL_SET_HIGH();  mcsdl_delay(MCSDL_DELAY_3US);       
                MCSDL_GPIO_SCL_SET_LOW();       mcsdl_delay(MCSDL_DELAY_3US);

			if ( MCSDL_GPIO_SDA_IS_HIGH() )
				pData[i] |= 0x01;
                

		}
	}

}

static void mcsdl_write_bits(UINT32 wordData, int nBits)
{
	int i;

	MCSDL_GPIO_SDA_SET_LOW();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	for (i=0; i<nBits; i++){

		if ( wordData & 0x80000000 ) {	MCSDL_GPIO_SDA_SET_HIGH();	}
		else						 {	MCSDL_GPIO_SDA_SET_LOW();	}

		mcsdl_delay(MCSDL_DELAY_3US);

		MCSDL_GPIO_SCL_SET_HIGH();		mcsdl_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();		mcsdl_delay(MCSDL_DELAY_3US);

		wordData <<= 1;
	}
}

static void mcsdl_scl_toggle_twice(void)
{

	MCSDL_GPIO_SDA_SET_HIGH();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_20US);
	MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_20US);

	MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_20US);
	MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_20US);
}

//============================================================
//
//	ISC Download Function
//
//============================================================

static UINT8 mcsdl_read_ack(void)
{
	int i;
	UINT8 pData = 0x00;
	MCSDL_GPIO_SDA_SET_LOW();
	MCSDL_GPIO_SDA_SET_INPUT();

	MCSDL_GPIO_SCL_SET_HIGH();  mcsdl_delay(MCSDL_DELAY_3US);       
	if ( MCSDL_GPIO_SDA_IS_HIGH()) pData = 0x01;
	MCSDL_GPIO_SCL_SET_LOW();mcsdl_delay(MCSDL_DELAY_3US);
	return pData;
}

static UINT8 mcsdl_read_byte(void)
{
	int i;
	UINT8 pData = 0x00;
	MCSDL_GPIO_SDA_SET_LOW();
	MCSDL_GPIO_SDA_SET_INPUT();

	for (i = 0; i < 8; i++)
	{
		pData <<= 1;
		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_3US);       
		if ( MCSDL_GPIO_SDA_IS_HIGH())	pData |= 0x01;
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_3US);
	}
	return pData;
}

static void mcsdl_i2c_start(void)
{
	MCSDL_GPIO_ISC_SDA_SET_OUTPUT(1);mcsdl_delay(MCSDL_DELAY_1US);
	MCSDL_GPIO_ISC_SCL_SET_OUTPUT(1);mcsdl_delay(MCSDL_DELAY_1US);

	MCSDL_GPIO_SDA_SET_LOW();mcsdl_delay(MCSDL_DELAY_1US);
	MCSDL_GPIO_SCL_SET_LOW();
}

static void mcsdl_i2c_stop(void)
{
	MCSDL_GPIO_ISC_SCL_SET_OUTPUT(0);mcsdl_delay(MCSDL_DELAY_1US);
	MCSDL_GPIO_ISC_SDA_SET_OUTPUT(0);mcsdl_delay(MCSDL_DELAY_1US);

	MCSDL_GPIO_SCL_SET_HIGH();mcsdl_delay(MCSDL_DELAY_1US);
	MCSDL_GPIO_SDA_SET_HIGH();
}

static void mcsdl_ISC_write_bits(UINT32 wordData, int nBits)
{
	int i;

	MCSDL_GPIO_ISC_SDA_SET_OUTPUT(0);
	MCSDL_GPIO_SDA_SET_LOW();

	for (i = 0; i < nBits; i++)
	{
		if ( wordData & 0x80000000 )	{ MCSDL_GPIO_SDA_SET_HIGH(); }
		else						{ MCSDL_GPIO_SDA_SET_LOW(); }

		mcsdl_delay(MCSDL_DELAY_3US);

		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_delay(MCSDL_DELAY_3US);

		wordData <<= 1;
		if ((i % 8) == 7)
		{
			mcsdl_read_ack(); //read Ack

			MCSDL_GPIO_ISC_SDA_SET_OUTPUT(0);
			MCSDL_GPIO_SDA_SET_LOW();
		}
	}
}

static void mcsdl_ISC_enter_download_mode()
{
	UINT32 wordData = 0x00000000;
	UINT8  write_buffer[4];

	mcsdl_i2c_start();
	write_buffer[0] = ISC_MODE_SLAVE_ADDRESS << 1; // slave addr
	write_buffer[1] = ISC_DOWNLOAD_MODE_ENTER; // command
	write_buffer[2] = 0x01; // sub_command
	wordData = (write_buffer[0] << 24) | (write_buffer[1]<< 16) | (write_buffer[2]<< 8);
	mcsdl_ISC_write_bits( wordData, 24 );
	mcsdl_i2c_stop();
}

static int mcsdl_ISC_download(const UINT8 *pBianry, const UINT16 unLength, INT8 IdxNum)
{
	int nRet;
	int i=0;
	UINT8 fw_status = 0;
	INT8 dl_enable_bit = 0x00;
	UINT8 private_flash_start = ISC_PRIVATE_CONFIG_FLASH_START;
	UINT8 public_flash_start = ISC_PUBLIC_CONFIG_FLASH_START;
	UINT8 core_version;
	UINT8 flash_start[3] = {0,};
	UINT8 flash_end[3] =  {0,};

	//---------------------------------
	// Check Binary Size
	//---------------------------------
	if (unLength >= MELFAS_FIRMWARE_MAX_SIZE)
	{
		nRet = MCSDL_RET_PROGRAM_SIZE_IS_WRONG;
		goto MCSDL_DOWNLOAD_FINISH;
	}

	//---------------------------------
	// set download enable mode
	//---------------------------------
	if (MELFAS_CORE_FIRWMARE_UPDATE_ENABLE) 
	{
		dl_enable_bit |= 0x01;
		printk("<MELFAS> Core firmware download.\n");
	}
	if (MELFAS_PRIVATE_CONFIGURATION_UPDATE_ENABLE)
	{
		dl_enable_bit |= 0x02;
		printk("<MELFAS> Private Configration download.\n");
	}
	if (MELFAS_PUBLIC_CONFIGURATION_UPDATE_ENABLE)
	{
		dl_enable_bit |= 0x04;
		printk("<MELFAS> Public Configration download.\n");
	}

	//for (i = 0; i < 3; i++)
	{
		//if (dl_enable_bit & (1 << i))
		{
			//---------------------------------
			// Make it ready
			//---------------------------------
#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
			printk("<MELFAS> Ready\n");
#endif

			mcsdl_ISC_set_ready();

#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
			printk("<MELFAS> firmware_download_via_ISC start!!!\n");
#endif

			//--------------------------------------------------------------
			// INITIALIZE
			//--------------------------------------------------------------
			printk("<MELFAS> ISC_DOWNLOAD_MODE_ENTER\n\n");            
			mcsdl_ISC_enter_download_mode();
			mcsdl_delay(MCSDL_DELAY_100MS);

#if ISC_READ_DOWNLOAD_POSITION
			printk("<MELFAS> Read download position.\n\n");            
			private_flash_start = mcsdl_ISC_read_data(ISC_PRIVATE_CONFIGURATION_START_ADDR);
			public_flash_start = mcsdl_ISC_read_data(ISC_PUBLIC_CONFIGURATION_START_ADDR);
#endif

			flash_start[0] = 0;
			flash_end[0] = flash_end[2] = 31;
            
			flash_start[1] = private_flash_start;
			flash_start[2] = flash_end[1] = public_flash_start;
			printk("<MELFAS> Private Configration start at %2dKB, Public Configration start at %2dKB\n", private_flash_start, public_flash_start );

			mcsdl_delay(MCSDL_DELAY_60MS);

			//--------------------------------------------------------------
			// FIRMWARE UPDATE MODE ENTER
			//--------------------------------------------------------------
			printk("<MELFAS> FIRMWARE_UPDATE_MODE_ENTER\n\n");            
			mcsdl_ISC_firmware_update_mode_enter();
			mcsdl_delay(MCSDL_DELAY_60MS);

			fw_status = mcsdl_ISC_read_firmware_status();

			if (fw_status == 0x01)
			{
				printk("<MELFAS> Firmware update mode enter success!!!\n");
			}
			else
			{
				printk("<MELFAS> Error detected!! firmware status is 0x%02x.\n", fw_status);
				nRet = MCSDL_FIRMWARE_UPDATE_MODE_ENTER_FAILED;
				goto MCSDL_DOWNLOAD_FINISH;
			}

			mcsdl_delay(MCSDL_DELAY_60MS);

			//--------------------------------------------------------------
			// FIRMWARE UPDATE 
			//--------------------------------------------------------------
			printk("<MELFAS> FIRMWARE UPDATE\n\n");            
			nRet = mcsdl_ISC_firmware_update((UINT8 *)pBianry, (UINT16)unLength, flash_start[0],flash_end[0]);

			if(nRet != MCSDL_RET_SUCCESS) goto MCSDL_DOWNLOAD_FINISH;

			//--------------------------------------------------------------
			// LEAVE FIRMWARE UPDATE MODE
			//--------------------------------------------------------------
			printk("<MELFAS> LEAVE FIRMWARE UPDATE MODE\n\n");            

			nRet = MCSDL_RET_SUCCESS;

MCSDL_DOWNLOAD_FINISH :

#if MELFAS_ENABLE_DBG_PRINT
			mcsdl_ISC_print_result( nRet );								// Show result
#endif

#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
			printk("<MELMAS> Rebooting\n");
			printk("<MELMAS>  - Fin.\n\n");
#endif

			mcsdl_ISC_reboot_mcs();
		}
	}

	return nRet;
}

    
static void mcsdl_ISC_set_ready(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------

	MCSDL_VDD_SET_LOW(); // power 

	//MCSDL_SET_GPIO_I2C();

	MCSDL_GPIO_ISC_SDA_SET_OUTPUT(1);
	MCSDL_GPIO_SDA_SET_HIGH();

	MCSDL_GPIO_ISC_SCL_SET_OUTPUT(1);
	MCSDL_GPIO_SCL_SET_HIGH();

	MCSDL_RESETB_SET_INPUT();

	//MCSDL_CE_SET_HIGH;
	//MCSDL_CE_SET_OUTPUT();
	mcsdl_delay(MCSDL_DELAY_60MS);						// Delay for Stable VDD

	MCSDL_VDD_SET_HIGH();

	mcsdl_delay(MCSDL_DELAY_60MS);						// Delay for Stable VDD
}    

static UINT8 mcsdl_ISC_read_data(UINT8 addr)
{
	UINT32 wordData = 0x00000000;
	UINT8  write_buffer[4];
	UINT8 flash_start;

	mcsdl_i2c_start();
	write_buffer[0] = ISC_MODE_SLAVE_ADDRESS << 1;
	write_buffer[1] = addr; // command
	wordData = (write_buffer[0] << 24) | (write_buffer[1]<< 16);

	mcsdl_ISC_write_bits( wordData, 16 );
	mcsdl_delay(MCSDL_DELAY_10MS);

	mcsdl_i2c_start();
	// 1byte read
	wordData = (ISC_MODE_SLAVE_ADDRESS << 1 | 0x01) << 24;
	mcsdl_ISC_write_bits( wordData, 8 );
	flash_start = mcsdl_read_byte();
	wordData = (0x01) << 31;
	mcsdl_ISC_write_bits( wordData, 1 ); //Nack
	mcsdl_i2c_stop();
	return flash_start;
}

static void mcsdl_ISC_firmware_update_mode_enter()
{
	UINT32 wordData = 0x00000000;
	mcsdl_i2c_start();
	wordData = (ISC_MODE_SLAVE_ADDRESS << 1) << 24 | (0xAE << 16) | (0x55 << 8) | (0x00);
	mcsdl_ISC_write_bits( wordData, 32 );
	wordData = 0x00000000;
	mcsdl_ISC_write_bits( wordData, 32 );
	mcsdl_ISC_write_bits( wordData, 24 );
	mcsdl_i2c_stop();
}

static UINT8 mcsdl_ISC_read_firmware_status()
{
	UINT32 wordData = 0x00000000;
	UINT8 fw_status;
	mcsdl_i2c_start();
	// WRITE 0xAF
	wordData = (ISC_MODE_SLAVE_ADDRESS << 1) << 24 | (0xAF << 16);
	mcsdl_ISC_write_bits( wordData, 16 );
	mcsdl_i2c_stop();
	mcsdl_delay(MCSDL_DELAY_100MS);

	mcsdl_i2c_start();
	// 1byte read
	wordData = (ISC_MODE_SLAVE_ADDRESS << 1 | 0x01) << 24;
	mcsdl_ISC_write_bits( wordData, 8 );
	fw_status = mcsdl_read_byte();
	wordData = (0x01) << 31;
	mcsdl_ISC_write_bits( wordData, 1 ); //Nack
	mcsdl_i2c_stop();
	return fw_status;
}

static UINT8 mcsdl_ISC_firmware_update(UINT8 *_pBinary_reordered, UINT16 _unDownload_size, UINT8 flash_start, UINT8 flash_end)
{
	int i = 0, j = 0, n, m;
	UINT8 fw_status;
	UINT32 wordData = 0x00000000;
	UINT16 nOffset = 0;
	UINT16 cLength = 8;
	UINT16 CRC_check_buf,CRC_send_buf,IN_data;
	UINT16 XOR_bit_1,XOR_bit_2,XOR_bit_3;
	UINT8  write_buffer[64];

	nOffset =  0;
	cLength = 8; //256

	printk("<MELFAS> flash start : %2d, flash end : %2d\n", flash_start, flash_end);    

	while (flash_start + nOffset < flash_end)
	{  
		CRC_check_buf = 0xFFFF;
		mcsdl_i2c_start();
		write_buffer[0] = ISC_MODE_SLAVE_ADDRESS << 1;
		write_buffer[1] = 0xAE; // command
		write_buffer[2] = 0xF1; // sub_command 
		write_buffer[3] = flash_start + nOffset;

		wordData = (write_buffer[0] << 24) | (write_buffer[1]<< 16) | (write_buffer[2]<< 8) | write_buffer[3];
		mcsdl_ISC_write_bits( wordData, 32 );
		mcsdl_delay(MCSDL_DELAY_100MS);
		mcsdl_delay(MCSDL_DELAY_100MS);

#if MELFAS_CRC_CHECK_ENABLE
		for (m = 7; m >= 0; m--)
		{
			IN_data =(write_buffer[3] >>m) & 0x01;
			XOR_bit_1 = (CRC_check_buf & 0x0001) ^ IN_data;
			XOR_bit_2 = XOR_bit_1^(CRC_check_buf>>11 & 0x01);
			XOR_bit_3 = XOR_bit_1^(CRC_check_buf>>4 & 0x01);
			CRC_send_buf = (XOR_bit_1 <<4) | (CRC_check_buf >> 12 & 0x0F);
			CRC_send_buf = (CRC_send_buf<<7) | (XOR_bit_2 <<6) | (CRC_check_buf >>5 & 0x3F);
			CRC_send_buf = (CRC_send_buf<<4) | (XOR_bit_3 <<3) | (CRC_check_buf>>1 & 0x0007);
			CRC_check_buf = CRC_send_buf;
		}
		//	printk("<MELFAS> CRC_check_buf 0x%02x, 0x%02x\n", (UINT8)(CRC_check_buf >> 8 & 0xFF), (UINT8)(CRC_check_buf & 0xFF));    				
#endif               
		if (nOffset < _unDownload_size/1024 +1)
		{
			for (j = 0; j < 32; j++)
			{
				for (i = 0; i < cLength; i++)
				{
					write_buffer[i*4+3] = _pBinary_reordered[(flash_start+nOffset)*1024+j*32+i*4+0];
					write_buffer[i*4+2] = _pBinary_reordered[(flash_start+nOffset)*1024+j*32+i*4+1];
					write_buffer[i*4+1] = _pBinary_reordered[(flash_start+nOffset)*1024+j*32+i*4+2];
					write_buffer[i*4+0] = _pBinary_reordered[(flash_start+nOffset)*1024+j*32+i*4+3];
					//printk("<MELFAS> write buffer : 0x%02x,0x%02x,0x%02x,0x%02x\n", write_buffer[i*4+0],write_buffer[i*4+1],write_buffer[i*4+2],write_buffer[i*4+3]);
#if MELFAS_CRC_CHECK_ENABLE
					for (n = 0; n < 4; n++)
					{
						for (m = 7; m >= 0; m--)
						{
							IN_data =(write_buffer[i*4+n]>>m) & 0x0001;
							XOR_bit_1 = (CRC_check_buf & 0x0001) ^ IN_data;
							XOR_bit_2 = XOR_bit_1^(CRC_check_buf>>11 & 0x01);
							XOR_bit_3 = XOR_bit_1^(CRC_check_buf>>4 & 0x01);
							CRC_send_buf = (XOR_bit_1 <<4) | (CRC_check_buf >> 12 & 0x0F);
							CRC_send_buf = (CRC_send_buf<<7) | (XOR_bit_2 <<6) | (CRC_check_buf >>5 & 0x3F);
							CRC_send_buf = (CRC_send_buf<<4) | (XOR_bit_3 <<3) | (CRC_check_buf>>1 & 0x0007);
							CRC_check_buf = CRC_send_buf;
						}
					}
					//printk("<MELFAS> CRC_check_buf 0x%02x, 0x%02x\n", (UINT8)(CRC_check_buf >> 8 & 0xFF), (UINT8)(CRC_check_buf & 0xFF));
#endif                               
				}

				for(i=0;i<cLength;i++)
				{
					wordData = (write_buffer[i*4+0] << 24) | (write_buffer[i*4+1]<< 16) | (write_buffer[i*4+2]<< 8) | write_buffer[i*4+3];
					mcsdl_ISC_write_bits( wordData, 32 );
					mcsdl_delay(MCSDL_DELAY_100US);								
				}
			}
		}

#if MELFAS_CRC_CHECK_ENABLE
		write_buffer[1] =  CRC_check_buf & 0xFF;
		write_buffer[0] = CRC_check_buf >> 8 & 0xFF;

		wordData = (write_buffer[0] << 24) | (write_buffer[1]<< 16);
		mcsdl_ISC_write_bits( wordData, 16 );
		//printk("<MELFAS> CRC_data = 0x%02x 0x%02x\n",write_buffer[0],write_buffer[1]);
		mcsdl_delay(MCSDL_DELAY_100US); 							
#endif
		mcsdl_i2c_stop();

#if MELFAS_CRC_CHECK_ENABLE
		fw_status = mcsdl_ISC_read_firmware_status();

		if(fw_status == 0x03)
		{
			printk("<MELFAS> Firmware update success!!!\n");
		}
		else
		{
			printk("<MELFAS> Error detected!! firmware status is 0x%02x.\n", fw_status);
			return MCSDL_FIRMWARE_UPDATE_FAILED;
		}
#endif
		nOffset += 1;
		printk("<MELFAS> %d KB Downloaded...\n",nOffset);            
	}

	return MCSDL_RET_SUCCESS;

}

static void mcsdl_ISC_reboot_mcs(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------
	mcsdl_ISC_set_ready();
}

//============================================================
//
//	Delay Function
//
//============================================================
static void mcsdl_delay(UINT32 nCount)
{

		switch(nCount) 
	{
		case MCSDL_DELAY_1US :		
			udelay(1); 
			break;
		case MCSDL_DELAY_2US :		
			udelay(2); 
			break;
		case MCSDL_DELAY_3US :		
			udelay(3); 
			break;
		case MCSDL_DELAY_5US :		
			udelay(5); 
			break;
		case MCSDL_DELAY_7US :		
			udelay(7); 
			break;
		case MCSDL_DELAY_10US :		
			udelay(10); 
			break;
		case MCSDL_DELAY_15US : 	
			udelay(15); 
			break;
		case MCSDL_DELAY_20US : 	
			udelay(20); 
			break;
        case MCSDL_DELAY_40US :
            udelay(40); 
            break;  
        case MCSDL_DELAY_70US :
            udelay(70); 
            break;             
		case MCSDL_DELAY_100US :
			udelay(100); 
			break;          
		case MCSDL_DELAY_150US :
			udelay(150);
			break;
		case MCSDL_DELAY_300US :
			udelay(300); 
			break;              
		case MCSDL_DELAY_500US :
			udelay(500);
			break;
		case MCSDL_DELAY_800US :
			udelay(800);
			break;
		case MCSDL_DELAY_1MS :
			msleep(1);
			break;
		case MCSDL_DELAY_5MS :
			msleep(5);
			break;
		case MCSDL_DELAY_10MS :
			msleep(10);
			break;
		case MCSDL_DELAY_25MS :
			msleep(25);
			break;
		case MCSDL_DELAY_30MS :
			msleep(30);
			break;
		case MCSDL_DELAY_40MS :
			msleep(40);
			break;            
		case MCSDL_DELAY_45MS :
			msleep(45);
			break;
            break;
        case MCSDL_DELAY_60MS :
            msleep(60);
            break;              
        case MCSDL_DELAY_100MS :
            mdelay(100);
            break;
		default : 
			break;	
	}// Please, Use your delay function


}



//============================================================
//
//	Debugging print functions.
//
//============================================================

#ifdef MELFAS_ENABLE_DBG_PRINT

static void mcsdl_print_result(int nRet)
{
	if( nRet == MCSDL_RET_SUCCESS ){

		printk(" > MELFAS Firmware downloading SUCCESS.\n");

	}else{

		printk(" > MELFAS Firmware downloading FAILED  :  ");

		switch( nRet ){

			case MCSDL_RET_SUCCESS                  		:   printk("MCSDL_RET_SUCCESS\n" );                 		break;
			case MCSDL_RET_ERASE_FLASH_VERIFY_FAILED		:   printk("MCSDL_RET_ERASE_FLASH_VERIFY_FAILED\n" );		break;
			case MCSDL_RET_PROGRAM_VERIFY_FAILED			:   printk("MCSDL_RET_PROGRAM_VERIFY_FAILED\n" );      	    break;

			case MCSDL_RET_PROGRAM_SIZE_IS_WRONG			:   printk("MCSDL_RET_PROGRAM_SIZE_IS_WRONG\n" );    		break;
			case MCSDL_RET_VERIFY_SIZE_IS_WRONG				:   printk("MCSDL_RET_VERIFY_SIZE_IS_WRONG\n" );      		break;
			case MCSDL_RET_WRONG_BINARY						:   printk("MCSDL_RET_WRONG_BINARY\n" );      				break;

			case MCSDL_RET_READING_HEXFILE_FAILED       	:   printk("MCSDL_RET_READING_HEXFILE_FAILED\n" );			break;
			case MCSDL_RET_FILE_ACCESS_FAILED       		:   printk("MCSDL_RET_FILE_ACCESS_FAILED\n" );				break;
			case MCSDL_RET_MELLOC_FAILED     		  		:   printk("MCSDL_RET_MELLOC_FAILED\n" );      			    break;

			case MCSDL_RET_WRONG_MODULE_REVISION     		:   printk("MCSDL_RET_WRONG_MODULE_REVISION\n" );      	    break;

			default                             			:	printk("UNKNOWN ERROR. [0x%02X].\n", nRet );      		break;
		}

		printk("\n");
	}

}

static void mcsdl_ISC_print_result(int nRet)
{
	if( nRet == MCSDL_RET_SUCCESS )
	{
		printk("<MELFAS> Firmware downloading SUCCESS.\n");
	}
	else
	{
		printk("<MELFAS> Firmware downloading FAILED  :  ");
		switch( nRet )
		{
			case MCSDL_RET_SUCCESS :						printk("<MELFAS> MCSDL_RET_SUCCESS\n" );						break;
			case MCSDL_FIRMWARE_UPDATE_MODE_ENTER_FAILED :	printk("<MELFAS> MCSDL_FIRMWARE_UPDATE_MODE_ENTER_FAILED\n" );	break;
			case MCSDL_RET_PROGRAM_VERIFY_FAILED :			printk("<MELFAS> MCSDL_RET_PROGRAM_VERIFY_FAILED\n" );			break;

			case MCSDL_RET_PROGRAM_SIZE_IS_WRONG :			printk("<MELFAS> MCSDL_RET_PROGRAM_SIZE_IS_WRONG\n" );			break;
			case MCSDL_RET_VERIFY_SIZE_IS_WRONG :				printk("<MELFAS> MCSDL_RET_VERIFY_SIZE_IS_WRONG\n" );		break;
			case MCSDL_RET_WRONG_BINARY :					printk("<MELFAS> MCSDL_RET_WRONG_BINARY\n" );					break;

			case MCSDL_RET_READING_HEXFILE_FAILED :			printk("<MELFAS> MCSDL_RET_READING_HEXFILE_FAILED\n" );			break;
			case MCSDL_RET_FILE_ACCESS_FAILED :				printk("<MELFAS> MCSDL_RET_FILE_ACCESS_FAILED\n" );				break;
			case MCSDL_RET_MELLOC_FAILED :					printk("<MELFAS> MCSDL_RET_MELLOC_FAILED\n" );					break;

			case MCSDL_RET_WRONG_MODULE_REVISION :			printk("<MELFAS> MCSDL_RET_WRONG_MODULE_REVISION\n" );			break;

			default :										printk("<MELFAS> UNKNOWN ERROR. [0x%02X].\n", nRet );			break;
		}

		printk("\n");
	}

}


#endif


#if MELFAS_ENABLE_DELAY_TEST

//============================================================
//
//	For initial testing of delay and gpio control
//
//	You can confirm GPIO control and delay time by calling this function.
//
//============================================================

void mcsdl_delay_test(INT32 nCount)
{
	INT16 i;

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	//--------------------------------
	//	Repeating 'nCount' times
	//--------------------------------

	MCSDL_SET_GPIO_I2C();
	MCSDL_GPIO_SCL_SET_OUTPUT();
	MCSDL_GPIO_SDA_SET_OUTPUT();
	MCSDL_RESETB_SET_OUTPUT();

	MCSDL_GPIO_SCL_SET_HIGH();

	for( i=0; i<nCount; i++ ){

		#if 1

		MCSDL_GPIO_SCL_SET_LOW();

		mcsdl_delay(MCSDL_DELAY_20US);

		MCSDL_GPIO_SCL_SET_HIGH();

		mcsdl_delay(MCSDL_DELAY_100US);

		#elif 0

		MCSDL_GPIO_SCL_SET_LOW();

	   	mcsdl_delay(MCSDL_DELAY_500US);

		MCSDL_GPIO_SCL_SET_HIGH();

    	mcsdl_delay(MCSDL_DELAY_1MS);

		#else

		MCSDL_GPIO_SCL_SET_LOW();

    	mcsdl_delay(MCSDL_DELAY_25MS);

		TKEY_INTR_SET_LOW();

    	mcsdl_delay(MCSDL_DELAY_45MS);

		TKEY_INTR_SET_HIGH();

    	#endif
	}

	MCSDL_GPIO_SCL_SET_HIGH();

	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer
}
#endif
