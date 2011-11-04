//--------------------------------------------------------
//
//
//	Melfas MCS7000 Series Download base v1.0 2010.04.05
//
// 
//--------------------------------------------------------
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <asm/gpio.h>
#include <asm/io.h>

#include "mcs7000_download.h"


//============================================================
//
//	Include MELFAS Binary code File ( ex> MELFAS_FIRM_bin.c)
//
//	Warning!!!!
//		Please, don't add binary.c file into project
//		Just #include here !!
//
//============================================================
//#include "MTH_SR720_RA30_VA75_bin.c"
//#include "MTH_SR720_RA21_VA57_bin.c"
//#include "MTH_SR720_RA21_VA56_bin.c"
//#include "MTH_SR720_RA21_VA55_bin.c"
//#include "MDH_R720_RA21_VA54_bin.c"
//#include "MTH_SR720_RA21_VA54_bin.c"
//#include "MTH_SR720_RA21_VA53_bin.c"
//#include "MTH_R720_R21_V52___bin.c"
//#include "MTH_SR720F_R03_V50_bin.c"
//#include "MTH_SR720_RA30_VA58_bin.c"
//#include "MTH_SR720_RA30_VA59_bin.c"
//#include "MDH_CHIEF_RA00_VA80_bin.c"
//#include "MTH_SR720_RA30_VA60_bin.c"
#include "MTH_SR720_RA42_VA60_bin.c"

#define ENABLE_LITTLE_ENDIAN


UINT8  ucVerifyBuffer[MELFAS_TRANSFER_LENGTH];		//	You may melloc *ucVerifyBuffer instead of this

//---------------------------------
//	Downloading functions
//---------------------------------
static int  mcsdl_download(const UINT8 *pData, const UINT16 nLength);

static void mcsdl_set_ready(void);
static void mcsdl_reboot_mcs(void);

static int  mcsdl_erase_flash(void);
static int  mcsdl_program_flash( UINT8 *pDataOriginal, UINT16 unLength );
static void mcsdl_program_flash_part( UINT8 *pData, UINT8 ucLength );

static int  mcsdl_verify_flash( UINT8 *pData, UINT16 nLength );

static void mcsdl_read_flash( UINT8 *pBuffer, UINT16 unLength);
static int  mcsdl_read_flash_from( UINT8 *pBuffer, UINT16 unStart_addr, UINT16 unLength);

static void mcsdl_select_isp_mode(UINT8 ucMode);
static void mcsdl_unselect_isp_mode(void);

static void mcsdl_read_32bits( UINT8 *pData );
static void mcsdl_write_bits(UINT32 wordData, int nBits);
static void mcsdl_scl_toggle_twice(void);


//---------------------------------
//	Delay functions
//---------------------------------
static void mcsdl_mcs7000_delay(UINT32 nCount);


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
//============================================================

int mcsdl_mcs7000_download_binary_data(void)
{
	int nRet;

	#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
	melfas_send_download_enable_command();
	mcsdl_mcs7000_delay(MCSDL_DELAY_100US);
	#endif

	MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
	MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

	//------------------------
	// Run Download
	//------------------------
	nRet = mcsdl_download( (const UINT8*) MELFAS_binary, (const UINT16)MELFAS_binary_nLength );

	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer

	return ( nRet == MCSDL_RET_SUCCESS );
}



int mcsdl_download_binary_file(void)
{
	int nRet;

	UINT8  *pBinary = NULL;
	UINT16 nBinary_length =0;

	//==================================================
	//
	//	1. Read '.bin file'
	//  2. *pBinary       : Binary data
	//	   nBinary_length : Firmware size
	//	3. Run mcsdl_download( pBinary, nBinary_length);
	//
	//==================================================

	#if 0

		// TO DO : File Process & Get file Size(== Binary size)
		//			This is just a simple sample

		FILE *fp;
		INT  nRead;

		//------------------------------
		// Open a file
		//------------------------------

		if( fopen( fp, "MELFAS_FIRMWARE.bin", "rb" ) == NULL ){
			return MCSDL_RET_FILE_ACCESS_FAILED;
		}

		//------------------------------
		// Get Binary Size
		//------------------------------

		fseek( fp, 0, SEEK_END );

		nBinary_length = (UINT16)ftell(fp);

		//------------------------------
		// Memory allocation
		//------------------------------

		pBinary = (UINT8*)malloc( (INT)nBinary_length );

		if( pBinary == NULL ){

			return MCSDL_RET_FILE_ACCESS_FAILED;
		}

		//------------------------------
		// Read binary file
		//------------------------------

		fseek( fp, 0, SEEK_SET );

		nRead = fread( pBinary, 1, (INT)nBinary_length, fp );		// Read binary file

		if( nRead != (INT)nBinary_length ){

			fclose(fp);												// Close file

			if( pBinary != NULL )										// free memory alloced.
				free(pBinary);

			return MCSDL_RET_FILE_ACCESS_FAILED;
		}

		//------------------------------
		// Close file
		//------------------------------

		fclose(fp);

	#endif

	if( pBinary != NULL && nBinary_length > 0 && nBinary_length < 32*1024 ){

		#if MELFAS_USE_PROTOCOL_COMMAND_FOR_DOWNLOAD
		melfas_send_download_enable_command();
		mcsdl_mcs7000_delay(MCSDL_DELAY_100US);
		#endif

		MELFAS_DISABLE_BASEBAND_ISR();					// Disable Baseband touch interrupt ISR.
		MELFAS_DISABLE_WATCHDOG_TIMER_RESET();			// Disable Baseband watchdog timer

		//------------------------
		// Run Download
		//------------------------
		nRet = mcsdl_download( (const UINT8*) MELFAS_binary, (const UINT16)MELFAS_binary_nLength );

		MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
		MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer

	}else{

		nRet = MCSDL_RET_WRONG_BINARY;
	}

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

static int mcsdl_download(const UINT8 *pBianry, const UINT16 unLength )
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

	nRet = mcsdl_erase_flash();

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;


	//---------------------------------
	// Program Flash
	//---------------------------------
	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Program   ");
	#endif

	nRet = mcsdl_program_flash( (UINT8*)pBianry, (UINT16)unLength );
	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;


	//---------------------------------
	// Verify flash
	//---------------------------------

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk(" > Verify    ");
	#endif

	nRet = mcsdl_verify_flash( (UINT8*)pBianry, (UINT16)unLength );

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;

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

static int mcsdl_erase_flash(void)
{
	int	  i;
	UINT8 readBuffer[32];

	//----------------------------------------
	//	Do erase
	//----------------------------------------
	mcsdl_select_isp_mode(ISP_MODE_ERASE_FLASH);
	mcsdl_unselect_isp_mode();


	//----------------------------------------
	//	Check 'erased well'
	//----------------------------------------

	mcsdl_read_flash_from(  readBuffer	  , 0x0000, 16 );
	mcsdl_read_flash_from( &readBuffer[16], 0x7FF0, 16 );
	
	// Compare with '0xFF'	
	for(i=0; i<32; i++){
		if( readBuffer[i] != 0xFF )
			return MCSDL_RET_ERASE_FLASH_VERIFY_FAILED;
	}
	return MCSDL_RET_SUCCESS;
}


static int mcsdl_program_flash( UINT8 *pDataOriginal, UINT16 unLength )
{
	int		i;

	UINT8	*pData;
	UINT8   ucLength;

	UINT16  addr;
	UINT32  header;

	addr   = 0;
	pData  = pDataOriginal;

	//--------------------------------------
	//	Select ISP Mode  [In System Programming Mode]
	//--------------------------------------

	mcsdl_select_isp_mode( ISP_MODE_SERIAL_WRITE );

	//---------------------------------------------
	//	Header
	//	Length[13ibts] <<17 ) | Address[13ibts]<<2
	//---------------------------------------------
	header = ( ((unLength/4+1)&0x1FFF) << 17 ) | ( (0x0000&0x1FFF) << 2 );

	// Write 32bits
	mcsdl_write_bits( header, 32 );
	
	//---------------------------------
	//	Repeat Writing
	//---------------------------------

	ucLength = MELFAS_TRANSFER_LENGTH;

	for( i = 0; i< (int)unLength; i+=(int)ucLength ){

		addr = (UINT16)i;

		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
		printk("#");
		#endif

		if( ( unLength - addr ) < MELFAS_TRANSFER_LENGTH ){
			ucLength  = (UINT8)(unLength - addr);
		}

		mcsdl_program_flash_part( pData, ucLength );

		pData  += ucLength;

	}

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk("\n");
	#endif

	mcsdl_unselect_isp_mode();

	return MCSDL_RET_SUCCESS;
}

static void mcsdl_program_flash_part( UINT8 *pData, UINT8 ucLength )
{
	int     i;
	UINT32	data;

	for( i=0; i< ucLength; i+=4 ){

		//---------------------------------
		//	Body
		//---------------------------------

#ifdef ENABLE_LITTLE_ENDIAN
				data  = (UINT32)pData[i+0] << 24;
				data |= (UINT32)pData[i+1] << 16;
				data |= (UINT32)pData[i+2] << 8;
				data |= (UINT32)pData[i+3] << 0;
#else  // BIG_ENDIAN
				data  = (UINT32)pData[i+0] <<  0;
				data |= (UINT32)pData[i+1] <<  8;
				data |= (UINT32)pData[i+2] << 16;
				data |= (UINT32)pData[i+3] << 24;
#endif

		
//		printk("\ndata 0x%08X, ", data);
		mcsdl_write_bits(data, 32);

		mcsdl_scl_toggle_twice();
	}

	//---------------------------------
	//	Intermission : Every 32 Words
	//---------------------------------
	if( i == MELFAS_TRANSFER_LENGTH ){

		mcsdl_scl_toggle_twice();
		mcsdl_scl_toggle_twice();

	}
}

static int mcsdl_verify_flash( UINT8 *pDataOriginal, UINT16 unLength )
{
	int	  i, j;
	int	  nRet;

	UINT8 *pData;
	UINT8 ucLength;

	UINT16 addr;
	UINT32 wordData;
	
	addr  = 0;
	pData = (UINT8 *) pDataOriginal;

	mcsdl_select_isp_mode(ISP_MODE_SERIAL_READ);

	wordData   = ( (addr&0x1FFF) << 7 ) | 0x7F;
	wordData <<= 1;

	mcsdl_write_bits( wordData, 31 );

	ucLength  = MELFAS_TRANSFER_LENGTH;

	for( i = 0; i < (int)unLength; i+=(int)ucLength ){

		addr = (UINT16)i;

		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT 
		printk("#");
		#endif

		if( ( unLength - addr ) < MELFAS_TRANSFER_LENGTH ){
			ucLength = (UINT8)(unLength - addr);
		}

		//--------------------
		// Read flash
		//--------------------

		mcsdl_read_flash( ucVerifyBuffer, ucLength );

		
		//--------------------
		// Comparing
		//--------------------

		for(j=0; j<(int)ucLength; j++){

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

	}

	nRet = MCSDL_RET_SUCCESS;

MCSDL_VERIFY_FLASH_FINISH:

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	printk("\n");
	#endif

	mcsdl_unselect_isp_mode();

	return nRet;
}


static void mcsdl_read_flash( UINT8 *pBuffer, UINT16 unLength)
{
	int i;

	for ( i=0; i< (int)unLength; i+=4 ){

		mcsdl_read_32bits( pBuffer );

		pBuffer += 4;
	}
}

static int mcsdl_read_flash_from( UINT8 *pBuffer, UINT16 unStart_addr, UINT16 unLength)
{
	int	  i;
	//int j;

	UINT8  ucLength;

	UINT16 addr;
	UINT32 wordData;

	if( unLength >= MELFAS_FIRMWARE_MAX_SIZE ){
		return MCSDL_RET_PROGRAM_SIZE_IS_WRONG;
	}

	addr  = 0;

	mcsdl_select_isp_mode(ISP_MODE_SERIAL_READ);

	wordData   = ( (unStart_addr&0x1FFF) << 7 ) | 0x7F;
	wordData <<= 1;

	mcsdl_write_bits( wordData, 31 );

	ucLength  = MELFAS_TRANSFER_LENGTH;

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	//printk(" %04X : ", unStart_addr );
	#endif


	for( i = 0; i < (int)unLength; i+=(int)ucLength ){

		addr = (UINT16)i;

		if( ( unLength - addr ) < MELFAS_TRANSFER_LENGTH ){

			ucLength = (UINT8)(unLength - addr);
		}

		//--------------------
		// Read flash
		//--------------------
		mcsdl_read_flash( &pBuffer[addr], ucLength );


		#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
		//for(j=0; j<(int)ucLength; j++){
		//	printk("%02X ", pBuffer[j] );
		//}
		#endif

	}

	#if MELFAS_ENABLE_DBG_PROGRESS_PRINT
	//printk("\n");
	#endif

	mcsdl_unselect_isp_mode();

	return MCSDL_RET_SUCCESS;

}


static void mcsdl_set_ready(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------

	MCSDL_VDD_SET_LOW();

    MCSDL_CE_SET_OUTPUT();
	MCSDL_CE_SET_LOW();

	MCSDL_SET_GPIO_I2C();

    MCSDL_GPIO_SDA_SET_LOW(); 
	MCSDL_GPIO_SDA_SET_OUTPUT();
	
	
   	MCSDL_GPIO_SCL_SET_LOW(); 
	MCSDL_GPIO_SCL_SET_OUTPUT();

    MCSDL_RESETB_SET_LOW();
	MCSDL_RESETB_SET_OUTPUT();
	
	mcsdl_mcs7000_delay(MCSDL_DELAY_25MS);						// Delay for Stable VDD

	MCSDL_VDD_SET_HIGH();
	MCSDL_CE_SET_HIGH();

	MCSDL_GPIO_SDA_SET_HIGH();

	mcsdl_mcs7000_delay(MCSDL_DELAY_30MS); 						// Delay '30 msec'

}


static void mcsdl_reboot_mcs(void)
{
	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------

	MCSDL_VDD_SET_LOW();

	MCSDL_CE_SET_LOW();
	MCSDL_CE_SET_OUTPUT();

	MCSDL_GPIO_SDA_SET_HIGH();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_GPIO_SCL_SET_HIGH();
	MCSDL_GPIO_SCL_SET_OUTPUT();

	MCSDL_SET_HW_I2C();

	MCSDL_RESETB_SET_LOW();
	MCSDL_RESETB_SET_OUTPUT();

	mcsdl_mcs7000_delay(MCSDL_DELAY_25MS);						// Delay for Stable VDD

	MCSDL_RESETB_SET_INPUT();
	MCSDL_VDD_SET_HIGH();
	MCSDL_CE_SET_HIGH();

	mcsdl_mcs7000_delay(MCSDL_DELAY_30MS); 						// Delay '25 msec'

}


//--------------------------------------------
//
//   Write ISP Mode entering signal
//
//--------------------------------------------

static void mcsdl_select_isp_mode(UINT8 ucMode)
{
	int    i;

	UINT8 enteringCodeMassErase[3]   = { 1, 1, 1 };
	UINT8 enteringCodeSerialWrite[3] = { 0, 0, 1 };
	UINT8 enteringCodeSerialRead[3]  = { 0, 1, 0 };

	UINT8 *pCode;


	//------------------------------------
	// Entering ISP mode : Part 1
	//------------------------------------

	for(i=0; i<4; i++){

		MCSDL_RESETB_SET_LOW();		mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);

		MCSDL_RESETB_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
   }

	MCSDL_RESETB_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
	MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
	MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);

	//------------------------------------
	// Entering ISP mode : Part 2
	//------------------------------------

		 if( ucMode == ISP_MODE_ERASE_FLASH  ) pCode = enteringCodeMassErase;
	else if( ucMode == ISP_MODE_SERIAL_WRITE ) pCode = enteringCodeSerialWrite;
	else if( ucMode == ISP_MODE_SERIAL_READ  ) pCode = enteringCodeSerialRead;

	for(i=0; i<3; i++){

		if( pCode[i] == 1 )	MCSDL_RESETB_SET_HIGH();
		else				MCSDL_RESETB_SET_LOW();

		mcsdl_mcs7000_delay(MCSDL_DELAY_3US);

		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
   }

	MCSDL_RESETB_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_3US);			// High


	//---------------------------------------------------
	// Entering ISP mode : Part 3	- Only Mass Erase
	//---------------------------------------------------

	 if( ucMode == ISP_MODE_ERASE_FLASH   ){

		for(i=0; i<6; i++){

				 if( i==3 ) mcsdl_mcs7000_delay(MCSDL_DELAY_25MS);
			else if( i==4 ) mcsdl_mcs7000_delay(MCSDL_DELAY_150US);

			MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_5US);
			MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_5US);
		}
	}
}


static void mcsdl_unselect_isp_mode(void)
{
	int i;

	MCSDL_GPIO_SDA_SET_HIGH();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_RESETB_SET_LOW();		mcsdl_mcs7000_delay(MCSDL_DELAY_3US);

	for(i=0; i<9; i++){

		MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_5US);
		MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_5US);
	}

	mcsdl_mcs7000_delay(MCSDL_DELAY_500US);
}



static void mcsdl_read_32bits( UINT8 *pData )
{
	int i, j;

	MCSDL_GPIO_SDA_SET_INPUT();
	

#ifdef ENABLE_LITTLE_ENDIAN
	for (i=0; i<=3; i++){
#else  // BIG_ENDIAN
	for (i=3; i>=0; i--){
#endif
		pData[i] = 0;

		for (j=0; j<8; j++){

			pData[i] <<= 1;

			MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_5US);

			if ( MCSDL_GPIO_SDA_IS_HIGH())
				pData[i] |= 0x01;

			MCSDL_GPIO_SCL_SET_LOW();		mcsdl_mcs7000_delay(MCSDL_DELAY_5US);

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

		mcsdl_mcs7000_delay(MCSDL_DELAY_3US);

		MCSDL_GPIO_SCL_SET_HIGH();		mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		MCSDL_GPIO_SCL_SET_LOW();		mcsdl_mcs7000_delay(MCSDL_DELAY_3US);
		
		wordData <<= 1;
	}
}


static void mcsdl_scl_toggle_twice(void)
{

	MCSDL_GPIO_SDA_SET_HIGH();
	MCSDL_GPIO_SDA_SET_OUTPUT();

	MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_15US);
	MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_15US);

	MCSDL_GPIO_SCL_SET_HIGH();	mcsdl_mcs7000_delay(MCSDL_DELAY_15US);
	MCSDL_GPIO_SCL_SET_LOW();	mcsdl_mcs7000_delay(MCSDL_DELAY_15US);
}


//============================================================
//
//	Delay Function
//
//============================================================
static void mcsdl_mcs7000_delay(UINT32 nCount)
{

		switch(nCount) 
	{
		case MCSDL_DELAY_2US :		
			udelay(2); 
			break;
		case MCSDL_DELAY_3US :		
			udelay(3); 
			break;
		case MCSDL_DELAY_5US :		
			udelay(5); 
			break;
		case MCSDL_DELAY_15US :		
			udelay(15); 
			break;
		case MCSDL_DELAY_100US :
			udelay(100); 
			break;
		case MCSDL_DELAY_150US :
			udelay(150);
			break;
		case MCSDL_DELAY_500US :
			udelay(500);
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
		case MCSDL_DELAY_45MS :
			msleep(45);
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
			case MCSDL_RET_PROGRAM_VERIFY_FAILED			:   printk("MCSDL_RET_PROGRAM_VERIFY_FAILED\n" );      	break;

			case MCSDL_RET_PROGRAM_SIZE_IS_WRONG			:   printk("MCSDL_RET_PROGRAM_SIZE_IS_WRONG\n" );    		break;
			case MCSDL_RET_VERIFY_SIZE_IS_WRONG				:   printk("MCSDL_RET_VERIFY_SIZE_IS_WRONG\n" );      		break;
			case MCSDL_RET_WRONG_BINARY						:   printk("MCSDL_RET_WRONG_BINARY\n" );      				break;

			case MCSDL_RET_READING_HEXFILE_FAILED       	:   printk("MCSDL_RET_READING_HEXFILE_FAILED\n" );			break;
			case MCSDL_RET_FILE_ACCESS_FAILED       		:   printk("MCSDL_RET_FILE_ACCESS_FAILED\n" );				break;
			case MCSDL_RET_MELLOC_FAILED     		  		:   printk("MCSDL_RET_MELLOC_FAILED\n" );      			break;

			case MCSDL_RET_WRONG_MODULE_REVISION     		:   printk("MCSDL_RET_WRONG_MODULE_REVISION\n" );      	break;

			default                             			:	printk("UNKNOWN ERROR. [0x%02X].\n", nRet );      		break;
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

void mcsdl_mcs7000_delay_test(INT32 nCount)
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

		mcsdl_mcs7000_delay(MCSDL_DELAY_15US);

		MCSDL_GPIO_SCL_SET_HIGH();

		mcsdl_mcs7000_delay(MCSDL_DELAY_100US);

		#elif 0

		MCSDL_GPIO_SCL_SET_LOW();

	   	mcsdl_mcs7000_delay(MCSDL_DELAY_500US);

		MCSDL_GPIO_SCL_SET_HIGH();

    	mcsdl_mcs7000_delay(MCSDL_DELAY_1MS);

		#else

		MCSDL_GPIO_SCL_SET_LOW();

    	mcsdl_mcs7000_delay(MCSDL_DELAY_25MS);

		TKEY_INTR_SET_LOW();

    	mcsdl_mcs7000_delay(MCSDL_DELAY_45MS);

		TKEY_INTR_SET_HIGH();

    	#endif
	}

	MCSDL_GPIO_SCL_SET_HIGH();

	MELFAS_ROLLBACK_BASEBAND_ISR();					// Roll-back Baseband touch interrupt ISR.
	MELFAS_ROLLBACK_WATCHDOG_TIMER_RESET();			// Roll-back Baseband watchdog timer
}


#endif



