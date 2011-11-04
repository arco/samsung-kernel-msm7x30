
#define MAX_ABS_X  240
#define MAX_ABS_Y  400

//#define TOUCH_EN	16

#define TOUCH_INT	19
#define TOUCH_I2C_SCL_F 30
#define TOUCH_I2C_SDA_F 29

#define CONFIG_MELFAS_TOUCH_VERSION33

#define FEATURE_MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_NOT_MATCH

#define FEATURE_MELFAS_ENABLE_DBG_PRINT
//=====================================================================
//
//   MELFAS Firmware download
//
//=====================================================================

#define MELFAS_TRANSFER_LENGTH        64	    // Program & Read flash block size

//-----------------------------------------------
//	MELFAS Version information address
//-----------------------------------------------
#define MCSDL_ADDR_MODULE_REVISION    0x98
#define MCSDL_ADDR_FIRMWARE_VERSION   0x9C

#ifdef MELFAS_ENABLE_DOWNLOAD_ENABLE_COMMAND
//-----------------------------------------------
//	Command address of requesting enable reset. 
//-----------------------------------------------
#define MCSDL_ADDR_ENABLE_MODULE_RESET          0xED

//-----------------------------------------------
//	Command of requesting enable reset. 
//-----------------------------------------------
#define MCSDL_CMD_ENABLE_MODULE_RESET           0xED
#endif

//----------------------------------------------------
//   Return values of download function
//----------------------------------------------------
#define MCSDL_RET_SUCCESS						0x00
#define MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED	0x01
#define MCSDL_RET_ERASE_FLASH_FAILED			0x02
#define MCSDL_RET_ERASE_VERIFY_FAILED			0x03
#define MCSDL_RET_READ_FLASH_FAILED				0x04
#define MCSDL_RET_READ_EEPROM_FAILED			0x05
#define MCSDL_RET_READ_INFORMAION_FAILED		0x06
#define MCSDL_RET_PROGRAM_FLASH_FAILED			0x07
#define MCSDL_RET_PROGRAM_EEPROM_FAILED			0x08
#define MCSDL_RET_PROGRAM_INFORMAION_FAILED		0x09
#define MCSDL_RET_PROGRAM_VERIFY_FAILED			0x0A

#define MCSDL_RET_WRONG_MODE_ERROR				0xF0
#define MCSDL_RET_WRONG_SLAVE_SELECTION_ERROR	0xF1
#define MCSDL_RET_WRONG_PARAMETER				0xF2
#define MCSDL_RET_COMMUNICATION_FAILED			0xF3
#define MCSDL_RET_READING_HEXFILE_FAILED		0xF4
#define MCSDL_RET_FILE_ACCESS_FAILED			0xF5
#define MCSDL_RET_MELLOC_FAILED					0xF6
#define MCSDL_RET_WRONG_MODULE_REVISION			0xF7
#define MCSDL_RET_SAME_FIRMWARE_VERSION		    0xF8  
#define MCSDL_RET_NOT_SUPPORT_HW_VERSION		0xF9  
//-----------------------------------------------
//	MELFAS Firmware source type
//-----------------------------------------------
//#define MELFAS_DOWNLOAD_TYPE_BINARY 0x01

//------------------------------
// MDS ISP mode entering
//------------------------------
#define MCSDL_MDS_ENTERING_ISP_MODE_CODE2		0x00

#define MCSDL_MDS_ENTERING_ISP_MODE_ACK_1		0x55
#define MCSDL_MDS_ENTERING_ISP_MODE_ACK_2		0x80

//------------------------------
// ISP commands - MDS & I2C
//------------------------------
#define MCSDL_ISP_CMD_ERASE	                	0x02
#define MCSDL_ISP_CMD_PROGRAM_FLASH	        	0x03
#define MCSDL_ISP_CMD_READ_FLASH	        	0x04
#define MCSDL_ISP_CMD_PROGRAM_INFORMATION		0x05
#define MCSDL_ISP_CMD_READ_INFORMATION	    	0x06
#define MCSDL_ISP_CMD_RESET	                	0x07

//------------------------------
// MCS5000's responses
//------------------------------
#define MCSDL_ISP_ACK_ERASE_DONE_2ND_MDS		0x81
#define MCSDL_ISP_ACK_ERASE_DONE	        	0x82
#define MCSDL_I2C_ACK_PROGRAM_INFORMATION		0x85
#define MCSDL_MDS_ACK_PROGRAM_FLASH	        	0x83
#define MCSDL_MDS_ACK_READ_FLASH	        	0x84
#define MCSDL_MDS_ACK_PROGRAM_INFORMATION		0x88
#define MCSDL_MDS_ACK_PROGRAM_LOCKED	    	0xFE
#define MCSDL_MDS_ACK_READ_LOCKED	        	0xFE
#define MCSDL_MDS_ACK_FAIL	                	0xFE

//------------------------------
//	I2C ISP
//------------------------------

#define MCSDL_I2C_SLAVE_ADDR_ORG                0x20                            
#define MCSDL_I2C_SLAVE_ADDR_DN	            	0x7F	                        /* Down load Address - 7bit*/
#define MCSDL_I2C_SLAVE_ADDR_DN_SHIFTED	        (MCSDL_I2C_SLAVE_ADDR_DN<<1)    // Adress after sifting.

#define MCSDL_I2C_SLAVE_READY_STATUS	    	0x55

#define MCSDL_ISP_PROGRAM_TIMING_VALUE	    	0x78
#define MCSDL_ISP_PROGRAM_TIMING_VALUE_0		0xC0
#define MCSDL_ISP_PROGRAM_TIMING_VALUE_1		0xD4
#define MCSDL_ISP_PROGRAM_TIMING_VALUE_2		0x01

#if 1 // KHG_0619
//-----------------------------
// Program Finish-Mark
//-----------------------------
#define MCSDL_I2C_ADDRESS_FINISH_MARK 0xF000//At 60Kbyte
#define MCSDL_I2C_DATA_FINISH_MARK 0x55
#endif


//----------------------------------------------------
//	Functions
//----------------------------------------------------
int  melfas_ts_download(const uint8_t *pData, const uint16_t nLength);


// MELFAS HEX Studio v0.5 [2008.12.11]

#if defined(CONFIG_MELFAS_TOUCH_VERSION3)
#include "melfas_fw_binary/5S_MTH_SCAPELA_RA10_VA03_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION4)
#include "melfas_fw_binary/5S_MTH_SCAPELA_RA10_VA03_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION5)
#include "melfas_fw_binary/5S_MTH_SI7500P_RB10_VA05_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION6)
#include "melfas_fw_binary/5S_MTH_SI7500P_RB10_VA06_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION8)
#include "melfas_fw_binary/5S_MTH_SI7500P_RB10_VA08_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION9)
#include "melfas_fw_binary/5S_MTH_SI7500P_RB10_VA09_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION13)	// temperature added
#include "melfas_fw_binary/5S_MTH_SI7500P_RE40_VA13_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION15)	// ESD added
#include "melfas_fw_binary/5S_MTH_SI7500P_RE50_VA15_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION17)	// more ESD added
#include "melfas_fw_binary/5S_MTH_SI7500P_RE50_VA17_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION18)	
// ESD latch-up protect code added
// touch cordination improved (more accurate qwerty keypad)
// ignore side 5 pixel, because of Android cannot display that.
#include "melfas_fw_binary/5S_MTH_SI7500P_RE60_VA18_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION19)
#include "melfas_fw_binary/5S_MTH_SI7500P_RE60_VA19_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION22)
#include "melfas_fw_binary/5S_MTH_SI7500P_RG60_VA22_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION26)
#include "melfas_fw_binary/5S_MTH_SI7500P_RI81_VA26_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION30)
#include "melfas_fw_binary/5S_MTH_SI7500P_RKA1_VA30_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION31)
#include "melfas_fw_binary/5S_MTH_SI7500P_RKA1_VA31_bin.c"
#elif defined(CONFIG_MELFAS_TOUCH_VERSION33)
//hojun_kim #include "melfas_fw_binary/5S_MTH_SI7500P_RKA1_VA33_bin.c"
#elif defined(CONFIG_MELFAS_T939_R10_V06)
#include "melfas_fw_binary/MTH_T939_R10_V06_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V09)
#include "melfas_fw_binary/MTH_T939_R12_V09_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V10)
#include "melfas_fw_binary/MTH_T939_R12_V10_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V11)
#include "melfas_fw_binary/MTH_T939_R12_V11_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V12)
#include "melfas_fw_binary/MTH_T939_R12_V12_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V13)
#include "melfas_fw_binary/MTH_T939_R12_V13_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V14)
#include "melfas_fw_binary/MTH_T939_R12_V14_bin.c"
#elif defined(CONFIG_MELFAS_T939_R12_V16)
#include "melfas_fw_binary/MTH_T939_R12_V16_bin.c"
#elif defined(CONFIG_MELFAS_T939_R20_V17)
#include "melfas_fw_binary/MTH_T939_R20_V17_bin.c"
#endif
