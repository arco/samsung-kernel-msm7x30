#ifndef __LINUX_I2C_YDA165_H
#define __LINUX_I2C_YDA165_H

/* user setting */
/****************************************************************/
#define HP_HIZ_ON				/* HP Hi-Z is on */
/* #define SP_HIZ_ON */			/* SP Hi-Z is on */

#define D4HP3_OFFSEQUENCE_WAITTIME	30	/* "power off" sequence wait time [msec] */
/****************************************************************/

/* user implementation function */
/****************************************************************/
signed long d4Write(unsigned char bWriteRN, unsigned char bWritePrm);									/* write register function */
signed long d4WriteN(unsigned char bWriteRN, unsigned char *pbWritePrm, unsigned char dWriteSize);	/* write register function */
signed long d4Read(unsigned char bReadRN, unsigned char *pbReadPrm);		/* read register function */
void d4Wait(unsigned long dTime);				/* wait function */
void d4Sleep(unsigned long dTime);			/* sleep function */
void d4ErrHandler(signed long dError);		/* error handler function */
/****************************************************************/

/********************************************************************************************/
/* 
	Register Name
	bit 31 - 16	: register address
	bit 15 -  8	: mask bit
	bit  7 -  0	: shift bit
*/

/* #0 */
#define D4HP3_SRST		0x808007
#define D4HP3_CPMOD		0x804006
#define D4HP3_VLEVEL	0x800100

/* #1 */
#define D4HP3_ECO_MODE	0x814006
#define D4HP3_DIFA		0x812005
#define D4HP3_DIFB		0x811004
#define D4HP3_HIZ_HP	0x810803
#define D4HP3_HIZ_SP	0x810402
#define D4HP3_HP_GAIN	0x810300

/* #2 */
#define D4HP3_DPLT		0x82F004
#define D4HP3_NG_ATRT	0x820C02
#define D4HP3_DATRT		0x820300

/* #3 */
#define D4HP3_NG_RATIO	0x83E005
#define D4HP3_DALC		0x831C02
#define D4HP3_SP_GAIN	0x830300

/* #4 */
#define D4HP3_VA		0x847004
#define D4HP3_VB		0x840700

/* #5 */
#define D4HP3_SPSVOFF	0x858007
#define D4HP3_SPZCSOFF	0x854006
#define D4HP3_SPATT		0x851F00

/* #6 */
#define D4HP3_HPSVOFF	0x868007
#define D4HP3_HPZCSOFF	0x864006
#define D4HP3_HPATT		0x861F00

/* #7 */
#define D4HP3_SP_AMIX	0x872005
#define D4HP3_SP_BMIX	0x871004
#define D4HP3_HP_MONO	0x870803
#define D4HP3_HP_AMIX	0x870201
#define D4HP3_HP_BMIX	0x870100

/* #8 */
#define D4HP3_OCP_ERR	0x888007
#define D4HP3_OTP_ERR	0x884006
#define D4HP3_DC_ERR	0x882005
/********************************************************************************************/

/* return value */
#define D4HP3_SUCCESS			0
#define D4HP3_ERROR				-1
#define D4HP3_ERROR_ARGUMENT	-2

/* D-4HP3 value */
#define D4HP3_MIN_REGISTERADDRESS			0x80
#define D4HP3_MAX_WRITE_REGISTERADDRESS		0x87
#define D4HP3_MAX_READ_REGISTERADDRESS		0x88

/* type */
#define SINT32 signed long
#define UINT32 unsigned long
#define SINT8 signed char
#define UINT8 unsigned char

/* structure */
/********************************************************************************************/
/* D-4HP3 setting information */
typedef struct {
	/* input */
	unsigned char bLine1Gain;		/* LINE1 Gain Amp */
	unsigned char bLine2Gain;		/* LINE2 Gain Amp */

	unsigned char bLine1Balance;	/* LINE1 Single-ended(0) or Differential(1) */
	unsigned char bLine2Balance;	/* LINE2 Single-ended(0) or Differential(1) */

	/* HP */
	unsigned char bHpCpMode;			/* HP charge pump mode setting, 3stage mode(0) / 2stage mode(1) */
	unsigned char bHpAvddLev;			/* HP charge pump AVDD level, 1.65V<=AVDD<2.40V(0) / 2.40V<=AVDD<=2.86V(1) */
	unsigned char bHpEco;				/* HP eco mode, normal(0) / eco mode(1) */
	unsigned char bHpAtt;				/* HP attenuator */
	unsigned char bHpGainUp;			/* HP gain up */
	unsigned char bHpSvol;				/* HP soft volume setting, on(0) / off(1) */
	unsigned char bHpZcs;				/* HP zero cross mute setting, on(0) / off(1) */
	unsigned char bHpCh;				/* HP channel, stereo(0)/mono(1) */
	unsigned char bHpMixer_Line1;		/* HP mixer LINE1 setting */
	unsigned char bHpMixer_Line2;		/* HP mixer LINE2 setting */

	/* SP */
	unsigned char bSpAtt;				/* SP attenuator */
	unsigned char bSpGainUp;			/* SP gain up */
	unsigned char bSpSvol;				/* SP soft volume setting, on(0) / off(1) */
	unsigned char bSpZcs;				/* SP zero cross mute setting, on(0) / off(1) */
	unsigned char bSpMixer_Line1;		/* SP mixer LINE1 setting */
	unsigned char bSpMixer_Line2;		/* SP mixer LINE2 setting */
	unsigned char bSpNg_DetectionLv;	/* SP Noise Gate : detection level */
	unsigned char bSpNg_AttackTime;		/* SP Noise Gate : attack time */
	unsigned char bSpNcpl_NonClipRatio;	/* SP Non-Clip power limiter : Non-Clip distortion ratio */
	unsigned char bSpNcpl_PowerLimit;	/* SP Non-Clip power limiter : Power Limit */
	unsigned char bSpNcpl_AttackTime;	/* SP Non-Clip power limiter : attack Time */
	unsigned char bSpNcpl_ReleaseTime;	/* SP Non-Clip power limiter : release Time */
} D4HP3_SETTING_INFO;
/********************************************************************************************/

/* D-4HP3 Control module API */
/********************************************************************************************/
void D4Hp3_PowerOn(D4HP3_SETTING_INFO *pstSettingInfo);		/* power on function */
void D4Hp3_PowerOff(void);									/* power off function */
void D4Hp3_ControlMixer(unsigned char bHpFlag, unsigned char bSpFlag, D4HP3_SETTING_INFO *pstSetMixer);	/* control mixer function */
void D4Hp3_WriteRegisterBit(unsigned long bName, unsigned char bPrm);		/* 1bit write register function */
void D4Hp3_WriteRegisterByte(unsigned char bAddress, unsigned char bPrm);	/* 1byte write register function */
void D4Hp3_WriteRegisterByteN(unsigned char bAddress, unsigned char *pbPrm, unsigned char bPrmSize);	/* N byte write register function */
void D4Hp3_ReadRegisterBit(unsigned long bName, unsigned char *pbPrm);		/* 1bit read register function */
void D4Hp3_ReadRegisterByte(unsigned char bAddress, unsigned char *pbPrm);	/* 1byte read register function */
/********************************************************************************************/

struct snd_set_ampgain {
	int in1_gain;
	int in2_gain;
	int hp_att;
	int hp_gainup;
	int sp_att;
	int sp_gainup;
};

struct yda165_i2c_data {
	struct snd_set_ampgain *ampgain;
	int num_modes;
};

void yda165_speaker_onoff(int onoff); /* speaker path amp onoff */
void yda165_speaker_call_onoff(int onoff); /* speaker_call path amp onoff */
void yda165_headset_onoff(int onoff); /* headset path amp onoff */
void yda165_headset_call_onoff(int onoff); /* headset_call path amp onoff */
void yda165_speaker_headset_onoff(int onoff); /* speaker+headset path amp onoff */
void yda165_tty_onoff(int onoff); 				/* tty path amp onoff */

#define SND_IOCTL_MAGIC 's'
#define SND_SET_AMPGAIN _IOW(SND_IOCTL_MAGIC, 2, int mode)

#endif

