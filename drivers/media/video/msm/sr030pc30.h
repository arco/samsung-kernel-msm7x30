/*
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */



#ifndef SR030PC30_H
#define SR030PC30_H

#include <mach/board.h> //PGH

#if 0
#if 0
//unuse standby pin #define CAM_STB						85
#define CAM_RST						17
#define CAM_ON						76//REV02
//#define CAM_ON						22//REV01
#define CAM_FLASH_EN				23
#define CAM_FLASH_SET				31
#endif
#include <linux/mfd/pmic8058.h>

#define	CAM_TYPE_8M	0
#define	CAM_TYPE_VGA	1

#if (CONFIG_BOARD_REVISION >= 0x01)
#define	CAM_8M_RST			PM8058_GPIO_PM_TO_SYS(25)	// must sub 1 - PM_GPIO_26
#define	CAM_VGA_RST			PM8058_GPIO_PM_TO_SYS(29)	//PM_GPIO_30
#else
#define	CAM_8M_RST			0
#define	CAM_VGA_RST		31
#endif
#define	CAM_MEGA_EN			1	// STEALTH REV03 below
#define	CAM_MEGA_EN_REV04	74	// STEALTH REV04 
#if (CONFIG_BOARD_REVISION >= 0x01)
#define	CAM_VGA_EN		PM8058_GPIO_PM_TO_SYS(24)	// PM_GPIO_25
#else
#define	CAM_VGA_EN		2
#endif
#define	CAM_PMIC_STBY			3	// PMIC EN (CAM_IO_EN)

#define	CAM_SENSOR_A_EN		PM8058_GPIO_PM_TO_SYS(39)	// STEALTH HW REV04 PM8058 GPIO 40

#define	ON		1
#define	OFF		0
#define LOW							0
#define HIGH							1
#endif

/* EV */
#define SR030PC30_EV_MINUS_4				-4
#define SR030PC30_EV_MINUS_3				-3
#define SR030PC30_EV_MINUS_2				-2
#define SR030PC30_EV_MINUS_1				-1
#define SR030PC30_EV_DEFAULT				0
#define SR030PC30_EV_PLUS_1					1
#define SR030PC30_EV_PLUS_2					2
#define SR030PC30_EV_PLUS_3					3
#define SR030PC30_EV_PLUS_4					4

/* DTP */
#define SR030PC30_DTP_OFF		0
#define SR030PC30_DTP_ON		1
#define SR030PC30_DTP_OFF_ACK		2
#define SR030PC30_DTP_ON_ACK		3

#define CAMERA_MODE			0
#define CAMCORDER_MODE		1


//////////////////////////////////////////////////////////////
//==========================================================//
// MODEL NO.:  SHW-A310S                              	     //
// SENSOR   :  SILICON-FILE                       //
// LENS     :                                 //
// MCLK     :  24.0MHZ                                      //
// FPS      :  Self-Camera 8-15fps // SKT VT-call  8fps   //
// DATA     :  2010:02:10
//==========================================================//
//////////////////////////////////////////////////////////////


//==========================================================
//	CAMERA INITIAL (VGA SETTING)
//==========================================================
const unsigned short sr030pc30_init_reg[] = {
{0x0300},
{0x01f1},
{0x01f3},
{0x01f1},

{0x0320},
{0x101c},
{0x0322},
{0x107b},

//Page0
{0x0300}, 
{0x0baa},
{0x0caa},
{0x0daa},
{0x1000},
{0x1190}, //On Var Frame - None : 90, X Flip : 91, Y Flip :
{0x1204},//PCLK Inversion
{0x2000},
{0x2106},
{0x2200},
{0x2306},
{0x2401},
{0x25e0},
{0x2602},
{0x2780},
{0x4001}, //Hblank 336
{0x4150},

{0x4201}, //Vblank 300
{0x432c},

//BLC  
{0x803e},
{0x8196},
{0x8290},
{0x8300},
{0x842c},

{0x900f}, //BLC_TIME_TH_ON
{0x910f}, //BLC_TIME_TH_OFF 
{0x9278}, //BLC_AG_TH_ON
{0x9370}, //BLC_AG_TH_OFF
{0x9488},
{0x9580},

{0x9820},
{0xa040},
{0xa241},
{0xa440},
{0xa641},
{0xa844},
{0xaa43},
{0xac43},
{0xae43},

//Page2
{0x0302},
{0x1000},
{0x1300},
{0x181C},
{0x1900},
{0x1a00},
{0x1b08},
{0x1C00},
{0x1D00},
{0x2033},
{0x21aa},
{0x22a6},
{0x23b0},
{0x3199},
{0x3200},
{0x3300},
{0x343C},
{0x5021},
{0x5430},
{0x56fe},
{0x6278},
{0x639E},
{0x6478},
{0x659E},
{0x727A},
{0x739A},
{0x747A},
{0x759A},
{0x8209},
{0x8409},
{0x8609},
{0xa003},
{0xa81D},
{0xaa49},
{0xb98A},
{0xbb8A},
{0xbc04},
{0xbd10},
{0xbe04},
{0xbf10},

//Page10
{0x0310},//
{0x1001},//ISPCTL1
{0x1230},//Y offet, dy offseet enable
{0x4080},
{0x4108},//
{0x5078},//

{0x601f},
{0x6182},
{0x6282},
{0x63c0},
{0x6480},

//Page11
{0x0311},
{0x1099},
{0x110e},
{0x2129},
{0x5005},
{0x600f},
{0x6243},
{0x6363},
{0x7408},
{0x7508},

//Page12
{0x0312},
{0x4023},
{0x413b},
{0x5005},
{0x701d},
{0x7405},
{0x7504},
{0x7720},
{0x7810},
{0x9134},
{0xb0c9},
{0xd0b1},

//Page13
{0x0313},
{0x103b},
{0x1103},
{0x1200},
{0x1302},
{0x1418},

{0x2002},
{0x2101},
{0x2324},
{0x2406},
{0x254a},
{0x2800},
{0x2978},
{0x30ff},
{0x800D},
{0x8113},

{0x835d},

{0x9003},
{0x9102},
{0x933d},
{0x9403},
{0x958f},

//Page14

{0x0314},
{0x1001},
{0x2080},
{0x2178},
{0x2261},
{0x233b},
{0x2433},

//Page15 CmC
{0x0315},
{0x1003},

{0x1449},
{0x1640},
{0x172f},

{0x30cb},
{0x3161},
{0x3216},
{0x3323},
{0x34ce},
{0x352b},
{0x3603},
{0x3737},
{0x387a},

{0x4002},
{0x4114},
{0x4296},
{0x43a4},
{0x4400},
{0x4524},
{0x4604},
{0x4782},
{0x4882},

{0x0316},
{0x1001},
{0x3000},
{0x3109},
{0x321b},
{0x3335},
{0x345d},
{0x357a},
{0x3693},
{0x37a7},
{0x38b8},
{0x39c6},
{0x3ad2},
{0x3be4},
{0x3cf1},
{0x3df9},
{0x3eff},


//Page20 AE
{0x0320},
{0x100c},
{0x1104},

{0x2001},
{0x283f},
{0x29a3},
{0x2af0},
{0x2bf4},

{0x30f8},

{0x60d5},

{0x6834},
{0x696e},
{0x6A28},
{0x6Bc8},


{0x7034},//Y Target

{0x7812},//Yth 1
{0x7911},//Yth 2
{0x7A23},

{0x8300}, //EXP Normal 33.33 fps 
{0x84af}, 
{0x85c8}, 
{0x8600}, //EXPMin 6000.00 fps
{0x87fa}, 
{0x8802}, //EXP Max 10.00 fps 
{0x8949}, 
{0x8af0}, 
{0x8B3a}, //EXP100 
{0x8C98}, 
{0x8D30}, //EXP120 
{0x8Ed4}, 

{0x8f04},
{0x9093},

{0x9102},
{0x92d2},
{0x93a8},

{0x988C},
{0x9923},

{0x9c06}, //EXP Limit 857.14 fps 
{0x9dd6}, 
{0x9e00}, //EXP Unit 
{0x9ffa}, 

{0xb014},
{0xb114},
{0xb2d0},
{0xb314},
{0xb41c},
{0xb548},
{0xb632},
{0xb72b},
{0xb827},
{0xb925},
{0xba23},
{0xbb22},
{0xbc22},
{0xbd21},

{0xc014},

{0xc870},
{0xc980},

//Page22 AWB
{0x0322},
{0x10e2},
{0x112e},
{0x2104},

{0x3080},
{0x3180},
{0x3811},
{0x3933},
{0x40f0},
{0x4133},
{0x4233},
{0x43f3},
{0x4455},
{0x4544},
{0x4602},

{0x50d0},
{0x51a0},
{0x52aa},

{0x8045},
{0x8120},
{0x8248},

{0x8354},//54
{0x8420},//22 29 27
{0x8558},//54 53 50
{0x8620},//22

{0x874c},
{0x8834},
{0x8930},
{0x8a22},

{0x8b00},
{0x8d22},
{0x8e71},

{0x8f63},
{0x9060},
{0x915c},
{0x9259},
{0x9355},
{0x9450},
{0x9548},
{0x963e},
{0x9737},
{0x9830},
{0x9929},
{0x9a26},
{0x9b09},

{0x0322},
{0x10fb},

{0x0320},
{0x109c},

{0x0300},
{0x01f0},

};

//==========================================================
//	CAMERA INITIAL for VT Preview  (VGA SETTING)
//==========================================================
const unsigned short sr030pc30_init_vt_reg[] = {
{0x0300},
{0x01f1},
{0x01f3},
{0x01f1},

{0x0320}, //page 3
{0x101c}, //ae off
{0x0322}, //page 4
{0x107b}, //awb off

//Page0
{0x0300},
{0x1000},
{0x1194}, //On Fix Frame - None : 94, X Flip : 95, Y Flip : 96, XY : 97
{0x1204},
{0x2000},
{0x2106},
{0x2200},
{0x2306},
{0x2401},
{0x25e0},
{0x2602},
{0x2780},
{0x4001}, //Hblank 336
{0x4150},

{0x4200}, //Vblank 20
{0x4314},

//BLC
{0x803e},
{0x8196},
{0x8290},
{0x8300},
{0x842c},

{0x900e},
{0x910f},
{0x923a},
{0x9330},
{0x9488},
{0x9580},

{0x9820},
{0xa040},
{0xa241},
{0xa440},
{0xa641},
{0xa844},
{0xaa43},
{0xac44},
{0xae43},

//Page2

{0x0302}, //
{0x1000}, //
{0x1300}, //
{0x181C}, //
{0x1900}, //
{0x1a00}, //
{0x1b08}, //
{0x1C00}, //
{0x1D00}, //
{0x2033}, //
{0x21aa}, //
{0x22a6}, //
{0x23b0}, //
{0x3199}, //
{0x3200}, //
{0x3300}, //
{0x343C}, //
{0x5021}, //
{0x5430}, //
{0x56fe}, //
{0x6278}, //
{0x639E}, //
{0x6478}, //
{0x659E}, //
{0x727A}, //
{0x739A}, //
{0x747A}, //
{0x759A}, //
{0x8209}, // 
{0x8409}, // 
{0x8609}, // 
{0xa003}, //
{0xa81D}, //
{0xaa49}, //
{0xb98A}, // 
{0xbb8A}, // 
{0xbc04}, // 
{0xbd10}, // 
{0xbe04}, // 
{0xbf10}, //

//Page10
{0x0310},//
{0x1001},//ISPCTL1
{0x1230},//Y offet, dy offseet enable
{0x4080},
{0x4122},//24->22
{0x50a5},//

{0x601f},
{0x6180},
{0x6280},
{0x63f0},
{0x6480},

//Page11
{0x0311},
{0x1099},
{0x110e},
{0x2129},
{0X5005},
{0x600f},
{0x6243},
{0x6363},
{0x7408},
{0x7508},

//Page12
{0x0312},
{0x4023},
{0x413b},
{0x5005},
{0x701d},
{0x7405},
{0x7504},
{0x7720},
{0x7810},
{0x9134},
{0xb0c9},
{0xd0b1},

//Page13
{0x0313},
{0x103b},
{0x1103},
{0x1200},
{0x1302},
{0x1418},

{0x2002},
{0x2101},
{0x2324},
{0x2406},
{0x254a},
{0x2800},
{0x2978},
{0x30ff},
{0x800D},
{0x8113},

{0x835d},

{0x9003},
{0x9102},
{0x933d},
{0x9403},
{0x958f},

//Page14
{0x0314},
{0x1001},
{0x2080},
{0x2178},
{0x2261},
{0x233b},
{0x2433},

//Page 15 CmC
//Page15 CmC
{0x0315},
{0x1003},

{0x1449},
{0x1640},
{0x172f},

{0x30cb},
{0x3161},
{0x3216},
{0x3323},
{0x34ce},
{0x352b},
{0x3603},
{0x3737},
{0x387a},

{0x4002},
{0x4114},
{0x4296},
{0x43a4},
{0x4400},
{0x4524},
{0x4604},
{0x4782},
{0x4882},

{0x0316},
{0x1001},
{0x3000},
{0x3114},
{0x3223},
{0x333b},
{0x345d},
{0x3579},
{0x368e},
{0x379f},
{0x38af},
{0x39bd},
{0x3aca},
{0x3bdd},
{0x3cec},
{0x3df7},
{0x3eff},


//Page20 AE
{0x0320},
{0x100c},
{0x1104},

{0x2001},
{0x283f},
{0x29a3},
{0x2a93},
{0x2bf5},

{0x30f8},

{0x6095},

{0x6834},
{0x696e},
{0x6A28},
{0x6Bc8},

{0x7042}, //Y LVL

{0x7812}, //yth1
{0x7916}, //yth2
{0x7A23}, //yth3

{0x8300}, //EXP Normal 33.33 fps 
{0x84af}, 
{0x85c8}, 
{0x8600}, //EXPMin 6000.00 fps
{0x87fa}, 
{0x8802}, //EXP Max 10.00 fps 
{0x8949}, 
{0x8af0},  
{0x8B3a}, //EXP100 
{0x8C98}, 
{0x8D30}, //EXP120 
{0x8Ed4}, 

{0x8f04},
{0x9093},

{0x9103}, //EXP Fix 7.00 fps
{0x923b}, 
{0x9326}, 

{0x988C},
{0x9923},

{0x9c06}, //EXP Limit 857.14 fps 
{0x9dd6}, 
{0x9e00}, //EXP Unit 
{0x9ffa}, 


{0xb014}, //
{0xb114}, // 08->14
{0xb2e0}, // a0->f0->e0
{0xb314}, //
{0xb414}, //
{0xb538}, //
{0xb626}, //
{0xb720}, //
{0xb81d}, //
{0xb91b}, //
{0xba1a}, //
{0xbb19}, //
{0xbc19}, //
{0xbd18}, //

{0xc014},

{0xc870},
{0xc980},

//Page 22 AWB
{0x0322}, //
{0x10e2},
{0x112e}, // AD CMC off
{0x2140}, //

{0x3080}, //
{0x3180}, //
{0x3811}, //
{0x3933},
{0x40f0}, //
{0x4133},
{0x4233},
{0x43f3}, //
{0x4455},
{0x4544},
{0x4602}, //

{0x8040}, //
{0x8120}, //
{0x8243}, //

{0x8366}, // RMAX Default : 50 
{0x841f}, // RMIN Default : 20
{0x8561}, // BMAX Default : 50
{0x8620}, // BMIN Default : 20

{0x8750}, // RMAXB Default : 50 
{0x8845}, // RMINB Default : 3e
{0x892d}, // BMAXB Default : 2e
{0x8a22}, // BMINB Default : 20

{0x8b00},
{0x8d21},
{0x8e71},

{0x8f63},
{0x9060},
{0x915c},
{0x9259},
{0x9355},
{0x9450},
{0x9548},
{0x963e},
{0x9737},
{0x9830},
{0x9929},
{0x9a26},
{0x9b09},

{0x0322},
{0x10fb},

{0x0320},
{0x109c},

{0x01f0},


};

#define SR030PC30_INIT_REGS	(sizeof(sr030pc30_init_reg) / sizeof(sr030pc30_init_reg[0]))
#define SR030PC30_INIT_VT_REGS	(sizeof(sr030pc30_init_vt_reg) / sizeof(sr030pc30_init_vt_reg[0]))

/************** Exposure Value Setting ****************/
/*
 * EV bias
 */
//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (1/9)  : ¹à±â 
//==========================================================
	/* unused variables */
const unsigned short sr030pc30_ev_m5[] = {	
 {0x0310},
 {0x40d0},
};
const unsigned short sr030pc30_ev_m4[] = {
 0x0310,
 0x40c0,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (2/9)
//==========================================================
const unsigned short sr030pc30_ev_m3[] = {
 0x0310,
 0x40b0,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (3/9)
//==========================================================
const unsigned short sr030pc30_ev_m2[] = {
 0x0310,
 0x40a0,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (4/9)
//==========================================================
const unsigned short sr030pc30_ev_m1[] = {
 0x0310,
 0x4090,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (5/9)
//==========================================================
const unsigned short sr030pc30_ev_default[] = { 
0x0310,
0x4080,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (6/9)
//==========================================================
const unsigned short sr030pc30_ev_p1[] = { 
0x0310,
0x4010,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (7/9)
//==========================================================
const unsigned short sr030pc30_ev_p2[] = {
0x0310,
0x4020,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (8/9)
//==========================================================
const unsigned short sr030pc30_ev_p3[] = {
0x0310,
0x4030,
};

//==========================================================
//	CAMERA_BRIGHTNESS_LEVEL1 (9/9)
//==========================================================
const unsigned short sr030pc30_ev_p4[] = {
0x0310,
0x4040,
};

/*
 * EV bias for VT
 */
const unsigned short sr030pc30_ev_vt_m4[] = {
};

const unsigned short sr030pc30_ev_vt_m3[] = {
};

const unsigned short sr030pc30_ev_vt_m2[] = {
};

const unsigned short sr030pc30_ev_vt_m1[] = {
};

const unsigned short sr030pc30_ev_vt_default[] = {
};

const unsigned short sr030pc30_ev_vt_p1[] = {
};

const unsigned short sr030pc30_ev_vt_p2[] = {
};

const unsigned short sr030pc30_ev_vt_p3[] = {
};

const unsigned short sr030pc30_ev_vt_p4[] = {
};
const unsigned short sr030pc30_ev_p5[] = 	{
{0x0310},
{0x4050},
};


/************** White Balance Setting ******************/
//==========================================================
//	CAMERA_WB_AUTO  (1/5)
//==========================================================
const unsigned short sr030pc30_wb_auto[] = {
0x0322,
0x10e2,
0x1126,
0x8045, 
0x8120, 
0x8248, 
0x8366, //66 RMAX Default : 50 
0x8426, //28 26 RMIN Default : 20
0x8555, //58 5a BMAX Default : 50
0x8620, //20 BMIN Default : 20
0x10fb,
};

//==========================================================
//	CAMERA_WB_DAYLIGHT  (2/5)
//==========================================================
const unsigned short sr030pc30_wb_sunny[] = { 
0x0322,
0x107b,
0x1126,
0x8052, 
0x8120, 
0x8230, 
0x8353, 
0x8448, 
0x8535, 
0x862b, 
0x10fb,
0xffff,
};

//==========================================================
//	CAMERA_WB_CLOUDY  (3/5)
//==========================================================
const unsigned short sr030pc30_wb_cloudy[] = {
0x0322,
0x107b,
0x1126,
0x8052,
0x8120,
0x8230,
0x837f, //7f 7d
0x8470, //7d 7d 7f RMIN
0x851f, //1c 1e 1a 21 BMAX
0x8610, //10 1e 
0x10fb,
};

//==========================================================
//	CAMERA_WB_INCANDESCENT  (4/5)
//==========================================================
const unsigned short sr030pc30_wb_tungsten[] = 	{
0x0322,
0x107b,
0x1126,
0x8052, 
0x8120, 
0x8230, 
0x8353, 
0x8448, 
0x8535, 
0x862b, 
0x10fb,
};

//==========================================================
//	CAMERA_WB_FLUORESCENT  (5/5)
//==========================================================
const unsigned short sr030pc30_wb_fluorescent[] = { 
0x0322,
0x107b,
0x1126,
0x8042,
0x8120,
0x8251,
0x834a, //4c 4e RMAX
0x843a, //40
0x8555, //58
0x8645, //48 4a BMIN
0x10fb,
};


//==========================================================
//	CAMERA_EFFECT_NONE  (1/6) 
//==========================================================
const unsigned short sr030pc30_effect_none[] = { 
0x0310,
0x1103,
0x1230,
0x0313,
0x103b,
0x2002,
};

//==========================================================
//	CAMERA_EFFECT_MONO  (2/6) 
//==========================================================
const unsigned short sr030pc30_effect_gray[] = { 
0x0310,
0x1103,
0x1233,
0x4480,
0x4580,
0x0313,
0x103b,
0x2002,
};

//==========================================================
//	CAMERA_EFFECT_SEPIA  (3/6) 
//==========================================================
const unsigned short sr030pc30_effect_sepia[] = { 
0x0310,
0x1103,
0x1233,
0x4470,
0x4598,
0x0313,
0x103b,
0x2002,
};

//==========================================================
//	CAMERA_EFFECT_GREEN  (4/6) 
//==========================================================
const unsigned short sr030pc30_effect_green[] = {
0x0310,
0x1103,
0x1233,
0x4080,
0x4440,
0x4540,
0x0313,
0x103b,
0x2002,
};

//==========================================================
//	CAMERA_EFFECT_SKETCH 
//==========================================================
const unsigned short sr030pc30_effect_sketch[] = {
0x0310,
0x1153,
0x123b,
0x4015,
0x4480,
0x4580,
0x0313,
0x1001,
0x20ff,
};  

//==========================================================
//	CAMERA_EFFECT_AQUA  (5/6) 
//==========================================================
const unsigned short sr030pc30_effect_aqua[] = { 
0x0310,
0x1103,
0x1233,
0x44b0,
0x4540,
0x0313,
0x103b,
0x2002,
};

//==========================================================
//	CAMERA_EFFECT_NEGATIVE  (6/6) 
//==========================================================
const unsigned short sr030pc30_effect_negative[] = { 
0x0310,
0x1103,
0x1238,
0x0313,
0x103b,
0x2002,
};

//==========================================================
//	CAMERA_AEC_SPOT_METERING  (1/3) : Ãø±¤ ¹æ½Ä , ½ºÆÌ 
//==========================================================
const unsigned short sr030pc30_meter_spot[] =
{     
0x0320,
0x60c5,
};     
       
//==========================================================
//	CAMERA_AEC_FRAME_AVERAGE  (2/3) : Ãø±¤ ¹æ½Ä,  ÀÏ¹Ý
//==========================================================

const unsigned short sr030pc30_meter_frame[] =
{    
0x0320,
0x6055,
};     
       
//==========================================================
//	CAMERA_AEC_FRAME_AVERAGE  (3/3)  : Ãø±¤ ¹æ½Ä, Áß¾ÓÁßÁ¡	
//==========================================================
       
const unsigned short sr030pc30_meter_center[] =
{  
0x0320,
0x6095, 
};    

//==========================================================
//	CAMERA_FLIP_NONE  (1/4) : »óÇÏ ÁÂ¿ì ¹ÝÀü 
//==========================================================

const unsigned short sr030pc30_flip_none[] =
{
0x0300,
0x1190,
};     
       
//==========================================================
//	CAMERA_FLIP_WATER_ONLY   (2/4) 
//==========================================================

const unsigned short sr030pc30_flip_water[] =
{
0x0300,
0x1191,
};     
       
//==========================================================
//	CAMERA_FLIP_MIRROR_ONLY  (3/4)
//==========================================================

const unsigned short sr030pc30_flip_mirror[] =
{
0x0300,
0x1192,
};     
       
//==========================================================
//	CAMERA_FLIP_WATER_MIRROR (4/4) 
//==========================================================

const unsigned short sr030pc30_flip_water_mirror[] =
{
0x0300,
0x1193,
};     


/************** Blur Setting ********************/
/*Self shot*/
const unsigned short sr030pc30_blur_none[] = {
};

const unsigned short sr030pc30_blur_p1[] = {
};

const unsigned short sr030pc30_blur_p2[] = {
};

const unsigned short sr030pc30_blur_p3[] = {
};

/*vt call*/
const unsigned short sr030pc30_blur_vt_none[] = {
};

const unsigned short sr030pc30_blur_vt_p1[] = {
};

const unsigned short sr030pc30_blur_vt_p2[] = {
};

const unsigned short sr030pc30_blur_vt_p3[] = {
};

const unsigned short sr030pc30_dataline[] = {
0x0300,
0x5005, //Test Pattern
0x0311,
0x1098,
0x0312,
0x4022,
0x701c,
0x0313,
0x103a,
0x800c,
0x0314,
0x1000,
0x0315,
0x1002,
0x0316,
0x1000,
0x0320,
0x100c,
0x0322,
0x107b,
};

#define SR030PC30_DATALINE_REGS	(sizeof(sr030pc30_dataline) / sizeof(sr030pc30_dataline[0]))

const unsigned short sr030pc30_dataline_stop[] = 	{
0x0300,
0x5000, //Test Pattern
0x0311,
0x1099,
0x0312,
0x4023,
0x701d,
0x0313,
0x103b,
0x800d,
0x0314,
0x1001,
0x0315,
0x1003,
0x0316,
0x1001,
0x0320,
0x108c,
0x0322,
0x10fb,
};


/************** FPS********************/
const unsigned short sr030pc30_fps_default[] = {
0x0300, //7
0x1000,
0xffaa, //170ms 
0x01f1,//Sleep On
0x1190,//Fixed Disable
0x4001, //Hblank 336
0x4150, 
0x4201, //Vblank 20
0x432c, 
0x0320, //Page 20
0x8300, //EXP Normal 33.33 fps 
0x84af, 
0x85c8, 
0x8600, //EXPMin 6000.00 fps
0x87fa, 
0x8802, //EXP Max 10.00 fps 
0x8949, 
0x8af0, 
0x8B3a, //EXP100 
0x8C98, 
0x8D30, //EXP120 
0x8Ed4, 
0x9c06, //EXP Limit 857.14 fps 
0x9dd6, 
0x9e00, //EXP Unit 
0x9ffa, 
0x109c, // AE on
0x0300,
0x1000,
0x1190, //Fixed disable
0x01f0, //Sleep Off
};

const unsigned short sr030pc30_fps_7[] = {
0x0300, //7
0x1000,
0xffaa, //170ms 
0x01f1,//Sleep On
0x1194,//Fixed Disable
0x4001, //Hblank 336
0x4150, 
0x4200, //Vblank 20
0x4314, 
0x0320, //Page 20
0x8300, //EXP Normal 33.33 fps 
0x84af, 
0x85c8, 
0x8600, //EXPMin 6000.00 fps
0x87fa, 
0x8802, //EXP Max 10.00 fps 
0x8949, 
0x8af0,
0x8B3a, //EXP100 
0x8C98, 
0x8D30, //EXP120 
0x8Ed4, 
0x9103, //EXP Fix 7.00 fps
0x923b, 
0x9326, 
0x9c06, //EXP Limit 857.14 fps 
0x9dd6, 
0x9e00, //EXP Unit 
0x9ffa, 
0x109c, // AE on
0x0300,
0x1000,
0x1194, //Fixed enable
0x01f0, //Sleep Off
};

const unsigned short sr030pc30_fps_10[] = {
};

const unsigned short sr030pc30_fps_15[] = {
0x0300, //7
0x1000,
0xffaa, //170ms 
0x01f1,//Sleep On
0x1194,//Fixed Disable
0x4001, //Hblank 336
0x4150, 
0x4200, //Vblank 20
0x4314, 
0x0320, //Page 20
0x8300, //EXP Normal 33.33 fps 
0x84af, 
0x85c8, 
0x8600, //EXPMin 6000.00 fps
0x87fa, 
0x8801,
0x8924, 
0x8af8,
0x8B3a, //EXP100 
0x8C98, 
0x8D30, //EXP120 
0x8Ed4, 
0x9101, //EXP Fix 15.00 fps
0x927c, 
0x93dc, 
0x9c06, //EXP Limit 857.14 fps 
0x9dd6, 
0x9e00, //EXP Unit 
0x9ffa, 
0x109c, // AE on
0x0300,
0x1000,
0x1194, //Fixed enable
0x01f0, //Sleep Off
};

const unsigned short sr030pc30_QVGA_fps_15[] = {
0x0300, //7
0x1000,
0xffaa, //170ms 
0x01f1,//Sleep On
0x1194,//Fixed Disable
0x4001, //Hblank 336
0x4150, 
0x4200, //Vblank 20
0x4314,
0x0318,
0x1007,
0x2002,
0x2180,
0x2201,
0x23e0,
0x2400,
0x2500,
0x2600,
0x2700,
0x2801,
0x2940,
0x2a00,
0x2bf0,
0x2c10,
0x2d00,
0x2e10,
0x2f00,
0x3053,
0x3053,
0x0313,
0x800c,
0x0320, //Page 20
0x8300, //EXP Normal 33.33 fps 
0x84af, 
0x85c8, 
0x8600, //EXPMin 6000.00 fps
0x87fa, 
0x8801,
0x8924, 
0x8af8,
0x8B3a, //EXP100 
0x8C98, 
0x8D30, //EXP120 
0x8Ed4, 
0x9101, //EXP Fix 15.00 fps
0x927c, 
0x93dc, 
0x9c06, //EXP Limit 857.14 fps 
0x9dd6, 
0x9e00, //EXP Unit 
0x9ffa, 
0x109c, // AE on
0x0300,
0x1000,
0x1194, //Fixed enable
0x01f0, //Sleep Off
};

const unsigned short sr030pc30_QVGA_fps_20[] = {
0x0300, //7
0x1000,
0xffaa, //170ms 
0x01f1,//Sleep On
0x1194,//Fixed Disable
0x4001, //Hblank 336
0x4150, 
0x4200, //Vblank 4
0x4304,
0x0318,
0x1007,
0x2002,
0x2180,
0x2201,
0x23e0,
0x2400,
0x2500,
0x2600,
0x2700,
0x2801,
0x2940,
0x2a00,
0x2bf0,
0x2c10,
0x2d00,
0x2e10,
0x2f00,
0x3053,
0x3053,
0x0313,
0x800c,
0x0320, //Page 20
0x8300, //EXP Normal 33.33 fps 
0x84af, 
0x85c8, 
0x8600, //EXPMin 6000.00 fps
0x87fa, 
0x8800, //EXP Max 25.00 fps 
0x89ea, 
0x8a60, 
0x8B3a, //EXP100 
0x8C98, 
0x8D30, //EXP120 
0x8Ed4, 
0x9101, //EXP Fix 20.00 fps
0x9232, 
0x9327,  
0x9c06, //EXP Limit 857.14 fps 
0x9dd6, 
0x9e00, //EXP Unit 
0x9ffa, 
0x109c, // AE on
0x0300,
0x1000,
0x1194, //Fixed enable
0x01f0, //Sleep Off
};

const unsigned short sr030pc30_vt_fps_7[] = {
};

const unsigned short sr030pc30_vt_fps_10[] = {
};

const unsigned short sr030pc30_vt_fps_15[] = {
};


//==========================================================
//  CAMERA_PRETTY_OFF  (1/4) »Ç»þ½Ã È¿°ú off
//==========================================================
const unsigned short reg_pretty_none_table[] = 
{
0x0313, 
0x2005, 
0x2103, 
0x9005, 
0x9103, 
0x0316,
0x1001,
0x3000,
0x3114,
0x3223,
0x333b,
0x345d,
0x3579,
0x368e,
0x379f,
0x38af,
0x39bd,
0x3aca,
0x3bdd,
0x3cec,
0x3df7,
0x3eff,
};

//==========================================================
//  CAMERA_PRETTY_LEVEL1  (2/4) 
//==========================================================
const unsigned short reg_pretty_level1_table[] = 
{
0x0313, //Page 13 Edge Enhancement
0x2003,
0x2102,
0x9003,
0x9102,
0x0316,
0x3000,
0x3113,
0x3221,
0x333b,
0x3469,
0x358e,
0x36aa,
0x37c1,
0x38d1,
0x39dd,
0x3ae7,
0x3bf3,
0x3cfa,
0x3dfd,
0x3eff,
};

//==========================================================
//  CAMERA_PRETTY_LEVEL3  (3/4) 
//==========================================================
const unsigned short reg_pretty_level2_table[] = 
{
0x0313, //Page 13 Edge Enhancement
0x2002, 
0x2101, 
0x9002, 
0x9101, 
0x0316,
0x3000,
0x3110,
0x321f,
0x333e,
0x3475,
0x359f,
0x36be,
0x37d3,
0x38e2,
0x39eb,
0x3af2,
0x3bfa,
0x3cfd,
0x3dff,
0x3eff,
};

//==========================================================
//  CAMERA_PRETTY_LEVEL3  (4/4) 
//==========================================================
const unsigned short reg_pretty_level3_table[] = 
{
0x0313, //Page 13 Edge Enhancement
0x2000, 
0x2100, 
0x9000,
0x9100,
0x0316,
0x3000,
0x310c,
0x321d,
0x3343,
0x3489,
0x35b9,
0x36d7,
0x37e8,
0x38f2,
0x39f8,
0x3afb,
0x3bfe,
0x3cff,
0x3dff,
0x3eff,
}; 

const unsigned short reg_pretty_vt_none_table[] =
{
};

const unsigned short reg_pretty_vt_level1_table[] =
{
};

const unsigned short reg_pretty_vt_level2_table[] =
{
};

const unsigned short reg_pretty_vt_level3_table[] =
{
}; 

const unsigned short reg_self_capture_table[] = 
{
#if 0
0x0000, 
0x0000, 
0x0000, 
0x0000, 
0x0000, 
0x0000, 
0x0000, 
0x0000, 
#endif
};
#endif /* SR030PC30_H */

