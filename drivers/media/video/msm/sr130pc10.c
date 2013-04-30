/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <media/msm_camera.h>
#include <linux/gpio.h>
#include "sr130pc10.h"
#include <linux/slab.h>
#include <mach/vreg.h>
#include <mach/camera.h>

//#define SENSOR_DEBUG 0
#undef CONFIG_LOAD_FILE 
//#define CONFIG_LOAD_FILE 

////////////////////////////////////////
#define CAM_RESET    174
#define CAM_STANDBY  175
#define CAM_EN       2
#define CAM_I2C_SCL  0
#define CAM_I2C_SDA  1
#define CAM_VT_nSTBY 31
#define CAM_VT_RST   132
#define CAM_MCLK     15

#define CAM_FLASH_ENSET 57
#define CAM_FLASH_FLEN 56

#define PCAM_CONNECT_CHECK		0
#define PCAM_VT_MODE	        1
#define PCAM_EXPOSURE_TIME		2
#define PCAM_ISO_SPEED			3
#define PCAM_FIXED_FRAME		4
#define PCAM_AUTO_FRAME			5
#define PCAM_NIGHT_SHOT			6
#define PCAM_FLASH_OFF			7
#define PCAM_MOVIE_FLASH_ON		8
#define PCAM_PREVIEW_FPS		9
#define PCAM_GET_FLASH			10
#define PCAM_LOW_TEMP			11

/* EV */
#define SR130PC10_EV_MINUS_4   -4
#define SR130PC10_EV_MINUS_3   -3
#define SR130PC10_EV_MINUS_2   -2
#define SR130PC10_EV_MINUS_1   -1
#define SR130PC10_EV_DEFAULT    0
#define SR130PC10_EV_PLUS_1     1
#define SR130PC10_EV_PLUS_2     2
#define SR130PC10_EV_PLUS_3     3
#define SR130PC10_EV_PLUS_4     4

/* DTP */
#define SR130PC10_DTP_OFF		0
#define SR130PC10_DTP_ON		1
#define SR130PC10_DTP_OFF_ACK		2
#define SR130PC10_DTP_ON_ACK		3

#define SR130PC10_WRITE_LIST(A) \
    {\
        sr130pc10_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);\
    }
struct sr130pc10_work_t {
	struct work_struct work;
};

static struct  sr130pc10_work_t *sr130pc10_sensorw;
static struct  i2c_client *sr130pc10_client;
static int prev_vtcall_mode=-1;
static int brightness = 0;
static int preview_enable = 0;

struct sr130pc10_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
	int dtp_mode;
    int vtcall_mode;
};

static unsigned int probe_init_retry = 0;
static struct sr130pc10_ctrl_t *sr130pc10_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(sr130pc10_wait_queue);
DECLARE_MUTEX(sr130pc10_sem);

#ifdef CONFIG_LOAD_FILE
static int sr130pc10_regs_table_write(char *name);
#endif

//static int16_t sr130pc10_effect = CAMERA_EFFECT_OFF;
static int rotation_status = 0;
static int factory_test = 0;
static char sr1_effect = 0;
static char sr1_whiteBalance = 0;
static int sr130pc10_set_power(int onoff);
/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct sr130pc10_reg sr130pc10_regs;

//paul_1013
extern struct i2c_client *lp8720_i2c_client;

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
    printk("[CAMDRV/SR130PC10] lp8720_i2c_write begin\n");

    int rc;
    unsigned char buf[2];
    struct i2c_msg msg = {
        .addr = lp8720_i2c_client->addr,
        .flags = 0,
        .len = 2,
        .buf = buf,
    };

    buf[0] = addr;
    buf[1] = data;

    rc = i2c_transfer(lp8720_i2c_client->adapter, &msg, 1);
    if (rc < 0)
        printk(KERN_ERR "[CAMDRV/SR130PC10] %s: lp8720_i2c_write failed: %d\n",__func__, rc);        

    printk("[CAMDRV/SR130PC10] lp8720_i2c_write finish\n");

    return (rc == 1) ? 0 : -EIO;
}

/*=============================================================*/
static int sr130pc10_sensor_read(unsigned short subaddr, unsigned short *data)
{
	//printk("<=ASWOOGI=> sr130pc10_sensor_read\n");

	int ret;
	unsigned char buf[1] = {0};
	struct i2c_msg msg = { sr130pc10_client->addr, 0, 1, buf };
	
	buf[0] = subaddr;
//	buf[1] = 0x0;

	ret = i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

//	*data = ((buf[0] << 8) | buf[1]);
	*data = buf[0];

error:
	//printk("[ASWOOGI] on read func  sr130pc10_client->addr : %x\n",  sr130pc10_client->addr);    
	//printk("[ASWOOGI] on read func  subaddr : %x\n", subaddr);
	//printk("[ASWOOGI] on read func  data : %x\n", data);

    
	return ret;
}

static int sr130pc10_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[2] = {0};
	struct i2c_msg msg = { sr130pc10_client->addr, 0, 2, buf };

	buf[0] = subaddr;
	buf[1] = val;
	
        if(i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1)
        {
            return 0;
        }
        else
        {
            printk("[sr130pc10] sr130pc10_sensor_write fail \n");        
            return -EIO;
        }
}


static int sr130pc10_sensor_write_list(struct samsung_short_t *list,int size, char *name)
{
  int ret = 0;
#ifdef CONFIG_LOAD_FILE
  ret = sr130pc10_regs_table_write(name);
#else
	int i;

  for (i = 0; i < size; i++)
  {
    if(list[i].subaddr == 0xff)
    {
      printk("<=PCAM=> now SLEEP!!!!\n");
      msleep(list[i].value*8);
    }
    else
    {
      if(sr130pc10_sensor_write(list[i].subaddr, list[i].value) < 0)
      {
        printk("<=PCAM=> sensor_write_list fail...-_-\n");
        return -1;
      }
    }
  }
#endif
	return ret;
}

void sr130pc10_set_preview(void)
{
    unsigned short value =0;
    int shade_value = 0;
    unsigned short agc_value = 0;
	preview_enable = 1; //Setting preview flag 
#if 0
    if(prev_vtcall_mode==sr130pc10_ctrl->vtcall_mode)
        return;
#endif
    printk(KERN_ERR "[SR130PC10] sr130pc10_set_preview : dtp(%d), vt(%d)\n",sr130pc10_ctrl->dtp_mode, sr130pc10_ctrl->vtcall_mode);

    if(!sr130pc10_ctrl->dtp_mode) {
        if(sr130pc10_ctrl->vtcall_mode) {
            SR130PC10_WRITE_LIST(sr130pc10_preview_reg); // preview start
            //SR130PC10_WRITE_LIST(sr130pc10_init_vt_reg);
            SR130PC10_WRITE_LIST(sr130pc10_fps_15);
        } else {
            SR130PC10_WRITE_LIST(sr130pc10_preview_reg); // preview start
        }
        msleep(300);
    }
    prev_vtcall_mode=sr130pc10_ctrl->vtcall_mode;

}
static long sr130pc10_set_sensor_mode(int mode)
{
	printk("[CAM-SENSOR] =Sensor Mode\n ");
        int cnt, vsync_value;
    
	switch (mode) {

	case SENSOR_PREVIEW_MODE:
		printk("[SR130PC10]-> Preview \n");
		sr130pc10_set_preview();
        //factory_test = 0;

#if 0
        for(cnt=0; cnt<200; cnt++)
        {
          vsync_value = gpio_get_value(14);
          if(vsync_value)
          {         
            printk(" on preview,  start cnt:%d vsync_value:%d\n", cnt, vsync_value);			                        
            break;
          }
          else
          {
            printk(" on preview,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);			
            msleep(1);
          }
        }
#endif

                
	//	sr130pc10_sensor_write_list(sr130pc10_preview_reg, sizeof(sr130pc10_preview_reg)/\ arun
	//	sizeof(sr130pc10_preview_reg[0]),"sr130pc10_preview_reg"); // preview start
		break;

	case SENSOR_SNAPSHOT_MODE:
		printk("[PGH}-> Capture \n");		
		//Snapshot mode is enabled so setting the preview flag
		preview_enable = 0;
       	brightness = 0;
		sr130pc10_sensor_write_list(sr130pc10_capture_reg, sizeof(sr130pc10_capture_reg)/\
		sizeof(sr130pc10_capture_reg[0]),"sr130pc10_capture_reg"); // preview start
		/* //SecFeature : for Android CCD preview mirror / snapshot non-mirror
		if(factory_test == 0)
                {      
                    if(rotation_status == 90 || rotation_status == 270)
                    {
                        sr130pc10_sensor_write(0x03, 0x00);
                        sr130pc10_sensor_write(0x11, 0x93);                    
                    }
                    else
                    {
                        sr130pc10_sensor_write(0x03, 0x00);
                        sr130pc10_sensor_write(0x11, 0x90);                    
                    }
                }
                */
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		printk("[PGH}-> Capture RAW \n");		
		break;

	default:
		return 0;
	}

	return 0;
}

static long sr130pc10_set_effect(char effect)
{
	long rc = 0;
	switch (effect) {
            case CAMERA_EFFECT_OFF:
                    printk("[SR130PC10] CAMERA_EFFECT_OFF\n");
                    sr130pc10_sensor_write_list(sr130pc10_effect_none, sizeof(sr130pc10_effect_none)/sizeof(sr130pc10_effect_none[0]),"sr130pc10_effect_none"); 
                    break;

            case CAMERA_EFFECT_MONO:
                    printk("[SR130PC10] CAMERA_EFFECT_MONO\n");
                    sr130pc10_sensor_write_list(sr130pc10_effect_gray, sizeof(sr130pc10_effect_gray)/sizeof(sr130pc10_effect_gray[0]),"sr130pc10_effect_gray");
                    break;

            case CAMERA_EFFECT_NEGATIVE:
                    printk("[SR130PC10] CAMERA_EFFECT_NEGATIVE\n");
                    sr130pc10_sensor_write_list(sr130pc10_effect_negative, sizeof(sr130pc10_effect_negative)/sizeof(sr130pc10_effect_negative[0]),"sr130pc10_effect_negative"); 
                    break;

            case CAMERA_EFFECT_SEPIA:
                    printk("[SR130PC10] CAMERA_EFFECT_SEPIA\n");
                    sr130pc10_sensor_write_list(sr130pc10_effect_sepia, sizeof(sr130pc10_effect_sepia)/sizeof(sr130pc10_effect_sepia[0]),"sr130pc10_effect_sepia"); 
                    break;

            case CAMERA_EFFECT_AQUA:
                    printk("[SR130PC10] CAMERA_EFFECT_AQUA\n");
                    sr130pc10_sensor_write_list(sr130pc10_effect_aqua, sizeof(sr130pc10_effect_aqua)/sizeof(sr130pc10_effect_aqua[0]),"sr130pc10_effect_aqua"); 
                    break;

            default:
               	printk("[SR130PC10] default .dsfsdf\n");
		        sr130pc10_sensor_write_list(sr130pc10_effect_none, sizeof(sr130pc10_effect_none)/sizeof(sr130pc10_effect_none[0]),"sr130pc10_effect_none"); 
                       //return -EINVAL;
                      return 0;
	}
	return rc;
}

static long sr130pc10_set_exposure_value(int exposure)
{
	long rc = 0;

	printk("exposure value  : %d\n", exposure);

	switch (exposure) {

		case SR130PC10_EV_MINUS_4:
			printk("CAMERA_EXPOSURE_VALUE_-4\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_m4, sizeof(sr130pc10_ev_m4)/sizeof(sr130pc10_ev_m4[0]),"sr130pc10_ev_m4"); 

			break;
		case SR130PC10_EV_MINUS_3:
			printk("CAMERA_EXPOSURE_VALUE_-3\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_m3, sizeof(sr130pc10_ev_m3)/sizeof(sr130pc10_ev_m3[0]),"sr130pc10_ev_m3"); 

			break;
		case SR130PC10_EV_MINUS_2:
			printk("CAMERA_EXPOSURE_VALUE_-2\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_m2, sizeof(sr130pc10_ev_m2)/sizeof(sr130pc10_ev_m2[0]),"sr130pc10_ev_m2"); 

			break;

		case SR130PC10_EV_MINUS_1:
			printk("CAMERA_EXPOSURE_VALUE_-1\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_m1, sizeof(sr130pc10_ev_m1)/sizeof(sr130pc10_ev_m1[0]),"sr130pc10_ev_m1"); 

			break;

		case SR130PC10_EV_DEFAULT:
			printk("CAMERA_EXPOSURE_VALUE_0\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_default, sizeof(sr130pc10_ev_default)/sizeof(sr130pc10_ev_default[0]),"sr130pc10_ev_default"); 

			break;

		case SR130PC10_EV_PLUS_1:
			printk("CAMERA_EXPOSURE_VALUE_1\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_p1, sizeof(sr130pc10_ev_p1)/sizeof(sr130pc10_ev_p1[0]),"sr130pc10_ev_p1"); 

			break;

		case SR130PC10_EV_PLUS_2:
			printk("CAMERA_EXPOSURE_VALUE_2\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_p2, sizeof(sr130pc10_ev_p2)/sizeof(sr130pc10_ev_p2[0]),"sr130pc10_ev_p2"); 

			break;
		case SR130PC10_EV_PLUS_3:
			printk("CAMERA_EXPOSURE_VALUE_3\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_p3, sizeof(sr130pc10_ev_p3)/sizeof(sr130pc10_ev_p3[0]),"sr130pc10_ev_p3"); 

			break;
		case SR130PC10_EV_PLUS_4:
			printk("CAMERA_EXPOSURE_VALUE_4\n");
        		sr130pc10_sensor_write_list(sr130pc10_ev_p4, sizeof(sr130pc10_ev_p4)/sizeof(sr130pc10_ev_p4[0]),"sr130pc10_ev_p4"); 

			break;
		default:
			printk("<=PCAM=> unexpected Exposure Value %s/%d\n", __func__, __LINE__);
//			return -EINVAL;
                        return 0;
	}

	return rc;
}    

static long sr130pc10_set_whitebalance(char wb)
{
	long rc = 0;

	printk("whitebalance : %d\n", wb);

	switch (wb) {
		case CAMERA_WB_AUTO:
            printk("CAMERA_WB_AUTO\n");
            sr130pc10_sensor_write_list(sr130pc10_wb_auto, \
                    sizeof(sr130pc10_wb_auto)/sizeof(sr130pc10_wb_auto[0]),"sr130pc10_wb_auto"); 
			break;

		case CAMERA_WB_INCANDESCENT:
            printk("CAMERA_WB_INCANDESCENT\n");
            sr130pc10_sensor_write_list(sr130pc10_wb_tungsten, \
                    sizeof(sr130pc10_wb_tungsten)/sizeof(sr130pc10_wb_tungsten[0]),"sr130pc10_wb_tungsten"); 
			break;

		case CAMERA_WB_FLUORESCENT:
            printk("CAMERA_WB_FLUORESCENT\n");
            sr130pc10_sensor_write_list(sr130pc10_wb_fluorescent, \
                    sizeof(sr130pc10_wb_fluorescent)/sizeof(sr130pc10_wb_fluorescent[0]),"sr130pc10_wb_fluorescent"); 
			break;

		case CAMERA_WB_DAYLIGHT:
            printk("<=PCAM=> CAMERA_WB_DAYLIGHT\n");
            sr130pc10_sensor_write_list(sr130pc10_wb_sunny, \
                    sizeof(sr130pc10_wb_sunny)/sizeof(sr130pc10_wb_sunny[0]),"sr130pc10_wb_sunny"); 
			break;

		case CAMERA_WB_CLOUDY_DAYLIGHT:
            printk("<=PCAM=> CAMERA_WB_CLOUDY_DAYLIGHT\n");
            sr130pc10_sensor_write_list(sr130pc10_wb_cloudy, \
                    sizeof(sr130pc10_wb_cloudy)/sizeof(sr130pc10_wb_cloudy[0]),"sr130pc10_wb_cloudy"); 
			break;

		default:
			printk("<=PCAM=> unexpected WB mode %s/%d\n", __func__, __LINE__);
//			return -EINVAL;
            return 0;
 	}

	return rc;
}

static long sr130pc10_set_rotation(int rotation)
{
    rotation_status = rotation;
    printk("[SR130PC10] current rotation status : %d\n",  rotation_status);
    
	return 0;
}

static int sr130pc10_start(void)
{
    int rc = 0;
    u8 data[2] = {0xEF, 0x01};
    u8 vender[1] = {0xC5};
    u8 version[1] = {0xC6};
    u8 vendor_id = 0xff, sw_ver = 0xff;
    
    printk(KERN_ERR "[CAMDRV/SR130PC10] %s E\n",__func__);

/*
    rc = sr130pc10_sensor_write(0xEF, 0x01);
    rc = sr130pc10_sensor_read(0xC5, &vendor_id);
    rc = sr130pc10_sensor_write(0xEF, 0x01);    
    rc = sr130pc10_sensor_read(0xC6, &sw_ver);
*/

    rc = sr130pc10_sensor_write(0x03, 0x00);
    rc = sr130pc10_sensor_read(0x04, &vendor_id);
    
    printk("[CAMDRV/SR130PC10]=================================rc=%d\n",rc);
    printk("[CAMDRV/SR130PC10]  [VGA CAM] vendor_id ID : 0x%x\n", vendor_id);
//    printk("[CAMDRV/SR130PC10]  [VGA CAM] software version : 0x%x\n", sw_ver);
//    printk("[CAMDRV/SR130PC10]=================================\n");

	//TELECA_PS_UPDATE
	if (vendor_id != 0x94){
		printk(" VENDOR not equal\n");
		//return  -1;
	}
    return rc;
}

static int sr130pc10_sensor_init_probe()
{
	int err = 0;

	printk("%s/%d \n", __func__, __LINE__);	

	err = sr130pc10_sensor_write_list(sr130pc10_reg_init,\
          sizeof(sr130pc10_reg_init) / sizeof(sr130pc10_reg_init[0]),"sr130pc10_reg_init");
	msleep(10);	

	return err;
}

static int sr130pc10_set_power(int onoff)
{
  int rc = 0;
    struct vreg *vreg_ldo20, *vreg_ldo11;

/* test1*/
    vreg_ldo20 = vreg_get(NULL, "gp13");
    if(!vreg_ldo20){
        printk("[S5K4ECGX]%s: VREG L20 get failed\n", __func__);
    }
    if(vreg_set_level(vreg_ldo20, 1800)){
        printk("[S5K4ECGX]%s: vreg_set_level failed\n", __func__);
    }

    vreg_ldo11 = vreg_get(NULL, "gp2");
    if (!vreg_ldo11) {
        printk("[S5K4ECGX]%s: VREG L11 get failed\n", __func__);
    }
    if (vreg_set_level(vreg_ldo11, 2800)) {
        printk("[S5K4ECGX]%s: vreg_set_level failed\n", __func__);	
    }
/* end of test1 */

  if(onoff)
  {
    printk("<=PCAM=> ++++++++++++++++++++++++++sr130pc10 test driver++++++++++++++++++++++++++++++++++++ \n");
    gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //CAM_RESET
    gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //CAM_STANDBY
    gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //CAM_EN
    gpio_tlmm_config(GPIO_CFG(CAM_VT_RST, 0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //CAM_VT_RST
    gpio_tlmm_config(GPIO_CFG(CAM_VT_nSTBY, 0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //CAM_VT_nSTBY
#if 0
    vreg_ldo20 = vreg_get(NULL, "gp13");
    vreg_ldo11 = vreg_get(NULL, "gp2");
    vreg_set_level(vreg_ldo20, 1800);
    vreg_set_level(vreg_ldo11, 2800);
    vreg_disable(vreg_ldo20);
    vreg_disable(vreg_ldo11);
#endif
    gpio_set_value(CAM_RESET, 0);	
    gpio_set_value(CAM_STANDBY, 0);	
    gpio_set_value(CAM_EN, 0);	
    gpio_set_value(CAM_VT_RST, 0);	
    gpio_set_value(CAM_VT_nSTBY, 0);	
    
    mdelay(1);

    //LDO3 1.8v
    lp8720_i2c_write(0x03, 0x0C);            // 010 01100
        
    lp8720_i2c_write(0x08, 0x00);
    gpio_set_value(CAM_EN, 1); //CAM_EN->UP	lp8720 enable

    //LDO3 1.8v enable        
    lp8720_i2c_write(0x08, 0x04);
    mdelay(1);

    if (vreg_enable(vreg_ldo20)) {//LDO20 powers both VDDIO 1.8V and 1.3M Core 1.8V
      printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR130PC10]%s: reg_enable failed\n", __func__);
    }
    if (vreg_enable(vreg_ldo11)) { //AVDD 2.8V
      printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR130PC10]%s: reg_enable failed\n", __func__);
    }
    udelay(100);
    
    gpio_set_value(CAM_VT_nSTBY, 1); //VGA_STBY UP
    udelay(5);
    
    gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 1,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
    mdelay(10);
    //CAM_MCLK
    // msm_camio_clk_rate_set(24000000);	//MCLK
    // msm_camio_camif_pad_reg_reset();
    
    gpio_set_value(CAM_VT_RST, 1); //VGA_RESET UP
    mdelay(2);
    printk("I2C Enable \n");  
    gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);            
    gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE); 
  }
  else
  {
    gpio_set_value(CAM_RESET, 0); //REST -> DOWN
    gpio_set_value(CAM_STANDBY,0);

    mdelay(1);
    lp8720_i2c_write(0x08, 0x00);
    gpio_set_value(CAM_EN, 0); //CAM_EN->UP	

    printk("I2C Disable \n"); 
    gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
    gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
    mdelay(7/*2*/);
    gpio_set_value(CAM_VT_RST, 0); //REST -> DOWN
    mdelay(2);
    gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 1,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
    udelay(50);
    gpio_set_value(CAM_VT_nSTBY, 0); //STBY -> DOWN
    udelay(5);

    //Entering shutdown mode
    if (vreg_disable(vreg_ldo11)) {  //Power down AVDD 2.8V 
      printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR130PC10]%s: reg_disable failed\n", __func__);
    }
    mdelay(1);
    if (vreg_disable(vreg_ldo20)) {  //Power down VDDIO 1.8V and 1.3Mcore 1.8V
      printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR130PC10]%s: reg_disable failed\n", __func__);
    }

  }
}

void sensor_rough_control_sr130pc10(void __user *arg)      
{
	sensor_ext_cfg_data		ctrl_info;
/*
	int Exptime;
	int Expmax;
	unsigned short read_1, read_2, read_3;	
*/
	printk("[SR130PC10] sensor_rough_control\n");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=SR130PC10=> %s fail copy_from_user!\n", __func__);
	}
	printk("<=SR130PC10=> TEST %d %d %d %d %d \n", ctrl_info.cmd, ctrl_info.device_id, ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);


	switch(ctrl_info.cmd)
	{
		case PCAM_CONNECT_CHECK:
                    printk("[SR130PC10] PCAM_CONNECT_CHECK\n");   
                    int rc = 0;
                    rc = sr130pc10_sensor_write(0x03, 0x00);
                    if(rc < 0) //check sensor connection
                    {
                       printk("[SR130PC10] Connect error\n");                       
                       ctrl_info.value_1 = 1;
                    }
                    break;
	
		case PCAM_EXPOSURE_TIME:
                    printk("[SR130PC10] PCAM_EXPOSURE_TIME\n");            
                    sr130pc10_sensor_write(0x03, 0x20);
                    sr130pc10_sensor_read(0x80, &ctrl_info.value_1);
                    sr130pc10_sensor_read(0x81, &ctrl_info.value_2);
                    sr130pc10_sensor_read(0x82, &ctrl_info.value_3);
                    printk("[SR130PC10] PCAM_EXPOSURE_TIME : A(%x), B(%x), C(%x)\n]",ctrl_info.value_1,ctrl_info.value_2,ctrl_info.value_3);
                    break;

		case PCAM_ISO_SPEED:
                    printk("[SR130PC10] PCAM_ISO_SPEED\n");            
                    sr130pc10_sensor_write(0x03, 0x20);
                    sr130pc10_sensor_read(0xb0, &ctrl_info.value_1);
                    break;

		case PCAM_PREVIEW_FPS:
                    printk("[SR130PC10] PCAM_PREVIEW_FPS : %d\n", ctrl_info.device_id);  
                    if(ctrl_info.device_id == 15)
                        sr130pc10_sensor_write_list(sr130pc10_vt_fps_15, sizeof(sr130pc10_vt_fps_15)/sizeof(sr130pc10_vt_fps_15[0]),"sr130pc10_vt_fps_15"); 
                    break;

		default :
			printk("<=SR130PC10=> Unexpected mode on sensor_rough_control!!!\n");
			break;
	}

	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("<=SR130PC10=> %s fail on copy_to_user!\n", __func__);
	}
	
}

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static char *sr130pc10_regs_table = NULL;
static int sr130pc10_regs_table_size;

int sr130pc10_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());

	filp = filp_open("/data/sr130pc10.h", O_RDONLY, 0);

	if (IS_ERR(filp)) {
		printk("file open error %d\n", PTR_ERR(filp));
		return -1;
	}
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	//dp = kmalloc(l, GFP_KERNEL);
	dp = vmalloc(l);
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
		//kfree(dp);
		vfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);
	set_fs(fs);
	sr130pc10_regs_table = dp;
	sr130pc10_regs_table_size = l;
	*((sr130pc10_regs_table + sr130pc10_regs_table_size) - 1) = '\0';

//	printk("sr130pc10_regs_table 0x%04x, %ld\n", dp, l);
	return 0;
}

void sr130pc10_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (sr130pc10_regs_table) {
		//kfree(sr130pc10_regs_table);
		vfree(sr130pc10_regs_table);
		sr130pc10_regs_table = NULL;
	}	
}

static int sr130pc10_regs_table_write(char *name)
{
	char *start, *end, *reg;	
	unsigned short addr, value;
	char reg_buf[5], data_buf[5];

	*(reg_buf + 4) = '\0';
	*(data_buf + 4) = '\0';

	start = strstr(sr130pc10_regs_table, name);
	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{0x");		
		if (reg)
			start = (reg + 11);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 4);	
			memcpy(data_buf, (reg + 7), 4);	
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
//			printk("addr 0x%04x, value 0x%04x\n", addr, value);

			if (addr == 0xdd)
			{
//		    	msleep(value);
//				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else if (addr == 0xff){
		    	msleep(value * 8);
				printk("delay 0x%04x, value 0x%04x\n", addr, value);
					}
			else
				sr130pc10_sensor_write(addr, value);
		}
	}
	return 0;
}
#endif

int sr130pc10_sensor_init(const struct msm_camera_sensor_info *data)
{
  int i, rc = 0;
	printk("[SR130PC10] %s/%d \n", __func__, __LINE__);	

#ifdef CONFIG_LOAD_FILE
	if(0 > sr130pc10_regs_table_init()) {
			CDBG("%s file open failed!\n",__func__);
			rc = -1;
			goto init_fail;
	}
#endif
        
	sr130pc10_ctrl = kzalloc(sizeof(struct sr130pc10_ctrl_t), GFP_KERNEL);
	if (!sr130pc10_ctrl) {
		printk("[SR130PC10]sr130pc10_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
    prev_vtcall_mode=-1;

	if (data)
		sr130pc10_ctrl->sensordata = data;

  /* Input MCLK = 24MHz */
  msm_camio_clk_rate_set(24000000);
  msleep(5);
  
  msm_camio_camif_pad_reg_reset();
  sr130pc10_set_power(true);
  
  rc = sr130pc10_sensor_init_probe();

  if(rc < 0)
  {
    for(i=0;i<5;i++)
    {
      printk("[SR130PC10]sr130pc10_sensor_init failed!\n");
      sr130pc10_set_power(false);
      mdelay(50);
      msm_camio_camif_pad_reg_reset();
      sr130pc10_set_power(true);
      msleep(5);
      rc = sr130pc10_sensor_init_probe();
      probe_init_retry++;
      if(rc >= 0)break;
    }
    if(rc < 0)goto init_fail;
  }

init_done:
	return rc;

init_fail:
	kfree(sr130pc10_ctrl);
	return rc;
}

static int sr130pc10_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&sr130pc10_wait_queue);
	return 0;
}
int sr130pc10_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    long   rc = 0;

    if (copy_from_user(&cfg_data,
            (void *)argp,
            sizeof(struct sensor_cfg_data)))
        return -EFAULT;

  
   // printk("sr130pc10_ioctl,************ cfgtype = %d, mode = %d\n",
//        cfg_data.cfgtype, cfg_data.mode);
        switch (cfg_data.cfgtype)
        {
            case CFG_SET_MODE:
				//printk("CFG_SET_MODE************ \n");
                rc = sr130pc10_set_sensor_mode(
                        cfg_data.mode);
            break;        
            
			case CFG_SET_EFFECT:
				printk("sr130pc10_sensor_ext_config, EXT_CFG_SET_EFFECT\n");         
	            rc = sr130pc10_set_effect(cfg_data.mode);			    
			    break;			    
			default:
			    rc = -EFAULT;
			    break;
        } 
    
    return rc;
}

static int sr130pc10_set_dtp(int onoff)
{
    printk(KERN_ERR "[SR130PC10] dtp onoff : %d ",onoff);

    switch(onoff)
    {
        case  SR130PC10_DTP_OFF:
			printk("arun in case SR130PC10_DTP_OFF\n");			
            sr130pc10_ctrl->dtp_mode = 0;
            SR130PC10_WRITE_LIST(sr130pc10_dataline_stop);
			printk("arun after dataline_stop\n");
            break;
        
        case SR130PC10_DTP_ON:
			printk("arun in case SR130PC10_DTP_ON\n");
            sr130pc10_ctrl->dtp_mode = 1;
			//SR130PC10_WRITE_LIST(sr130pc10_reg_init);
			//printk("arun after reg_init\n");
            SR130PC10_WRITE_LIST(sr130pc10_dataline);
			printk("arun after dataline\n");
            break;
        
        default:
            printk("[DTP]Invalid DTP mode!!!\n");
            return -EINVAL;
    }
    return 0;
}

static int sr130pc10_set_fps_mode(unsigned int mode) 
{
    printk(KERN_ERR "[CAMDRV/sr130pc10]  %s -mode : %d \n",__FUNCTION__,mode);
    
    if((mode == EXT_CFG_FRAME_AUTO) || (mode > EXT_CFG_FRAME_FIX_30))
    { 
        printk(KERN_ERR "[CAMDRV/sr130pc10] mode change to CAMERA_MODE");
        sr130pc10_ctrl->vtcall_mode = 0;
    }     
    else
    {
        printk(KERN_ERR "[CAMDRV/sr130pc10] mode change to CAMCORDER_MODE");
        sr130pc10_ctrl->vtcall_mode = 1;
    }
   
    return 0;
}

int sr130pc10_sensor_ext_config(void __user *argp)
{
    long ext_config_return = 0;
    sensor_ext_cfg_data cfg_data;
    int exposureTime_value1 = 0, exposureTime_value2 = 0, exposureTime_value3 = 0;
    int exposureTime = 0;

    if (copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
        return -EFAULT;

    //printk("sr130pc10_sensor_ext_config, cfg_data.cmd=%d\n", cfg_data.cmd);

    switch(cfg_data.cmd)
    {
        case EXT_CFG_SET_EFFECT:
            sr1_effect = cfg_data.value_1;
            ext_config_return = sr130pc10_set_effect(sr1_effect);
            break;

        case EXT_CFG_GET_EXIF_INFO:
            printk("[SR130PC10] EXT_CFG_GET_EXIF_INFO\n");
            sr130pc10_sensor_write(0x03, 0x20);
            sr130pc10_sensor_read(0x80, &exposureTime_value1);
            sr130pc10_sensor_read(0x81, &exposureTime_value2);
            sr130pc10_sensor_read(0x82, &exposureTime_value3);
            // MCLK = 24 MHz. Divide by 24000 to get exposure time in msec.
            exposureTime = ((exposureTime_value1<<19) + (exposureTime_value2<<11) + (exposureTime_value3<<3)) / 24000;
            cfg_data.value_1 = exposureTime;
            printk("[SR130PC10] EXT_CFG_GET_EXIF_INFO: A(%x), B(%x), C(%x)\n", exposureTime_value1, exposureTime_value2, exposureTime_value3);
            printk("[SR130PC10] exposureTime=%d\n", exposureTime);

            // ISO
            sr130pc10_sensor_write(0x03, 0x20);
            sr130pc10_sensor_read(0xB0, &cfg_data.cmd);
            printk(KERN_ERR "[SR130PC10] ISO **iso(%d)\n", cfg_data.cmd);
            break;
         case EXT_CFG_SET_BRIGHTNESS:
         //  printk(KERN_ERR "[CAMDRV/SR130PC10] EXT_CFG_SET_BRIGHTNESS *** ( %d) brightness =%d preview_enable = %d \n",cfg_data.value_1, ,brightness,preview_enable);
			if((brightness == 0) && (preview_enable == 0)){ 
				//Brightness control should be applied only once before preview is enabled
            	ext_config_return = sr130pc10_set_exposure_value(cfg_data.value_1);
         		brightness = 1;
			}
			//P110909-1364 : running camera, control  Brightness, take a shot and then check image in quick view, return to preview 
			//error : there\92s difference on brightness before Quick view and after
			//This created a side effect so chaged the condition to  ||  
			if((brightness) || (preview_enable))
				//This enables when the used tries to change the exposure from UI
            	ext_config_return = sr130pc10_set_exposure_value(cfg_data.value_1);

            break;
         case EXT_CFG_SET_FPS_MODE:
         // printk(KERN_ERR "[CAMDRV/SR130PC100] EXT_CFG_SET_FPS_MODE ***(%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            sr130pc10_set_fps_mode(cfg_data.value_1); 
            break;
        case EXT_CFG_SET_WB:
            sr1_whiteBalance = cfg_data.value_1;
            ext_config_return = sr130pc10_set_whitebalance(sr1_whiteBalance);
	    break;

	case EXT_CFG_SET_DTP:
	    printk(KERN_ERR "[SR130PC10] EXT_CFG_SET_DTP (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
	    ext_config_return = sr130pc10_set_dtp(cfg_data.value_1);
	    if(cfg_data.value_1 == 0) {
	        cfg_data.value_2 = 2;
	    } else if(cfg_data.value_1 == 1) {
	        cfg_data.value_2 = 3;
	    }        
	break;

        default: break;
    }

    if(copy_to_user((void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
        printk("[sr130pc10]%s fail on copy_to_user!\n", __func__);

#if 0
  struct sensor_cfg_data cfg_data;
  long   rc = 0;
  
  unsigned int value_1, value_2, value_3;
  
  if (copy_from_user(&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data)))
    return -EFAULT;*/
  
  /* down(&sr130pc10_sem); */
  
  printk("sr130pc10_ioctl, cfgtype = %d, mode = %d\n", \
          cfg_data.cfgtype, cfg_data.mode);
  
  switch (cfg_data.cfgtype) {
    case CFG_SET_MODE:
    rc = sr130pc10_set_sensor_mode(cfg_data.mode);
    break;
    
    case CFG_SET_EFFECT:
    rc = sr130pc10_set_effect(cfg_data.mode, cfg_data.cfg.effect);
    break;
    
    case CFG_SET_EXPOSURE_VALUE:
    rc = sr130pc10_set_exposure_value(cfg_data.mode, cfg_data.cfg.ev);			
    break;
    
    case CFG_SET_WB:
    rc = sr130pc10_set_whitebalance(cfg_data.mode, cfg_data.cfg.wb);
    break;
    
    case CFG_SET_ROTATION:
    rc = sr130pc10_set_rotation(cfg_data.cfg.rotation);
    break;
    
    
    case CFG_SET_DATALINE_CHECK:
    if(cfg_data.cfg.dataline)
    {
      printk("[SR130PC10] CFG_SET_DATALINE_CHECK ON\n");
      sr130pc10_sensor_write(0x03, 0x00);
      sr130pc10_sensor_write(0x50, 0x05);
      factory_test = 1;                        
    }
    else
    {         
      printk("[SR130PC10] CFG_SET_DATALINE_CHECK OFF \n");
      sr130pc10_sensor_write(0x03, 0x00);
      sr130pc10_sensor_write(0x50, 0x00);
    }                            
    break;
    
    case CFG_GET_AF_MAX_STEPS:
    default:
    //			rc = -EINVAL;
    rc = 0;
    break;
  }
  /* up(&sr130pc10_sem); */
#endif
  
  return ext_config_return;
}

int sr130pc10_sensor_release(void)
{
	int rc = 0;

  //Disalbing the variables since sensor going to shutdown
  preview_enable = 0;
  brightness = 0;

  sr130pc10_set_power(false);

	/* down(&sr130pc10_sem); */
	kfree(sr130pc10_ctrl);
	/* up(&sr130pc10_sem); */

	return rc;
}

static int sr130pc10_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	printk("[SR130PC10] %s/%d \n", __func__, __LINE__);	

	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	sr130pc10_sensorw =
		kzalloc(sizeof(struct sr130pc10_work_t), GFP_KERNEL);

	if (!sr130pc10_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, sr130pc10_sensorw);
	sr130pc10_init_client(client);
	sr130pc10_client = client;

	printk("[SR130PC10] sr130pc10_probe succeeded!  %s/%d \n", __func__, __LINE__);	

	printk("sr130pc10_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(sr130pc10_sensorw);
	sr130pc10_sensorw = NULL;
	printk("[SR130PC10]sr130pc10_probe failed!\n");
	return rc;
}

static const struct i2c_device_id sr130pc10_i2c_id[] = {
	{ "sr130pc10_i2c", 0},
	{ },
};

static struct i2c_driver sr130pc10_i2c_driver = {
	.id_table = sr130pc10_i2c_id,
	.probe  = sr130pc10_i2c_probe,
	.remove = __exit_p(sr130pc10_i2c_remove),
	.driver = {
		.name = "sr130pc10",
	},
};

static int sr130pc10_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
    int rc = 0;
    printk("[SR130PC10] %s/%d \n", __func__, __LINE__);	

  rc = i2c_add_driver(&sr130pc10_i2c_driver);
  if (rc < 0 || sr130pc10_client == NULL) {
    printk("[SR130PC10]sr130pc10_sensor_probe fail\n");
    rc = -ENOTSUPP;
    i2c_del_driver(&sr130pc10_i2c_driver);
    goto probe_done;
  }

  printk("[SR130PC10] %s/%d \n", __func__, __LINE__);	
  printk("[SR130PC10] sr130pc10_client->addr : %x\n", sr130pc10_client->addr);
  printk("[SR130PC10] sr130pc10_client->adapter->nr : %d\n", sr130pc10_client->adapter->nr);
  
  //TELECA_PS_UPDATE
  sr130pc10_set_power(true);
  sr130pc10_start();
  
  s->s_init     = sr130pc10_sensor_init;
  s->s_release 	= sr130pc10_sensor_release;
  s->s_config  	= sr130pc10_sensor_config; //sr130pc10_sensor_ext_config;
  s->s_camera_type = FRONT_CAMERA_2D;
//   	s->s_mount_angle = 180; //SecFeature : for Android CCD preview mirror / snapshot non-mirror

  sr130pc10_set_power(false);
  
probe_done:
	printk("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;

//TELECA_PS_UPDATE
probe_fail:
    printk(KERN_ERR "[CAMDRV/SR130PC10] %s: failed\n", __func__);
	return rc;
}

static int __sr130pc10_probe(struct platform_device *pdev)
{
	printk("[SR130PC10]  %s/%d \n", __func__, __LINE__);	
	return msm_camera_drv_start(pdev, sr130pc10_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sr130pc10_probe,
	.driver = {
	.name = "msm_camera_sr130pc10",
	.owner = THIS_MODULE,
	},
};

static int __init sr130pc10_init(void)
{
	printk("[SR130PC10]  %s/%d E\n", __func__, __LINE__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(sr130pc10_init);
MODULE_DESCRIPTION("LSI sr130pc10 1.3M camera driver");
MODULE_LICENSE("GPL v2");
