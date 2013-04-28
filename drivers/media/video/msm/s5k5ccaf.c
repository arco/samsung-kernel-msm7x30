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

//PCAM 1/5" s5k5ccaf

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <linux/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>
#include <linux/module.h>

#undef PCAM_ENABLE_DEBUG
//#define PCAM_ENABLE_DEBUG
#include "s5k5ccaf.h"

#undef CONFIG_LOAD_FILE
//#define CONFIG_LOAD_FILE
#ifndef CONFIG_LOAD_FILE
#define S5K5CCAF_USE_BURSTMODE
//#define WORKAROUND_FOR_LOW_SPEED_I2C
#endif

#ifdef  S5K5CCAF_USE_BURSTMODE
#define S5K5CCAF_WRITE_LIST(A) \
        s5k5ccaf_sensor_burst_write(A,(sizeof(A) / sizeof(A[0])),#A);
#else
#define S5K5CCAF_WRITE_LIST(A) \
        s5k5ccaf_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#endif

#undef MEASURE_TIME_ELAPSED
#ifdef MEASURE_TIME_ELAPSED
#define BEGIN_TIME_STAMP(A) \
        measuring_time_elapsed(#A,1);
#define FINISH_TIME_STAMP(A) \
        measuring_time_elapsed(#A,0);
#else
#define BEGIN_TIME_STAMP(A)
#define FINISH_TIME_STAMP(A)
#endif
static char first_start_camera = 1;//  1 is not init a sensor
static char set_init0 = 0;
static char af_current_lux = 0;
static char flash_mode = 0;
static char afcanceled;
static char camera_status;
static char ae_lock;
static char awb_lock;
static char mEffect = 0;
static char mBrightness = 0;
static char mContrast = 0;
static char mSaturation = 0;
static char mSharpness = 0;
static char mWhiteBalance = 0;
static char mISO = 0;
static char mAutoExposure = 0;
static char mScene = 0;
static char mAfMode = 0;
static char mDTP = 0;
static char mInit = 0;
static char mMode = 0;
static int flash_status;
static char mFPS = 0;
static int preview_size = 0;
static int low_temp = 0; //TELECA_BATTERY
static char camera_mode = 0;
int b_esd_detected=false;

#define CAM_FLASH_ENSET 57
#define CAM_FLASH_FLEN 56

#define FULL_FLASH 20
#define PRE_FLASH 7
#define MOVIE_FLASH 8 // 109 mA

#define MACRO_FLASH 14
#define PRE_FLASH_OFF -1
#define FLASH_OFF 0

#define PREVIEW 1
#define SNAPSHOT 2

// ANCORA USA TMO GPIO Setting
#define CAM_MEGA_RST 174
#define CAM_MEGA_STANDBY 175
#define CAM_VGA_RST 132
#define CAM_VGA_STANDBY 31

#define INNER_WINDOW_WIDTH    143
#define INNER_WINDOW_HEIGHT   143
#define OUTER_WINDOW_WIDTH    320
#define OUTER_WINDOW_HEIGHT   266

#define	S5K5CCGX_PREVIEW_VGA		0	/* 640x480 */
#define	S5K5CCGX_PREVIEW_D1		    1   /* 720x480 */
#define	S5K5CCGX_PREVIEW_WVGA		2	/* 800x480 */
#define	S5K5CCGX_PREVIEW_SVGA		3	/* 800x600 */
#define	S5K5CCGX_PREVIEW_XGA		4	/* 1024x768*/
#define	S5K5CCGX_PREVIEW_PVGA		5	/* 1280*720*/
#define	S5K5CCGX_PREVIEW_528x432	6	/* 528*432 */


#define INTELLIGENT_TIMER_INTERVAL  (5)

struct s5k5ccaf_work {
	struct work_struct work;
};

struct s5k5ccaf_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

typedef enum {
  CAM_ANTIBANDING_50HZ = 50,
  CAM_ANTIBANDING_60HZ = 60
} cam_antibanding_setting;

struct s5k5ccaf_enum_framesize {
	/* mode is 0 for preview, 1 for capture */
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

/*static struct s5k5ccaf_enum_framesize s5k5ccaf_framesize_list[] = {
{ EXT_CFG_PREVIEW_SIZE_1280x720_D1,  1280, 720 },
{ EXT_CFG_PREVIEW_SIZE_800x480_WVGA, 800, 480 },
{ EXT_CFG_PREVIEW_SIZE_720x480_D1,   720, 480 },
{ EXT_CFG_PREVIEW_SIZE_640x480_VGA,  640, 480 },
{ EXT_CFG_PREVIEW_SIZE_320x240_QVGA, 320, 240 },
{ EXT_CFG_PREVIEW_SIZE_176x144_QCIF, 176, 144 },
{ 0, 0, 0},
};*/

static struct s5k5ccaf_enum_framesize s5k5ccaf_framesize_list[] = { //TELECA_TOUCHAF
{ EXT_CFG_PREVIEW_SIZE_640x480_VGA,  640, 480 },
{ EXT_CFG_PREVIEW_SIZE_720x480_D1,   720, 480 },
{ EXT_CFG_PREVIEW_SIZE_800x480_WVGA, 800, 480 },
{ EXT_CFG_PREVIEW_SIZE_1280x720_D1,  1280, 720 },
{ EXT_CFG_PREVIEW_SIZE_320x240_QVGA, 320, 240 },
{ EXT_CFG_PREVIEW_SIZE_176x144_QCIF, 176, 144 },
{ 0, 0, 0},
};


static struct  s5k5ccaf_work *s5k5ccaf_sensorw;
static struct  i2c_client *s5k5ccaf_client;
struct i2c_client *lp8720_i2c_client;

static struct s5k5ccaf_ctrl *s5k5ccaf_ctrl;
#ifdef USE_FLASHOFF_TIMER
static struct timer_list flashoff_timer;
#endif
static DECLARE_WAIT_QUEUE_HEAD(s5k5ccaf_wait_queue);
DECLARE_MUTEX(s5k5ccaf_sem);
static int16_t s5k5ccaf_effect = CAMERA_EFFECT_OFF;

static int s5k5ccaf_regs_table_init_for_antibanding(void);
static int s5k5ccaf_sensor_init_probe(void);
static void s5k5ccaf_set_preview(void); //venkata

#ifdef CONFIG_LOAD_FILE
static int s5k5ccaf_regs_table_write(char *name);
#endif
#ifdef MEASURE_TIME_ELAPSED
static void measuring_time_elapsed(char *name, int begin);
#endif
 // FACTORY_TEST START
static bool camtype_init = false;
extern struct class *sec_class;
struct device *sec_cam_dev = NULL;
 // FACTORY_TEST END

typedef void (*_pw_enable)(int);
_pw_enable s5k5ccaf_pw_enable = NULL;

static unsigned int probe_init_retry = 0;
/*=============================================================
    EXTERNAL DECLARATIONS
==============================================================*/
extern int board_hw_revision;
#ifdef MEASURE_TIME_ELAPSED
static unsigned int before_time;
static unsigned int after_time = 0;
static unsigned int time_gab = 0;
static void measuring_time_elapsed(char *name, int begin)
{
  if(begin)
  {
    before_time = get_jiffies_64();
  }
  else
  {
    after_time = get_jiffies_64();
    time_gab = jiffies_to_msecs(after_time-before_time);
    printk("<=PCAM=> elapsed time of %s : %dmsec\n", name, time_gab);
  }
}
#endif

static int s5k5ccaf_sensor_read(unsigned short subaddr, unsigned short *data)
{
  int ret;
  unsigned char buf[2];
  struct i2c_msg msg = { s5k5ccaf_client->addr, 0, 2, buf };
  
  buf[0] = (subaddr >> 8);
  buf[1] = (subaddr & 0xFF);
  
  ret = i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
  if (ret == -EIO) 
    goto error;
  
  msg.flags = I2C_M_RD;
  
  ret = i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
  if (ret == -EIO) 
    goto error;
  
  *data = ((buf[0] << 8) | buf[1]);
  
error:
  return ret;
}

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
  int rc;
  unsigned char buf[2];
  struct i2c_msg msg = {lp8720_i2c_client->addr,0,2,buf};

  buf[0] = addr;
  buf[1] = data;

  rc = i2c_transfer(lp8720_i2c_client->adapter, &msg, 1);
  if (rc < 0)
    printk(KERN_ERR "[CAMDRV/CE147] %s: lp8720_i2c_write failed: %d\n",__func__, rc);        
  
  return (rc == 1) ? 0 : -EIO;
}

static int s5k5ccaf_sensor_write(unsigned short subaddr, unsigned short val)
{
  unsigned char buf[4];
  struct i2c_msg msg = { s5k5ccaf_client->addr, 0, 4, buf };
  
//	PCAM_DEBUG("[PGH] on write func s5k5ccaf_client->addr : %x\n", s5k5ccaf_client->addr);
//	PCAM_DEBUG("[PGH] on write func  s5k5ccaf_client->adapter->nr : %d\n", s5k5ccaf_client->adapter->nr);
  
  buf[0] = (subaddr >> 8);
  buf[1] = (subaddr & 0xFF);
  buf[2] = (val >> 8);
  buf[3] = (val & 0xFF);
  
  return i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int s5k5ccaf_sensor_write_list(struct samsung_short_t *list,int size, char *name)	
{
  int ret = 0;
  
#ifdef CONFIG_LOAD_FILE	
  ret = s5k5ccaf_regs_table_write(name);
#else
  int i = 0;
  
  PCAM_DEBUG("<=PCAM=> %s\n", name);;
  
  for (i = 0; i < size; i++)
  {
    PCAM_DEBUG("[PGH] %x      %x\n", list[i].subaddr, list[i].value);
    if(list[i].subaddr == 0xffff)
    {
      PCAM_DEBUG("<=PCAM=> now SLEEP!!!!\n  %d", list[i].value);
      msleep(list[i].value);
    }
    else
    {
      if(s5k5ccaf_sensor_write(list[i].subaddr, list[i].value) < 0)
      {
        PCAM_DEBUG("<=PCAM=> sensor_write_list fail...-_-\n");
        return -1;
      }
    }
  }
#endif

  return ret;
}

#ifdef S5K5CCAF_USE_BURSTMODE
#define BURST_MODE_BUFFER_MAX_SIZE 2700
unsigned char s5k5ccaf_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];
static int s5k5ccaf_sensor_burst_write(struct samsung_short_t *list,int size, char *name)	
{
  int err = -EINVAL;
  int i = 0;
  int idx = 0;
  int retry = 0;
 // int retry_esd =0 ;
  int rc = 0;

  unsigned short subaddr = 0, next_subaddr = 0;
  unsigned short value = 0;

  struct i2c_msg msg = {  s5k5ccaf_client->addr, 0, 0, s5k5ccaf_buf_for_burstmode};

  PCAM_DEBUG("%s(size:%d)\n",name,size);

I2C_RETRY:
  idx = 0;
  for (i = 0; i < size; i++)
  {
    if(idx > (BURST_MODE_BUFFER_MAX_SIZE-10))
    {
      PCAM_DEBUG("<=PCAM=> BURST MODE buffer overflow!!!\n");
      return err;
    }

    subaddr = list[i].subaddr;
    
    if(subaddr == 0x0F12)
      next_subaddr = list[i+1].subaddr;
    
    value = list[i].value;

    switch(subaddr)
    {
      case 0x0F12:
      {
        // make and fill buffer for burst mode write
        if(idx ==0) 
        {
          s5k5ccaf_buf_for_burstmode[idx++] = 0x0F;
          s5k5ccaf_buf_for_burstmode[idx++] = 0x12;
        }
        s5k5ccaf_buf_for_burstmode[idx++] = value>> 8;
        s5k5ccaf_buf_for_burstmode[idx++] = value & 0xFF;

        //write in burstmode	
        if(next_subaddr != 0x0F12)
        {
          msg.len = idx;
          err = i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
          //PCAM_DEBUG("s5k5ccaf_sensor_burst_write, idx = %d\n",idx);
          idx=0;
#if 0
        if(err < 0)
        {   
            printk("[S5K5CCAF]%s: register set failed. try again.*******\n",__func__);  
            msleep(50);
            if(retry_esd == 5)
            {
                printk("[S5K5CCAF] i2c transation is failed try again by power off and on for the ESD \n");
                retry_esd = 0;
                s5k5ccaf_pw_enable(false);
                msleep(50);
                s5k5ccaf_pw_enable(true);
                msm_camio_camif_pad_reg_reset();
                msleep(5);
                rc = s5k5ccaf_sensor_init_probe();       
                if(rc >= 0)break;
            } 
            retry_esd++;
            goto I2C_RETRY;  
        }
#endif
       }
      }
      break;
      
      case 0xFFFF:
        msleep(value);
        break;
      default:
        idx = 0;
        err = s5k5ccaf_sensor_write(subaddr, value);
        break;
    }
    
    
    if(err < 0)
    {
      printk("[S5K5CCAF]%s: register set failed. try again.\n",__func__);
      retry++;
      msleep(20);
      if((retry++)<10) goto I2C_RETRY;
      
      return err;
    }


  }

  //PCAM_DEBUG("end!\n");
  return 0;
}
#endif /* #ifdef S5K4ECGX_USE_BURSTMODE */

void sensor_effect_control(char value)
{
  switch(value)
  {
    case EXT_CFG_EFFECT_NORMAL :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_off);} break;		
    case EXT_CFG_EFFECT_NEGATIVE :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_negative); } break;	
    case EXT_CFG_EFFECT_MONO :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_mono); } break;	
    case EXT_CFG_EFFECT_SEPIA :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_sepia); } break;	
    default :{ PCAM_DEBUG("<=PCAM=> Unexpected Effect mode : %d\n",  value); } break;
  }
}

void sensor_whitebalance_control(char value)
{
  switch(value)
  {
    case EXT_CFG_WB_AUTO :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_auto); } break;	
    case EXT_CFG_WB_DAYLIGHT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_daylight); } break;	
    case EXT_CFG_WB_CLOUDY :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_cloudy); } break;	
    case EXT_CFG_WB_FLUORESCENT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_fluorescent); } break;	
    case EXT_CFG_WB_INCANDESCENT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_incandescent); } break;	
    default :{ PCAM_DEBUG("<=PCAM=> Unexpected WB mode : %d\n",  value); } break;
  }// end of switch
}

void sensor_brightness_control(char value)
{
  switch(value)
  {
    case PCAM_BR_STEP_P_4 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_4); } break;
    case PCAM_BR_STEP_P_3 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_3); } break;
    case PCAM_BR_STEP_P_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_2); } break;
    case PCAM_BR_STEP_P_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_1); } break;
    case PCAM_BR_STEP_0 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_0); } break;
    case PCAM_BR_STEP_M_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_1); } break;
    case PCAM_BR_STEP_M_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_2); } break;
    case PCAM_BR_STEP_M_3 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_3); } break;
    case PCAM_BR_STEP_M_4 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_4); } break;
    default :{ PCAM_DEBUG("<=PCAM=> Unexpected BR mode : %d\n",  value); } break;
  }
}

void sensor_iso_control(char value)
{
  switch(value)
  {
    case EXT_CFG_ISO_AUTO :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_auto); } break;
    case EXT_CFG_ISO_50 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_50); } break;
    case EXT_CFG_ISO_100 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_100); } break;
    case EXT_CFG_ISO_200 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_200); } break;
    case EXT_CFG_ISO_400 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_400); } break;
    default :{ PCAM_DEBUG("<=PCAM=> Unexpected ISO mode : %d\n",  value); } break;
  }
}

void sensor_metering_control(char value)
{
  switch(value)
  {
    case EXT_CFG_METERING_NORMAL :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_metering_normal); } break;
    case EXT_CFG_METERING_SPOT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_metering_spot); } break;
    case EXT_CFG_METERING_CENTER :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_metering_center); } break;
    default :{ PCAM_DEBUG("<=PCAM=> Unexpected METERING mode : %d\n",  value); } break;
  }
}

void sensor_scene_control(char value)
{
  switch(value)
  {
    case EXT_CFG_SCENE_OFF :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off); } break;
    case EXT_CFG_SCENE_PORTRAIT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                   S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_portrait); } break;
    case EXT_CFG_SCENE_LANDSCAPE :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                    S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_landscape); } break;
    case EXT_CFG_SCENE_SPORTS :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                 S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_sports); } break;
    case EXT_CFG_SCENE_PARTY :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_party); } break;
    case EXT_CFG_SCENE_BEACH :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_beach); } break;
    case EXT_CFG_SCENE_SUNSET :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                 S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_sunset); } break;
    case EXT_CFG_SCENE_DAWN :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                               S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_dawn); } break;
    case EXT_CFG_SCENE_FALL :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                               S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_fall); } break;
    case EXT_CFG_SCENE_NIGHTSHOT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                    S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_nightshot); } break;
    case EXT_CFG_SCENE_BACKLIGHT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                    S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_backlight); } break;
    case EXT_CFG_SCENE_FIREWORK :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                   S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_firework); } break;
    case EXT_CFG_SCENE_TEXT :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                               S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_text); } break;
    case EXT_CFG_SCENE_CANDLE :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
                                 S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_candle);	} break;
    default :{ PCAM_DEBUG("<=PCAM=> Unexpected SCENE mode : %d\n",  value); } break;				
  }
}

void sensor_contrast_control(char value)
{
 	switch(value)
 	{
  		case EXT_CFG_CR_STEP_M_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_m_2); } break;
  		case EXT_CFG_CR_STEP_M_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_m_1);	} break;
  		case EXT_CFG_CR_STEP_0 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_0); } break;
  		case EXT_CFG_CR_STEP_P_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_p_1); } break;
  		case EXT_CFG_CR_STEP_P_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_p_2); } break;
  		default :{ PCAM_DEBUG("<=PCAM=> Unexpected EXT_CFG CR_CONTROL mode : %d\n",  value); } break;
 	}
}

void sensor_saturation_control(char value)
{
 	switch(value)
 	{
  		case EXT_CFG_SA_STEP_M_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_m_2); } break;
  		case EXT_CFG_SA_STEP_M_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_m_1); } break;
  		case EXT_CFG_SA_STEP_0 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_0); } break;
  		case EXT_CFG_SA_STEP_P_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_p_1); } break;
  		case EXT_CFG_SA_STEP_P_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_p_2); } break;
  		default :{ PCAM_DEBUG("<=PCAM=> Unexpected EXT_CFG_SA_CONTROL mode : %d\n",  value); } break;
 	}
}


void sensor_sharpness_control(char value)
{
 	switch(value)
 	{
  		case EXT_CFG_SP_STEP_M_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_m_2); }	break;
  		case EXT_CFG_SP_STEP_M_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_m_1);	} break;
  		case EXT_CFG_SP_STEP_0 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_0);	} break;
  		case EXT_CFG_SP_STEP_P_1 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_p_1); }	break;
  		case EXT_CFG_SP_STEP_P_2 :{ S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_p_2); } break;
  		default :{ PCAM_DEBUG("<=PCAM=> Unexpected EXT_CFG_SP_CONTROL mode : %d\n",  value); }	break;
 	}
}

static int s5k5ccaf_get_lux(int* lux)
{
  int msb = 0;
  int lsb = 0;
  int cur_lux = -1;
  
  s5k5ccaf_sensor_write(0xFCFC, 0xD000);    
  s5k5ccaf_sensor_write(0x002C, 0x7000);
  s5k5ccaf_sensor_write(0x002E, 0x2A3C);
  s5k5ccaf_sensor_read(0x0F12, (unsigned short*)&lsb);
  s5k5ccaf_sensor_read(0x0F12, (unsigned short*)&msb);
  
  cur_lux = (msb<<16) | lsb;
  *lux = cur_lux;
  PCAM_DEBUG("%af_current_lux is 0x%x\n", af_current_lux);
  return cur_lux; //this value is under 0x0032 in low light condition 
}

static  int s5k5ccaf_set_flash(int lux_val)
{
  int i = 0;

//  printk("[[PCAM - 2]@@set_flash@@ flash_mode: %d, lux_val: %d\n", flash_mode, lux_val);
  
  if(flash_mode == EXT_CFG_FLASH_OFF) return 0;
  //Battery check for the temperature of -5 degress. TELECA_BATTERY
  if(low_temp == 1 && lux_val == FULL_FLASH)
     lux_val = MOVIE_FLASH;  

  /* initailize falsh IC */
  gpio_set_value(CAM_FLASH_ENSET,0);
  gpio_set_value(CAM_FLASH_FLEN,0);
  mdelay(1); // to enter a shutdown mode
  
  /* set to flash mode */
  if(lux_val>16) // FULL_FLASH
  {
    S5K5CCAF_WRITE_LIST(S5K5CCAF_Flash_Start_EVT1);
    gpio_set_value(CAM_FLASH_FLEN,1);

#ifdef USE_FLASHOFF_TIMER
    add_timer(&flashoff_timer); // for prevent LED 
#endif
  }
  else if(lux_val == MACRO_FLASH)
  {
    /* set to movie mode */
    S5K5CCAF_WRITE_LIST(S5K5CCAF_Flash_Start_EVT1);
    for(i=0;i<lux_val;i++)
    {
      udelay(1);
      gpio_set_value(CAM_FLASH_ENSET,1);
      udelay(1);
      gpio_set_value(CAM_FLASH_ENSET,0);
    }
    gpio_set_value(CAM_FLASH_ENSET,1); //value set

  }
  else if(lux_val > 0 &&  lux_val<=16) // PRE_FLASH, MOVIE_FLASH
  {
    /* set to movie mode */
    S5K5CCAF_WRITE_LIST(S5K5CCAF_Pre_Flash_Start_EVT1);
    for(i=0;i<lux_val;i++)
    {
      udelay(1);
      gpio_set_value(CAM_FLASH_ENSET,1);
      udelay(1);
      gpio_set_value(CAM_FLASH_ENSET,0);
    }
    gpio_set_value(CAM_FLASH_ENSET,1); //value set
  }
  flash_status = lux_val;
  
  /* setting a sensor #2*/
  if(lux_val==PRE_FLASH_OFF){
    S5K5CCAF_WRITE_LIST(S5K5CCAF_Pre_Flash_End_EVT1);}
  else if(lux_val==FLASH_OFF && afcanceled == false){
    S5K5CCAF_WRITE_LIST(S5K5CCAF_Flash_End_EVT1);}
  else if(lux_val==FLASH_OFF && afcanceled == true){
    S5K5CCAF_WRITE_LIST(S5K5CCAF_Pre_Flash_End_EVT1);}
  
  return 0;
}

#ifdef USE_FLASHOFF_TIMER
static void s5k5ccaf_flashoff_timer_handler(unsigned long data)
{
  s5k5ccaf_set_flash(FLASH_OFF);
}
#endif

int sensor_af_control(char value)
{
  static int pre_flash_on = 0;
  switch(value)
  { /* 1st */
    case EXT_CFG_AF_SET_AE_FOR_FLASH :
      s5k5ccaf_get_lux(&af_current_lux);

      PCAM_DEBUG("EXT_CFG_AF_SET_AE_FOR_FLASH -flash_mode:%d, -af_current_lux : %d\n",\
                  flash_mode, af_current_lux);

      if(flash_mode != EXT_CFG_FLASH_OFF)
      {
        if(flash_mode == EXT_CFG_FLASH_AUTO)
        {
          if(af_current_lux > 0x004C)break;
        }
        s5k5ccaf_sensor_write(0x0028, 0x7000);
        s5k5ccaf_sensor_write(0x002A, 0x0500); /* set Fast AE for flash */
        s5k5ccaf_sensor_write(0x0F12, 0x0000);
        s5k5ccaf_set_flash(PRE_FLASH);
        mdelay(500);
        pre_flash_on = 1;
      }
    break;

    case EXT_CFG_AF_CHECK_AE_STATUS :
    {
      unsigned short af_status;
      s5k5ccaf_sensor_write(0x002C, 0x7000);
      s5k5ccaf_sensor_write(0x002E, 0x2A70);
      s5k5ccaf_sensor_read(0x0F12, (unsigned short*)&af_status);
      switch(af_status)
      {
        case 1:
          PCAM_DEBUG("%s : EXT_CFG_AF_CHECK_AE_STATUS -EXT_CFG_AE_STABLE :%d \n", __func__, af_status);
          return EXT_CFG_AE_STABLE;
        break;
        default:
          PCAM_DEBUG("%s : EXT_CFG_AF_CHECK_AE_STATUS -EXT_CFG_AE_UNSTABLE :%d \n", __func__, af_status);
          return EXT_CFG_AE_UNSTABLE;
        break;
      }
    }
    break;

    case EXT_CFG_AF_CHECK_STATUS:
    {
      unsigned short af_status;
      PCAM_DEBUG("EXT_CFG_AF_CHECK_STATUS\n");
      s5k5ccaf_sensor_write(0x002C, 0x7000);
      s5k5ccaf_sensor_write(0x002E, 0x2D12);
      s5k5ccaf_sensor_read(0x0F12, &af_status);
      return af_status;
    }
    break;

    case EXT_CFG_AF_CHECK_2nd_STATUS:
    {
      unsigned short af_status;
      s5k5ccaf_sensor_write(0x002C, 0x7000);
      s5k5ccaf_sensor_write(0x002E, 0x1F2F);
      s5k5ccaf_sensor_read(0x0F12, &af_status);
      PCAM_DEBUG("<=PCAM=> AF 2nd check status : %x\n", af_status);
            switch(af_status)
            {
                case 1:
                    PCAM_DEBUG("%s : EXT_CFG_AF_CHECK_2nd_STATUS -EXT_CFG_AF_PROGRESS \n", __func__);
                    return EXT_CFG_AF_PROGRESS;
                break;
                
                case 0:
                    PCAM_DEBUG("%s : EXT_CFG_AF_CHECK_2nd_STATUS -EXT_CFG_AF_SUCCESS \n", __func__);
                    return EXT_CFG_AF_SUCCESS;
                break;
                default:
                    PCAM_DEBUG("%s : EXT_CFG_AF_CHECK_2nd_STATUS -EXT_CFG_AF_PROGRESS \n", __func__);
                    return EXT_CFG_AF_PROGRESS;
                break;
            }
    }        
    
    case EXT_CFG_AF_OFF:
    {
      afcanceled = true;
      S5K5CCAF_WRITE_LIST(s5k5ccaf_af_off); 			
    }
    break;
    
    case EXT_CFG_AF_SET_NORMAL:
    {
      S5K5CCAF_WRITE_LIST(s5k5ccaf_af_normal_on); 						
    }
    break;
    
    case EXT_CFG_AF_SET_MACRO:
    {
      S5K5CCAF_WRITE_LIST(s5k5ccaf_af_macro_on); 									
    }
    break;
    
    case EXT_CFG_AF_DO:
    {
      afcanceled = false;
      S5K5CCAF_WRITE_LIST(s5k5ccaf_af_do);
    }
    break;

    case EXT_CFG_AF_BACK_AE_FOR_FLASH :
      PCAM_DEBUG("%s : EXT_CFG_AF_BACK_AE_FOR_FLASH \n", __func__);
      if(pre_flash_on)
      {
        s5k5ccaf_sensor_write(0x0028, 0x7000);
        s5k5ccaf_sensor_write(0x002A, 0x0500); /* unset Fast AE for flash */
        s5k5ccaf_sensor_write(0x0F12, 0x0002);
        s5k5ccaf_set_flash(PRE_FLASH_OFF);
		mdelay(200);
        pre_flash_on=0;
      }
    break;

#if 0 // JHKWON_TEMP
    case EXT_CFG_AF_ABORT: 
    {
      S5K5CCAF_WRITE_LIST(s5k5ccaf_af_abort);
    }
    break;
#endif
    default:
    {
      PCAM_DEBUG("<=PCAM=> unexpected AF command : %d\n",value);
    }		
    break;
  }	

  return 0;
}

static void sensor_DTP_control(char value)
{
  switch(value)
  {
    case EXT_CFG_DTP_OFF:{
      S5K5CCAF_WRITE_LIST(s5k5ccaf_dtp_off);
    }
    break;
    
    case EXT_CFG_DTP_ON:{
      S5K5CCAF_WRITE_LIST(s5k5ccaf_dtp_on);
    }
    break;
    
    default:{
      PCAM_DEBUG("<=PCAM=> unexpected DTP control on PCAM\n");
    }
    break;
  }
}

static unsigned short get_sensor_value(unsigned short addr)
{
  unsigned short retval;
  s5k5ccaf_sensor_write(0x002C, 0x7000);
  s5k5ccaf_sensor_write(0x002E, addr);
  s5k5ccaf_sensor_read(0x0F12, &retval);
  PCAM_DEBUG("<=PCAM=> get_sensor_value : %d\n", retval);
  
  return retval;
}

static void set_sensor_value(unsigned short addr, unsigned short val)
{
  s5k5ccaf_sensor_write(0xFCFC, 0xD000);
  s5k5ccaf_sensor_write(0x0028, 0x7000);
  s5k5ccaf_sensor_write(0x002A, addr);
  s5k5ccaf_sensor_write(0x0F12, val);
}


static void sensor_TouchAF_control(unsigned short touch_X, unsigned short touch_Y)
{
  #define sensor_display_H (640) //640x480 sensor display
  #define sensor_display_V (480) //640x480 sensor display

  int index, err, i=0;
  unsigned int width,height;
  unsigned int Mapping_H, Mapping_V;
  int mapped_X, mapped_Y;
  int inner_window_start_X, inner_window_start_Y, outer_window_start_X, outer_window_start_Y;
  int inner_window_width, inner_window_height, outer_window_width, outer_window_height;
  
  
  if(mMode > EXT_CFG_PREVIEW_SIZE_640x480_VGA) {
    //index = EXT_CFG_PREVIEW_SIZE_640x480_VGA;
    index = 0;//TELECA_TOUCHAF
//    printk("[[PCAM - 7]] preview size %d is not support!!\n", mMode);
  }
  else {
    index = mMode;
//    printk("[[PCAM - 7]] preview idx : %d\n", mMode);
  }

  s5k5ccaf_short_t S5K5CCGX_TOUCH_AF[]=
  {
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x022C}, //AF window setting
    {0x0F12, 0x0100},	//REG_TC_AF_FstWinStartX 
    {0x0F12, 0x00E3},	//REG_TC_AF_FstWinStartY
    {0x0F12, 0x0200},	//REG_TC_AF_FstWinSizeX [5]
    {0x0F12, 0x0238},	//REG_TC_AF_FstWinSizeY [6]
    {0x0F12, 0x018C},	//REG_TC_AF_ScndWinStartX
    {0x0F12, 0x0166},	//REG_TC_AF_ScndWinStartY
    {0x0F12, 0x00E6},	//REG_TC_AF_ScndWinSizeX [9]
    {0x0F12, 0x0132},	//REG_TC_AF_ScndWinSizeY [10]
    {0x0F12, 0x0001},	//REG_TC_AF_WinSizesUpdated
    {0x0000, 0x0000}
  };
  
  /* get preview width, height */
  width = s5k5ccaf_framesize_list[index].width;
  height = s5k5ccaf_framesize_list[index].height;
  
  Mapping_H = sensor_display_H / width;
  Mapping_V = sensor_display_V / height;
  
  mapped_X = touch_X * Mapping_H; //sensor display size ·Î mapping
  mapped_Y = touch_Y * Mapping_V; //sensor display size ·Î mapping

  S5K5CCGX_TOUCH_AF[9].value = get_sensor_value(0x238);
  S5K5CCGX_TOUCH_AF[10].value = get_sensor_value(0x23A);
  S5K5CCGX_TOUCH_AF[5].value = get_sensor_value(0x230);
  S5K5CCGX_TOUCH_AF[6].value = get_sensor_value(0x232);

  inner_window_width = ( (S5K5CCGX_TOUCH_AF[9].value * sensor_display_H ) / 1024 );
  inner_window_height = ( (S5K5CCGX_TOUCH_AF[10].value * sensor_display_V ) /1024 );
  outer_window_width = ( (S5K5CCGX_TOUCH_AF[5].value * sensor_display_H ) /1024);
  outer_window_height = ( (S5K5CCGX_TOUCH_AF[6].value * sensor_display_V ) / 1024);

  PCAM_DEBUG("[[PCAM - 7]] mapped_X: %d, mapped_Y: %d\n", mapped_X, mapped_Y);
  PCAM_DEBUG("[[PCAM - 7]] inner_window_width: %d, inner_window_height: %d, outer_window_width: %d, outer_window_height: %d \n",\
            inner_window_width, inner_window_height, outer_window_width, outer_window_height);

  PCAM_DEBUG("[[PCAM - 7]] ScndWinSizeX: %d, ScndWinSizeY: %d, FstWinSizeX: %d, FstWinSizeY: %d\n",\
                      S5K5CCGX_TOUCH_AF[9].value, S5K5CCGX_TOUCH_AF[10].value,\
                      S5K5CCGX_TOUCH_AF[5].value, S5K5CCGX_TOUCH_AF[6].value);
  

  //2. set X axis
  if (mapped_X <= inner_window_width/2)
  {
    inner_window_start_X = 0;
    outer_window_start_X = 0;
  }
  else if (mapped_X <= outer_window_width/2)
  {
    inner_window_start_X = mapped_X - inner_window_width/2;
    outer_window_start_X = 0; //TELECA_TOUCHAF
  }
  else if (mapped_X >= ((sensor_display_H-1) - inner_window_width/2))
  // ( sensor_display_H-1) : 639, H size
  {
    inner_window_start_X = (sensor_display_H-1) - inner_window_width;
    outer_window_start_X = (sensor_display_H-1) - outer_window_width;
  }
  else if (mapped_X >= ((sensor_display_H-1) - outer_window_width/2))
  // (sensor_display_H-1) : 639, H size
  {
    inner_window_start_X = mapped_X - inner_window_width/2;
    outer_window_start_X = (sensor_display_H-1) - outer_window_width;
  }
  else
  {
    inner_window_start_X = mapped_X - inner_window_width/2;
    outer_window_start_X = mapped_X - outer_window_width/2;
  }
  
  //3. set Y axis
  if (mapped_Y <= inner_window_height/2)
  {
    inner_window_start_Y = 0;
    outer_window_start_Y = 0;
  }
  else if (mapped_Y <= outer_window_height/2)
  {
    inner_window_start_Y = mapped_Y - inner_window_height/2;
    outer_window_start_Y = 0;
  }
  else if (mapped_Y >= ((sensor_display_V-1 ) - inner_window_height/2))
  // ( sensor display V -1) : 479, V size
  {
    inner_window_start_Y = (sensor_display_V -1) - inner_window_height;
    outer_window_start_Y = (sensor_display_V -1) - outer_window_height;
  }
  else if (mapped_Y >= ((sensor_display_V -1) - outer_window_width/2))
  // (sensor_display_V -1) : 479, V size
  {
    inner_window_start_Y = mapped_Y - inner_window_height/2;
    outer_window_start_Y = (sensor_display_V -1) - outer_window_height;
  }
  else
  {
    inner_window_start_Y = mapped_Y - inner_window_height/2;
    outer_window_start_Y = mapped_Y - outer_window_height/2;
  }
  
  /*S5K5CCGX_TOUCH_AF[3].value = (outer_window_start_X * (2^10) / sensor_display_H);
  S5K5CCGX_TOUCH_AF[4].value = (outer_window_start_Y * (2^10) / sensor_display_V);
  
  S5K5CCGX_TOUCH_AF[7].value = (inner_window_start_X * (2^10) / sensor_display_H);
  S5K5CCGX_TOUCH_AF[8].value = (inner_window_start_Y * (2^10) / sensor_display_V);*/
  //TELECA_TOUCHAF
  S5K5CCGX_TOUCH_AF[3].value = (outer_window_start_X * (1024) / sensor_display_H);
  S5K5CCGX_TOUCH_AF[4].value = (outer_window_start_Y * (1024) / sensor_display_V);
  S5K5CCGX_TOUCH_AF[7].value = (inner_window_start_X * (1024) / sensor_display_H);
  S5K5CCGX_TOUCH_AF[8].value = (inner_window_start_Y * (1024) / sensor_display_V);

  PCAM_DEBUG("[[PCAM - 6] outer_window_start_X: %d, outer_window_start_Y: %d, inner_window_start_X: %d, inner_window_start_Y: %d\n",\
                      outer_window_start_X, outer_window_start_Y, inner_window_start_X, inner_window_start_Y);
  PCAM_DEBUG("[[PCAM - 6] fisrt reg(%d, %d, %d, %d)\n",\
          S5K5CCGX_TOUCH_AF[3].value,S5K5CCGX_TOUCH_AF[4].value,\
          S5K5CCGX_TOUCH_AF[5].value,S5K5CCGX_TOUCH_AF[6].value);
  PCAM_DEBUG("[[PCAM - 6] second reg(%d, %d, %d, %d)\n",\
          S5K5CCGX_TOUCH_AF[7].value,S5K5CCGX_TOUCH_AF[8].value,\
          S5K5CCGX_TOUCH_AF[9].value,S5K5CCGX_TOUCH_AF[10].value);
 
  for (i=0 ; S5K5CCGX_TOUCH_AF[i].subaddr != 0; i++)
  {
    err = s5k5ccaf_sensor_write(S5K5CCGX_TOUCH_AF[i].subaddr, S5K5CCGX_TOUCH_AF[i].value);
    if(err < 0){
      printk("%s: failed: i2c_write for touch_auto_focus\n", __func__);
    }
  }

  mdelay(100); //(100ms(camera preview) or 50ms (720P) // 1frame delay
  
  set_sensor_value(0x0224, 0x0005); //single AF

}

void s5k5ccaf_set_fps(char fps)
{
    PCAM_DEBUG("s5k5ccaf_set_fps %d\n",fps);		
    switch(fps)
    {
        case EXT_CFG_FRAME_AUTO :{
            PCAM_DEBUG("EXT_CFG_FRAME_AUTO\n");		
        }					
        break;
        case EXT_CFG_FRAME_FIX_15 :{
            S5K5CCAF_WRITE_LIST(s5k5ccaf_fps_15fix);
            PCAM_DEBUG("EXT_CFG_FRAME_FIX_15\n");
        }
        break;
        case EXT_CFG_FRAME_FIX_30 :{
            S5K5CCAF_WRITE_LIST(s5k5ccaf_fps_30fix);
			PCAM_DEBUG("EXT_CFG_FRAME_FIX_30\n");
        }
        break;
        default :{
            PCAM_DEBUG("<=PCAM=> Unexpected EXT_CFG_FRAME_CONTROL mode : %d\n", fps);
        }
        break;				
    }
}


void s5k5ccaf_set_preview_size(int preivewsize)
{
  PCAM_DEBUG(" preview size : %d \n",preivewsize );
  
  switch(preivewsize){
    case S5K5CCGX_PREVIEW_VGA: S5K5CCAF_WRITE_LIST(s5k5ccaf_640_preview);break;/* 640x480 */
    case S5K5CCGX_PREVIEW_D1: S5K5CCAF_WRITE_LIST(s5k5ccaf_720_preview);break; /* 720x480 */
    case S5K5CCGX_PREVIEW_WVGA: S5K5CCAF_WRITE_LIST(s5k5ccaf_800_Preview);break; /* 800x480 */
    case S5K5CCGX_PREVIEW_PVGA: S5K5CCAF_WRITE_LIST(s5k5ccaf_1280_preview);break; /* 1280*720*/

    case S5K5CCGX_PREVIEW_SVGA: break; /* 800x600 */
    case S5K5CCGX_PREVIEW_XGA: break;  /* 1024x768*/
    case S5K5CCGX_PREVIEW_528x432: break; /* 528x432 */
    default: PCAM_DEBUG("[S5K5CCAF]not supported previewsize : %d\n",  preivewsize);break;
}
}

int s5k5ccaf_sensor_esd_detected() //venkata
{
    printk("[s5k5ccaf] ESD Detected!!\n");
    b_esd_detected=true;
    return 0;
}



int s5k5ccaf_sensor_ext_config(void __user *arg)
{
  long   rc = 0;
  
    sensor_ext_cfg_data    cfg_data;
    
    if (copy_from_user((void *)&cfg_data, (const void *)arg, sizeof(cfg_data)))
        return -EFAULT;


  switch(cfg_data.cmd)
  {

    case EXT_CFG_SET_AF_OPERATION:
    {
      mAfMode= cfg_data.value_1;
      cfg_data.value_2 = sensor_af_control(mAfMode);
    }
    break;

    case EXT_CFG_SET_EFFECT:
    {
      mEffect = cfg_data.value_1;
      sensor_effect_control(mEffect);
    }// end of EXT_CFG_EFFECT_CONTROL
    break;

    case EXT_CFG_SET_WB:
    {
      mWhiteBalance = cfg_data.value_1;
      sensor_whitebalance_control(mWhiteBalance);
    }//end of EXT_CFG_WB_CONTROL
    break;

    case EXT_CFG_SET_BRIGHTNESS:
    {
      mBrightness = cfg_data.value_1;
      if(mInit)
        sensor_brightness_control(mBrightness);
    }//end of EXT_CFG_BR_CONTROL
    break;

    case EXT_CFG_SET_ISO:
    {
      mISO = cfg_data.value_1;
      sensor_iso_control(mISO);
    }
    break;

    case EXT_CFG_SET_METERING:
    {
      mAutoExposure = cfg_data.value_1;
      sensor_metering_control(mAutoExposure);
    }//end of case
    break;

    case EXT_CFG_SET_SCENE:
    {
      mScene = cfg_data.value_1;
      sensor_scene_control(mScene);
    }
    break;

    case EXT_CFG_SET_AE_AWB:
    {
      switch(cfg_data.value_1)
      {
        case EXT_CFG_AE_LOCK :
          ae_lock = EXT_CFG_AE_LOCK;
          S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_lock);
        break;
        case EXT_CFG_AE_UNLOCK :
          ae_lock = EXT_CFG_AE_UNLOCK;
          S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_unlock);
        break;
        case EXT_CFG_AWB_LOCK :
          awb_lock = EXT_CFG_AWB_LOCK;
          S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_lock);
        break;
        case EXT_CFG_AWB_UNLOCK :
          awb_lock = EXT_CFG_AWB_UNLOCK;
          S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_unlock);
        break;
#if 0 // JKKWON_TEMP       
        case EXT_CFG_AE_AWB_LOCK :{
          if(mWhiteBalance == 0)
          {
            awb_lock = EXT_CFG_AWB_LOCK;
            ae_lock = EXT_CFG_AE_LOCK;
            S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_lock);
            S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_lock);
          }
          else
          {
            ae_lock = EXT_CFG_AE_LOCK;
            S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_lock);
          }
        }
        break;
        case EXT_CFG_AE_AWB_UNLOCK :{
          if(mWhiteBalance == 0)
          {
            awb_lock = EXT_CFG_AWB_UNLOCK;
            ae_lock = EXT_CFG_AE_UNLOCK;
            S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_unlock);
            S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_unlock);
          }
          else
          {
            ae_lock = EXT_CFG_AE_UNLOCK;
            S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_unlock);
          }
        }
        break;
#endif        
        default :{
          PCAM_DEBUG("<=PCAM=> Unexpected AWB_AE mode : %d\n", cfg_data.value_1);
        }
        break;						
      
      }
    }
    break;
    
    case EXT_CFG_SET_CONTRAST:
    {
      mContrast = cfg_data.value_1;
      if(mInit)
        sensor_contrast_control(mContrast);
    }
    break;
    
    case EXT_CFG_SET_SATURATION:
    {
      mSaturation = cfg_data.value_1;
      if(mInit)
        sensor_saturation_control(mSaturation);
    }
    break;

    case EXT_CFG_SET_SHARPNESS:
    {
      mSharpness = cfg_data.value_1;
      if(mInit)
        sensor_sharpness_control(mSharpness);
    }
    break;
    
    case EXT_CFG_SET_DTP:
    {
      if(mInit == 0)
      {
        if(cfg_data.value_1 == 0)
          cfg_data.value_2 = 2;
        else if(cfg_data.value_1 == 1)
          cfg_data.value_2 = 3;

        mDTP = 1;
      }
      else
      {
        sensor_DTP_control(cfg_data.value_1);
        if(cfg_data.value_1 == 0)
          cfg_data.value_2 = 2;
        else if(cfg_data.value_1 == 1)
          cfg_data.value_2 = 3;

        mDTP = 0;
      }
    }
    break;
    
    case EXT_CFG_SET_PREVIEW_SIZE:
    {
      mMode = cfg_data.value_1; //TELECA_TOUCHAF
	preview_size = cfg_data.value_1;
    }
    break;

#if 0  //JHKWON_TEMP      
    case EXT_CFG_GET_MODULE_STATUS:
    {
      unsigned short	id = 0; //CAM FOR FW
      //ctrl_info.value_3 = gpio_get_value(0);
      
      s5k5ccaf_sensor_write(0x002C, 0x0000);
      s5k5ccaf_sensor_write(0x002E, 0x0040);
      s5k5ccaf_sensor_read(0x0F12, &id);
      
      cfg_data.value_3 = id;
      
      PCAM_DEBUG("<=PCAM=> check current module status : %x\n", cfg_data.value_3);
      PCAM_DEBUG("<=PCAM=> PINON/OFF : %d\n", gpio_get_value(126));
     
    }
    break;
#endif 

    case EXT_CFG_SET_FLASH_MODE:
    {
      PCAM_DEBUG("EXT_CFG_SET_FLASH_MODE, ctrl_info.value_1: %d\n", cfg_data.value_2);

      PCAM_DEBUG("[[PCAM - 3]] SET!! FLASH_MODE, ctrl_info.value_1: %x\n", \
                  cfg_data.value_2);

      if(cfg_data.value_2 != EXT_CFG_FLASH_TURN_ON && cfg_data.value_2 != EXT_CFG_FLASH_TURN_OFF)
      {
        flash_mode = cfg_data.value_2;
      }
      if(cfg_data.value_2 == EXT_CFG_FLASH_TURN_ON)
      {
        if(flash_mode == EXT_CFG_FLASH_AUTO)
        {
          s5k5ccaf_get_lux(&af_current_lux);
          if( af_current_lux <= 0x004C)s5k5ccaf_set_flash(MOVIE_FLASH);
        }
        else
          s5k5ccaf_set_flash(FULL_FLASH);
      }
      else if(cfg_data.value_2 == EXT_CFG_FLASH_TURN_OFF) {
        s5k5ccaf_set_flash(FLASH_OFF);
      }

      PCAM_DEBUG("[[PCAM - 3]] flash_mode:%d,  af_current_lux:%d\n", flash_mode, af_current_lux);

    }
    break;
	
	case EXT_CFG_GET_EXIF_INFO:
	{
		unsigned short exposure_time_lsb, exposure_time_msb, ISO_gain, ISO_value;

      s5k5ccaf_sensor_write(0xFCFC, 0xD000);
      s5k5ccaf_sensor_write(0x002C, 0x7000);
      s5k5ccaf_sensor_write(0x002E, 0x2A14);
		s5k5ccaf_sensor_read(0x0F12, &exposure_time_lsb);
		s5k5ccaf_sensor_read(0x0F12, &exposure_time_msb);
		//s5k5ccaf_sensor_write(0x002E, 0x2A18);
		//s5k5ccaf_sensor_read(0x0F12, &ISO_gain);
		PCAM_DEBUG("exposure_time_lsb=%d \n", exposure_time_lsb);
		PCAM_DEBUG("exposure_time_msb=%d \n", exposure_time_msb);

		cfg_data.value_1 = exposure_time_lsb;
		cfg_data.value_2 = exposure_time_msb;

		// Below logic is implemented in HAL
		/*if(exposure_time_msb==0)
			cfg_data.value_1 = exposure_time_lsb/400;
		else
		{
			//cfg_data.value_1 = exposure_time/400;
			cfg_data.value_1 = exposure_time_msb;
			cfg_data.value_1 = cfg_data.value_1<<16;
			cfg_data.value_1 = cfg_data.value_1|exposure_time_lsb;
		}*/

		s5k5ccaf_sensor_write(0xFCFC, 0xD000);
		s5k5ccaf_sensor_write(0x002C, 0x7000);
      s5k5ccaf_sensor_write(0x002E, 0x2A18);
      s5k5ccaf_sensor_read(0x0F12, &ISO_gain);

		// Below logic is implemented in HAL
		/*
		if (ISO_gain <= 384 )
			ISO_value=50;
		else if ( 384 < ISO_gain  && ISO_gain <= 640 )
			ISO_value=100;
		else if ( 640 < ISO_gain  && ISO_gain <= 896 )
			ISO_value=200;
		else
			ISO_value=400;
	      */
		cfg_data.cmd = ISO_gain;
    
		PCAM_DEBUG("exposure time : %x \n", cfg_data.value_1);
		PCAM_DEBUG("ISO : %x \n", ISO_gain);
    }
    break;
	
    /* Turn off flash after snapshot */
    case EXT_CFG_GET_FLASH_INFO:
      PCAM_DEBUG("[[PCAM - 4]] GET!! FLASH_INFO flash_mode: %d, flash_status: %d\n", flash_mode, flash_status);

      if(flash_mode != EXT_CFG_FLASH_OFF \
      && flash_status != FLASH_OFF \
      && flash_status != PRE_FLASH_OFF) {
        cfg_data.value_1 = flash_mode;
        cfg_data.value_2 = flash_status;
        PCAM_DEBUG("[[PCAM - 4]] flash off\n");
        s5k5ccaf_set_flash(FLASH_OFF);
      }
    break;

    case EXT_CFG_SET_TOUCHAF_POS:
    {
      unsigned short touchX, touchY = 0;
      
      touchX = cfg_data.value_1;
      touchY = cfg_data.value_2;
      PCAM_DEBUG("[[PCAM - 5]]SET_TOUCHAF_POS -X: %d, -Y: %d\n", touchX, touchY);
      sensor_TouchAF_control(touchX, touchY);
    }
    break;

        case EXT_CFG_SET_FPS:
            mFPS = cfg_data.value_1;
            if(set_init0)
              s5k5ccaf_set_fps(mFPS);
    break;                
	//TELECA_BATTERY
	case EXT_CFG_TEMP:
     PCAM_DEBUG("[S5K5CCAF] EXT_CFG_TEMP \n");  
     low_temp = 1;
     break;
    /*Camera Mode*/
    case EXT_CFG_SET_CAM_MODE:
	 camera_mode = cfg_data.value_1;
	 PCAM_DEBUG("CAMERA MODE %d\n",camera_mode);
	 if(camera_mode == EXT_CFG_CAM_MODE_CAMCORDER)
  	  S5K5CCAF_WRITE_LIST(s5k5ccaf_fps_30fix);
	break;
	case EXT_CFG_TEST_ESD: //TELECA_ESD venkata
		printk("[S5K5CCAF] Reset the Sensor for the ESD case from HAL\n");

		if(cfg_data.value_1 == 1)
            {
                printk(KERN_ERR "[S5K5CCAF]: ESD Value %d\n",b_esd_detected);
                cfg_data.value_2 = b_esd_detected;
                b_esd_detected=false;
            }
        else
	        {

				printk(KERN_ERR "[S5K5CCAF]:EXT_CFG_TEST_ESD Sensor Reset\n");
				s5k5ccaf_pw_enable(false);
				msleep(50);
				s5k5ccaf_pw_enable(true);
				//msm_camio_camif_pad_reg_reset();
				msleep(5);
				s5k5ccaf_sensor_init_probe();
				s5k5ccaf_set_preview();
			

	        }
	break;

/* DUMMY FUNCTION */ // JHKWON_TEMP
        case EXT_CFG_SET_FPS_MODE:
         break;        
        case EXT_CFG_SET_THUMB_NULL:
         break;               
        case EXT_CFG_SET_PICTURE_SIZE:
         break;
        case EXT_CFG_SET_JPEG_QUALITY:
        break;
        case EXT_CFG_SET_AUTO_CONTRAST:
        break;
        case EXT_CFG_SET_ZOOM:
        break;

/* DUMMY FUNCTION */
        default :
            PCAM_DEBUG("[s5k5ccaf]Unexpected mode on sensor_rough_control : %d\n", cfg_data.cmd);
        break;
    }
 
    if(copy_to_user((void *)arg, (const void *)&cfg_data, sizeof(cfg_data)))
    {
    PCAM_DEBUG("<=PCAM=> %s fail on copy_to_user!\n", __func__);
    }

  return rc;
}

static void s5k5ccaf_pw_enable_rev00(int enable)
{
    struct vreg *vreg_ldo20;
    vreg_ldo20 = vreg_get(NULL, "gp13");

    if(!vreg_ldo20){
        PCAM_DEBUG("[S5K5CCAF]%s: VREG L20 get failed\n", __func__);
    }

    PCAM_DEBUG("(vreg_set_level(vreg_ldo20, 1800 \n");  

    if(vreg_set_level(vreg_ldo20, 1800)){
        PCAM_DEBUG("[S5K5CCAF]%s: vreg_set_level failed\n", __func__);
    }
    if(enable == 1) //POWER ON
    {
        PCAM_DEBUG("initailize power control pin \n");  

        gpio_set_value(CAM_MEGA_RST, 0);
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        gpio_set_value(CAM_VGA_RST, 0);
        gpio_set_value(CAM_VGA_STANDBY, 0);

        PCAM_DEBUG("CAM_FLASH_ENSET \n");  

        /* initailize power control pin */    
        PCAM_DEBUG("LDO Core 1.2v \n");  

        //LDO Core 1.2v
        lp8720_i2c_write(0x06, 0x09);            //000 01001
        lp8720_i2c_write(0x07, 0x09);            //000 01001
        //LDO2 2.8v
        lp8720_i2c_write(0x02, 0x19);            // 001 11001
        //LDO3 1.8v
        lp8720_i2c_write(0x03, 0x0C);            // 010 01100
        //LDO5 2.8v
        lp8720_i2c_write(0x05, 0x19);            //011 11001

        /* LP8720 enable */
        lp8720_i2c_write(0x08, 0x00);
        gpio_set_value(2, 1);
        
        lp8720_i2c_write(0x08, 0x20);
        mdelay(1);
        lp8720_i2c_write(0x08, 0x22);
        mdelay(1);
        lp8720_i2c_write(0x08, 0x26);
        mdelay(1);
        lp8720_i2c_write(0x08, 0x36);
        mdelay(1);

        if(vreg_enable(vreg_ldo20)){
            PCAM_DEBUG("[S5K5CCAF]%s: reg_enable failed\n", __func__);
        }

        PCAM_DEBUG("I2C Enable \n");  
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);            
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);                      
        mdelay(30);   

        PCAM_DEBUG("VGA CAM_STANDBY High \n");       
        gpio_set_value(CAM_VGA_STANDBY, 1);
        
        mdelay(5); // >4ms
    
        PCAM_DEBUG("MCLK 24Mhz Enabled \n");   
        msm_camio_clk_rate_set(24000000);
    
        msm_camio_camif_pad_reg_reset();
        
        udelay(1);        
        
        PCAM_DEBUG("MCLK High \n");
        gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        udelay(20); // >10us
        
        PCAM_DEBUG("VGA CAM_RST High \n");       
        gpio_set_value(CAM_VGA_RST, 1);	   		
        mdelay(10); // >6.5ms
        
        PCAM_DEBUG("VGA CAM_STANDBY Low \n");       
        gpio_set_value(CAM_VGA_STANDBY, 0); 
        mdelay(6); // >5ms
        
        PCAM_DEBUG("VGA CAM_STANDBY High \n");       
        gpio_set_value(CAM_VGA_STANDBY, 1); 
        udelay(20); // >10us
        
        
        PCAM_DEBUG("Main CAM_STANDBY High \n");       
        gpio_set_value(CAM_MEGA_STANDBY, 1);
        mdelay(20); // >15us
        
        PCAM_DEBUG("Main CAM_RST High \n");           
        gpio_set_value(CAM_MEGA_RST, 1);
        mdelay(90); // >50ms
    }
    else //POWER OFF
    {
        PCAM_DEBUG("[S5K5CCAF]Camera Sensor Power OFF\n");

        PCAM_DEBUG("Main CAM_RST Low \n");
        gpio_set_value(CAM_MEGA_RST, 0);
        mdelay(1);
        
        PCAM_DEBUG("Main CAM_STANDBY Low \n");
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        mdelay(1);
        
        PCAM_DEBUG("VGA CAM_RST Low \n");
        gpio_set_value(CAM_VGA_RST, 0);         
        mdelay(1);
        
        PCAM_DEBUG("VGA CAM_STANDBY Low \n");
        gpio_set_value(CAM_VGA_STANDBY, 0); 
        mdelay(1);

        PCAM_DEBUG("FLASH Low \n");
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);
        mdelay(1);

        gpio_tlmm_config(GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

        /* LP8720 enable */
        lp8720_i2c_write(0x08, 0x00);

        if(vreg_disable(vreg_ldo20)){
            PCAM_DEBUG("[S5K5CCAF]%s: reg_enable failed\n", __func__);
        }

        udelay(50);
        udelay(50);
        //vreg_disable(vreg_ldo20);

        /*initailize power control pin*/    
        PCAM_DEBUG("I2C Disable \n"); 
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);                    
        mdelay(10);

        mdelay(1); // to enter a shutdown mode        
    }
}

static void s5k5ccaf_pw_enable_rev01(int enable)
{

  struct vreg *vreg_cam_out8_vddio;
  struct vreg *vreg_cam_out9_vdda;
  struct vreg *vreg_cam_out10_vddreg;
  struct vreg *vreg_cam_out17_af; 
  struct vreg *vreg_cam_gp15_vddreg;
  
  vreg_cam_out8_vddio		= vreg_get(NULL, "gp13");
  vreg_cam_out9_vdda		= vreg_get(NULL, "wlan2");
  vreg_cam_out10_vddreg	= vreg_get(NULL, "gp2");
  vreg_cam_out17_af		= vreg_get(NULL, "wlan");
  vreg_cam_gp15_vddreg	= vreg_get(NULL, "gp15");

  PCAM_DEBUG(" %s, enable? %d\n", __func__, enable);

  if(enable == 1) //POWER ON
  {
     b_esd_detected=false; //venkata
    vreg_set_level(vreg_cam_out17_af,  2800);	    // VDDAF 2.8V
    vreg_set_level(vreg_cam_out9_vdda,  2800);		// VDDA 2.8V
    vreg_set_level(vreg_cam_out10_vddreg, 1800);	// VGA Core 1.8V		
    vreg_set_level(vreg_cam_out8_vddio,  1800);	// VDDIO 1.8V		
    vreg_set_level(vreg_cam_gp15_vddreg,  1200);	// Mega Core VDD 1.2V			

    gpio_set_value(CAM_MEGA_RST, 0);
    gpio_set_value(CAM_MEGA_STANDBY, 0);
    gpio_set_value(CAM_VGA_RST, 0);
    gpio_set_value(CAM_VGA_STANDBY, 0);
    
    PCAM_DEBUG("CAM_FLASH_ENSET \n");  
#if 0
#if defined(CONFIG_USE_QUP_I2C)
    gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);            
    gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);                    
    mdelay(5);
#endif
#endif
    PCAM_DEBUG("Sensor AVDD 2.8V On \n");    
    vreg_enable(vreg_cam_out9_vdda);
    
    PCAM_DEBUG("VT Sensor Core VDD_REG 1.8V On \n");                      
    vreg_enable(vreg_cam_out10_vddreg);
    udelay(20);
    
    PCAM_DEBUG("Main Sensor Core VDD 1.2V On \n");   
    vreg_enable(vreg_cam_gp15_vddreg);		
    udelay(15);
    
    PCAM_DEBUG("Main Sensor VDDAF 2.8V On \n");  
    vreg_enable(vreg_cam_out17_af);		
    udelay(20);        
    
    PCAM_DEBUG("Sensor I/O VDDIO 1.8V On \n");                       
    vreg_enable(vreg_cam_out8_vddio);
    mdelay(1);
  
#if defined(CONFIG_USE_QUP_I2C)
    PCAM_DEBUG("I2C Enable \n");  
    gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);            
    gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);                      
    mdelay(5);
#endif
    PCAM_DEBUG("VGA CAM_STANDBY High \n");       
    gpio_set_value(CAM_VGA_STANDBY, 1);
    
    mdelay(5); // >4ms
    
//pault, temperature
#if 0
    PCAM_DEBUG("MCLK 24Mhz Enabled \n");   
    msm_camio_clk_rate_set(24000000);
    
    msm_camio_camif_pad_reg_reset();
    
    udelay(1);        
#endif

    PCAM_DEBUG("MCLK High \n");
    gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    udelay(20); // >10us
    
    PCAM_DEBUG("VGA CAM_RST High \n");       
    gpio_set_value(CAM_VGA_RST, 1);	   		
    mdelay(1); // >1ms
    
    PCAM_DEBUG("VGA CAM_STANDBY Low \n");       
    gpio_set_value(CAM_VGA_STANDBY, 0); 
    udelay(10); // >10us
    
    PCAM_DEBUG("Main CAM_STANDBY High \n");       
    gpio_set_value(CAM_MEGA_STANDBY, 1);
    udelay(20); // >15us
    
    PCAM_DEBUG("Main CAM_RST High \n");           
    gpio_set_value(CAM_MEGA_RST, 1);
    mdelay(50); // >50ms

  }
  else //POWER OFF
  {

#if defined(CONFIG_USE_QUP_I2C)
    PCAM_DEBUG("I2C Disable \n"); 
    gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
    gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);                    
    mdelay(5);
#endif

    PCAM_DEBUG("Main CAM_RST Low \n");
    gpio_set_value(CAM_MEGA_RST, 0);
    mdelay(1);
    
    PCAM_DEBUG("FLASH Low \n");
    gpio_set_value(CAM_FLASH_ENSET,0);
    gpio_set_value(CAM_FLASH_FLEN,0);
    mdelay(1);

    gpio_tlmm_config(GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
    mdelay(1);
    
    PCAM_DEBUG("Main CAM_STANDBY Low \n");
    gpio_set_value(CAM_MEGA_STANDBY, 0);
    mdelay(1);
    
    PCAM_DEBUG("VGA CAM_RST Low \n");
    gpio_set_value(CAM_VGA_RST, 0);         
    mdelay(1);
    
    PCAM_DEBUG("VGA CAM_STANDBY Low \n");
    gpio_set_value(CAM_VGA_STANDBY, 0); 
    mdelay(1);

    PCAM_DEBUG("Main Sensor VDDAF 2.8V OFF \n");  
    vreg_disable(vreg_cam_out17_af);			
    mdelay(1);    
    
    PCAM_DEBUG("Sensor I/O VDDIO 1.8V OFF \n");
    vreg_disable(vreg_cam_out8_vddio);
    mdelay(1);
    
    PCAM_DEBUG("VT Sensor Core VDD_REG 1.8V OFF \n");
    vreg_disable(vreg_cam_out10_vddreg);
    mdelay(1);
    
    PCAM_DEBUG("Sensor AVDD 2.8V OFF \n");
    vreg_disable(vreg_cam_out9_vdda);
    mdelay(1);
    
    PCAM_DEBUG("Main Sensor Core VDD 1.2V OFF \n");
    vreg_disable(vreg_cam_gp15_vddreg);
    mdelay(1);

  }
}

static long s5k5ccaf_set_effect(int mode, int effect)
{
  long rc = 0;

  switch (mode) 
  {
    case SENSOR_PREVIEW_MODE: PCAM_DEBUG("SENSOR_PREVIEW_MODE"); break;
    case SENSOR_SNAPSHOT_MODE: PCAM_DEBUG("SENSOR_SNAPSHOT_MODE"); break;
    default: PCAM_DEBUG("[PGH] %s default\n", __func__); break;
  }
    
  switch (effect) 
  {
    case CAMERA_EFFECT_OFF: {PCAM_DEBUG("CAMERA_EFFECT_OFF"); } break;
    case CAMERA_EFFECT_MONO: { PCAM_DEBUG("CAMERA_EFFECT_MONO"); } break;
    case CAMERA_EFFECT_NEGATIVE: { PCAM_DEBUG("CAMERA_EFFECT_NEGATIVE"); } break;
    case CAMERA_EFFECT_SOLARIZE: { PCAM_DEBUG("CAMERA_EFFECT_SOLARIZE"); } break;
    case CAMERA_EFFECT_SEPIA: { PCAM_DEBUG("CAMERA_EFFECT_SEPIA"); } break;
    default: { PCAM_DEBUG("<=PCAM=> unexpeceted effect  %s/%d\n", __func__, __LINE__); return -EINVAL; } break;
  }

	s5k5ccaf_effect = effect;
	
	return rc;
}

static void s5k5ccaf_set_preview(void)
{
  int rc = 0;
  char vsync_value, cnt;

  printk("********* s5k5ccaf_set_preview start\n");

  low_temp = 0; // TELECA_BATTERY
  if(first_start_camera)
  {
    PCAM_DEBUG("<=PCAM=> camera init for COOPER+ ver0.1");
    
#ifdef CONFIG_LOAD_FILE
    S5K5CCAF_WRITE_LIST(s5k5ccaf_pre_init0);
    msleep(100);
#endif

BEGIN_TIME_STAMP(s5k5ccaf_init0);

    S5K5CCAF_WRITE_LIST(s5k5ccaf_init0);

FINISH_TIME_STAMP(s5k5ccaf_init0);

#if 0
    rc = s5k5ccaf_regs_table_init_for_antibanding(); 
    if (rc < 0) {
      PCAM_DEBUG("s5k5ccaf_sensor_init failed!\n");
    }  
#endif

    set_init0 = 1;
    first_start_camera = 0;
    mInit = 1;

      }

  /* initailize falsh IC */
  gpio_set_value(CAM_FLASH_ENSET,0);
  gpio_set_value(CAM_FLASH_FLEN,0);
  mdelay(1); // to enter a shutdown mode

BEGIN_TIME_STAMP(s5k5ccaf_set_preview);

  if(mDTP == 1) {
    S5K5CCAF_WRITE_LIST(s5k5ccaf_dtp_on);
  }
  else {
      if(mScene != EXT_CFG_SCENE_OFF)
      {
        s5k5ccaf_set_preview_size(preview_size);
        S5K5CCAF_WRITE_LIST(s5k5ccaf_preview);
      }
//    s5k5ccaf_set_preview_size(preview_size);
//    S5K5CCAF_WRITE_LIST(s5k5ccaf_preview);
  }

  if(mScene == EXT_CFG_SCENE_OFF)
    sensor_effect_control(mEffect);
  
  if((set_init0 ==1) &&(mScene == EXT_CFG_SCENE_OFF))
    PCAM_DEBUG("<=PCAM=> skip scene off\n");
  else {
    sensor_scene_control(mScene);
  }

  //if(mScene != PCAM_SCENE_SPORTS && mScene != PCAM_SCENE_NIGHTSHOT)
  // sensor_iso_control(mISO);   

//  if (mFPS == EXT_CFG_FRAME_FIX_30)
//      S5K5CCAF_WRITE_LIST(s5k5ccaf_fps_30fix);

  
  if(mScene == EXT_CFG_SCENE_OFF)
  {
    sensor_brightness_control(mBrightness);
    sensor_metering_control(mAutoExposure);
    sensor_contrast_control(mContrast);
    sensor_saturation_control(mSaturation);
    sensor_sharpness_control(mSharpness);
    if((set_init0 ==1) &&(mWhiteBalance == EXT_CFG_WB_AUTO))
      PCAM_DEBUG("<=PCAM=> awb aleady done!\n");
    else 
      sensor_whitebalance_control(mWhiteBalance);
    
    sensor_af_control(mAfMode);


    s5k5ccaf_set_preview_size(preview_size);
    S5K5CCAF_WRITE_LIST(s5k5ccaf_preview);    
  }

  /* reset status*/
  afcanceled = false;
  camera_status = PREVIEW;
  flash_status = FLASH_OFF;

FINISH_TIME_STAMP(s5k5ccaf_set_preview);

  printk("********* s5k5ccaf_set_preview finish");
}

static void s5k5ccaf_intelligent_timer_for_capture(void)
{
    unsigned short capture_status = 0;
    int capture_loop_counter = 0;

    //msleep(INTELLIGENT_TIMER_INTERVAL);

    while(1)
    {
        s5k5ccaf_sensor_write(0x002C, 0x7000);
        s5k5ccaf_sensor_write(0x002E, 0x01EA);
        s5k5ccaf_sensor_read(0x0F12, &capture_status);

        PCAM_DEBUG("capture_status=%d, capture_loop_counter=%d\n", capture_status, capture_loop_counter);

        if(capture_status==0 || capture_loop_counter>30)    // Max wait: 30 * INTELLIGENT_TIMER_INTERVAL
            return;
        else
        {
            msleep(INTELLIGENT_TIMER_INTERVAL);
            capture_loop_counter++;
            continue;
        }
    }
}

static void s5k5ccaf_set_capture(void)
{
  int vsync_value, cnt = 0;

  if(camera_status == SNAPSHOT)
  {
    PCAM_DEBUG("camera_status : %s",\
                (camera_status == SNAPSHOT) ? "SNAPSHOT" : "PREVIEW");
    return;
  }
  
  camera_status = SNAPSHOT;
  
  /* Check current lux */
  if(flash_status == FLASH_OFF)s5k5ccaf_get_lux(&af_current_lux);
#if 0 //temperary block
  for(cnt=0; cnt<700; cnt++)
  {
    vsync_value = gpio_get_value(14);
    
    if(vsync_value)
      break;
    else {
      PCAM_DEBUG(" on snapshot,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);   
      PCAM_DEBUG(" on snapshot,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
      msleep(3);
    }
  }
#endif

  PCAM_DEBUG("[[PCAM - 1]] @@capture@@ flash_status: %d, flash_mode: %d,\
          mScene: %d, mAfMode: %d, af_current_lux: 0x00%x\n", \
           flash_status, flash_mode, mScene, mAfMode, af_current_lux);

  //4 /* CASE 1 : Flash control in High/Normal light condition*/  
  if(af_current_lux > 0x0020)
  {
  	/* Flash ON Mode */
    if((flash_mode == EXT_CFG_FLASH_ON))
    {
      /* use torch mode on below condition */
      if(mAfMode == EXT_CFG_AF_SET_MACRO)
        s5k5ccaf_set_flash(MACRO_FLASH);
      else 
        s5k5ccaf_set_flash(FULL_FLASH);
    }
    if(flash_mode == EXT_CFG_FLASH_AUTO){
	  if(af_current_lux <= 0x004C){
	      if(mAfMode == EXT_CFG_AF_SET_MACRO)
	        s5k5ccaf_set_flash(MACRO_FLASH);
	      else 
	        s5k5ccaf_set_flash(FULL_FLASH);
	  	}
    }

    if(af_current_lux > 0xFFFE)
    { /* high light condition */
      S5K5CCAF_WRITE_LIST(s5k5ccaf_snapshot_normal_high);
    }
    else if(af_current_lux > 0x0020)
    { /* normal light condition */
      S5K5CCAF_WRITE_LIST(s5k5ccaf_snapshot_normal_normal);
    }

    //msleep(200); /* Capture Delay, 160ms on Normal/Low Light */
    return;

  }
  //4 /* CASE 2 : Flash control in low light condition*/
  else
  {
    	PCAM_DEBUG("Capture in low light condition\n");
    //4 /*  is Night && Firework Mode Off? */
    if((mScene == EXT_CFG_SCENE_NIGHTSHOT) || (mScene == EXT_CFG_SCENE_FIREWORK) )
    {
      S5K5CCAF_WRITE_LIST(s5k5ccaf_snapshot_nightmode);
        //msleep(300); /* Capture Delay, 300ms on NIGHT/FIREWORK MODE */ 
      return;
    }
    else
    {
#if 0
      if(flash_mode != EXT_CFG_FLASH_OFF)
      {
        /* use torch mode on below condition */
        if(mAfMode == EXT_CFG_AF_SET_MACRO)
          s5k5ccaf_set_flash(MACRO_FLASH);
        else
          s5k5ccaf_set_flash(FULL_FLASH);
      }
#endif
#if 1
	  if(flash_mode == EXT_CFG_FLASH_ON)
      {
      	PCAM_DEBUG("Flash is in ON Mode\n");
        /* use torch mode on below condition */
        if((camera_mode == EXT_CFG_CAM_MODE_FACTORY_TEST) || (mAfMode == EXT_CFG_AF_SET_MACRO))
          s5k5ccaf_set_flash(MACRO_FLASH);
        else
          s5k5ccaf_set_flash(FULL_FLASH);
      }
    else if(flash_mode == EXT_CFG_FLASH_AUTO){
		PCAM_DEBUG("Flash is in Auto Mode\n");
	    if(af_current_lux <= 0x004C){
	      if(mAfMode == EXT_CFG_AF_SET_MACRO)
	        s5k5ccaf_set_flash(MACRO_FLASH);
	      else 
	        s5k5ccaf_set_flash(FULL_FLASH);
	  	}
	  }
	else
	{
		PCAM_DEBUG("Flash is in OFF Mode\n");
	}
#endif

      S5K5CCAF_WRITE_LIST(s5k5ccaf_snapshot_normal_low);
        //msleep(160);
      return;
    }
  }
}

static long s5k5ccaf_set_sensor_mode(int mode)
{
  PCAM_DEBUG("<=PCAM=> mode: %d", mode);
  PCAM_DEBUG("<=PCAM=> sensor mode is %s\n", (mode == SENSOR_PREVIEW_MODE)?\
             "SENSOR_PREVIEW_MODE" : "SENSOR_SNAPSHOT_MODE");
  switch (mode)
  {
    case SENSOR_PREVIEW_MODE:
      s5k5ccaf_set_preview();
    break;
    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:        
      s5k5ccaf_set_capture();
    break;
    
    default:
      return -EINVAL;
  }
  return 0;
}

static int s5k5ccaf_sensor_init_probe()
{
  int rc = 0;
  
  PCAM_DEBUG("[S5K5CCAF]s5k5ccaf_sensor_init_probe()\n");

#ifndef CONFIG_LOAD_FILE
//  S5K5CCAF_WRITE_LIST(s5k5ccaf_pre_init0);
  rc = s5k5ccaf_sensor_burst_write(s5k5ccaf_pre_init0, \
                                   (sizeof(s5k5ccaf_pre_init0) / sizeof(s5k5ccaf_pre_init0[0])),\
                                   "s5k5ccaf_pre_init0");
  msleep(10);
#endif

#if 0
  unsigned short id = 0;
  s5k5ccaf_sensor_write(0x002C, 0x0000);
  s5k5ccaf_sensor_write(0x002E, 0x0040);
  s5k5ccaf_sensor_read(0x0F12, &id);
  PCAM_DEBUG("<=PCAM=> [ANCORA+] CURRENT SENSOR ID => id 0x%x \n", id);
#endif

  return rc;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static char *s5k5ccaf_regs_table = NULL;
static int s5k5ccaf_regs_table_size;

void s5k5ccaf_regs_table_init(void)
{
  struct file *filp;
  char *dp;
  long l;
  loff_t pos;
  int ret;
  mm_segment_t fs = get_fs();
  
  printk("%s %d\n", __func__, __LINE__);
  set_fs(get_ds());
#if 1
	filp = filp_open("/data/camera/s5k5ccaf.h", O_RDONLY, 0);
#else
	filp = filp_open("/mnt/sdcard/external_sd/s5k5ccaf.h", O_RDONLY, 0);
#endif
  if (IS_ERR(filp)) {
    PCAM_DEBUG("file open error\n");
    return;
  }

  l = filp->f_path.dentry->d_inode->i_size;	
  PCAM_DEBUG("l = %ld\n", l);
  dp = vmalloc(l);
  if (dp == NULL) {
    PCAM_DEBUG("Out of Memory\n");
    filp_close(filp, current->files);
  }
  pos = 0;
  memset(dp, 0, l);
  ret = vfs_read(filp, (char __user *)dp, l, &pos);
  if (ret != l) {
    PCAM_DEBUG("Failed to read file ret = %d\n", ret);
    vfree(dp);
    filp_close(filp, current->files);
    return;
  }
  filp_close(filp, current->files);
  set_fs(fs);
  
  s5k5ccaf_regs_table = dp;
  s5k5ccaf_regs_table_size = l;
  *((s5k5ccaf_regs_table + s5k5ccaf_regs_table_size) - 1) = '\0';
//	PCAM_DEBUG("s5k5ccaf_regs_table 0x%x, %ld\n", dp, l);
}

void s5k5ccaf_regs_table_exit(void)
{
  PCAM_DEBUG("%s %d\n", __func__, __LINE__);
  if (s5k5ccaf_regs_table) {
    vfree(s5k5ccaf_regs_table);
    s5k5ccaf_regs_table = NULL;
  }	
}

static int s5k5ccaf_regs_table_write(char *name)
{
  char *start, *end, *reg;//, *data;	
  unsigned short addr, value;
  char reg_buf[7], data_buf[7];
  
  addr = value = 0;

  if(s5k5ccaf_regs_table == NULL) return 0;
  *(reg_buf + 6) = '\0';
  *(data_buf + 6) = '\0';
  
  start = strstr(s5k5ccaf_regs_table, name);
  end = strstr(start, "};");
 
  while (1) {	
    /* Find Address */	
    reg = strstr(start,"{0x");		
    if (reg)
      start = (reg + 16);
    if ((reg == NULL) || (reg > end))
    break;
    /* Write Value to Address */
    if (reg != NULL) {
      memcpy(reg_buf, (reg + 1), 6);	
      memcpy(data_buf, (reg + 9), 6);	
      addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
      value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
      //			PCAM_DEBUG("addr 0x%04x, value 0x%04x\n", addr, value);
      if (addr == 0xffff)
      {
        msleep(value);
        PCAM_DEBUG("delay 0x%04x, value 0x%04x\n", addr, value);
      }	
      else
      {
        if( s5k5ccaf_sensor_write(addr, value) < 0 )
        {
          PCAM_DEBUG("<=PCAM=> %s fail on sensor_write\n", __func__);
        }
      }
    }
    else
      PCAM_DEBUG("<=PCAM=> EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
  }
  
  return 0;
}
#endif

//latin_cam : support anti-banding
cam_antibanding_setting s5k5ccaf_antibanding = CAM_ANTIBANDING_60HZ;

static int s5k5ccaf_get_anti_banding(void)
{
  return s5k5ccaf_antibanding;
}

static int s5k5ccaf_reg_init_60hz(void)
{
  PCAM_DEBUG("Init Table for Anti-banding 60 Hz\n");
  S5K5CCAF_WRITE_LIST(s5k5ccaf_init_60hz);
}

static int s5k5ccaf_regs_table_init_for_antibanding(void)
{
  int rc;
  PCAM_DEBUG("s5k5ccaf_regs_table_init_for_antibanding() : anti-banding ==> %d Hz\n", s5k5ccaf_get_anti_banding());
  
  if (CAM_ANTIBANDING_60HZ == s5k5ccaf_get_anti_banding())
    rc = s5k5ccaf_reg_init_60hz();
  return rc;
}

ssize_t s5k5ccaf_antibanding_show (struct device *dev, struct device_attribute *attr, char *buf)
{
 	int count;
 	count = sprintf(buf, "%d", s5k5ccaf_antibanding);
 	PCAM_DEBUG("The value of s5k5ccaf_antibanding is %d\n", s5k5ccaf_antibanding);
 	return count;
}

ssize_t s5k5ccaf_antibanding_store (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  int tmp = 0;
  
  sscanf(buf, "%d", &tmp);
  if ((CAM_ANTIBANDING_50HZ == (cam_antibanding_setting) tmp)
      || (CAM_ANTIBANDING_60HZ == (cam_antibanding_setting) tmp)) {
    s5k5ccaf_antibanding = (cam_antibanding_setting) tmp;
    PCAM_DEBUG("The s5k5ccaf_antibanding is set to %d\n",s5k5ccaf_antibanding);
  }
  
  return count;
}

static struct device_attribute s5k5ccaf_antibanding_attr = {
	.attr = {
		.name = "anti-banding",
		.mode = (S_IRUGO /*| S_IWUGO*/)}, //CTS test failed removed the write permissions for the UGO  use this incase for future S_IWGRP | S_IWUSR
	.show = s5k5ccaf_antibanding_show,
	.store = s5k5ccaf_antibanding_store
};

int s5k5ccaf_sensor_init(const struct msm_camera_sensor_info *data)
{
  int rc = 0;
  int i = 0;

  PCAM_DEBUG("%s\n", __func__);

BEGIN_TIME_STAMP(s5k5ccaf_sensor_init);

  s5k5ccaf_ctrl = kzalloc(sizeof(struct s5k5ccaf_ctrl), GFP_KERNEL);
  if (!s5k5ccaf_ctrl) {
    CDBG("s5k5ccaf_init failed!\n");
    rc = -ENOMEM;
    goto init_done;
  }
  
  if (data)
    s5k5ccaf_ctrl->sensordata = data;

  /* Input MCLK = 24MHz */  
  msm_camio_clk_rate_set(24000000);
  msleep(5);
  msm_camio_camif_pad_reg_reset();

  s5k5ccaf_pw_enable(true);
  rc = s5k5ccaf_sensor_init_probe();
  if(rc < 0)
  {
    for(i=0; i<5; i++)
    {
      printk("[S5K5CCAF]cam_fw_init failed. try again.\n");
      s5k5ccaf_pw_enable(false);
      msleep(50);
      s5k5ccaf_pw_enable(true);
      msm_camio_camif_pad_reg_reset();
      msleep(5);
      rc = s5k5ccaf_sensor_init_probe();
      probe_init_retry++;
      if(rc >= 0)break;
    }
    if(rc < 0)goto init_fail;
  }

FINISH_TIME_STAMP(s5k5ccaf_sensor_init);

#ifdef CONFIG_LOAD_FILE
  s5k5ccaf_regs_table_init();
#endif	

init_done:
  return rc;

init_fail:
  kfree(s5k5ccaf_ctrl);
  return rc;
}

static int s5k5ccaf_init_client(struct i2c_client *client)
{
  /* Initialize the MSM_CAMI2C Chip */
  init_waitqueue_head(&s5k5ccaf_wait_queue);
  return 0;
}

int s5k5ccaf_sensor_config(void __user *argp)
{
  struct sensor_cfg_data cfg_data;
  long   rc = 0;

  if (copy_from_user(&cfg_data,
     (void *)argp,
     sizeof(struct sensor_cfg_data)))
     return -EFAULT;

	/* down(&s5k5ccaf_sem); */

  CDBG("s5k5ccaf_ioctl, cfgtype = %d, mode = %d\n",
  cfg_data.cfgtype, cfg_data.mode);
  
  switch (cfg_data.cfgtype) {
    case CFG_SET_MODE:
      rc = s5k5ccaf_set_sensor_mode(
      cfg_data.mode);
      break;
    
    case CFG_SET_EFFECT:
      rc = s5k5ccaf_set_effect(cfg_data.mode,
      cfg_data.cfg.effect);
      break;
    
    case CFG_GET_AF_MAX_STEPS:
    default:
      rc = -EINVAL;
      break;
  }
  
	/* up(&s5k5ccaf_sem); */
  return rc;
}

int s5k5ccaf_sensor_release(void)
{
  int rc = 0;
  
  first_start_camera = 1;
  set_init0 = 0;
  
  //If did not init below that, it can keep the previous status. it depend on concept by PCAM
  flash_mode = EXT_CFG_FLASH_OFF;
  afcanceled = true;
  flash_status = FLASH_OFF;

  mEffect = 0;
  mBrightness = 0;
  mContrast = 0;
  mSaturation = 0;
  mSharpness = 0;
  mWhiteBalance = 0;
  mISO = 0;
  mAutoExposure = 0;
  mScene = 0;
  //mAfMode = 0;
  mDTP = 0;
  mInit = 0;
  
  if(mAfMode == EXT_CFG_AF_SET_MACRO)
  {
    PCAM_DEBUG("<=PCAM=> wait change lens position\n");
    mAfMode = EXT_CFG_AF_SET_NORMAL;
    sensor_af_control(EXT_CFG_AF_SET_NORMAL);
    msleep(150);
  }
  else
    PCAM_DEBUG("<=PCAM=> mAfMode : %d\n", mAfMode);
  
  PCAM_DEBUG("<=PCAM=> s5k5ccaf_sensor_release\n");
  
  kfree(s5k5ccaf_ctrl);
  /* up(&s5k5ccaf_sem); */
  
#ifdef CONFIG_LOAD_FILE
  s5k5ccaf_regs_table_exit();
#endif
  
  s5k5ccaf_pw_enable(0);	// off camera LDOs
  return rc;
}

static int cam_pm_lp8720_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    PCAM_DEBUG(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        PCAM_DEBUG(KERN_ERR "[CAMDRV/CE147] CE147: %s: lp8720 i2c probe failed\n", __func__);
        return -1;
    }

    lp8720_i2c_client = client;
    PCAM_DEBUG("lp8720 i2c probe successful\n");
    return 0;
}

static int cam_pm_lp8720_remove(struct i2c_client *client)
{
    PCAM_DEBUG(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    lp8720_i2c_client = NULL;
    return 0;
}

static const struct i2c_device_id cam_pm_lp8720_id[] = {
    { "cam_pm_lp8720", 0 },
    { }
};

static struct i2c_driver lp8720_i2c_driver = {
    .id_table     = cam_pm_lp8720_id,
    .probe      = cam_pm_lp8720_probe,
    .remove     = cam_pm_lp8720_remove,
    .driver     = {
        .name = "cam_pm_lp8720",
    },
};


static int s5k5ccaf_i2c_probe(struct i2c_client *client,
	                              const struct i2c_device_id *id)
{
  int rc = 0;
  
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    rc = -ENOTSUPP;
    goto probe_failure;
  }
  
  s5k5ccaf_sensorw =
  kzalloc(sizeof(struct s5k5ccaf_work), GFP_KERNEL);
  
  if (!s5k5ccaf_sensorw) {
    rc = -ENOMEM;
    goto probe_failure;
  }
  
  i2c_set_clientdata(client, s5k5ccaf_sensorw);
  s5k5ccaf_init_client(client);
  s5k5ccaf_client = client;

#ifdef USE_FLASHOFF_TIMER
    init_timer(&flashoff_timer);
    flashoff_timer.function = s5k5ccaf_flashoff_timer_handler;
    flashoff_timer.expires = jiffies + 60*HZ;
#endif

  CDBG("s5k5ccaf_probe succeeded!\n");
  
  return 0;

probe_failure:
  kfree(s5k5ccaf_sensorw);
  s5k5ccaf_sensorw = NULL;
  CDBG("s5k5ccaf_probe failed!\n");
  return rc;
}

static int __exit s5k5ccaf_i2c_remove(struct i2c_client *client)
{
  struct s5ka3dfx_work_t *sensorw = i2c_get_clientdata(client);
  free_irq(client->irq, sensorw);
  //i2c_detach_client(client);
  s5k5ccaf_client = NULL;
  s5k5ccaf_sensorw = NULL;
  kfree(sensorw);
  return 0;
}

static const struct i2c_device_id s5k5ccaf_i2c_id[] = {
  { "s5k5ccaf", 0},
  { },
};

static struct i2c_driver s5k5ccaf_i2c_driver = {
  .id_table = s5k5ccaf_i2c_id,
  .probe  = s5k5ccaf_i2c_probe,
  .remove = __exit_p(s5k5ccaf_i2c_remove),
  .driver = {
    .name = "s5k5ccaf",
  },
};

// FACTORY_TEST START
ssize_t camtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    PCAM_DEBUG("<=PCAM=> BVP: camtype_show START");
    char *sensorname = "NG";
#if 0
    if( camera_back_check ){
        switch (camera_active_type)
        {
            case CAMERA_ID_BACK:
                sensorname = "SILI_SR030PC30_NONE";
                break;
            case CAMERA_ID_MAX:
                sensorname = "SLSI_S5K5CCGX_NONE";
                break;
            default :
                 sensorname = "NG";
                 break;
        }
    }
#endif
    sensorname = "SLSI_S5K5CCAF_NONE";
    return sprintf(buf,"%s\n", sensorname);
}
// FACTORY_TEST END

static DEVICE_ATTR(camtype,0644, camtype_show, NULL);

static int s5k5ccaf_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
  int rc = i2c_add_driver(&s5k5ccaf_i2c_driver);
  if (rc < 0 || s5k5ccaf_client == NULL) {
    rc = -ENOTSUPP;
    goto probe_done;
  }

  PCAM_DEBUG("ANCORA REV 0.%d", (board_hw_revision == 0x03)? 0 : 1);
  
    // FACTORY_TEST START
        if (sec_cam_dev == NULL)
        {
            sec_cam_dev = device_create(sec_class, NULL, 0, NULL, "sec_cam");
            if (IS_ERR(sec_cam_dev))
                PCAM_DEBUG("<=PCAM=> __s5k5ccaf_probe() : sec_cam failed");
        }

        if (sec_cam_dev != NULL)
        {
            if (device_create_file(sec_cam_dev, &dev_attr_camtype) < 0)
                PCAM_DEBUG("<=PCAM=> __s5k5ccaf_probe() : camtype failed");
        }
    // FACTORY_TEST END
  
  if(board_hw_revision == 0x03) /* REV 0.0 */
  {
    s5k5ccaf_pw_enable = s5k5ccaf_pw_enable_rev00;

    rc = i2c_add_driver(&lp8720_i2c_driver);
    if (rc < 0) {
      PCAM_DEBUG(KERN_ERR "[CAMDRV/S5K5CCAF]%s lp8720 probe failed\n", __func__);
      rc = -ENOTSUPP;
      i2c_del_driver(&s5k5ccaf_i2c_driver);
      i2c_del_driver(&lp8720_i2c_driver);
      goto probe_done;
    }
  }
  else// if(board_hw_revision == 0x01) /* REV 0.1 */
  {
    s5k5ccaf_pw_enable = s5k5ccaf_pw_enable_rev01;
  }

  /* Initialize variables */
  flash_mode = EXT_CFG_FLASH_OFF;
  afcanceled = true;
  flash_status = FLASH_OFF;
  camera_mode = EXT_CFG_CAM_MODE_CAMERA;
  if (rc < 0)
    goto probe_done;

  /*sensor on/off for vfe initailization */
  s5k5ccaf_pw_enable(true);
  rc = s5k5ccaf_sensor_init_probe();

  s->s_init = s5k5ccaf_sensor_init;
  s->s_release = s5k5ccaf_sensor_release;
  s->s_config  = s5k5ccaf_sensor_config;
  s->s_camera_type = BACK_CAMERA_2D;
  s->s_mount_angle  = 0;

  s5k5ccaf_pw_enable(false);

  probe_done:
    CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
  
  return rc;
}

#if 0
ssize_t camtype_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}
#endif

static int __s5k5ccaf_probe(struct platform_device *pdev)
{
  int rv;
  rv = device_create_file(&pdev->dev,&s5k5ccaf_antibanding_attr);
  if(rv)	
    PCAM_DEBUG("<=PCAM=> s5k5ccaf_probe() : device_create_file() is failed with %d", rv);
  
  return msm_camera_drv_start(pdev, s5k5ccaf_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
  .probe = __s5k5ccaf_probe,
  .driver = {
    .name = "msm_camera_s5k5ccaf",
    .owner = THIS_MODULE,
  },
};

static int __init s5k5ccaf_init(void)
{
  return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5ccaf_init);

