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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <media/msm_camera.h>
#include <linux/gpio.h>

#if defined(CONFIG_MACH_ANCORA)
#include "s5k4ecgx.h"
#elif defined(CONFIG_MACH_APACHE)
#include "s5k4ecgx_I847.h"
#endif

#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>

#define SENSOR_DEBUG    (0)

#define SENSOR_SECCESS  (0)

#undef CONFIG_LOAD_FILE
//#define CONFIG_LOAD_FILE
#ifndef CONFIG_LOAD_FILE
#define S5K4ECGX_USE_BURSTMODE
//#define WORKAROUND_FOR_LOW_SPEED_I2C
#endif

#ifdef  S5K4ECGX_USE_BURSTMODE
#define S5K4ECGX_WRITE_LIST(A) \
    s5k4ecgx_sensor_burst_write(A,(sizeof(A) / sizeof(A[0])),#A);
#else
#define S5K4ECGX_WRITE_LIST(A) \
    s5k4ecgx_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#endif

#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_APACHE)
#define CAM_FLASH_ENSET 57
#define CAM_FLASH_FLEN 56
#else
#define CAM_FLASH_ENSET 56
#define CAM_FLASH_FLEN 57
#endif
#define FULL_FLASH 20
#define PRE_FLASH 7
#if defined(CONFIG_MACH_ANCORA) || defined(CONFIG_MACH_APACHE)
#define MOVIE_FLASH 3 // 109 mA
#else
#define MOVIE_FLASH 7
#endif
#define MACRO_FLASH 14
#define PRE_FLASH_OFF -1
#define FLASH_OFF 0

#define PREVIEW 1
#define SNAPSHOT 2

#define CHECK_EFFECT 0x00000001
#define CHECK_BRIGHTNESS 0x00000002
#define CHECK_CONTRAST 0x00000004
#define CHECK_SATURATION 0x00000008
#define CHECK_SHARPNESS 0x00000010
#define CHECK_WB 0x00000020
#define CHECK_ISO 0x00000040
#define CHECK_AE 0x00000080
#define CHECK_SCENE 0x00000100
#define CHECK_AFMODE 0x00000200
#define CHECK_DTP 0x00000400
#define CHECK_SNAPSHOT_SIZE 0x00000800
#define CHECK_PREVIEW_SIZE 0x00001000
#define CHECK_ZOOM 0x00002000
#define CHECK_JPEGQUALITY 0x00004000
#define CHECK_AUTOCONTRAST 0x00008000
#define CHECK_FPS 0x00010000
#define CHECK_AE_AWB_LOCK 0x00020000

struct s5k4ecgx_status_struct {
    char camera_initailized;//  1 is not init a sensor
    u32 need_configuration;
    char camera_mode;
    char effect;
    int brightness;
    char contrast;
    char saturation;
    char sharpness;
    char whiteBalance;
    char iso;
    char auto_exposure;
    char scene;
    char afmode;
    char afcanceled;
    char dtp;
    char snapshot_size;
    char preview_size;
    char flash_mode;
    char flash_exifinfo;
    char zoom;
    char lowcap_on;
    char nightcap_on;
    char power_on;
    char fps;
    char ae_lock;
    char awb_lock;
    char camera_status;
    char camera_on; // true:ON, false:OFF
    int flash_status;
    int jpeg_quality;
    int auto_contrast;
    int current_lux;
    unsigned short id;
};

struct s5k4ecgx_enum_framesize {
	/* mode is 0 for preview, 1 for capture */
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

static char mMode = 0;
static struct s5k4ecgx_enum_framesize s5k4ecgx_framesize_list[] = { //TELECA_TOUCHAF
{ EXT_CFG_PREVIEW_SIZE_640x480_VGA,  640, 480 },
{ EXT_CFG_PREVIEW_SIZE_720x480_D1,   720, 480 },
{ EXT_CFG_PREVIEW_SIZE_800x480_WVGA, 800, 480 },
{ EXT_CFG_PREVIEW_SIZE_1280x720_D1,  1280, 720 },
{ EXT_CFG_PREVIEW_SIZE_320x240_QVGA, 320, 240 },
{ EXT_CFG_PREVIEW_SIZE_176x144_QCIF, 176, 144 },
{ 0, 0, 0},
};



static struct s5k4ecgx_status_struct s5k4ecgx_status;

bool isPreviewReturnWrite = false;
static unsigned int i2c_retry = 0;
static unsigned int probe_init_retry = 0;
static int HD_mode = 0;
int b_esd_detected=false;
static int flashOnFromApps = 0;     // Set to 1 only if flash is turned on by EXT_CFG_SET_FLASH_MODE from HAL

struct s5k4ecgx_work {
    struct work_struct work;
};

static struct s5k4ecgx_work *s5k4ecgx_sensorw;
static struct i2c_client *s5k4ecgx_client;
struct i2c_client *lp8720_i2c_client;

struct s5k4ecgx_ctrl {
    const struct msm_camera_sensor_info *sensordata;
};

static struct s5k4ecgx_ctrl *s5k4ecgx_ctrl;
#ifdef USE_FLASHOFF_TIMER
static struct timer_list flashoff_timer;
#endif

#define FACTORY_CHECK
#ifdef FACTORY_CHECK
static bool camtype_init = false;
static bool main_camera_is_on = false;
#endif

static DECLARE_WAIT_QUEUE_HEAD(s5k4ecgx_wait_queue);
DECLARE_MUTEX(s5k4ecgx_sem);

/*=============================================================
    EXTERNAL DECLARATIONS
==============================================================*/
extern struct s5k4ecgx_reg s5k4ecgx_regs;
extern int cpufreq_direct_set_policy(unsigned int cpu, const char *buf);
#ifdef WORKAROUND_FOR_LOW_SPEED_I2C
extern void pcam_msm_i2c_pwr_mgmt(struct i2c_adapter *adap, int on);
#endif
extern int* get_i2c_clock_addr(struct i2c_adapter *adap);
/*=============================================================*/

static int s5k4ecgx_probe_init_sensor(void);
void s5k4ecgx_set_power(int status);

#ifdef CONFIG_LOAD_FILE
    static int s5k4ecgx_regs_table_write(char *name);
#endif

static int s5k4ecgx_sensor_read(unsigned short subaddr, unsigned short *data)
{
    int ret;
    unsigned char buf[2];
    struct i2c_msg msg = { s5k4ecgx_client->addr, 0, 2, buf };
    
    buf[0] = (subaddr >> 8);
    buf[1] = (subaddr & 0xFF);
    
    if(!s5k4ecgx_status.power_on)return -1;
    
    ret = i2c_transfer(s5k4ecgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    msg.flags = I2C_M_RD;
    
    ret = i2c_transfer(s5k4ecgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    *data = ((buf[0] << 8) | buf[1]);
    /*  [Arun c]Data should be written in Little Endian in parallel mode; So there is no need for byte swapping here */
    //*data = *(unsigned long *)(&buf);
error:
    return ret;
}

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
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
        printk(KERN_ERR "[CAMDRV/CE147] %s: lp8720_i2c_write failed: %d\n",__func__, rc);        
    
    return (rc == 1) ? 0 : -EIO;
}

static int s5k4ecgx_sensor_write(unsigned short subaddr, unsigned short val)
{
    unsigned char buf[4];
    struct i2c_msg msg = { s5k4ecgx_client->addr, 0, 4, buf };

    //CAMDRV_DEBUG("[PGH] on write func s5k4ecgx_client->addr : %x\n", s5k4ecgx_client->addr);
    //CAMDRV_DEBUG("[PGH] on write func  s5k4ecgx_client->adapter->nr : %d\n", s5k4ecgx_client->adapter->nr);

    buf[0] = (subaddr >> 8);
    buf[1] = (subaddr & 0xFF);
    buf[2] = (val >> 8);
    buf[3] = (val & 0xFF);

    return i2c_transfer(s5k4ecgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int s5k4ecgx_sensor_write_list(struct s5k4ecgx_reg *list,int size, char *name)
{
    int ret, i;
    unsigned short subaddr;
    unsigned short value;

    if(!s5k4ecgx_status.power_on)return -1;
    
#ifdef CONFIG_LOAD_FILE
    ret = s5k4ecgx_regs_table_write(name);
#else
    CAMDRV_DEBUG("s5k4ecgx_sensor_write_list : %s\n",name);
      
    for (i = 0; i < size; i++)
    {
        //CAMDRV_DEBUG("[PGH] %x      %x\n", list[i].subaddr, list[i].value);
        subaddr = ((list[i].value)>> 16); //address
        value = ((list[i].value)& 0xFFFF); //value
        if(subaddr == 0xffff)
        {
            CAMDRV_DEBUG("SETFILE DELAY : %dms\n",value);
            msleep(value);
        }
        else
        {
            if(s5k4ecgx_sensor_write(subaddr, value) < 0)
            {
                printk("[S5K4ECGX]sensor_write_list fail...-_-\n");
                return -1;
            }
        }
    }
#endif
    return ret;
}

#ifdef S5K4ECGX_USE_BURSTMODE
#define BURST_MODE_BUFFER_MAX_SIZE 2700
unsigned char s5k4ecgx_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];
static int s5k4ecgx_sensor_burst_write(struct s5k4ecgx_reg *list, int size, char *name)
{
    int i = 0;
    int idx = 0;
    int err = 0;
    int retry = 0;
    unsigned short subaddr=0,next_subaddr=0;
    unsigned short value=0;
    struct i2c_msg msg = { s5k4ecgx_client->addr, 0, 0, s5k4ecgx_buf_for_burstmode };

    CAMDRV_DEBUG("s5k4ecgx_sensor_burst_write : %s(size:%d)\n",name,size);

I2C_RETRY:
    idx = 0;
    for (i = 0; i < size; i++) 
    {
        if(idx > (BURST_MODE_BUFFER_MAX_SIZE-10))
        {
            printk("[S5K4ECGX]s5k4ecgx_buf_for_burstmode overflow will occur!!!\n");
            return err;
        }
            
        subaddr = ((list[i].value)>> 16); //address
        if(subaddr == 0x0F12) next_subaddr= ((list[i+1].value)>> 16); //address
        value = ((list[i].value)& 0xFFFF); //value
        
        switch(subaddr)
        {
            case 0x0F12 :
                // make and fill buffer for burst mode write
                if(idx ==0) 
                {
                    s5k4ecgx_buf_for_burstmode[idx++] = 0x0F;
                    s5k4ecgx_buf_for_burstmode[idx++] = 0x12;
                }
                s5k4ecgx_buf_for_burstmode[idx++] = value>> 8;
                s5k4ecgx_buf_for_burstmode[idx++] = value & 0xFF;
                 //write in burstmode    
                if(next_subaddr != 0x0F12)
                {
                    msg.len = idx;
                    err = i2c_transfer(s5k4ecgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
//                    printk("s5k4ecgx_sensor_burst_write, idx = %d\n",idx);
                    idx=0;
                }
            break;
            case 0xFFFF :
                printk("s5k4ecgx_sensor_burst_write, sleep %dms\n",value);
                msleep(value);
            break;
            default:
                // Set Address
                idx=0;
                err = s5k4ecgx_sensor_write(subaddr,value);
//                printk("s5k4ecgx_sensor_burst_write, %x,%x\n",subaddr,value);
            break;
        }
        if(err < 0)
        {
            printk("[S5K4ECGX]%s: register set failed. try again.\n",__func__);
            retry++;
            msleep(100);
            if((retry++)<10) goto I2C_RETRY;

            return err;
        }
    }
    
    //CAMDRV_DEBUG("s5k4ecgx_sensor_burst_write end!\n");
    return 0;
}
#endif /*S5K4ECGX_USE_BURSTMODE*/

void s5k4ecgx_set_effect(char value)
{
    printk("********[diony] value = %d,.*****\n",value);
    switch(value)
    {
        case EXT_CFG_EFFECT_NORMAL :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Effect_Normal);
        break;
        case EXT_CFG_EFFECT_NEGATIVE :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Effect_Negative);
        break;
        case EXT_CFG_EFFECT_MONO :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Effect_Black_White);
        break;
        case EXT_CFG_EFFECT_SEPIA :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Effect_Sepia);
        break;
        default :
            printk("[S5K4ECGX]Unexpected Effect mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(int bit, int set)
{
    int REG_TC_DBG_AutoAlgEnBits = 0;
    
    /* Read 04E6 */
    s5k4ecgx_sensor_write(0x002C, 0x7000);
    s5k4ecgx_sensor_write(0x002E, 0x04E6);
    s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&REG_TC_DBG_AutoAlgEnBits);

    if(bit == 3 && set == true)
    {
        if(REG_TC_DBG_AutoAlgEnBits & 0x8 == 1)return;
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
        else mdelay(100);
        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits | 0x8;
        s5k4ecgx_sensor_write(0x0028, 0x7000);
        s5k4ecgx_sensor_write(0x002A, 0x04E6);
        s5k4ecgx_sensor_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    }
    else if(bit == 3 && set == false)
    {
        if(REG_TC_DBG_AutoAlgEnBits & 0x8 == 0)return;
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
        else mdelay(100);
        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits & 0xFFF7;
        s5k4ecgx_sensor_write(0x0028, 0x7000);
        s5k4ecgx_sensor_write(0x002A, 0x04E6);
        s5k4ecgx_sensor_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    }
    else if(bit == 5 && set == true)
    {
        if(REG_TC_DBG_AutoAlgEnBits & 0x20 == 1)return;
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
        else mdelay(100);
        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits | 0x20;
        s5k4ecgx_sensor_write(0x0028, 0x7000);
        s5k4ecgx_sensor_write(0x002A, 0x04E6);
        s5k4ecgx_sensor_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    }
    else if(bit == 5 && set == false)
    {
        if(REG_TC_DBG_AutoAlgEnBits & 0x20 == 0)return;
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
        else mdelay(100);
        REG_TC_DBG_AutoAlgEnBits = REG_TC_DBG_AutoAlgEnBits & 0xFFDF;
        s5k4ecgx_sensor_write(0x0028, 0x7000);
        s5k4ecgx_sensor_write(0x002A, 0x04E6);
        s5k4ecgx_sensor_write(0x0F12, REG_TC_DBG_AutoAlgEnBits);
    }

    return;
}

void s5k4ecgx_set_whitebalance(char value)
{
    switch(value)
    {
        case EXT_CFG_WB_AUTO :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,1);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_WB_Auto);
        break;
        case EXT_CFG_WB_DAYLIGHT :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_WB_Sunny);
        break;
        case EXT_CFG_WB_CLOUDY :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_WB_Cloudy);
        break;
        case EXT_CFG_WB_FLUORESCENT :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_WB_Fluorescent);
        break;
        case EXT_CFG_WB_INCANDESCENT :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_WB_Tungsten);
        break;
        default :
            printk("[S5K4ECGX]Unexpected WB mode : %d\n",  value);
        break;
        }
}

void s5k4ecgx_set_brightness(int value)
{
    switch(value)
    {
        case EXT_CFG_BR_STEP_P_4 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Plus_4);
        break;
        case EXT_CFG_BR_STEP_P_3 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Plus_3);
        break;
        case EXT_CFG_BR_STEP_P_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Plus_2);
        break;
        case EXT_CFG_BR_STEP_P_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Plus_1);
        break;
        case EXT_CFG_BR_STEP_0 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Default);
        break;
        case EXT_CFG_BR_STEP_M_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Minus_1);
        break;
        case EXT_CFG_BR_STEP_M_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Minus_2);
        break;
        case EXT_CFG_BR_STEP_M_3 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Minus_3);
        break;
        case EXT_CFG_BR_STEP_M_4 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_EV_Minus_4);
        break;
        default :
            printk("[S5K4ECGX]Unexpected BR mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_iso(char value)
{
    switch(value)
    {
        case EXT_CFG_ISO_AUTO :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,1);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ISO_Auto);
        break;
        case EXT_CFG_ISO_50 :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ISO_50);
        break;
        case EXT_CFG_ISO_100 :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ISO_100);
        break;
        case EXT_CFG_ISO_200 :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ISO_200);
        break;
        case EXT_CFG_ISO_400 :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ISO_400);
        break;
        default :
            printk("[S5K4ECGX]Unexpected ISO mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_metering(char value)
{
    switch(value)
    {
        case EXT_CFG_METERING_NORMAL :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Metering_Matrix);
        break;
        case EXT_CFG_METERING_SPOT :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Metering_Spot);
        break;
        case EXT_CFG_METERING_CENTER :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Metering_Center);
        break;
        default :
            printk("[S5K4ECGX]Unexpected METERING mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_contrast(char value)
{
    switch(value)
    {
        case EXT_CFG_CR_STEP_M_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Contrast_Minus_2);
        break;
        case EXT_CFG_CR_STEP_M_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Contrast_Minus_1);
        break;
        case EXT_CFG_CR_STEP_0 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Contrast_Default);
        break;
        case EXT_CFG_CR_STEP_P_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Contrast_Plus_1);
        break;
        case EXT_CFG_CR_STEP_P_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Contrast_Plus_2);
        break;
        default :
            printk("[S5K4ECGX]Unexpected EXT_CFG_CR_CONTROL mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_zoom(char value)
{
    switch(value)
    {
        case EXT_CFG_ZOOM_STEP_0 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_0);
        break;
        case EXT_CFG_ZOOM_STEP_1 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_1);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:                            
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_1);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_1);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_1);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_1);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_1);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_1);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_2 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_2);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_2);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_2);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_2);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_2);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_2);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_2);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_3 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_3);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:                        
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_3);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_3);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_3);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_3);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_3);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_3);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_4 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_4);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_4);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_4);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_4);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_4);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_4);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_4);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_5 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_5);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_5);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_5);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_5);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_5);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_5);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_5);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_6 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_6);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:                        
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_6);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_6);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_6);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_6);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_6);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_6);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_7 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_7);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:                                
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_7);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_7);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_7);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_7);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_7);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_7);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        case EXT_CFG_ZOOM_STEP_8 :
            if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_CAMERA)
            {
                switch(s5k4ecgx_status.snapshot_size)
                {
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_25_Zoom_8);break;
                    case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
                    case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:                            
                    case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
                    case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:                        
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_6_Zoom_8);break;
                    case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
                        S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_8);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
            else
            {
                switch(s5k4ecgx_status.preview_size)
                {
                    case EXT_CFG_PREVIEW_SIZE_720x480_D1: S5K4ECGX_WRITE_LIST(s5k4ecgx_X1_77_Zoom_8);break;
                    case EXT_CFG_PREVIEW_SIZE_640x480_VGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_8);break;
                    case EXT_CFG_PREVIEW_SIZE_320x240_QVGA: S5K4ECGX_WRITE_LIST(s5k4ecgx_X2_Zoom_8);break;
                    case EXT_CFG_PREVIEW_SIZE_176x144_QCIF: S5K4ECGX_WRITE_LIST(s5k4ecgx_X4_Zoom_8);break;
                    default: printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);break;
                }
            }
        break;
        default :
            printk("[S5K4ECGX]Unexpected ZOOM mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_saturation(char value)
{
    switch(value)
    {
        case EXT_CFG_SA_STEP_M_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Saturation_Minus_2);
        break;
        case EXT_CFG_SA_STEP_M_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Saturation_Minus_1);
        break;
        case EXT_CFG_SA_STEP_0 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Saturation_Default);
        break;
        case EXT_CFG_SA_STEP_P_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Saturation_Plus_1);
        break;
        case EXT_CFG_SA_STEP_P_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Saturation_Plus_2);
        break;
        default :
            printk("[S5K4ECGX]Unexpected EXT_CFG_SA_CONTROL mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_sharpness(char value)
{
    switch(value)
    {
        case EXT_CFG_SP_STEP_M_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Sharpness_Minus_2);
        break;
        case EXT_CFG_SP_STEP_M_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Sharpness_Minus_1);
        break;
        case EXT_CFG_SP_STEP_0 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Sharpness_Default);
        break;
        case EXT_CFG_SP_STEP_P_1 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Sharpness_Plus_1);
        break;
        case EXT_CFG_SP_STEP_P_2 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Sharpness_Plus_2);
        break;
        default :
            printk("[S5K4ECGX]Unexpected EXT_CFG_SP_CONTROL mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_auto_contrast(char value)
{
    switch(value)
    {
        case EXT_CFG_AUTO_CONTRAST_ON :
            s5k4ecgx_set_contrast(EXT_CFG_CR_STEP_0);
            s5k4ecgx_set_saturation(EXT_CFG_SA_STEP_0);
            s5k4ecgx_set_sharpness(EXT_CFG_SP_STEP_0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Auto_Contrast_ON);
        break;
        case EXT_CFG_AUTO_CONTRAST_OFF :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Auto_Contrast_OFF);
            s5k4ecgx_set_contrast(s5k4ecgx_status.contrast);
            s5k4ecgx_set_saturation(s5k4ecgx_status.saturation);
            s5k4ecgx_set_sharpness(s5k4ecgx_status.sharpness);
        break;
        default :
            printk("[S5K4ECGX]Unexpected auto_contrast mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_jpeg_quality(char value)
{
    switch(value)
    {
        case EXT_CFG_JPEG_QUALITY_SUPERFINE :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Jpeg_Quality_High);
        break;
        case EXT_CFG_JPEG_QUALITY_FINE :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Jpeg_Quality_Normal);
        break;
        case EXT_CFG_JPEG_QUALITY_NORMAL :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Jpeg_Quality_Low);
        break;
        default :
            printk("[S5K4ECGX]Unexpected Jpeg quality mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_fps(char value)
{
    switch(value)
    {
        case EXT_CFG_FRAME_AUTO :
            //S5K4ECGX_WRITE_LIST(s5k4ecgx_FPS_15);
        break;
        case EXT_CFG_FRAME_FIX_15 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_FPS_15);
        break;
        case EXT_CFG_FRAME_FIX_20 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_FPS_20);
            msleep(100); /*delay for pclk stabilization (noframe issue) 2011-01-20 */
        break;
        case EXT_CFG_FRAME_FIX_30 :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_FPS_30);
            msleep(100); /*delay for pclk stabilization (noframe issue) 2011-01-20 */
        break;
        default :
            printk("[S5K4ECGX]Unexpected EXT_CFG_FRAME_CONTROL mode : %d\n", value);
        break;
    }
}

static int s5k4ecgx_get_lux(int* lux)
{
    int msb = 0;
    int lsb = 0;
    int cur_lux = -1;
    
    s5k4ecgx_sensor_write(0x002C, 0x7000);
    if(s5k4ecgx_status.id == 0x0011)
        s5k4ecgx_sensor_write(0x002E, 0x2C18);//for EVT 1.1
    else
        s5k4ecgx_sensor_write(0x002E, 0x2B30);
    s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&lsb);
    s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&msb);

    cur_lux = (msb<<16) | lsb;
    *lux = cur_lux;
    CAMDRV_DEBUG("%s, s5k4ecgx_status.current_lux is %d\n", __func__, s5k4ecgx_status.current_lux);
    return cur_lux; //this value is under 0x0032 in low light condition 
}

static  int s5k4ecgx_set_flash(int lux_val)
{
    int i = 0;

    CAMDRV_DEBUG("%s, flash set is %d\n", __func__, lux_val);
    
//    if(s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_OFF)return 0;

    /* initailize falsh IC */
    gpio_set_value(CAM_FLASH_ENSET,0);
    gpio_set_value(CAM_FLASH_FLEN,0);
    mdelay(1); // to enter a shutdown mode

    extern int batt_temp_adc_info;
    
    /* set to flash mode */

    if(batt_temp_adc_info>1800 && lux_val>0) // lux_val!=PRE_FLASH_OFF && lux_val!=FLASH_OFF)    // Very low temperature.
    {
        /* set to movie mode */
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Pre_Flash_On);
        
        for(i=0;i<7;i++)
        {
            udelay(1);
            gpio_set_value(CAM_FLASH_ENSET,1);
            udelay(1);
            gpio_set_value(CAM_FLASH_ENSET,0);
        }
        
        gpio_set_value(CAM_FLASH_ENSET,1); //value set
    }
    else if(lux_val>16) // FULL_FLASH
    {
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Main_Flash_On);
        gpio_set_value(CAM_FLASH_FLEN,1);
        s5k4ecgx_status.flash_exifinfo = true;
#ifdef USE_FLASHOFF_TIMER
        add_timer(&flashoff_timer); // for prevent LED 
#endif
    }
    else if(lux_val == MACRO_FLASH)
    {
        /* set to movie mode */
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Main_Flash_On);
        for(i=0;i<lux_val;i++)
        {
            udelay(1);
            gpio_set_value(CAM_FLASH_ENSET,1);
            udelay(1);
            gpio_set_value(CAM_FLASH_ENSET,0);
        }
        gpio_set_value(CAM_FLASH_ENSET,1); //value set
        s5k4ecgx_status.flash_exifinfo = true;
    }
    else if(lux_val > 0 &&  lux_val<=16) // PRE_FLASH, MOVIE_FLASH
    {
        /* set to movie mode */
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Pre_Flash_On);
        for(i=0;i<lux_val;i++)
        {
            udelay(1);
            gpio_set_value(CAM_FLASH_ENSET,1);
            udelay(1);
            gpio_set_value(CAM_FLASH_ENSET,0);
        }
        gpio_set_value(CAM_FLASH_ENSET,1); //value set
    }
    s5k4ecgx_status.flash_status = lux_val;
    
    /* setting a sensor #2*/
        if(lux_val==PRE_FLASH_OFF)S5K4ECGX_WRITE_LIST(s5k4ecgx_Pre_Flash_Off)
        else if(lux_val==FLASH_OFF && s5k4ecgx_status.afcanceled == false)S5K4ECGX_WRITE_LIST(s5k4ecgx_Main_Flash_Off)
        else if(lux_val==FLASH_OFF && s5k4ecgx_status.afcanceled == true)S5K4ECGX_WRITE_LIST(s5k4ecgx_Pre_Flash_Off)
    
    return 0;
}

#ifdef USE_FLASHOFF_TIMER
static void s5k4ecgx_flashoff_timer_handler(unsigned long data)
{
    s5k4ecgx_set_flash(FLASH_OFF);
}
#endif

#ifdef SECOND_AF_SKIP
static int g_AF_expired=1;
static struct timer_list AF_expire_timer;
static int g_timer_valid=0;

void s5k4ecgx_AF_expire_timer_handler(unsigned long data)
{
    CAMDRV_DEBUG("s5k4ecgx_AF_expire_timer_handler Expire. \n", __func__);
    g_AF_expired=1;
    g_timer_valid=1;
}
void s5k4ecgx_af_timer_restart()
{
    del_timer(&AF_expire_timer);
    init_timer(&AF_expire_timer);
    AF_expire_timer.function = s5k4ecgx_AF_expire_timer_handler;
    AF_expire_timer.expires = jiffies + 2*HZ;
    add_timer(&AF_expire_timer); // for shot to shot
    g_timer_valid=1;
}
#endif

#if defined (CONFIG_MACH_APACHE)
static unsigned short get_sensor_value(unsigned short addr)
{
  unsigned short retval;
  s5k4ecgx_sensor_write(0x002C, 0x7000);
  s5k4ecgx_sensor_write(0x002E, addr);
  s5k4ecgx_sensor_read(0x0F12, &retval);
  CAMDRV_DEBUG("<=PCAM=> get_sensor_value : %d\n", retval);
  
  return retval;
}

static void set_sensor_value(unsigned short addr, unsigned short val)
{
  s5k4ecgx_sensor_write(0xFCFC, 0xD000);
  s5k4ecgx_sensor_write(0x0028, 0x7000);
  s5k4ecgx_sensor_write(0x002A, addr);
  s5k4ecgx_sensor_write(0x0F12, val);
}

static void s5k4ecgx_set_touchAF(unsigned short touch_X, unsigned short touch_Y)
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
    CAMDRV_DEBUG("[[PCAM - 7]] preview size %d is not support!!\n", mMode);
  }
  else {
    index = mMode;
    CAMDRV_DEBUG("[[PCAM - 7]] preview idx : %d\n", mMode);
  }


  typedef struct samsung_short_t{
    unsigned short subaddr;
    unsigned short value;
  } af_short_t;

  af_short_t S5K4ECGX_TOUCH_AF[]=
  {
    {0xFCFC, 0xD000},
    {0x0028, 0x7000},
    {0x002A, 0x0294},   //AF window setting
    {0x0F12, 0x0100},	//REG_TC_AF_FstWinStartX 
    {0x0F12, 0x00E2},	//REG_TC_AF_FstWinStartY
    {0x0F12, 0x0200},	//REG_TC_AF_FstWinSizeX [5]
    {0x0F12, 0x0237},	//REG_TC_AF_FstWinSizeY [6]
    {0x0F12, 0x018B},	//REG_TC_AF_ScndWinStartX
    {0x0F12, 0x0164},	//REG_TC_AF_ScndWinStartY
    {0x0F12, 0x00E4},	//REG_TC_AF_ScndWinSizeX [9]
    {0x0F12, 0x0131},	//REG_TC_AF_ScndWinSizeY [10]
    {0x0F12, 0x0001},	//REG_TC_AF_WinSizesUpdated
    {0x0000, 0x0000}

  };
  
  /* get preview width, height */
  width = s5k4ecgx_framesize_list[index].width;
  height = s5k4ecgx_framesize_list[index].height;
  
  Mapping_H = sensor_display_H / width;
  Mapping_V = sensor_display_V / height;
  
  mapped_X = touch_X * Mapping_H; //sensor display size  mapping
  mapped_Y = touch_Y * Mapping_V; //sensor display size  mapping

  S5K4ECGX_TOUCH_AF[9].value = get_sensor_value(0x2A0);
  S5K4ECGX_TOUCH_AF[10].value = get_sensor_value(0x2A2);
  S5K4ECGX_TOUCH_AF[5].value = get_sensor_value(0x298);
  S5K4ECGX_TOUCH_AF[6].value = get_sensor_value(0x29A);

  inner_window_width = ( (S5K4ECGX_TOUCH_AF[9].value * sensor_display_H ) / 1024 );
  inner_window_height = ( (S5K4ECGX_TOUCH_AF[10].value * sensor_display_V ) /1024 );
  outer_window_width = ( (S5K4ECGX_TOUCH_AF[5].value * sensor_display_H ) /1024);
  outer_window_height = ( (S5K4ECGX_TOUCH_AF[6].value * sensor_display_V ) / 1024);

  CAMDRV_DEBUG("[[PCAM - 7]] mapped_X: %d, mapped_Y: %d\n", mapped_X, mapped_Y);
  CAMDRV_DEBUG("[[PCAM - 7]] inner_window_width: %d, inner_window_height: %d, outer_window_width: %d, outer_window_height: %d \n",\
            inner_window_width, inner_window_height, outer_window_width, outer_window_height);

  CAMDRV_DEBUG("[[PCAM - 7]] ScndWinSizeX: %d, ScndWinSizeY: %d, FstWinSizeX: %d, FstWinSizeY: %d\n",\
                      S5K4ECGX_TOUCH_AF[9].value, S5K4ECGX_TOUCH_AF[10].value,\
                      S5K4ECGX_TOUCH_AF[5].value, S5K4ECGX_TOUCH_AF[6].value);
  

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
  S5K4ECGX_TOUCH_AF[3].value = (outer_window_start_X * (1024) / sensor_display_H);
  S5K4ECGX_TOUCH_AF[4].value = (outer_window_start_Y * (1024) / sensor_display_V);
  S5K4ECGX_TOUCH_AF[7].value = (inner_window_start_X * (1024) / sensor_display_H);
  S5K4ECGX_TOUCH_AF[8].value = (inner_window_start_Y * (1024) / sensor_display_V);

  CAMDRV_DEBUG("[[PCAM - 6] outer_window_start_X: %d, outer_window_start_Y: %d, inner_window_start_X: %d, inner_window_start_Y: %d\n",\
                      outer_window_start_X, outer_window_start_Y, inner_window_start_X, inner_window_start_Y);
  CAMDRV_DEBUG("[[PCAM - 6] fisrt reg(%d, %d, %d, %d)\n",\
          S5K4ECGX_TOUCH_AF[3].value,S5K4ECGX_TOUCH_AF[4].value,\
          S5K4ECGX_TOUCH_AF[5].value,S5K4ECGX_TOUCH_AF[6].value);
  CAMDRV_DEBUG("[[PCAM - 6] second reg(%d, %d, %d, %d)\n",\
          S5K4ECGX_TOUCH_AF[7].value,S5K4ECGX_TOUCH_AF[8].value,\
          S5K4ECGX_TOUCH_AF[9].value,S5K4ECGX_TOUCH_AF[10].value);
 
  for (i=0 ; S5K4ECGX_TOUCH_AF[i].subaddr != 0; i++)
  {
    err = s5k4ecgx_sensor_write(S5K4ECGX_TOUCH_AF[i].subaddr, S5K4ECGX_TOUCH_AF[i].value);
    if(err < 0){
      printk("%s: failed: i2c_write for touch_auto_focus\n", __func__);
    }
  }

  mdelay(100); //(100ms(camera preview) or 50ms (720P) // 1frame delay
  
  set_sensor_value(0x028C, 0x0005); //single AF

}
#endif /* #if defined (CONFIG_MACH_APACHE) */

int s5k4ecgx_set_af(char value)
{
    int val = 0, ret = 0;
    static int pre_flash_on = 0;
    //CAMDRV_DEBUG("%s : %d\n", __func__, value);

#ifdef SECOND_AF_SKIP
    if(!g_AF_expired)
    {
        s5k4ecgx_af_timer_restart();
        CAMDRV_DEBUG("s5k4ecgx_set_af Skip!! \n", __func__);
        return EXT_CFG_AF_SUCCESS;
    }
#endif

    switch(value)
    {
        case EXT_CFG_AF_CHECK_STATUS :
            s5k4ecgx_sensor_write(0x002C, 0x7000);
            if(s5k4ecgx_status.id == 0x0011)
                s5k4ecgx_sensor_write(0x002E, 0x2EEE);
            else
                s5k4ecgx_sensor_write(0x002E, 0x2E06);
			
			mdelay(200);
            s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&val);
            switch(val)
            {
                case 1:
                    CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_STATUS -EXT_CFG_AF_PROGRESS \n", __func__);
                    ret = EXT_CFG_AF_PROGRESS;
                break;
                case 2:
                    CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_STATUS -EXT_CFG_AF_SUCCESS \n", __func__);                    
                    ret = EXT_CFG_AF_SUCCESS;
                break;
                default:
                    CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_STATUS -EXT_CFG_AF_LOWCONF \n", __func__);
                    ret = EXT_CFG_AF_LOWCONF;
                break;
            }
        break;
        case EXT_CFG_AF_CHECK_2nd_STATUS :
            s5k4ecgx_sensor_write(0x002C, 0x7000);
            if(s5k4ecgx_status.id == 0x0011)
                s5k4ecgx_sensor_write(0x002E, 0x2207);
            else
                s5k4ecgx_sensor_write(0x002E, 0x2167);

			mdelay(200);
            s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&val);
            switch(val&0xFF)
            {
                case 1:
                    CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_2nd_STATUS -EXT_CFG_AF_PROGRESS \n", __func__);
                    ret = EXT_CFG_AF_PROGRESS;
                break;
                case 0:
                    CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_2nd_STATUS -EXT_CFG_AF_SUCCESS \n", __func__);
#ifdef SECOND_AF_SKIP
                    s5k4ecgx_af_timer_restart(); // for shot to shot
                    g_AF_expired=0;
#endif
                    ret = EXT_CFG_AF_SUCCESS;
                    if(flashOnFromApps == 0)
                        s5k4ecgx_set_flash(PRE_FLASH_OFF);
                break;
                default:
                    CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_2nd_STATUS -EXT_CFG_AF_PROGRESS \n", __func__);
                    ret = EXT_CFG_AF_PROGRESS;
                break;
            }
        break;
        case EXT_CFG_AF_CHECK_AE_STATUS :
            {
                s5k4ecgx_sensor_write(0x002C, 0x7000);
                if(s5k4ecgx_status.id == 0x0011)
                    s5k4ecgx_sensor_write(0x002E, 0x2C74);
                else
                    s5k4ecgx_sensor_write(0x002E, 0x2B8C);

				mdelay(200);
                s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&val);
                switch(val)
                {
                    case 1:
                        CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_AE_STATUS -EXT_CFG_AE_STABLE \n", __func__);
                        ret = EXT_CFG_AE_STABLE;
                    break;
                    default:
                        CAMDRV_DEBUG("%s : EXT_CFG_AF_CHECK_AE_STATUS -EXT_CFG_AE_UNSTABLE \n", __func__);
                        ret = EXT_CFG_AE_UNSTABLE;
                    break;
                }
            }
        break;
        case EXT_CFG_AF_SET_NORMAL :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_SET_NORMAL \n", __func__);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_1);
            if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
            else mdelay(200);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_2);
            if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
            else mdelay(200);
            if(s5k4ecgx_status.scene != EXT_CFG_SCENE_NIGHTSHOT)
            {    
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_3);
                mdelay(200);
            }
        break;
        case EXT_CFG_AF_SET_MACRO :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_SET_MACRO \n", __func__);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Macro_mode_1);
            if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
            else mdelay(200);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Macro_mode_2);
            if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
            else mdelay(200);
            if(s5k4ecgx_status.scene != EXT_CFG_SCENE_NIGHTSHOT)
            {
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Macro_mode_3);
                mdelay(200);
            }
        break;
        case EXT_CFG_AF_OFF :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_OFF (afmode:%d)\n", __func__,s5k4ecgx_status.afmode);
            s5k4ecgx_status.afcanceled = true;
            if(s5k4ecgx_status.afmode == EXT_CFG_AF_SET_NORMAL)
            {
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_1);
                if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
                else mdelay(100);
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_2);
                if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
                else mdelay(100);
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_3);
            }
            else
            {
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Macro_mode_1);
                if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
                else mdelay(100);
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Macro_mode_2);
                if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
                else mdelay(100);
                S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Macro_mode_3);
            }
        break;
        case EXT_CFG_AF_DO :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_DO \n", __func__);
            s5k4ecgx_status.afcanceled = false;
#if 0
            s5k4ecgx_sensor_write(0x002C, 0x7000);
            s5k4ecgx_sensor_write(0x002E, 0x2B30);
            s5k4ecgx_sensor_read(0x0F12, &val);
            
            if(val < 0x001E)
                S5K4ECGX_WRITE_LIST(s5k4ecgx_Low_Cap_On);
            else
                S5K4ECGX_WRITE_LIST(s5k4ecgx_Low_Cap_Off);
#endif
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Single_AF_Start);
        break;
        case EXT_CFG_AF_SET_AE_FOR_FLASH :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_SET_AE_FOR_FLASH \n", __func__);
            s5k4ecgx_get_lux(&s5k4ecgx_status.current_lux);
            if(s5k4ecgx_status.flash_mode != EXT_CFG_FLASH_OFF)
            {
                if(s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_AUTO)
                {
                    if(s5k4ecgx_status.current_lux > 0x0032)break;
                }
                s5k4ecgx_sensor_write(0x0028, 0x7000);
#if defined(CONFIG_MACH_ANCORA)
                s5k4ecgx_sensor_write(0x002A, 0x057C);
#elif defined(CONFIG_MACH_APACHE)
                s5k4ecgx_sensor_write(0x002A, 0x0588);
#endif
                s5k4ecgx_sensor_write(0x0F12, 0x0000);
                if(flashOnFromApps==0)
                {
                    if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_FACTORY_TEST)// || s5k4ecgx_status.afmode == EXT_CFG_AF_SET_MACRO)
                        s5k4ecgx_set_flash(MACRO_FLASH);
                    else
                        s5k4ecgx_set_flash(PRE_FLASH);
                    mdelay(500);
                    pre_flash_on = 1;
                }
            }
        break;
        case EXT_CFG_AF_BACK_AE_FOR_FLASH :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_BACK_AE_FOR_FLASH \n", __func__);
            if(pre_flash_on && flashOnFromApps==0)
            {
                s5k4ecgx_sensor_write(0x0028, 0x7000);
#if defined(CONFIG_MACH_ANCORA)
                s5k4ecgx_sensor_write(0x002A, 0x057C);
#elif defined(CONFIG_MACH_APACHE)
                s5k4ecgx_sensor_write(0x002A, 0x0588);
#endif
                s5k4ecgx_sensor_write(0x0F12, 0x0002);
//            }
            s5k4ecgx_set_flash(PRE_FLASH_OFF);
            mdelay(200);
            pre_flash_on=0;
            }
        break;
        case EXT_CFG_AF_POWEROFF :
            CAMDRV_DEBUG("%s : EXT_CFG_AF_POWEROFF \n", __func__);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_1);
            if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
            else mdelay(100);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_2);
            if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
            else mdelay(100);
            if(s5k4ecgx_status.scene != EXT_CFG_SCENE_NIGHTSHOT)S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_4);
        break;        
        default :
            printk("[S5K4ECGX] Unexpected AF command : %d\n",value);
        break;
    }    
    return ret;
}

void s5k4ecgx_set_DTP(char value)
{
    switch(value)
    {
        case EXT_CFG_DTP_OFF:
            S5K4ECGX_WRITE_LIST(s5k4ecgx_DTP_stop);
        break;
        case EXT_CFG_DTP_ON:
            S5K4ECGX_WRITE_LIST(s5k4ecgx_DTP_init);
        break;
        default:
            printk("[S5K4ECGX] Unexpected DTP control on PCAM\n");
        break;
    }
}

void s5k4ecgx_set_ae_lock(char value)
{
    switch(value)
    {
        case EXT_CFG_AE_LOCK :
            s5k4ecgx_status.ae_lock = EXT_CFG_AE_LOCK;
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ae_lock);
        break;
        case EXT_CFG_AE_UNLOCK :
            s5k4ecgx_status.ae_lock = EXT_CFG_AE_UNLOCK;
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ae_unlock);
        break;
        case EXT_CFG_AWB_LOCK :
            s5k4ecgx_status.awb_lock = EXT_CFG_AWB_LOCK;
            S5K4ECGX_WRITE_LIST(s5k4ecgx_awb_lock);
        break;
        case EXT_CFG_AWB_UNLOCK :
            s5k4ecgx_status.awb_lock = EXT_CFG_AWB_UNLOCK;
            S5K4ECGX_WRITE_LIST(s5k4ecgx_awb_unlock);
        break;
        default :
            printk("[S5K4ECGX]Unexpected AWB_AE mode : %d\n", value);
        break;
    }
}

void s5k4ecgx_set_scene(char value)
{
    if(value != EXT_CFG_SCENE_OFF)
    {
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Default);
        //if(value == EXT_CFG_SCENE_TEXT)s5k4ecgx_set_af(EXT_CFG_AF_SET_MACRO);
        //else s5k4ecgx_set_af(EXT_CFG_AF_SET_NORMAL);
        if( s5k4ecgx_status.auto_contrast != EXT_CFG_AUTO_CONTRAST_OFF)s5k4ecgx_set_auto_contrast(EXT_CFG_AUTO_CONTRAST_OFF);
    }
    
    switch(value)
    {
        case EXT_CFG_SCENE_OFF :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Default);
            //s5k4ecgx_set_af(s5k4ecgx_status.afmode);
            if(s5k4ecgx_status.effect != EXT_CFG_EFFECT_NORMAL)s5k4ecgx_set_effect(s5k4ecgx_status.effect);
            if(s5k4ecgx_status.brightness != EXT_CFG_BR_STEP_0)s5k4ecgx_set_brightness(s5k4ecgx_status.brightness);
            if(s5k4ecgx_status.auto_exposure != EXT_CFG_METERING_NORMAL)s5k4ecgx_set_metering(s5k4ecgx_status.auto_exposure);
            if(s5k4ecgx_status.iso != EXT_CFG_ISO_AUTO)s5k4ecgx_set_iso(s5k4ecgx_status.iso);
            if(s5k4ecgx_status.contrast != EXT_CFG_CR_STEP_0 )s5k4ecgx_set_contrast(s5k4ecgx_status.contrast);
            if(s5k4ecgx_status.saturation != EXT_CFG_SA_STEP_0)s5k4ecgx_set_saturation(s5k4ecgx_status.saturation);
            if(s5k4ecgx_status.sharpness != EXT_CFG_SP_STEP_0)s5k4ecgx_set_sharpness(s5k4ecgx_status.sharpness);
            if(s5k4ecgx_status.whiteBalance != EXT_CFG_WB_AUTO)s5k4ecgx_set_whitebalance(s5k4ecgx_status.whiteBalance);
            if(s5k4ecgx_status.auto_contrast != EXT_CFG_AUTO_CONTRAST_OFF)s5k4ecgx_set_auto_contrast(s5k4ecgx_status.auto_contrast);
        break;
        case EXT_CFG_SCENE_PORTRAIT :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Portrait);
        break;
        case EXT_CFG_SCENE_LANDSCAPE :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Landscape);
            s5k4ecgx_set_metering(EXT_CFG_METERING_NORMAL);
        break;
        case EXT_CFG_SCENE_SPORTS :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Sports);
        break;
        case EXT_CFG_SCENE_PARTY :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Party_Indoor);
        break;
        case EXT_CFG_SCENE_BEACH :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(5,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Beach_Snow);
        break;
        case EXT_CFG_SCENE_SUNSET :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Sunset);
        break;
        case EXT_CFG_SCENE_DAWN :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Duskdawn);
        break;
        case EXT_CFG_SCENE_FALL :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Fall_Color);
        break;
        case EXT_CFG_SCENE_NIGHTSHOT :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Nightshot);
        break;
        case EXT_CFG_SCENE_BACKLIGHT :
// when there's no flash on device, in this s5k4ecgx_Scene_Backlight should be applied,
// but having flash, no need to apply backlight command.
#ifndef CONFIG_MACH_APACHE
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Backlight);
#endif
            //if(s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_ON ||s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_AUTO)s5k4ecgx_set_metering(EXT_CFG_METERING_CENTER);
            //else s5k4ecgx_set_metering(EXT_CFG_METERING_SPOT);
        break;
        case EXT_CFG_SCENE_FIREWORK :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Fireworks);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_ISO_50);
        break;
        case EXT_CFG_SCENE_TEXT :
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Text);
        break;
        case EXT_CFG_SCENE_CANDLE :
            s5k4ecgx_set_REG_TC_DBG_AutoAlgEnBits(3,0);
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Candle_Light);
        break;
        default :
            printk("[S5K4ECGX]Unexpected SCENE mode : %d\n",  value);
        break;
    }
}

void s5k4ecgx_set_capture_size(int size)
{
    CAMDRV_DEBUG("s5k4ecgx_set_capture_size %d \n",size );

    if(size == EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M)
    {
        S5K4ECGX_WRITE_LIST(s5k4ecgx_5M_Capture);
        s5k4ecgx_set_zoom(EXT_CFG_ZOOM_STEP_0);
    }
    else if(size == EXT_CFG_SNAPSHOT_SIZE_2560x1536_4M_WIDE)S5K4ECGX_WRITE_LIST(s5k4ecgx_4M_WIDE_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M)S5K4ECGX_WRITE_LIST(s5k4ecgx_3M_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE)S5K4ECGX_WRITE_LIST(s5k4ecgx_2_4M_WIDE_Capture)   
    else if(size == EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M)S5K4ECGX_WRITE_LIST(s5k4ecgx_2M_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE)S5K4ECGX_WRITE_LIST(s5k4ecgx_1_5M_WIDE_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_1280x960_1M)S5K4ECGX_WRITE_LIST(s5k4ecgx_1M_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE)S5K4ECGX_WRITE_LIST(s5k4ecgx_4K_WIDE_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_640x480_VGA)S5K4ECGX_WRITE_LIST(s5k4ecgx_VGA_Capture)
    else if(size == EXT_CFG_SNAPSHOT_SIZE_320x240_QVGA)S5K4ECGX_WRITE_LIST(s5k4ecgx_QVGA_Capture)
        
    else printk("[S5K4ECGX]s5k4ecgx_set_capture : wrong size!!! \n");
    if(size != EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M && s5k4ecgx_status.zoom != EXT_CFG_ZOOM_STEP_0)s5k4ecgx_set_zoom(s5k4ecgx_status.zoom);

    if(size == EXT_CFG_SNAPSHOT_SIZE_320x240_QVGA)
    {
        /* change thumbnail size to 320x240*/
        s5k4ecgx_sensor_write(0x0028, 0x7000);
        s5k4ecgx_sensor_write(0x002A, 0x047E);
        s5k4ecgx_sensor_write(0x0F12, 0x0140);
        s5k4ecgx_sensor_write(0x0F12, 0x00F0);
    }
    else
    {
        /* change thumbnail size to 640x480*/
        s5k4ecgx_sensor_write(0x0028, 0x7000);
        s5k4ecgx_sensor_write(0x002A, 0x047E);
        s5k4ecgx_sensor_write(0x0F12, 0x0280);
        s5k4ecgx_sensor_write(0x0F12, 0x01E0);
    }
}

void s5k4ecgx_set_preview_size(int size)
{
    CAMDRV_DEBUG("s5k4ecgx_set_preview_size %d \n",size );

    if(HD_mode) {
        HD_mode = 0;
        S5K4ECGX_WRITE_LIST(s5k4ecgx_1280_Preview_D)
    }

    if(size == EXT_CFG_PREVIEW_SIZE_640x480_VGA)
        S5K4ECGX_WRITE_LIST(s5k4ecgx_640_Preview)
    else if(size == EXT_CFG_PREVIEW_SIZE_176x144_QCIF)
        S5K4ECGX_WRITE_LIST(s5k4ecgx_640_Preview)
    else if(size == EXT_CFG_PREVIEW_SIZE_800x480_WVGA)
        S5K4ECGX_WRITE_LIST(s5k4ecgx_800_Preview)        
    else if(size == EXT_CFG_PREVIEW_SIZE_320x240_QVGA)
        S5K4ECGX_WRITE_LIST(s5k4ecgx_176_Preview)
    else if(size == EXT_CFG_PREVIEW_SIZE_720x480_D1)
        S5K4ECGX_WRITE_LIST(s5k4ecgx_720_Preview)
    else if(size == EXT_CFG_PREVIEW_SIZE_1280x720_D1) {
            HD_mode = 1;
        S5K4ECGX_WRITE_LIST(s5k4ecgx_1280_Preview_E)        
    }
    else
        printk("[S5K4ECGX]s5k4ecgx_set_capture : wrong size!!! \n");

    s5k4ecgx_set_fps(s5k4ecgx_status.fps);
    
}

void s5k4ecgx_check_REG_TC_GP_EnableAWBChanged(void)
{
    int cnt = 0;
    int REG_TC_GP_EnableAWBChanged = 0;
    printk("[S5K4ECGX]s5k4ecgx_check_REG_TC_GP_EnableAWBChanged\n");
    while(cnt < 25)
    {
        s5k4ecgx_sensor_write(0x002C, 0x7000);
        s5k4ecgx_sensor_write(0x002E, 0x2C78);
        s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&REG_TC_GP_EnableAWBChanged);
        if(REG_TC_GP_EnableAWBChanged == 0x01)break;
        mdelay(10);
        cnt++;
    }
    if(cnt)printk("[S5K4ECGX] wait time for stablizing AE/AWB : %dms\n",cnt*10);
    if(REG_TC_GP_EnableAWBChanged != 0x01)printk("[S5K4ECGX] stablizing AE/AWB failed.\n");
}

void s5k4ecgx_check_REG_TC_GP_EnablePreviewChanged(void)
{
    int cnt = 0;
    int REG_TC_GP_EnablePreviewChanged = 0;
    printk("[S5K4ECGX]s5k4ecgx_check_REG_TC_GP_EnablePreviewChanged\n");
    while(cnt < 150)
    {
        s5k4ecgx_sensor_write(0x002C, 0x7000);
        s5k4ecgx_sensor_write(0x002E, 0x0244);    
        s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&REG_TC_GP_EnablePreviewChanged);
        if(!REG_TC_GP_EnablePreviewChanged)break;
        mdelay(10);
        cnt++;
    }    
    if(cnt)printk("[S5K4ECGX] wait time for preview frame : %dms\n",cnt*10);
    if(REG_TC_GP_EnablePreviewChanged)printk("[S5K4ECGX] start preview failed.\n");
}

void s5k4ecgx_set_preview(void)
{
    int rc = 0;
#ifdef WORKAROUND_FOR_LOW_SPEED_I2C
    unsigned int before_time, after_time, time_gab;
    int *i2c_clk_addr;
    //i2c_clk_addr = 0xd500c004;
    i2c_clk_addr = get_i2c_clock_addr(s5k4ecgx_client->adapter);
#endif    
    printk("[S5K4ECGX]s5k4ecgx_set_preview start\n");

    if(s5k4ecgx_status.camera_initailized == false)
    {
#ifdef WORKAROUND_FOR_LOW_SPEED_I2C    
        before_time = get_jiffies_64();
#endif

#ifdef CONFIG_LOAD_FILE
        S5K4ECGX_WRITE_LIST(s5k4ecgx_init_reg1);
        msleep(100);
#endif
        S5K4ECGX_WRITE_LIST(s5k4ecgx_init_reg2);


#ifdef WORKAROUND_FOR_LOW_SPEED_I2C    
        after_time = get_jiffies_64();
        time_gab = jiffies_to_msecs(after_time-before_time);
        if(time_gab > 200)
        {
            printk("<=PCAM=> i2c speed is going down : %dmsec\n", time_gab);
            pcam_msm_i2c_pwr_mgmt(s5k4ecgx_client->adapter, 1);
            //this funcion have to call after clk_enable by PCAM, do not use it without pcam's allowance
            printk("<=PCAM=> i2c clk set forcely 0x316\n");
            writel(0x316, i2c_clk_addr);
            printk("<=PCAM=> re-check i2c clk :%x\n", readl(i2c_clk_addr));
         }        
#endif        
    } 
    else if(s5k4ecgx_status.camera_initailized == true)
    {
        if(!isPreviewReturnWrite)
        {
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Preview_Return);
            isPreviewReturnWrite = true;
        }
    }
    
//#ifdef NOT_USE
    rc = s5k4ecgx_sensor_write(0x002C, 0x7000);
    s5k4ecgx_sensor_write(0x002E, 0x01A6);
    s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&s5k4ecgx_status.id);
    CAMDRV_DEBUG("[S5K4ECGX] SENSOR FW VERSION : 0x%x \n",s5k4ecgx_status.id);
//#endif

#ifdef NOT_USE
    /* workaround code for late arrival preview frame on FIREWORK mode */
    if(s5k4ecgx_status.scene == EXT_CFG_SCENE_FIREWORK)
    {
        CAMDRV_DEBUG("[S5K4ECGX] scene mode is EXT_CFG_SCENE_FIREWORK. so we wait 600ms..\n");
        msleep(600);
    }
#endif    
    s5k4ecgx_check_REG_TC_GP_EnablePreviewChanged();
    
#if 0 //disable Frmae skip
        if((s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT) ||(s5k4ecgx_status.scene == EXT_CFG_SCENE_FIREWORK) )
        {
            for(cnt=0; cnt<450; cnt++)
            {
                vsync_value = gpio_get_value(14);
                if(vsync_value)
                    break;
                else
                {
                    CAMDRV_DEBUG("wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
                    msleep(3);
                }
            }
        }
#endif
#define DATA_LINE_CHECK
#ifdef DATA_LINE_CHECK
    if(s5k4ecgx_status.need_configuration & CHECK_DTP&& s5k4ecgx_status.dtp != EXT_CFG_DTP_OFF)s5k4ecgx_set_DTP(s5k4ecgx_status.dtp);
#endif
    S5K4ECGX_WRITE_LIST(s5k4ecgx_Flash_init);
    CAMDRV_DEBUG("[S5K4ECGX] sensor configuration start\n");

    s5k4ecgx_set_preview_size(s5k4ecgx_status.preview_size);
        
    //S5K4ECGX_WRITE_LIST(s5k4ecgx_awb_ae_unlock);
    if(s5k4ecgx_status.scene == EXT_CFG_SCENE_OFF)
    {
        if(s5k4ecgx_status.need_configuration & CHECK_EFFECT && s5k4ecgx_status.effect != EXT_CFG_EFFECT_NORMAL)s5k4ecgx_set_effect(s5k4ecgx_status.effect);
        if(s5k4ecgx_status.need_configuration & CHECK_BRIGHTNESS && s5k4ecgx_status.brightness != EXT_CFG_BR_STEP_0)s5k4ecgx_set_brightness(s5k4ecgx_status.brightness);
        if(s5k4ecgx_status.need_configuration & CHECK_AE && s5k4ecgx_status.auto_exposure != EXT_CFG_METERING_CENTER)s5k4ecgx_set_metering(s5k4ecgx_status.auto_exposure);
        if(s5k4ecgx_status.need_configuration & CHECK_ISO && s5k4ecgx_status.iso != EXT_CFG_ISO_AUTO)s5k4ecgx_set_iso(s5k4ecgx_status.iso);
        if(s5k4ecgx_status.need_configuration & CHECK_CONTRAST && s5k4ecgx_status.contrast != EXT_CFG_CR_STEP_0 )s5k4ecgx_set_contrast(s5k4ecgx_status.contrast);
        if(s5k4ecgx_status.need_configuration & CHECK_SATURATION && s5k4ecgx_status.saturation != EXT_CFG_SA_STEP_0)s5k4ecgx_set_saturation(s5k4ecgx_status.saturation);
        if(s5k4ecgx_status.need_configuration & CHECK_SHARPNESS && s5k4ecgx_status.sharpness != EXT_CFG_SP_STEP_0)s5k4ecgx_set_sharpness(s5k4ecgx_status.sharpness);
        if(s5k4ecgx_status.need_configuration & CHECK_WB && s5k4ecgx_status.whiteBalance != EXT_CFG_WB_AUTO)s5k4ecgx_set_whitebalance(s5k4ecgx_status.whiteBalance);
        if(s5k4ecgx_status.need_configuration & CHECK_AFMODE && s5k4ecgx_status.camera_initailized == false)s5k4ecgx_set_af(s5k4ecgx_status.afmode);
        if(s5k4ecgx_status.need_configuration & CHECK_AUTOCONTRAST && s5k4ecgx_status.auto_contrast != EXT_CFG_AUTO_CONTRAST_OFF)s5k4ecgx_set_auto_contrast(s5k4ecgx_status.auto_contrast);
    }
    else 
    {
        s5k4ecgx_set_scene(s5k4ecgx_status.scene);
    }
    if(s5k4ecgx_status.need_configuration & CHECK_FPS)s5k4ecgx_set_fps(s5k4ecgx_status.fps);
    if(s5k4ecgx_status.need_configuration & CHECK_ZOOM && s5k4ecgx_status.zoom != EXT_CFG_ZOOM_STEP_0)s5k4ecgx_set_zoom(s5k4ecgx_status.zoom);
    if(s5k4ecgx_status.need_configuration & CHECK_JPEGQUALITY && s5k4ecgx_status.jpeg_quality != EXT_CFG_JPEG_QUALITY_SUPERFINE)s5k4ecgx_set_jpeg_quality(s5k4ecgx_status.jpeg_quality);
    if(s5k4ecgx_status.need_configuration & CHECK_SNAPSHOT_SIZE && s5k4ecgx_status.snapshot_size !=EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M)s5k4ecgx_set_capture_size(s5k4ecgx_status.snapshot_size);

    if(s5k4ecgx_status.camera_initailized == false)msleep(100); /*delay for pclk stabilization (noframe issue) 2011-01-20 */

    /* reset status*/
    //s5k4ecgx_status.preview_size = EXT_CFG_PREVIEW_SIZE_640x480_VGA;
    s5k4ecgx_status.flash_exifinfo = false;
    s5k4ecgx_status.flash_status = FLASH_OFF;
    s5k4ecgx_status.camera_initailized = true;
    s5k4ecgx_status.lowcap_on= false;
    if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)
    {
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Nightshot);
        s5k4ecgx_status.nightcap_on= true;
    }
    else
        s5k4ecgx_status.nightcap_on= false;
    s5k4ecgx_status.nightcap_on= false;
    s5k4ecgx_status.afcanceled = false;
    s5k4ecgx_status.camera_status = PREVIEW;

    if(s5k4ecgx_status.camera_on == false)
    {
//      mdelay(100);
      s5k4ecgx_status.camera_on = true;
      s5k4ecgx_check_REG_TC_GP_EnableAWBChanged();
    }
}

void s5k4ecgx_check_REG_TC_GP_EnableCaptureChanged(void)
{
    int cnt = 0;
    int REG_TC_GP_EnableCaptureChanged = 0;

    while(cnt < 150)
    {
        s5k4ecgx_sensor_write(0x002C, 0x7000);
        s5k4ecgx_sensor_write(0x002E, 0x0244);    
        s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&REG_TC_GP_EnableCaptureChanged);
        if(!REG_TC_GP_EnableCaptureChanged)break;
        mdelay(10);
        cnt++;
    }    
    if(cnt)printk("[S5K4ECGX] wait time for capture frame : %dms\n",cnt*10);
    if(REG_TC_GP_EnableCaptureChanged)printk("[S5K4ECGX] take picture failed.\n");
}

void s5k4ecgx_set_capture(void)
{
    isPreviewReturnWrite = false;
    s5k4ecgx_status.camera_status = SNAPSHOT;
    
    /* Check current lux */
    if(s5k4ecgx_status.flash_status == FLASH_OFF)s5k4ecgx_get_lux(&s5k4ecgx_status.current_lux);

    /* CASE 1 : Capture with no flash (System lag will be measured in this condition)*/
    if(s5k4ecgx_status.flash_mode != EXT_CFG_FLASH_ON && s5k4ecgx_status.current_lux > 0x0032)
    {
        CAMDRV_DEBUG("[S5K4ECGX] Capture in No Flash.\n");
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Capture_Start);
        s5k4ecgx_check_REG_TC_GP_EnableCaptureChanged();
        return;
    }
    /* CASE 2 : Flash turn on  in normal light condition*/
    else if(s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_ON && s5k4ecgx_status.current_lux > 0x0032)
    {
        CAMDRV_DEBUG("[S5K4ECGX] Capture in Normal Light.\n");
        /* ae/awb unlock */
        if(s5k4ecgx_status.ae_lock == EXT_CFG_AE_LOCK)s5k4ecgx_set_ae_lock(EXT_CFG_AE_UNLOCK);
        if(s5k4ecgx_status.awb_lock == EXT_CFG_AWB_LOCK)s5k4ecgx_set_ae_lock(EXT_CFG_AWB_UNLOCK);

        /* use torch mode on below condition */
        if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_FACTORY_TEST || s5k4ecgx_status.afmode == EXT_CFG_AF_SET_MACRO)
            s5k4ecgx_set_flash(MACRO_FLASH);
        else
            s5k4ecgx_set_flash(FULL_FLASH);
        mdelay(200);
        
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Capture_Start);
        s5k4ecgx_check_REG_TC_GP_EnableCaptureChanged();
        return;
    }
    /* Flash control in low light condition*/
    else
    {
        /* CASE 3 : flash off on below scene mode */
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)
        {
            CAMDRV_DEBUG("[S5K4ECGX] Capture in Night Mode.\n");
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Scene_Nightshot);
            mdelay(250);
            
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Capture_Start);
            s5k4ecgx_check_REG_TC_GP_EnableCaptureChanged();
            s5k4ecgx_status.nightcap_on= true;
            return;
        }
        else
        /* CASE 4 : flash auto mode & low light condition */
        {
            CAMDRV_DEBUG("[S5K4ECGX] Capture in Low Light.\n");
            S5K4ECGX_WRITE_LIST(s5k4ecgx_Low_Cap_On);
            if(s5k4ecgx_status.flash_mode != EXT_CFG_FLASH_OFF)
            {
                /* ae/awb unlock */
                if(s5k4ecgx_status.ae_lock == EXT_CFG_AE_LOCK)s5k4ecgx_set_ae_lock(EXT_CFG_AE_UNLOCK);
                if(s5k4ecgx_status.awb_lock == EXT_CFG_AWB_LOCK)s5k4ecgx_set_ae_lock(EXT_CFG_AWB_UNLOCK);

                /* use torch mode on below condition */
                if(s5k4ecgx_status.camera_mode == EXT_CFG_CAM_MODE_FACTORY_TEST || s5k4ecgx_status.afmode == EXT_CFG_AF_SET_MACRO)
                    s5k4ecgx_set_flash(MACRO_FLASH);
                else
                    s5k4ecgx_set_flash(FULL_FLASH);
            }
            if(s5k4ecgx_status.flash_exifinfo == true)mdelay(200);
            else mdelay(120);

            S5K4ECGX_WRITE_LIST(s5k4ecgx_Capture_Start);
            s5k4ecgx_check_REG_TC_GP_EnableCaptureChanged();
            s5k4ecgx_status.lowcap_on= true;
            return;
        }
    }
}

int s5k4ecgx_sensor_esd_detected()
{
    CAMDRV_DEBUG("[S5K4ECGX] ESD Detected!!\n");
    b_esd_detected=true;
    return SENSOR_SECCESS;
}
int s5k4ecgx_sensor_ext_config(void __user *arg)
{
    sensor_ext_cfg_data    cfg_data;

    if (copy_from_user((void *)&cfg_data, (const void *)arg, sizeof(cfg_data)))
        return -EFAULT;

    if(!s5k4ecgx_status.camera_initailized)
    {
      CAMDRV_DEBUG("s5k4ecgx_sensor_ext_config is skipped,sensor is initialinzing.(%d %d %d)\n",cfg_data.cmd, cfg_data.value_1,cfg_data.value_2);        
    }

    CAMDRV_DEBUG("s5k4ecgx_sensor_ext_config %d %d %d \n",cfg_data.cmd, cfg_data.value_1,cfg_data.value_2);
    
    switch(cfg_data.cmd)
    {
        //case EXT_CFG_AUTO_TUNNING:
            //S5K4ECGX_WRITE_LIST(s5k4ecgx_Preview_Return);
            //isPreviewReturnWrite = true;
        //break;
        //case EXT_CFG_SDCARD_DETECT:
        //break;
        case EXT_CFG_GET_EXIF_INFO:{
            unsigned short lsb, msb,a_gain,d_gain;
            s5k4ecgx_sensor_write(0xFCFC, 0xD000);
            s5k4ecgx_sensor_write(0x002C, 0x7000);
            if(s5k4ecgx_status.id == 0x0011)
                s5k4ecgx_sensor_write(0x002E, 0x2BC0);
            else
                s5k4ecgx_sensor_write(0x002E, 0x2AD8);
            s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&lsb);//8
            s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&msb);//A
            s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&a_gain);//C
            s5k4ecgx_sensor_read(0x0F12, (unsigned short*)&d_gain);//E
            cfg_data.value_1 = lsb;
            cfg_data.value_2 = msb;
            cfg_data.cmd = a_gain;
            cfg_data.device_id = d_gain;
            CAMDRV_DEBUG("exposure %x %x \n", lsb, msb);
            //CAMDRV_DEBUG("rough_iso %x \n", rough_iso);
            if(s5k4ecgx_status.nightcap_on== true)S5K4ECGX_WRITE_LIST(s5k4ecgx_Night_Mode_Off)
            else if(s5k4ecgx_status.lowcap_on== true)S5K4ECGX_WRITE_LIST(s5k4ecgx_Low_Cap_Off)
        }
        break;
        case EXT_CFG_GET_FLASH_INFO:
            if(s5k4ecgx_status.flash_mode != EXT_CFG_FLASH_OFF 
                && s5k4ecgx_status.flash_status != FLASH_OFF 
                && s5k4ecgx_status.flash_status != PRE_FLASH_OFF)
            {
                s5k4ecgx_set_flash(FLASH_OFF);
            }
            CAMDRV_DEBUG("EXT_CFG_FLASH_INFO %d \n", s5k4ecgx_status.flash_exifinfo);
            cfg_data.value_1 = s5k4ecgx_status.flash_exifinfo;
        break;
        //case EXT_CFG_LUX_INFO:
            //cfg_data.value_1 = s5k4ecgx_status.current_lux;
        break;        
        case EXT_CFG_SET_FPS:
            s5k4ecgx_status.fps = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_FPS;
            else
                s5k4ecgx_set_fps(s5k4ecgx_status.fps);
        break;
        case EXT_CFG_SET_AF_OPERATION:
            if(cfg_data.value_1 ==  2|| cfg_data.value_1 == 3)s5k4ecgx_status.afmode= cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_AFMODE;
            else
                cfg_data.value_2 = s5k4ecgx_set_af(cfg_data.value_1);
        break;
        case EXT_CFG_SET_EFFECT:
            s5k4ecgx_status.effect = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_EFFECT;
            else
                s5k4ecgx_set_effect(s5k4ecgx_status.effect);
        break;
        case EXT_CFG_SET_WB:
            s5k4ecgx_status.whiteBalance = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_WB;
            else
                s5k4ecgx_set_whitebalance(s5k4ecgx_status.whiteBalance);
        break;
        case EXT_CFG_SET_BRIGHTNESS:
            if(s5k4ecgx_status.ae_lock == EXT_CFG_AE_LOCK)
                s5k4ecgx_set_ae_lock(EXT_CFG_AE_UNLOCK);
            s5k4ecgx_status.brightness = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_BRIGHTNESS;
            else
                s5k4ecgx_set_brightness(s5k4ecgx_status.brightness);
        break;
        case EXT_CFG_SET_ISO:
            s5k4ecgx_status.iso = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_ISO;
            else
                s5k4ecgx_set_iso(s5k4ecgx_status.iso);
        break;
        case EXT_CFG_SET_METERING:
            s5k4ecgx_status.auto_exposure = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_AE;
            else
                s5k4ecgx_set_metering(s5k4ecgx_status.auto_exposure);
        break;
        case EXT_CFG_SET_SCENE:
            s5k4ecgx_status.scene = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration|= CHECK_SCENE;
            else
                s5k4ecgx_set_scene(s5k4ecgx_status.scene);
        break;
        case EXT_CFG_SET_AE_AWB:
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration|= CHECK_AE_AWB_LOCK;
            else
                s5k4ecgx_set_ae_lock(cfg_data.value_1);
        break;
        case EXT_CFG_SET_CONTRAST:
            s5k4ecgx_status.contrast = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration|= CHECK_CONTRAST;
            else
                s5k4ecgx_set_contrast(s5k4ecgx_status.contrast);
        break;
        case EXT_CFG_SET_SATURATION:
            s5k4ecgx_status.saturation = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_SATURATION;
            else
                s5k4ecgx_set_saturation(s5k4ecgx_status.saturation);
        break;
        case EXT_CFG_SET_SHARPNESS:
            s5k4ecgx_status.sharpness = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_SHARPNESS;
            else
                s5k4ecgx_set_sharpness(s5k4ecgx_status.sharpness);
        break;
     case EXT_CFG_SET_PREVIEW_SIZE:
         s5k4ecgx_status.preview_size = cfg_data.value_1;
          //  if(!s5k4ecgx_status.camera_initailized)
           //     s5k4ecgx_status.need_configuration |= CHECK_PREVIEW_SIZE;
         //   else
          //      s5k4ecgx_set_preview_size(s5k4ecgx_status.preview_size);
             break;
/*
        case EXT_CFG_CPU_CONTROL:
            switch(cfg_data.value_1)
            {
                case EXT_CFG_CPU_CONSERVATIVE:
                CAMDRV_DEBUG("now conservative\n");
                cpufreq_direct_set_policy(0, "conservative");
                break;
                case EXT_CFG_CPU_ONDEMAND:
                CAMDRV_DEBUG("now ondemand\n");
                cpufreq_direct_set_policy(0, "ondemand");
                break;
                case EXT_CFG_CPU_PERFORMANCE:
                CAMDRV_DEBUG("now performance\n");
                cpufreq_direct_set_policy(0, "performance");
                break;
                default:
                    printk("[S5K4ECGX] Unexpected CPU control on PCAM\n");
                break;
            }
        break;
        case EXT_CFG_SENSOR_RESET:
                printk("[S5K4ECGX] ************************CAM frame timeout\n");
                s5k4ecgx_status.camera_initailized = false;
                s5k4ecgx_set_power(false);
                s5k4ecgx_set_power(true);
                msm_camio_clk_rate_set(24000000);
                msm_camio_camif_pad_reg_reset();
                msleep(5);            
                s5k4ecgx_probe_init_sensor();
                s5k4ecgx_set_preview();
        break;
*/
        case EXT_CFG_SET_DTP:
            s5k4ecgx_status.dtp = cfg_data.value_1;
        printk("********[diony]  cfg_data.value_1 = %d.\n", cfg_data.value_1);
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_DTP;
            else
                s5k4ecgx_set_DTP(s5k4ecgx_status.dtp);
            if(cfg_data.value_1 == 0)
                    cfg_data.value_2 = 2;
            else if(cfg_data.value_1 == 1)
                    cfg_data.value_2 = 3;
        break;
        case EXT_CFG_SET_PICTURE_SIZE:
            s5k4ecgx_status.snapshot_size = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_SNAPSHOT_SIZE;
            else
                s5k4ecgx_set_capture_size(s5k4ecgx_status.snapshot_size);
        break;
        //case EXT_CFG_SET_CAPTURE_MODE:
            //s5k4ecgx_set_capture();
        //break;
        case EXT_CFG_SET_FLASH_MODE:
            CAMDRV_DEBUG("EXT_CFG_SET_FLASH_MODE: cmd=%d, value_1=%d, value_2=%d \n",cfg_data.cmd, cfg_data.value_1,cfg_data.value_2);
            if(cfg_data.value_2 != EXT_CFG_FLASH_TURN_ON && cfg_data.value_2 != EXT_CFG_FLASH_TURN_OFF)
            {
                s5k4ecgx_status.flash_mode = cfg_data.value_2;
                //P111026-1625:Appl Name : Tiny Flashlight, This third party app is sending the command EXT_CFG_FLASH_OFF
                if(cfg_data.value_2 == EXT_CFG_FLASH_OFF)
                s5k4ecgx_set_flash(FLASH_OFF);
                //if(s5k4ecgx_status.scene == EXT_CFG_SCENE_BACKLIGHT && s5k4ecgx_status.flash_mode != EXT_CFG_FLASH_OFF)s5k4ecgx_set_metering(EXT_CFG_METERING_CENTER);
                //else if(s5k4ecgx_status.scene == EXT_CFG_SCENE_BACKLIGHT && s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_OFF)s5k4ecgx_set_metering(EXT_CFG_METERING_SPOT);
            }
            else if(cfg_data.value_2 == EXT_CFG_FLASH_TURN_ON)
            {
                if(s5k4ecgx_status.flash_mode == EXT_CFG_FLASH_AUTO)
                {
                    s5k4ecgx_get_lux(&s5k4ecgx_status.current_lux);
                    if(s5k4ecgx_status.current_lux < 0x0032)
                    {
                        s5k4ecgx_set_flash(MOVIE_FLASH);
                        flashOnFromApps = 1;
                    }
                }
                else
                {
                    s5k4ecgx_set_flash(MOVIE_FLASH);
                    flashOnFromApps = 1;
                }
            }
            else if(cfg_data.value_2 == EXT_CFG_FLASH_TURN_OFF)
            {
                s5k4ecgx_set_flash(FLASH_OFF);
                flashOnFromApps = 0;
            }
        break;
        case EXT_CFG_SET_JPEG_QUALITY:
            s5k4ecgx_status.jpeg_quality = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_JPEGQUALITY;
            else
                s5k4ecgx_set_jpeg_quality(cfg_data.value_1);
        break;
        case EXT_CFG_SET_AUTO_CONTRAST:
            s5k4ecgx_status.auto_contrast = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_AUTOCONTRAST;
            else
                s5k4ecgx_set_auto_contrast(cfg_data.value_1);
        break;
        case EXT_CFG_SET_ZOOM:
            s5k4ecgx_status.zoom = cfg_data.value_1;
            if(!s5k4ecgx_status.camera_initailized)
                s5k4ecgx_status.need_configuration |= CHECK_ZOOM;
            else
                s5k4ecgx_set_zoom(cfg_data.value_1);
        break;
        case EXT_CFG_SET_CAM_MODE:
            s5k4ecgx_status.camera_mode = cfg_data.value_1;
            CAMDRV_DEBUG("CAMERA MODE : %d\n",s5k4ecgx_status.camera_mode);
        break;
        case EXT_CFG_TEST_ESD:
            CAMDRV_DEBUG("[S5K4ECGX] EXT_CFG_TEST_ESD\n");
            if(cfg_data.value_1 == 1)
            {
                printk(KERN_ERR "[S5K4ECGX]: ESD Value %d\n",b_esd_detected);
                cfg_data.value_2 = b_esd_detected;
                b_esd_detected=false;
            }
            else
            {
                printk(KERN_ERR "[S5K4ECGX]:EXT_CFG_TEST_ESD Sensor Reset\n");

                s5k4ecgx_set_power(false);
                mdelay(100);          
                s5k4ecgx_set_power(true);    
                s5k4ecgx_status.camera_initailized = false;
                s5k4ecgx_status.camera_on = false;
                s5k4ecgx_probe_init_sensor();    
                HD_mode=0;
				msleep(100);
                s5k4ecgx_set_preview();

            }
       break;

#if defined (CONFIG_MACH_APACHE)
       case EXT_CFG_SET_TOUCHAF_POS:
       {
         unsigned short touchX, touchY = 0;
         
         touchX = cfg_data.value_1;
         touchY = cfg_data.value_2;
         CAMDRV_DEBUG("[[PCAM - 5]]SET_TOUCHAF_POS -X: %d, -Y: %d\n", touchX, touchY);
         s5k4ecgx_set_touchAF(touchX, touchY);
       }
       break;
#endif
        default :
            printk("[S5K4ECGX]Unexpected mode on sensor_rough_control : %d\n", cfg_data.cmd);
        break;
    }

    if(copy_to_user((void *)arg, (const void *)&cfg_data, sizeof(cfg_data)))
    {
        printk("[S5K4ECGX]%s fail on copy_to_user!\n", __func__);
    }

    return SENSOR_SECCESS;
}

void cam_pw(int status)
{
    printk("[S5K4ECGX]We does not use this function anymore.\n");
}

void s5k4ecgx_set_power(int status)
{
    unsigned int mclk_cfg;
    struct vreg *vreg_ldo20, *vreg_ldo11;

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

    if(status == 1) //POWER ON
    {
#if defined(CONFIG_MACH_ANCORA)
        printk("[S5K4ECGX]Camera Sensor Power ON \n");

        b_esd_detected=false;
        
        mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

        /* initailize power control pin */    
        gpio_set_value(174, 0); // CAM_MEGA_nRST
        gpio_set_value(175, 0); // CAM_MERA_STBY
        gpio_set_value(31, 0);  // CAM_VT_nSTBY
        gpio_set_value(132, 0); // CAM_VT_nRST

        if (vreg_enable(vreg_ldo20)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_enable failed\n", __func__);
        }
        if (vreg_enable(vreg_ldo11)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_enable failed\n", __func__);
        }
        mdelay(1);        
        /* Enable CAM_VT_nSTBY */
        gpio_set_value(31, 1);
        mdelay(20);
        /* Enable CAM_VT_nRST */
        gpio_set_value(132, 1);
        mdelay(20);
        gpio_set_value(31, 0);  // CAM_VT go to HI-Z.

        
        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET, 0);
        gpio_set_value(CAM_FLASH_FLEN, 0);

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
        gpio_set_value(2, 1); // lp8720 enable
        
        lp8720_i2c_write(0x08, 0x20);
        mdelay(1);
        lp8720_i2c_write(0x08, 0x22);
        mdelay(1);
        lp8720_i2c_write(0x08, 0x26);
        mdelay(1);
        lp8720_i2c_write(0x08, 0x36);
        mdelay(1);
                
       /* Enable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        mdelay(5);

#elif defined(CONFIG_MACH_APACHE)

        CAMDRV_DEBUG("[S5K4ECGX]Camera Sensor Power ON \n");
        mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

        /* initailize power control pin */    
        gpio_set_value(174, 0); // CAM_MEGA_nRST
        gpio_set_value(175, 0); // CAM_MEGA_STBY
        gpio_set_value(31, 0);  // CAM_VT_nSTBY
        gpio_set_value(132, 0); // CAM_VT_nRST

        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET, 0);
        gpio_set_value(CAM_FLASH_FLEN, 0);


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
        gpio_set_value(2, 1); // lp8720 enable

        //LDO3 1.8v
        lp8720_i2c_write(0x08, 0x04);
        mdelay(1);
        //LDO2 2.8v
        lp8720_i2c_write(0x08, 0x06);
        mdelay(1);
        //LDO5 2.8v
        lp8720_i2c_write(0x08, 0x16);
        mdelay(1);

        if (vreg_enable(vreg_ldo20)) { //LDO20 powers both VDDIO 1.8V and 1.3M Core 1.8V
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_enable failed\n", __func__);
        }
        mdelay(1);
        if (vreg_enable(vreg_ldo11)) { //AVDD 2.8V
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_enable failed\n", __func__);
        }

        gpio_set_value(31, 1);  // CAM_VT_nSTBY
        udelay(5);
        
        /* Enable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        mdelay(10);
        
        gpio_set_value(132, 1); // CAM_VT_nRST ENABLE
        mdelay(2);
        gpio_set_value(31, 0);  // CAM_VT_nSTBY
        mdelay(1);

        //LDO Core 1.2v
        lp8720_i2c_write(0x08, 0x36);
        mdelay(1);
        
        gpio_set_value(175, 1); //CAM_MEGA_STBY ENABLE
        udelay(20);
        gpio_set_value(174, 1); // CAM_MEGA_nRST ENABLE
        udelay(70);
        CAMDRV_DEBUG("I2C Enable \n");  
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);            
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);

#endif
#ifdef WORKAROUND_FOR_LOW_SPEED_I2C
        //after power on, below function will be called.
        pcam_msm_i2c_pwr_mgmt(s5k4ecgx_client->adapter, 1);
#endif
    }
    else //POWER OFF
    {
#ifdef CONFIG_MACH_ANCORA
        printk("[S5K4ECGX]Camera Sensor AF init!\n");    //     remove tick noise.
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Preview_Return);    // for AF
        s5k4ecgx_set_af(EXT_CFG_AF_SET_NORMAL);

        printk("[S5K4ECGX]Camera Sensor Power OFF\n");
        mclk_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        /* workaround code for AF ACL noise issue */
        if(s5k4ecgx_status.camera_status != SNAPSHOT)
        {
            printk("[S5K4ECGX]=========> Power OFF DELAY : 300ms\n");
            mdelay(150);
        }
#ifdef WORKAROUND_FOR_LOW_SPEED_I2C
        //before power off, below function will be called.
        pcam_msm_i2c_pwr_mgmt(s5k4ecgx_client->adapter, 0); 
#endif        
        s5k4ecgx_status.power_on = false;
        /* LP8720 disable */
        lp8720_i2c_write(0x08, 0x00);
        gpio_set_value(2, 0);        // lp8720 disable

        if (vreg_disable(vreg_ldo11)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_disable failed\n", __func__);
        }        
        if (vreg_disable(vreg_ldo20)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_disable failed\n", __func__);
        }
        
        mdelay(1);
        gpio_set_value(174, 0);      //nRST
        udelay(50);
        
        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_DISABLE);
        udelay(50);

        gpio_set_value(175, 0);     //STBY

        gpio_set_value(31, 0);  // CAM_VT_nSTBY
        gpio_set_value(132, 0); // CAM_VT_nRST

        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);
        mdelay(1); // to enter a shutdown mode        

#elif defined(CONFIG_MACH_APACHE)

        CAMDRV_DEBUG("[S5K4ECGX]Camera Sensor AF init!\n");    //     remove tick noise.
        S5K4ECGX_WRITE_LIST(s5k4ecgx_Preview_Return);    // for AF
        //P111027-1359: Time that swiching camcorder is not sufficient ( result 3.1s, expect 2.5s)
//        s5k4ecgx_set_af(EXT_CFG_AF_SET_NORMAL);

        S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_1);
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250); 
        else mdelay(10);//previous value  200
        S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_2);
        if(s5k4ecgx_status.scene == EXT_CFG_SCENE_NIGHTSHOT)mdelay(250);
        else mdelay(10);//previous value  200
        if(s5k4ecgx_status.scene != EXT_CFG_SCENE_NIGHTSHOT)
        {    
            S5K4ECGX_WRITE_LIST(s5k4ecgx_AF_Normal_mode_3);
            if (s5k4ecgx_status.afmode == EXT_CFG_AF_SET_MACRO)mdelay(400);
            else mdelay(100);//previous value  200
        }		

        printk("[S5K4ECGX]Camera Sensor Power OFF\n");
        mclk_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        /* workaround code for AF ACL noise issue */
        if(s5k4ecgx_status.camera_status != SNAPSHOT)
        {
            printk("[S5K4ECGX]=========> Power OFF DELAY : 300ms\n");
            mdelay(50);//previous value  150
    }
#ifdef WORKAROUND_FOR_LOW_SPEED_I2C
        //before power off, below function will be called.
        pcam_msm_i2c_pwr_mgmt(s5k4ecgx_client->adapter, 0); 
#endif        
        s5k4ecgx_status.power_on = false;

        gpio_set_value(31, 0);  // CAM_VT_nSTBY Disable

        printk("I2C Disable \n"); 
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
        mdelay(2);
    
        gpio_set_value(132, 0); // CAM_VT_nRST Disable
        udelay(5);
        gpio_set_value(174, 0); //CAM_MEGA_nRST Disable
        udelay(60);
        
        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_DISABLE);
        udelay(5);

        gpio_set_value(175, 0);     //CAM_MEGA_STBY Disable
        udelay(5);
        /* Deinitailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);
        mdelay(1); 

        //Entering shutdown mode
        if (vreg_disable(vreg_ldo11)) {  //Power down VDDIO 1.8V and 1.3Mcore 1.8V
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_disable failed\n", __func__);
        }
        mdelay(1);
        if (vreg_disable(vreg_ldo20)) {  //Power down AVDD 2.8V 
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K4ECGX]%s: reg_disable failed\n", __func__);
        }
        mdelay(1);

#if 1
//paul_1013
        //LDO3 1.8v
        lp8720_i2c_write(0x08, 0x32);
        mdelay(1);
        //LDO2 2.8v
        lp8720_i2c_write(0x08, 0x30);
        mdelay(1);
        //LDO5 2.8v
        lp8720_i2c_write(0x08, 0x20);
        mdelay(1);
        //LDO Core 1.2v
        lp8720_i2c_write(0x08, 0x00);
        mdelay(1);
#endif
        gpio_set_value(2, 0);
        mdelay(1);

#endif
    }
    printk("[S5K4ECGX]s5k4ecgx_I2C_client :%x\n",s5k4ecgx_client);
    
}

static int s5k4ecgx_probe_init_sensor()
{
    int rc = 0;
    printk("[S5K4ECGX]s5k4ecgx_probe_init_sensor()\n");
    
#if defined(CONFIG_MACH_ANCORA)
    /* CAM RESET(GPIO 0) set to HIGH */
    gpio_set_value(175, 1);        // standby
    msleep(15);
    gpio_set_value(174, 1);      // reset
    msleep(50);
#endif

    s5k4ecgx_status.power_on = true;
    s5k4ecgx_status.id = 0x00;

#ifndef CONFIG_LOAD_FILE
    //S5K4ECGX_WRITE_LIST(s5k4ecgx_init_reg1);
    rc = s5k4ecgx_sensor_burst_write(s5k4ecgx_init_reg1,
            (sizeof(s5k4ecgx_init_reg1) / sizeof(s5k4ecgx_init_reg1[0])),
            "s5k4ecgx_init_reg1");
    msleep(50);
#endif    



#if 0//PGH I2C SPEED TEST
    unsigned int    before_time, after_time, i;//I2C SPEED TEST
    before_time = get_jiffies_64();
    for (i = 0; i < 3000; i++) 
    {
        s5k4ecgx_sensor_write(0x002E, 0x0040);
    }       
    after_time = get_jiffies_64();
    CAMDRV_DEBUG("Total Time 3000: %d\n",  jiffies_to_msecs(after_time-before_time));
#endif//PGH I2C SPEED TEST

#if 0
    unsigned short id = 0; //CAM FOR FW
    s5k4ecgx_sensor_write(0x002C, 0x7000);
    s5k4ecgx_sensor_write(0x002E, 0x01AC);
    s5k4ecgx_sensor_read(0x0F12, &id);
    CAMDRV_DEBUG("SENSOR FW VERSION : 0x%x \n", id);
#endif
    return rc;
}

#if 0//PGH
static long s5k4ecgx_reg_init(void)
{
    int32_t array_length;
    int32_t i;
    long rc;

    /* PLL Setup Start */
    rc = s5k4ecgx_i2c_write_table(&s5k4ecgx_regs.plltbl[0],
                    s5k4ecgx_regs.plltbl_size);

    if (rc < 0)
        return rc;
    /* PLL Setup End   */

    array_length = s5k4ecgx_regs.prev_snap_reg_settings_size;

    /* Configure sensor for Preview mode and Snapshot mode */
    for (i = 0; i < array_length; i++) {
        rc = s5k4ecgx_i2c_write(s5k4ecgx_client->addr,
          s5k4ecgx_regs.prev_snap_reg_settings[i].register_address,
          s5k4ecgx_regs.prev_snap_reg_settings[i].register_value,
          WORD_LEN);

        if (rc < 0)
            return rc;
    }

    /* Configure for Noise Reduction, Saturation and Aperture Correction */
    array_length = s5k4ecgx_regs.noise_reduction_reg_settings_size;

    for (i = 0; i < array_length; i++) {
        rc = s5k4ecgx_i2c_write(s5k4ecgx_client->addr,
            s5k4ecgx_regs.noise_reduction_reg_settings[i].register_address,
            s5k4ecgx_regs.noise_reduction_reg_settings[i].register_value,
            WORD_LEN);

        if (rc < 0)
            return rc;
    }

    /* Set Color Kill Saturation point to optimum value */
    rc =
    s5k4ecgx_i2c_write(s5k4ecgx_client->addr,
    0x35A4,
    0x0593,
    WORD_LEN);
    if (rc < 0)
        return rc;

    rc = s5k4ecgx_i2c_write_table(&s5k4ecgx_regs.stbl[0],
                    s5k4ecgx_regs.stbl_size);
    if (rc < 0)
        return rc;

    rc = s5k4ecgx_set_lens_roll_off();
    if (rc < 0)
        return rc;

    return 0;
}
#endif//PGH

//static int16_t s5k4ecgx_effect = CAMERA_EFFECT_OFF;
static long s5k4ecgx_config_effect(int mode, int effect)
{
    return 0;
/*
    long rc = 0;
    switch (mode) {
        case SENSOR_PREVIEW_MODE:
            //CAMDRV_DEBUG("SENSOR_PREVIEW_MODE\n");
        break;
        case SENSOR_SNAPSHOT_MODE:
            //CAMDRV_DEBUG("SENSOR_SNAPSHOT_MODE\n");
        break;
        default:
            //CAMDRV_DEBUG("[PGH] %s default\n", __func__);
        break;
    }

    switch (effect) {
        case CAMERA_EFFECT_OFF: 
            //CAMDRV_DEBUG("CAMERA_EFFECT_OFF\n");
        break;
        case CAMERA_EFFECT_MONO: 
            //CAMDRV_DEBUG("CAMERA_EFFECT_MONO\n");
        break;
        case CAMERA_EFFECT_NEGATIVE:
            //CAMDRV_DEBUG("CAMERA_EFFECT_NEGATIVE\n");
        break;
        case CAMERA_EFFECT_SOLARIZE:
            //CAMDRV_DEBUG("CAMERA_EFFECT_SOLARIZE\n");
        break;
        case CAMERA_EFFECT_SEPIA:
            //CAMDRV_DEBUG("CAMERA_EFFECT_SEPIA\n");
        break;
        default: 
            //printk("[S5K4ECGX]unexpeceted effect  %s/%d\n", __func__, __LINE__);
        return -EINVAL;
    }
    s5k4ecgx_effect = effect;
    
    return rc;
    */
}

static long s5k4ecgx_set_sensor_mode(int mode)
{
    CAMDRV_DEBUG("s5k4ecgx_set_sensor_mode : %d\n",mode);
    switch (mode) 
    {
        case SENSOR_PREVIEW_MODE:
            s5k4ecgx_set_preview();
        break;
        case SENSOR_SNAPSHOT_MODE:
        case SENSOR_RAW_SNAPSHOT_MODE:
            if(s5k4ecgx_status.camera_status != SNAPSHOT)s5k4ecgx_set_capture();
        break;
        default:
            return -EINVAL;
    }
    return 0;
}

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

static char *s5k4ecgx_regs_table = NULL;
static int s5k4ecgx_regs_table_size;

void s5k4ecgx_regs_table_init(void)
{
    struct file *filp;
    char *dp;
    long l;
    loff_t pos;
    int ret;
    mm_segment_t fs = get_fs();

    printk("%s %d\n", __func__, __LINE__);
    set_fs(get_ds());

#if defined(CONFIG_MACH_APACHE)
    filp = filp_open("/data/s5k4ecgx_I847.h", O_RDONLY, 0);
#elif defined(CONFIG_MACH_ANCORA)
    if(s5k4ecgx_status.id == 0x0011)
        filp = filp_open("/mnt/sdcard/s5k4ecgx.h", O_RDONLY, 0);
    else
        filp = filp_open("/mnt/sdcard/s5k4ecgx.h", O_RDONLY, 0);
#endif
    if (IS_ERR(filp)) {
        printk("[S5K4ECGX]file open error\n");
        return;
    }
    l = filp->f_path.dentry->d_inode->i_size;    
    CAMDRV_DEBUG("l = %ld\n", l);
    dp = vmalloc(l);
    if (dp == NULL) {
        printk("[S5K4ECGX]Out of Memory\n");
        filp_close(filp, current->files);
    }
    pos = 0;
    memset(dp, 0, l);
    ret = vfs_read(filp, (char __user *)dp, l, &pos);
    if (ret != l) {
        printk("[S5K4ECGX]Failed to read file ret = %d\n", ret);
        vfree(dp);
        filp_close(filp, current->files);
        return;
    }
    filp_close(filp, current->files);
    set_fs(fs);

    s5k4ecgx_regs_table = dp;
    s5k4ecgx_regs_table_size = l;
    *((s5k4ecgx_regs_table + s5k4ecgx_regs_table_size) - 1) = '\0';
//    CAMDRV_DEBUG("s5k4ecgx_regs_table 0x%x, %ld\n", dp, l);
}

void s5k4ecgx_regs_table_exit(void)
{
    CAMDRV_DEBUG("%s %d\n", __func__, __LINE__);
    if (s5k4ecgx_regs_table) {
        vfree(s5k4ecgx_regs_table);
        s5k4ecgx_regs_table = NULL;
    }
}

static int s5k4ecgx_regs_table_write(char *name)
{
    char *start, *end, *reg;
    unsigned short addr, value;
    char reg_buf[7], data_buf[7];

    addr = value = 0;

    *(reg_buf + 6) = '\0';
    *(data_buf + 4) = '\0';

    if(s5k4ecgx_regs_table==NULL)
        return 0;
    
    start = strstr(s5k4ecgx_regs_table, name);
    end = strstr(start, "};");

    while (1) {
        /* Find Address */
        reg = strstr(start,"{0x");
        if (reg)
            start = (reg + 12);
        if ((reg == NULL) || (reg > end))
            break;
        /* Write Value to Address */
        if (reg != NULL) {
            memcpy(reg_buf, (reg + 1), 6);
            memcpy(data_buf, (reg + 7), 4);
            addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
            value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
            //CAMDRV_DEBUG("[S5K4ECGX]addr 0x%04x, value 0x%04x\n", addr, value);
            if (addr == 0xffff)
            {
                msleep(value);
                CAMDRV_DEBUG("[S5K4ECGX]delay 0x%04x, value 0x%04x\n", addr, value);
            }
            else
            {
                if( s5k4ecgx_sensor_write(addr, value) < 0 )
                {
                    printk("[S5K4ECGX]%s fail on sensor_write\n", __func__);
                }
            }
        }
        else
            printk("[S5K4ECGX]EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
    }
    return 0;
}
#endif

//latin_cam : support anti-banding
#if defined(CONFIG_SAMSUNG_LTN_COMMON)

typedef enum {
    CAM_ANTIBANDING_50HZ = 50,
    CAM_ANTIBANDING_60HZ = 60
} cam_antibanding_setting;

cam_antibanding_setting s5k4ecgx_antibanding = CAM_ANTIBANDING_60HZ;

int s5k4ecgx_get_anti_banding(void)
{
    return s5k4ecgx_antibanding;
}

void s5k4ecgx_reg_init_60hz(void)
{
    CAMDRV_DEBUG("Init Table for Anti-banding 60 Hz\n");

    #define COPY_FROM_60HZ_TABLE(TABLE_NAME, ANTI_BANDING_SETTING) \
    memcpy (TABLE_NAME, TABLE_NAME##_##ANTI_BANDING_SETTING, \
            sizeof(TABLE_NAME))

    COPY_FROM_60HZ_TABLE (s5k4ecgx_init_reg2, 60hz);
    COPY_FROM_60HZ_TABLE (s5k4ecgx_ISO_Auto, 60hz);
//    COPY_FROM_60HZ_TABLE (s5k4ecgx_ISO_Auto_MWB_on, 60hz);
    COPY_FROM_60HZ_TABLE (s5k4ecgx_Scene_Default, 60hz);
}

int s5k4ecgx_check_table_size(void)
{
#define IS_SAME_NUM_OF_ROWS(TABLE_NAME) \
    (sizeof(TABLE_NAME) == sizeof(TABLE_NAME##_60hz))

    if ( !IS_SAME_NUM_OF_ROWS(s5k4ecgx_init_reg2) ) return (-1);
    if ( !IS_SAME_NUM_OF_ROWS(s5k4ecgx_ISO_Auto) ) return (-2);
//    if ( !IS_SAME_NUM_OF_ROWS(s5k4ecgx_ISO_Auto_MWB_on) ) return (-3);
    if ( !IS_SAME_NUM_OF_ROWS(s5k4ecgx_Scene_Default) ) return (-4);

    return 0;
}

void s5k4ecgx_regs_table_init_for_antibanding(void)
{
    printk("s5k4ecgx_regs_table_init_for_antibanding() : anti-banding ==> %d Hz\n", s5k4ecgx_get_anti_banding());

    if (CAM_ANTIBANDING_60HZ == s5k4ecgx_get_anti_banding())
        s5k4ecgx_reg_init_60hz();

    return;
}

ssize_t s5k4ecgx_antibanding_show (struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;

    count = sprintf(buf, "%d", s5k4ecgx_antibanding);
    printk("The value of s5k4ecgx_antibanding is %d\n", s5k4ecgx_antibanding);
    return count;
}

ssize_t s5k4ecgx_antibanding_store (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int tmp = 0;

    sscanf(buf, "%d", &tmp);
    if ((CAM_ANTIBANDING_50HZ == (cam_antibanding_setting) tmp)
            || (CAM_ANTIBANDING_60HZ == (cam_antibanding_setting) tmp)) {
        s5k4ecgx_antibanding = (cam_antibanding_setting) tmp;
        printk("The s5k4ecgx_antibanding is set to %d\n",s5k4ecgx_antibanding);
    }

    return count;
}

static struct device_attribute s5k4ecgx_antibanding_attr = {
    .attr = {
        .name = "anti-banding",
        .mode = (S_IRUGO | S_IWUGO)},
    .show = s5k4ecgx_antibanding_show,
    .store = s5k4ecgx_antibanding_store
};
#endif //CONFIG_SAMSUNG_LTN_COMMON
//hmin84.park 101201

int s5k4ecgx_sensor_init(const struct msm_camera_sensor_info *data)
{
    int rc = 0, i = 0;

    s5k4ecgx_ctrl = kzalloc(sizeof(struct s5k4ecgx_ctrl), GFP_KERNEL);
    if (!s5k4ecgx_ctrl) {
        CDBG("s5k4ecgx_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }

    if (data)
        s5k4ecgx_ctrl->sensordata = data;

    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    msleep(5);

    msm_camio_camif_pad_reg_reset();

    s5k4ecgx_set_power(true);
    rc = s5k4ecgx_probe_init_sensor();
    if(rc < 0)
    {
        for(i=0;i<5;i++)
        {
            printk("[S5K4ECGX]cam_fw_init failed. try again.\n");
            s5k4ecgx_set_power(false);
            mdelay(50);
            s5k4ecgx_set_power(true);
            msm_camio_camif_pad_reg_reset();
            msleep(5);
            rc = s5k4ecgx_probe_init_sensor();
            probe_init_retry++;
            if(rc >= 0)break;
        }
        if(rc < 0)goto init_fail;
    }

#ifdef CONFIG_LOAD_FILE
    s5k4ecgx_regs_table_init();
#endif

//latin_cam : support anti-banding
#if defined(CONFIG_SAMSUNG_LTN_COMMON)
    rc = s5k4ecgx_check_table_size();
    if (rc < 0) {
        printk("s5k4ecgx_check_table_size() failed. (rv : %d)\n", rc);
        goto init_fail;
    }
    s5k4ecgx_regs_table_init_for_antibanding();
#endif //CONFIG_SAMSUNG_LTN_COMMON
//hmin84.park 101203

init_done:
    return rc;

init_fail:
    kfree(s5k4ecgx_ctrl);
    return rc;
}

static int s5k4ecgx_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&s5k4ecgx_wait_queue);
    return 0;
}

int s5k4ecgx_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    long   rc = 0;

    if (copy_from_user(&cfg_data,
            (void *)argp,
            sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    /* down(&s5k4ecgx_sem); */
    CDBG("s5k4ecgx_ioctl, cfgtype = %d, mode = %d\n",
        cfg_data.cfgtype, cfg_data.mode);
        switch (cfg_data.cfgtype)
        {
            case CFG_SET_MODE:
                rc = s5k4ecgx_set_sensor_mode(
                        cfg_data.mode);
            break;
            case CFG_SET_EFFECT:
                rc = s5k4ecgx_config_effect(cfg_data.mode,
                        cfg_data.cfg.effect);
            break;
            case CFG_GET_AF_MAX_STEPS:
            default:
                rc = -EINVAL;
            break;
        }
    /* up(&s5k4ecgx_sem); */
    
    return rc;
}

int s5k4ecgx_sensor_release(void)
{
    int rc = 0;

    /* down(&s5k4ecgx_sem); */
    /*
    CAMDRV_DEBUG("lens moving to Base before CAM OFF\n");
    s5k4ecgx_sensor_write(0x0028, 0x7000);
    s5k4ecgx_sensor_write(0x002A, 0x0254);
    s5k4ecgx_sensor_write(0x0F12, 0x0030); //Lens Pistion (0x00 ~ 0xfF) normally (0x30 ~ 0x80)
    */

    s5k4ecgx_set_power(false);

    s5k4ecgx_status.camera_initailized = false;
    s5k4ecgx_status.need_configuration = 0;
    s5k4ecgx_status.effect = 0;
    s5k4ecgx_status.brightness = 0;
    s5k4ecgx_status.contrast = 0;
    s5k4ecgx_status.saturation = 0;
    s5k4ecgx_status.sharpness = 0;
    s5k4ecgx_status.whiteBalance = 0;
    s5k4ecgx_status.iso = 0;
    s5k4ecgx_status.auto_exposure = 0;
    s5k4ecgx_status.scene = 0;
    s5k4ecgx_status.afmode = EXT_CFG_AF_SET_NORMAL;
    s5k4ecgx_status.afcanceled = true;
    s5k4ecgx_status.dtp = 0;
    s5k4ecgx_status.flash_status = 0;
    s5k4ecgx_status.camera_on = false;
    
//    s5k4ecgx_set_power(false);
    //cpufreq_direct_set_policy(0, "ondemand");
    
    CAMDRV_DEBUG("s5k4ecgx_sensor_release\n");
    kfree(s5k4ecgx_ctrl);
    /* up(&s5k4ecgx_sem); */

#ifdef CONFIG_LOAD_FILE
    s5k4ecgx_regs_table_exit();
#endif

    return rc;
}

static int cam_pm_lp8720_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: lp8720 i2c probe failed\n", __func__);
        return -1;
    }

    lp8720_i2c_client = client;
    CAMDRV_DEBUG("lp8720 i2c probe successful\n");
    return 0;
}

static int cam_pm_lp8720_remove(struct i2c_client *client)
{
    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
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

static int s5k4ecgx_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int rc = 0;
    
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    s5k4ecgx_sensorw =
        kzalloc(sizeof(struct s5k4ecgx_work), GFP_KERNEL);

    if (!s5k4ecgx_sensorw) {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, s5k4ecgx_sensorw);
    s5k4ecgx_init_client(client);
    s5k4ecgx_client = client;
    
#ifdef USE_FLASHOFF_TIMER
    init_timer(&flashoff_timer);
    flashoff_timer.function = s5k4ecgx_flashoff_timer_handler;
    flashoff_timer.expires = jiffies + 60*HZ;
#endif

    printk("[S5K4ECGX]s5k4ecgx_probe succeeded!\n");

    return 0;

probe_failure:
    kfree(s5k4ecgx_sensorw);
    s5k4ecgx_sensorw = NULL;
    printk("[S5K4ECGX]s5k4ecgx_probe failed!\n");
    return rc;
}

static const struct i2c_device_id s5k4ecgx_i2c_id[] = {
    { "s5k4ecgx", 0},
    { },
};

static struct i2c_driver s5k4ecgx_i2c_driver = {
    .id_table = s5k4ecgx_i2c_id,
    .probe  = s5k4ecgx_i2c_probe,
    .remove = __exit_p(s5k4ecgx_i2c_remove),
    .driver = {
        .name = "s5k4ecgx",
    },
};


#ifdef FACTORY_CHECK
ssize_t camtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    char *sensorname = "NG";
#if 0
    if( camera_back_check ){
        switch (camera_active_type)
        {
            case CAMERA_ID_BACK:
                sensorname = "SONY_ISX005_NONE";
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
    sensorname = "LSI_S5K4ECGX_NONE";
    return sprintf(buf,"%s\n", sensorname);
}

ssize_t camtype_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}

static DEVICE_ATTR(camtype,0644, camtype_show, camtype_store);

extern struct class *sec_class;
struct device *sec_cam_dev = NULL;
#endif


static int s5k4ecgx_sensor_probe(const struct msm_camera_sensor_info *info,
                struct msm_sensor_ctrl *s)
{

    int rc = i2c_add_driver(&s5k4ecgx_i2c_driver);
    if (rc < 0){// || s5k4ecgx_client == NULL) {
        printk("[S5K4ECGX]s5k4ecgx_sensor_probe fail\n");
        rc = -ENOTSUPP;
        i2c_del_driver(&s5k4ecgx_i2c_driver);
        goto probe_done;
    }
    
    rc = i2c_add_driver(&lp8720_i2c_driver);
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/S5K4ECGX]%s lp8720 probe failed\n", __func__);
        rc = -ENOTSUPP;
        i2c_del_driver(&s5k4ecgx_i2c_driver);
        i2c_del_driver(&lp8720_i2c_driver);
        goto probe_done;
    }
    
    /*initailize states*/
    s5k4ecgx_status.camera_initailized = false;
    s5k4ecgx_status.need_configuration = 0;
    s5k4ecgx_status.camera_mode = EXT_CFG_CAM_MODE_CAMERA;
    s5k4ecgx_status.effect = EXT_CFG_EFFECT_NORMAL;
    s5k4ecgx_status.brightness = EXT_CFG_BR_STEP_0;
    s5k4ecgx_status.contrast = EXT_CFG_CR_STEP_0;
    s5k4ecgx_status.saturation = EXT_CFG_SA_STEP_0;
    s5k4ecgx_status.sharpness = EXT_CFG_SP_STEP_0;
    s5k4ecgx_status.whiteBalance = EXT_CFG_WB_AUTO;
    s5k4ecgx_status.iso = EXT_CFG_ISO_AUTO;
    s5k4ecgx_status.auto_exposure = EXT_CFG_METERING_NORMAL;
    s5k4ecgx_status.scene = EXT_CFG_SCENE_OFF;
    s5k4ecgx_status.afmode  = EXT_CFG_AF_SET_NORMAL;
    s5k4ecgx_status.afcanceled = true;
    s5k4ecgx_status.dtp = EXT_CFG_DTP_OFF;
    s5k4ecgx_status.snapshot_size = EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M;
    s5k4ecgx_status.preview_size = EXT_CFG_PREVIEW_SIZE_640x480_VGA;
    s5k4ecgx_status.flash_mode = EXT_CFG_FLASH_OFF;
    s5k4ecgx_status.flash_status = FLASH_OFF;
    s5k4ecgx_status.zoom = EXT_CFG_ZOOM_STEP_0;
    s5k4ecgx_status.current_lux = 0;
    s5k4ecgx_status.jpeg_quality = EXT_CFG_JPEG_QUALITY_SUPERFINE;
    s5k4ecgx_status.auto_contrast = EXT_CFG_AUTO_CONTRAST_OFF;
    s5k4ecgx_status.camera_on = false;
    
    /*sensor on/off for vfe initailization */
    s5k4ecgx_set_power(true);
    /* Input MCLK = 24MHz */
//    msm_camio_clk_rate_set(24000000);
   rc = s5k4ecgx_probe_init_sensor();

    s->s_init = s5k4ecgx_sensor_init;
    s->s_release = s5k4ecgx_sensor_release;
    s->s_config  = s5k4ecgx_sensor_config;
    s->s_camera_type = BACK_CAMERA_2D;
    s->s_mount_angle  = 0;
#if 1    
    s5k4ecgx_set_power(false);
#endif

probe_done:
    CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
    return rc;
}

static int __s5k4ecgx_probe(struct platform_device *pdev)
{
//latin_cam : support anti-banding
#if defined(CONFIG_SAMSUNG_LTN_COMMON)
    int rv;
    rv = device_create_file(&pdev->dev,&s5k4ecgx_antibanding_attr);
    if(rv)    
        printk("<=PCAM=> s5k4ecgx_probe() : device_create_file() is failed with %d", rv);
#endif //CONFIG_SAMSUNG_LTN_COMMON
//hmin84.park 101130

#ifdef FACTORY_CHECK                                                                      
    {                                                                                     
        if (sec_cam_dev == NULL)                                                          
        {                                                                                 
            sec_cam_dev = device_create(sec_class, NULL, 0, NULL, "sec_cam");             
            if (IS_ERR(sec_cam_dev))                                                      
                pr_err("Failed to create device(sec_cam_dev)!\n");                        
        }                                                                                 
                                                                                          
        if (sec_cam_dev != NULL && camtype_init == false)                                 
        {                                                                                 
            camtype_init = true;                                                          
            if (device_create_file(sec_cam_dev, &dev_attr_camtype) < 0)                   
                pr_err("Failed to create device file(%s)!\n", dev_attr_camtype.attr.name);
        }                                                                                 
    }                                                                                     
#endif        

    return msm_camera_drv_start(pdev, s5k4ecgx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __s5k4ecgx_probe,
    .driver = {
        .name = "msm_camera_s5k4ecgx",
        .owner = THIS_MODULE,
    },
};

static int __init s5k4ecgx_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

module_init(s5k4ecgx_init);



