/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, SAMSUNG. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/module.h> 
#include <media/msm_camera.h>
#include <linux/gpio.h>
#include <mach/camera.h>
#include <asm/gpio.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <mach/board.h>
#include "s5ka3dfx.h"

#define S5KA3DFX_WRITE_LIST(A) \
    {\
        s5ka3dfx_i2c_write_list(A,(sizeof(A) / sizeof(A[0])),#A);\
    }

/*    Read setting file from SDcard
    - There must be no "0x" string in comment. If there is, it cause some problem.
*/
//#define CONFIG_LOAD_FILE

struct s5ka3dfx_work_t {
    struct work_struct work;
};

static struct  s5ka3dfx_work_t *s5ka3dfx_sensorw;
static struct  i2c_client *s5ka3dfx_client;

struct s5ka3dfx_ctrl_t {
    int8_t  opened;
    struct  msm_camera_sensor_info     *sensordata;
    int dtp_mode;
    int vtcall_mode;
};

static struct s5ka3dfx_ctrl_t *s5ka3dfx_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(s5ka3dfx_wait_queue);
DECLARE_MUTEX(s5ka3dfx_sem);

#ifdef CONFIG_LOAD_FILE
static int s5ka3dfx_regs_table_write(char *name);
#endif
static int s5ka3dfx_start(void);
extern struct i2c_client *lp8720_i2c_client;
extern int lp8720_init;
extern bool vgacam_rotated;

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
    int rc;
    unsigned char buf[2];

    if(!lp8720_init)
        return -EIO;

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
        printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: lp8720_i2c_write failed: %d\n",__func__, rc);        
    
    return (rc == 1) ? 0 : -EIO;
}

int s5ka3dfx_i2c_tx_data(char* txData, int length)
{
    int rc; 

    struct i2c_msg msg[] = {
        {
            .addr = s5ka3dfx_client->addr,
            .flags = 0,
            .len = length,
            .buf = txData,        
        },
    };
    
    rc = i2c_transfer(s5ka3dfx_client->adapter, msg, 1);
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/S5KA3DFX] s5ka3dfx_i2c_tx_data error %d\n", rc);
        return rc;
    }

    return 0;
}

static int s5ka3dfx_i2c_read(unsigned short page, unsigned short subaddr, unsigned short *data)
{
    int ret;
    unsigned char buf[1] = {0};
    struct i2c_msg msg = { s5ka3dfx_client->addr, 0, 2, buf };

    /* page select */
    buf[0] = 0xFC;
    buf[1] = page;
    ret = i2c_transfer(s5ka3dfx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;
    
    /* read data */
    msg.buf[0] = subaddr;
    msg.len = 1;
    ret = i2c_transfer(s5ka3dfx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    msg.flags = I2C_M_RD;
    
    ret = i2c_transfer(s5ka3dfx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    //*data = ((buf[0] << 8) | buf[1]);
    *data = buf[0];

error:
    return ret;
}

static int s5ka3dfx_i2c_write(unsigned char u_addr, unsigned char u_data)
{
    unsigned char buf[2] = {0};
    struct i2c_msg msg = { s5ka3dfx_client->addr, 0, 2, buf };

    buf[0] = u_addr;
    buf[1] = u_data;

    //printk("addr : 0x%x , value : 0x%x\n",buf[0],buf[1]);
    return i2c_transfer(s5ka3dfx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int s5ka3dfx_i2c_write_read(u8 writedata_num, const u8* writedata, u8 readdata_num, u8* readdata)
{
  int err = 0, i = 0;
  struct i2c_msg msg[1];
  unsigned char writebuf[writedata_num];
  unsigned char readbuf[readdata_num];

  if (!s5ka3dfx_client->adapter)
  {
    printk("[CAMDRV/S5KA3DFX] can't search i2c client adapter\n");
    return -ENODEV;
  }

  /* Write */
  msg->addr  = s5ka3dfx_client->addr;
  msg->len   = writedata_num;
  memcpy(writebuf, writedata, writedata_num);    
  msg->buf   = writebuf;
  
  for(i = 0; i < 10; i++)  
  {
    err = i2c_transfer(s5ka3dfx_client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if(err == 0) break;
    mdelay(1);
  }

  if(i == 10)
  {
    printk("[CAMDRV/S5KA3DFX] s5ka3dfx_i2c_write_read is failed... %d\n", err);
    return err;  
  }

  /* Read */
  msg->addr  = s5ka3dfx_client->addr;
  msg->flags = I2C_M_RD;
  msg->len   = readdata_num;
  memset(readbuf, 0x0, readdata_num);
  msg->buf   = readbuf;
  
  for(i = 0; i < 10; i++)
  {
    err = i2c_transfer(s5ka3dfx_client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if (err == 0) 
    {
      memcpy(readdata, readbuf, readdata_num);
      return 0;
    }
    mdelay(1);
  }

  printk("[CAMDRV/S5KA3DFX] s5ka3dfx_i2c_write_read is failed... %d\n", err);

  return err;
}

static int s5ka3dfx_i2c_write_list(const unsigned short *list,int size, char *name)
{
    int ret = 0;
    int i;
    unsigned char addr, value;

#ifdef CONFIG_LOAD_FILE        
    ret = s5ka3dfx_regs_table_write(name);
#else
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] list name : %s\n",name);
    for (i = 0; i < size; i++)
    {
        addr = (unsigned char)((list[i] & 0xFF00)>>8);
        value =(unsigned char)( list[i] & 0x00FF);

        //printk("addr = 0x%x, value=0x%x \n",addr,value);
        if(addr == 0xff)
        {
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] delays for Snapshot - %d ms\n",value*10);
            msleep(value*10);
        }
        else
        {
            if(s5ka3dfx_i2c_write(addr, value) < 0)
            {
                printk("[CAMDRV/S5KA3DFX] sensor_write_list failed.\n");
                return -1;
            }
        }
        udelay(10);
    }
#endif
    return ret;
}

#ifdef CONFIG_LOAD_FILE
static char *s5ka3dfx_regs_table = NULL;

static int s5ka3dfx_regs_table_size;

void s5ka3dfx_regs_table_init(void)
{
    struct file *filp;
    char *dp;
    long l;
    loff_t pos;
//    int i;
    int ret;
    mm_segment_t fs = get_fs();

    printk("%s %d\n", __func__, __LINE__);

    set_fs(get_ds());

    filp = filp_open("/sdcard/5AAsensorsetting.h", O_RDONLY, 0);

    if (IS_ERR(filp)) {
        printk("file open error\n");
        return;
    }
    l = filp->f_path.dentry->d_inode->i_size;    
    printk("l = %ld\n", l);
    dp = kmalloc(l, GFP_KERNEL);
    if (dp == NULL) {
        printk("Out of Memory\n");
        filp_close(filp, current->files);
    }
    pos = 0;
    memset(dp, 0, l);
    ret = vfs_read(filp, (char __user *)dp, l, &pos);
    if (ret != l) {
        printk("Failed to read file ret = %d\n", ret);
        kfree(dp);
        filp_close(filp, current->files);
        return;
    }

    filp_close(filp, current->files);
    
    set_fs(fs);

    s5ka3dfx_regs_table = dp;
    
    s5ka3dfx_regs_table_size = l;

    *((s5ka3dfx_regs_table + s5ka3dfx_regs_table_size) - 1) = '\0';

    //printk("s5ka3dfx_regs_table 0x%x, %ld\n", dp, l);
}

void s5ka3dfx_regs_table_exit(void)
{
    printk("%s %d\n", __func__, __LINE__);
    if (s5ka3dfx_regs_table) {
        kfree(s5ka3dfx_regs_table);
        s5ka3dfx_regs_table = NULL;
    }    
}

static int s5ka3dfx_regs_table_write(char *name)
{
    char *start, *end, *reg;//, *data;    
    unsigned short addr, value;
    char reg_buf[3]={0,}, data_buf[3]={0,};

    addr = value = 0;

/*    *(reg_buf + 4) = '\0';
    *(data_buf + 4) = '\0';
*/
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] list name : %s\n",name);

    start = strstr(s5ka3dfx_regs_table, name);
    
    end = strstr(start, "};");

    while (1) {    
        /* Find Address */    
        reg = strstr(start,"0x");        
        if (reg)
            start = (reg + 7);
        if ((reg == NULL) || (reg > end))
            break;
        /* Write Value to Address */    
        if (reg != NULL) {
            memcpy(reg_buf, (reg + 2), 2);    
            memcpy(data_buf, (reg + 4), 2);    

            addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
            value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 

            //printk("addr 0x%x, value 0x%x\n", addr, value);

            if(addr == 0xff)
            {
                printk(KERN_ERR "[CAMDRV/S5KA3DFX] Delays for Snapshot - %d ms\n",value*10);
                msleep(value*10);
            }    
            else
            {
                if( s5ka3dfx_i2c_write(addr, value) < 0 )
                {
                    printk(KERN_ERR "[CAMDRV/S5KA3DFX]  sensor_write_list fail...-_-\n");
                    return -1;
                }
            }
            udelay(10);    
        }
        else
            printk(KERN_ERR "[CAMDRV/S5KA3DFX]  EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
    }

    return 0;
}
#endif

void s5ka3dfx_set_power(int onoff)
{
    unsigned int mclk_cfg; 

    if (gpio_request(2, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_LDO_EN' failed\n", __func__);
    if (gpio_request(3, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_EN2' failed\n", __func__);
    if (gpio_request(31, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_VT_standby' failed\n", __func__);
    if (gpio_request(132, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_VT_rst' failed\n", __func__);    
    if (gpio_request(174, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_MEGA_nRST' failed\n", __func__);
    if (gpio_request(175, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_MEGA_EN' failed\n", __func__);
    if (gpio_request(177, "s5ka3dfx"))printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: gpio request 'CAM_MEGA_EN' failed\n", __func__);
    
    if(onoff)
    {
        printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: POWER ON.\n", __func__);

        mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);        
        gpio_direction_output(2, 0);
        gpio_direction_output(3, 0);    
        gpio_direction_output(31, 0);    
        gpio_direction_output(132, 0);
        gpio_direction_output(174, 0);
        gpio_direction_output(175, 0);    
        gpio_direction_output(177, 0);    
    
        /* Main standby, Main reset set to LOW */
        gpio_set_value(174, 0);        
        gpio_set_value(175, 0);
        
        /* M_I/O 1.8v(GPIO_177), I_Core 1.2v(LP8720 bucksw), M_Core 1.2v(LP8720 LDO1) set to LOW*/
        gpio_set_value(177, 0);
        
        /* VT_Core 1.8v(CAM_ISP_RAM_1.8v, LP8720 LDO 4) set to HIGH */
        lp8720_i2c_write(0x04, 0x11);            // 000 10001

        /* VT_I/O 1.8v(CAM_ISP_HOST_1.8v, LP8720 LDO 3) set to HIGH */
        lp8720_i2c_write(0x03, 0x2C);            // 001 01100

        /* LP8720 0x8 register set */
        lp8720_i2c_write(0x08, 0xCC);

        /* VT_AVDD 2.8v(CAM_SENSOR_A2.8v, GPIO_003) set to HIGH */
        gpio_set_value(3, 1);
        udelay(50);

       /* LP8720 enable */
        gpio_set_value(2, 1);
        udelay(350);
        
        /* VGA Standby set to HIGH */
        gpio_set_value(31,1);
        udelay(50);
        
        /* MCLK enable */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        mdelay(5);
        
        /* VGA Reset set to HIGH */
        gpio_set_value(132,1);

        /* Delay for first I2C communication */
        mdelay(10);
    }
    else
    {
        printk(KERN_ERR "[CAMDRV] %s: POWER OFF.\n", __func__);
        
        mclk_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        gpio_direction_output(2, 1);
        gpio_direction_output(3, 1);    
        gpio_direction_output(31, 1);    
        gpio_direction_output(132, 1);
        gpio_direction_output(174, 0);
        gpio_direction_output(175, 0);    
        gpio_direction_output(177, 0);

        /* VGA Reset set to LOW */
        gpio_set_value(132,0);
        udelay(50);
        
        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        udelay(50);

        /* VGA Standby set to LOW */
        gpio_set_value(31,0);
        udelay(50);    

        /* LP8720 disable */
        lp8720_i2c_write(0x08, 0x00);
        gpio_set_value(2, 0);        

        /* VT_AVDD 2.8v(CAM_SENSOR_A2.8v, GPIO_003) set to LOW */
        gpio_set_value(3, 0);
    }
    
    gpio_free(2);
    gpio_free(3);
    gpio_free(31);
    gpio_free(132);
    gpio_free(174);
    gpio_free(175);
    gpio_free(177);

    return;
}

void s5ka3dfx_set_prevew(void)
{
    unsigned short value =0;
    int shade_value = 0;
    unsigned short agc_value = 0;
    
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] SENSOR_PREVIEW_MODE START\n");

    if(!s5ka3dfx_ctrl->dtp_mode) {
        if(s5ka3dfx_ctrl->vtcall_mode) {
            S5KA3DFX_WRITE_LIST(s5ka3dfx_init_vt_reg);
        } else {
            S5KA3DFX_WRITE_LIST(s5ka3dfx_init_reg);
        }
        msleep(300);
    }
}
void s5ka3dfx_set_snapshot(void)
{
#ifdef NOT_USE
    unsigned short value =0;
    int shade_value = 0;
    unsigned short agc_value = 0;
    
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] SENSOR_SNAPSHOT_MODE START\n");        
    printk("<=PCAM=> SENSOR_SNAPSHOT_MODE\n");

    s5ka3dfx_i2c_read(0x00,0xE5,&value);
    printk(KERN_ERR "[CAMDRV/S5KA3DFX]  0x00,0xE5 value : 0x%x\n",value);
    shade_value = value * 0x100;
    s5ka3dfx_i2c_read(0x00,0xE6,&value);
    printk(KERN_ERR "[CAMDRV/S5KA3DFX]  0x00,0xE6 value : 0x%x\n",value);
    shade_value += value;
    if((shade_value >= reg_AgcValue_list[3]) && (shade_value <= reg_AgcValue_list[4]))
    {
        s5ka3dfx_i2c_write_list(reg_shade_dnp_list,sizeof(reg_shade_dnp_list)/sizeof(reg_shade_dnp_list[0]),"reg_shade_dnp_list");
    }
    else 
    {
        s5ka3dfx_i2c_write_list(reg_shade_etc_list,sizeof(reg_shade_etc_list)/sizeof(reg_shade_etc_list[0]),"reg_shade_etc_list");                
    }

    s5ka3dfx_i2c_read(0x00,0xf8,&agc_value);
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] agc_value = 0x%x\n",agc_value);

    if(agc_value >= reg_AgcValue_list[2])
    {
        s5ka3dfx_i2c_write_list(reg_snapshot_lowlight_list,sizeof(reg_snapshot_lowlight_list)/sizeof(reg_snapshot_lowlight_list[0]),"reg_snapshot_lowlight_list");
    }
    else if(agc_value >= reg_AgcValue_list[1])
    {
        s5ka3dfx_i2c_write_list(reg_snapshot_mid_lowlight_list,sizeof(reg_snapshot_mid_lowlight_list)/sizeof(reg_snapshot_mid_lowlight_list[0]),"reg_snapshot_mid_lowlight_list");
    }
    else if(agc_value <= reg_AgcValue_list[0])
    {
        s5ka3dfx_i2c_write_list(reg_snapshot_list,sizeof(reg_snapshot_list)/sizeof(reg_snapshot_list[0]),"reg_snapshot_list");
    }
    else
    {
        s5ka3dfx_i2c_write_list(reg_snapshot_midlight_list,sizeof(reg_snapshot_midlight_list)/sizeof(reg_snapshot_midlight_list[0]),"reg_snapshot_midlight_list");
    }
    //test
    printk("<=PCAM=> so many 200msecdelay~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    //mdelay(200);
    msleep(200);
#endif    
}

static long s5ka3dfx_set_sensor_mode(int mode)
{
    switch (mode) 
    {
        case SENSOR_PREVIEW_MODE:
            s5ka3dfx_set_prevew();
            break;
        case SENSOR_SNAPSHOT_MODE:
            s5ka3dfx_set_snapshot();
            break;
        case SENSOR_SNAPSHOT_TRANSFER:
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] SENSOR_SNAPSHOT_TRANSFER START\n");
            break;
        default:
            return -EFAULT;
    }
    return 0;
}

static long s5ka3dfx_set_effect(int mode,int8_t effect)
{
    long rc = 0;
    switch(effect)
    {
/*        default:
            printk("[Effect]Invalid Effect !!!\n");
            return -EINVAL;
*/
    }
    return rc;
}

static int s5ka3dfx_reset(void)
{
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] s5ka3dfx_reset");

    s5ka3dfx_set_power(0);
    mdelay(5);
    s5ka3dfx_set_power(1);
    mdelay(5);
    return 0;
}

static int s5ka3dfx_set_ev(int8_t ev)
{
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] ev : %d \n",ev);

    switch(ev)
    {
        case S5KA3DFX_EV_MINUS_4:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_m4);
        break;
        
        case S5KA3DFX_EV_MINUS_3:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_m3);
        break;
        
        case S5KA3DFX_EV_MINUS_2:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_m2);
        break;
        
        case S5KA3DFX_EV_MINUS_1:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_m1);
        break;
        
        case S5KA3DFX_EV_DEFAULT:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_default);
        break;
        
        case S5KA3DFX_EV_PLUS_1:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_p1);
        break;
        
        case S5KA3DFX_EV_PLUS_2:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_p2);
        break;    
        
        case S5KA3DFX_EV_PLUS_3:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_p3);
        break;
        
        case S5KA3DFX_EV_PLUS_4:
            S5KA3DFX_WRITE_LIST(s5ka3dfx_ev_p4);
        break;
        
        default:
            printk("[EV] Invalid EV !!!\n");
            return -EINVAL;
    }

    return 0;
}

static int s5ka3dfx_set_dtp(int onoff)
{
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] dtp onoff : %d ",onoff);

    switch(onoff)
    {
        case S5KA3DFX_DTP_OFF:
            if(s5ka3dfx_ctrl->dtp_mode)s5ka3dfx_reset();
            s5ka3dfx_ctrl->dtp_mode = 0;
            s5ka3dfx_set_sensor_mode(SENSOR_PREVIEW_MODE);
            break;
        
        case S5KA3DFX_DTP_ON:
            //s5ka3dfx_reset();
            s5ka3dfx_ctrl->dtp_mode = 1;
            S5KA3DFX_WRITE_LIST(s5ka3dfx_dataline);
            break;
        
        default:
            printk("[DTP]Invalid DTP mode!!!\n");
            return -EINVAL;
    }
    return 0;
}

static int s5ka3dfx_set_fps_mode(unsigned int mode)
{
    printk(KERN_ERR "[CAMDRV/S5KA3DFX]  %s -mode : %d \n",__FUNCTION__,mode);

    if(mode) { //fixed
        printk(KERN_ERR "[CAMDRV/S5KA3DFX] mode change to CAMCORDER_MODE");
        s5ka3dfx_ctrl->vtcall_mode = 1;
    } else { //auto
        printk(KERN_ERR "[CAMDRV/S5KA3DFX] mode change to CAMERA_MODE");
        s5ka3dfx_ctrl->vtcall_mode = 0;
    }
    //we will use a auto mode on i9001 project. 2011-04-05
    s5ka3dfx_ctrl->vtcall_mode = 0;
    return 0;
}

static int s5ka3dfx_set_blur(unsigned int vt_mode, unsigned int blurlevel)
{
    int err = -EINVAL;
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] vt_mode : %d, blur [%d] \n", vt_mode, blurlevel);

    if(vt_mode == 1) {
        switch(blurlevel)
        {
            case BLUR_LEVEL_0:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_vt_none);
                break;
            case BLUR_LEVEL_1:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_vt_p1);
                break;
            case BLUR_LEVEL_2:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_vt_p2);
                break;
            case BLUR_LEVEL_3:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_vt_p3);
                break;
            default:
                printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: Not Support value \n", __func__);
                err = 0;
                break;

        }
    } else {
        switch(blurlevel)
        {
            case BLUR_LEVEL_0:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_none);
                break;
            case BLUR_LEVEL_1:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_p1);
                break;
            case BLUR_LEVEL_2:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_p2);
                break;
            case BLUR_LEVEL_3:
                S5KA3DFX_WRITE_LIST(s5ka3dfx_blur_p3);
                break;

            default:
                printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s: Not Support value \n", __func__);
                err = 0;
                break;
        }
    }
    return err;
}

static int s5ka3dfx_start(void)
{
    int rc = 0;
    u8 data[2] = {0xEF, 0x01};
    u8 vender[1] = {0xC5};
    u8 version[1] = {0xC6};
    u8 vendor_id = 0xff, sw_ver = 0xff;    
    
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s E\n",__func__);
    rc = s5ka3dfx_i2c_write(0xEF, 0x01);
    rc = s5ka3dfx_i2c_write_read(1, vender, 1, &vendor_id);
    rc = s5ka3dfx_i2c_write(0xEF, 0x01);    
    rc = s5ka3dfx_i2c_write_read(1, version, 1, &sw_ver);

    printk("[CAMDRV/S5KA3DFX]=================================\n");
    printk("[CAMDRV/S5KA3DFX]  [VGA CAM] vendor_id ID : 0x%x\n", vendor_id);
    printk("[CAMDRV/S5KA3DFX]  [VGA CAM] software version : 0x%x\n", sw_ver);
    printk("[CAMDRV/S5KA3DFX]=================================\n");

    return rc;
}

static int s5ka3dfx_sensor_init_probe(struct msm_camera_sensor_info *data)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] s5ka3dfx_sensor_init_probe start");
    s5ka3dfx_set_power(1);
    s5ka3dfx_start();

    return rc;

init_probe_fail:
    return rc;
}

int s5ka3dfx_sensor_init(struct msm_camera_sensor_info *data)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s E\n",__func__);
    
    s5ka3dfx_ctrl = kzalloc(sizeof(struct s5ka3dfx_ctrl_t), GFP_KERNEL);
    if (!s5ka3dfx_ctrl) {
        CDBG("s5ka3dfx_sensor_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }    
    
    if (data)
        s5ka3dfx_ctrl->sensordata = data;

    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);

    msm_camio_camif_pad_reg_reset();

#ifdef CONFIG_LOAD_FILE
    s5ka3dfx_regs_table_init();
#endif

    rc = s5ka3dfx_sensor_init_probe(data);
    if (rc < 0) {
        CDBG("s5ka3dfx_sensor_init failed!\n");
        goto init_fail;
    }

init_done:
    return rc;

init_fail:
    kfree(s5ka3dfx_ctrl);
    return rc;
}

static int s5ka3dfx_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&s5ka3dfx_wait_queue);
    return 0;
}

int s5ka3dfx_sensor_ext_config(void __user *argp)
{
    sensor_ext_cfg_data        cfg_data;
    int rc=0;

    if(copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data))) {
        printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s fail copy_from_user!\n", __func__);
    }
    
    switch(cfg_data.cmd) {
        case EXT_CFG_SET_BRIGHTNESS:
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] EXT_CFG_SET_BRIGHTNESS (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = s5ka3dfx_set_ev(cfg_data.value_1);
            break;
        case EXT_CFG_SET_DTP:
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] EXT_CFG_SET_DTP (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = s5ka3dfx_set_dtp(cfg_data.value_1);
            if(cfg_data.value_1 == 0) {
                cfg_data.value_1 = 2;
            } else if(cfg_data.value_1 == 1) {
                cfg_data.value_1 = 3;
            }        
            break;
        case EXT_CFG_SET_FPS_MODE:
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] EXT_CFG_SET_FPS_MODE (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = s5ka3dfx_set_fps_mode(cfg_data.value_1);
            break;
        case EXT_CFG_SET_BLUR:
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] EXT_CFG_SET_BLUR (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = s5ka3dfx_set_blur(s5ka3dfx_ctrl->vtcall_mode, cfg_data.value_1);
            break;
        case EXT_CFG_SET_FRONT_CAMERA_MODE:
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] EXT_CFG_SET_FRONT_CAMERA_MODE (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            s5ka3dfx_ctrl->vtcall_mode = cfg_data.value_1;
            break;
        case EXT_CFG_GET_VGACAM_ROTATED:
            cfg_data.value_1 = (int)vgacam_rotated;
            printk(KERN_ERR "[CAMDRV/S5KA3DFX] EXT_CFG_GET_CAMETA_ROTATED (%d)\n",cfg_data.value_1);
            break;			
        default:
            break;
    }

    if(copy_to_user((void *)argp, (const void *)&cfg_data, sizeof(cfg_data))) {
        printk(" %s : copy_to_user Failed \n", __func__);
    }
    
    return rc;    
}

int s5ka3dfx_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    long   rc = 0;

    if (copy_from_user(
                &cfg_data,
                (void *)argp,
                sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    CDBG("s5ka3dfx_sensor_config, cfgtype = %d, mode = %d\n",
        cfg_data.cfgtype, cfg_data.mode);

    switch (cfg_data.cfgtype) {
    case CFG_SET_MODE:
        rc = s5ka3dfx_set_sensor_mode(cfg_data.mode);
        break;

    case CFG_SET_EFFECT:
        rc = s5ka3dfx_set_effect(cfg_data.mode, cfg_data.cfg.effect);
        break;
        
    default:
        rc = -EFAULT;
        break;
    }
    return rc;
}

static int s5ka3dfx_sensor_release(void)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s E\n",__FUNCTION__);

    s5ka3dfx_set_power(0);
    kfree(s5ka3dfx_ctrl);
#ifdef CONFIG_LOAD_FILE
    s5ka3dfx_regs_table_exit();
#endif
    return rc;
}

static int s5ka3dfx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s E\n",__FUNCTION__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    s5ka3dfx_sensorw =
        kzalloc(sizeof(struct s5ka3dfx_work_t), GFP_KERNEL);

    if (!s5ka3dfx_sensorw) {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, s5ka3dfx_sensorw);
    s5ka3dfx_init_client(client);
    s5ka3dfx_client = client;

    CDBG("s5ka3dfx_i2c_probe successed!\n");
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s X\n",__FUNCTION__);

    return 0;

probe_failure:
    kfree(s5ka3dfx_sensorw);
    s5ka3dfx_sensorw = NULL;
    CDBG("s5ka3dfx_i2c_probe failed!\n");
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] s5ka3dfx_i2c_probe failed!\n");
    return rc;
}

static int __exit s5ka3dfx_i2c_remove(struct i2c_client *client)
{
    struct s5ka3dfx_work_t *sensorw = i2c_get_clientdata(client);
    free_irq(client->irq, sensorw);
    //i2c_detach_client(client);
    s5ka3dfx_client = NULL;
    s5ka3dfx_sensorw = NULL;
    kfree(sensorw);
    return 0;
}

static const struct i2c_device_id s5ka3dfx_id[] = {
    { "s5ka3dfx_i2c", 0 },
    { }
};

static struct i2c_driver s5ka3dfx_i2c_driver = {
    .id_table    = s5ka3dfx_id,
    .probe      = s5ka3dfx_i2c_probe,
    .remove     = __exit_p(s5ka3dfx_i2c_remove),
    .driver     = {
        .name = "s5ka3dfx",
    },
};

int32_t s5ka3dfx_i2c_init(void)
{
    int32_t rc = 0;

    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s E\n",__FUNCTION__);
    rc = i2c_add_driver(&s5ka3dfx_i2c_driver);

    if (IS_ERR_VALUE(rc))
        goto init_failure;

    return rc;

init_failure:
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] failed to s5ka3dfx_i2c_init, rc = %d\n", rc);
    i2c_del_driver(&s5ka3dfx_i2c_driver);
    return rc;
}

int s5ka3dfx_sensor_probe(const struct msm_camera_sensor_info *info,    struct msm_sensor_ctrl *s)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s E\n",__FUNCTION__);

    rc = s5ka3dfx_i2c_init();
    if (rc < 0)
        goto probe_fail;

    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);

    if (!lp8720_init){
        rc = -EIO;
        goto probe_fail;
    }

    s->s_init        = s5ka3dfx_sensor_init;
    s->s_release    = s5ka3dfx_sensor_release;
    s->s_config    = s5ka3dfx_sensor_config;
    s->s_camera_type = FRONT_CAMERA_2D;

    printk(KERN_ERR "[CAMDRV/S5KA3DFX] S5KA3DFX sensor probe successful\n");
    return rc;

probe_fail:
    printk(KERN_ERR "[CAMDRV/S5KA3DFX] %s:probe failed\n", __func__);
    return rc;
}

static int __devinit __s5ka3dfx_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, s5ka3dfx_sensor_probe);
}

static struct platform_driver msm_vga_camera_driver = {
    .probe = __s5ka3dfx_probe,
    .driver = {
        .name = "msm_camera_s5ka3dfx",
        .owner = THIS_MODULE,
    },
};

static int __init s5ka3dfx_init(void)
{
    printk(KERN_INFO "[CAMDRV/S5KA3DFX] %s: E\n", __func__);
    return platform_driver_register(&msm_vga_camera_driver);
}

module_init(s5ka3dfx_init);
MODULE_DESCRIPTION("LSI S5KA3DFX VGA camera driver");
MODULE_LICENSE("GPL v2");

