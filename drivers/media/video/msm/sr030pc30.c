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
#include <mach/vreg.h>

#ifdef CONFIG_SENSOR_SR030PC30_T679
#include "sr030pc30_T679.h"
#else
#include "sr030pc30.h"
#endif


#ifdef CONFIG_SENSOR_SR030PC30_T679
#define CAM_MEGA_RST 174
#define CAM_MEGA_STANDBY 175
#define CAM_VGA_RST 132
#define CAM_VGA_STANDBY 31
#define CAM_FLASH_ENSET 57
#define CAM_FLASH_FLEN 56
#else
#define CAM_FLASH_ENSET 56
#define CAM_FLASH_FLEN 57
#endif

#define SR030PC30_WRITE_LIST(A) \
    {\
        sr030pc30_i2c_write_list(A,(sizeof(A) / sizeof(A[0])),#A);\
    }

/*    Read setting file from SDcard
    - There must be no "0x" string in comment. If there is, it cause some problem.
*/
//#define CONFIG_LOAD_FILE

struct sr030pc30_work_t {
    struct work_struct work;
};

static struct  sr030pc30_work_t *sr030pc30_sensorw;
static struct  i2c_client *sr030pc30_client;

extern struct class *sec_class;
extern struct device *sec_cam_dev;

struct sr030pc30_ctrl_t {
    int8_t  opened;
    struct  msm_camera_sensor_info     *sensordata;
    int dtp_mode;
    int vtcall_mode;
};

static struct sr030pc30_ctrl_t *sr030pc30_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(sr030pc30_wait_queue);
DECLARE_MUTEX(sr030pc30_sem);

#ifdef CONFIG_LOAD_FILE
static int sr030pc30_regs_table_write(char *name);
#endif
static int sr030pc30_start(void);
static int sr030pc30_set_fps(unsigned int mode);
extern struct i2c_client *lp8720_i2c_client;
//extern int lp8720_init;

static int8_t cur_ev_value = SR030PC30_EV_DEFAULT;
static int prev_vtcall_mode=-1;
static int b_VGA_mirror=0;
static int mPreviewRegistersSet = 0;    // Flag to avoid setting preview registers again and again. Set this to 1 first time when preview is started. 
static unsigned int g_FpsMode=0;
static unsigned int g_bfps_set=0;
extern int b_esd_detected; //ESD
static int esd_enabled=0;

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
    int rc;
    unsigned char buf[2];

  //  if(!lp8720_init)
  //      return -EIO;

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
        printk(KERN_ERR "[CAMDRV/SR030PC30] %s: lp8720_i2c_write failed: %d\n",__func__, rc);        
    
    return (rc == 1) ? 0 : -EIO;
}

int sr030pc30_i2c_tx_data(char* txData, int length)
{
    int rc; 

    struct i2c_msg msg[] = {
        {
            .addr = sr030pc30_client->addr,
            .flags = 0,
            .len = length,
            .buf = txData,        
        },
    };
    
    rc = i2c_transfer(sr030pc30_client->adapter, msg, 1);
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/SR030PC30] sr030pc30_i2c_tx_data error %d\n", rc);
        return rc;
    }

    return 0;
}
#if 1
static int sr030pc30_i2c_read(unsigned short subaddr, unsigned short *data)
{
	//printk("<=ASWOOGI=> sr030pc30_i2c_read\n");

	int ret;
	unsigned char buf[1] = {0};
	struct i2c_msg msg = { sr030pc30_client->addr, 0, 1, buf };
	
	buf[0] = subaddr;
//	buf[1] = 0x0;

	ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

//	*data = ((buf[0] << 8) | buf[1]);
	*data = buf[0];

error:
	//printk("[ASWOOGI] on read func  sr030pc30_client->addr : %x\n",  sr130pc10_client->addr);    
	//printk("[ASWOOGI] on read func  subaddr : %x\n", subaddr);
	//printk("[ASWOOGI] on read func  data : %x\n", data);

    
	return ret;
}

#else
static int sr030pc30_i2c_read(unsigned short page, unsigned short subaddr, unsigned short *data)
{
    int ret;
    unsigned char buf[1] = {0};
    struct i2c_msg msg = { sr030pc30_client->addr, 0, 2, buf };

    /* page select */
    buf[0] = 0xFC;
    buf[1] = page;
    ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;
    
    /* read data */
    msg.buf[0] = subaddr;
    msg.len = 1;
    ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    msg.flags = I2C_M_RD;
    
    ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    //*data = ((buf[0] << 8) | buf[1]);
    *data = buf[0];

error:
    return ret;
}
#endif

static int sr030pc30_i2c_write(unsigned char u_addr, unsigned char u_data)
{
    unsigned char buf[2] = {0};
    struct i2c_msg msg = { sr030pc30_client->addr, 0, 2, buf };

    buf[0] = u_addr;
    buf[1] = u_data;

    //printk("addr : 0x%x , value : 0x%x\n",buf[0],buf[1]);
    return i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int sr030pc30_i2c_write_read(u8 writedata_num, const u8* writedata, u8 readdata_num, u8* readdata)
{
  int err = 0, i = 0;
  struct i2c_msg msg[1];
  unsigned char writebuf[writedata_num];
  unsigned char readbuf[readdata_num];

  if (!sr030pc30_client->adapter)
  {
    printk("[CAMDRV/SR030PC30] can't search i2c client adapter\n");
    return -ENODEV;
  }

  /* Write */
  msg->addr  = sr030pc30_client->addr;
  msg->len   = writedata_num;
  memcpy(writebuf, writedata, writedata_num);    
  msg->buf   = writebuf;
  
  for(i = 0; i < 10; i++)  
  {
    err = i2c_transfer(sr030pc30_client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if(err == 0) break;
    mdelay(1);
  }

  if(i == 10)
  {
    printk("[CAMDRV/SR030PC30] sr030pc30_i2c_write_read is failed... %d\n", err);
    return err;  
  }

  /* Read */
  msg->addr  = sr030pc30_client->addr;
  msg->flags = I2C_M_RD;
  msg->len   = readdata_num;
  memset(readbuf, 0x0, readdata_num);
  msg->buf   = readbuf;
  
  for(i = 0; i < 10; i++)
  {
    err = i2c_transfer(sr030pc30_client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if (err == 0) 
    {
      memcpy(readdata, readbuf, readdata_num);
      return 0;
    }
    mdelay(1);
  }

  printk("[CAMDRV/SR030PC30] sr030pc30_i2c_write_read is failed... %d\n", err);

  return err;
}

static int sr030pc30_i2c_write_list(const unsigned short *list,int size, char *name)
{
    int ret = 0;
    int i;
    unsigned char addr, value;

#ifdef CONFIG_LOAD_FILE        
    ret = sr030pc30_regs_table_write(name);
#else
    printk(KERN_ERR "[CAMDRV/SR030PC300] list name : %s\n",name);
    for (i = 0; i < size; i++)
    {
        addr = (unsigned char)((list[i] & 0xFF00)>>8);
        value =(unsigned char)( list[i] & 0x00FF);

        //printk("addr = 0x%x, value=0x%x \n",addr,value);
        if(addr == 0xff)
        {
            printk(KERN_ERR "[CAMDRV/SR030PC30] sensor_write_list Delays: %d ms\n",value);
            msleep(value);
        }
        else
        {
            if(sr030pc30_i2c_write(addr, value) < 0)
            {
                printk("[CAMDRV/SR030PC30] sensor_write_list failed.\n");
                return -1;
            }
        }
        udelay(10);
    }
#endif
    return ret;
}

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>
static char *sr030pc30_regs_table = NULL;

static int sr030pc30_regs_table_size;

void sr030pc30_regs_table_init(void)
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

#ifdef CONFIG_SENSOR_SR030PC30_T679
    filp = filp_open("/mnt/sdcard/external_sd/sr030pc30_T679.h", O_RDONLY, 0);
#else
    filp = filp_open("/sdcard/5AAsensorsetting.h", O_RDONLY, 0);
#endif

    if (IS_ERR(filp)) {
        printk("file open error\n");
        return;
    }
    l = filp->f_path.dentry->d_inode->i_size;    
    printk("l = %ld\n", l);
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
        vfree(dp);
        filp_close(filp, current->files);
        return;
    }

    filp_close(filp, current->files);
    
    set_fs(fs);

    sr030pc30_regs_table = dp;
    
    sr030pc30_regs_table_size = l;

    *((sr030pc30_regs_table + sr030pc30_regs_table_size) - 1) = '\0';

    //printk("sr030pc30_regs_table 0x%x, %ld\n", dp, l);
}

void sr030pc30_regs_table_exit(void)
{
    printk("%s %d\n", __func__, __LINE__);
    if (sr030pc30_regs_table) {
        vfree(sr030pc30_regs_table);
        sr030pc30_regs_table = NULL;
    }    
}

static int sr030pc30_regs_table_write(char *name)
{
    char *start, *end, *reg;//, *data;    
    unsigned short addr, value;
    char reg_buf[3]={0,}, data_buf[3]={0,};

    addr = value = 0;

/*    *(reg_buf + 4) = '\0';
    *(data_buf + 4) = '\0';
*/
    printk(KERN_ERR "[CAMDRV/SR030PC30] list name : %s\n",name);

    start = strstr(sr030pc30_regs_table, name);
    
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
                printk(KERN_ERR "[CAMDRV/SR030PC30] sr030pc30_regs_table_write Delays: %d ms\n",value);
                msleep(value);
            }    
            else
            {
                if( sr030pc30_i2c_write(addr, value) < 0 )
                {
                    printk(KERN_ERR "[CAMDRV/SR030PC30]  sr030pc30_regs_table_write fail...-_-\n");
                    return -1;
                }
            }
            udelay(10);    
        }
        else
            printk(KERN_ERR "[CAMDRV/SR030PC30]  EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
    }

    return 0;
}
#endif

#ifdef CONFIG_SENSOR_SR030PC30_T679
static void sr030pc30_set_power_rev00( int onoff)
{
    unsigned int mclk_cfg; 
    struct vreg *vreg_ldo20;
    vreg_ldo20 = vreg_get(NULL, "gp13");
    mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        
    if(!vreg_ldo20){
        CAMDRV_DEBUG("[SR030PC30]%s: VREG L20 get failed\n", __func__);
    }

    CAMDRV_DEBUG("(vreg_set_level(vreg_ldo20, 1800 \n");  

    if(vreg_set_level(vreg_ldo20, 1800)){
        CAMDRV_DEBUG("[SR030PC30]%s: vreg_set_level failed\n", __func__);
    }
    if(onoff == 1) //POWER ON
    {
        CAMDRV_DEBUG("initailize power control pin \n");  

        /* initailize power control pin */    
        gpio_set_value(CAM_MEGA_RST, 0);
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        gpio_set_value(CAM_VGA_STANDBY, 0);
        gpio_set_value(CAM_VGA_RST, 0);


        CAMDRV_DEBUG("CAM_FLASH_ENSET \n");  

        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);


        CAMDRV_DEBUG("LDO Core 1.2v \n");  


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
            printk("[SR030PC30]%s: reg_enable failed\n", __func__);
        }

        CAMDRV_DEBUG("I2C Enable \n");  
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);            
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);                      
        mdelay(30);   
    
        CAMDRV_DEBUG("VGA CAM_STANDBY High \n");       
        gpio_set_value(CAM_VGA_STANDBY, 1);
        
        mdelay(5); // >4ms
#if 0
        CAMDRV_DEBUG("MCLK 24Mhz Enabled \n");   
        msm_camio_clk_rate_set(24000000);
    
        msm_camio_camif_pad_reg_reset();
        
        udelay(1);        

        CAMDRV_DEBUG("MCLK High \n");
        gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif
        /* Enable MCLK */
        CAMDRV_DEBUG("MCLK High \n");
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);

        udelay(20); // >10us

        CAMDRV_DEBUG("Main CAM_STANDBY High \n");       
        gpio_set_value(CAM_MEGA_STANDBY, 1);
        mdelay(6); // >5ms


        CAMDRV_DEBUG("Main CAM_RST High \n");           
        gpio_set_value(CAM_MEGA_RST, 1);
        mdelay(10); // >6.5ms


        CAMDRV_DEBUG("Main CAM_STANDBY Low \n");       
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        udelay(10); // >10us     

        CAMDRV_DEBUG("VGA CAM_RST High \n");       
        gpio_set_value(CAM_VGA_RST, 1);               
        mdelay(90); // >50ms
   

    }
    else //POWER OFF
    {
        CAMDRV_DEBUG("[SR030PC30]Camera Sensor Power OFF\n");

        CAMDRV_DEBUG("Main CAM_RST Low \n");
        gpio_set_value(CAM_MEGA_RST, 0);
        mdelay(1);
        
        CAMDRV_DEBUG("Main CAM_STANDBY Low \n");
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        mdelay(1);
        
        CAMDRV_DEBUG("VGA CAM_RST Low \n");
        gpio_set_value(CAM_VGA_RST, 0);         
        mdelay(1);
        
        CAMDRV_DEBUG("VGA CAM_STANDBY Low \n");
        gpio_set_value(CAM_VGA_STANDBY, 0); 
        mdelay(1);

#ifdef WORKAROUND_FOR_LOW_SPEED_I2C
        //before power off, below function will be called.
        pcam_msm_i2c_pwr_mgmt(SR030PC30_client->adapter, 0); 
#endif        

//        gpio_tlmm_config(GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

        /* Disable MCLK */
        CAMDRV_DEBUG("MCLK Low \n");
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_DISABLE);

        /* LP8720 enable */
        lp8720_i2c_write(0x08, 0x00);

        if(vreg_disable(vreg_ldo20)){
            CAMDRV_DEBUG("[SR030PC30]%s: reg_enable failed\n", __func__);
        }

        gpio_set_value(CAM_MEGA_RST, 0);      //nRST
        udelay(50);
        udelay(50);
         //vreg_disable(vreg_ldo20);

        /*initailize power control pin*/    
        
    
        CAMDRV_DEBUG("I2C Disable \n"); 
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);                    
        mdelay(10);


        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);
        mdelay(1); // to enter a shutdown mode        
    }
}

static void sr030pc30_set_power_rev01( int onoff)
{
    unsigned int mclk_cfg; 
    struct vreg *vreg_cam_out8_vddio;
    struct vreg *vreg_cam_out9_vdda;
    struct vreg *vreg_cam_out10_vddreg;
    struct vreg *vreg_cam_out17_af; 
    struct vreg *vreg_cam_gp15_vddreg;

    vreg_cam_out8_vddio        = vreg_get(NULL, "gp13");
    vreg_cam_out9_vdda        = vreg_get(NULL, "wlan2");
    vreg_cam_out10_vddreg    = vreg_get(NULL, "gp2");
    vreg_cam_out17_af        = vreg_get(NULL, "wlan");
    vreg_cam_gp15_vddreg    = vreg_get(NULL, "gp15");
    mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
    
    if(onoff== 1) //POWER ON
    {
        CAMDRV_DEBUG("[START] SUB Camera Sensor POWER ON Sequence \n");

        vreg_set_level(vreg_cam_out17_af,  2800);        // VDDAF 2.8V
        vreg_set_level(vreg_cam_out9_vdda,  2800);        // VDDA 2.8V
        vreg_set_level(vreg_cam_out10_vddreg, 1800);    // VGA Core 1.8V        
        vreg_set_level(vreg_cam_out8_vddio,  1800);    // VDDIO 1.8V        
        vreg_set_level(vreg_cam_gp15_vddreg,  1200);    // Mega Core VDD 1.2V            

        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);            
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);                    
        mdelay(20);
        
        CAMDRV_DEBUG("Sensor AVDD 2.8V On \n");    
        vreg_enable(vreg_cam_out9_vdda);

        CAMDRV_DEBUG("VT Sensor Core VDD_REG 1.8V On \n");                      
        vreg_enable(vreg_cam_out10_vddreg);
        udelay(20);

        CAMDRV_DEBUG("Main Sensor Core VDD 1.2V On \n");   
        vreg_enable(vreg_cam_gp15_vddreg);        
          udelay(15);
        

        CAMDRV_DEBUG("Sensor I/O VDDIO 1.8V On \n");                       
        vreg_enable(vreg_cam_out8_vddio);
        mdelay(30);

        CAMDRV_DEBUG("I2C Enable \n");  
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);            
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_ENABLE);                      
        mdelay(20);      


        CAMDRV_DEBUG("VGA CAM_STANDBY High \n");       
        gpio_set_value(CAM_VGA_STANDBY, 1);
        
        mdelay(5); // >4ms
#if 0
        CAMDRV_DEBUG("MCLK 24Mhz Enabled \n");   
        msm_camio_clk_rate_set(24000000);
    
        msm_camio_camif_pad_reg_reset();
        
        udelay(1);        

        CAMDRV_DEBUG("MCLK High \n");
        gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif

        /* Enable MCLK */
        CAMDRV_DEBUG("MCLK High \n");
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        udelay(20); // >10us

        CAMDRV_DEBUG("Main CAM_STANDBY High \n");       
        gpio_set_value(CAM_MEGA_STANDBY, 1);
        mdelay(6); // >5ms

        CAMDRV_DEBUG("Main CAM_RST High \n");           
        gpio_set_value(CAM_MEGA_RST, 1);
        mdelay(10); // >6.5ms


        CAMDRV_DEBUG("Main CAM_STANDBY Low \n");       
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        udelay(10); // >10us     

        CAMDRV_DEBUG("VGA CAM_RST High \n");       
        gpio_set_value(CAM_VGA_RST, 1);               
        mdelay(90); // >50ms
   

    }
    else //POWER OFF
    {
        CAMDRV_DEBUG("[SR030PC30]Camera Sensor Power OFF\n");

        CAMDRV_DEBUG("Main CAM_RST Low \n");
        gpio_set_value(CAM_MEGA_RST, 0);
        mdelay(1);
        
        CAMDRV_DEBUG("Main CAM_STANDBY Low \n");
        gpio_set_value(CAM_MEGA_STANDBY, 0);
        mdelay(1);
        
        CAMDRV_DEBUG("VGA CAM_RST Low \n");
        gpio_set_value(CAM_VGA_RST, 0);         
        mdelay(1);

        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_DISABLE);
        mdelay(1);
        
        CAMDRV_DEBUG("VGA CAM_STANDBY Low \n");
        gpio_set_value(CAM_VGA_STANDBY, 0); 
        mdelay(1);

        CAMDRV_DEBUG("Sensor I/O VDDIO 1.8V OFF \n");                       
        vreg_disable(vreg_cam_out8_vddio);

        CAMDRV_DEBUG("VT Sensor Core VDD_REG 1.8V OFF \n");                      
        vreg_disable(vreg_cam_out10_vddreg);

        CAMDRV_DEBUG("Sensor AVDD 2.8V OFF \n");    
        vreg_disable(vreg_cam_out9_vdda);

        CAMDRV_DEBUG("Main Sensor Core VDD 1.2V OFF \n");   
        vreg_disable(vreg_cam_gp15_vddreg);      

        CAMDRV_DEBUG("I2C Disable \n"); 
        gpio_tlmm_config(GPIO_CFG(0, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);            
        gpio_tlmm_config(GPIO_CFG(1, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) , GPIO_CFG_DISABLE);                    
         
        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);
        mdelay(1); // to enter a shutdown mode        
    }
}

static void sr030pc30_set_power(int onoff)
{
    extern int board_hw_revision;

    if (board_hw_revision == 3)
        sr030pc30_set_power_rev00(onoff);
    else
        sr030pc30_set_power_rev01(onoff);    
}


#else
void sr030pc30_set_power(int onoff)
{
    unsigned int mclk_cfg; 
    struct vreg *vreg_ldo20, *vreg_ldo11;

    vreg_ldo20 = vreg_get(NULL, "gp13");
    if (!vreg_ldo20) {
        printk("[S5K4ECGX]%s: VREG L20 get failed\n", __func__);
    }
    if (vreg_set_level(vreg_ldo20, 1800)) {
        printk("[S5K4ECGX]%s: vreg_set_level failed\n", __func__);
    }

    vreg_ldo11 = vreg_get(NULL, "gp2");
    if (!vreg_ldo11) {
        printk("[S5K4ECGX]%s: VREG L11 get failed\n", __func__);
    }
    if (vreg_set_level(vreg_ldo11, 2800)) {
        printk("[S5K4ECGX]%s: vreg_set_level failed\n", __func__);    
    }
    
    if(onoff)
    {
    	b_esd_detected=false; //ESD
        printk(KERN_ERR "[CAMDRV/SR030PC300] %s: POWER ON.\n", __func__);
        mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        
        /* initailize power control pin */    
        gpio_set_value(174, 0); // CAM_MEGA_nRST
        gpio_set_value(175, 0); // CAM_MERA_STBY
        gpio_set_value(31, 0);  // CAM_VT_nSTBY
        gpio_set_value(132, 0); // CAM_VT_nRST
        
        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET, 0);
        gpio_set_value(CAM_FLASH_FLEN, 0);  
        
        if (vreg_enable(vreg_ldo20)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR030PC300]%s: reg_enable failed\n", __func__);
        }
        if (vreg_enable(vreg_ldo11)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR030PC300]%s: reg_enable failed\n", __func__);
        }
        mdelay(1);
        
        /* Enable CAM_VT_nSTBY */
        gpio_set_value(31, 1);
        mdelay(1);

        /* Enable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        mdelay(20);

        /* Enable CAM_VT_nRST */
        gpio_set_value(132, 1);
    }
    else
    {
        printk(KERN_ERR "[CAMDRV] %s: POWER OFF.\n", __func__);
        mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

        /* Disable CAM_VT_nRST */
        gpio_set_value(132, 0);
        mdelay(20);

        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_DISABLE);
        mdelay(1);
        
        gpio_set_value(31, 0);    // VGA_STBY
        mdelay(1);

        if (vreg_disable(vreg_ldo11)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR030PC300]%s: reg_disable failed\n", __func__);
        }        
        if (vreg_disable(vreg_ldo20)) {
            printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR030PC300]%s: reg_disable failed\n", __func__);
        }
        mdelay(1);
    }
    mdelay(100);    // Power Stablity time.
    
    return;
}
#endif


void sr030pc30_set_preview(void)
{
    // If preview registers are already set, return. No need to write preview registers again.
    #ifdef CONFIG_SENSOR_SR030PC30_T679
        if(mPreviewRegistersSet)
        {
            CAMDRV_DEBUG(" mPreviewRegistersSet=%d. Preview registers not changed. Returning...\n", mPreviewRegistersSet);
            return;
        }

        mPreviewRegistersSet = 1;   // Setting preview registers for the 1st time. Set this flag to 1.
    #endif

    if(prev_vtcall_mode==sr030pc30_ctrl->vtcall_mode)
        return;
    
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_set_preview : dtp(%d), vt(%d)\n",sr030pc30_ctrl->dtp_mode, sr030pc30_ctrl->vtcall_mode);

    if(!sr030pc30_ctrl->dtp_mode) {
        if(sr030pc30_ctrl->vtcall_mode) {
            SR030PC30_WRITE_LIST(sr030pc30_init_vt_reg);
            SR030PC30_WRITE_LIST(sr030pc30_fps_15);
        } else {
            SR030PC30_WRITE_LIST(sr030pc30_init_reg);
            sr030pc30_set_fps(g_FpsMode);
            g_bfps_set = 1;
        }
        if(!sr030pc30_ctrl->vtcall_mode)
        {
            if(b_VGA_mirror)
            {
                SR030PC30_WRITE_LIST(sr030pc30_flip_water);
            }
            else
            {
                SR030PC30_WRITE_LIST(sr030pc30_flip_none);
            }
        }
        msleep(500);
    }
    prev_vtcall_mode=sr030pc30_ctrl->vtcall_mode;

    
//    SR030PC30_WRITE_LIST(sr030pc30_flip_none);
//    SR030PC30_WRITE_LIST(sr030pc30_flip_water);
//    SR030PC30_WRITE_LIST(sr030pc30_flip_mirror);
//    SR030PC30_WRITE_LIST(sr030pc30_flip_water_mirror);
    
}

void sr030pc30_set_snapshot(void)
{    
    printk(KERN_ERR "[CAMDRV/SR030PC300] SENSOR_SNAPSHOT_MODE START\n");        
    printk("<=PCAM=> SENSOR_SNAPSHOT_MODE\n");

    SR030PC30_WRITE_LIST(reg_self_capture_table);
    msleep(200);
}

static long sr030pc30_set_sensor_mode(int mode)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_set_sensor_mode (%d)\n", mode);
    
    switch (mode) 
    {
        case SENSOR_PREVIEW_MODE:
            sr030pc30_set_preview();
            break;
        case SENSOR_SNAPSHOT_MODE:
            sr030pc30_set_snapshot();
            break;
        case SENSOR_SNAPSHOT_TRANSFER:
            printk(KERN_ERR "[CAMDRV/SR030PC300] SENSOR_SNAPSHOT_TRANSFER START\n");
        sr030pc30_set_snapshot();
            break;
        default:
            return -EFAULT;
    }
    return 0;
}

static long sr030pc30_set_effect(int mode,int8_t effect)
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

static int sr030pc30_reset(void)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_reset");

    sr030pc30_set_power(0);
    mdelay(5);
    sr030pc30_set_power(1);
    mdelay(5);
    return 0;
}

static int sr030pc30_set_ev(int8_t ev)
{

    if(sr030pc30_ctrl->dtp_mode)
        return 0;
    
    printk(KERN_ERR "[CAMDRV/SR030PC300] ev : %d \n",ev);

    if(cur_ev_value == ev)
        return 0;

    cur_ev_value = ev;
    switch(ev)
    {
        case  SR030PC30_EV_MINUS_4:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m4);
        break;
        
        case  SR030PC30_EV_MINUS_3:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m3);
        break;
        
        case  SR030PC30_EV_MINUS_2:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m2);
        break;
        
        case  SR030PC30_EV_MINUS_1:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m1);
        break;
        
        case  SR030PC30_EV_DEFAULT:
            SR030PC30_WRITE_LIST(sr030pc30_ev_default);
        break;
        
        case  SR030PC30_EV_PLUS_1:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p1);
        break;
        
        case  SR030PC30_EV_PLUS_2:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p2);
        break;    
        
        case  SR030PC30_EV_PLUS_3:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p3);
        break;
        
        case  SR030PC30_EV_PLUS_4:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p4);
        break;
        
        default:
            printk("[EV] Invalid EV !!!\n");
            return -EINVAL;
    }

    return 0;
}

static int sr030pc30_set_dtp(int onoff)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300] dtp onoff : %d ",onoff);

    switch(onoff)
    {
        case  SR030PC30_DTP_OFF:
            sr030pc30_ctrl->dtp_mode = 0;
           SR030PC30_WRITE_LIST(sr030pc30_dataline_stop);
            break;
        
        case SR030PC30_DTP_ON:
            sr030pc30_ctrl->dtp_mode = 1;
        SR030PC30_WRITE_LIST(sr030pc30_init_reg);
            SR030PC30_WRITE_LIST(sr030pc30_dataline);
            break;
        
        default:
            printk("[DTP]Invalid DTP mode!!!\n");
            return -EINVAL;
    }
    return 0;
}

static int sr030pc30_set_fps_mode(unsigned int mode)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300]  %s -mode : %d \n",__FUNCTION__,mode);
    
    if((mode == EXT_CFG_FRAME_AUTO) || (mode > EXT_CFG_FRAME_FIX_30))
    { 
        printk(KERN_ERR "[CAMDRV/SR030PC300] mode change to CAMERA_MODE");
        sr030pc30_ctrl->vtcall_mode = 0;
    }     
    else
    {
        printk(KERN_ERR "[CAMDRV/SR030PC300] mode change to CAMCORDER_MODE");
        sr030pc30_ctrl->vtcall_mode = 1;
    }
   
    return 0;
}

static int sr030pc30_set_fps(unsigned int mode)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300]  %s : %d-fps \n",__FUNCTION__,mode);
#ifndef CONFIG_SENSOR_SR030PC30_T679

    g_FpsMode=mode;
	
    if(g_bfps_set)
        return 0;

    if(mode == EXT_CFG_FRAME_FIX_15)
    {
        SR030PC30_WRITE_LIST(sr030pc30_fps_15);
 
    }
    else if(mode == EXT_CFG_FRAME_FIX_20)
    {
        SR030PC30_WRITE_LIST(sr030pc30_fps_15);
//        SR030PC30_WRITE_LIST(sr030pc30_QVGA_fps_20);
    }
    else
    { 
        SR030PC30_WRITE_LIST(sr030pc30_fps_default);
    }     
#endif
    return 0;
}

static int sr030pc30_set_blur(unsigned int vt_mode, unsigned int blurlevel)
{
    int err = -EINVAL;
    printk(KERN_ERR "[CAMDRV/SR030PC300] vt_mode : %d, blur [%d] \n", vt_mode, blurlevel);

    if(vt_mode == 1) {
        switch(blurlevel)
        {
            case BLUR_LEVEL_0:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_none);
                break;
            case BLUR_LEVEL_1:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_p1);
                break;
            case BLUR_LEVEL_2:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_p2);
                break;
            case BLUR_LEVEL_3:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_p3);
                break;
            default:
                printk(KERN_ERR "[CAMDRV/SR030PC300] %s: Not Support value \n", __func__);
                err = 0;
                break;

        }
    } else {
        switch(blurlevel)
        {
            case BLUR_LEVEL_0:
                SR030PC30_WRITE_LIST(sr030pc30_blur_none);
                break;
            case BLUR_LEVEL_1:
                SR030PC30_WRITE_LIST(sr030pc30_blur_p1);
                break;
            case BLUR_LEVEL_2:
                SR030PC30_WRITE_LIST(sr030pc30_blur_p2);
                break;
            case BLUR_LEVEL_3:
                SR030PC30_WRITE_LIST(sr030pc30_blur_p3);
                break;

            default:
                printk(KERN_ERR "[CAMDRV/SR030PC300] %s: Not Support value \n", __func__);
                err = 0;
                break;
        }
    }
    return err;
}

static int sr030pc30_start(void)
{
    int rc = 0;
    u8 vendor_id = 0xff;
    
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__func__);

    rc = sr030pc30_i2c_write(0x03, 0x00);
    sr030pc30_i2c_read(0x04, &vendor_id);

    printk("[CAMDRV/SR030PC300]=================================\n");
    printk("[CAMDRV/SR030PC300]  [VGA CAM] vendor_id ID : 0x%x\n", vendor_id);

    return rc;
}

static int sr030pc30_sensor_init_probe(struct msm_camera_sensor_info *data)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_sensor_init_probe start");
    sr030pc30_set_power(1);
    sr030pc30_start();

    return rc;

init_probe_fail:
    return rc;
}

int sr030pc30_sensor_init(struct msm_camera_sensor_info *data)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__func__);
    
    sr030pc30_ctrl = kzalloc(sizeof(struct sr030pc30_ctrl_t), GFP_KERNEL);
    if (!sr030pc30_ctrl) {
        CDBG("sr030pc30_sensor_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }    
    prev_vtcall_mode=-1;

    if (data)
        sr030pc30_ctrl->sensordata = data;

    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);

    msm_camio_camif_pad_reg_reset();

#ifdef CONFIG_LOAD_FILE
    sr030pc30_regs_table_init();
#endif

    cur_ev_value = -1;
    g_bfps_set = 0;
    
    rc = sr030pc30_sensor_init_probe(data);
    if (rc < 0) {
        CDBG("sr030pc30_sensor_init failed!\n");
        goto init_fail;
    }

init_done:
    return rc;

init_fail:
    kfree(sr030pc30_ctrl);
    return rc;
}

static int sr030pc30_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&sr030pc30_wait_queue);
    return 0;
}

int sr030pc30_sensor_esd_detected() //ESD
{
    printk("[sr030pc30] ESD Detected!!\n");
    b_esd_detected=true;
    return 0;
}

int sr030pc30_sensor_ext_config(void __user *argp)

{
    sensor_ext_cfg_data cfg_data;
    int rc = 0;
    int exposureTime_value1 = 0, exposureTime_value2 = 0, exposureTime_value3 = 0;
    int exposureTime = 0;


    if (copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
    {
        printk(KERN_ERR "[CAMDRV/SR030PC300] %s fail copy_from_user!\n", __func__);
    }

    switch (cfg_data.cmd)
    {
    case EXT_CFG_SET_BRIGHTNESS:
        printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_BRIGHTNESS (%d %d)\n", cfg_data.cmd, cfg_data.value_1);
        rc = sr030pc30_set_ev(cfg_data.value_1);
        break;
    case EXT_CFG_SET_DTP:
        printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_DTP (%d %d)\n", cfg_data.cmd, cfg_data.value_1);
        rc = sr030pc30_set_dtp(cfg_data.value_1);
        if (cfg_data.value_1 == 0)
        {
            cfg_data.value_2 = 2;
        }
        else if (cfg_data.value_1 == 1)
        {
            cfg_data.value_2 = 3;
        }
        break;
    case EXT_CFG_SET_FPS_MODE:
        printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_FPS_MODE (%d %d)\n", cfg_data.cmd, cfg_data.value_1);

        rc = sr030pc30_set_fps_mode(cfg_data.value_1);
        break;
    case EXT_CFG_SET_FPS:
        printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_FPS (%d %d)\n", cfg_data.cmd, cfg_data.value_1);

        rc = sr030pc30_set_fps(cfg_data.value_1);
        break;
    case EXT_CFG_SET_BLUR:
        printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_BLUR (%d %d)\n", cfg_data.cmd, cfg_data.value_1);
        rc = sr030pc30_set_blur(sr030pc30_ctrl->vtcall_mode, cfg_data.value_1);
        break;
    case EXT_CFG_SET_FRONT_CAMERA_MODE:
        printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_FRONT_CAMERA_MODE (%d %d)\n", cfg_data.cmd, cfg_data.value_1);
        sr030pc30_ctrl->vtcall_mode = cfg_data.value_1;
        break;
    case EXT_CFG_GET_VGACAM_ROTATED:
        printk(KERN_ERR "[CAMDRV/SR030PC300] Set Camera Mirror (%d %d)\n", cfg_data.cmd, cfg_data.value_1);
        b_VGA_mirror = cfg_data.value_1;
        if (b_VGA_mirror)
        {
            SR030PC30_WRITE_LIST(sr030pc30_flip_water);
        }
        else
        {
            SR030PC30_WRITE_LIST(sr030pc30_flip_none);
        }
        msleep(300);
        break;


    case EXT_CFG_GET_EXIF_INFO:
        {
            printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_GET_EXIF_INFO E\n");

            /* read ISO */
            rc = sr030pc30_i2c_write(0x03, 0x20);
            sr030pc30_i2c_read(0xB0, &cfg_data.cmd);

            printk(KERN_ERR "[CAMDRV/SR030PC300] ISO **iso(%d)\n", cfg_data.cmd);

            /* Exposure Time */
            rc = sr030pc30_i2c_write(0x03, 0x20);
            sr030pc30_i2c_read(0x80, &cfg_data.value_1);
            sr030pc30_i2c_read(0x81, &cfg_data.value_2);
            sr030pc30_i2c_read(0x82, &cfg_data.value_3);
            cfg_data.value_3 = 0xff & cfg_data.value_3;
            exposureTime = 12000000 / ((cfg_data.value_3 << 3) | (cfg_data.value_2 << 11) | (cfg_data.value_1 << 19));
            printk("[SR130PC10] cfg_data.value_1=0x%x, cfg_data.value_2=0x%x, cfg_data.value_3=0x%x\n", \
                   cfg_data.value_1, cfg_data.value_2, cfg_data.value_3);
            cfg_data.value_1 = exposureTime;
            printk("[SR130PC10] exposureTime=%d\n", exposureTime);
        }
        break;


        case EXT_CFG_TEST_ESD: //TELECA_ESD 
          printk("[SR030PC300] Reset the Sensor for the ESD case from HAL\n");
          
          if(cfg_data.value_1 == 1)
          {
            printk(KERN_ERR "[SR030PC300]: ESD Value %d\n",b_esd_detected);
            cfg_data.value_2 = b_esd_detected;
            b_esd_detected=false;
          }
          else
          {
            if(esd_enabled)
            {
              printk("[SR030PC300] esd_enabled\n");
              return rc;
            }
              
            esd_enabled=1;
            mPreviewRegistersSet = 0;
            prev_vtcall_mode=-1;
            printk(KERN_ERR "[SR030PC300]:EXT_CFG_TEST_ESD Sensor Reset\n");
            sr030pc30_set_power(0);
            msleep(5);

//            msm_camio_camif_pad_reg_reset();
                
            sr030pc30_set_power(1);			
            msleep(5);
            sr030pc30_set_preview();		
          
          }
        break;
    default:
        break;
    }

    if (copy_to_user((void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
    {
        printk(" %s : copy_to_user Failed \n", __func__);
    }

    return rc;
}



int sr030pc30_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    long   rc = 0;

    if (copy_from_user(
                &cfg_data,
                (void *)argp,
                sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    CDBG("sr030pc30_sensor_config, cfgtype = %d, mode = %d\n",
        cfg_data.cfgtype, cfg_data.mode);

    switch (cfg_data.cfgtype) {
    case CFG_SET_MODE:
        rc = sr030pc30_set_sensor_mode(cfg_data.mode);
         //sr030pc30_set_ev(cur_ev_value);
        break;

    case CFG_SET_EFFECT:
        rc = sr030pc30_set_effect(cfg_data.mode, cfg_data.cfg.effect);
        break;
        
    default:
        rc = -EFAULT;
        break;
    }
    return rc;
}

static int sr030pc30_sensor_release(void)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);
    mPreviewRegistersSet = 0;   // Exiting sensor driver. Hence set this flag back to 0.
    esd_enabled=0;
    
    sr030pc30_set_power(0);
    kfree(sr030pc30_ctrl);
#ifdef CONFIG_LOAD_FILE
    sr030pc30_regs_table_exit();
#endif
    return rc;
}

static int sr030pc30_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    sr030pc30_sensorw =
        kzalloc(sizeof(struct sr030pc30_work_t), GFP_KERNEL);

    if (!sr030pc30_sensorw) {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, sr030pc30_sensorw);
    sr030pc30_init_client(client);
    sr030pc30_client = client;

    CDBG("sr030pc30_i2c_probe successed!\n");
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s X\n",__FUNCTION__);

    return 0;

probe_failure:
    kfree(sr030pc30_sensorw);
    sr030pc30_sensorw = NULL;
    CDBG("sr030pc30_i2c_probe failed!\n");
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_i2c_probe failed!\n");
    return rc;
}

static int __exit sr030pc30_i2c_remove(struct i2c_client *client)
{
    struct sr030pc30_work_t *sensorw = i2c_get_clientdata(client);
    free_irq(client->irq, sensorw);
    //i2c_detach_client(client);
    sr030pc30_client = NULL;
    sr030pc30_sensorw = NULL;
    kfree(sensorw);
    return 0;
}

static const struct i2c_device_id sr030pc30_id[] = {
    { "sr030pc30_i2c", 0 },
    { }
};

static struct i2c_driver sr030pc30_i2c_driver = {
    .id_table    = sr030pc30_id,
    .probe      = sr030pc30_i2c_probe,
    .remove     = __exit_p(sr030pc30_i2c_remove),
    .driver     = {
        .name = "sr030pc30",
    },
};

int32_t sr030pc30_i2c_init(void)
{
    int32_t rc = 0;

    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);
    rc = i2c_add_driver(&sr030pc30_i2c_driver);

    if (IS_ERR_VALUE(rc))
        goto init_failure;

    return rc;

init_failure:
    printk(KERN_ERR "[CAMDRV/SR030PC300] failed to sr030pc30_i2c_init, rc = %d\n", rc);
    i2c_del_driver(&sr030pc30_i2c_driver);
    return rc;
}

// /sys/devices/virtual/sec/sec_cam  
ssize_t cam_ver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 version = 0xff;    
    version = 0x8c;
    return sprintf(buf,"%d\n", version);
}

static DEVICE_ATTR(cam_ver,0644, cam_ver_show, NULL);


int sr030pc30_sensor_probe(const struct msm_camera_sensor_info *info,    struct msm_sensor_ctrl *s)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);

#if defined(CONFIG_MACH_APACHE)
    return rc;
#endif

    rc = sr030pc30_i2c_init();
    if (rc < 0)
        goto probe_fail;

#if 0
    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);
#endif    

 //   if (!lp8720_init){
    //    rc = -EIO;
   //     goto probe_fail;
 //   }

// /sys/devices/virtual/sec/sec_cam                                                                            
    if (sec_cam_dev == NULL)                                                          
    {                                                                                 
        sec_cam_dev = device_create(sec_class, NULL, 0, NULL, "sec_cam");             
        if (IS_ERR(sec_cam_dev))                                                      
            pr_err("Failed to create device(sec_cam_ver) for VGA ver!\n");                        
    }                                                                                 
                                                                                          
    if (sec_cam_dev != NULL)                                 
    {                                                                                 
        if (device_create_file(sec_cam_dev, &dev_attr_cam_ver) < 0)                   
        pr_err("Failed to create device file for VGA ver(%s)!\n", dev_attr_cam_ver.attr.name);
    }

    sr030pc30_set_power(1);
    rc=sr030pc30_start();
    sr030pc30_set_power(0);
    if (rc < 0)
    {
        printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30 sensor probe i2c version read fail\n");
        goto probe_fail;
    }
    
    s->s_init        = sr030pc30_sensor_init;
    s->s_release    = sr030pc30_sensor_release;
    s->s_config    = sr030pc30_sensor_config;
    s->s_camera_type = FRONT_CAMERA_2D;
    s->s_mount_angle = 180;
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30 sensor probe successful\n");
    return rc;

probe_fail:
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s: failed\n", __func__);
    return rc;
}

static int __init __sr030pc30_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, sr030pc30_sensor_probe);
}

static struct platform_driver msm_vga_camera_driver = {
    .probe = __sr030pc30_probe,
    .driver = {
        .name = "msm_camera_sr030pc30",
        .owner = THIS_MODULE,
    },
};

static int __init sr030pc30_init(void)
{
    printk(KERN_INFO "[CAMDRV/SR030PC300] %s: E\n", __func__);
    return platform_driver_register(&msm_vga_camera_driver);
}

module_init(sr030pc30_init);
MODULE_DESCRIPTION("LSI sr030pc30 VGA camera driver");
MODULE_LICENSE("GPL v2");
