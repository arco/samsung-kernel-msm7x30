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
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <media/msm_camera.h>
#include <linux/gpio.h>
#include <mach/camera.h>
#include "ce147.h"

/* values for firmware information */
static unsigned char MAIN_SW_FW[4] = {0x0, 0x0, 0x0, 0x0};    /* {FW Maj, FW Min, PRM Maj, PRM Min} */
static int MAIN_SW_DATE_INFO[3] = {0x0, 0x0, 0x0};        /* {Year, Month, Date} */

/* dzoom values */
static unsigned char ce147_buf_set_dzoom[31] = {0xff,0xe7,0xd3,0xc2,0xb4,0xa7,0x9c,0x93,0x8b,0x83,0x7c,0x76,0x71,0x6c,0x67,0x63,0x5f,0x5b,0x58,0x55,0x52,0x4f,0x4d,0x4a,0x48,0x46,0x44,0x42,0x41,0x40,0x3f};
static int DZoom_State = 0;

/* Used to save the i2c client structure */
static struct i2c_client *ce147_i2c_client;
struct i2c_client *lp8720_i2c_client;
int lp8720_init;
/* Sensor related informations are saved here */
static struct ce147_ctrls_t *ce147_ctrls;
struct ce147_status_t *ce147_status;
#define FACTORY_CHECK
#ifdef FACTORY_CHECK
static bool camtype_init = false;
static bool main_camera_is_on = false;
#endif

bool vgacam_rotated;

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

static int ce147_i2c_write_multi(unsigned char cmd, unsigned char *w_data, unsigned int w_len)
{
    int retry_count = 1;
    unsigned char buf[w_len+1];
    struct i2c_msg msg = {ce147_i2c_client->addr, 0, w_len+1, buf};

    int ret = -1;

    buf[0] = cmd;
    memcpy(buf+1, w_data, w_len);

//#ifdef CE147_DEBUG
#if 0
    {
        int j;
        printk("[CE147 I2C] W: ");
        for(j = 0; j <= w_len; j++){
            printk("0x%02x ", buf[j]);
        }
        printk("\n");
    }
#endif

    while(retry_count--){
        ret  = i2c_transfer(ce147_i2c_client->adapter, &msg, 1);
        if(ret == 1)
            break;
        msleep(10);
        }

    return (ret == 1) ? 0 : -EIO;
}

static int ce147_i2c_read_multi(unsigned char cmd, unsigned char *w_data, unsigned int w_len, 
        unsigned char *r_data, unsigned int r_len)
{
    unsigned char buf[w_len+1];
    struct i2c_msg msg = {ce147_i2c_client->addr, 0, w_len + 1, buf};
    int ret = -1;
    int retry_count = 1;

    buf[0] = cmd;
    memcpy(buf+1, w_data, w_len);

//#ifdef CE147_DEBUG
#if 0
    {
        int j;
        printk("[CE147 I2C] R: ");
        for(j = 0; j <= w_len; j++){
            printk("0x%02x ", buf[j]);
        }
        printk("\n");
    }
#endif

    while(retry_count--){
        ret = i2c_transfer(ce147_i2c_client->adapter, &msg, 1);
        if(ret == 1)
            break;
        msleep(10);
    }

    if(ret < 0)
        return -EIO;

    msg.flags = I2C_M_RD;
    msg.len = r_len;
    msg.buf = r_data;

    retry_count = 1;
    while(retry_count--){
        ret  = i2c_transfer(ce147_i2c_client->adapter, &msg, 1);
        if(ret == 1)
            break;
        msleep(10);
    }

    return (ret == 1) ? 0 : -EIO;
}

static int ce147_waitfordone_timeout(unsigned char cmd, unsigned char value, 
                    int timeout, int polling_interval)
{
    int err;
    unsigned char cam_status = 0xFF;
    unsigned long jiffies_start = jiffies;
    unsigned long jiffies_timeout = jiffies_start + msecs_to_jiffies(timeout);

    if(polling_interval < 0)
        polling_interval = POLL_TIME_MS;

    while(time_before(jiffies, jiffies_timeout)){
        cam_status = 0xFF;
        err = ce147_i2c_read_multi(cmd, NULL, 0, &cam_status, 1);
        if(err < 0)
            return -EIO;

        CAMDRV_DEBUG("Status check returns %02x\n", cam_status);

        if(cam_status == value) 
            break;

        msleep(polling_interval);
    }

    if(cam_status != value)
        return -EBUSY;
    else
        return jiffies_to_msecs(jiffies - jiffies_start);
}

static int ce147_get_batch_reflection_status(void)
{
    int err;        
    int end_cnt = 0;

    unsigned char ce147_buf_batch_data[1] = {0x00};
    unsigned char ce147_batch_ref_status = 0x00;

    err = ce147_i2c_write_multi(0x01, ce147_buf_batch_data, 1);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] %s: failed: i2c_write forget_batch_reflection_status\n", __func__);
        return -EIO;
    }

    //To Do: This code needs timeout API for do-while
    do
    {
        msleep(10); 
        err = ce147_i2c_read_multi(CMD_GET_BATCH_REFLECTION_STATUS, NULL, 0, &ce147_batch_ref_status, 1);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] %s: failed: i2c_read for get_batch_reflection_status\n", __func__);
            return -EIO;
        }
        end_cnt++;
    } while(ce147_batch_ref_status && end_cnt < 200);

    if(end_cnt > 5) 
    {
        CAMDRV_DEBUG("%s: count(%d) status(%02x) \n", __func__, end_cnt, ce147_batch_ref_status);
    }
        
    if (ce147_batch_ref_status != 0x00)
    {
        printk(KERN_ERR "[CAMDRV/CE147] %s: failed: to get_batch_reflection_status\n", __func__);
        return -EINVAL;
    }

    

    return 0;
}

static int ce147_read_fw_bin(const char *path, char *fwBin, int *fwSize)
{
    char*       buffer = NULL;
    unsigned int file_size = 0;

    struct file *filep = NULL;
    mm_segment_t old_fs;

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_read_main_SW_fw_version is called...\n");

    filep = filp_open(path, O_RDONLY, 0) ;

    if (filep && (filep != ERR_PTR(-ENOENT)))
    {
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        
        file_size = filep->f_op->llseek(filep, 0, SEEK_END);
        filep->f_op->llseek(filep, 0, SEEK_SET);
        
        buffer = (char*)kmalloc(file_size+1, GFP_KERNEL);
        
        filep->f_op->read(filep, buffer, file_size, &filep->f_pos);
        buffer[file_size] = '\0';
        
        filp_close(filep, current->files);

        set_fs(old_fs);

        printk(KERN_ERR "[CAMDRV/CE147] CE147 :File size : %d\n", file_size);
    }
    else
    {
        return -EINVAL;
    }

    memcpy(fwBin, buffer, file_size);
    *fwSize = file_size;

    kfree(buffer);

    return 0;
}

static int ce147_get_main_sw_fw_version(void)
{
    char fw_data[20] = {0, };
    int fw_size = 0;
    int main_sw_fw_prm_offset = 4;
    int main_sw_date_offset = 10;
    int err = 0;
    int i;

    printk(KERN_DEBUG "ce147_get_main_sw_fw_version Enter \n");
    if((MAIN_SW_DATE_INFO[0] == 0x00) && (MAIN_SW_DATE_INFO[1] == 0x00) && (MAIN_SW_DATE_INFO[2] == 0x00))
    {
        err = ce147_read_fw_bin(CE147_FW_F2_PATH, fw_data, &fw_size);
        if(err < 0)
        {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :fail : read main_sw_version \n");
            return err;
        }
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :fw_size : %d \n", fw_size);

#if 0 // kurtlee 
        for(i = 0; i < fw_size; i++)
        {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :fw_data : %x \n", fw_data[i]);
        }
#endif		

        /* Main SW FW/PRM info */
        MAIN_SW_FW[0] = fw_data[main_sw_fw_prm_offset];
        MAIN_SW_FW[1] = fw_data[main_sw_fw_prm_offset+1];
        MAIN_SW_FW[2] = fw_data[main_sw_fw_prm_offset+2];
        MAIN_SW_FW[3] = fw_data[main_sw_fw_prm_offset+3];

        /* Main SW Date info */
        MAIN_SW_DATE_INFO[0] = (2000 + fw_data[main_sw_date_offset]);
        MAIN_SW_DATE_INFO[1] = fw_data[main_sw_date_offset+1];
        MAIN_SW_DATE_INFO[2] = fw_data[main_sw_date_offset+2];

        printk(KERN_DEBUG "fw M:%d m:%d |prm M:%d m:%d \n", MAIN_SW_FW[0], MAIN_SW_FW[1], MAIN_SW_FW[2], MAIN_SW_FW[3]);
        printk(KERN_DEBUG "y. m. d = %d.%d.%d \n", MAIN_SW_DATE_INFO[0], MAIN_SW_DATE_INFO[1], MAIN_SW_DATE_INFO[2]);
    }
    else
    {
        printk(KERN_DEBUG "already read main sw version \n");
    }

    ce147_status->main_sw_fw.major = MAIN_SW_FW[0];
    ce147_status->main_sw_fw.minor = MAIN_SW_FW[1];
    ce147_status->main_sw_prm.major = MAIN_SW_FW[2];
    ce147_status->main_sw_prm.minor = MAIN_SW_FW[3];
    ce147_status->main_sw_dateinfo.year = MAIN_SW_DATE_INFO[0];
    ce147_status->main_sw_dateinfo.month = MAIN_SW_DATE_INFO[1];
    ce147_status->main_sw_dateinfo.date = MAIN_SW_DATE_INFO[2];
    
    return 0;
}

static int ce147_load_fw(void)
{
    int ce147_reglen_init = 1;
    unsigned char ce147_regbuf_init[1] = { 0x00 };
    int err;
    
        /** Just before this function call, we enable the power and clock. Hence
     *  we need to wait for some time before we can start communicating with the sensor.
     */
        //msleep(10);
    CAMDRV_DEBUG("%s START\n", __func__);
        err = ce147_i2c_write_multi(CMD_INIT, ce147_regbuf_init, ce147_reglen_init);
        if(err < 0)
                return -EIO;

        /* At least 700ms delay required to load the firmware for ce147 camera ISP */
        msleep(700);
    CAMDRV_DEBUG("%s END\n", __func__);
    ce147_status->runmode = CE147_RUNMODE_IDLE;

    return 0;
}


static int ce147_get_version(int object_id, unsigned char version_info[])
{
    unsigned char cmd_buf[1] = {0x00};
    unsigned int cmd_len = 1;
    unsigned int info_len = 4;
    int err;
    
    switch(object_id)
    {
    case DATA_VERSION_FW:
    case DATA_VERSION_DATE:
    case DATA_VERSION_SENSOR:
    case DATA_VERSION_SENSOR_MAKER:
    case DATA_VERSION_AF:            
        cmd_buf[0] = object_id;
        break;
    default:
        return -EINVAL;
    }

        err = ce147_i2c_read_multi( CMD_VERSION, cmd_buf, cmd_len, version_info, info_len);
        if(err < 0)
                return -EIO;

    return 0;
}

static int ce147_get_fw_version(void)
{
    unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
    int err = -1;

    err = ce147_get_version(DATA_VERSION_FW, version_info);

    if(err < 0) 
        return  err;

    ce147_status->fw.minor = version_info[0];
    ce147_status->fw.major = version_info[1];

    ce147_status->prm.minor = version_info[2];
    ce147_status->prm.major = version_info[3];

    return 0;
}

static int ce147_get_dateinfo(void)
{
    unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
    int err = -1;

    err = ce147_get_version(DATA_VERSION_DATE, version_info);

    if(err < 0) 
        return  err;

    ce147_status->dateinfo.year  = version_info[0] - 'A' + 2007;
    ce147_status->dateinfo.month = version_info[1] - 'A' + 1;
    ce147_status->dateinfo.date  = version_info[2];

    return 0;
}

static int ce147_get_sensor_version(void)
{
    unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
    int err = -1;

    err = ce147_get_version(DATA_VERSION_SENSOR, version_info);

    if(err < 0) 
        return  err;

    ce147_status->sensor_version = version_info[0];

    return 0;
}

static int ce147_get_sensor_maker_version(void)
{
    unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
    int err = -1;

    err = ce147_get_version(DATA_VERSION_SENSOR_MAKER, version_info);

    if(err < 0) 
        return  err;

    ce147_status->sensor_info.maker = version_info[0];
    ce147_status->sensor_info.optical = version_info[1];

    return 0;
}

static int ce147_get_af_version(void)
{
    unsigned char version_info[4] = {0x00, 0x00, 0x00, 0x00};
    int err = -1;

    err = ce147_get_version( DATA_VERSION_AF, version_info);

    if(err < 0) 
        return  err;

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_af_version: data0: 0x%02x, data1: 0x%02x\n", version_info[0], version_info[1]);    

    ce147_status->af_info.low = version_info[1];
    ce147_status->af_info.high = version_info[0];

    return 0;
}

static int ce147_get_gamma_version(void)
{
    unsigned char gamma_info[2] = {0x00, 0x00};
    unsigned int info_len = 2;    
    int err = -1;

    unsigned char rg_low_buf[2] = {0x0C, 0x00};
    unsigned char rg_high_buf[2] = {0x0D, 0x00};
    unsigned char bg_low_buf[2] = {0x0E, 0x00};
    unsigned char bg_high_buf[2] = {0x0F, 0x00};    
    unsigned int buf_len = 2;


    err = ce147_i2c_read_multi( DATA_VERSION_GAMMA, rg_low_buf, buf_len, gamma_info, info_len);
    if(err < 0)
        return -EIO;    

    ce147_status->gamma.rg_low = gamma_info[1];
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);        

    err = ce147_i2c_read_multi( DATA_VERSION_GAMMA, rg_high_buf, buf_len, gamma_info, info_len);
    if(err < 0)
        return -EIO;    

    ce147_status->gamma.rg_high = gamma_info[1];
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);    

        err = ce147_i2c_read_multi( DATA_VERSION_GAMMA, bg_low_buf, buf_len, gamma_info, info_len);
    if(err < 0)
        return -EIO;    
    
    ce147_status->gamma.bg_low = gamma_info[1];
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);    

        err = ce147_i2c_read_multi( DATA_VERSION_GAMMA, bg_high_buf, buf_len, gamma_info, info_len);
    if(err < 0)
        return -EIO;        

    ce147_status->gamma.bg_high= gamma_info[1];
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_gamma_version1: data1: 0x%02x, data1: 0x%02x\n", gamma_info[0], gamma_info[1]);    

    return 0;
}

#ifndef MDNIE_TUNING
static int ce147_update_fw(void)
{
    unsigned char *mbuf = NULL;
    unsigned char *fw_buf[4];
    int fw_size[4];
    int index = 0;
    int i = 0;
    int err;

    const unsigned int packet_size = 129; //Data 128 + Checksum 1
    unsigned int packet_num, k, j = 0, l = 0;
    unsigned char res = 0x00;
    unsigned char data[129];
    unsigned char data2[129];

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: ce147_fw: buf = 0x%p, len = %d\n", __func__, (void*)ce147_status->fw_info.addr, ce147_status->fw_info.size);

    mbuf = vmalloc(ce147_status->fw_info.size);

    if(NULL == mbuf){
        return -ENOMEM;
    }

    if (copy_from_user(mbuf, (void*)ce147_status->fw_info.addr, ce147_status->fw_info.size)){
        err = -EFAULT;
        goto out;
    }

    /** The firmware buffer is now copied to mbuf, so the firmware code is now in mbuf.
      *  We can use mbuf with i2c_tranfer call  */
    for(i = 0; i < 4; i++){
        if(index > ce147_status->fw_info.size - 4){
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s:Error size parameter\n", __func__);
            break;
        }
        memcpy(fw_size+i, mbuf + index, 4);
        index += 4;
        fw_buf[i] = mbuf + index;
        index += ((fw_size[i]-1) & (~0x3)) + 4;
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: [%d] fw_size = %d, fw_buf = 0x%p\n", __func__, i, fw_size[i], fw_buf[i]);
    }

    /* reset/on sensor */
    err = ce147_set_power(0);
    if(err < 0) {    
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(on)\n", __func__);
        err = -EIO;
        goto out;
    }
    
    err = ce147_set_power(1);
    if(err < 0) {    
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(on)\n", __func__);
        err = -EIO;
        goto out;
    }
    
    msleep(100);

    // [1] set fw updater info
    err = ce147_i2c_write_multi(CMD_FW_INFO, fw_buf[0], 4);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for 0xf2, fw_size[0]: %d, fw_buf[0]: 0x%02x\n", __func__, fw_size[0], (unsigned int)(fw_buf[0]));
        err = -EIO;
        goto out;
    }
    msleep(100);

    printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_write for 0xf2, fw_size[0]: %d, fw_buf[0]: 0x%02x\n", fw_size[0], fw_buf[0]);

    packet_num = *(fw_buf[0]) + (*(fw_buf[0]+1)<<8);

    // [2] update firmware
    for (k = 0; k < packet_num; k++){    
        memcpy(&data[0], fw_buf[1]+j, packet_size);
    err = ce147_i2c_read_multi(CMD_FWU_UPDATE, data, packet_size, &res, 1);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xf3, data: 0x%02x\n", __func__, data[0]);
        err = -EIO;
        goto out;
    }
    msleep(10);
    j = j + 129;
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xf3, data: 0x%02x, count: %d\n", data[0], k);        
    }

    k = 0;
    // [3] get fw status
    do {
        msleep(100);
                    
        err = ce147_i2c_read_multi(CMD_FW_STATUS, NULL, 0, &res, 1);
            if(err < 0) {
                    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xf5", __func__);                
            err = -EIO;
            goto out;
        } 
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xf5, data: 0x%02x\n", res);    

        k++;
        if(k == 500) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: check status TIMEOUT !!!", __func__);
            break;
        }
    } while(res != 0x05);

    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: FWU update is success.", __func__);
    msleep(500);

    fw_size[2] = 4;

    // [4] set fw updater info
    err = ce147_i2c_write_multi(CMD_FW_INFO, fw_buf[2], fw_size[2]);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for 0xf2, fw_size[2]: %d, fw_buf[2]: 0x%02x\n", __func__, fw_size[2], (unsigned int)(fw_buf[2]));
        err = -EIO;
        goto out;
    }
    msleep(100);

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_write for 0xf2, fw_size[2]: %d, fw_buf[2]: 0x%02x\n", fw_size[2], fw_buf[2]);    

    packet_num = *(fw_buf[2]) + (*(fw_buf[2]+1)<<8);

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: packet_num: %d\n", packet_num);

    j = 0;
    
    // [5] update firmware
    for (l = 0; l < packet_num; l++){    
        memcpy(&data2[0], fw_buf[3]+j, packet_size);
        err = ce147_i2c_write_multi(CMD_FW_UPDATE, data2, packet_size);
        if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xf4, data:2 0x%02x\n", __func__, data2[0]);    
            err = -EIO;
            goto out;
        }

        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_write for 0xf4, data2: 0x%02x, count: %d\n", data2[0], l);
        
        msleep(10);
        j = j + 129;
    }
    
    l = 0;
    // [6] get fw status
    do {
        
        msleep(100);
                    
        err = ce147_i2c_read_multi(CMD_FW_STATUS, NULL, 0, &res, 1);
            if(err < 0) {
                    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xf5", __func__);            
            err = -EIO;
            goto out;
            }     
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xf5, data: 0x%02x\n", res);        

        l++;    
        if(l == 500) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: check status TIMEOUT !!!", __func__);
            break;
        }
    } while(res != 0x06);

    vfree(mbuf);

    /*
    err = ce147_set_power(0);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(off)\n", __func__);
        return -EIO;
    }
    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: ce147_set_power(off)\n", __func__);
    */
        
    return 0;
out:
    vfree(mbuf);

    return err;
}

#else
unsigned short *test[1];
EXPORT_SYMBOL(test);
extern void mDNIe_txtbuf_to_parsing(void);
extern void mDNIe_txtbuf_to_parsing_for_lightsensor(void);
extern void mDNIe_txtbuf_to_parsing_for_backlight(void);

static int ce147_update_fw(void)
{
    unsigned char *mbuf = NULL;
    unsigned char *fw_buf[4];
    int fw_size[4];
    int index = 0;
    int i = 0;
    int err;

    const unsigned int packet_size = 129; //Data 128 + Checksum 1
    unsigned int packet_num, k, j, l = 0;
    unsigned char res = 0x00;
    unsigned char data[129];
    unsigned char data2[129];

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: ce147_fw: buf = 0x%p, len = %d\n", __func__, (void*)ce147_status->fw_info.addr, ce147_status->fw_info.size);

    mbuf = vmalloc(ce147_status->fw_info.size);

    if(NULL == mbuf){
        return -ENOMEM;
    }

    if (copy_from_user(mbuf, (void*)ce147_status->fw_info.addr, ce147_status->fw_info.size)){
        vfree(mbuf);
        return -EFAULT;
    }

    /** The firmware buffer is now copied to mbuf, so the firmware code is now in mbuf.
      *  We can use mbuf with i2c_tranfer call  */
    for(i = 0; i < 4; i++){
        if(index > ce147_status->fw_info.size - 4){
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s:Error size parameter\n", __func__);
            break;
        }
        memcpy(fw_size+i, mbuf + index, 4);
        index += 4;
        fw_buf[i] = mbuf + index;
        index += ((fw_size[i]-1) & (~0x3)) + 4;
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: [%d] fw_size = %d, fw_buf = 0x%p\n", __func__, i, fw_size[i], fw_buf[i]);
    }
    
    test[0] = fw_buf[0];
    
    for(j = 0; j < fw_size[0]; j++){
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: , fw_size[0]: %d, test[0]: 0x%x\n", fw_size[0], test[0][j]);
            test[0][j] = ((test[0][j]&0xff00)>>8)|((test[0][j]&0x00ff)<<8);
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: , test1[0]: 0x%x\n", test[0][j]);                
        }

    /*for mdnie tuning*/
    
    mDNIe_txtbuf_to_parsing();
    //mDNIe_txtbuf_to_parsing_for_lightsensor();
    //mDNIe_txtbuf_to_parsing_for_backlight();

    return 0;
}
#endif


static int ce147_dump_fw(void)
{
    unsigned char *mbuf = NULL;
    unsigned char *fw_buf[4];
    int fw_size[4];
    int index = 0;
    int i = 0;
    int err;

    const unsigned int packet_size = 129; //Data 128 + Checksum 1
    unsigned int packet_num, k, j = 0, l = 0;
    unsigned char res = 0x00;
    unsigned char data[129];
    unsigned char data2[130];
    unsigned char addr[4] = {0x03, 0x00, 0x00, 0x01};    
    unsigned int addr_len = 4;    
    unsigned char dump[1] = {0x00};    

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: ce147_fw: buf = 0x%p, len = %d\n", __func__, (void*)ce147_status->fw_info.addr, ce147_status->fw_info.size);

    mbuf = vmalloc(ce147_status->fw_info.size);

    if(NULL == mbuf){
        return -ENOMEM;
    }

    if (copy_from_user(mbuf, (void*)ce147_status->fw_info.addr, ce147_status->fw_info.size)){
        err = -EFAULT;
        goto out;
    }

    /** The firmware buffer is now copied to mbuf, so the firmware code is now in mbuf.
      *  We can use mbuf with i2c_tranfer call  */
    for(i = 0; i < 4; i++){
        if(index > ce147_status->fw_info.size - 4){
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s:Error size parameter\n", __func__);
            break;
        }
        memcpy(fw_size+i, mbuf + index, 4);
        index += 4;
        fw_buf[i] = mbuf + index;
        index += ((fw_size[i]-1) & (~0x3)) + 4;
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: [%d] fw_size = %d, fw_buf = 0x%p\n", __func__, i, fw_size[i], fw_buf[i]);
    }

    err = ce147_set_power(1);
    if(err < 0) {    
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(on)\n", __func__);
        err = -EIO;
        goto out;
    }

    msleep(100);

    // [1] set fw updater info
    err = ce147_i2c_write_multi(CMD_FW_INFO, fw_buf[0], 4);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for 0xf2, fw_size[0]: %d, fw_buf[0]: 0x%02x\n", __func__, fw_size[0], (unsigned int)(fw_buf[0]));
        err = -EIO;
        goto out;
    }
    msleep(100);

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_write for 0xf2, fw_size[0]: %d, fw_buf[0]: 0x%02x\n", fw_size[0], fw_buf[0]);

    packet_num = *(fw_buf[0]) + (*(fw_buf[0]+1)<<8);

    // [2] update firmware
    for (k = 0; k < packet_num; k++){    
        memcpy(&data[0], fw_buf[1]+j, packet_size);
        err = ce147_i2c_read_multi(CMD_FWU_UPDATE, data, packet_size, &res, 1);
        if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xf3, data: 0x%02x\n", __func__, data[0]);
            err = -EIO;
            goto out;
        }
        msleep(10);
        j = j + 129;
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xf3, data: 0x%02x, count: %d\n", data[0], k);        
    }

    k = 0;
    // [3] get fw status
    do {
        msleep(100);
                    
        err = ce147_i2c_read_multi(CMD_FW_STATUS, NULL, 0, &res, 1);
            if(err < 0) {
                    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xf5", __func__);                
            err = -EIO;
            goto out;
        } 
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xf5, data: 0x%02x\n", res);    

        k++;
        if(k == 500) break;
    } while(res != 0x05);

    msleep(500);

    // [4] change from dump mode
    err = ce147_i2c_write_multi(CMD_FW_DUMP, dump, 1);
    if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for 0xfb, 0x00", __func__);
        err = -EIO;
        goto out;
    }
    msleep(100);

    dump[0] = 0x02;

    // [5] check fw mode is in dump mode
    err = ce147_i2c_read_multi(CMD_FW_DUMP, dump, 1, &res, 1);
    if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xfb", __func__);             
        err = -EIO;
        goto out;
    } 

    if(res != 1){
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: res is %x", __func__, res);             
        err = -EIO;
        goto out;
    }    

    msleep(100);

    // [6] set dump start address
    err = ce147_i2c_write_multi(CMD_FW_DUMP, addr, addr_len);
    if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for 0xfb, 0x03", __func__);
        err = -EIO;
        goto out;
    }
    msleep(100);

    j = 0;

    packet_num = *(fw_buf[2]) + (*(fw_buf[2]+1)<<8);
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xfb, packet_num: %d\n", packet_num);    

    dump[0] = 0x04;

    // [7] dump firmware data
    for (l = 0; l < packet_num; l++){    
        err = ce147_i2c_read_multi(CMD_FW_DUMP, dump, 1, data2, packet_size+1);
        if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: fail: i2c_read for 0xfb,0x04\n", __func__);
            err = -EIO;
            goto out;
        }
        memcpy(fw_buf[3]+j, &data2[0], packet_size - 1);
        
        msleep(10);
        j = j + 129;
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_update_fw: i2c_read for 0xfb, count: %d\n", l);        
    }

    ce147_status->fw_dump_size = packet_num * packet_size;

    if (copy_to_user((void *)(ce147_status->fw_info.addr), fw_buf[3], ce147_status->fw_dump_size))    {
        err = -EIO;
        goto out;
    }
    
    vfree(mbuf);
    
    err = ce147_set_power(0);
    if(err < 0) {    
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(off)\n", __func__);
            return -EIO;
    }

    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: ce147_set_power(off)\n", __func__);
    
    return 0;
out:
    vfree(mbuf);

    return err;
}

static int ce147_check_dataline(void)
{
    int err;
    unsigned char ce147_buf_check_dataline[2] = { 0x01, 0x01 };
    unsigned int ce147_len_check_dataline = 2;

    CAMDRV_DEBUG("%s start\n", __func__);
    err = ce147_i2c_write_multi(CMD_CHECK_DATALINE, ce147_buf_check_dataline, ce147_len_check_dataline);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for check_dataline\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_check_dataline_stop(void)
{
    int err;
    
    unsigned char ce147_buf_check_dataline[2] = { 0x00, 0x00 };
    unsigned int ce147_len_check_dataline = 2;

    CAMDRV_DEBUG("%s\n", __func__);
    err = ce147_i2c_write_multi(CMD_CHECK_DATALINE, ce147_buf_check_dataline, ce147_len_check_dataline);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for check_dataline stop\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_preview_size(void)
{
    int err;    
    int index = ce147_status->preview_size;
    unsigned char ce147_regbuf_preview_size[2] = { 0x04, 0x01 }; /* Default VGA resolution */
    unsigned int ce147_reglen_preview_size = 2;
    unsigned char ce147_regbuf_hd_preview[1] = { 0x00 };
    unsigned int ce147_reglen_hd_preview = 1;

    switch(index){
        case CE147_PREVIEW_QCIF:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_QCIF\n", __func__);
            ce147_regbuf_preview_size[0] = 0x1E;
            break;

        case CE147_PREVIEW_QVGA:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_QVGA\n", __func__);
            ce147_regbuf_preview_size[0] = 0x02;
            break;

        case CE147_PREVIEW_592x480:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_592x480\n", __func__);
            ce147_regbuf_preview_size[0] = 0x24;
            break;

        case CE147_PREVIEW_VGA:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_VGA\n", __func__);
            ce147_regbuf_preview_size[0] = 0x04;
            break;

        case CE147_PREVIEW_WVGA:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_WVGA\n", __func__);
            ce147_regbuf_preview_size[0] = 0x13;
            break;

        case CE147_PREVIEW_D1:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_D1\n", __func__);
            ce147_regbuf_preview_size[0] = 0x20;
            break;
            
        case CE147_PREVIEW_720P:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_720P\n", __func__);
            ce147_regbuf_preview_size[0] = 0x16;
            ce147_regbuf_preview_size[1] = 0x02;
            break;
            
        case CE147_PREVIEW_VERTICAL_QCIF:
            CAMDRV_DEBUG("%s: CE147_PREVIEW_VERTICAL_QCIF\n", __func__);
            ce147_regbuf_preview_size[0] = 0x26;
            break;
            
        default:
            /* When running in image capture mode, the call comes here.
              * Set the default video resolution - CE147_PREVIEW_VGA
              */ 
            CAMDRV_DEBUG("Setting preview resoution as VGA for image capture mode\n");
            break;
    }
    
    if(index == CE147_PREVIEW_720P) {
        ce147_regbuf_hd_preview[0] = 0x01;
        ce147_status->hd_preview_on = 1;

        err = ce147_i2c_write_multi( CMD_HD_PREVIEW, ce147_regbuf_hd_preview, ce147_reglen_hd_preview);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: CMD_HD_PREVIEW\n", __func__);
            return -EIO; 
        }
        ce147_status->hd_gamma = GAMMA_ON;
        ce147_status->hd_slow_ae = SLOW_AE_ON;
    } else {
        ce147_regbuf_hd_preview[0] = 0x00;
        ce147_status->hd_preview_on = 0;

        err = ce147_i2c_write_multi( CMD_HD_PREVIEW, ce147_regbuf_hd_preview, ce147_reglen_hd_preview);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: CMD_HD_PREVIEW\n", __func__);
            return -EIO; 
        }
        ce147_status->hd_gamma = GAMMA_OFF;
        ce147_status->hd_slow_ae = SLOW_AE_OFF;
    }

    mdelay(5);

    err = ce147_i2c_write_multi( CMD_PREVIEW_SIZE, ce147_regbuf_preview_size, ce147_reglen_preview_size);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: CMD_PREVIEW_SIZE\n", __func__);
        return -EIO; 
    }
    return err;
}

static int ce147_set_frame_rate(void)
{
    int err;
    unsigned char ce147_regbuf_fps[2] = { 0x1E, 0x00 };
    unsigned int ce147_reglen_fps = 2;
    
    switch(ce147_status->fps)
    {
        case FRAME_RATE_7:
            CAMDRV_DEBUG("%s: FRAME_RATE_7\n", __func__);
            ce147_regbuf_fps[0] = 0x07;
            break;

        case FRAME_RATE_15:
            CAMDRV_DEBUG("%s: FRAME_RATE_15\n", __func__);
            ce147_regbuf_fps[0] = 0x0F;
            break;

        case FRAME_RATE_60:
            CAMDRV_DEBUG("%s: FRAME_RATE_60\n", __func__);
            ce147_regbuf_fps[0] = 0x3C;
            break;

        case FRAME_RATE_120:
            CAMDRV_DEBUG("%s: FRAME_RATE_120\n", __func__);
            ce147_regbuf_fps[0] = 0x78;
            break;
            
        case FRAME_RATE_30:
        default:
            CAMDRV_DEBUG("%s: FRAME_RATE_30\n", __func__);
            ce147_regbuf_fps[0] = 0x1E;

        break;
    }

    err = ce147_i2c_write_multi(CMD_FPS, ce147_regbuf_fps, ce147_reglen_fps);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_frame_rate\n", __func__);
        return -EIO;
    }
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_frame_rate\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_anti_banding(void)
{
    int err;
    unsigned char ce147_regbuf_anti_banding[1] = { 0x02 };
    unsigned int ce147_reglen_anti_banding = 1;
    
    switch(ce147_status->anti_banding)
    {
        case ANTI_BANDING_OFF:
            CAMDRV_DEBUG("%s: ANTI_BANDING_OFF\n", __func__);
            ce147_regbuf_anti_banding[0] = 0x00;
        break;

        case ANTI_BANDING_AUTO:
            CAMDRV_DEBUG("%s: ANTI_BANDING_AUTO\n", __func__);
            ce147_regbuf_anti_banding[0] = 0x01;
        break;

        case ANTI_BANDING_50HZ:
            CAMDRV_DEBUG("%s: ANTI_BANDING_50HZ\n", __func__);
            ce147_regbuf_anti_banding[0] = 0x02;
        break;

        case ANTI_BANDING_60HZ:
        default:
            CAMDRV_DEBUG("%s: ANTI_BANDING_60HZ\n", __func__);
            ce147_regbuf_anti_banding[0] = 0x03;
        break;
    }

    ce147_regbuf_anti_banding[0] = 0x02; // temporary patch. hc.hyun 2011-02-09
    
    err = ce147_i2c_write_multi( CMD_SET_ANTI_BANDING, ce147_regbuf_anti_banding, ce147_reglen_anti_banding);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for anti_banding\n", __func__);
        return -EIO;
    }

    

    return 0;
}

static int ce147_set_preview_stop(void)
{
    int ce147_reglen_preview = 1;
    unsigned char ce147_regbuf_preview_stop[1] = { 0x00 };
    int err;

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :[5B] ce147_set_preview_stop: (%d)\n", ce147_status->runmode);

    if(CE147_RUNMODE_RUNNING == ce147_status->runmode){
        err = ce147_i2c_write_multi(CMD_PREVIEW, ce147_regbuf_preview_stop, ce147_reglen_preview);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for preview_stop\n", __func__);
            return -EIO;
        }

        err = ce147_waitfordone_timeout(CMD_PREVIEW_STATUS, 0x00, 3000, POLL_TIME_MS);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Wait for preview_stop failed\n", __func__ );
            return err;
        }
        CAMDRV_DEBUG("%s: preview_stop - wait time %d ms\n", __func__, err);    

        ce147_status->runmode = CE147_RUNMODE_READY;
    }

    return 0;
}

static int ce147_set_dzoom(int val)
{
    int err;
    int count;
    unsigned char ce147_buf_get_dzoom_status[2] = { 0x00, 0x00 };
    unsigned int ce147_len_get_dzoom_status = 2;

    CAMDRV_DEBUG("%s: val\n", __func__);
    
    if(CE147_RUNMODE_RUNNING == ce147_status->runmode) {
        err = ce147_i2c_write_multi( CMD_SET_DZOOM, &ce147_buf_set_dzoom[val], 1);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_dzoom\n", __func__);
            return -EIO;
        }
        //To Do: This code needs to use ce147_waitfordone_timeout() API
        for(count = 0; count < 300; count++) {
            err = ce147_i2c_read_multi( CMD_GET_DZOOM_LEVEL, NULL, 0, ce147_buf_get_dzoom_status, ce147_len_get_dzoom_status);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for set_dzoom\n", __func__);
                return -EIO;
            }
            if(ce147_buf_get_dzoom_status[1] == 0x00) break;
        }
    }
    DZoom_State = val;

    return 0;
}

static int ce147_set_preview_start(void)
{
    int err;
    int ce147_reglen_preview = 1;
    unsigned char ce147_regbuf_preview_start[1] = { 0x01 };

    int count;
    unsigned char ce147_buf_get_dzoom_status[2] = { 0x00, 0x00 };
    unsigned int ce147_len_get_dzoom_status = 2;    

    CAMDRV_DEBUG("%s\n", __func__);
    //if( !ce147_status->pix.width || !ce147_status->pix.height || !ce147_status->fps){
    //    return -EINVAL;
    //}

    //This is for 15 testmode
    if(ce147_status->check_dataline)
    {
        err = ce147_check_dataline();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Could not check data line.\n", __func__);
            return -EIO;
        }
    }
    //Normal preview sequence
    else 
    {
        /* Stop it if it is already running */
        err = ce147_set_preview_stop();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Could not stop the running preview.\n", __func__);
            return -EIO;
        }

        err = ce147_set_preview_size();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Could not set preview size\n", __func__);
            return -EIO;
        }

        if(DZoom_State != 0){
            err = ce147_i2c_write_multi( CMD_SET_DZOOM, &ce147_buf_set_dzoom[DZoom_State], 1);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_dzoom in preview_start\n", __func__);
                return -EIO;
            }

            for(count = 0; count < 300; count++)
            {
                err = ce147_i2c_read_multi( CMD_GET_DZOOM_LEVEL, NULL, 0, ce147_buf_get_dzoom_status, ce147_len_get_dzoom_status);
                if(err < 0) {
                    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for set_dzoom in preview_start\n", __func__);
                    return -EIO;
                }
                if(ce147_buf_get_dzoom_status[1] == 0x00) break;
            }
        }

        err = ce147_set_anti_banding();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Could not set anti banding\n", __func__);
            return -EIO;
        }

        err = ce147_set_frame_rate();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Could not set fps\n", __func__);
            return -EIO;
            }

        if(ce147_status->runmode != CE147_RUNMODE_READY)
        {
            /* iso */
            err = ce147_set_iso(ce147_status->iso);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_iso, err %d\n", __func__, err);
                return -EIO;
            }

            /* metering */
            err = ce147_set_metering(ce147_status->metering);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_metering, err %d\n", __func__, err);
                //return -EIO;
            }

            /* ev */
            err = ce147_set_ev(ce147_status->ev);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_ev, err %d\n", __func__, err);
                //return -EIO;
            }
            
            /* effect */
            err = ce147_set_effect(ce147_status->effect);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_effect, err %d\n", __func__, err);
                //return -EIO;
            }

            /* wb*/
            err = ce147_set_white_balance(ce147_status->wb);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_white_balance, err %d\n", __func__, err);
                //return -EIO;
            }
                        
        }

        /* slow ae */
        err = ce147_set_slow_ae(ce147_status->hd_slow_ae);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_slow_ae, err %d\n", __func__, err);
            //return -EIO;
        }

        /* RGB gamma */
        err = ce147_set_gamma(ce147_status->hd_gamma);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_gamma, err %d\n", __func__, err);
            //return -EIO;
        }

        /* batch reflection */
        err = ce147_get_batch_reflection_status();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_frame_rate\n", __func__);
            //return -EIO; 
        }

        /* Release AWB unLock */ 
        err = ce147_set_ae_awb(AE_UNLOCK_AWB_UNLOCK);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_ae_awb, err %d\n", __func__, err);
            //return -EIO;
        }
                    
        /* Start preview */
        err = ce147_i2c_write_multi(CMD_PREVIEW, ce147_regbuf_preview_start, ce147_reglen_preview);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for preview_start\n", __func__);
            //return -EIO;
        }

        err = ce147_waitfordone_timeout(CMD_PREVIEW_STATUS, 0x08, 3000, POLL_TIME_MS);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Wait for preview_start failed\n", __func__ );
            //return err;
        }
        CAMDRV_DEBUG("%s: wait time %d ms\n", __func__, err);    
    }

    ce147_status->runmode = CE147_RUNMODE_RUNNING;

    return 0;
}

static int ce147_set_capture_size(void)
{
    int err;

    int index = ce147_status->framesize_index;
    unsigned char ce147_regbuf_capture_size[4] = { 0x0B, 0x00, 0x01, 0x00}; /* Default 5 MP */
    unsigned int ce147_reglen_capture_size = 4;

    switch(index){
    case CE147_CAPTURE_VGA: /* 640x480 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_VGA\n", __func__);
        ce147_regbuf_capture_size[0] = 0x04;
        break;
    case CE147_CAPTURE_WVGA: /* 800x480 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_WVGA\n", __func__);
        ce147_regbuf_capture_size[0] = 0x13;
        break;
    case CE147_CAPTURE_W1MP: /* 1600x960 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_W1MP\n", __func__);
        ce147_regbuf_capture_size[0] = 0x0E;
        break;        
    case CE147_CAPTURE_2MP: /* 1600x1200 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_2MP\n", __func__);
        ce147_regbuf_capture_size[0] = 0x08;
        break;
    case CE147_CAPTURE_W2MP: /* 2048x1232 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_W2MP\n", __func__);
        ce147_regbuf_capture_size[0] = 0x0F;
        break;
    case CE147_CAPTURE_3MP: /* 2048x1536 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_3MP\n", __func__);
        ce147_regbuf_capture_size[0] = 0x09;
        break;
    case CE147_CAPTURE_W4MP: /* 2560x1536 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_W4MP\n", __func__);
        ce147_regbuf_capture_size[0] = 0x15;
        break;
    case CE147_CAPTURE_5MP: /* 2560x1920 */
        CAMDRV_DEBUG("%s: CE147_CAPTURE_5MP\n", __func__);
        ce147_regbuf_capture_size[0] = 0x0B;
        break;
    default:
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: invalid size.\n", __func__);
        /* The framesize index was not set properly. 
          * Check s_fmt call - it must be for video mode. */
        return -EINVAL;
    }

    /* Set capture image size */
    err = ce147_i2c_write_multi( CMD_CAPTURE_SIZE, ce147_regbuf_capture_size, ce147_reglen_capture_size);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for capture_resolution\n", __func__);
        return -EIO; 
    }

    //This is for postview
    if(ce147_regbuf_capture_size[0] < 0x0C) {
        ce147_status->postview_size = CE147_PREVIEW_VGA; 
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :[5B] ce147_set_capture_size: preview_size is VGA (%d)\n", ce147_status->preview_size);
    } else {
        ce147_status->postview_size = CE147_PREVIEW_WVGA; 
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :[5B] ce147_set_capture_size: preview_size is WVGA (%d)\n", ce147_status->preview_size);        
    }

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_set_capture_size: 0x%02x\n", index);
    return 0;    
}

static int ce147_set_ae_awb(int val)
{
    int err;
    unsigned char ce147_buf_set_ae_awb[1] = { 0x00 };

    switch(val)
    {
        case AE_LOCK_AWB_UNLOCK: 
            CAMDRV_DEBUG("%s: AE_LOCK_AWB_UNLOCK\n", __func__);
            ce147_buf_set_ae_awb[0] = 0x01;
            break;

        case AE_UNLOCK_AWB_LOCK: 
            CAMDRV_DEBUG("%s: AE_UNLOCK_AWB_LOCK\n", __func__);
            ce147_buf_set_ae_awb[0] = 0x10;
            break;
            
        case AE_LOCK_AWB_LOCK: 
            CAMDRV_DEBUG("%s: AE_LOCK_AWB_LOCK\n", __func__);
            ce147_buf_set_ae_awb[0] = 0x11;
            break;

        case AE_UNLOCK_AWB_UNLOCK:     
        default:
            CAMDRV_DEBUG("%s: AE_UNLOCK_AWB_UNLOCK\n", __func__);
            ce147_buf_set_ae_awb[0] = 0x00;            
            break;
    }
    
    err = ce147_i2c_write_multi( CMD_AE_WB_LOCK, ce147_buf_set_ae_awb, 1);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_effect\n", __func__);
        return -EIO;
    }
    ce147_status->aeawb = val;
    return 0;
}

static int ce147_set_capture_cmd(void)
{
    int err;
    unsigned char ce147_regbuf_buffering_capture[1] = { 0x00 };
    unsigned int ce147_reglen_buffering_capture = 1;

    CAMDRV_DEBUG("%s \n", __func__);
    err = ce147_i2c_write_multi( CMD_BUFFERING_CAPTURE, ce147_regbuf_buffering_capture, ce147_reglen_buffering_capture);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for buffering_capture\n", __func__);
        return -EIO;
    }

    
    return 0;
}

static int ce147_set_exif_ctrl(int onoff)
{
    int err;
    unsigned char ce147_regbuf_exif_ctrl[2] = { 0x10, 0x00 };
    unsigned int ce147_reglen_exif_ctrl = 2;

    CAMDRV_DEBUG("%s onoff[%d], thumb_null[%d]\n", __func__,onoff, ce147_status->thumb_null);
    
    if ((onoff == 0) && (ce147_status->thumb_null ==0))
        ce147_regbuf_exif_ctrl[1] = 0x00;
    else if ((onoff == 1) && (ce147_status->thumb_null ==0))
        ce147_regbuf_exif_ctrl[1] = 0x01;
    else if ((onoff == 0) && (ce147_status->thumb_null ==0))
        ce147_regbuf_exif_ctrl[1] = 0x02;
    else if ((onoff == 0) && (ce147_status->thumb_null ==1))
        ce147_regbuf_exif_ctrl[1] = 0x03;
    else if((onoff == 1) && (ce147_status->thumb_null ==1))
        ce147_regbuf_exif_ctrl[1] = 0x04;

    //ce147_regbuf_exif_ctrl[1] = 0x01; //temporary patch 2011-03-03 hc.hyun

    err = ce147_i2c_write_multi(CMD_SET_EXIF_CTRL, ce147_regbuf_exif_ctrl, ce147_reglen_exif_ctrl);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for ce147_reglen_exif_ctrl\n", __func__);
        return -EIO;
    }
    
    return 0;
}

static int ce147_set_gps_info(int val, void* data)
{
    int err = -ENOIOCTLCMD;
    static int condition = -1;
    unsigned long temp = 0;
    char *temp2;
    struct gps_info_common * tempGPSType = NULL;
    
    ce147_status->exif_ctrl = 0;

    switch (val) {
    
        case EXT_CFG_SET_GPS_LATITUDE:
            tempGPSType = (struct gps_info_common *)data;
            ce147_status->gpsInfo.ce147_gps_buf[0] = tempGPSType ->direction;
            ce147_status->gpsInfo.ce147_gps_buf[1] = tempGPSType ->dgree;
            ce147_status->gpsInfo.ce147_gps_buf[2] = tempGPSType ->minute;
            ce147_status->gpsInfo.ce147_gps_buf[3] = tempGPSType ->second;
            
            if((tempGPSType ->direction == 0)&&(tempGPSType ->dgree == 0)&&(tempGPSType ->minute == 0)&&(tempGPSType ->second == 0))
                condition = 1;
            else 
                condition = 0;
            printk(KERN_ERR "[CAMDRV/CE147] gps_info_latiude NS: %d, dgree: %d, minute: %d, second: %d \n",ce147_status->gpsInfo.ce147_gps_buf[0], ce147_status->gpsInfo.ce147_gps_buf[1], ce147_status->gpsInfo.ce147_gps_buf[2], ce147_status->gpsInfo.ce147_gps_buf[3]);
            err = 0;
            break;
    
        case EXT_CFG_SET_GPS_LONGITUDE:
            tempGPSType = (struct gps_info_common *)data;
            ce147_status->gpsInfo.ce147_gps_buf[4] = tempGPSType ->direction;
            ce147_status->gpsInfo.ce147_gps_buf[5] = tempGPSType ->dgree;
            ce147_status->gpsInfo.ce147_gps_buf[6] = tempGPSType ->minute;
            ce147_status->gpsInfo.ce147_gps_buf[7] = tempGPSType ->second;
            
            if((tempGPSType ->direction == 0)&&(tempGPSType ->dgree == 0)&&(tempGPSType ->minute == 0)&&(tempGPSType ->second == 0))
                condition = 1;
            else 
                condition = 0;
            printk(KERN_ERR "[CAMDRV/CE147]gps_info_longitude EW: %d, dgree: %d, minute: %d, second: %d \n", ce147_status->gpsInfo.ce147_gps_buf[4], ce147_status->gpsInfo.ce147_gps_buf[5], ce147_status->gpsInfo.ce147_gps_buf[6], ce147_status->gpsInfo.ce147_gps_buf[7]);
            err = 0;
            break;
    
        case EXT_CFG_SET_GPS_ALTITUDE:
            tempGPSType = (struct gps_info_common *)data;
            ce147_status->gpsInfo.ce147_altitude_buf[0] = tempGPSType ->direction;
            ce147_status->gpsInfo.ce147_altitude_buf[1] = (tempGPSType ->dgree)&0x00ff; //lower byte
            ce147_status->gpsInfo.ce147_altitude_buf[2] = ((tempGPSType ->dgree)&0xff00)>>8; //upper byte
            ce147_status->gpsInfo.ce147_altitude_buf[3] = tempGPSType ->minute;//float
    
            printk(KERN_ERR "[CAMDRV/CE147]gps_info_altitude PLUS_MINUS: %d, dgree_lower: %d, degree_lower: %d, minute: %d \n", ce147_status->gpsInfo.ce147_altitude_buf[0], ce147_status->gpsInfo.ce147_altitude_buf[1], ce147_status->gpsInfo.ce147_altitude_buf[2], ce147_status->gpsInfo.ce147_altitude_buf[3]);
            err = 0;
            break;
    
        case EXT_CFG_SET_GPS_TIMESTAMP:
            temp = *((unsigned long *)data);
            ce147_status->gpsInfo.gps_timeStamp = temp;
            err = 0;
            break;
            
        case EXT_CFG_SET_EXIF_TIME_INFO:
            ce147_status->exifTimeInfo =(struct tm *)data;
            err = 0;
            break;
            
        case EXT_CFG_SET_GPS_PROCESSINGMETHOD:
            temp2 = ((char *)data);
            if(temp2)
                strcpy(ce147_status->gpsInfo.gps_processingmethod, temp2);
            else 
                memset(ce147_status->gpsInfo.gps_processingmethod,0,sizeof(ce147_status->gpsInfo.gps_processingmethod));
            err = 0;
            break;
    }

    if(condition)
        ce147_status->exif_ctrl = 1;
    
    if (err < 0)
        printk(KERN_ERR "[CAMDRV/CE147] %s: vidioc_s_ext_ctrl failed %d\n", __func__, err);

    return err;
}

static int ce147_set_capture_exif(void)
{
    int err;
    struct rtc_time gps_timestamp;//ykh

    unsigned char ce147_regbuf_exif[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned int ce147_reglen_exif = 7;

    unsigned char ce147_regbuf_timestamp[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned int ce147_reglen_timestamp = 7;

    unsigned char ce147_regbuf_rot[1] = { 0x01 };
    unsigned int ce147_reglen_rot = 1;

    unsigned char ce147_model_name[130] = {0x00,};
    unsigned int ce147_reglen_model = 130;    
    
    unsigned char ce147_gps_processing[130] = {0x00,};
    unsigned int ce147_reglen_gps_processing = 130;
    unsigned char ce147_str_model[9] = "GT-I9001\0";

    struct timeval curr_time;
    struct rtc_time time;

    unsigned char ce147_gps_buf[8] = {0x00,};//test
    unsigned char ce147_altitude_buf[4] = {0x00,};//test
    
    ce147_model_name[0] = 0x06;
    ce147_model_name[1] = 0x09;

    memcpy(ce147_model_name+2, ce147_str_model, sizeof(ce147_str_model));

    ce147_gps_processing[0] = 0x10;
    ce147_gps_processing[1] = 0x32;


    memcpy(ce147_gps_processing+2, ce147_status->gpsInfo.gps_processingmethod, sizeof(ce147_status->gpsInfo.gps_processingmethod));

#if 1
    do_gettimeofday(&curr_time);
    rtc_time_to_tm(curr_time.tv_sec, &time);

    time.tm_year += 1900;
    time.tm_mon += 1;

    ce147_regbuf_exif[0] = (time.tm_year & 0x00FF);
    ce147_regbuf_exif[1] = (time.tm_year & 0xFF00) >> 8;
    ce147_regbuf_exif[2] = time.tm_mon;
    ce147_regbuf_exif[3] = time.tm_mday;
    ce147_regbuf_exif[4] = time.tm_hour;
    ce147_regbuf_exif[5] = time.tm_min;
    ce147_regbuf_exif[6] = time.tm_sec;        
#else
    ce147_status->exifTimeInfo->tm_year += 1900;
    ce147_status->exifTimeInfo->tm_mon += 1;
    ce147_regbuf_exif[0] = (ce147_status->exifTimeInfo->tm_year & 0x00FF);
    ce147_regbuf_exif[1] = (ce147_status->exifTimeInfo->tm_year & 0xFF00) >> 8;
    ce147_regbuf_exif[2] = ce147_status->exifTimeInfo->tm_mon;
    ce147_regbuf_exif[3] = ce147_status->exifTimeInfo->tm_mday;
    ce147_regbuf_exif[4] = ce147_status->exifTimeInfo->tm_hour;
    ce147_regbuf_exif[5] = ce147_status->exifTimeInfo->tm_min;
    ce147_regbuf_exif[6] = ce147_status->exifTimeInfo->tm_sec;

    printk(KERN_DEBUG "Exif Time YEAR: %d, MONTH: %d, DAY: %d, HOUR: %d, MIN: %d, SEC: %d\n", \
    ce147_status->exifTimeInfo->tm_year, ce147_status->exifTimeInfo->tm_mon, ce147_status->exifTimeInfo->tm_mday, \
    ce147_status->exifTimeInfo->tm_hour, ce147_status->exifTimeInfo->tm_min, ce147_status->exifTimeInfo->tm_sec);
#endif

    rtc_time_to_tm(ce147_status->gpsInfo.gps_timeStamp, &gps_timestamp);
    gps_timestamp.tm_year += 2000;
    gps_timestamp.tm_mon += 1;

    printk(KERN_DEBUG "====!! Exif Time YEAR: %d, MONTH: %d, DAY: %d, HOUR: %d, MIN: %d, SEC: %d\n", \
    gps_timestamp.tm_year, gps_timestamp.tm_mon, gps_timestamp.tm_mday, \
    gps_timestamp.tm_hour, gps_timestamp.tm_min, gps_timestamp.tm_sec);

    ce147_regbuf_timestamp[0] = (gps_timestamp.tm_year & 0x00FF);
    ce147_regbuf_timestamp[1] = (gps_timestamp.tm_year & 0xFF00) >> 8;
    ce147_regbuf_timestamp[2] = gps_timestamp.tm_mon;
    ce147_regbuf_timestamp[3] = gps_timestamp.tm_mday;
    ce147_regbuf_timestamp[4] = gps_timestamp.tm_hour;
    ce147_regbuf_timestamp[5] = gps_timestamp.tm_min;
    ce147_regbuf_timestamp[6] = gps_timestamp.tm_sec;
    ce147_regbuf_rot[0] = ce147_status->exif_orientation_info;

    err = ce147_i2c_write_multi(CMD_INFO_MODEL, ce147_model_name, ce147_reglen_model);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for exif model name\n", __func__);
        return -EIO;
    }

    err = ce147_i2c_write_multi(CMD_INFO_EXIF, ce147_regbuf_exif, ce147_reglen_exif);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for exif\n", __func__);
        return -EIO;
    }

    err = ce147_i2c_write_multi(CMD_INFO_ROT, ce147_regbuf_rot, ce147_reglen_rot);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for exif\n", __func__);
        return -EIO;
    }

    //err = ce147_i2c_write_multi(CMD_INFO_LONGITUDE_LATITUDE, ce147_status->gpsInfo.ce147_gps_buf, sizeof(ce147_status->gpsInfo.ce147_gps_buf));
    err = ce147_i2c_write_multi(CMD_INFO_LONGITUDE_LATITUDE, ce147_gps_buf, sizeof(ce147_gps_buf));
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for gps longitude latitude\n", __func__);
        return -EIO;
    }
    
    //err = ce147_i2c_write_multi(CMD_INFO_ALTITUDE, ce147_status->gpsInfo.ce147_altitude_buf, sizeof(ce147_status->gpsInfo.ce147_altitude_buf));
    err = ce147_i2c_write_multi(CMD_INFO_ALTITUDE, ce147_altitude_buf, sizeof(ce147_altitude_buf));
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for gps altitude\n", __func__);
        return -EIO;
    }
        
    err = ce147_i2c_write_multi(CMD_GPS_TIMESTAMP, ce147_regbuf_timestamp, ce147_reglen_timestamp);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for gps timestamp\n", __func__);
        return -EIO;
    }
    
    err = ce147_i2c_write_multi(CMD_INFO_MODEL, ce147_gps_processing, ce147_reglen_gps_processing);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147]%s: failed: i2c_write for gps method\n", __func__);
        return -EIO;
    }
    
    return 0;
}

static int ce147_set_jpeg_quality(void)
{
    unsigned char ce147_regbuf_jpeg_comp_level[7] = { 0x00, 0xA4, 0x06, 0x78, 0x05, 0x05, 0x01 };
    unsigned int ce147_reglen_jpeg_comp_level = 7;
    unsigned int quality = ce147_status->jpeg.quality;
    unsigned int compressionRatio = 0;
    unsigned int minimumCompressionRatio = 0;
    int err;


	if(quality >= 91 && quality <= 100) { // 91 ~ 100
		compressionRatio = 17; // 17%
	}

	else if(quality >= 81 && quality <= 90) {	// 81 ~ 90
		compressionRatio = 16; // 16%
	}

	else if(quality >= 71 && quality <= 80) { // 71 ~ 80
		compressionRatio = 15; // 15%
	}

	else if(quality >= 61 && quality <= 70) { // 61 ~ 70
		compressionRatio = 14; // 14%
	}

	else if(quality >= 51 && quality <= 60) { // 51 ~ 60
		compressionRatio = 13; // 13%
	}
	
	else if(quality >= 41 && quality <= 50) { // 41 ~ 50
		compressionRatio = 12; // 12%
	}

	else if(quality >= 31 && quality <= 40) { // 31 ~ 40
		compressionRatio = 11; // 11%
	}

	else if(quality >= 21 && quality <= 30) { // 21 ~ 30
		compressionRatio = 10; // 10%
	}

	else if(quality >= 11 && quality <= 20) { // 11 ~ 20
		compressionRatio = 9; // 9%
	}
	
	else if(quality >= 1 && quality <= 10) { // 1 ~ 10
		compressionRatio = 8; // 8%
	}

	else {		
		printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Invalid Quality(%d)\n", __func__, quality);
		compressionRatio = 8; // 8%
	}

	minimumCompressionRatio = compressionRatio - 3; // ex) if compression ratio is 17%, minimum compression ratio is 14%
	ce147_regbuf_jpeg_comp_level[1] = (compressionRatio * 100) & 0xFF;
	ce147_regbuf_jpeg_comp_level[2] = ((compressionRatio * 100) & 0xFF00) >> 8;
	ce147_regbuf_jpeg_comp_level[3] = (minimumCompressionRatio * 100) & 0xFF;
	ce147_regbuf_jpeg_comp_level[4] = ((minimumCompressionRatio * 100) & 0xFF00) >> 8;
    
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: compression ratio low byte: 0x%x, high byte: 0x%x\n", __func__, ce147_regbuf_jpeg_comp_level[1], ce147_regbuf_jpeg_comp_level[2]);
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: minimum compression ratio low byte: 0x%x, high byte: 0x%x\n", __func__, ce147_regbuf_jpeg_comp_level[3], ce147_regbuf_jpeg_comp_level[4]);

    err = ce147_i2c_write_multi( CMD_JPEG_CONFIG, ce147_regbuf_jpeg_comp_level, ce147_reglen_jpeg_comp_level);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for jpeg_comp_level\n", __func__);
        return -EIO;
    }

    
    return 0;
}

static int ce147_set_jpeg_config(void)
{
    int err;
    int postview_size = ce147_status->postview_size;

    unsigned char ce147_regbuf_set_lump[2] = { 0x00, 0x04};
    unsigned int ce147_reglen_set_lump = 2;

    unsigned char ce147_regbuf_set_lump2[1] = {0x00};
    unsigned int ce147_reglen_set_lump2 = 1;

    CAMDRV_DEBUG("%s\n", __func__);
    err = ce147_set_jpeg_quality();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_jpeg_quality\n", __func__);
        return -EIO;
    }

    if(postview_size != CE147_PREVIEW_VGA) {
        //printk(KERN_ERR "[CAMDRV/CE147] CE147 :[5B] ce147_set_jpeg_config: preview_size is WVGA (%d)\n", preview_size);
        ce147_regbuf_set_lump[1] = 0x13;
    }

    //if(!ce147_status->thumb_null)
    err = ce147_i2c_write_multi( CMD_JPEG_BUFFERING, ce147_regbuf_set_lump, ce147_reglen_set_lump);
    //else if(ce147_status->thumb_null)
        //err = ce147_i2c_write_multi( CMD_JPEG_BUFFERING2, ce147_regbuf_set_lump2, ce147_reglen_set_lump2);

    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_lump\n", __func__);
        return -EIO;
    }
    
    return 0;
}

static int ce147_get_snapshot_data(void)
{
    int err;

    unsigned char cmd_buf_framesize[1] = { 0x00 };
    unsigned int cmd_len_framesize = 1;

    unsigned char cmd_buf_setdata[2] = { 0x02, 0x00 };
    unsigned int cmd_len_setdata = 2;

    unsigned char jpeg_status[3] = { 0x00, 0x00, 0x00 };
    unsigned char jpeg_status_len = 3;

    unsigned char jpeg_framesize[4] = { 0x00, 0x00, 0x00, 0x00 };
    unsigned int jpeg_framesize_len = 4;

    CAMDRV_DEBUG("%s\n", __func__);
    
    if(ce147_status->jpeg.enable){
        /* Get main JPEG size */
        cmd_buf_framesize[0] = 0x00;
        err = ce147_i2c_read_multi( CMD_JPEG_SIZE, cmd_buf_framesize, cmd_len_framesize, jpeg_framesize, jpeg_framesize_len);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for jpeg_framesize\n", __func__);
            return -EIO;
        }            
        ce147_status->jpeg.main_size = jpeg_framesize[1] | (jpeg_framesize[2] << 8) | (jpeg_framesize[3] << 16);

        CAMDRV_DEBUG("%s: JPEG main filesize = %d bytes\n", __func__, ce147_status->jpeg.main_size );

        /* Get Thumbnail size */
        if(!ce147_status->thumb_null)
        {
            cmd_buf_framesize[0] = 0x01;
            err = ce147_i2c_read_multi( CMD_JPEG_SIZE, cmd_buf_framesize, cmd_len_framesize, jpeg_framesize, jpeg_framesize_len);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for jpeg_framesize\n", __func__);
                return -EIO;
            }            
            ce147_status->jpeg.thumb_size = jpeg_framesize[1] | (jpeg_framesize[2] << 8) | (jpeg_framesize[3] << 16);
        }
        else
            ce147_status->jpeg.thumb_size = 0;

        CAMDRV_DEBUG("%s: JPEG thumb filesize = %d bytes\n", __func__, ce147_status->jpeg.thumb_size );

        ce147_status->jpeg.main_offset = 0;
        ce147_status->jpeg.thumb_offset = 0x271000;
        ce147_status->jpeg.postview_offset = 0x280A00;
    }

    if(ce147_status->jpeg.enable)
        cmd_buf_setdata[0] = 0x02;
    else
        cmd_buf_setdata[0] = 0x01;
    
    /* Set Data out */
    err = ce147_i2c_read_multi( CMD_SET_DATA, cmd_buf_setdata, cmd_len_setdata, jpeg_status, jpeg_status_len);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for set_data\n", __func__);
        return -EIO;
    }
    
    CAMDRV_DEBUG("%s:JPEG framesize (after set_data_out) = 0x%02x.%02x.%02x\n",__func__,\
            jpeg_status[2], jpeg_status[1],jpeg_status[0]);

    /* 0x66 */
    err = ce147_i2c_read_multi( CMD_DATA_OUT_REQ, NULL, 0, jpeg_status, jpeg_status_len);
    //err = ce147_i2c_write_multi(0x64,YcoOuputSetting[0], 10);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for set_data_request\n", __func__);
        return -EIO;
    }
    
    CAMDRV_DEBUG("%s:JPEG framesize (after set_data_request) = 0x%02x.%02x.%02x\n",__func__,\
            jpeg_status[2], jpeg_status[1],jpeg_status[0]);

    
    return 0;
}

static int ce147_set_capture_config(void)
{
    int err;
    
    CAMDRV_DEBUG("%s\n", __func__);
    
    /* 1.Set image size */
    err = ce147_set_capture_size();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for capture_resolution\n", __func__);
        return -EIO; 
    }
    
    /* Set DZoom */
    if(DZoom_State != 0){
        err = ce147_set_dzoom(DZoom_State);
        if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Could not set Zoom in Capture_start \n", __func__);
        return -EIO;
        }
    }
    
    /* Set AWB Lock */ 
    err = ce147_set_ae_awb(AE_LOCK_AWB_LOCK);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_ae_awb, err %d\n", __func__, err);
        return -EIO;
    }
    
    /* 2.Set Capture Command */
    err = ce147_set_capture_cmd();
    if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: set_capture_cmd failed\n", __func__);
                return err;
    }
    
    /*
     * Right after ce147_set_capture_config,
     * 3. Wait for capture to complete for ce147_set_capture_cmd() in ce147_set_capture_config()
     */
    err = ce147_waitfordone_timeout( 0x6C, 0x00, 3000, POLL_TIME_MS);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Wait for buffering_capture\n", __func__ );
        return err;
    }
    CAMDRV_DEBUG("%s: buffering_capture - wait time %d ms\n", __func__, err);

    err = ce147_set_exif_ctrl(ce147_status->exif_ctrl);
    if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: set_capture_cmd failed\n", __func__);
                return err;
    }

    if(ce147_status->jpeg.enable){
        /* 4. Set EXIF information */ 
        err = ce147_set_capture_exif();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for exif\n", __func__);
            return -EIO;
        }

        /* 6. Set JPEG Encoding parameters */
        err = ce147_set_jpeg_config();
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Setting JPEG encoding parameters\n", __func__);
            return err;
        }
        /* 7. Wait for encoding to complete */
        err = ce147_waitfordone_timeout( 0x6C, 0x00, 3000, POLL_TIME_MS);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Wait for jpeg_encoding\n", __func__ );
            return err;
        }
        CAMDRV_DEBUG("%s: jpeg_encoding - wait time %d ms\n", __func__, err);
    }

    return 0;
}

static int ce147_set_capture_start()
{
    int err;
    
    CAMDRV_DEBUG("%s\n", __func__);
    /* 8. Get JPEG Main Data */ 
    err = ce147_get_snapshot_data();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: get_snapshot_data\n", __func__);
        return err;
    }
    /* 9. Wait for done */
    err = ce147_waitfordone_timeout( 0x61, 0x00, 3000, POLL_TIME_MS);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: Wait for data_transfer\n", __func__ );
        return err;
    }
    CAMDRV_DEBUG("%s: data_transfer - wait time %d ms\n", __func__, err);
    
    return 0;
}

static int ce147_get_focus_mode(unsigned char cmd, unsigned char *value)
{
    int err;
    int count;
    unsigned char ce147_buf_get_af_status[1] = { 0x00 };

    CAMDRV_DEBUG("%s\n", __func__);
    // set focus mode: AF or MACRO
    err = ce147_i2c_write_multi(cmd, value, 1);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for get_focus_mode\n", __func__);
        return -EIO;
    }    
    //check whether af data is valid or not
    for(count = 0; count < 600; count++)
    {
        msleep(10);
        ce147_buf_get_af_status[0] = 0xFF;
        err = ce147_i2c_read_multi(CMD_CHECK_AUTO_FOCUS_SEARCH, NULL, 0, ce147_buf_get_af_status, 1);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for get_focus_mode\n", __func__);
            return -EIO;
        }
        if((ce147_buf_get_af_status[0]&0x01) == 0x00) break;
    }
    
    CAMDRV_DEBUG("%s: [%d] done\n", __func__,ce147_buf_get_af_status[0]&0x01);
    
    if((ce147_buf_get_af_status[0]&0x01) != 0x00)
        return -EBUSY;
    else
        return ce147_buf_get_af_status[0]&0x01;
}

static int ce147_set_af_softlanding(void)
{
    int err;
    int count;

    unsigned char ce147_buf_get_af_status[1] = { 0x00 };
    unsigned char ce147_buf_set_af_land[1] = { 0x08 };
    unsigned int ce147_len_set_af_land = 1;

    CAMDRV_DEBUG("%s\n", __func__);
    
    if(ce147_status->runmode > CE147_RUNMODE_IDLE)
    {
        // make lens landing mode
        err = ce147_i2c_write_multi( CMD_SET_AUTO_FOCUS_MODE, ce147_buf_set_af_land, ce147_len_set_af_land);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for auto_focus\n", __func__);
            return -EIO;
        }    
        //check whether af data is valid or not
        for(count = 0; count < 50; count++)
        {
            msleep(10);
            ce147_buf_get_af_status[0] = 0xFF;
            err = ce147_i2c_read_multi( CMD_CHECK_AUTO_FOCUS_SEARCH, NULL, 0, ce147_buf_get_af_status, 1);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for get_focus_mode\n", __func__);
                return -EIO;
            }
            if((ce147_buf_get_af_status[0]) == 0x08) break;
        }
    }
    
    return 0;
}

static int ce147_set_flash(int val)
{
    int err;
    unsigned char ce147_buf_set_flash[2] = { 0x03, 0x00 };
    unsigned int ce147_len_set_flash = 2;
    unsigned char ce147_buf_set_flash_manual[2] = { 0x00, 0x00 };
    unsigned int ce147_len_set_flash_manual = 2;

    switch(val)
    {
        case FLASH_MODE_OFF:
            ce147_buf_set_flash[1] = 0x00;
        break;

        case FLASH_MODE_AUTO:
            ce147_buf_set_flash[1] = 0x02;
        break;

        case FLASH_MODE_ON:
            ce147_buf_set_flash[1] = 0x01;
        break;

        case FLASH_MODE_TORCH:
            ce147_buf_set_flash_manual[1] = 0x01;
        break;

        default:
            ce147_buf_set_flash[1] = 0x00;

        break;
    }

    //need to modify flash off for torch mode
    if(val == FLASH_MODE_OFF)
    {
        err = ce147_i2c_write_multi( CMD_SET_FLASH_MANUAL, ce147_buf_set_flash_manual, ce147_len_set_flash_manual);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_flash\n", __func__);
            return -EIO;
        }    
    }

    err = ce147_i2c_write_multi( CMD_SET_FLASH, ce147_buf_set_flash, ce147_len_set_flash);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_flash\n", __func__);
        return -EIO;
    }
    
    CAMDRV_DEBUG("%s: done, flash: 0x%02x\n", __func__, ce147_buf_set_flash[1]);

    return 0;
}

static int ce147_set_effect(int val)
{
    int err;
    unsigned char ce147_buf_set_effect[2] = { 0x05, 0x00 };
    unsigned int ce147_len_set_effect = 2;

    ce147_status->effect = val;
        
    switch(val)
    {
        case IMAGE_EFFECT_NONE:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_NONE\n", __func__);
            ce147_buf_set_effect[1] = 0x00;
        break;

        case IMAGE_EFFECT_BNW:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_BNW\n", __func__);
            ce147_buf_set_effect[1] = 0x01;
        break;

        case IMAGE_EFFECT_SEPIA:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_SEPIA\n", __func__);
            ce147_buf_set_effect[1] = 0x03;
        break;

        case IMAGE_EFFECT_AQUA:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_AQUA\n", __func__);
            ce147_buf_set_effect[1] = 0x0D;
        break;

        case IMAGE_EFFECT_ANTIQUE:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_ANTIQUE\n", __func__);
            ce147_buf_set_effect[1] = 0x06;
        break;

        case IMAGE_EFFECT_NEGATIVE:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_NEGATIVE\n", __func__);
            ce147_buf_set_effect[1] = 0x05;
        break;

        case IMAGE_EFFECT_SHARPEN:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_SHARPEN\n", __func__);
            ce147_buf_set_effect[1] = 0x04;
        break;

        default:
            CAMDRV_DEBUG("%s: IMAGE_EFFECT_NONE\n", __func__);
            ce147_buf_set_effect[1] = 0x00;

        break;
    }

    err = ce147_i2c_write_multi(CMD_SET_EFFECT, ce147_buf_set_effect, ce147_len_set_effect);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_effect\n", __func__);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_effect\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_saturation(int val)
{
    int err;
    unsigned char ce147_buf_set_saturation[2] = { 0x06, 0x00 };
    unsigned int ce147_len_set_saturation = 2;
    
    switch(val)
    {
        case SATURATION_MINUS_2:
            CAMDRV_DEBUG("%s: SATURATION_MINUS_2\n", __func__);
            ce147_buf_set_saturation[1] = 0x01;
        break;

        case SATURATION_MINUS_1:
            CAMDRV_DEBUG("%s: SATURATION_MINUS_1\n", __func__);
            ce147_buf_set_saturation[1] = 0x02;
        break;

        case SATURATION_DEFAULT:
            CAMDRV_DEBUG("%s: SATURATION_DEFAULT\n", __func__);
            ce147_buf_set_saturation[1] = 0x03;
        break;

        case SATURATION_PLUS_1:
            CAMDRV_DEBUG("%s: SATURATION_PLUS_1\n", __func__);
            ce147_buf_set_saturation[1] = 0x04;
        break;

        case SATURATION_PLUS_2:
            CAMDRV_DEBUG("%s: SATURATION_PLUS_2\n", __func__);
            ce147_buf_set_saturation[1] = 0x05;
        break;

        default:
            CAMDRV_DEBUG("%s: SATURATION_DEFAULT\n", __func__);
            ce147_buf_set_saturation[1] = 0x03;

        break;
    }

    ce147_status->saturation = val;
    err = ce147_i2c_write_multi( CMD_SET_EFFECT, ce147_buf_set_saturation, ce147_len_set_saturation);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_saturation\n", __func__);
        return -EIO;
    }

#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_saturation\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_contrast(int val)
{
    int err;    
    unsigned char ce147_buf_set_contrast[2] = { 0x07, 0x00 };
    unsigned int ce147_len_set_contrast = 2;
    
    switch(val)
    {
        case CONTRAST_MINUS_2:
            CAMDRV_DEBUG("%s: CONTRAST_MINUS_2\n", __func__);
            ce147_buf_set_contrast[1] = 0x01;
            break;

        case CONTRAST_MINUS_1:
            CAMDRV_DEBUG("%s: CONTRAST_MINUS_1\n", __func__);
            ce147_buf_set_contrast[1] = 0x02;
            break;

        case CONTRAST_DEFAULT:
            CAMDRV_DEBUG("%s: CONTRAST_DEFAULT\n", __func__);
            ce147_buf_set_contrast[1] = 0x03;
            break;

        case CONTRAST_PLUS_1:
            CAMDRV_DEBUG("%s: CONTRAST_PLUS_1\n", __func__);
            ce147_buf_set_contrast[1] = 0x04;
            break;

        case CONTRAST_PLUS_2:
            CAMDRV_DEBUG("%s: CONTRAST_PLUS_2\n", __func__);
            ce147_buf_set_contrast[1] = 0x05;
            break;

        default:
            CAMDRV_DEBUG("%s: CONTRAST_DEFAULT\n", __func__);
            ce147_buf_set_contrast[1] = 0x03;
            break;
    }

    ce147_status->contrast = val;
    err = ce147_i2c_write_multi(CMD_SET_EFFECT, ce147_buf_set_contrast, ce147_len_set_contrast);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_contrast\n", __func__);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_contrast\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_sharpness(int val)
{
    int err;
    unsigned char ce147_buf_set_saturation[2] = { 0x02, 0x00 };
    unsigned int ce147_len_set_saturation = 2;
    
    switch(val)
    {
        case SHARPNESS_MINUS_2:
            CAMDRV_DEBUG("%s: SHARPNESS_MINUS_2\n", __func__);
            ce147_buf_set_saturation[1] = 0x01;
            break;

        case SHARPNESS_MINUS_1:
            CAMDRV_DEBUG("%s: SHARPNESS_MINUS_1\n", __func__);
            ce147_buf_set_saturation[1] = 0x02;
            break;

        case SHARPNESS_DEFAULT:
            CAMDRV_DEBUG("%s: SHARPNESS_DEFAULT\n", __func__);
            ce147_buf_set_saturation[1] = 0x03;
            break;

        case SHARPNESS_PLUS_1:
            CAMDRV_DEBUG("%s: SHARPNESS_PLUS_1\n", __func__);
            ce147_buf_set_saturation[1] = 0x04;
            break;

        case SHARPNESS_PLUS_2:
            CAMDRV_DEBUG("%s: SHARPNESS_PLUS_2\n", __func__);
            ce147_buf_set_saturation[1] = 0x05;
            break;

        default:
            CAMDRV_DEBUG("%s: SHARPNESS_DEFAULT\n", __func__);
            ce147_buf_set_saturation[1] = 0x03;
            break;
    }

    ce147_status->sharpness = val;
    err = ce147_i2c_write_multi(CMD_SET_EFFECT, ce147_buf_set_saturation, ce147_len_set_saturation);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_saturation\n", __func__);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_saturation\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_wdr(int val)
{
    int err;
    unsigned char ce147_buf_set_wdr[1] = { 0x00 };
    unsigned int ce147_len_set_wdr = 1;
    
    switch(val)
    {
        case WDR_ON:
            CAMDRV_DEBUG("%s: WDR_ON\n", __func__);
            ce147_buf_set_wdr[0] = 0x01;
            break;
        
        case WDR_OFF:            
        default:
            CAMDRV_DEBUG("%s: WDR_OFF\n", __func__);
            ce147_buf_set_wdr[0] = 0x00;
        break;
    }

    err = ce147_i2c_write_multi(CMD_SET_WDR, ce147_buf_set_wdr, ce147_len_set_wdr);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_wdr\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_anti_shake(int val)
{
    int err;
    unsigned char ce147_buf_set_anti_shake[1] = { 0x00 };
    unsigned int ce147_len_set_anti_shake = 1;
    
    switch(val)
    {
        case ANTI_SHAKE_STILL_ON:
            CAMDRV_DEBUG("%s: ANTI_SHAKE_STILL_ON\n", __func__);
            ce147_buf_set_anti_shake[0] = 0x01;
        break;

        case ANTI_SHAKE_MOVIE_ON:
            CAMDRV_DEBUG("%s: ANTI_SHAKE_MOVIE_ON\n", __func__);
            ce147_buf_set_anti_shake[0] = 0x10;
        break;

        case ANTI_SHAKE_OFF:
        default:
            CAMDRV_DEBUG("%s: ANTI_SHAKE_OFF\n", __func__);
            ce147_buf_set_anti_shake[0] = 0x00;
        break;
    }

    err = ce147_i2c_write_multi(CMD_SET_ANTI_SHAKE, ce147_buf_set_anti_shake, ce147_len_set_anti_shake);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for anti_shake\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_continuous_af(int val)
{
    int err;
    unsigned char ce147_buf_set_caf[1] = { 0x02 };
    unsigned char ce147_buf_start_af_search[1] = { 0x00 };
    unsigned int ce147_len_start_af_search = 1;
#if 0
    unsigned char ce147_buf_set_af[1] = { 0x00 };
#endif
    unsigned char ce147_buf_stop_lens_movement[1] = { 0x00 };

    CAMDRV_DEBUG("%s\n", __func__);
    
    /* need to set face_detection with noline */
    //
    
    if(val) {
        err = ce147_get_focus_mode(CMD_SET_AUTO_FOCUS_MODE, ce147_buf_set_caf);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: start_continous_af\n", __func__);
            return -EIO;
        }

        //start af search
        err = ce147_i2c_write_multi(CMD_START_AUTO_FOCUS_SEARCH, ce147_buf_start_af_search, ce147_len_start_af_search);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for start_continous_af\n", __func__);
            return -EIO;
        }
    } else {
        err = ce147_get_focus_mode(CMD_STOP_LENS_MOVEMENT, ce147_buf_stop_lens_movement);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: stop_continous_af\n", __func__);
            return -EIO;
        }
#if 0
        err = ce147_get_focus_mode(client, CMD_SET_AUTO_FOCUS_MODE, ce147_buf_set_af);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: stop_continous_af\n", __func__);
            return -EIO;
        }
#endif
    }

    return 0;
}

static int ce147_set_object_tracking(int val)
{
    int err;
    int count;
    unsigned short x;
    unsigned short y;

    unsigned char ce147_buf_set_object_tracking[7] = { 0x00, };
    unsigned int ce147_len_set_object_tracking = 7;
    unsigned char ce147_buf_check_object_tracking[9] = { 0x00, };
    unsigned int ce147_len_check_object_tracking = 9;
    unsigned char ce147_buf_stop_lens[1] = { 0x00 };    

    /* get x,y touch position */
    x = ce147_status->position.x;
    y = ce147_status->position.y;

    CAMDRV_DEBUG("%s x[%d], y[%d]\n", __func__,x,y);

    if(OT_START) {            
        ce147_buf_set_object_tracking[3] = (x & 0x00FF);    
        ce147_buf_set_object_tracking[4] = ((x    & 0xFF00) >> 8);
        ce147_buf_set_object_tracking[5] = (y  & 0x00FF);    
        ce147_buf_set_object_tracking[6] = ((y & 0xFF00) >> 8);
    
        err = ce147_i2c_write_multi(CMD_START_OT, ce147_buf_set_object_tracking, ce147_len_set_object_tracking);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for object_tracking\n", __func__);
            return -EIO;
        }

        /* Read status whether AF Tracking is successful or fail */
        for(count = 0; count < 300; count++) {
            msleep(10);
            ce147_buf_check_object_tracking[0] = 0xFF;
            err = ce147_i2c_read_multi( CMD_CHECK_OT, NULL, 0, ce147_buf_check_object_tracking, ce147_len_check_object_tracking);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for object_tracking\n", __func__);
                return -EIO;
            }
            if(ce147_buf_check_object_tracking[0] == 0x02 || ce147_buf_check_object_tracking[0] == 0x03) break;
        }    

        /* OT status: searching an object in progess */
        if(ce147_buf_check_object_tracking[0] == 0x01) {
            ce147_status->ot_status = 1;
        }    
        /* OT status: an object is detected successfully */
        else if(ce147_buf_check_object_tracking[0] == 0x02) {
            err = ce147_set_continuous_af(val);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_start_continous_af for object_tracking\n", __func__);
                return -EIO;
            }
            ce147_status->ot_status = 2;
        }
        /* OT status: an object detecting is failed */
        else if(ce147_buf_check_object_tracking[0] == 0x03) {
            ce147_status->ot_status = 3;
        }
    } else    {
        err = ce147_get_focus_mode(CMD_STOP_LENS_MOVEMENT, ce147_buf_stop_lens);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_start_continous_af for object_tracking\n", __func__);
            return -EIO;
        }    

        err = ce147_i2c_write_multi(CMD_START_OT, ce147_buf_set_object_tracking, ce147_len_set_object_tracking);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for object_tracking\n", __func__);
            return -EIO;
        }
    }
        
    return 0;
}

static int ce147_get_object_tracking(void)
{
    int err;

    unsigned char ce147_buf_check_object_tracking[9] = { 0x00, };
    unsigned int ce147_len_check_object_tracking = 9;

    ce147_buf_check_object_tracking[0] = 0xFF;
    err = ce147_i2c_read_multi(CMD_CHECK_OT, NULL, 0, ce147_buf_check_object_tracking, ce147_len_check_object_tracking);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for object_tracking\n", __func__);
        return -EIO;
    }

    /* OT status: searching an object in progess */
    if(ce147_buf_check_object_tracking[0] == 0x01)
    {
        ce147_status->ot_status = 1;
    }    
    
    /* OT status: an object is detected successfully */
    else if(ce147_buf_check_object_tracking[0] == 0x02)
    {
        ce147_status->ot_status = 2;
    }

    /* OT status: an object detecting is failed */
    else if(ce147_buf_check_object_tracking[0] == 0x03)
    {
        ce147_status->ot_status = 3;
    }
    
    /* OT status: detected object is missing  */
    else if(ce147_buf_check_object_tracking[0] == 0x04)
    {
        ce147_status->ot_status = 4;
    }    
    
    CAMDRV_DEBUG("%s: ot_status [%d]\n", __func__, ce147_status->ot_status);

    return 0;
}

static int ce147_set_face_detection(int val)
{
    int err;
    unsigned char ce147_buf_set_fd[3] = { 0x00, 0x00, 0x00 };
    unsigned int ce147_len_set_fd = 3;
    
    switch(val)
    {
        case FACE_DETECTION_ON:
            CAMDRV_DEBUG("%s: FACE_DETECTION_ON\n", __func__);
            ce147_buf_set_fd[0] = 0x03;
            ce147_buf_set_fd[1] = 0x01;
            ce147_buf_set_fd[2] = 0x0A;
            break;

        case FACE_DETECTION_ON_BEAUTY:
            CAMDRV_DEBUG("%s: FACE_DETECTION_ON_BEAUTY\n", __func__);
            ce147_buf_set_fd[0] = 0x01;
            ce147_buf_set_fd[1] = 0x01;
            ce147_buf_set_fd[2] = 0x0A;
            break;

        case FACE_DETECTION_NOLINE:
            CAMDRV_DEBUG("%s: FACE_DETECTION_NOLINE\n", __func__);
            ce147_buf_set_fd[0] = 0x03;
            ce147_buf_set_fd[1] = 0x00;
            ce147_buf_set_fd[2] = 0x0A;
            break;

        case FACE_DETECTION_OFF:
        default:
            CAMDRV_DEBUG("%s: FACE_DETECTION_OFF\n", __func__);
            ce147_buf_set_fd[0] = 0x00;
            ce147_buf_set_fd[1] = 0x00;
            ce147_buf_set_fd[2] = 0x00;
            break;
    }
    
    err = ce147_i2c_write_multi(CMD_SET_FACE_DETECTION, ce147_buf_set_fd, ce147_len_set_fd);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for face_detection\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_smart_auto(int val)
{
    int err;

    unsigned char ce147_buf_set_smart_auto[1] = { 0x00 };
    unsigned int ce147_len_set_smart_auto = 1;

    CAMDRV_DEBUG("%s: val[%d]\n", __func__,val);
    
    if(val == SMART_AUTO_ON)
    {
        ce147_buf_set_smart_auto[0] = 0x01;
        err = ce147_i2c_write_multi(CMD_SET_SMART_AUTO, ce147_buf_set_smart_auto, ce147_len_set_smart_auto);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for smart_auto\n", __func__);
            return -EIO;
        }
#if 0
        err = ce147_set_continous_af( CAF_START);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: CAF_START for smart_auto\n", __func__);
            return -EIO;
        }
#endif
    }
    else
    {
#if 0    
        err = ce147_set_continous_af( CAF_STOP);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: CAF_START for smart_auto\n", __func__);
            return -EIO;
        }
#endif
        
        ce147_buf_set_smart_auto[0] = 0x00;
        err = ce147_i2c_write_multi(CMD_SET_SMART_AUTO, ce147_buf_set_smart_auto, ce147_len_set_smart_auto);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for smart_auto\n", __func__);
            return -EIO;
        }
    }

    return 0;
}

static int ce147_get_smart_auto_status(void)
{
    int err;
    unsigned char ce147_buf_smart_auto_status[2] = {0x00, 0x00};

    ce147_buf_smart_auto_status[0] = 0xFF;
    err = ce147_i2c_read_multi( CMD_GET_SMART_AUTO_STATUS, NULL, 0, ce147_buf_smart_auto_status, 2);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for auto_status\n", __func__);
        return -EIO;
    }

    if(ce147_buf_smart_auto_status[0] == 0x00 || ce147_buf_smart_auto_status[0] == 0x01) {
        ce147_status->sa_status = SMART_AUTO_STATUS_AUTO;
    } else {
        switch(ce147_buf_smart_auto_status[1])
        {
            case 0x00:
                ce147_status->sa_status = SMART_AUTO_STATUS_LANDSCAPE;
            break;
        
            case 0x01:
                ce147_status->sa_status = SMART_AUTO_STATUS_PORTRAIT;
            break;
        
            case 0x02:
                ce147_status->sa_status = SMART_AUTO_STATUS_NIGHT;
            break;
        
            case 0x03:
                ce147_status->sa_status = SMART_AUTO_STATUS_PORTRAIT_NIGHT;
            break;
                
            case 0x04:
                ce147_status->sa_status = SMART_AUTO_STATUS_MACRO;
            break;            
            
            case 0x05:
                ce147_status->sa_status = SMART_AUTO_STATUS_PORTRAIT_BACKLIT;
            break;
        
            case 0x06:
                ce147_status->sa_status = SMART_AUTO_STATUS_PORTRAIT_ANTISHAKE;
            break;
            
            case 0x07:
                ce147_status->sa_status = SMART_AUTO_STATUS_ANTISHAKE;
            break;    
        }    
    }

    CAMDRV_DEBUG("%s: smartauto_status [%d]\n", __func__,ce147_status->sa_status);

    return 0;
}

static int ce147_set_touch_auto_focus(int val)
{
    int err;
    unsigned short x;
    unsigned short y;

    unsigned char ce147_buf_set_touch_af[11] = { 0x00, };
    unsigned int ce147_len_set_touch_af = 11;

    CAMDRV_DEBUG("%s: x[%d], y[%d]\n", __func__,ce147_status->position.x,ce147_status->position.y);

    err = ce147_set_ae_awb(AE_UNLOCK_AWB_UNLOCK);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_ae_awb, err %d\n", __func__, err);
        return -EIO;
    }

    /* get x,y touch position */
    x = ce147_status->position.x;
    y = ce147_status->position.y;
    
    if(val == TOUCH_AF_START)
    {
        ce147_buf_set_touch_af[0] = 0x01;
        ce147_buf_set_touch_af[1] = 0x03;
        ce147_buf_set_touch_af[2] = 0x00;
        ce147_buf_set_touch_af[3] = ((x - 0x32) & 0x00FF);    
        ce147_buf_set_touch_af[4] = (((x - 0x32) & 0xFF00) >> 8);
        ce147_buf_set_touch_af[5] = ((y - 0x32) & 0x00FF);    
        ce147_buf_set_touch_af[6] = (((y - 0x32) & 0xFF00) >> 8);
        ce147_buf_set_touch_af[7] = ((x + 0x32) & 0x00FF);    
        ce147_buf_set_touch_af[8] = (((x + 0x32) & 0xFF00) >> 8);
        ce147_buf_set_touch_af[9] = ((y + 0x32) & 0x00FF);    
        ce147_buf_set_touch_af[10] = (((y + 0x32) & 0xFF00) >> 8);
    }
    
    err = ce147_i2c_write_multi(CMD_SET_TOUCH_AUTO_FOCUS, ce147_buf_set_touch_af, ce147_len_set_touch_af);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for touch_auto_focus\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_focus_mode(int val)
{
    int err;
    unsigned char ce147_buf_set_focus_mode[1] = { 0x00 };

    //err = ce147_set_face_detection(FACE_DETECTION_OFF);
    switch(val)
    {
        case FOCUS_MODE_MACRO:
            CAMDRV_DEBUG("%s: FOCUS_MODE_MACRO\n", __func__);
            ce147_buf_set_focus_mode[0] = 0x01;
            break;
        
        case FOCUS_MODE_FD:
            break;
        
        case FOCUS_MODE_TOUCH:
            CAMDRV_DEBUG("%s: FOCUS_MODE_TOUCH\n", __func__);
            ce147_set_touch_auto_focus(TOUCH_AF_START);
            break;
        
        case FOCUS_MODE_AUTO:
        default:
            CAMDRV_DEBUG("%s: FOCUS_MODE_AUTO\n", __func__);
            ce147_buf_set_focus_mode[0] = 0x00;
        break;
    }
    
#if 0
    if(ce147_status->hd_preview_on == 1)
    {
        ce147_buf_set_focus_mode[0] = 0x07;
    }
#endif

    ce147_status->cur_af_status = ce147_buf_set_focus_mode[0];

    if(val != FOCUS_MODE_FD)
    {
        if((ce147_status->pre_af_status != ce147_status->cur_af_status))
        //    || (val == FOCUS_MODE_MACRO_DEFAULT)||(val == FOCUS_MODE_AUTO_DEFAULT)|| (val == FOCUS_MODE_FD_DEFAULT))
        {
            CAMDRV_DEBUG("%s: unlock\n", __func__);        
            err = ce147_set_ae_awb(AE_UNLOCK_AWB_UNLOCK);
            if(err < 0) {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_awb_unlock, err %d\n", __func__, err);
                return -EIO;
            }

            err = ce147_get_focus_mode(CMD_SET_AUTO_FOCUS_MODE, ce147_buf_set_focus_mode);
            if(err < 0)    {
                printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: get_focus_mode\n", __func__);
                return -EIO;
            }
        ce147_status->pre_af_status = ce147_status->cur_af_status;
        }
    }        
    
    return 0;
}

static int ce147_set_vintage_mode(int val)
{
    int err;

    unsigned char ce147_buf_set_vintage_mode[4] = { 0x10, 0x01, 0x32, 0x00 };
    unsigned int ce147_len_set_vintage_mode = 4;
    
    switch(val)
    {
        case VINTAGE_MODE_OFF:
            CAMDRV_DEBUG("%s: VINTAGE_MODE_OFF\n", __func__);
            ce147_buf_set_vintage_mode[1] = 0x00;
            ce147_buf_set_vintage_mode[2] = 0x00;
            ce147_buf_set_vintage_mode[3] = 0x00;
            break;

        case VINTAGE_MODE_NORMAL:
            CAMDRV_DEBUG("%s: VINTAGE_MODE_NORMAL\n", __func__);
            ce147_buf_set_vintage_mode[3] = 0x00;
            break;

        case VINTAGE_MODE_WARM:
            CAMDRV_DEBUG("%s: VINTAGE_MODE_WARM\n", __func__);
            ce147_buf_set_vintage_mode[3] = 0x02;
            break;

        case VINTAGE_MODE_COOL:
            CAMDRV_DEBUG("%s: VINTAGE_MODE_COOL\n", __func__);
            ce147_buf_set_vintage_mode[3] = 0x01;
            break;
        
        case VINTAGE_MODE_BNW:
            CAMDRV_DEBUG("%s: VINTAGE_MODE_BNW\n", __func__);
            ce147_buf_set_vintage_mode[3] = 0x03;
            break;

        default:
            CAMDRV_DEBUG("%s: VINTAGE_MODE_OFF\n", __func__);
            ce147_buf_set_vintage_mode[1] = 0x00;
            ce147_buf_set_vintage_mode[2] = 0x00;
            ce147_buf_set_vintage_mode[3] = 0x00;
            break;
    }

    err = ce147_i2c_write_multi(CMD_SET_EFFECT_SHOT, ce147_buf_set_vintage_mode, ce147_len_set_vintage_mode);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for vintage_mode\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_face_beauty(int val)
{
    int err;
    unsigned char ce147_buf_set_face_beauty[4] = { 0x00, 0x00, 0x00, 0x00 };
    unsigned int ce147_len_set_face_beauty = 4;

    switch(val)
    {
        case BEAUTY_SHOT_ON:
            CAMDRV_DEBUG("%s: BEAUTY_SHOT_ON\n", __func__);
            ce147_buf_set_face_beauty[1] = 0x01;
            ce147_buf_set_face_beauty[2] = 0x32;
            ce147_buf_set_face_beauty[3] = 0x01;
            break;
        
        case BEAUTY_SHOT_OFF:
        default:
            CAMDRV_DEBUG("%s: BEAUTY_SHOT_OFF\n", __func__);
            break;
    }

    //Need to set face detection as 'face beauty on' mode.
    err = ce147_i2c_write_multi(CMD_SET_EFFECT_SHOT, ce147_buf_set_face_beauty, ce147_len_set_face_beauty);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_face_beauty\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_white_balance(int val)
{
    int err;
    
    unsigned char ce147_buf_set_wb_auto[1] = { 0x01 };
    unsigned char ce147_buf_set_wb[2] = { 0x10, 0x00 };
    unsigned int ce147_len_set_wb_auto = 1;
    unsigned int ce147_len_set_wb = 2;

    ce147_status->wb = val;
    
    switch(val)
    {
        case WHITE_BALANCE_AUTO:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_AUTO\n", __func__);

               ce147_buf_set_wb_auto[0] = 0x00;
            break;

        case WHITE_BALANCE_SUNNY:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_SUNNY\n", __func__);
            ce147_buf_set_wb[1] = 0x00;
            break;

        case WHITE_BALANCE_CLOUDY:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_CLOUDY\n", __func__);
            ce147_buf_set_wb[1] = 0x01;
            break;

        case WHITE_BALANCE_TUNGSTEN:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_TUNGSTEN\n", __func__);
            ce147_buf_set_wb[1] = 0x02;
            break;

        case WHITE_BALANCE_FLUORESCENT:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_FLUORESCENT\n", __func__);
            ce147_buf_set_wb[1] = 0x03;
            break;

        case WHITE_BALANCE_FALLCOLOR:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_FALLCOLOR\n", __func__);
            ce147_buf_set_wb[1] = 0x11;
            break;

        case WHITE_BALANCE_SUNSET:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_SUNSET\n", __func__);
            ce147_buf_set_wb[1] = 0x00; //0x10;
            break;

        case WHITE_BALANCE_DAWN:
            CAMDRV_DEBUG("%s: WHITE_BALANCE_DAWN\n", __func__);
            ce147_buf_set_wb[1] = 0x03;
            break;        

        default:
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: to set_white_balance, enum: %d\n", __func__, val);
            return -EINVAL;
        break;
    }

    if(val != WHITE_BALANCE_AUTO)
    {
        err = ce147_i2c_write_multi(CMD_SET_WB, ce147_buf_set_wb, ce147_len_set_wb);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for white_balance\n", __func__);
            return -EIO;
        }
    }
    
    err = ce147_i2c_write_multi(CMD_SET_WB_AUTO, ce147_buf_set_wb_auto, ce147_len_set_wb_auto);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for white_balance\n", __func__);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for white_balance\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_ev(int val)
{
    int err;

    unsigned char ce147_buf_set_ev[2] = { 0x02, 0x00 };
    unsigned int ce147_len_set_ev = 2;
    unsigned int ce147_ev_offset = 13;

    ce147_status->ev = val;

    switch(val)
    {
        case EV_MINUS_4:
            CAMDRV_DEBUG("%s: EV_MINUS_4\n", __func__);
            ce147_buf_set_ev[1] = 0x02;
            break;

        case EV_MINUS_3:
            CAMDRV_DEBUG("%s: EV_MINUS_3\n", __func__);
            ce147_buf_set_ev[1] = 0x03;
            break;

        case EV_MINUS_2:
            CAMDRV_DEBUG("%s: EV_MINUS_2\n", __func__);
            ce147_buf_set_ev[1] = 0x04;
            break;

        case EV_MINUS_1:
            CAMDRV_DEBUG("%s: EV_MINUS_1\n", __func__);
            ce147_buf_set_ev[1] = 0x05;
            break;

        case EV_DEFAULT:
            CAMDRV_DEBUG("%s: EV_DEFAULT\n", __func__);
            ce147_buf_set_ev[1] = 0x06;
            break;

        case EV_PLUS_1:
            CAMDRV_DEBUG("%s: EV_PLUS_1\n", __func__);
            ce147_buf_set_ev[1] = 0x07;
            break;

        case EV_PLUS_2:
            CAMDRV_DEBUG("%s: EV_PLUS_2\n", __func__);
            ce147_buf_set_ev[1] = 0x08;
            break;
        
        case EV_PLUS_3:
            CAMDRV_DEBUG("%s: EV_PLUS_3\n", __func__);
            ce147_buf_set_ev[1] = 0x09;
            break;

        case EV_PLUS_4:
            CAMDRV_DEBUG("%s: EV_PLUS_4\n", __func__);
            ce147_buf_set_ev[1] = 0x0A;
            break;            

        default:
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: to set_ev, enum: %d\n", __func__, val);
            return -EINVAL;
            break;
    }

    if(ce147_status->hd_preview_on) //This is for HD REC preview
    {
        ce147_buf_set_ev[1] += ce147_ev_offset;
    }
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_set_ev: set_ev:, data: 0x%02x\n", ce147_buf_set_ev[1]);

    err = ce147_i2c_write_multi(CMD_SET_WB, ce147_buf_set_ev, ce147_len_set_ev);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_ev, HD preview(%d)\n", __func__, ce147_status->hd_preview_on);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_ev\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_metering(int val)
{
    int err;

    unsigned char ce147_buf_set_metering[2] = { 0x00, 0x00 };
    unsigned int ce147_len_set_metering = 2;
    
    switch(val)
    {
        case METERING_MATRIX:
            CAMDRV_DEBUG("%s: METERING_MATRIX\n", __func__);
            ce147_buf_set_metering[1] = 0x02;
            break;

        case METERING_CENTER:
            CAMDRV_DEBUG("%s: METERING_CENTER\n", __func__);
            ce147_buf_set_metering[1] = 0x00;
            break;

        case METERING_SPOT:
            CAMDRV_DEBUG("%s: METERING_SPOT\n", __func__);
            ce147_buf_set_metering[1] = 0x01;
            break;
        
	     case METERING_HD:
            CAMDRV_DEBUG("%s: METERING_HD\n", __func__);
            ce147_buf_set_metering[1] = 0x03;
            break;  
        case METERING_LAND:
            CAMDRV_DEBUG("%s: METERING_LAND\n", __func__);
            ce147_buf_set_metering[1] = 0x05;
            break;        

        default:
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: to set_photometry, enum: %d\n", __func__, val);
            //return -EINVAL;// temporary patch. hc.hyun 2011-02-09
        break;
    }

    if(ce147_status->hd_preview_on)
    {
        ce147_buf_set_metering[1] = 0x03;
    }

    err = ce147_i2c_write_multi(CMD_SET_WB, ce147_buf_set_metering, ce147_len_set_metering);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_photometry\n", __func__);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_photometry\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_iso(int val)
{
    int err;
    unsigned char ce147_buf_set_iso[2] = { 0x01, 0x00 };
    unsigned int ce147_len_set_iso = 2;

    ce147_status->iso = val;
    
    switch(val)
    {
        case ISO_AUTO:
            CAMDRV_DEBUG("%s: ISO_AUTO\n", __func__);
            ce147_buf_set_iso[1] = 0x06;
            break;

        case ISO_50:
            CAMDRV_DEBUG("%s: ISO_50\n", __func__);
            ce147_buf_set_iso[1] = 0x07;
            break;

        case ISO_100:
            CAMDRV_DEBUG("%s: ISO_100\n", __func__);
            ce147_buf_set_iso[1] = 0x08;
            break;

        case ISO_200:
            CAMDRV_DEBUG("%s: ISO_200\n", __func__);
            ce147_buf_set_iso[1] = 0x09;
            break;

        case ISO_400:
            CAMDRV_DEBUG("%s: ISO_400\n", __func__);
            ce147_buf_set_iso[1] = 0x0A;
            break;

        case ISO_800:
            CAMDRV_DEBUG("%s: ISO_800\n", __func__);
            ce147_buf_set_iso[1] = 0x0B;
            break;

        case ISO_1600:
            CAMDRV_DEBUG("%s: ISO_1600\n", __func__);
            ce147_buf_set_iso[1] = 0x0C;
            break;

        /* This is additional setting for Sports' scene mode */
        case ISO_SPORTS:
            CAMDRV_DEBUG("%s: ISO_SPORTS\n", __func__);
            ce147_buf_set_iso[1] = 0x12;
            break;
        
        /* This is additional setting for 'Night' scene mode */
        case ISO_NIGHT:
            CAMDRV_DEBUG("%s: ISO_NIGHT\n", __func__);
            ce147_buf_set_iso[1] = 0x17;
            break;

        case ISO_FIREWORK:
            CAMDRV_DEBUG("%s: ISO_FIREWORK\n", __func__);
            ce147_buf_set_iso[1] = 0x11;
            break;        

        /* This is additional setting for video recording mode */
        case ISO_MOVIE:
            CAMDRV_DEBUG("%s: ISO_MOVIE\n", __func__);
            ce147_buf_set_iso[1] = 0x02;
            break;

        default:
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: to set_iso, enum: %d\n", __func__, val);
            ce147_buf_set_iso[1] = 0x06;
            //return -EINVAL;
            break;
    }

    if(ce147_status->hd_preview_on)ce147_buf_set_iso[1] = 0x02;

    err = ce147_i2c_write_multi(CMD_SET_WB, ce147_buf_set_iso, ce147_len_set_iso);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for set_iso\n", __func__);
        return -EIO;
    }
    
#if 0 //remove batch    
    err = ce147_get_batch_reflection_status();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_get_batch_reflection_status for set_iso\n", __func__);
        return -EIO; 
    }
#endif

    return 0;
}

static int ce147_set_gamma(int val)
{
    int err;

    unsigned char ce147_buf_set_gamma[2] = { 0x01, 0x00 };
    unsigned int ce147_len_set_gamma = 2;

    unsigned char ce147_buf_set_uv[2] = { 0x03, 0x00 };
    unsigned int ce147_len_set_uv = 2;

    if(val == GAMMA_ON)
    {
        if(ce147_status->hd_preview_on)
        {
            ce147_buf_set_gamma[1] = 0x01;
            ce147_buf_set_uv[1] = 0x01;
        }
    }
    
    err = ce147_i2c_write_multi(CMD_SET_EFFECT, ce147_buf_set_gamma, ce147_len_set_gamma);
    if(err < 0)
    {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for ce147_set_gamma\n", __func__);
        return -EIO;
    }
    mdelay(10);   
    err = ce147_i2c_write_multi(CMD_SET_EFFECT, ce147_buf_set_uv, ce147_len_set_uv);
    if(err < 0)
    {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for ce147_set_gamma\n", __func__);
        return -EIO;
    }
    
    CAMDRV_DEBUG("%s: done, gamma: 0x%02x, uv: 0x%02x, hd: %d\n", __func__, ce147_buf_set_gamma[1], ce147_buf_set_uv[1], ce147_status->hd_preview_on);

    return 0;
}

static int ce147_set_slow_ae(int val)
{
    int err;

    unsigned char ce147_buf_set_slow_ae[2] = { 0x03, 0x00 };
    unsigned int ce147_len_set_slow_ae = 2;

    if(val == SLOW_AE_ON)
    {
        if(ce147_status->hd_preview_on)
        {
            ce147_buf_set_slow_ae[1] = 0x02;
        }
    }

    err = ce147_i2c_write_multi(CMD_SET_WB, ce147_buf_set_slow_ae, ce147_len_set_slow_ae);
    if(err < 0)
    {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for ce147_set_slow_ae\n", __func__);
        return -EIO;
    }
    
    CAMDRV_DEBUG("%s: done, slow_ae: 0x%02x, hd: %d\n", __func__, ce147_buf_set_slow_ae[1], ce147_status->hd_preview_on);

    return 0;
}

static int ce147_set_scene_mode(int val)
{
    int err = 0;
    
    switch(val){
           case SCENE_MODE_PORTRAIT:
                CAMDRV_DEBUG("%s: SCENE_MODE_PORTRAIT\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_CENTER);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_MINUS_1);
                err = ce147_set_face_detection(FACE_DETECTION_ON);
                //err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);
                break;
            case SCENE_MODE_LANDSCAPE:
                CAMDRV_DEBUG("%s: SCENE_MODE_LANDSCAPE\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_MATRIX);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_PLUS_1);
                err = ce147_set_sharpness(SHARPNESS_PLUS_1);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
            case SCENE_MODE_NIGHT:
                CAMDRV_DEBUG("%s: SCENE_MODE_NIGHT\n", __func__);
                err = ce147_set_iso(ISO_NIGHT);
                err = ce147_set_metering(METERING_MATRIX);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
            case SCENE_MODE_SPORTS:
                CAMDRV_DEBUG("%s: SCENE_MODE_SPORTS\n", __func__);
                err = ce147_set_iso(ISO_SPORTS);
                err = ce147_set_metering(METERING_MATRIX);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_PLUS_1);
                err = ce147_set_sharpness(SHARPNESS_PLUS_1);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
            case SCENE_MODE_PARTY:
                CAMDRV_DEBUG("%s: SCENE_MODE_PARTY\n", __func__);
                err = ce147_set_iso(ISO_200);
                err = ce147_set_metering(METERING_MATRIX);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_PLUS_1);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_PLUS_2);            
                break;
            case SCENE_MODE_BEACH:
                CAMDRV_DEBUG("%s: SCENE_MODE_BEACH\n", __func__);
                err = ce147_set_iso(ISO_50);
                err = ce147_set_metering(METERING_CENTER);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_PLUS_1);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_PLUS_1);            
                break;
            case SCENE_MODE_SUNSET:
                CAMDRV_DEBUG("%s: SCENE_MODE_SUNSET\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_CENTER);
                err = ce147_set_white_balance(WHITE_BALANCE_SUNNY);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);            
                break;
            case SCENE_MODE_DAWN:
                CAMDRV_DEBUG("%s: SCENE_MODE_DAWN\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_CENTER);
                err = ce147_set_white_balance(WHITE_BALANCE_DAWN);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
            case SCENE_MODE_FALL_COLOR:
                CAMDRV_DEBUG("%s: SCENE_MODE_FALL_COLOR\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_MATRIX);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_PLUS_2);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
            case SCENE_MODE_FIREWORKS:
                CAMDRV_DEBUG("%s: SCENE_MODE_FIREWORKS\n", __func__);
                err = ce147_set_iso(ISO_50);
                err = ce147_set_metering(METERING_CENTER);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);            
                break;
            case SCENE_MODE_TEXT:
                CAMDRV_DEBUG("%s: SCENE_MODE_TEXT\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_CENTER);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_PLUS_2);
                err = ce147_set_focus_mode(FOCUS_MODE_MACRO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
            case SCENE_MODE_CANDLELIGHT:/* same with NIGHTMODE*/
                CAMDRV_DEBUG("%s: SCENE_MODE_CANDLELIGHT\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_MATRIX);
                err = ce147_set_white_balance(WHITE_BALANCE_SUNSET);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);            
                break;
            case SCENE_MODE_BACKLIGHT:
                CAMDRV_DEBUG("%s: SCENE_MODE_BACKLIGHT\n", __func__);
                err = ce147_set_iso(ISO_AUTO);
                err = ce147_set_metering(METERING_SPOT);
                err = ce147_set_white_balance(WHITE_BALANCE_AUTO);
                err = ce147_set_saturation(SATURATION_DEFAULT);
                err = ce147_set_sharpness(SHARPNESS_DEFAULT);
                err = ce147_set_focus_mode(FOCUS_MODE_AUTO);
                err = ce147_set_ev(EV_DEFAULT);                
                break;
    }

	if(val != SCENE_MODE_PORTRAIT)
		err = ce147_set_face_detection(FACE_DETECTION_OFF);
		
    return err;
}

static int ce147_set_face_lock(int val)
{
    int err;

    unsigned char ce147_buf_set_fd_lock[1] = { 0x00 };
    unsigned int ce147_len_set_fd_lock = 1;
    
    switch(val) 
    {
        case FACE_LOCK_ON:
            CAMDRV_DEBUG("%s: FACE_LOCK_ON\n", __func__);
            ce147_buf_set_fd_lock[0] = 0x01;
            break;

        case FIRST_FACE_TRACKING:
            CAMDRV_DEBUG("%s: FIRST_FACE_TRACKING\n", __func__);
            ce147_buf_set_fd_lock[0] = 0x02;
            break;

        case FACE_LOCK_OFF:
        default:
            CAMDRV_DEBUG("%s: FACE_LOCK_OFF\n", __func__);
            ce147_buf_set_fd_lock[0] = 0x00;
            break;
    }
    
    err = ce147_i2c_write_multi(CMD_SET_FACE_LOCK, ce147_buf_set_fd_lock, ce147_len_set_fd_lock);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for face_lock\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_auto_focus(int val)
{
    int err;
    
    unsigned char ce147_buf_set_af[1] = { 0x00 };
    unsigned int ce147_len_set_af = 1;

    if (val) {
        CAMDRV_DEBUG("%s: START AF\n", __func__);
        if(ce147_status->aeawb = AE_LOCK_AWB_LOCK)err = ce147_set_ae_awb(AE_UNLOCK_AWB_UNLOCK);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_ae_awb, err %d\n", __func__, err);
            return -EIO;
        }
        //start af
        err = ce147_i2c_write_multi(CMD_START_AUTO_FOCUS_SEARCH, ce147_buf_set_af, ce147_len_set_af);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for auto_focus\n", __func__);
            return -EIO;
        }
    } else {
        CAMDRV_DEBUG("%s: STOP AF\n", __func__);
        //stop af
        err = ce147_i2c_write_multi(CMD_STOP_LENS_MOVEMENT, ce147_buf_set_af, ce147_len_set_af);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_write for auto_focus\n", __func__);
            return -EIO;
        }
        err = ce147_set_ae_awb(AE_UNLOCK_AWB_UNLOCK);
        if(err < 0) {
            printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_ae_awb, err %d\n", __func__, err);
            return -EIO;
        }
    }

    return 0;
}

static int ce147_get_auto_focus_status(uint32_t* status)
{
    int err, val;
    unsigned char ce147_buf_get_af_status[1] = { 0x00 };

    ce147_buf_get_af_status[0] = 0xFF;
    err = ce147_i2c_read_multi(CMD_CHECK_AUTO_FOCUS_SEARCH, NULL, 0, ce147_buf_get_af_status, 1);
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: i2c_read for auto_focus_status\n", __func__);
        return -EIO;
    }    

    val = ce147_buf_get_af_status[0];
    *status = val;
    CAMDRV_DEBUG("%s: sTATUS [%d]\n", __func__,val);

    return 0;
}

static void ce147_init_parameters(void)
{
    /* Set initial values for the sensor stream parameters */
    //ce147_status->strm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //ce147_status->strm.parm.capture.timeperframe.numerator = 1;
    //ce147_status->strm.parm.capture.capturemode = 0;
    //ce147_status->framesize_index = CE147_PREVIEW_VGA;
    ce147_status->fps = 30; /* Default value */
    ce147_status->jpeg.enable = 1;
    ce147_status->jpeg.quality = 100;
    ce147_status->jpeg.main_offset = 0;
    ce147_status->jpeg.main_size = 0;
    ce147_status->jpeg.thumb_offset = 0;
    ce147_status->jpeg.thumb_size = 0;
    ce147_status->jpeg.postview_offset = 0;
}

int ce147_get_fw_data(void __user *argp)
{
    int err = -EINVAL;
    struct sensor_firmware_info firmware_info;
    
    //if (copy_from_user((void *)&firmware_info, (const void *)argp, sizeof(firmware_info)))
        //return -EFAULT;
    
    err = ce147_set_power(1);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(on)\n", __func__);
        return -EIO;
    }    

    err = ce147_load_fw();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Camera Initialization\n", __func__);
        return -EIO;
    }
    
    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_fw_data: ce147_load_fw is ok\n");    
    
    ce147_init_parameters();

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_fw_data: ce147_init_parameters is ok\n");

    err = ce147_get_fw_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading firmware version\n",__func__);
        return -EIO;
    }

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_fw_data: ce147_get_fw_version is ok\n");

    err = ce147_get_dateinfo();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading dateinfo\n",__func__);
        return -EIO;
    }

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_fw_data: ce147_get_dateinfo is ok\n");

    err = ce147_get_sensor_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading sensor info\n",__func__);
        return -EIO;
    }

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_fw_data: ce147_get_sensor_version is ok\n");

    err = ce147_get_sensor_maker_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading maker info\n",__func__);
        return -EIO;
    }

    err = ce147_get_af_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading af info\n",__func__);
        return -EIO;
    }

    err = ce147_get_gamma_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading camera gamma info\n",__func__);
        return -EIO;
    }

    err = ce147_set_power(0);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(off)\n", __func__);
        return -EIO;
    }

    //printk(KERN_ERR "[CAMDRV/CE147] CE147 :ce147_get_fw_data: ce147_set_power is ok\n");

    ce147_info("FW  Version: %d.%d\n", ce147_status->fw.major, ce147_status->fw.minor);
    ce147_info("PRM Version: %d.%d\n", ce147_status->prm.major, ce147_status->prm.minor);
    ce147_info("Date(y.m.d): %d.%d.%d\n", ce147_status->dateinfo.year, ce147_status->dateinfo.month, ce147_status->dateinfo.date);    
    ce147_info("Sensor version: %d\n", ce147_status->sensor_version);    

    firmware_info.fw.major = ce147_status->fw.major;
    firmware_info.fw.minor = ce147_status->fw.minor;

    firmware_info.prm.major = ce147_status->prm.major;
    firmware_info.prm.minor = ce147_status->prm.minor;

    firmware_info.dateinfo.year = ce147_status->dateinfo.year;
    firmware_info.dateinfo.month = ce147_status->dateinfo.month;
    firmware_info.dateinfo.date = ce147_status->dateinfo.date;

    firmware_info.sensor_info.maker= ce147_status->sensor_info.maker;
    firmware_info.sensor_info.optical = ce147_status->sensor_info.optical;
    
    firmware_info.af_info.low = ce147_status->af_info.low;
    firmware_info.af_info.high = ce147_status->af_info.high;
    
    firmware_info.gamma.rg_low = ce147_status->gamma.rg_low;
    firmware_info.gamma.rg_high = ce147_status->gamma.rg_high;
    firmware_info.gamma.bg_low = ce147_status->gamma.bg_low;
    firmware_info.gamma.bg_high = ce147_status->gamma.bg_high;

    if(copy_to_user((void *)argp, (const void *)&firmware_info, sizeof(firmware_info)))
        printk(" %s : copy_to_user Failed \n", __func__);
    
    return 0;
}

static int ce147_reset(void)
{
    int err = -EINVAL;

    printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Enter \n", __func__);

    err = ce147_set_power(0);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(off)\n", __func__);
        return -EIO;
        }

    mdelay(5);
    
    err = ce147_set_power(1);
    if(err < 0) {    
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: failed: ce147_set_power(off)\n", __func__);
        return -EIO;
        }
    
    err = ce147_load_fw();        //ce147_init();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Camera Initialization\n", __func__);
        return -EIO;
    }

    return 0;
}

static int ce147_set_power(int onoff)
{
    int rc = 0;
    unsigned int mclk_cfg;

    if (gpio_request(2, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_LDO_EN' failed\n", __func__);
    if (gpio_request(3, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_EN2' failed\n", __func__);
    if (gpio_request(31, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_VT_standby' failed\n", __func__);
    if (gpio_request(132, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_VT_rst' failed\n", __func__);    
    if (gpio_request(174, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_MEGA_nRST' failed\n", __func__);
    if (gpio_request(175, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_MEGA_EN' failed\n", __func__);
    if (gpio_request(177, "ce147"))printk(KERN_ERR "[CAMDRV/CE147] %s: gpio request 'CAM_MEGA_EN' failed\n", __func__);

    if (onoff) {   
        printk(KERN_ERR "[CAMDRV/CE147] %s: POWER ON.\n", __func__);
        main_camera_is_on = true;

        mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        gpio_direction_output(2, 0);
        gpio_direction_output(3, 0);    
        gpio_direction_output(31, 0);    
        gpio_direction_output(132, 0);
        gpio_direction_output(174, 0);
        gpio_direction_output(175, 0);    
        gpio_direction_output(177, 0);
        
        /*Turn CAM_ISP_CORE_1.2V on            LP8720 buck(SW), time step : 0us */
        lp8720_i2c_write(0x06, 0x09);            //000 01001
        lp8720_i2c_write(0x07, 0x09);            //000 01001
        /*Turn CAM_SENSOR_CORE_1.2V on        LP8720 LDO 1, time step : 0us */
        lp8720_i2c_write(0x01, 0x00);
    
        /*Turn CAM_AF_2.8V on                LP8720 LDO 5, time step : 50us */
        lp8720_i2c_write(0x05, 0x39);            //001 11001
        /*Turn CAM_ISP_RAM_1.8V on            LP8720 LDO 4, time step : 100us */
        lp8720_i2c_write(0x04, 0x51);            // 010 10001
        /*Turn CAM_ISP_HOST_1.8V on            LP8720 LDO 3, time step : 150us */
        lp8720_i2c_write(0x03, 0x6C);            // 011 01100
        /*Turn CAM_ISP_SYS_2.8V on            LP8720 LDO 2, time step : 200us*/
        lp8720_i2c_write(0x02, 0x99);            // 100 11001

        /* LP8720 enable */
        lp8720_i2c_write(0x08, 0xFF);
        gpio_set_value(2, 1);
        udelay(325);
        
        /*Turn CAM_SENSOR_A2.8V on            RP109 LDO VOUT (CAM_EN2, GPIO_003)*/
        gpio_set_value(3, 1);

        /*Turn CAM_SENSOR_IO_1.8V on            GPIO_177*/
        mdelay(1);
        gpio_set_value(177, 1);

        mdelay(1);
        /* Make VT standby High */
        gpio_set_value(31,1);
        mdelay(1);
        
        /* Enable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        mdelay(4);

        /* Make VT reset High */
        gpio_set_value(132,1);
        mdelay(5);
        
        /* Make VT standby Low */
        gpio_set_value(31,0);
        mdelay(1);
        
        /* Make standby pin high */
        gpio_set_value(175, 1);
        mdelay(1);
        
        /* Make RST pin high */
        gpio_set_value(174, 1);
        mdelay(5);
    } else {    
        printk(KERN_ERR "[CAMDRV/CE147] %s: POWER OFF.\n", __func__);
        main_camera_is_on = false;

        mclk_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        gpio_direction_output(2, 1);
        gpio_direction_output(3, 1);    
        gpio_direction_output(31, 1);    
        gpio_direction_output(132, 1);
        gpio_direction_output(174, 1);
        gpio_direction_output(175, 1);    
        gpio_direction_output(177, 1);
        
        /* Make VT standby low */
        gpio_set_value(31,0);
        mdelay(1);
        
        /* Make RST pin low */
        gpio_set_value(174, 0);
        udelay(50);
        
        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        udelay(50);

        /* Make standby pin low */
        gpio_set_value(175, 0);
        udelay(50);
        
        /* Make VT reset low */
        gpio_set_value(132,0);
        mdelay(1);
        
        /*Turn CAM_SENSOR_A2.8V off            RP109 LDO VOUT (CAM_EN2, GPIO_003)*/
        gpio_set_value(3, 0);
        /*Turn CAM_SENSOR_IO_1.8V off            GPIO_177*/
        gpio_set_value(177, 0);
        /*Turn CAM_AF_2.8V off                LP8720 LDO 5, time step : 50us */
        /*Turn CAM_ISP_RAM_1.8V off            LP8720 LDO 4, time step : 100us */
        /*Turn CAM_ISP_HOST_1.8V off            LP8720 LDO 3, time step : 150us */
        /*Turn CAM_ISP_SYS_2.8V off            LP8720 LDO 2, time step : 200us*/
        lp8720_i2c_write(0x08, 0xE1);
        mdelay(1);
        /*Turn CAM_SENSOR_CORE_1.2V off        LP8720 LDO 1, time step : 0us */
        lp8720_i2c_write(0x08, 0xE0);
        mdelay(1);
        /*Turn CAM_ISP_CORE_1.2V off            LP8720 buck(SW), time step : 0us */
        lp8720_i2c_write(0x08, 0x00);
        gpio_set_value(2, 0);    
    }
    
    gpio_free(2);
    gpio_free(3);
    gpio_free(31);
    gpio_free(132);
    gpio_free(174);
    gpio_free(175);
    gpio_free(177);
    printk(KERN_ERR "[CAMDRV/CE147] %s: POWER CONTROL END.\n", __func__);
    return rc;
}

static int ce147_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
    int err = 0;
    
    ce147_set_power(1);

    err = ce147_load_fw();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Camera Initialization\n", __func__);
        return -EIO;
    }
    
    ce147_init_parameters();
    
    err = ce147_get_fw_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading firmware version\n",__func__);
        return -EIO;
    }

    err = ce147_get_dateinfo();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading dateinfo\n",__func__);
        return -EIO;
    }
    
    err = ce147_get_sensor_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading sensor info\n",__func__);
        return -EIO;
    }

    err = ce147_get_main_sw_fw_version();
    if(err < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147 :%s: Failed: Reading Main SW Version\n",__func__);
        return -EIO;
    }

    printk(KERN_DEBUG "fw M:%d m:%d |prm M:%d m:%d \n", MAIN_SW_FW[0], MAIN_SW_FW[1], MAIN_SW_FW[2], MAIN_SW_FW[3]);
    printk(KERN_DEBUG "y. m. d = %d.%d.%d \n", MAIN_SW_DATE_INFO[0], MAIN_SW_DATE_INFO[1], MAIN_SW_DATE_INFO[2]);

    printk(KERN_DEBUG "FW  Version: %d.%d\n", ce147_status->fw.major, ce147_status->fw.minor);
    printk(KERN_DEBUG "PRM Version: %d.%d\n", ce147_status->prm.major, ce147_status->prm.minor);
    printk(KERN_DEBUG "Date(y.m.d): %d.%d.%d\n", ce147_status->dateinfo.year, ce147_status->dateinfo.month, ce147_status->dateinfo.date);    
    printk(KERN_DEBUG "Sensor version: %d\n", ce147_status->sensor_version);    

    return err;
}

static int ce147_sensor_init(const struct msm_camera_sensor_info *data)
{
    int err = 0;
    CAMDRV_DEBUG("%s\n", __func__);
    
    ce147_ctrls = kzalloc(sizeof(struct ce147_ctrls_t), GFP_KERNEL);
    if (!ce147_ctrls) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: probe from VFE failed\n", __func__);
        err = -ENOMEM;
        goto init_fail;
    }

    if (data)
        ce147_ctrls->sensordata = data;
    
    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);
    
    msm_camio_camif_pad_reg_reset();
    
    err = ce147_sensor_init_probe(data);
    if (err < 0) {
        CDBG("mt9d112_sensor_init failed!\n");
        goto init_fail;
    }
    
init_done:
    return err;

init_fail:
    kfree(ce147_ctrls);
    return err;
}

static int ce147_set_sensor_mode(int mode, int res)
{
    int rc = 0;

    switch (mode) {
    case SENSOR_PREVIEW_MODE:
        CAMDRV_DEBUG("%s: SENSOR_PREVIEW_MODE\n", __func__);
        rc = ce147_set_preview_start();
        break;
    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:    
        CAMDRV_DEBUG("%s: SENSOR_RAW_SNAPSHOT_MODE\n", __func__);
        rc = ce147_set_capture_config();
        break;
    case SENSOR_SNAPSHOT_TRANSFER:
        CAMDRV_DEBUG("%s: SENSOR_SNAPSHOT_TRANSFER\n", __func__);
        rc = ce147_set_capture_start();
        break;
    default:
        CAMDRV_DEBUG("%s: ERROR!\n", __func__);
        rc = -EINVAL;
        break;
    }
    return rc;
}

bool ce147_get_vgacam_rotated()
{
    bool rotated = true;


    if(ce147_status->prm.major == 5 && ce147_status->prm.minor > 32)
    {
        rotated = true;
    }
    else
    {
        rotated = false;    
    }

    printk(KERN_DEBUG "PRM Version: %d.%d\n", ce147_status->prm.major, ce147_status->prm.minor);
    printk(KERN_DEBUG "ce147_get_vgacam_rotated: %d\n", rotated);    

    return rotated;	
}

int ce147_sensor_ext_config(void __user *argp)
{
    sensor_ext_cfg_data cfg_data;
    int rc = 0;

    if (copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
        return -EFAULT;

    switch(cfg_data.cmd) {

    case EXT_CFG_SET_DTP:
        CAMDRV_DEBUG(" EXT_CFG_SET_DTP\n");
        ce147_status->check_dataline = cfg_data.value_1;
        if(cfg_data.value_1 == 0) {
            ce147_check_dataline_stop();
            cfg_data.value_1 = 2;
        } else if(cfg_data.value_1 == 1) {
            ce147_status->check_dataline = cfg_data.value_1;
            cfg_data.value_1 = 3;
        }
        break;        
    case EXT_CFG_DUMP_FIRMWARE:
        CAMDRV_DEBUG(" EXT_CFG_DUMP_FIRMWARE (%d %d)\n",cfg_data.value_1,cfg_data.value_2);
        ce147_status->fw_info.addr = cfg_data.value_1;
        ce147_status->fw_info.size = cfg_data.value_2;
        rc = ce147_dump_fw();
        cfg_data.value_1 = ce147_status->fw_dump_size;
        break;        
    case EXT_CFG_UPDATE_FIRMWARE:
        CAMDRV_DEBUG(" EXT_CFG_UPDATE_FIRMWARE (%d %d)\n",cfg_data.value_1,cfg_data.value_2);
        ce147_status->fw_info.addr = cfg_data.value_1;
        ce147_status->fw_info.size = cfg_data.value_2;
        rc = ce147_update_fw();
        break;            
    case EXT_CFG_GET_JPEG_SIZE:
        CAMDRV_DEBUG(" EXT_CFG_GET_JPEG_SIZE (%d)\n",ce147_status->jpeg.main_size);
        cfg_data.value_1 = ce147_status->jpeg.main_size;
        break;            
    case EXT_CFG_SET_AE_AWB:
        CAMDRV_DEBUG(" EXT_CFG_SET_AE_AWB (%d)\n",cfg_data.value_1);
        rc = ce147_set_ae_awb(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_FLASH:
        CAMDRV_DEBUG(" EXT_CFG_SET_FLASH (%d)\n",cfg_data.value_1);
        //rc = ce147_set_flash(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_SCENE:
        CAMDRV_DEBUG(" EXT_CFG_SET_SCENE (%d)\n",cfg_data.value_1);
        rc = ce147_set_scene_mode(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_EFFECT:
        CAMDRV_DEBUG(" EXT_CFG_SET_EFFECT (%d)\n",cfg_data.value_1);
        rc = ce147_set_effect(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_SHARPNESS:
        CAMDRV_DEBUG(" EXT_CFG_SET_SHARPNESS (%d)\n",cfg_data.value_1);
        rc = ce147_set_sharpness(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_SATURATION:
        CAMDRV_DEBUG(" EXT_CFG_SET_SATURATION (%d)\n",cfg_data.value_1);
        rc = ce147_set_saturation(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_ISO:
        CAMDRV_DEBUG(" EXT_CFG_SET_ISO (%d)\n",cfg_data.value_1);
        rc = ce147_set_iso(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_WB:
        CAMDRV_DEBUG(" EXT_CFG_SET_WB (%d)\n",cfg_data.value_1);
        rc = ce147_set_white_balance(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_CONTRAST:
        CAMDRV_DEBUG(" EXT_CFG_SET_CONTRAST (%d)\n",cfg_data.value_1);
        rc = ce147_set_contrast(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_BRIGHTNESS:
        CAMDRV_DEBUG(" EXT_CFG_SET_BRIGHTNESS (%d)\n",cfg_data.value_1);
        //rc = ce147_set_ev(cfg_data.value_1);
        ce147_set_ev(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_ZOOM:
        CAMDRV_DEBUG(" EXT_CFG_SET_ZOOM (%d)\n",cfg_data.value_1);
        rc = ce147_set_dzoom(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_FPS:
        CAMDRV_DEBUG(" EXT_CFG_SET_FPS (%d)\n",cfg_data.value_1);
        ce147_status->fps=cfg_data.value_1;
        break;    
    case EXT_CFG_SET_FPS_MODE:
        CAMDRV_DEBUG(" EXT_CFG_SET_FPS_MODE (%d)\n",cfg_data.value_1);
        ce147_status->fps_mode=cfg_data.value_1;
        break;  
    case EXT_CFG_SET_AF_MODE:
        CAMDRV_DEBUG(" EXT_CFG_SET_AF_MODE (%d)\n",cfg_data.value_1);
        rc = ce147_set_focus_mode(cfg_data.value_1);
        ce147_status->focus_mode = cfg_data.value_1;
        break;        
    case EXT_CFG_SET_AF_START:
        CAMDRV_DEBUG(" EXT_CFG_SET_AF_START (%d)\n",cfg_data.value_1);
        rc = ce147_set_auto_focus(1);
        break;
    case EXT_CFG_SET_AF_STOP:
        CAMDRV_DEBUG(" EXT_CFG_SET_AF_STOP (%d)\n",cfg_data.value_1);
        rc = ce147_set_auto_focus(0);
        break;        
    case EXT_CFG_GET_AF_STATUS:
        CAMDRV_DEBUG(" EXT_CFG_GET_AF_STATUS (%d)\n",cfg_data.value_1);
        rc = ce147_get_auto_focus_status(&cfg_data.value_1);
        break;        
    case EXT_CFG_SET_FACE_DETECT:
        CAMDRV_DEBUG(" EXT_CFG_SET_FACE_DETECT (%d)\n",cfg_data.value_1);
        rc = ce147_set_face_detection(cfg_data.value_1);
        ce147_status->focus_mode = FOCUS_MODE_FD;
        break;        
    case EXT_CFG_SET_METERING:
        CAMDRV_DEBUG(" EXT_CFG_SET_METERING (%d)\n",cfg_data.value_1);
        rc = ce147_set_metering(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_CONTINUOUS_AF:
        CAMDRV_DEBUG(" EXT_CFG_SET_CONTINUOUS_AF (%d)\n",cfg_data.value_1);
        rc = ce147_set_continuous_af(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_PREVIEW_SIZE:
        CAMDRV_DEBUG(" EXT_CFG_SET_PREVIEW_SIZE (%d)\n",cfg_data.value_1);
        ce147_status->preview_size = cfg_data.value_1;
        break;        
    case EXT_CFG_SET_PICTURE_SIZE:
        CAMDRV_DEBUG(" EXT_CFG_SET_PICTURE_SIZE (%d,%d)\n",cfg_data.value_1,cfg_data.value_2);
        if(cfg_data.value_1 == 640 && cfg_data.value_2 == 480)ce147_status->framesize_index = CE147_CAPTURE_VGA;
        else if(cfg_data.value_1 == 800 && cfg_data.value_2 == 480)ce147_status->framesize_index = CE147_CAPTURE_WVGA;
        else if(cfg_data.value_1 == 1600 && cfg_data.value_2 == 960)ce147_status->framesize_index = CE147_CAPTURE_W1MP;
        else if(cfg_data.value_1 == 1600 && cfg_data.value_2 == 1200)ce147_status->framesize_index = CE147_CAPTURE_2MP;
        else if(cfg_data.value_1 == 2048 && cfg_data.value_2 == 1232)ce147_status->framesize_index = CE147_CAPTURE_W2MP;
        else if(cfg_data.value_1 == 2048 && cfg_data.value_2 == 1536)ce147_status->framesize_index = CE147_CAPTURE_3MP;
        else if(cfg_data.value_1 == 2560 && cfg_data.value_2 == 1536)ce147_status->framesize_index = CE147_CAPTURE_W4MP;
        else if(cfg_data.value_1 == 2560 && cfg_data.value_2 == 1920)ce147_status->framesize_index = CE147_CAPTURE_5MP;
        else CAMDRV_DEBUG(" EXT_CFG_SET_PICTURE_SIZE -invalid size.\n");
        break;        
    case EXT_CFG_SET_JPEG_QUALITY:
        CAMDRV_DEBUG(" EXT_CFG_SET_JPEG_QUALITY (%d)\n",cfg_data.value_1);
        ce147_status->jpeg.quality = cfg_data.value_1;
        break;        
    case EXT_CFG_SET_TOUCHAF_POS:
        CAMDRV_DEBUG(" EXT_CFG_SET_TOUCHAF_POS (%d %d)\n",cfg_data.value_1,cfg_data.value_2);
        ce147_status->position.x = cfg_data.value_1;
        ce147_status->position.y = cfg_data.value_2;
        break;        
    case EXT_CFG_SET_ANTISHAKE:
        CAMDRV_DEBUG(" EXT_CFG_SET_ANTISHAKE (%d)\n",cfg_data.value_1);
        rc = ce147_set_anti_shake(cfg_data.value_1);
        break;        
    case EXT_CFG_SET_WDR:
        CAMDRV_DEBUG(" EXT_CFG_SET_WDR (%d)\n",cfg_data.value_1);
        rc = ce147_set_wdr(cfg_data.value_1);
        break;
    case EXT_CFG_SET_BEAUTY:
        CAMDRV_DEBUG(" EXT_CFG_SET_BEAUTY (%d)\n",cfg_data.value_1);
        rc = ce147_set_face_beauty(cfg_data.value_1);
        break;    
    case EXT_CFG_SET_VINTAGEMODE:
        CAMDRV_DEBUG(" EXT_CFG_SET_VINTAGEMODE (%d)\n",cfg_data.value_1);
        rc = ce147_set_vintage_mode(cfg_data.value_1);
        break;            
    case EXT_CFG_SET_EXIF:
        CAMDRV_DEBUG(" EXT_CFG_SET_EXIF (%d)\n",cfg_data.value_1);
        //rc = ce147_set_exif(cfg_data.value_1);
        break;  
    case EXT_CFG_SET_THUMB_NULL:
        CAMDRV_DEBUG(" EXT_CFG_SET_THUMBNAIL_ONOFF (%d)\n",cfg_data.value_1);
        ce147_status->thumb_null = cfg_data.value_1;
        break;
    case EXT_CFG_SET_EXIF_ORIENTATION_INFO:
        CAMDRV_DEBUG(" EXT_CFG_SET_EXIF_ORIENTATION_INFO (%d)\n",cfg_data.value_1);
        ce147_status->exif_orientation_info = cfg_data.value_1;
        break;
    case EXT_CFG_SET_GPS_LATITUDE:
    case EXT_CFG_SET_GPS_LONGITUDE:
    case EXT_CFG_SET_GPS_ALTITUDE:
    case EXT_CFG_SET_GPS_TIMESTAMP:
    case EXT_CFG_SET_EXIF_TIME_INFO:
    case EXT_CFG_SET_GPS_PROCESSINGMETHOD:
        CAMDRV_DEBUG(" EXT_CFG_SET_EXIF (%d)\n",cfg_data.cmd);
        rc = ce147_set_gps_info(cfg_data.cmd, cfg_data.p);
        break;  
    case EXT_CFG_SET_BATCH_REFLECTION:
        CAMDRV_DEBUG(" EXT_CFG_SET_BATCH_REFLECTION\n");
        rc = ce147_get_batch_reflection_status();
        break;    
    case EXT_CFG_GET_VGACAM_ROTATED: // kurtlee
        cfg_data.value_1 = (int)ce147_get_vgacam_rotated();
        CAMDRV_DEBUG(" EXT_CFG_GET_CAMETA_ROTATED (%d)\n",cfg_data.value_1);
        break;
    case EXT_CFG_TEST_ESD:
        CAMDRV_DEBUG(" EXT_CFG_TEST_ESD\n");		
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: EXT_CFG_TEST_ESD\n", __func__);
		
        ce147_set_auto_focus(0);
        ce147_set_power(false);
        mdelay(500);		
        msm_camio_clk_rate_set(24000000);
        mdelay(5);
        msm_camio_camif_pad_reg_reset();	  
        mdelay(5);
        ce147_set_power(true);		
        ce147_load_fw();
	  ce147_set_preview_start();
	  msleep(500);
	  msm_camio_camif_pad_reg_reset_2();
	  break;	  
    default:
        CAMDRV_DEBUG(" EXT_CFG_SET, Invailid command (%d)\n",cfg_data.cmd);
        break;
    }

    if(copy_to_user((void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
        printk(" %s : copy_to_user Failed \n", __func__);
    
    return rc;
}

static int ce147_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    int rc = 0;

    if (copy_from_user(&cdata,
        (void *)argp,
        sizeof(struct sensor_cfg_data)))
        return -EFAULT;
    
    CAMDRV_DEBUG("ce147_sensor_config: cfgtype = %d\n", cdata.cfgtype);
    
    switch (cdata.cfgtype) {
    case CFG_SET_MODE:
        rc = ce147_set_sensor_mode(cdata.mode, cdata.rs);
        break;
    default:
        break;
    }
    return 0;
}

static int ce147_sensor_release(void)
{
    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    ce147_set_af_softlanding();
    ce147_set_power(0);
    kfree(ce147_ctrls);
    //kfree(ce147_status);
    ce147_ctrls = NULL;
    //ce147_status = NULL;
    return 0;
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

static int ce147_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int rc = 0;

    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: ce147 i2c probe failed\n", __func__);
        return -1;
    }

    //rc = sysfs_create_file(&client->dev.kobj, &dev_attr_ce147_firmware.attr);
    //if (rc)
        //printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: sysfs create entry failed\n", __func__);

    ce147_i2c_client = client;
    CAMDRV_DEBUG("ce147 i2c probe successful\n");
    return 0;
}

static int ce147_i2c_remove(struct i2c_client *client)
{
    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    //sysfs_remove_file(&client->dev.kobj, &dev_attr_ce147_firmware.attr);
    ce147_i2c_client = NULL;
    return 0;
}

static const struct i2c_device_id ce147_id[] = {
    {"ce147", 0},
    { }
};

static struct i2c_driver ce147_i2c_driver = {
    .id_table    = ce147_id,
    .probe      = ce147_i2c_probe,
    .remove     = ce147_i2c_remove,
    .driver     = {
        .name = "ce147",
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
        sensorname = "LSI_S5K4ECGX_CE147";
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

static int ce147_sensor_probe(const struct msm_camera_sensor_info *info,
        struct msm_sensor_ctrl *s)
{
    int rc = 0;

    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    rc = i2c_add_driver(&ce147_i2c_driver);
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: sensor probe failed\n", __func__);
        goto probe_fail0;
    }

    rc = i2c_add_driver(&lp8720_i2c_driver);
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: lp8720 probe failed\n", __func__);
        goto probe_fail1;
    }
    
    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);
    
    /* Check whether sensor is present and its powering on */
    //ce147_sensor_init(NULL);
    ce147_set_power(1);
    rc = ce147_load_fw();
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/CE147] CE147: %s: ce147_load_fw failed\n", __func__);
        goto probe_fail1;
    }
    ce147_get_fw_version();	
    vgacam_rotated = ce147_get_vgacam_rotated();
	
    /* Power off the sensor */
    ce147_set_power(0);

    s->s_init = ce147_sensor_init;
    s->s_release = ce147_sensor_release;
    s->s_config  = ce147_sensor_config;
    s->s_camera_type = BACK_CAMERA_2D;
    s->s_mount_angle  = 0;

    lp8720_init = true;
    CAMDRV_DEBUG("ce147 sensor probe successful\n");
    return rc;

probe_fail1:
    i2c_del_driver(&lp8720_i2c_driver);
probe_fail0:
    i2c_del_driver(&ce147_i2c_driver);
    printk(KERN_ERR "[CAMDRV/CE147] CE147: %s:probe failed\n", __func__);
    lp8720_init = false;
    return rc;
}

static int __ce147_probe(struct platform_device *pdev)
{
    ce147_status = kzalloc(sizeof(struct ce147_status_t), GFP_KERNEL);
    if (ce147_status == NULL)
        return -ENOMEM;
    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);    
    ce147_status->runmode = CE147_RUNMODE_NOTREADY;
    ce147_status->pre_af_status = -1;
    ce147_status->cur_af_status = -2;
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
    
    return msm_camera_drv_start(pdev, ce147_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __ce147_probe,
    .driver = {
        .name = "msm_camera_ce147",
        .owner = THIS_MODULE,
    },
};

static int __init ce147_init(void)
{
    printk(KERN_INFO "[CAMDRV/CE147] %s: E\n", __func__);
    return platform_driver_register(&msm_camera_driver);
}

module_init(ce147_init);
MODULE_DESCRIPTION("NEC CE147-NEC 5MP camera driver");
MODULE_LICENSE("GPL v2");
