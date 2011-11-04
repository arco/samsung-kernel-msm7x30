/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : l3g4200d.c
* Authors            : MH - C&I BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version            : V 0.1
* Date               : 29/01/2010
* Description        : L3G4200D digital output gyroscope sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
******************************************************************************/

#define YAMAHA_HAL

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/l3g4200d.h>
#include <linux/delay.h>
#include <linux/slab.h>


#ifdef YAMAHA_HAL
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#endif //YAMAHA_HAL


#define L3G4200D_MAJOR   102
#define L3G4200D_MINOR   4

/* l3g4200d gyroscope registers */
#define WHO_AM_I    	0x0F

#define CTRL_REG1       0x20    /* power control reg */
#define CTRL_REG2       0x21    /* power control reg */
#define CTRL_REG3       0x22    /* power control reg */
#define CTRL_REG4       0x23    /* interrupt control reg */
#define CTRL_REG5       0x24    /* interrupt control reg */
#define OUT_TEMP		0x26	/* Temperature data */
#define AXISDATA_REG    0x28

#define STATUS_REG		0x27	

#define PM_OFF			0x00
#define PM_NORMAL		0x08
#define ENABLE_ALL_AXES	0x07

#define ODR200_BW12_5	0x00  /* ODR = 200Hz; BW = 12.5Hz */
#define ODR200_BW25		0x10  /* ODR = 200Hz; BW = 25Hz   */
#define ODR200_BW50		0x20  /* ODR = 200Hz; BW = 50Hz   */
#define ODR100_BW12_5	0x40  /* ODR = 100Hz; BW = 12.5Hz */
#define ODR100_BW25		0x50  /* ODR = 100Hz; BW = 25Hz   */
#define ODR400_BW25		0x90  /* ODR = 400Hz; BW = 25Hz   */
#define ODR400_BW50		0xA0  /* ODR = 400Hz; BW = 50Hz   */
#define ODR400_BW110	0xB0  /* ODR = 400Hz; BW = 110Hz  */
#define ODR800_BW50		0xE0  /* ODR = 800Hz; BW = 50Hz   */
#define ODR800_BW100	0xF0  /* ODR = 800Hz; BW = 100Hz  */

#define MIN_ST			175
#define MAX_ST			875

/*#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2*/

#define DEBUG 1

#ifdef YAMAHA_HAL
#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)
#endif //YAMAHA_HAL


static unsigned char reg_backup[5];
static unsigned int fd_cnt = 0;

extern unsigned int HWREV;

/*
 * L3G4200D gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */

struct l3g4200d_t {
	short	y,	/* yaw data. Range -2048 to 2047. */
		p,	/* pitch data. Range -2048 to 2047. */
		r;	/* roll data. Range -2048 to 2047. */
};

/* static struct i2c_client *l3g4200d_client; */

struct l3g4200d_data {
	struct i2c_client *client;
	struct l3g4200d_platform_data *pdata;
	/* u8 shift_adj; */
#ifdef YAMAHA_HAL
    struct delayed_work work;
    struct mutex enable_mutex;
    struct l3g4200d_t *last;
    atomic_t enabled;
    atomic_t delay;
    char mode; //pm_status
#endif //YAMAHA_HAL
};

static struct l3g4200d_data *gyro;
#ifdef YAMAHA_HAL
static struct input_dev *this_data = NULL;
#endif //YAMAHA_HAL

static struct class *l3g_gyro_dev_class;

static char l3g4200d_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char l3g4200d_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

/* set l3g4200d digital gyroscope bandwidth */
int l3g4200d_set_bandwidth(char bw)
{
	int res = 0;
	unsigned char data = 0;

	res = i2c_smbus_read_word_data(gyro->client, CTRL_REG1);
	if (res >= 0)
	{
		data = res & 0x00cf;

	data = data + bw;
	res = l3g4200d_i2c_write(CTRL_REG1, &data, 1);
	}
	return res;
}

/* read selected bandwidth from l3g4200d */
int l3g4200d_get_bandwidth(unsigned char *bw)
{
	int res = 1;
	/* TO DO */
	return res;
}

int l3g4200d_set_mode(char mode)
{
	int res = 0;
	unsigned char data = 0;

    gyro->mode = mode;

	res = i2c_smbus_read_word_data(gyro->client, CTRL_REG1);
	if (res >= 0)
	{
		data = res & 0x00f7;

	data = mode + data;
	/* printk(KERN_INFO "set mode CTRL_REG1=%x\n",data); */
	res = l3g4200d_i2c_write(CTRL_REG1, &data, 1);
	}
	return res;
}

int l3g4200d_set_range(char range)
{
	int res = 0;
	unsigned char data = 0;

	res = i2c_smbus_read_word_data(gyro->client, CTRL_REG4);
	if (res >= 0)
	{
		data = res & 0x00cf;
		
	data = range + data;	
	res = l3g4200d_i2c_write(CTRL_REG4, &data, 1);
	}
	return res;
}

/* gyroscope data readout */
int l3g4200d_read_gyro_values(struct l3g4200d_t *data)
{
	int res;
	unsigned char gyro_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	res = l3g4200d_i2c_read(AXISDATA_REG, &gyro_data[0], 6);

	hw_d[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
	hw_d[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
	hw_d[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);

	/* hw_d[0] >>= gyro->shift_adj;
	hw_d[1] >>= gyro->shift_adj;
	hw_d[2] >>= gyro->shift_adj; */
#if 0
	data->y = ((gyro->pdata->negate_x) ? (-hw_d[gyro->pdata->axis_map_x])
		   : (hw_d[gyro->pdata->axis_map_x]));
	data->p = ((gyro->pdata->negate_y) ? (-hw_d[gyro->pdata->axis_map_y])
		   : (hw_d[gyro->pdata->axis_map_y]));
	data->r = ((gyro->pdata->negate_z) ? (-hw_d[gyro->pdata->axis_map_z])
		   : (hw_d[gyro->pdata->axis_map_z]));
#else
	gyro->pdata->negate_x = 0;
	gyro->pdata->negate_y = 0;
	gyro->pdata->negate_z = 0;
	data->y = ((gyro->pdata->negate_x) ? (-hw_d[2])
		   : (hw_d[2]));
	data->p = ((gyro->pdata->negate_y) ? (-hw_d[1])
		   : (hw_d[1]));
	data->r = ((gyro->pdata->negate_z) ? (-hw_d[0])
		   : (hw_d[0]));
#endif

	//printk(KERN_INFO "%s : read y=%d, p=%d, r=%d\n", __func__, data->y, data->p, data->r);
	return res;
}


/* Device Initialization  */
static int device_init(void)
{
	int res;
	unsigned char buf[5];
	buf[0] = 0x6f;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	res = l3g4200d_i2c_write(CTRL_REG1, &buf[0], 5);
	return res;
}

/*  i2c write routine for l3g4200d digital gyroscope */
static char l3g4200d_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int dummy;
	int i;

	if (gyro->client == NULL)  /*  No global client pointer? */
		return -1;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(gyro->client,
						  reg_addr++, data[i]);
		if (dummy) {
			printk(KERN_INFO "i2c write error\n");
			return dummy;
		}
	}
	return 0;
}

/*  i2c read routine for l3g4200d digital gyroscope */
static char l3g4200d_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	int dummy = 0;
	int i = 0;

	if (gyro->client == NULL)  /*  No global client pointer? */
		return -1;
	while (i < len) {
		dummy = i2c_smbus_read_word_data(gyro->client, reg_addr++);
		if (dummy >= 0) {
			data[i] = dummy & 0x00ff;
			/*printk("data[%d]=%x",i,dummy);*/
			i++;
		} else {
			printk(KERN_INFO" i2c read error\n ");
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}

/*  read command for l3g4200d device file  */
static ssize_t l3g4200d_read(struct file *file, char __user *buf,
				  size_t count, loff_t *offset)
{
	#if DEBUG
	struct l3g4200d_t data;
	#endif
	if (gyro->client == NULL)
		return -1;
	#if DEBUG
	l3g4200d_read_gyro_values(&data);
	printk(KERN_INFO "Yaw axis: %d\n", data.y);
	printk(KERN_INFO "Pitch axis: %d\n", data.p);
	printk(KERN_INFO "Roll axis: %d\n", data.r);
	#endif
	return 0;
}

/*  write command for l3g4200d device file */
static ssize_t l3g4200d_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *offset)
{
	if (gyro->client == NULL)
		return -1;
	#if DEBUG
	printk(KERN_INFO "l3g4200d should be accessed with ioctl command\n");
	#endif
	return 0;
}

/*************************************************************************/
/*					Start of L3G4200D Sysfs	  							 */
/*************************************************************************/
//TEST
//#define POWER_ON_TEST
static ssize_t l3g4200d_power_on(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res;
#ifdef POWER_ON_TEST
	unsigned char gyro_data[6] = {0,0,0,0,0,0};
	unsigned char bZYXDA = 0;
	unsigned char temp[1] = {0};
	short raw[3] = {0,0,0};	
#endif
	
	res = device_init();
	
	mdelay(300);
	
#ifdef POWER_ON_TEST	// For Test
	// check ZYXDA ready bit
	l3g4200d_i2c_read(STATUS_REG, &temp[0], 1);
	bZYXDA = ((unsigned int)temp[0] & 0x08) >> 3;		
	
	printk("[l3g4200d_power_on] temp = 0x%x, bZYXDA = %d\n", temp[0], bZYXDA);
		
	res = l3g4200d_i2c_read(AXISDATA_REG, &gyro_data[0], 6);

	raw[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
	raw[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
	raw[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);	

	printk("[l3g4200d_power_on] raw[0] = %d\n", raw[0]);
	printk("[l3g4200d_power_on] raw[1] = %d\n", raw[1]);
	printk("[l3g4200d_power_on] raw[2] = %d\n", raw[2]);
	printk("\n");
#endif
	
	printk("[%s] result of device init = %d\n", __func__, res);
	
	count = sprintf(buf,"%d\n", (res < 0 ? 0:1));

	return count;
}

static ssize_t l3g4200d_get_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;	
	char temp = 0;
	
	// before starting self-test, backup register
	l3g4200d_i2c_read(OUT_TEMP, &temp, 1);
	
	printk("[%s] read temperature : %d\n", __func__, temp);
	
	count = sprintf(buf,"%d\n", temp);

	return count;
}

static ssize_t l3g4200d_self_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res;	
	unsigned char temp[1] = {0};
	unsigned char gyro_data[6] = {0,0,0,0,0,0};
	short raw[3] = {0,0,0};
	int NOST[3], ST[3];
	int differ_x = 0, differ_y = 0, differ_z = 0;
	unsigned char bak_reg[5] = {0,0,0,0,0};
	unsigned char reg[5] = {0,0,0,0,0};
	unsigned char bZYXDA = 0;
	unsigned char pass = 0;
	int i = 0;
	int fail_count = 0;
	
	memset(NOST, 0, sizeof(int)*3);
	memset(ST, 0, sizeof(int)*3);
	
	// before starting self-test, backup register
	l3g4200d_i2c_read(CTRL_REG1, &bak_reg[0], 5);
	
	for(i = 0; i < 5; i++)
		printk("[gyro_self_test] backup reg[%d] = %2x\n", i, bak_reg[i]);
	
	// Initialize Sensor, turn on sensor, enable P/R/Y
	// Set BDU=1, Set ODR=200Hz, Cut-Off Frequency=50Hz, FS=2000dps
	reg[0] = 0x6f;
	reg[1] = 0x00;
	reg[2] = 0x00;
	reg[3] = 0xA0;
	reg[4] = 0x02;

	l3g4200d_i2c_write(CTRL_REG1, &reg[0], 5);
	
	// Power up, wait for 800ms for stable output
	mdelay(800);
		
	// Read 5 samples output before self-test on
	i = 0;
	fail_count = 0;
	while(i < 5)
	{
		// check ZYXDA ready bit
		l3g4200d_i2c_read(STATUS_REG, &temp[0], 1);
		bZYXDA = ((unsigned int)temp[0] & 0x08) >> 3;		
		
		if(!bZYXDA)
		{
			fail_count++;
			mdelay(100);
			if(fail_count < 10)
				continue;
			else
				goto exit;
		}
		
		res = l3g4200d_i2c_read(AXISDATA_REG, &gyro_data[0], 6);

		raw[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
		raw[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
		raw[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);
		
		NOST[0] += raw[0];
		NOST[1] += raw[1];
		NOST[2] += raw[2];
		
		printk("[gyro_self_test] raw[0] = %d\n", raw[0]);
		printk("[gyro_self_test] raw[1] = %d\n", raw[1]);
		printk("[gyro_self_test] raw[2] = %d\n", raw[2]);
		printk("\n");
		
		i++;
	}	
	
	for(i = 0; i < 3; i++)
		printk("[gyro_self_test] SUM of NOST[%d] = %d\n", i, NOST[i]);
	
	// calculate average of NOST and covert from ADC to DPS
	for(i = 0; i < 3; i++)
	{
		NOST[i] = (NOST[i] / 5) * 70 / 1000;
		printk("[gyro_self_test] AVG of NOST[%d] = %d\n", i, NOST[i]);
	}
	printk("\n");
	
	// Enable Self Test
	reg[0] = 0xA2;
	l3g4200d_i2c_write(CTRL_REG4, &reg[0], 1);	
	
	mdelay(100);
	
	// Read 5 samples output after self-test on
	i = 0;
	fail_count = 0;
	while(i < 5)
	{
		// check ZYXDA ready bit
		l3g4200d_i2c_read(STATUS_REG, &temp[0], 1);
		bZYXDA = ((unsigned int)temp[0] & 0x08) >> 3;		
		
		if(!bZYXDA)
		{
			fail_count++;
			mdelay(100);
			if(fail_count < 10)
				continue;
			else
				goto exit;
		}
		
		res = l3g4200d_i2c_read(AXISDATA_REG, &gyro_data[0], 6);

		raw[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
		raw[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
		raw[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);
		
		ST[0] += raw[0];
		ST[1] += raw[1];
		ST[2] += raw[2];
		
		printk("[gyro_self_test] raw[0] = %d\n", raw[0]);
		printk("[gyro_self_test] raw[1] = %d\n", raw[1]);
		printk("[gyro_self_test] raw[2] = %d\n", raw[2]);
		printk("\n");
		
		i++;
	}	
	
	for(i = 0; i < 3; i++)
		printk("[gyro_self_test] SUM of ST[%d] = %d\n", i, ST[i]);
	
	// calculate average of ST and convert from ADC to dps
	for(i = 0; i < 3; i++)
	{
		ST[i] = (ST[i] / 5) * 70 / 1000; // When FS=2000, 70 mdps/digit
		printk("[gyro_self_test] AVG of ST[%d] = %d\n", i, ST[i]);
	}	
		
	// check whether pass or not
	if( ST[0] >= NOST[0] )  // for x
		differ_x = ST[0] - NOST[0];
	else
		differ_x = NOST[0] - ST[0];
	
	if( ST[1] >= NOST[1] )  // for y
		differ_y = ST[1] - NOST[1];
	else
		differ_y = NOST[1] - ST[1];
		
	if( ST[2] >= NOST[2] )  // for z
		differ_z = ST[2] - NOST[2];
	else
		differ_z = NOST[2] - ST[2];
		
	printk("[gyro_self_test] differ x:%d, y:%d, z:%d\n", differ_x, differ_y, differ_z); 
		
	if( (MIN_ST <= differ_x && differ_x <= MAX_ST) && (MIN_ST <= differ_y && differ_y <= MAX_ST) &&
		(MIN_ST <= differ_z && differ_z <= MAX_ST) )
		pass = 1;	

exit:		
	// restore backup register
	l3g4200d_i2c_write(CTRL_REG1, &bak_reg[0], 5);
	
	printk("[gyro_self_test] self-test result : %s\n", pass ? "pass" : "fail");
	count = sprintf(buf,"%d,%d,%d,%d,%d,%d,%d\n", NOST[0], NOST[1], NOST[2], ST[0], ST[1], ST[2], pass);

	return count;
}

static ssize_t l3g4200d_fs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//buf[size]=0;
	printk("input data --> %s\n", buf);

	return size;
}

#ifdef YAMAHA_HAL

static ssize_t
l3g4200d_sensor_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int delay = atomic_read(&gyro->delay);

    printk(KERN_INFO "%s : (%d)\n", __func__, delay);

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
l3g4200d_sensor_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int value = simple_strtoul(buf, NULL, 10);
    int enabled = atomic_read(&gyro->enabled);

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    atomic_set(&gyro->delay, value);

    mutex_lock(&gyro->enable_mutex);

    if (enabled) {
        cancel_delayed_work_sync(&gyro->work);
        schedule_delayed_work(&gyro->work, msecs_to_jiffies(value));
    } else {
        //power_up();
        //set some values
        //power_down();
    }

    mutex_unlock(&gyro->enable_mutex);

    input_report_abs(this_data, ABS_CONTROL_REPORT, (enabled<<16) | value);

    printk(KERN_INFO "%s : (%d)\n", __func__, value);

    return count;
}

static ssize_t
l3g4200d_sensor_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int enabled = atomic_read(&gyro->enabled);

    printk(KERN_INFO "%s : (%d)\n", __func__, enabled);

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
l3g4200d_sensor_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int value = simple_strtoul(buf, NULL, 10);
    int enabled = atomic_read(&gyro->enabled);
    int delay = atomic_read(&gyro->delay);

    if (value != 0 && value != 1) {
        return count;
    }

    mutex_lock(&gyro->enable_mutex);

    if (enabled && !value) {
        cancel_delayed_work_sync(&gyro->work);
        //suspend();  //power off
        fd_cnt--;
        if(fd_cnt == 0){
          l3g4200d_set_mode(PM_OFF);
          printk(KERN_INFO "%s : PM_OFF(%d, %d)\n", __func__, enabled, value);
        }
        printk(KERN_INFO "%s : timer canceled(%d, %d)\n", __func__, enabled, value);
    }
    if (!enabled && value) {
        //resume(); //power on
        device_init();
        mdelay(300);
        fd_cnt++;
        l3g4200d_set_range(L3G4200D_FS_500DPS);
        l3g4200d_set_mode(PM_NORMAL);
        schedule_delayed_work(&gyro->work, 0);
        printk(KERN_INFO "%s : start(%d, %d)\n", __func__, enabled, value);
    }
    atomic_set(&gyro->enabled, value);

    input_report_abs(this_data, ABS_CONTROL_REPORT, (value<<16) | delay);

    mutex_unlock(&gyro->enable_mutex);

    return count;
}

static ssize_t
l3g4200d_sensor_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    static int cnt = 1;

    input_report_abs(this_data, ABS_WAKE, cnt++);

    printk(KERN_INFO "%s\n", __func__);

    return count;
}

static ssize_t
l3g4200d_sensor_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned long flags;
    int x, y, z;

    spin_lock_irqsave(&this_data->event_lock, flags);

    x = this_data->abs[ABS_X];
    y = this_data->abs[ABS_Y];
    z = this_data->abs[ABS_Z];

    spin_unlock_irqrestore(&this_data->event_lock, flags);

    printk(KERN_INFO "%s : (%d, %d, %d)\n", __func__, x, y, z);

    return sprintf(buf, "%d %d %d\n", x, y, z);
}

static ssize_t
l3g4200d_sensor_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned long flags;
   // int status;
   char status;

/*    spin_lock_irqsave(&this_data->event_lock, flags);

    status = this_data->abs[ABS_STATUS];

    spin_unlock_irqrestore(&this_data->event_lock, flags);*/

    mutex_lock(&gyro->enable_mutex);
    status = gyro->mode;
    mutex_unlock(&gyro->enable_mutex);

    printk(KERN_INFO "%s : mode(%d)\n", __func__, status);

    return sprintf(buf, "%d\n", status);
}
#endif //YAMAHA_HAL


static DEVICE_ATTR(gyro_power_on, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, l3g4200d_power_on, NULL);
static DEVICE_ATTR(gyro_get_temp, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, l3g4200d_get_temp, NULL);
static DEVICE_ATTR(gyro_selftest, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, l3g4200d_self_test, l3g4200d_fs_write);

#ifdef YAMAHA_HAL
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP, l3g4200d_sensor_delay_show, l3g4200d_sensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, l3g4200d_sensor_enable_show, l3g4200d_sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, l3g4200d_sensor_wake_store);
static DEVICE_ATTR(data, S_IRUGO, l3g4200d_sensor_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, l3g4200d_sensor_status_show, NULL);

static struct attribute *l3g4200d_attributes[] = {
    &dev_attr_gyro_power_on.attr,
    &dev_attr_gyro_get_temp.attr,
    &dev_attr_gyro_selftest.attr,
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    NULL
};

static struct attribute_group l3g4200d_attribute_group = {
    .attrs = l3g4200d_attributes
};
#endif

/*************************************************************************/
/*					End of L3G4200D Sysfs	  							 */
/*************************************************************************/

/*  open command for l3g4200d device file  */
static int l3g4200d_open(struct inode *inode, struct file *file)
{
	if (gyro->client == NULL) {
		#if DEBUG
		printk(KERN_ERR "I2C driver not install\n");
		#endif
		return -1;
	}
	device_init();
	
	mdelay(300);
	
	fd_cnt++;

	#if DEBUG
	printk(KERN_INFO "l3g4200d has been opened\n");
	#endif
	return 0;
}

/*  release command for l3g4200d device file */
static int l3g4200d_close(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_INFO "L3G4200D has been closed\n");
#endif
	fd_cnt--;
	
#if DEBUG
	printk(KERN_INFO "[%s] fd count = %d\n", __func__, fd_cnt);
#endif
	if(fd_cnt == 0)
	{
#if DEBUG
		printk(KERN_INFO "L3G4200D has been powered off.\n");
#endif
		
		l3g4200d_set_mode(PM_OFF);
	}
		
	return 0;
}


/*  ioctl command for l3g4200d device file */
static int l3g4200d_ioctl(struct inode *inode, struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check l3g4200d_client */
	if (gyro->client == NULL) {
		#if DEBUG
		printk(KERN_ERR "I2C driver not install\n");
		#endif
		return -EFAULT;
	}

	/* cmd mapping */

	switch (cmd) {

	/*case L3G4200D_SELFTEST:
	//TO DO
	return err;*/

	case L3G4200D_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = l3g4200d_set_range(*data);
		return err;

	case L3G4200D_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		err = l3g4200d_set_mode(*data);
		return err;

	case L3G4200D_SET_BANDWIDTH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = l3g4200d_set_bandwidth(*data);
		return err;

	case L3G4200D_READ_GYRO_VALUES:
		err = l3g4200d_read_gyro_values(
				(struct l3g4200d_t *)data);

		if (copy_to_user((struct l3g4200d_t *)arg,
				 (struct l3g4200d_t *)data, 6) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	default:
		return 0;
	}
}


static const struct file_operations l3g4200d_fops = {
	.owner = THIS_MODULE,
	.read = l3g4200d_read,
	.write = l3g4200d_write,
	.open = l3g4200d_open,
	.release = l3g4200d_close,
	.ioctl = l3g4200d_ioctl,
};


static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	if (gyro->pdata->axis_map_x > 2 ||
	    gyro->pdata->axis_map_y > 2 ||
	    gyro->pdata->axis_map_z > 2) {
		dev_err(&gyro->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			gyro->pdata->axis_map_x, gyro->pdata->axis_map_y,
			gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	printk(KERN_INFO "%s : check map!!\n", __func__);

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
	    gyro->pdata->negate_y > 1 ||
	    gyro->pdata->negate_z > 1) {
		dev_err(&gyro->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			gyro->pdata->negate_x, gyro->pdata->negate_y,
			gyro->pdata->negate_z);
		//TO be fixed - start
		//return -EINVAL;
		gyro->pdata->negate_x = 0;
		dev_err(&gyro->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			gyro->pdata->negate_x, gyro->pdata->negate_y,
			gyro->pdata->negate_z);
		//TO be fixed - end

	}

	printk(KERN_INFO "%s : check negate!!\n", __func__);

	return 0;
}

#ifdef YAMAHA_HAL
static void l3g4200d_work_func(struct work_struct *work)
{
    struct l3g4200d_t values;
    int err = 0;
    int delay = atomic_read(&gyro->delay);

    err = l3g4200d_read_gyro_values(&values);
    if(err < 0)
      printk(KERN_ERR "l3g4200d_read_gyro_values() return err %d", err);

    input_report_abs(this_data, ABS_X, values.y);
    input_report_abs(this_data, ABS_Y, values.p);
    input_report_abs(this_data, ABS_Z, values.r);
    input_sync(this_data);

    schedule_delayed_work(&gyro->work, msecs_to_jiffies(delay));

    //printk(KERN_INFO "%s\n", __func__);
}
#endif //YAMAHA_HAL

static int l3g4200d_probe(struct i2c_client *client,
			       const struct i2c_device_id *devid)
{
	struct l3g4200d_data *data;
	struct device *dev;
#ifdef YAMAHA_HAL
    struct input_dev *input_data = NULL;
    int input_registered = 0;
    int sysfs_created = 0;
#endif //YAMAHA_HAL

	int err = 0;
	int tempvalue;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit;
	}
	printk(KERN_INFO "%s : platform data ok!!\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	printk(KERN_INFO "%s : i2c_check_functionality I2C_FUNC_I2C!!\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
	{
		err = -ENODEV;
		goto exit;
	}

	printk(KERN_INFO "%s : i2c_check_functionality I2C_FUNC_SMBUS_I2C_BLOCK!!\n", __func__);

	/*
	 * OK. For now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */
	data = (struct l3g4200d_data *) kzalloc(sizeof(struct l3g4200d_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	printk(KERN_INFO "%s : kzalloc!!\n", __func__);

#ifdef YAMAHA_HAL
    atomic_set(&data->enabled, 0);
    atomic_set(&data->delay, SENSOR_DEFAULT_DELAY);
    data->mode = 0;

    INIT_DELAYED_WORK(&data->work, l3g4200d_work_func);
    mutex_init(&data->enable_mutex);
#endif //YAMAHA_HAL

	i2c_set_clientdata(client, data);
	data->client = client;

	printk(KERN_INFO "%s : i2c_set_clientdata!!\n", __func__);

	data->pdata = (struct l3g4200d_platform_data *) kmalloc(sizeof(*data->pdata), GFP_KERNEL);
	printk(KERN_INFO "%s : kmalloc!!\n", __func__);
	if (data->pdata == NULL)
	{
		err = -ENOMEM;
		goto exit_kfree;
	}

	memcpy(data->pdata, client->dev.platform_data, sizeof(*data->pdata));
	printk(KERN_INFO "%s : memcpy!!\n", __func__);
	
	err = l3g4200d_validate_pdata(data);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}
	
	printk(KERN_INFO "%s : validate_pdata!!\n", __func__);
	printk(KERN_INFO "%s : slave addr = %x\n", __func__, client->addr);	

	if ( (err = i2c_smbus_read_byte(client)) < 0) {
		printk(KERN_ERR "%s : i2c_smbus_read_byte error!!. err = %d\n", __func__, err);
		goto exit_kfree;
	} else {
		printk(KERN_INFO "%s : L3G4200D Device detected!\n", __func__);
	}

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	if ((tempvalue & 0x00FF) == 0x00D3) {
		printk(KERN_INFO "%s : I2C driver registered!\n", __func__);
	} else {
		printk(KERN_ERR "%s : It's not L3G4200D device. tempvalue = %d\n", __func__, tempvalue);
		err = -ENODEV;
		data->client = NULL;
		goto exit_kfree;
	}

	gyro = data;

#ifdef YAMAHA_HAL
    input_data = input_allocate_device();
    if (!input_data) {
        err = -ENOMEM;
        printk(KERN_ERR
               "sensor_probe: Failed to allocate input_data device\n");
        goto err_input_device;
    }

    set_bit(EV_ABS, input_data->evbit);
    input_set_capability(input_data, EV_ABS, ABS_X);
    input_set_capability(input_data, EV_ABS, ABS_Y);
    input_set_capability(input_data, EV_ABS, ABS_Z);
    input_set_capability(input_data, EV_ABS, ABS_STATUS); /* status */
    input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
    input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */
    input_data->name = "gyroscope";

    err = input_register_device(input_data);
    if (err) {
        printk(KERN_ERR
               "sensor_probe: Unable to register input_data device: %s\n",
               input_data->name);
        goto err_input_device;
    }
    input_set_drvdata(input_data, data);
    input_registered = 1;
#endif //YAMAHA_HAL

	/* register a char dev */
	err = register_chrdev(L3G4200D_MAJOR,
			      "l3g4200d",
			      &l3g4200d_fops);
	if (err)
		goto out;
		
	printk(KERN_INFO "%s : register_chrdev\n", __func__);

#ifndef YAMAHA_HAL	
	/* create l3g-dev device class */
	l3g_gyro_dev_class = class_create(THIS_MODULE, "L3G_GYRO-dev");
	if (IS_ERR(l3g_gyro_dev_class)) {
		err = PTR_ERR(l3g_gyro_dev_class);
		goto out_unreg_chrdev;
	}
	
	printk(KERN_INFO "%s : Dopo class_create\n", __func__);
	
	/* create device node for l3g4200d digital gyroscope */
	dev = device_create(l3g_gyro_dev_class, NULL,
		MKDEV(L3G4200D_MAJOR, 0),
		NULL,
		"l3g4200d");
	
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto error_destroy;
	}
	
	if ( (err = device_create_file(dev, &dev_attr_gyro_power_on)) < 0)
	{
		printk("Failed to create device file(%s)!\n", dev_attr_gyro_power_on.attr.name);
		goto error_destroy;
	}
	
	if ( (err = device_create_file(dev, &dev_attr_gyro_get_temp)) < 0)
	{
		printk("Failed to create device file(%s)!\n", dev_attr_gyro_get_temp.attr.name);
		goto error_destroy;
	}
	
	if ( (err = device_create_file(dev, &dev_attr_gyro_selftest)) < 0)
	{
		printk("Failed to create device file(%s)!\n", dev_attr_gyro_selftest.attr.name);
		goto error_destroy;
	}
#else
    err = sysfs_create_group(&input_data->dev.kobj, &l3g4200d_attribute_group);
    if (err < 0) {
        goto error_destroy;
    }

    sysfs_created = 1;

    this_data = input_data;
#endif	//YAMAHA_HAL
	printk(KERN_INFO "%s : L3G4200D device created successfully\n", __func__);

	return 0;
error_destroy:
#ifndef YAMAHA_HAL
	class_destroy(l3g_gyro_dev_class);
#else
	sysfs_remove_group(&client->dev.kobj, &l3g4200d_attribute_group);
#endif
out_unreg_chrdev:
	unregister_chrdev(L3G4200D_MAJOR, "l3g4200d");
out:
	printk(KERN_ERR "%s: Driver Initialization failed\n", __FILE__);
#ifdef YAMAHA_HAL
err_input_device:
  if (input_registered) {
    input_unregister_device(input_data);
  }
  else {
    input_free_device(input_data);
  }
  input_data = NULL;
#endif //YAMAHA_HAL
exit_kfree_pdata:
	printk(KERN_INFO "%s : Prima di Kfree data->pdata\n", __func__);
	kfree((void*) data->pdata);
exit_kfree:
	printk(KERN_INFO "%s : Prima di Kfree data\n", __func__);
	kfree(data);
exit:
	printk(KERN_INFO "%s : Prima di return err=%d\n", __func__, err);
	return err;
}

static int l3g4200d_remove(struct i2c_client *client)
{
	struct l3g4200d_data *lis = i2c_get_clientdata(client);
	#if DEBUG
	printk(KERN_INFO "L3G4200D driver removing\n");
	#endif
	printk(KERN_INFO "Sono in out\n");

#ifndef YAMAHA_HAL
	device_destroy(l3g_gyro_dev_class, MKDEV(L3G4200D_MAJOR, 0));
	printk(KERN_INFO "Dopo device_destroy\n");
	printk(KERN_INFO "Dopo 12c_del_driver\n");
	class_destroy(l3g_gyro_dev_class);
	printk(KERN_INFO "Dopo class_destroy\n");
#else
    sysfs_remove_group(&this_data->dev.kobj, &l3g4200d_attribute_group);
    input_unregister_device(this_data);
#endif
	unregister_chrdev(L3G4200D_MAJOR, "l3g4200d");
	printk(KERN_INFO "Dopo unregister_chrdev\n");

	kfree(lis);
	gyro->client = NULL;
	return 0;
}
#ifdef CONFIG_PM
static int l3g4200d_suspend(struct i2c_client *client, pm_message_t state)
{
	int i;
	#if DEBUG
	printk(KERN_INFO "l3g4200d_suspend\n");
	#endif
	
	/* TO DO */
	// before starting self-test, backup register
	l3g4200d_i2c_read(CTRL_REG1, &reg_backup[0], 5);
	
#if DEBUG
	for(i = 0; i < 5; i++)
		printk("[l3g4200d_suspend] backup reg[%d] = %2x\n", i, reg_backup[i]);
#endif
		
	l3g4200d_set_mode(PM_OFF);
	return 0;
}

static int l3g4200d_resume(struct i2c_client *client)
{
	int i;
	#if DEBUG
	printk(KERN_INFO "l3g4200d_resume\n");
	#endif
	
	/* TO DO */
	// restore backup register
	l3g4200d_i2c_write(CTRL_REG1, &reg_backup[0], 5);
	
#if DEBUG
	for(i = 0; i < 5; i++)
		printk("[l3g4200d_resume] backup reg[%d] = %2x\n", i, reg_backup[i]);
#endif
	
	return 0;
}
#endif

static const struct i2c_device_id l3g4200d_id[] = {
	{ "l3g4200d", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, l3g4200d_id);

static struct i2c_driver l3g4200d_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = l3g4200d_probe,
	.remove = __devexit_p(l3g4200d_remove),
	.id_table = l3g4200d_id,
	#ifdef CONFIG_PM
	.suspend = l3g4200d_suspend,
	.resume = l3g4200d_resume,
	#endif
	.driver = {
	.owner = THIS_MODULE,
	.name = "l3g4200d",
	},
	/*
	.detect = l3g4200d_detect,
	*/
};

static int __init l3g4200d_init(void)
{
	int result = 0;
	
#if defined(CONFIG_TARGET_LOCALE_LTN)
	if( HWREV <= 13)
	{
		printk( "%s : gyro sensor only work on rev0.7 and above\n", __func__ );
		return 0;
	}	
#elif defined(CONFIG_TARGET_LOCALE_EUR) || defined(CONFIG_TARGET_LOCALE_HKTW) || defined (CONFIG_TARGET_LOCALE_HKTW_FET) || defined(CONFIG_TARGET_LOCALE_VZW) || defined(CONFIG_TARGET_LOCALE_USAGSM)
	if( HWREV <= 11)
	{
		printk( "%s : gyro sensor only work on rev0.7 and above\n", __func__ );
		return result;
	}
#elif defined(CONFIG_TARGET_LOCALE_KOR)
	if( HWREV <= 10)
	{
		printk( "%s : gyro sensor only work on rev0.9 and above\n", __func__ );
		return result;
	}	
#endif
	
	printk("%s \n",__func__);

	printk(KERN_INFO "L3G4200D init driver\n");
	return i2c_add_driver(&l3g4200d_driver);
}

static void __exit l3g4200d_exit(void)
{
	#if DEBUG
	printk(KERN_INFO "L3G4200D exit\n");
	#endif
	i2c_del_driver(&l3g4200d_driver);
	return;
}

module_init(l3g4200d_init);
module_exit(l3g4200d_exit);

MODULE_DESCRIPTION("l3g4200d digital gyroscope driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

