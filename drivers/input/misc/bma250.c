/*
 * Copyright (c) 2011 Yamaha Corporation
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "bma.h"

#define BMA250_VERSION                                                "4.0.0"
#define BMA250_NAME                                                  "bma250"
#define BMA250_SLAVEADDR                                                  0x18
#define BMA250_RESOLUTION                                                 256

/* Axes data range  [um/s^2] */
#define BMA250_GRAVITY_EARTH                                          9806550
#define BMA250_ABSMIN_2G                      (-BMA250_GRAVITY_EARTH * 2)
#define BMA250_ABSMAX_2G                       (BMA250_GRAVITY_EARTH * 2)


/* Default parameters */
#define BMA250_DEFAULT_DELAY                                              100
#define BMA250_DEFAULT_POSITION                                             1
#define BMA250_DEFAULT_FILTER_THRESH       														 320000 

#define BMA250_MAX_DELAY                                                  200
#define BMA250_MIN_DELAY                                                   10

/* Registers */
#define BMA250_CHIP_ID_REG                                               0x00
#define BMA250_CHIP_ID                                                   0x03

#define BMA250_SOFT_RESET_REG                                            0x14
#define BMA250_SOFT_RESET_VAL                                            0xb6

#define BMA250_POWERMODE_REG                                             0x11
#define BMA250_POWERMODE_MASK                                            0xc0
#define BMA250_POWERMODE_SHIFT                                              6
#define BMA250_POWERMODE_NORMAL                                             0
#define BMA250_POWERMODE_LOW                                                1
#define BMA250_POWERMODE_OFF                                                2

#define BMA250_DATA_ENBL_REG                                             0x17
#define BMA250_DATA_ENBL_MASK                                            0x10
#define BMA250_DATA_ENBL_SHIFT                                              4

#define BMA250_SLEEP_DUR_REG                                             0x11
#define BMA250_SLEEP_DUR_MASK                                            0x1e
#define BMA250_SLEEP_DUR_SHIFT                                              1
#define BMA250_SLEEP_DUR_0                                                  0
#define BMA250_SLEEP_DUR_1                                                  6
#define BMA250_SLEEP_DUR_2                                                  7
#define BMA250_SLEEP_DUR_4                                                  8
#define BMA250_SLEEP_DUR_6                                                  9
#define BMA250_SLEEP_DUR_10                                                10
#define BMA250_SLEEP_DUR_25                                                11
#define BMA250_SLEEP_DUR_50                                                12
#define BMA250_SLEEP_DUR_100                                               13
#define BMA250_SLEEP_DUR_500                                               14
#define BMA250_SLEEP_DUR_1000                                              15

#define BMA250_RANGE_REG                                                 0x0f
#define BMA250_RANGE_MASK                                                0x0f
#define BMA250_RANGE_SHIFT                                                  0
#define BMA250_RANGE_2G                                                     3
#define BMA250_RANGE_4G                                                     5
#define BMA250_RANGE_8G                                                     8
#define BMA250_RANGE_16G                                                   12

#define BMA250_BANDWIDTH_REG                                             0x10
#define BMA250_BANDWIDTH_MASK                                            0x1f
#define BMA250_BANDWIDTH_SHIFT                                              0
#define BMA250_BANDWIDTH_1000HZ                                            15
#define BMA250_BANDWIDTH_500HZ                                             14
#define BMA250_BANDWIDTH_250HZ                                             13
#define BMA250_BANDWIDTH_125HZ                                             12
#define BMA250_BANDWIDTH_63HZ                                              11
#define BMA250_BANDWIDTH_32HZ                                              10
#define BMA250_BANDWIDTH_16HZ                                               9
#define BMA250_BANDWIDTH_8HZ                                                8 
#define BMA250_ACC_REG                                                   0x02

#define BMA250_EEPROM_CTRL_REG          0x33
#define BMA250_EEPROM_CTRL_MASK         0x0f
#define BMA250_EEPROM_CTRL_SHIFT        0

/* SETTING THIS BIT  UNLOCK'S WRITING SETTING REGISTERS TO EEPROM */
#define BMA250_UNLOCK_EE_WRITE_SETTING__POS     0
#define BMA250_UNLOCK_EE_WRITE_SETTING__LEN     1
#define BMA250_UNLOCK_EE_WRITE_SETTING__MSK     0x01
#define BMA250_UNLOCK_EE_WRITE_SETTING__REG     BMA250_EEPROM_CTRL_REG


/* SETTING THIS BIT STARTS WRITING SETTING REGISTERS TO EEPROM */
#define BMA250_START_EE_WRITE_SETTING__POS      1
#define BMA250_START_EE_WRITE_SETTING__LEN      1
#define BMA250_START_EE_WRITE_SETTING__MSK      0x02
#define BMA250_START_EE_WRITE_SETTING__REG      BMA250_EEPROM_CTRL_REG


/* STATUS OF WRITING TO EEPROM */
#define BMA250_EE_WRITE_SETTING_S__POS          2
#define BMA250_EE_WRITE_SETTING_S__LEN          1
#define BMA250_EE_WRITE_SETTING_S__MSK          0x04
#define BMA250_EE_WRITE_SETTING_S__REG          BMA250_EEPROM_CTRL_REG


/* UPDATE IMAGE REGISTERS WRITING TO EEPROM */
#define BMA250_UPDATE_IMAGE_SHIFT              3
#define BMA250_UPDATE_IMAGE_LEN                1
#define BMA250_UPDATE_IMAGE_MASK               0x08
#define BMA250_UPDATE_IMAGE_REG                BMA250_EEPROM_CTRL_REG


/* STATUS OF IMAGE REGISTERS WRITING TO EEPROM */
#define BMA250_IMAGE_REG_EE_WRITE_S_SHIFT      3
#define BMA250_IMAGE_REG_EE_WRITE_S_LEN        1
#define BMA250_IMAGE_REG_EE_WRITE_S_MASK       0x08
#define BMA250_IMAGE_REG_EE_WRITE_S_REG        BMA250_EEPROM_CTRL_REG


#define BMA250_SERIAL_CTRL_REG          0x34
#define BMA250_SERIAL_CTRL_MASK         0x07
#define BMA250_SERIAL_CTRL_SHIFT        0

#define BMA250_OFFSET_CTRL_REG          0x36
#define BMA250_OFFSET_CTRL_MASK         0xff
#define BMA250_OFFSET_CTRL_SHIFT        0

/**    SLOW COMPENSATION FOR X,Y,Z AXIS      **/

#define BMA250_EN_SLOW_COMP_X_SHIFT            0
#define BMA250_EN_SLOW_COMP_X_LEN              1
#define BMA250_EN_SLOW_COMP_X_MASK             0x01
#define BMA250_EN_SLOW_COMP_X_REG              BMA250_OFFSET_CTRL_REG

#define BMA250_EN_SLOW_COMP_Y_SHIFT            1
#define BMA250_EN_SLOW_COMP_Y_LEN              1
#define BMA250_EN_SLOW_COMP_Y_MASK             0x02
#define BMA250_EN_SLOW_COMP_Y_REG              BMA250_OFFSET_CTRL_REG

#define BMA250_EN_SLOW_COMP_Z_SHIFT            2
#define BMA250_EN_SLOW_COMP_Z_LEN              1
#define BMA250_EN_SLOW_COMP_Z_MASK             0x04
#define BMA250_EN_SLOW_COMP_Z_REG              BMA250_OFFSET_CTRL_REG

#define BMA250_EN_SLOW_COMP_XYZ_SHIFT            0
#define BMA250_EN_SLOW_COMP_XYZ_LEN              3
#define BMA250_EN_SLOW_COMP_XYZ_MASK             0x07
#define BMA250_EN_SLOW_COMP_XYZ_REG              BMA250_OFFSET_CTRL_REG

/**    FAST COMPENSATION READY FLAG          **/

#define BMA250_FAST_COMP_RDY_S__POS            4
#define BMA250_FAST_COMP_RDY_S_LEN             1
#define BMA250_FAST_COMP_RDY_S__MSK            0x10
#define BMA250_FAST_COMP_RDY_S_REG             BMA250_OFFSET_CTRL_REG

/**    FAST COMPENSATION FOR X,Y,Z AXIS      **/

#define BMA250_EN_FAST_COMP__POS                5
#define BMA250_EN_FAST_COMP__LEN                2
#define BMA250_EN_FAST_COMP__MSK                0x60
#define BMA250_EN_FAST_COMP__REG                BMA250_OFFSET_CTRL_REG

/**    RESET OFFSET REGISTERS                **/

#define BMA250_RESET_OFFSET_REGS_SHIFT         7
#define BMA250_RESET_OFFSET_REGS_LEN           1
#define BMA250_RESET_OFFSET_REGS_MASK          0x80
#define BMA250_RESET_OFFSET_REGS_REG           BMA250_OFFSET_CTRL_REG

#define BMA250_OFFSET_PARAMS_REG        0x37
#define BMA250_OFFSET_PARAMS_MASK       0x7f
#define BMA250_OFFSET_PARAMS_SHIFT      0

#define BMA250_COMP_CUTOFF_SHIFT               0
#define BMA250_COMP_CUTOFF_LEN                 1
#define BMA250_COMP_CUTOFF_MASK                0x01
#define BMA250_COMP_CUTOFF_REG                 BMA250_OFFSET_PARAMS_REG

/**     COMPENSATION TARGET                  **/

#define BMA250_COMP_TARGET_OFFSET_X__POS   			1
#define BMA250_COMP_TARGET_OFFSET_X__LEN        2
#define BMA250_COMP_TARGET_OFFSET_X__MSK       	0x06
#define BMA250_COMP_TARGET_OFFSET_X__REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_COMP_TARGET_OFFSET_Y__POS      	3
#define BMA250_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA250_COMP_TARGET_OFFSET_Y__MSK       	0x18
#define BMA250_COMP_TARGET_OFFSET_Y__REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_COMP_TARGET_OFFSET_Z__POS      	5
#define BMA250_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA250_COMP_TARGET_OFFSET_Z__MSK       	0x60
#define BMA250_COMP_TARGET_OFFSET_Z__REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_GET_BITSLICE(regvar, bitname)\
                        (regvar & bitname##__MSK) >> bitname##__POS


#define BMA250_SET_BITSLICE(regvar, bitname, val)\
                  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

#define BMA250_OFFSET_FILT_X_REG        0x38
#define BMA250_OFFSET_FILT_X_MASK       0xff
#define BMA250_OFFSET_FILT_X_SHIFT      0

#define BMA250_OFFSET_FILT_Y_REG        0x39
#define BMA250_OFFSET_FILT_Y_MASK       0xff
#define BMA250_OFFSET_FILT_Y_SHIFT      0

#define BMA250_OFFSET_FILT_Z_REG        0x3A
#define BMA250_OFFSET_FILT_Z_MASK       0xff
#define BMA250_OFFSET_FILT_Z_SHIFT      0

#define BMA250_OFFSET_UNFILT_X_REG      0x3B
#define BMA250_OFFSET_UNFILT_X_MASK     0xff
#define BMA250_OFFSET_UNFILT_X_SHIFT    0

#define BMA250_OFFSET_UNFILT_Y_REG      0x3C
#define BMA250_OFFSET_UNFILT_Y_MASK     0xff
#define BMA250_OFFSET_UNFIILT_Y_SHIFT   0

#define BMA250_OFFSET_UNFILT_Z_REG      0x3D
#define BMA250_OFFSET_UNFILT_Z_MASK     0xff
#define BMA250_OFFSET_UNFILT_Z_SHIFT    0

/* --------------------------------------------------------------------------- */
/*  Structure definition                                                       */
/* --------------------------------------------------------------------------- */
/* Output data rate */
struct bma250_odr {
    unsigned long delay;               /* min delay (msec) in the range of ODR */
    unsigned char odr;                 /* bandwidth register value             */
};

/* Axes data */
struct bma250_acceleration {
    int x;
    int y;
    int z;
    int x_raw;
    int y_raw;
    int z_raw;
};

/* Driver private data */
struct bma250_data {
    int initialize;
    int i2c_open;
    int enable;
    int delay;
    int position;
    int threshold;
    int filter_enable;
    struct bma_vector offset;
    struct bma250_acceleration last;
};

/* Sleep duration */
struct bma250_sd {
    uint8_t bw;
    uint8_t sd;
};

/* --------------------------------------------------------------------------- */
/*  Data                                                                       */
/* --------------------------------------------------------------------------- */
/* Control block */
static struct bma_acc_driver  cb;
static struct bma_acc_driver *pcb = NULL;
static struct bma250_data acc_data;

/* Output data rate */
static const struct bma250_odr bma250_odr_tbl[] = {
    {1,   BMA250_BANDWIDTH_1000HZ},
    {2,   BMA250_BANDWIDTH_500HZ},
    {4,   BMA250_BANDWIDTH_250HZ},
    {8,   BMA250_BANDWIDTH_125HZ},
    {16,  BMA250_BANDWIDTH_63HZ},
    {32,  BMA250_BANDWIDTH_32HZ},
    {64,  BMA250_BANDWIDTH_16HZ},
    {128, BMA250_BANDWIDTH_8HZ},
};

/* Sleep duration */
static const struct bma250_sd bma250_sd_table[] = {
    {BMA250_BANDWIDTH_8HZ    /* 128ms */, BMA250_SLEEP_DUR_100},
    {BMA250_BANDWIDTH_16HZ   /*  64ms */, BMA250_SLEEP_DUR_50},
    {BMA250_BANDWIDTH_32HZ   /*  32ms */, BMA250_SLEEP_DUR_25},
    {BMA250_BANDWIDTH_63HZ   /*  16ms */, BMA250_SLEEP_DUR_10},
    {BMA250_BANDWIDTH_125HZ  /*   8ms */, BMA250_SLEEP_DUR_6},
    {BMA250_BANDWIDTH_250HZ  /*   4ms */, BMA250_SLEEP_DUR_2},
    {BMA250_BANDWIDTH_500HZ  /*   2ms */, BMA250_SLEEP_DUR_1},
    {BMA250_BANDWIDTH_1000HZ /*   1ms */, BMA250_SLEEP_DUR_0},
};

/* Transformation matrix for chip mounting position */
static const int bma250_position_map[][3][3] = {
    {{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}},          /* top/upper-left     */
    {{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}},          /* top/upper-right    */
    {{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}},          /* top/lower-right    */
    {{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}},          /* top/lower-left     */
    {{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}},          /* bottom/upper-right */
    {{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}},          /* bottom/upper-left  */
    {{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}},          /* bottom/lower-left  */
    {{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}},          /* bottom/lower-right */
};

/* --------------------------------------------------------------------------- */
/*  Prototype declaration                                                      */
/* --------------------------------------------------------------------------- */
static void bma250_init_data(void);
static int bma250_ischg_enable(int);
static int bma250_read_reg(unsigned char, unsigned char *, unsigned char);
static int bma250_write_reg(unsigned char, unsigned char *, unsigned char);
static int bma250_read_reg_byte(unsigned char);
static int bma250_write_reg_byte(unsigned char, unsigned char);
static int bma250_lock(void);
static int bma250_unlock(void);
static int bma250_i2c_open(void);
static int bma250_i2c_close(void);
static int bma250_msleep(int);
static int bma250_power_up(void);
static int bma250_power_down(void);
static int bma250_init(void);
static int bma250_term(void);
static int bma250_get_delay(void);
static int bma250_set_delay(int);
static int bma250_get_offset(struct bma_vector *);
static int bma250_set_offset(struct bma_vector *);
static int bma250_get_enable(void);
static int bma250_set_enable(int);
static int bma250_get_filter(struct bma_acc_filter *);
static int bma250_set_filter(struct bma_acc_filter *);
static int bma250_get_filter_enable(void);
static int bma250_set_filter_enable(int);
static int bma250_get_position(void);
static int bma250_set_position(int);
static int bma250_measure(int *, int *);
#if DEBUG
static int bma_get_register(uint8_t, uint8_t *);
#endif

//110324  hm83.cho   For calibration
int bma250_set_offset_target_x(unsigned char offsettarget);
/* EasyCASE ) */
/* EasyCASE ( 1175
   bma250_get_offset_target_x */
int bma250_get_offset_target_x(unsigned char *offsettarget );
/* EasyCASE ) */
/* EasyCASE ( 1179
   bma250_set_offset_target_y */
int bma250_set_offset_target_y(unsigned char offsettarget);
/* EasyCASE ) */
/* EasyCASE ( 1183
   bma250_get_offset_target_y */
int bma250_get_offset_target_y(unsigned char *offsettarget );
/* EasyCASE ) */
/* EasyCASE ( 1185
   bma250_set_offset_target_z */
int bma250_set_offset_target_z(unsigned char offsettarget);
/* EasyCASE ) */
/* EasyCASE ( 1187
   bma250_get_offset_target_z */
int bma250_get_offset_target_z(unsigned char *offsettarget );
/* EasyCASE ) */
/* EasyCASE ( 1189
   bma250_set_offset_filt_x */
int bma250_set_offset_filt_x(unsigned char offsetfilt);
/* EasyCASE ) */
/* EasyCASE ( 1191
   bma250_get_offset_filt_x */
int bma250_get_offset_filt_x(unsigned char *offsetfilt );
/* EasyCASE ) */
/* EasyCASE ( 1195
   bma250_set_offset_filt_y */
int bma250_set_offset_filt_y(unsigned char offsetfilt);
/* EasyCASE ) */
/* EasyCASE ( 1197
   bma250_get_offset_filt_y */
int bma250_get_offset_filt_y(unsigned char *offsetfilt );
/* EasyCASE ) */
/* EasyCASE ( 1199
   bma250_set_offset_filt_z */
int bma250_set_offset_filt_z(unsigned char offsetfilt);
/* EasyCASE ) */
/* EasyCASE ( 1201
   bma250_get_offset_filt_z */
int bma250_get_offset_filt_z(unsigned char *offsetfilt );
int bma250_set_ee_w(unsigned char eew);
int bma250_set_ee_prog_trig(void);
int bma250_get_eeprom_writing_status(unsigned char *eewrite );
int bma250_get_cal_ready(unsigned char *calrdy );
int bma250_set_cal_trigger(unsigned char caltrigger);

static int bma250_set_calibration(signed char*,int);
static int bma250_get_calibration(signed char*);
/* --------------------------------------------------------------------------- */
/*  Local functions                                                            */
/* --------------------------------------------------------------------------- */

static void bma250_init_data(void) {
    acc_data.initialize = 0;
    acc_data.enable = 0;
    acc_data.delay = BMA250_DEFAULT_DELAY;
    acc_data.offset.v[0] = 0;
    acc_data.offset.v[1] = 0;
    acc_data.offset.v[2] = 0;
    acc_data.position = BMA250_DEFAULT_POSITION;
    acc_data.threshold = BMA250_DEFAULT_FILTER_THRESH;
    acc_data.filter_enable = 1;
    acc_data.last.x = 0;
    acc_data.last.y = 0;
    acc_data.last.z = 0;
    acc_data.last.x_raw = 0;
    acc_data.last.y_raw = 0;
    acc_data.last.z_raw = 0;
}

static int bma250_ischg_enable(int enable)
{
    if (acc_data.enable == enable) {
        return 0;
    }

    return 1;
}

/* register access functions */
static int bma250_read_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_read(BMA250_SLAVEADDR, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return BMA_NO_ERROR;
}

static int bma250_write_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_write(BMA250_SLAVEADDR, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return BMA_NO_ERROR;
}

static int bma250_read_reg_byte(unsigned char adr)
{
    unsigned char buf=0xff;
    int err;

    err = bma250_read_reg(adr, &buf, 1);
    if (err == 0) {
        return buf;
    }

    return 0;
}

static int bma250_write_reg_byte(unsigned char adr, unsigned char val)
{
    return bma250_write_reg(adr, &val, 1);
}

#define bma250_read_bits(r) \
    ((bma250_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define bma250_update_bits(r,v) \
    bma250_write_reg_byte(r##_REG, \
                           ((bma250_read_reg_byte(r##_REG) & ~r##_MASK) | ((v) << r##_SHIFT)))

static int bma250_lock(void)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (cbk->lock != NULL && cbk->unlock != NULL) {
        err = cbk->lock();
    } else {
        err = BMA_NO_ERROR;
    }

    return err;
}

static int bma250_unlock(void)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (cbk->lock != NULL && cbk->unlock != NULL) {
        err = cbk->unlock();
    } else {
        err = BMA_NO_ERROR;
    }

    return err;
}

static int bma250_i2c_open(void)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open == 0) {
        err = cbk->i2c_open();
        if (err != BMA_NO_ERROR) {
            return BMA_ERROR_I2C;
        }
        acc_data.i2c_open = 1;
    }

    return BMA_NO_ERROR;
}

static int bma250_i2c_close(void)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open != 0) {
        err = cbk->i2c_close();
        if (err != BMA_NO_ERROR) {
            return BMA_ERROR_I2C;
        }
        acc_data.i2c_open = 0;
    }
    return BMA_NO_ERROR;
}

static int bma250_msleep(int msec)
{
    struct bma_acc_driver_callback *cbk = &pcb->callback;

    if (msec <= 0) {
        return BMA_ERROR_ARG;
    }

    cbk->msleep(msec);

    return BMA_NO_ERROR;
}

static int bma250_set_sleep_dur(unsigned char bw)
{
    int i;
    int delay = acc_data.delay;

    if (bw == BMA250_BANDWIDTH_8HZ) {
        if (1000 < delay && delay < 2000) {
            return BMA250_SLEEP_DUR_500;
        }
        if (2000 <= delay) {
            return BMA250_SLEEP_DUR_1000;
        }
    }
    for (i = 0; i < (int)(sizeof(bma250_sd_table) / sizeof(struct bma250_sd)); i++) {
        if (bma250_sd_table[i].bw == bw) {
            /* Success */
            return bma250_sd_table[i].sd;
        }
    }

    /* Error */
    return -1;
}

static int bma250_power_up(void)
{
    bma250_update_bits(BMA250_DATA_ENBL, 1);
    bma250_update_bits(BMA250_POWERMODE, BMA250_POWERMODE_NORMAL);

    return BMA_NO_ERROR;
}

static int bma250_power_down(void)
{
    bma250_update_bits(BMA250_DATA_ENBL, 0);
    bma250_update_bits(BMA250_POWERMODE, BMA250_POWERMODE_OFF);

    return BMA_NO_ERROR;
}

static int bma250_init(void)
{
    struct bma_acc_filter filter;
    int err;
    int id;
    
    printk("[acc]bma-250_init!!\n");
    /* Check intialize */
    if (acc_data.initialize == 1) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    /* Init data */
    bma250_init_data();

    /* Open i2c */
    err = bma250_i2c_open();
    if (err != BMA_NO_ERROR) {
        return err;
    }

    /* Check id */
    id = bma250_read_reg_byte(BMA250_CHIP_ID_REG);
    if (id != BMA250_CHIP_ID) {
        bma250_i2c_close();
        return BMA_ERROR_CHIP_ID;
    }

    /* Reset chip */
    bma250_write_reg_byte(BMA250_SOFT_RESET_REG, BMA250_SOFT_RESET_VAL);
    bma250_msleep(1);

    /* Set axes range*/
    bma250_update_bits(BMA250_RANGE, BMA250_RANGE_2G);

    acc_data.initialize = 1;

    bma250_set_delay(BMA250_DEFAULT_DELAY);
    bma250_set_position(BMA250_DEFAULT_POSITION);
    filter.threshold = BMA250_DEFAULT_FILTER_THRESH;
    bma250_set_filter(&filter);

    return BMA_NO_ERROR;
}

static int bma250_term(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_set_enable(0);

    /* Close I2C */
    bma250_i2c_close();

    acc_data.initialize = 0;

    return BMA_NO_ERROR;
}

static int bma250_get_delay(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    return acc_data.delay;
}

static int bma250_set_delay(int delay)
{
    unsigned char odr;
    int i;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    /* Determine optimum odr */
    for (i = 1; i < (int)(sizeof(bma250_odr_tbl) / sizeof(struct bma250_odr)) &&
             delay >= (int)bma250_odr_tbl[i].delay; i++)
        ;

    odr = bma250_odr_tbl[i-1].odr;
    acc_data.delay = delay;

    if (bma250_get_enable()) {
        bma250_update_bits(BMA250_BANDWIDTH, odr);
        bma250_update_bits(BMA250_SLEEP_DUR, bma250_set_sleep_dur(odr));
    } else {
        bma250_power_up();
        bma250_update_bits(BMA250_BANDWIDTH, odr);
        bma250_update_bits(BMA250_SLEEP_DUR, bma250_set_sleep_dur(odr));
        bma250_power_down();
    }

    return BMA_NO_ERROR;
}

static int bma250_get_offset(struct bma_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    *offset = acc_data.offset;

    return BMA_NO_ERROR;
}

static int bma250_set_offset(struct bma_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    acc_data.offset = *offset;

    return BMA_NO_ERROR;
}

static int bma250_get_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    return acc_data.enable;
}

static int bma250_set_enable(int enable)
{
    int err;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (bma250_ischg_enable(enable)) {
        if (enable) {
            /* Open i2c */
            err = bma250_i2c_open();
            if (err != BMA_NO_ERROR) {
                return err;
            }
            /* Reset chip */
            bma250_write_reg_byte(BMA250_SOFT_RESET_REG, BMA250_SOFT_RESET_VAL);
            bma250_msleep(1);
            /* Set axes range*/
            bma250_update_bits(BMA250_RANGE, BMA250_RANGE_2G);
            bma250_set_delay(acc_data.delay);
            bma250_power_up();
        } else {
            bma250_power_down();
            err = bma250_i2c_close();
            if (err != BMA_NO_ERROR) {
                return err;
            }
        }
    }

    acc_data.enable = enable;

    return BMA_NO_ERROR;
}

static int bma250_get_filter(struct bma_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    filter->threshold = acc_data.threshold;

    return BMA_NO_ERROR;
}

static int bma250_set_filter(struct bma_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    acc_data.threshold = filter->threshold;

    return BMA_NO_ERROR;
}

static int bma250_get_filter_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    return acc_data.filter_enable;
}

static int bma250_set_filter_enable(int enable)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    acc_data.filter_enable = enable;

    return BMA_NO_ERROR;
}

static int bma250_get_position(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    return acc_data.position;
}

static int bma250_set_position(int position)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    acc_data.position = position;

    return BMA_NO_ERROR;
}

static int bma250_data_filter(int data[], int raw[], struct bma250_acceleration *accel)
{
    int filter_enable = acc_data.filter_enable;
    int threshold = acc_data.threshold;

    if (filter_enable) {
        if ((ABS(acc_data.last.x - data[0]) > threshold) ||
            (ABS(acc_data.last.y - data[1]) > threshold) ||
            (ABS(acc_data.last.z - data[2]) > threshold)) {
            accel->x = data[0];
            accel->y = data[1];
            accel->z = data[2];
            accel->x_raw = raw[0];
            accel->y_raw = raw[1];
            accel->z_raw = raw[2];
        } else {
            *accel = acc_data.last;
        }
    } else {
        accel->x = data[0];
        accel->y = data[1];
        accel->z = data[2];
        accel->x_raw = raw[0];
        accel->y_raw = raw[1];
        accel->z_raw = raw[2];
    }

    return BMA_NO_ERROR;
}

static int bma250_measure(int *out_data, int *out_raw)
{
    struct bma250_acceleration accel;
    unsigned char buf[6];
    int32_t raw[3], data[3];
    int pos = acc_data.position;
    int i,j;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    /* Read acceleration data */
    if (bma250_read_reg(BMA250_ACC_REG, buf, 6) != 0) {
        for (i = 0; i < 3; i++) raw[i] = 0;
    } else {
        for (i = 0; i < 3; i++) raw[i] = ((int16_t)((buf[i*2+1] << 8)) | (buf[i*2] & 0xfe)) >> 6;
    }

    /* for X, Y, Z axis */
    for (i = 0; i < 3; i++) {
        /* coordinate transformation */
        data[i] = 0;
        for (j = 0; j < 3; j++) {
            data[i] += raw[j] * bma250_position_map[pos][i][j];
        }
        /* normalization */
        data[i] *= (BMA250_GRAVITY_EARTH / BMA250_RESOLUTION);
    }

    bma250_data_filter(data, raw, &accel);

    out_data[0] = accel.x - acc_data.offset.v[0];
    out_data[1] = accel.y - acc_data.offset.v[1];
    out_data[2] = accel.z - acc_data.offset.v[2];
    out_raw[0] = accel.x_raw;
    out_raw[1] = accel.y_raw;
    out_raw[2] = accel.z_raw;
    acc_data.last = accel;

    return BMA_NO_ERROR;
}
static int bma250_set_calibration(signed char* data_cal, int cal_init)
{
	int count = 0;
  // for debug
    printk("[diony] bma250_set_calibration!!!! .\n");
    signed char tmp;
    data_cal[0] = data_cal[1] = 0;
    data_cal[2]=1;
    //bma250_write_reg_byte(bma250_SOFT_RESET_REG, bma250_SOFT_RESET_VAL);
    //bma250_msleep(1);
     /* Set axes range*/
    //bma250_update_bits(bma250_RANGE, bma250_RANGE_2G);
    //bma250_set_delay(acc_data.delay);
    //bma250_update_bits(bma250_DATA_ENBL, 1);
	if (cal_init == 1)
	{
		printk(KERN_INFO "[HSS] Calibraion Init\n");
		bma250_set_offset_filt_x(0);
		bma250_set_offset_filt_y(0);
		bma250_set_offset_filt_z(0);     
	}
	else
	{    
#ifdef DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
    printk(KERN_INFO "data are %d,%d,%d\n",data_cal[0],data_cal[1],data_cal[2]);
    printk(KERN_INFO "start x axis fast calibration\n");
#endif
    bma250_set_offset_target_x(data_cal[0]);
    tmp=1;//selet x axis in cal_trigger
    bma250_set_cal_trigger(tmp);
    do
    {
		count++;
        mdelay(2);
       bma250_get_cal_ready(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got cal ready flag is %d\n",tmp);
#endif
		if (count > 1500)
		{
			printk("[HSS] Calibration ready fail for x\n");
			return -1;
		}
    }while(tmp==0);
	
#ifdef DEBUG
   bma250_get_offset_filt_x(&tmp);
    printk(KERN_INFO "x offset filt is %d\n",tmp);
    printk(KERN_INFO "x axis fast calibration finished\n");
    printk(KERN_INFO "start y axis fast calibration\n");
#endif
    bma250_set_offset_target_y(data_cal[1]);
    //bma250_get_offset_target_y(&tmp);
    //printk(KERN_INFO "y offset is %d\n",tmp);
    //bma250_get_offset_filt_y(&tmp);
    //printk(KERN_INFO "y offset filt is %d\n",tmp);
    tmp=2;//selet y axis in cal_trigger
    bma250_set_cal_trigger(tmp);
	count = 0;
    do
    {
		count++;
        mdelay(2); 
        bma250_get_cal_ready(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got cal ready flag is %d\n",tmp);
#endif
		if (count > 1500)
		{
			printk("[HSS] Calibration ready fail for y\n");
			return -1;
		}
    }while(tmp==0);
	
#ifdef DEBUG
    bma250_get_offset_filt_y(&tmp);
    printk(KERN_INFO "y offset filt is %d\n",tmp);
    printk(KERN_INFO "y axis fast calibration finished\n");
    printk(KERN_INFO "start z axis fast calibration\n");
#endif
    bma250_set_offset_target_z(data_cal[2]);

    //bma250_get_offset_target_z(&tmp);
    //printk(KERN_INFO "z offset is %d\n",tmp);
    //bma250_get_offset_filt_z(&tmp);
    //printk(KERN_INFO "z offset filt is %d\n",tmp);
    tmp=3;//selet z axis in cal_trigger
    bma250_set_cal_trigger(tmp);
    count = 0;
    do
    {
		count++;
        mdelay(2);   //for test 
        bma250_get_cal_ready(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got cal ready flag is %d\n",tmp);
#endif  
		if (count > 1500)
		{
			printk("[HSS] Calibration ready fail for z\n");
			return -1;
		}
    }while(tmp==0);
	
#ifdef DEBUG
    bma250_get_offset_filt_z(&tmp);
    printk(KERN_INFO "z offset filt is %d\n",tmp);
    printk(KERN_INFO "z axis fast calibration finished\n");
    printk(KERN_INFO "store xyz offset to eeprom\n");
#endif
}
    tmp=1;//unlock eeprom
    bma250_set_ee_w(tmp);
    bma250_set_ee_prog_trig();//update eeprom
	count = 0;
    do
    {
		count++;      
        mdelay(2); 
        bma250_get_eeprom_writing_status(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got eeprom writing status is %d\n",tmp);
#endif  
		if (count > 1500)
		{
			printk("[HSS] Calibration eeprom writing fai\n");
			return -1;
		}
    }while(tmp==0);
	
    tmp=0;//lock eemprom
    bma250_set_ee_w(tmp);
#ifdef DEBUG
    printk(KERN_INFO "eeprom writing is finished\n");
    printk("[diony] -------------bma250_set_calibration End!!!! --------------.\n");
#endif  
    return 0;
}
static int bma250_get_calibration(signed char* cal_result)
{
	signed char tmp = 0;
	printk("[HSS] [%s] \n", __func__);

	bma250_get_offset_filt_x(&tmp);
	cal_result[0] = tmp;
	bma250_get_offset_filt_y(&tmp);
	cal_result[1] = tmp;
	bma250_get_offset_filt_z(&tmp);
	cal_result[2] = tmp;
    
	return 0;
}
int bma250_set_offset_target_x(unsigned char offsettarget)
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
   else
      {
      err = bma250_read_reg(BMA250_COMP_TARGET_OFFSET_X__REG, &data, 1);
	  printk("[diony] bma250_set_offset_target_data  = %d .  (read)  \n",data);
      data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_X, offsettarget );
	  printk("[diony] bma250_set_offset_target_data  = %d .  (after BITSLICE,write)  \n",data);
      err =bma250_write_reg(BMA250_COMP_TARGET_OFFSET_X__REG, &data, 1);
      }
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100369
   bma250_get_offset_target_x */
/* Compiler Switch if applicable
#ifdef

#endif
*/
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API gets the offset_target_x
 *
 *
 *
 *
 *  \param unsigned char *offsettarget
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_get_offset_target_x(unsigned char *offsettarget )
{
    int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = bma250_read_reg(BMA250_COMP_TARGET_OFFSET_X__REG, &data, 1);
      data = BMA250_GET_BITSLICE(data,BMA250_COMP_TARGET_OFFSET_X);
      *offsettarget = data;
   }
   
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100368
   bma250_set_offset_target_y */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API sets the offset_target_y
 *
 *
 *
 *
 *  \param unsigned char offsettarget
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_set_offset_target_y(unsigned char offsettarget)
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = bma250_read_reg(BMA250_COMP_TARGET_OFFSET_Y__REG, &data, 1);
      data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Y, offsettarget );
      err = bma250_write_reg(BMA250_COMP_TARGET_OFFSET_Y__REG, &data, 1);
   }
   return err;
}


/* EasyCASE ) */
/* EasyCASE ( 100364
   bma250_get_offset_target_y */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API gets the offset_target_y
 *
 *
 *
 *
 *  \param unsigned char *offsettarget
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_get_offset_target_y(unsigned char *offsettarget )
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = bma250_read_reg(BMA250_COMP_TARGET_OFFSET_Y__REG, &data, 1);
      data = BMA250_GET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Y);
      *offsettarget = data;
   }
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100371
   bma250_set_offset_target_z */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API sets the offset_target_z
 *
 *
 *
 *
 *  \param unsigned char offsettarget
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_set_offset_target_z(unsigned char offsettarget)
{
    int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
   else
   {
       err = bma250_read_reg(BMA250_COMP_TARGET_OFFSET_Z__REG, &data, 1);
      data = BMA250_SET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Z, offsettarget );
       err = bma250_write_reg(BMA250_COMP_TARGET_OFFSET_Z__REG, &data, 1);
   }
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100372
   bma250_get_offset_target_z */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API gets the offset_target_z
 *
 *
 *
 *
 *  \param unsigned char *offsettarget
 *
 *
 *
 *  \return result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_get_offset_target_z(unsigned char *offsettarget )
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = bma250_read_reg(BMA250_COMP_TARGET_OFFSET_Z__REG, &data, 1);
      data =BMA250_GET_BITSLICE(data, BMA250_COMP_TARGET_OFFSET_Z);
      *offsettarget = data;
   }
   return err;
}



int bma250_set_cal_trigger(unsigned char caltrigger)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err = bma250_read_reg(BMA250_EN_FAST_COMP__REG, &data, 1);
		printk("[diony] bma250_set_cal_trigger  data = %d .  (read)  \n",data);
	 	data = BMA250_SET_BITSLICE(data, BMA250_EN_FAST_COMP, caltrigger );
		 printk("[diony] bma250_set_cal_trigger  data = %d .  (after BITSLICE,write)  \n",data);
		err = bma250_write_reg(BMA250_EN_FAST_COMP__REG, &data, 1);
	}
	return err;
}


int bma250_get_cal_ready(unsigned char *calrdy )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err = bma250_read_reg(BMA250_OFFSET_CTRL_REG, &data, 1);
		printk("[diony] bma250_get_cal_ready data= %d (read) \n",data);
		data = BMA250_GET_BITSLICE(data, BMA250_FAST_COMP_RDY_S);
		printk("[diony] bma250_get_cal_ready data = %d  (after BITSLICE)\n",data);
		*calrdy = data;
	}
	
	return err;
}


int bma250_set_offset_filt_x(unsigned char offsetfilt)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = bma250_write_reg(BMA250_OFFSET_FILT_X_REG, &data, 1);
    }
   	return err;
}

/* EasyCASE ) */
/* EasyCASE ( 100377
   bma250_get_offset_filt_x */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API gets the offset_filt_x
 *
 *
 *
 *
 *  \param
 *                    unsigned char *offsetfilt
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  \return Result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_get_offset_filt_x(unsigned char *offsetfilt )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	err = bma250_read_reg(BMA250_OFFSET_FILT_X_REG, &data, 1);
	*offsetfilt =  data;
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100381
   bma250_set_offset_filt_y */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API sets the offset_filt_y
 *
 *
 *
 *
 *  \param
 *                    unsigned char offsetfilt
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  \return Result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_set_offset_filt_y(unsigned char offsetfilt)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = bma250_write_reg(BMA250_OFFSET_FILT_Y_REG, &data, 1);
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100382
   bma250_get_offset_filt_y */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API gets the offset_filt_x
 *
 *
 *
 *
 *  \param
 *                    unsigned char *offsetfilt
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  \return Result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_get_offset_filt_y(unsigned char *offsetfilt )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	err = bma250_read_reg(BMA250_OFFSET_FILT_Y_REG, &data, 1);
        *offsetfilt =  data;
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100384
   bma250_set_offset_filt_z */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API sets the offset_filt_z
 *
 *
 *
 *
 *  \param
 *                    unsigned char offsetfilt
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  \return Result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_set_offset_filt_z(unsigned char offsetfilt)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = bma250_write_reg(BMA250_OFFSET_FILT_Z_REG, &data, 1);
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100385
   bma250_get_offset_filt_z */
/* EasyCASE F */
/*******************************************************************************
 * Description: *//**\brief This API sets the offset_filt_z
 *
 *
 *
 *
 *  \param
 *                    unsigned char *offsetfilt
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  \return Result of communication routines
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
int bma250_get_offset_filt_z(unsigned char *offsetfilt )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	err = bma250_read_reg(BMA250_OFFSET_FILT_Z_REG, &data, 1);
	*offsetfilt =  data;
    }
   	return err;
}
int bma250_set_ee_w(unsigned char eew)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err =  bma250_read_reg(BMA250_UNLOCK_EE_WRITE_SETTING__REG, &data, 1);
		data = BMA250_SET_BITSLICE(data, BMA250_UNLOCK_EE_WRITE_SETTING, eew);
		err =  bma250_write_reg(BMA250_UNLOCK_EE_WRITE_SETTING__REG, &data, 1);
	}
	return err;
}

int bma250_set_ee_prog_trig(void)
{
	int err=0;
       unsigned char data;
	 unsigned char eeprog;
	eeprog = 0x01;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err =  bma250_read_reg(BMA250_START_EE_WRITE_SETTING__REG, &data, 1);
		data = BMA250_SET_BITSLICE(data, BMA250_START_EE_WRITE_SETTING, eeprog);
		err =  bma250_write_reg(BMA250_START_EE_WRITE_SETTING__REG, &data, 1);
	}
	return err;
}

int bma250_get_eeprom_writing_status(unsigned char *eewrite )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return BMA_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err =  bma250_read_reg(BMA250_EEPROM_CTRL_REG, &data, 1);
		data = BMA250_GET_BITSLICE(data, BMA250_EE_WRITE_SETTING_S);
		*eewrite = data;
	}
	return err;
}


/* --------------------------------------------------------------------------- */
static int bma_init(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_lock();
    err = bma250_init();
    bma250_unlock();

    return err;
}

static int bma_term(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_lock();
    err = bma250_term();
    bma250_unlock();

    return err;
}

static int bma_get_delay(void)
{
    int ret;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_lock();
    ret = bma250_get_delay();
    bma250_unlock();

    return ret;
}

static int bma_set_delay(int delay)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (delay < 0 || delay > BMA250_MAX_DELAY) {
        return BMA_ERROR_ARG;
    } else if (delay < BMA250_MIN_DELAY) {
        delay = BMA250_MIN_DELAY;
    }

    bma250_lock();
    err = bma250_set_delay(delay);
    bma250_unlock();

    return err;
}

static int bma_get_offset(struct bma_vector *offset)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (offset == NULL) {
        return BMA_ERROR_ARG;
    }

    bma250_lock();
    err = bma250_get_offset(offset);
    bma250_unlock();

    return err;
}

static int bma_set_offset(struct bma_vector *offset)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (offset == NULL ||
        offset->v[0] < BMA250_ABSMIN_2G || BMA250_ABSMAX_2G < offset->v[0] ||
        offset->v[1] < BMA250_ABSMIN_2G || BMA250_ABSMAX_2G < offset->v[1] ||
        offset->v[2] < BMA250_ABSMIN_2G || BMA250_ABSMAX_2G < offset->v[2]) {
        return BMA_ERROR_ARG;
    }

    bma250_lock();
    err = bma250_set_offset(offset);
    bma250_unlock();

    return err;
}

static int bma_get_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_lock();
    err = bma250_get_enable();
    bma250_unlock();

    return err;
}

static int bma_set_enable(int enable)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (enable != 0) {
        enable = 1;
    }

    bma250_lock();
    err = bma250_set_enable(enable);
    bma250_unlock();

    return err;
}

static int bma_get_filter(struct bma_acc_filter *filter)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (filter == NULL) {
        return BMA_ERROR_ARG;
    }

    bma250_lock();
    err = bma250_get_filter(filter);
    bma250_unlock();

    return err;
}

static int bma_set_filter(struct bma_acc_filter *filter)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (filter == NULL || filter->threshold < 0 || filter->threshold > BMA250_ABSMAX_2G) {
        return BMA_ERROR_ARG;
    }

    bma250_lock();
    err = bma250_set_filter(filter);
    bma250_unlock();

    return err;
}

static int bma_get_filter_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_lock();
    err = bma250_get_filter_enable();
    bma250_unlock();

    return err;
}

static int bma_set_filter_enable(int enable)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (enable != 0) {
        enable = 1;
    }

    bma250_lock();
    err = bma250_set_filter_enable(enable);
    bma250_unlock();

    return err;
}

static int bma_get_position(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    bma250_lock();
    err = bma250_get_position();
    bma250_unlock();

    return err;
}

static int bma_set_position(int position)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (!((position >= 0) && (position <= 7))) {
        return BMA_ERROR_ARG;
    }

    bma250_lock();
    err = bma250_set_position(position);
    bma250_unlock();

    return err;
}

static int bma_measure(struct bma_acc_data *data)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    if (data == NULL) {
        return BMA_ERROR_ARG;
    }

    bma250_lock();
    err = bma250_measure(data->xyz.v, data->raw.v);
    bma250_unlock();

    return err;
}
#if DEBUG
static int bma_get_register(uint8_t adr, uint8_t *val)
{
    if (pcb == NULL) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }

    *val = bma250_read_reg_byte(adr);

    return BMA_NO_ERROR;
}
#endif
static int bma_set_calibration(signed char* data_cal, int cal_init)
{
    int err;
    // for debug
    printk("[diony] bma_set_calibration!!!! .\n");
         if (pcb == NULL) {
          return BMA_ERROR_NOT_INITIALIZED;
    }

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return BMA_ERROR_NOT_INITIALIZED;
    }
    bma250_lock();
    err = bma250_set_calibration(&data_cal,cal_init);
    bma250_unlock();
    
    return err;
}
static int bma_get_calibration(signed char* cal_result)
{
	int err;
	// for debug
	printk("[HSS] [%s] \n", __func__);
	if (pcb == NULL) {
		return BMA_ERROR_NOT_INITIALIZED;
	}

	/* Check initialize */
	if (acc_data.initialize == 0) {
		return BMA_ERROR_NOT_INITIALIZED;
	}
	bma250_lock();
	err = bma250_get_calibration(cal_result);
	bma250_unlock();
    
	return err;
}

/* --------------------------------------------------------------------------- */
/*  Global function                                                            */
/* --------------------------------------------------------------------------- */
int bma_acc_driver_init(struct bma_acc_driver *f)
{
    struct bma_acc_driver_callback *cbk;

    /* Check parameter */
    if (f == NULL) {
        return BMA_ERROR_ARG;
    }
    cbk = &f->callback;
    if (cbk->i2c_open == NULL ||
        cbk->i2c_close == NULL ||
        cbk->i2c_write == NULL ||
        cbk->i2c_read == NULL ||
        cbk->msleep == NULL) {
        return BMA_ERROR_ARG;
    }

    /* Clear intialize */
    bma250_term();

    /* Set callback interface */
    cb.callback = *cbk;

    /* Set driver interface */
    f->init = bma_init;
    f->term = bma_term;
    f->get_delay = bma_get_delay;
    f->set_delay = bma_set_delay;
    f->get_offset = bma_get_offset;
    f->set_offset = bma_set_offset;
    f->get_enable = bma_get_enable;
    f->set_enable = bma_set_enable;
    f->get_filter = bma_get_filter;
    f->set_filter = bma_set_filter;
    f->get_filter_enable = bma_get_filter_enable;
    f->set_filter_enable = bma_set_filter_enable;
    f->get_position = bma_get_position;
    f->set_position = bma_set_position;
    f->measure = bma_measure;
    f->set_calibration = bma_set_calibration;
    f->get_calibration = bma_get_calibration;    
#if DEBUG
    f->get_register = bma_get_register;
#endif
    pcb = &cb;

    return BMA_NO_ERROR;
}
