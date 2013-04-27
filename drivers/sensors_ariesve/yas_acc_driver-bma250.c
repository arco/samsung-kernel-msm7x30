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

#include "yas.h"

#define YAS_BMA250_RESOLUTION                                                 256

/* Axes data range  [um/s^2] */
#define YAS_BMA250_GRAVITY_EARTH                                          9806550
#define YAS_BMA250_ABSMIN_2G                      (-YAS_BMA250_GRAVITY_EARTH * 2)
#define YAS_BMA250_ABSMAX_2G                       (YAS_BMA250_GRAVITY_EARTH * 2)


/* Default parameters */
#define YAS_BMA250_DEFAULT_DELAY                                              100
#define YAS_BMA250_DEFAULT_POSITION                                             3

#define YAS_BMA250_MAX_DELAY                                                  200
#define YAS_BMA250_MIN_DELAY                                                   10

/* Registers */
#define YAS_BMA250_CHIP_ID_REG                                               0x00
#define YAS_BMA250_CHIP_ID                                                   0x03

#define YAS_BMA250_SOFT_RESET_REG                                            0x14
#define YAS_BMA250_SOFT_RESET_VAL                                            0xb6

#define YAS_BMA250_POWERMODE_REG                                             0x11
#define YAS_BMA250_POWERMODE_MASK                                            0xc0
#define YAS_BMA250_POWERMODE_SHIFT                                              6
#define YAS_BMA250_POWERMODE_NORMAL                                             0
#define YAS_BMA250_POWERMODE_LOW                                                1
#define YAS_BMA250_POWERMODE_OFF                                                2

#define YAS_BMA250_DATA_ENBL_REG                                             0x17
#define YAS_BMA250_DATA_ENBL_MASK                                            0x10
#define YAS_BMA250_DATA_ENBL_SHIFT                                              4

#define YAS_BMA250_SLEEP_DUR_REG                                             0x11
#define YAS_BMA250_SLEEP_DUR_MASK                                            0x1e
#define YAS_BMA250_SLEEP_DUR_SHIFT                                              1
#define YAS_BMA250_SLEEP_DUR_0                                                  0
#define YAS_BMA250_SLEEP_DUR_1                                                  6
#define YAS_BMA250_SLEEP_DUR_2                                                  7
#define YAS_BMA250_SLEEP_DUR_4                                                  8
#define YAS_BMA250_SLEEP_DUR_6                                                  9
#define YAS_BMA250_SLEEP_DUR_10                                                10
#define YAS_BMA250_SLEEP_DUR_25                                                11
#define YAS_BMA250_SLEEP_DUR_50                                                12
#define YAS_BMA250_SLEEP_DUR_100                                               13
#define YAS_BMA250_SLEEP_DUR_500                                               14
#define YAS_BMA250_SLEEP_DUR_1000                                              15

#define YAS_BMA250_RANGE_REG                                                 0x0f
#define YAS_BMA250_RANGE_MASK                                                0x0f
#define YAS_BMA250_RANGE_SHIFT                                                  0
#define YAS_BMA250_RANGE_2G                                                     3
#define YAS_BMA250_RANGE_4G                                                     5
#define YAS_BMA250_RANGE_8G                                                     8
#define YAS_BMA250_RANGE_16G                                                   12

#define YAS_BMA250_BANDWIDTH_REG                                             0x10
#define YAS_BMA250_BANDWIDTH_MASK                                            0x1f
#define YAS_BMA250_BANDWIDTH_SHIFT                                              0
#define YAS_BMA250_BANDWIDTH_1000HZ                                            15
#define YAS_BMA250_BANDWIDTH_500HZ                                             14
#define YAS_BMA250_BANDWIDTH_250HZ                                             13
#define YAS_BMA250_BANDWIDTH_125HZ                                             12
#define YAS_BMA250_BANDWIDTH_63HZ                                              11
#define YAS_BMA250_BANDWIDTH_32HZ                                              10
#define YAS_BMA250_BANDWIDTH_16HZ                                               9
#define YAS_BMA250_BANDWIDTH_8HZ                                                8 
#define YAS_BMA250_ACC_REG                                                   0x02

#define BMA250_EEPROM_CTRL_REG          0x33
#define BMA250_EEPROM_CTRL_MASK         0x0f
#define BMA250_EEPROM_CTRL_SHIFT        0

/* SETTING THIS BIT  UNLOCK'S WRITING SETTING REGISTERS TO EEPROM */
#define BMA250_UNLOCK_EE_WRITE_SETTING_SHIFT   0
#define BMA250_UNLOCK_EE_WRITE_SETTING_LEN     1
#define BMA250_UNLOCK_EE_WRITE_SETTING_MASK    0x01
#define BMA250_UNLOCK_EE_WRITE_SETTING_REG     BMA250_EEPROM_CTRL_REG


/* SETTING THIS BIT STARTS WRITING SETTING REGISTERS TO EEPROM */
#define BMA250_START_EE_WRITE_SETTING_SHIFT    1
#define BMA250_START_EE_WRITE_SETTING_LEN      1
#define BMA250_START_EE_WRITE_SETTING_MASK     0x02
#define BMA250_START_EE_WRITE_SETTING_REG      BMA250_EEPROM_CTRL_REG


/* STATUS OF WRITING TO EEPROM */
#define BMA250_EE_WRITE_SETTING_S_SHIFT        2
#define BMA250_EE_WRITE_SETTING_S_LEN          1
#define BMA250_EE_WRITE_SETTING_S_MASK         0x04
#define BMA250_EE_WRITE_SETTING_S_REG          BMA250_EEPROM_CTRL_REG


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

#define BMA250_FAST_COMP_RDY_S_SHIFT           4
#define BMA250_FAST_COMP_RDY_S_LEN             1
#define BMA250_FAST_COMP_RDY_S_MASK            0x10
#define BMA250_FAST_COMP_RDY_S_REG             BMA250_OFFSET_CTRL_REG

/**    FAST COMPENSATION FOR X,Y,Z AXIS      **/

#define BMA250_EN_FAST_COMP_SHIFT              5
#define BMA250_EN_FAST_COMP_LEN                2
#define BMA250_EN_FAST_COMP_MASK               0x60
#define BMA250_EN_FAST_COMP_REG                BMA250_OFFSET_CTRL_REG

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

#define BMA250_COMP_TARGET_OFFSET_X_SHIFT      1
#define BMA250_COMP_TARGET_OFFSET_X_LEN        2
#define BMA250_COMP_TARGET_OFFSET_X_MASK       0x06
#define BMA250_COMP_TARGET_OFFSET_X_REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_COMP_TARGET_OFFSET_Y_SHIFT      3
#define BMA250_COMP_TARGET_OFFSET_Y_LEN        2
#define BMA250_COMP_TARGET_OFFSET_Y_MASK       0x18
#define BMA250_COMP_TARGET_OFFSET_Y_REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_COMP_TARGET_OFFSET_Z_SHIFT      5
#define BMA250_COMP_TARGET_OFFSET_Z_LEN        2
#define BMA250_COMP_TARGET_OFFSET_Z_MASK       0x60
#define BMA250_COMP_TARGET_OFFSET_Z_REG        BMA250_OFFSET_PARAMS_REG

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
struct yas_bma250_odr {
    unsigned long delay;               /* min delay (msec) in the range of ODR */
    unsigned char odr;                 /* bandwidth register value             */
};

/* Axes data */
struct yas_bma250_acceleration {
    int x;
    int y;
    int z;
    int x_raw;
    int y_raw;
    int z_raw;
};

/* Driver private data */
struct yas_bma250_data {
    int initialize;
    int i2c_open;
    int enable;
    int delay;
    int position;
    int threshold;
    int filter_enable;
    struct yas_vector offset;
    struct yas_bma250_acceleration last;
};

/* Sleep duration */
struct yas_bma250_sd {
    uint8_t bw;
    uint8_t sd;
};

/* --------------------------------------------------------------------------- */
/*  Data                                                                       */
/* --------------------------------------------------------------------------- */
/* Control block */
static struct yas_acc_driver  cb;
static struct yas_acc_driver *pcb = NULL;
static struct yas_bma250_data acc_data;

/* Output data rate */
static const struct yas_bma250_odr yas_bma250_odr_tbl[] = {
    {1,   YAS_BMA250_BANDWIDTH_1000HZ},
    {2,   YAS_BMA250_BANDWIDTH_500HZ},
    {4,   YAS_BMA250_BANDWIDTH_250HZ},
    {8,   YAS_BMA250_BANDWIDTH_125HZ},
    {16,  YAS_BMA250_BANDWIDTH_63HZ},
    {32,  YAS_BMA250_BANDWIDTH_32HZ},
    {64,  YAS_BMA250_BANDWIDTH_16HZ},
    {128, YAS_BMA250_BANDWIDTH_8HZ},
};

/* Sleep duration */
static const struct yas_bma250_sd yas_bma250_sd_table[] = {
    {YAS_BMA250_BANDWIDTH_8HZ    /* 128ms */, YAS_BMA250_SLEEP_DUR_100},
    {YAS_BMA250_BANDWIDTH_16HZ   /*  64ms */, YAS_BMA250_SLEEP_DUR_50},
    {YAS_BMA250_BANDWIDTH_32HZ   /*  32ms */, YAS_BMA250_SLEEP_DUR_25},
    {YAS_BMA250_BANDWIDTH_63HZ   /*  16ms */, YAS_BMA250_SLEEP_DUR_10},
    {YAS_BMA250_BANDWIDTH_125HZ  /*   8ms */, YAS_BMA250_SLEEP_DUR_6},
    {YAS_BMA250_BANDWIDTH_250HZ  /*   4ms */, YAS_BMA250_SLEEP_DUR_2},
    {YAS_BMA250_BANDWIDTH_500HZ  /*   2ms */, YAS_BMA250_SLEEP_DUR_1},
    {YAS_BMA250_BANDWIDTH_1000HZ /*   1ms */, YAS_BMA250_SLEEP_DUR_0},
};

/* Transformation matrix for chip mounting position */
static const int yas_bma250_position_map[][3][3] = {
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
static void yas_bma250_init_data(void);
static int yas_bma250_ischg_enable(int);
static int yas_bma250_read_reg(unsigned char, unsigned char *, unsigned char);
static int yas_bma250_write_reg(unsigned char, unsigned char *, unsigned char);
static int yas_bma250_read_reg_byte(unsigned char);
static int yas_bma250_write_reg_byte(unsigned char, unsigned char);
static int yas_bma250_lock(void);
static int yas_bma250_unlock(void);
static int yas_bma250_i2c_open(void);
static int yas_bma250_i2c_close(void);
static int yas_bma250_msleep(int);
static int yas_bma250_power_up(void);
static int yas_bma250_power_down(void);
static int yas_bma250_init(void);
static int yas_bma250_term(void);
static int yas_bma250_get_delay(void);
static int yas_bma250_set_delay(int);
static int yas_bma250_get_offset(struct yas_vector *);
static int yas_bma250_set_offset(struct yas_vector *);
static int yas_bma250_get_enable(void);
static int yas_bma250_set_enable(int);
static int yas_bma250_get_filter(struct yas_acc_filter *);
static int yas_bma250_set_filter(struct yas_acc_filter *);
static int yas_bma250_get_filter_enable(void);
static int yas_bma250_set_filter_enable(int);
static int yas_bma250_get_position(void);
static int yas_bma250_set_position(int);
static int yas_bma250_measure(int *, int *);
#if DEBUG
static int yas_get_register(uint8_t, uint8_t *);
#endif

/* --------------------------------------------------------------------------- */
/*  Local functions                                                            */
/* --------------------------------------------------------------------------- */

static void yas_bma250_init_data(void) {
    acc_data.initialize = 0;
    acc_data.enable = 0;
    acc_data.delay = YAS_BMA250_DEFAULT_DELAY;
    acc_data.offset.v[0] = 0;
    acc_data.offset.v[1] = 0;
    acc_data.offset.v[2] = 0;
    acc_data.position = YAS_BMA250_DEFAULT_POSITION;
    acc_data.threshold = YAS_ACC_DEFAULT_FILTER_THRESH;
    acc_data.filter_enable = 1;
    acc_data.last.x = 0;
    acc_data.last.y = 0;
    acc_data.last.z = 0;
    acc_data.last.x_raw = 0;
    acc_data.last.y_raw = 0;
    acc_data.last.z_raw = 0;
}

static int yas_bma250_ischg_enable(int enable)
{
    if (acc_data.enable == enable) {
        return 0;
    }

    return 1;
}

/* register access functions */
static int yas_bma250_read_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_read(YAS_ACC_I2C_SLAVEADDR, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
}

static int yas_bma250_write_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_write(YAS_ACC_I2C_SLAVEADDR, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
}

static int yas_bma250_read_reg_byte(unsigned char adr)
{
    unsigned char buf=0xff;
    int err;

    err = yas_bma250_read_reg(adr, &buf, 1);
    if (err == 0) {
        return buf;
    }

    return 0;
}

static int yas_bma250_write_reg_byte(unsigned char adr, unsigned char val)
{
    return yas_bma250_write_reg(adr, &val, 1);
}

#define yas_bma250_read_bits(r) \
    ((yas_bma250_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_bma250_update_bits(r,v) \
    yas_bma250_write_reg_byte(r##_REG, \
                           ((yas_bma250_read_reg_byte(r##_REG) & ~r##_MASK) | ((v) << r##_SHIFT)))

static int yas_bma250_lock(void)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (cbk->lock != NULL && cbk->unlock != NULL) {
        err = cbk->lock();
    } else {
        err = YAS_NO_ERROR;
    }

    return err;
}

static int yas_bma250_unlock(void)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (cbk->lock != NULL && cbk->unlock != NULL) {
        err = cbk->unlock();
    } else {
        err = YAS_NO_ERROR;
    }

    return err;
}

static int yas_bma250_i2c_open(void)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open == 0) {
        err = cbk->i2c_open();
        if (err != YAS_NO_ERROR) {
            return YAS_ERROR_I2C;
        }
        acc_data.i2c_open = 1;
    }

    return YAS_NO_ERROR;
}

static int yas_bma250_i2c_close(void)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open != 0) {
        err = cbk->i2c_close();
        if (err != YAS_NO_ERROR) {
            return YAS_ERROR_I2C;
        }
        acc_data.i2c_open = 0;
    }
    return YAS_NO_ERROR;
}

static int yas_bma250_msleep(int msec)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;

    if (msec <= 0) {
        return YAS_ERROR_ARG;
    }

    cbk->msleep(msec);

    return YAS_NO_ERROR;
}

static int bma250_set_sleep_dur(unsigned char bw)
{
    int i;
    int delay = acc_data.delay;

    if (bw == YAS_BMA250_BANDWIDTH_8HZ) {
        if (1000 < delay && delay < 2000) {
            return YAS_BMA250_SLEEP_DUR_500;
        }
        if (2000 <= delay) {
            return YAS_BMA250_SLEEP_DUR_1000;
        }
    }
    for (i = 0; i < (int)(sizeof(yas_bma250_sd_table) / sizeof(struct yas_bma250_sd)); i++) {
        if (yas_bma250_sd_table[i].bw == bw) {
            /* Success */
            return yas_bma250_sd_table[i].sd;
        }
    }

    /* Error */
    return -1;
}

static int yas_bma250_power_up(void)
{
    yas_bma250_update_bits(YAS_BMA250_DATA_ENBL, 1);
    yas_bma250_update_bits(YAS_BMA250_POWERMODE, YAS_BMA250_POWERMODE_NORMAL);

    return YAS_NO_ERROR;
}

static int yas_bma250_power_down(void)
{
    yas_bma250_update_bits(YAS_BMA250_DATA_ENBL, 0);
    yas_bma250_update_bits(YAS_BMA250_POWERMODE, YAS_BMA250_POWERMODE_OFF);

    return YAS_NO_ERROR;
}

static int yas_bma250_init(void)
{
    struct yas_acc_filter filter;
    int err;
    int id;
    
    printk("[acc]bma-250_init!!\n");
    /* Check intialize */
    if (acc_data.initialize == 1) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Init data */
    yas_bma250_init_data();

    /* Open i2c */
    err = yas_bma250_i2c_open();
    if (err != YAS_NO_ERROR) {
        return err;
    }

    /* Check id */
    id = yas_bma250_read_reg_byte(YAS_BMA250_CHIP_ID_REG);
    if (id != YAS_BMA250_CHIP_ID) {
        yas_bma250_i2c_close();
        return YAS_ERROR_CHIP_ID;
    }

    /* Reset chip */
    yas_bma250_write_reg_byte(YAS_BMA250_SOFT_RESET_REG, YAS_BMA250_SOFT_RESET_VAL);
    yas_bma250_msleep(1);

    /* Set axes range*/
    yas_bma250_update_bits(YAS_BMA250_RANGE, YAS_BMA250_RANGE_2G);

    acc_data.initialize = 1;

    yas_bma250_set_delay(YAS_BMA250_DEFAULT_DELAY);
    yas_bma250_set_position(YAS_BMA250_DEFAULT_POSITION);
    filter.threshold = YAS_ACC_DEFAULT_FILTER_THRESH;
    yas_bma250_set_filter(&filter);

    return YAS_NO_ERROR;
}

static int yas_bma250_term(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_set_enable(0);

    /* Close I2C */
    yas_bma250_i2c_close();

    acc_data.initialize = 0;

    return YAS_NO_ERROR;
}

static int yas_bma250_get_delay(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.delay;
}

static int yas_bma250_set_delay(int delay)
{
    unsigned char odr;
    int i;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Determine optimum odr */
    for (i = 1; i < (int)(sizeof(yas_bma250_odr_tbl) / sizeof(struct yas_bma250_odr)) &&
             delay >= (int)yas_bma250_odr_tbl[i].delay; i++)
        ;

    odr = yas_bma250_odr_tbl[i-1].odr;
    acc_data.delay = delay;

    if (yas_bma250_get_enable()) {
        yas_bma250_update_bits(YAS_BMA250_BANDWIDTH, odr);
        yas_bma250_update_bits(YAS_BMA250_SLEEP_DUR, bma250_set_sleep_dur(odr));
    } else {
        yas_bma250_power_up();
        yas_bma250_update_bits(YAS_BMA250_BANDWIDTH, odr);
        yas_bma250_update_bits(YAS_BMA250_SLEEP_DUR, bma250_set_sleep_dur(odr));
        yas_bma250_power_down();
    }

    return YAS_NO_ERROR;
}

static int yas_bma250_get_offset(struct yas_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    *offset = acc_data.offset;

    return YAS_NO_ERROR;
}

static int yas_bma250_set_offset(struct yas_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.offset = *offset;

    return YAS_NO_ERROR;
}

static int yas_bma250_get_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.enable;
}

static int yas_bma250_set_enable(int enable)
{
    int err;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (yas_bma250_ischg_enable(enable)) {
        if (enable) {
            /* Open i2c */
            err = yas_bma250_i2c_open();
            if (err != YAS_NO_ERROR) {
                return err;
            }
            /* Reset chip */
            yas_bma250_write_reg_byte(YAS_BMA250_SOFT_RESET_REG, YAS_BMA250_SOFT_RESET_VAL);
            yas_bma250_msleep(1);
            /* Set axes range*/
            yas_bma250_update_bits(YAS_BMA250_RANGE, YAS_BMA250_RANGE_2G);
            yas_bma250_set_delay(acc_data.delay);
            yas_bma250_power_up();
        } else {
            yas_bma250_power_down();
            err = yas_bma250_i2c_close();
            if (err != YAS_NO_ERROR) {
                return err;
            }
        }
    }

    acc_data.enable = enable;

    return YAS_NO_ERROR;
}

static int yas_bma250_get_filter(struct yas_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    filter->threshold = acc_data.threshold;

    return YAS_NO_ERROR;
}

static int yas_bma250_set_filter(struct yas_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.threshold = filter->threshold;

    return YAS_NO_ERROR;
}

static int yas_bma250_get_filter_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.filter_enable;
}

static int yas_bma250_set_filter_enable(int enable)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.filter_enable = enable;

    return YAS_NO_ERROR;
}

static int yas_bma250_get_position(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.position;
}

static int yas_bma250_set_position(int position)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.position = position;

    return YAS_NO_ERROR;
}

static int yas_bma250_data_filter(int data[], int raw[], struct yas_bma250_acceleration *accel)
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

    return YAS_NO_ERROR;
}

static int yas_bma250_measure(int *out_data, int *out_raw)
{
    struct yas_bma250_acceleration accel;
    unsigned char buf[6];
    int32_t raw[3], data[3];
    int pos = acc_data.position;
    int i,j;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Read acceleration data */
    if (yas_bma250_read_reg(YAS_BMA250_ACC_REG, buf, 6) != 0) {
        for (i = 0; i < 3; i++) raw[i] = 0;
    } else {
        for (i = 0; i < 3; i++) raw[i] = ((int16_t)((buf[i*2+1] << 8)) | (buf[i*2] & 0xfe)) >> 6;
    }

    /* for X, Y, Z axis */
    for (i = 0; i < 3; i++) {
        /* coordinate transformation */
        data[i] = 0;
        for (j = 0; j < 3; j++) {
            data[i] += raw[j] * yas_bma250_position_map[pos][i][j];
        }
        /* normalization */
        data[i] *= (YAS_BMA250_GRAVITY_EARTH / YAS_BMA250_RESOLUTION);
    }

    yas_bma250_data_filter(data, raw, &accel);

    out_data[0] = accel.x - acc_data.offset.v[0];
    out_data[1] = accel.y - acc_data.offset.v[1];
    out_data[2] = accel.z - acc_data.offset.v[2];
    out_raw[0] = accel.x_raw;
    out_raw[1] = accel.y_raw;
    out_raw[2] = accel.z_raw;
    acc_data.last = accel;

    return YAS_NO_ERROR;
}

static int bma250_fast_calibration(char layout[])
{
    char tmp = 1;	// select x axis in cal_trigger by default
	int power_off_after_calibration = 0;
	
    if(!yas_bma250_get_enable())
    {
        yas_bma250_power_up();
        power_off_after_calibration = 1;
    }
	
    yas_bma250_update_bits(BMA250_COMP_TARGET_OFFSET_X, layout[0]);

    yas_bma250_update_bits(BMA250_EN_FAST_COMP, tmp);
    do
    {
        mdelay(2);
        tmp = yas_bma250_read_bits(BMA250_FAST_COMP_RDY_S);
    } while(tmp == 0);

    yas_bma250_update_bits(BMA250_COMP_TARGET_OFFSET_Y, layout[1]);

    tmp = 2;	//selet y axis in cal_trigger
   yas_bma250_update_bits(BMA250_EN_FAST_COMP, tmp);
    do
    {
        mdelay(2); 
        tmp = yas_bma250_read_bits(BMA250_FAST_COMP_RDY_S);
    } while(tmp == 0);

    yas_bma250_update_bits(BMA250_COMP_TARGET_OFFSET_Z, layout[2]);
    tmp = 3;	//selet z axis in cal_trigger
    yas_bma250_update_bits(BMA250_EN_FAST_COMP, tmp);
    do
    {
        mdelay(2); 
        tmp = yas_bma250_read_bits(BMA250_FAST_COMP_RDY_S);
    } while(tmp == 0);

    tmp = 1;	//unlock eeprom
    yas_bma250_update_bits(BMA250_UNLOCK_EE_WRITE_SETTING, tmp);
    yas_bma250_update_bits(BMA250_START_EE_WRITE_SETTING, 0x01);

    do
    {
        mdelay(2); 
        tmp = yas_bma250_read_bits(BMA250_EE_WRITE_SETTING_S);
    } while(tmp==0);

    tmp = 0; 	//lock eemprom	
    yas_bma250_update_bits(BMA250_UNLOCK_EE_WRITE_SETTING, tmp);

    if(power_off_after_calibration)
    {
        yas_bma250_power_down();
    }
	
    return 0;
}

/* --------------------------------------------------------------------------- */
static int yas_init(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_lock();
    err = yas_bma250_init();
    yas_bma250_unlock();

    return err;
}

static int yas_term(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_lock();
    err = yas_bma250_term();
    yas_bma250_unlock();

    return err;
}

static int yas_get_delay(void)
{
    int ret;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_lock();
    ret = yas_bma250_get_delay();
    yas_bma250_unlock();

    return ret;
}

static int yas_set_delay(int delay)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (delay < 0 || delay > YAS_BMA250_MAX_DELAY) {
        return YAS_ERROR_ARG;
    } else if (delay < YAS_BMA250_MIN_DELAY) {
        delay = YAS_BMA250_MIN_DELAY;
    }

    yas_bma250_lock();
    err = yas_bma250_set_delay(delay);
    yas_bma250_unlock();

    return err;
}

static int yas_get_offset(struct yas_vector *offset)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (offset == NULL) {
        return YAS_ERROR_ARG;
    }

    yas_bma250_lock();
    err = yas_bma250_get_offset(offset);
    yas_bma250_unlock();

    return err;
}

static int yas_set_offset(struct yas_vector *offset)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (offset == NULL ||
        offset->v[0] < YAS_BMA250_ABSMIN_2G || YAS_BMA250_ABSMAX_2G < offset->v[0] ||
        offset->v[1] < YAS_BMA250_ABSMIN_2G || YAS_BMA250_ABSMAX_2G < offset->v[1] ||
        offset->v[2] < YAS_BMA250_ABSMIN_2G || YAS_BMA250_ABSMAX_2G < offset->v[2]) {
        return YAS_ERROR_ARG;
    }

    yas_bma250_lock();
    err = yas_bma250_set_offset(offset);
    yas_bma250_unlock();

    return err;
}

static int yas_get_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_lock();
    err = yas_bma250_get_enable();
    yas_bma250_unlock();

    return err;
}

static int yas_set_enable(int enable)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (enable != 0) {
        enable = 1;
    }

    yas_bma250_lock();
    err = yas_bma250_set_enable(enable);
    yas_bma250_unlock();

    return err;
}

static int yas_get_filter(struct yas_acc_filter *filter)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (filter == NULL) {
        return YAS_ERROR_ARG;
    }

    yas_bma250_lock();
    err = yas_bma250_get_filter(filter);
    yas_bma250_unlock();

    return err;
}

static int yas_set_filter(struct yas_acc_filter *filter)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (filter == NULL || filter->threshold < 0 || filter->threshold > YAS_BMA250_ABSMAX_2G) {
        return YAS_ERROR_ARG;
    }

    yas_bma250_lock();
    err = yas_bma250_set_filter(filter);
    yas_bma250_unlock();

    return err;
}

static int yas_get_filter_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_lock();
    err = yas_bma250_get_filter_enable();
    yas_bma250_unlock();

    return err;
}

static int yas_set_filter_enable(int enable)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (enable != 0) {
        enable = 1;
    }

    yas_bma250_lock();
    err = yas_bma250_set_filter_enable(enable);
    yas_bma250_unlock();

    return err;
}

static int yas_get_position(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma250_lock();
    err = yas_bma250_get_position();
    yas_bma250_unlock();

    return err;
}

static int yas_set_position(int position)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (!((position >= 0) && (position <= 7))) {
        return YAS_ERROR_ARG;
    }

    yas_bma250_lock();
    err = yas_bma250_set_position(position);
    yas_bma250_unlock();

    return err;
}

static int yas_measure(struct yas_acc_data *data)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (data == NULL) {
        return YAS_ERROR_ARG;
    }

    yas_bma250_lock();
    err = yas_bma250_measure(data->xyz.v, data->raw.v);
    yas_bma250_unlock();

    return err;
}
#if DEBUG
static int yas_get_register(uint8_t adr, uint8_t *val)
{
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    *val = yas_bma250_read_reg_byte(adr);

    return YAS_NO_ERROR;
}
#endif
/* --------------------------------------------------------------------------- */
/*  Global function                                                            */
/* --------------------------------------------------------------------------- */
int yas_acc_driver_init(struct yas_acc_driver *f)
{
    struct yas_acc_driver_callback *cbk;

    /* Check parameter */
    if (f == NULL) {
        return YAS_ERROR_ARG;
    }
    cbk = &f->callback;
    if (cbk->i2c_open == NULL ||
        cbk->i2c_close == NULL ||
        cbk->i2c_write == NULL ||
        cbk->i2c_read == NULL ||
        cbk->msleep == NULL) {
        return YAS_ERROR_ARG;
    }

    /* Clear intialize */
    yas_bma250_term();

    /* Set callback interface */
    cb.callback = *cbk;

    /* Set driver interface */
    f->init = yas_init;
    f->term = yas_term;
    f->get_delay = yas_get_delay;
    f->set_delay = yas_set_delay;
    f->get_offset = yas_get_offset;
    f->set_offset = yas_set_offset;
    f->get_enable = yas_get_enable;
    f->set_enable = yas_set_enable;
    f->get_filter = yas_get_filter;
    f->set_filter = yas_set_filter;
    f->get_filter_enable = yas_get_filter_enable;
    f->set_filter_enable = yas_set_filter_enable;
    f->get_position = yas_get_position;
    f->set_position = yas_set_position;
    f->measure = yas_measure;
#if DEBUG
    f->get_register = yas_get_register;
#endif
    pcb = &cb;

    return YAS_NO_ERROR;
}
