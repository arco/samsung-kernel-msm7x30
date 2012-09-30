/*
 * Copyright (c) 2010 Yamaha Corporation
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

#define YAS_BMA222_VERSION                                                "4.0.0"
#define YAS_BMA222_NAME                                                  "bma222"
#define YAS_BMA222_SLAVEADR                                                  0x08
#define YAS_BMA222_RESOLUTION                                                  64

/* Axes data range  [um/s^2] */
#define YAS_BMA222_GRAVITY_EARTH                                          9806550
#define YAS_BMA222_ABSMIN_2G                      (-YAS_BMA222_GRAVITY_EARTH * 2)
#define YAS_BMA222_ABSMAX_2G                       (YAS_BMA222_GRAVITY_EARTH * 2)


/* Default parameters */
#define YAS_BMA222_DEFAULT_DELAY                                              100
#define YAS_BMA222_DEFAULT_POSITION                                             3
#define YAS_BMA222_DEFAULT_THRESHOLD                                       320000

#define YAS_BMA222_MAX_DELAY                                                  200
#define YAS_BMA222_MIN_DELAY                                                   10

/* Registers */
#define YAS_BMA222_CHIP_ID_REG                                               0x00
#define YAS_BMA222_CHIP_ID                                                   0x03

#define YAS_BMA222_SOFT_RESET_REG                                            0x14
#define YAS_BMA222_SOFT_RESET_VAL                                            0xb6

#define YAS_BMA222_POWERMODE_REG                                             0x11
#define YAS_BMA222_POWERMODE_MASK                                            0xc0
#define YAS_BMA222_POWERMODE_SHIFT                                              6
#define YAS_BMA222_POWERMODE_NORMAL                                             0
#define YAS_BMA222_POWERMODE_LOW                                                1
#define YAS_BMA222_POWERMODE_OFF                                                2

#define YAS_BMA222_DATA_ENBL_REG                                             0x17
#define YAS_BMA222_DATA_ENBL_MASK                                            0x10
#define YAS_BMA222_DATA_ENBL_SHIFT                                              4

#define YAS_BMA222_SLEEP_DUR_REG                                             0x11
#define YAS_BMA222_SLEEP_DUR_MASK                                            0x1e
#define YAS_BMA222_SLEEP_DUR_SHIFT                                              1
#define YAS_BMA222_SLEEP_DUR_0                                                  0
#define YAS_BMA222_SLEEP_DUR_1                                                  6
#define YAS_BMA222_SLEEP_DUR_2                                                  7
#define YAS_BMA222_SLEEP_DUR_4                                                  8
#define YAS_BMA222_SLEEP_DUR_6                                                  9
#define YAS_BMA222_SLEEP_DUR_10                                                10
#define YAS_BMA222_SLEEP_DUR_25                                                11
#define YAS_BMA222_SLEEP_DUR_50                                                12
#define YAS_BMA222_SLEEP_DUR_100                                               13
#define YAS_BMA222_SLEEP_DUR_500                                               14
#define YAS_BMA222_SLEEP_DUR_1000                                              15

#define YAS_BMA222_RANGE_REG                                                 0x0f
#define YAS_BMA222_RANGE_MASK                                                0x0f
#define YAS_BMA222_RANGE_SHIFT                                                  0
#define YAS_BMA222_RANGE_2G                                                     3
#define YAS_BMA222_RANGE_4G                                                     5
#define YAS_BMA222_RANGE_8G                                                     8
#define YAS_BMA222_RANGE_16G                                                   12

#define YAS_BMA222_BANDWIDTH_REG                                             0x10
#define YAS_BMA222_BANDWIDTH_MASK                                            0x1f
#define YAS_BMA222_BANDWIDTH_SHIFT                                              0
#define YAS_BMA222_BANDWIDTH_1000HZ                                            15
#define YAS_BMA222_BANDWIDTH_500HZ                                             14
#define YAS_BMA222_BANDWIDTH_250HZ                                             13
#define YAS_BMA222_BANDWIDTH_125HZ                                             12
#define YAS_BMA222_BANDWIDTH_63HZ                                              11
#define YAS_BMA222_BANDWIDTH_32HZ                                              10
#define YAS_BMA222_BANDWIDTH_16HZ                                               9
#define YAS_BMA222_BANDWIDTH_8HZ                                                8 
#define YAS_BMA222_ACC_REG                                                   0x02

#define YAS_BMA150_VERSION                                                "4.0.0"
#define YAS_BMA150_NAME                                                  "bma150"
#define YAS_BMA150_RESOLUTION                                                 256

/* Axes data range  [um/s^2] */
#define YAS_BMA150_GRAVITY_EARTH                                          9806550
#define YAS_BMA150_ABSMIN_2G                      (-YAS_BMA150_GRAVITY_EARTH * 2)
#define YAS_BMA150_ABSMAX_2G                       (YAS_BMA150_GRAVITY_EARTH * 2)


/* Default parameters */
#define YAS_BMA150_DEFAULT_DELAY                                              100
#define YAS_BMA150_DEFAULT_POSITION                                             1
#define YAS_BMA150_DEFAULT_THRESHOLD                                        80000

#define YAS_BMA150_MAX_DELAY                                                  200
#define YAS_BMA150_MIN_DELAY                                                   10

/* Registers */
#define YAS_BMA150_CHIP_ID_REG                                               0x00
#define YAS_BMA150_CHIP_ID                                                   0x02

#define YAS_BMA150_SOFT_RESET_REG                                            0x0a
#define YAS_BMA150_SOFT_RESET_MASK                                           0x02
#define YAS_BMA150_SOFT_RESET_SHIFT                                             1

#define YAS_BMA150_SLEEP_REG                                                 0x0a
#define YAS_BMA150_SLEEP_MASK                                                0x01
#define YAS_BMA150_SLEEP_SHIFT                                                  0

#define YAS_BMA150_RANGE_REG                                                 0x14
#define YAS_BMA150_RANGE_MASK                                                0x18
#define YAS_BMA150_RANGE_SHIFT                                                  3
#define YAS_BMA150_RANGE_2G                                                     0
#define YAS_BMA150_RANGE_4G                                                     1
#define YAS_BMA150_RANGE_8G                                                     2

#define YAS_BMA150_BANDWIDTH_REG                                             0x14
#define YAS_BMA150_BANDWIDTH_MASK                                            0x07
#define YAS_BMA150_BANDWIDTH_SHIFT                                              0
#define YAS_BMA150_BANDWIDTH_25HZ                                               0
#define YAS_BMA150_BANDWIDTH_50HZ                                               1
#define YAS_BMA150_BANDWIDTH_100HZ                                              2
#define YAS_BMA150_BANDWIDTH_190HZ                                              3
#define YAS_BMA150_BANDWIDTH_375HZ                                              4
#define YAS_BMA150_BANDWIDTH_750HZ                                              5
#define YAS_BMA150_BANDWIDTH_1500HZ                                             6
#define YAS_BMA150_ACC_REG                                                   0x02
#define YAS_BMA222_COMP_TARGET_OFFSET_X__POS        1
#define YAS_BMA222_COMP_TARGET_OFFSET_X__LEN        2
#define YAS_BMA222_COMP_TARGET_OFFSET_X__MSK        0x06
#define YAS_BMA222_COMP_TARGET_OFFSET_X__REG                       0x37
#define YAS_BMA222_COMP_TARGET_OFFSET_Y__POS        3
#define YAS_BMA222_COMP_TARGET_OFFSET_Y__LEN        2
#define YAS_BMA222_COMP_TARGET_OFFSET_Y__MSK        0x18
#define YAS_BMA222_COMP_TARGET_OFFSET_Y__REG                       0x37
#define YAS_BMA222_COMP_TARGET_OFFSET_Z__POS        5
#define YAS_BMA222_COMP_TARGET_OFFSET_Z__LEN        2
#define YAS_BMA222_COMP_TARGET_OFFSET_Z__MSK        0x60
#define YAS_BMA222_COMP_TARGET_OFFSET_Z__REG                       0x37
#define YAS_BMA222_OFFSET_FILT_X_REG                0x38
#define YAS_BMA222_OFFSET_FILT_Y_REG                0x39
#define YAS_BMA222_OFFSET_FILT_Z_REG                0x3A
#define YAS_BMA222_UNLOCK_EE_WRITE_SETTING__POS     0
#define YAS_BMA222_UNLOCK_EE_WRITE_SETTING__LEN     1
#define YAS_BMA222_UNLOCK_EE_WRITE_SETTING__MSK     0x01
#define YAS_BMA222_UNLOCK_EE_WRITE_SETTING__REG               0x33
#define YAS_BMA222_START_EE_WRITE_SETTING__POS      1
#define YAS_BMA222_START_EE_WRITE_SETTING__LEN      1
#define YAS_BMA222_START_EE_WRITE_SETTING__MSK      0x02
#define YAS_BMA222_START_EE_WRITE_SETTING__REG                 0x33
#define YAS_BMA222_EE_WRITE_SETTING_S__POS          2
#define YAS_BMA222_EE_WRITE_SETTING_S__LEN          1
#define YAS_BMA222_EE_WRITE_SETTING_S__MSK          0x04
#define YAS_BMA222_EEPROM_CTRL_REG                                      0x33
#define YAS_BMA222_EN_FAST_COMP__POS                5
#define YAS_BMA222_EN_FAST_COMP__LEN                2
#define YAS_BMA222_EN_FAST_COMP__MSK                0x60
#define YAS_BMA222_EN_FAST_COMP__REG                0x36
#define YAS_BMA222_FAST_COMP_RDY_S__POS             4
#define YAS_BMA222_FAST_COMP_RDY_S__LEN             1
#define YAS_BMA222_FAST_COMP_RDY_S__MSK             0x10
#define YAS_BMA222_FAST_COMP_RDY_S__REG             0x36
#define YAS_BMA222_OFFSET_CTRL_REG                  0x36
/*
        SMB380 API error codes
*/

#define E_SMB_NULL_PTR          (char)-127
#define E_COMM_RES              (char)-1
#define E_OUT_OF_RANGE          (char)-2
#define E_EEPROM_BUSY           (char)-3


/* [HSS] Additional Define */
#define ACC_BMA150_I2C_SLAVE_ADDRESS                                          0x38
#define ACC_BMA222_I2C_SLAVE_ADDRESS                                          0x08
extern int board_hw_revision;
/* Return type is True */
#define C_Successful_S8X                        (signed   char)0
/* return type is False */
#define C_Unsuccessful_S8X                      (signed   char)-1

#define YAS_BMA222_GET_BITSLICE(regvar, bitname)\
                        (regvar & bitname##__MSK) >> bitname##__POS


#define YAS_BMA222_SET_BITSLICE(regvar, bitname, val)\
                  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)

/* --------------------------------------------------------------------------- */
/*  Structure definition                                                       */
/* --------------------------------------------------------------------------- */
/* Output data rate */
struct yas_bma_odr {
    unsigned long delay;               /* min delay (msec) in the range of ODR */
    unsigned char odr;                 /* bandwidth register value             */
};

/* Axes data */
struct yas_bma_acceleration {
    int x;
    int y;
    int z;
    int x_raw;
    int y_raw;
    int z_raw;
};

/* Driver private data */
struct yas_bma_data {
    int initialize;
    int i2c_open;
    int enable;
    int delay;
    int position;
    int threshold;
    int filter_enable;
    struct yas_vector offset;
    struct yas_bma_acceleration last;
};

/* Sleep duration */
struct yas_bma_sd {
    uint8_t bw;
    uint8_t sd;
};

/* --------------------------------------------------------------------------- */
/*  Data                                                                       */
/* --------------------------------------------------------------------------- */
/* Control block */
static struct yas_acc_driver  cb;
static struct yas_acc_driver *pcb = NULL;
static struct yas_bma_data acc_data;

/* Output data rate */
static const struct yas_bma_odr yas_bma222_odr_tbl[] = {
    {1,   YAS_BMA222_BANDWIDTH_250HZ},
    {2,   YAS_BMA222_BANDWIDTH_125HZ},
    {4,   YAS_BMA222_BANDWIDTH_63HZ},
    {8,   YAS_BMA222_BANDWIDTH_32HZ},
    {16,  YAS_BMA222_BANDWIDTH_16HZ},
    {32,  YAS_BMA222_BANDWIDTH_8HZ},
    {64,  YAS_BMA222_BANDWIDTH_8HZ},
    {128, YAS_BMA222_BANDWIDTH_8HZ},
};

static const struct yas_bma_odr yas_bma150_odr_tbl[] = {
    {1,  YAS_BMA150_BANDWIDTH_1500HZ},
    {2,  YAS_BMA150_BANDWIDTH_750HZ},
    {3,  YAS_BMA150_BANDWIDTH_375HZ},
    {6,  YAS_BMA150_BANDWIDTH_190HZ},
    {10, YAS_BMA150_BANDWIDTH_100HZ},
    {20, YAS_BMA150_BANDWIDTH_50HZ},
    {40, YAS_BMA150_BANDWIDTH_25HZ},
};

/* Sleep duration */
static const struct yas_bma_sd yas_bma222_sd_table[] = {
    {YAS_BMA222_BANDWIDTH_8HZ    /* 128ms */, YAS_BMA222_SLEEP_DUR_100},
    {YAS_BMA222_BANDWIDTH_16HZ   /*  64ms */, YAS_BMA222_SLEEP_DUR_50},
    {YAS_BMA222_BANDWIDTH_32HZ   /*  32ms */, YAS_BMA222_SLEEP_DUR_25},
    {YAS_BMA222_BANDWIDTH_63HZ   /*  16ms */, YAS_BMA222_SLEEP_DUR_10},
    {YAS_BMA222_BANDWIDTH_125HZ  /*   8ms */, YAS_BMA222_SLEEP_DUR_6},
    {YAS_BMA222_BANDWIDTH_250HZ  /*   4ms */, YAS_BMA222_SLEEP_DUR_2},
    {YAS_BMA222_BANDWIDTH_500HZ  /*   2ms */, YAS_BMA222_SLEEP_DUR_1},
    {YAS_BMA222_BANDWIDTH_1000HZ /*   1ms */, YAS_BMA222_SLEEP_DUR_0},
};

/* Transformation matrix for chip mounting position */
static const int yas_bma_position_map[][3][3] = {
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
static void yas_bma222_init_data(void);
static int yas_bma222_ischg_enable(int);
static int yas_bma222_read_reg(unsigned char, unsigned char *, unsigned char);
static int yas_bma222_write_reg(unsigned char, unsigned char *, unsigned char);
static int yas_bma222_read_reg_byte(unsigned char);
static int yas_bma222_write_reg_byte(unsigned char, unsigned char);
static int yas_bma222_lock(void);
static int yas_bma222_unlock(void);
static int yas_bma222_i2c_open(void);
static int yas_bma222_i2c_close(void);
static int yas_bma222_msleep(int);
static int yas_bma222_power_up(void);
static int yas_bma222_power_down(void);
static int yas_bma222_init(void);
static int yas_bma222_term(void);
static int yas_bma222_get_delay(void);
static int yas_bma222_set_delay(int);
static int yas_bma222_get_offset(struct yas_vector *);
static int yas_bma222_set_offset(struct yas_vector *);
static int yas_bma222_get_enable(void);
static int yas_bma222_set_enable(int);
static int yas_bma222_get_filter(struct yas_acc_filter *);
static int yas_bma222_set_filter(struct yas_acc_filter *);
static int yas_bma222_get_filter_enable(void);
static int yas_bma222_set_filter_enable(int);
static int yas_bma222_get_position(void);
static int yas_bma222_set_position(int);
static int yas_bma222_measure(int *, int *);
#if DEBUG
static int yas_get_register(uint8_t, uint8_t *);
#endif

//110324  hm83.cho   For calibration
int yas_bma222_set_offset_target_x(unsigned char offsettarget);
/* EasyCASE ) */
/* EasyCASE ( 1175
   bma222_get_offset_target_x */
int yas_bma222_get_offset_target_x(unsigned char *offsettarget );
/* EasyCASE ) */
/* EasyCASE ( 1179
   bma222_set_offset_target_y */
int yas_bma222_set_offset_target_y(unsigned char offsettarget);
/* EasyCASE ) */
/* EasyCASE ( 1183
   bma222_get_offset_target_y */
int yas_bma222_get_offset_target_y(unsigned char *offsettarget );
/* EasyCASE ) */
/* EasyCASE ( 1185
   bma222_set_offset_target_z */
int yas_bma222_set_offset_target_z(unsigned char offsettarget);
/* EasyCASE ) */
/* EasyCASE ( 1187
   bma222_get_offset_target_z */
int yas_bma222_get_offset_target_z(unsigned char *offsettarget );
/* EasyCASE ) */
/* EasyCASE ( 1189
   bma222_set_offset_filt_x */
int yas_bma222_set_offset_filt_x(unsigned char offsetfilt);
/* EasyCASE ) */
/* EasyCASE ( 1191
   bma222_get_offset_filt_x */
int yas_bma222_get_offset_filt_x(unsigned char *offsetfilt );
/* EasyCASE ) */
/* EasyCASE ( 1195
   bma222_set_offset_filt_y */
int yas_bma222_set_offset_filt_y(unsigned char offsetfilt);
/* EasyCASE ) */
/* EasyCASE ( 1197
   bma222_get_offset_filt_y */
int yas_bma222_get_offset_filt_y(unsigned char *offsetfilt );
/* EasyCASE ) */
/* EasyCASE ( 1199
   bma222_set_offset_filt_z */
int yas_bma222_set_offset_filt_z(unsigned char offsetfilt);
/* EasyCASE ) */
/* EasyCASE ( 1201
   bma222_get_offset_filt_z */
int yas_bma222_get_offset_filt_z(unsigned char *offsetfilt );
int yas_bma222_set_ee_w(unsigned char eew);
int yas_bma222_set_ee_prog_trig(void);
int yas_bma222_get_eeprom_writing_status(unsigned char *eewrite );
int yas_bma222_get_cal_ready(unsigned char *calrdy );
int yas_bma222_set_cal_trigger(unsigned char caltrigger);

static void yas_bma150_init_data(void);
static int yas_bma150_ischg_enable(int);
static int yas_bma150_read_reg(unsigned char, unsigned char *, unsigned char);
static int yas_bma150_write_reg(unsigned char, unsigned char *, unsigned char);
static int yas_bma150_read_reg_byte(unsigned char);
static int yas_bma150_write_reg_byte(unsigned char, unsigned char);
static int yas_bma150_lock(void);
static int yas_bma150_unlock(void);
static int yas_bma150_i2c_open(void);
static int yas_bma150_i2c_close(void);
static int yas_bma150_msleep(int);
static int yas_bma150_power_up(void);
static int yas_bma150_power_down(void);
static int yas_bma150_init(void);
static int yas_bma150_term(void);
static int yas_bma150_get_delay(void);
static int yas_bma150_set_delay(int);
static int yas_bma150_get_offset(struct yas_vector *);
static int yas_bma150_set_offset(struct yas_vector *);
static int yas_bma150_get_enable(void);
static int yas_bma150_set_enable(int);
static int yas_bma150_get_filter(struct yas_acc_filter *);
static int yas_bma150_set_filter(struct yas_acc_filter *);
static int yas_bma150_get_filter_enable(void);
static int yas_bma150_set_filter_enable(int);
static int yas_bma150_get_position(void);
static int yas_bma150_set_position(int);
static int yas_bma150_measure(int *, int *);

static int yas_bma222_set_calibration(signed char*);
/* --------------------------------------------------------------------------- */
/*  Local functions                                                            */
/* --------------------------------------------------------------------------- */

static void yas_bma222_init_data(void) {
    acc_data.initialize = 0;
    acc_data.enable = 0;
    acc_data.delay = YAS_BMA222_DEFAULT_DELAY;
    acc_data.offset.v[0] = 0;
    acc_data.offset.v[1] = 0;
    acc_data.offset.v[2] = 0;
    acc_data.position = YAS_BMA222_DEFAULT_POSITION;
    acc_data.threshold = YAS_BMA222_DEFAULT_THRESHOLD;
    acc_data.filter_enable = 0;
    acc_data.last.x = 0;
    acc_data.last.y = 0;
    acc_data.last.z = 0;
    acc_data.last.x_raw = 0;
    acc_data.last.y_raw = 0;
    acc_data.last.z_raw = 0;
}

static int yas_bma222_ischg_enable(int enable)
{
    if (acc_data.enable == enable) {
        return 0;
    }

    return 1;
}

/* register access functions */
static int yas_bma222_read_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_read(ACC_BMA222_I2C_SLAVE_ADDRESS, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
}

static int yas_bma222_write_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_write(ACC_BMA222_I2C_SLAVE_ADDRESS, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
}

static int yas_bma222_read_reg_byte(unsigned char adr)
{
    unsigned char buf=0xff;
    int err;

    err = yas_bma222_read_reg(adr, &buf, 1);
    if (err == 0) {
        return buf;
    }

    return 0;
}

static int yas_bma222_write_reg_byte(unsigned char adr, unsigned char val)
{
    return yas_bma222_write_reg(adr, &val, 1);
}

#define yas_bma222_read_bits(r) \
    ((yas_bma222_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_bma222_update_bits(r,v) \
    yas_bma222_write_reg_byte(r##_REG, \
                           ((yas_bma222_read_reg_byte(r##_REG) & ~r##_MASK) | ((v) << r##_SHIFT)))

static int yas_bma222_lock(void)
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

static int yas_bma222_unlock(void)
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

static int yas_bma222_i2c_open(void)
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

static int yas_bma222_i2c_close(void)
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

static int yas_bma222_msleep(int msec)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;

    if (msec <= 0) {
        return YAS_ERROR_ARG;
    }

    cbk->msleep(msec);

    return YAS_NO_ERROR;
}

static int bma222_set_sleep_dur(unsigned char bw)
{
    int i;
    int delay = acc_data.delay;

    if (bw == YAS_BMA222_BANDWIDTH_8HZ) {
        if (1000 < delay && delay < 2000) {
            return YAS_BMA222_SLEEP_DUR_500;
        }
        if (2000 <= delay) {
            return YAS_BMA222_SLEEP_DUR_1000;
        }
    }
    for (i = 0; i < (int)(sizeof(yas_bma222_sd_table) / sizeof(struct yas_bma_sd)); i++) {
        if (yas_bma222_sd_table[i].bw == bw) {
            /* Success */
            return yas_bma222_sd_table[i].sd;
        }
    }

    /* Error */
    return -1;
}

static int yas_bma222_power_up(void)
{
    yas_bma222_update_bits(YAS_BMA222_DATA_ENBL, 1);
    yas_bma222_update_bits(YAS_BMA222_POWERMODE, YAS_BMA222_POWERMODE_NORMAL);

    return YAS_NO_ERROR;
}

static int yas_bma222_power_down(void)
{
    yas_bma222_update_bits(YAS_BMA222_DATA_ENBL, 0);
    yas_bma222_update_bits(YAS_BMA222_POWERMODE, YAS_BMA222_POWERMODE_OFF);

    return YAS_NO_ERROR;
}

static int yas_bma222_init(void)
{
    struct yas_acc_filter filter;
    int err;
    int id;

    /* Check intialize */
    if (acc_data.initialize == 1) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Init data */
    yas_bma222_init_data();

    /* Open i2c */
    err = yas_bma222_i2c_open();
    if (err != YAS_NO_ERROR) {
        return err;
    }

    /* Check id */
    id = yas_bma222_read_reg_byte(YAS_BMA222_CHIP_ID_REG);
    if (id != YAS_BMA222_CHIP_ID) {
        yas_bma222_i2c_close();
        return YAS_ERROR_CHIP_ID;
    }

    /* Reset chip */
    yas_bma222_write_reg_byte(YAS_BMA222_SOFT_RESET_REG, YAS_BMA222_SOFT_RESET_VAL);
    yas_bma222_msleep(1);

    /* Set axes range*/
    yas_bma222_update_bits(YAS_BMA222_RANGE, YAS_BMA222_RANGE_2G);

    acc_data.initialize = 1;

    yas_bma222_set_delay(YAS_BMA222_DEFAULT_DELAY);
    yas_bma222_set_position(YAS_BMA222_DEFAULT_POSITION);
    filter.threshold = YAS_BMA222_DEFAULT_THRESHOLD;
    yas_bma222_set_filter(&filter);

    return YAS_NO_ERROR;
}

static int yas_bma222_term(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma222_set_enable(0);

    /* Close I2C */
    yas_bma222_i2c_close();

    acc_data.initialize = 0;

    return YAS_NO_ERROR;
}

static int yas_bma222_get_delay(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.delay;
}

static int yas_bma222_set_delay(int delay)
{
    unsigned char odr;
    int i;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Determine optimum odr */
    for (i = 1; i < (int)(sizeof(yas_bma222_odr_tbl) / sizeof(struct yas_bma_odr)) &&
             delay >= (int)yas_bma222_odr_tbl[i].delay; i++)
        ;

    odr = yas_bma222_odr_tbl[i-1].odr;
    acc_data.delay = delay;

    if (yas_bma222_get_enable()) {
        yas_bma222_update_bits(YAS_BMA222_BANDWIDTH, odr);
        yas_bma222_update_bits(YAS_BMA222_SLEEP_DUR, bma222_set_sleep_dur(odr));
    } else {
        yas_bma222_power_up();
        yas_bma222_update_bits(YAS_BMA222_BANDWIDTH, odr);
        yas_bma222_update_bits(YAS_BMA222_SLEEP_DUR, bma222_set_sleep_dur(odr));
        yas_bma222_power_down();
    }

    return YAS_NO_ERROR;
}

static int yas_bma222_get_offset(struct yas_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    *offset = acc_data.offset;

    return YAS_NO_ERROR;
}

static int yas_bma222_set_offset(struct yas_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.offset = *offset;

    return YAS_NO_ERROR;
}

static int yas_bma222_get_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.enable;
}

static int yas_bma222_set_enable(int enable)
{
    int err;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (yas_bma222_ischg_enable(enable)) {
        if (enable) {
            /* Open i2c */
            err = yas_bma222_i2c_open();
            if (err != YAS_NO_ERROR) {
                return err;
            }
            /* Reset chip */
            yas_bma222_write_reg_byte(YAS_BMA222_SOFT_RESET_REG, YAS_BMA222_SOFT_RESET_VAL);
            yas_bma222_msleep(1);
            /* Set axes range*/
            yas_bma222_update_bits(YAS_BMA222_RANGE, YAS_BMA222_RANGE_2G);
            yas_bma222_set_delay(acc_data.delay);
            yas_bma222_power_up();
        } else {
            yas_bma222_power_down();
            err = yas_bma222_i2c_close();
            if (err != YAS_NO_ERROR) {
                return err;
            }
        }
    }

    acc_data.enable = enable;

    return YAS_NO_ERROR;
}

static int yas_bma222_get_filter(struct yas_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    filter->threshold = acc_data.threshold;

    return YAS_NO_ERROR;
}

static int yas_bma222_set_filter(struct yas_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.threshold = filter->threshold;

    return YAS_NO_ERROR;
}

static int yas_bma222_get_filter_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.filter_enable;
}

static int yas_bma222_set_filter_enable(int enable)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.filter_enable = enable;

    return YAS_NO_ERROR;
}

static int yas_bma222_get_position(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.position;
}

static int yas_bma222_set_position(int position)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.position = position;

    return YAS_NO_ERROR;
}

static int yas_bma222_data_filter(int data[], int raw[], struct yas_bma_acceleration *accel)
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

static int yas_bma222_measure(int *out_data, int *out_raw)
{
    struct yas_bma_acceleration accel;
    unsigned char buf[6];
    int32_t raw[3], data[3];
    int pos = acc_data.position;
    int i,j;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Read acceleration data */
    if (yas_bma222_read_reg(YAS_BMA222_ACC_REG, buf, 6) != 0) {
        for (i = 0; i < 3; i++) raw[i] = 0;
    } else {
        for (i = 0; i < 3; i++) raw[i] = *(int8_t *)&buf[i*2+1];
    }

    /* for X, Y, Z axis */
    for (i = 0; i < 3; i++) {
        /* coordinate transformation */
        data[i] = 0;
        for (j = 0; j < 3; j++) {
            data[i] += raw[j] * yas_bma_position_map[pos][i][j];
        }
        /* normalization */
        data[i] *= (YAS_BMA222_GRAVITY_EARTH / YAS_BMA222_RESOLUTION);
    }

    yas_bma222_data_filter(data, raw, &accel);

    out_data[0] = accel.x - acc_data.offset.v[0];
    out_data[1] = accel.y - acc_data.offset.v[1];
    out_data[2] = accel.z - acc_data.offset.v[2];
    out_raw[0] = accel.x_raw;
    out_raw[1] = accel.y_raw;
    out_raw[2] = accel.z_raw;
    acc_data.last = accel;
	//printk("[diony] x = %d, y= %d, z=%d,accel.\n",accel.x_raw,accel.y_raw,accel.z_raw);
    return YAS_NO_ERROR;
}

static void yas_bma150_init_data(void) {
    acc_data.initialize = 0;
    acc_data.enable = 0;
    acc_data.delay = YAS_BMA150_DEFAULT_DELAY;
    acc_data.position = YAS_BMA150_DEFAULT_POSITION;
    acc_data.threshold = YAS_BMA150_DEFAULT_THRESHOLD;
    acc_data.filter_enable = 0;
    acc_data.offset.v[0] = 0;
    acc_data.offset.v[1] = 0;
    acc_data.offset.v[2] = 0;
    acc_data.last.x = 0;
    acc_data.last.y = 0;
    acc_data.last.z = 0;
    acc_data.last.x_raw = 0;
    acc_data.last.y_raw = 0;
    acc_data.last.z_raw = 0;
}

static int yas_bma150_ischg_enable(int enable)
{
    if (acc_data.enable == enable) {
        return 0;
    }

    return 1;
}

/* register access functions */
static int yas_bma150_read_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_read(ACC_BMA150_I2C_SLAVE_ADDRESS, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
}

static int yas_bma150_write_reg(unsigned char adr, unsigned char *buf, unsigned char len)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;
    int err;

    if (acc_data.i2c_open) {
        err = cbk->i2c_write(ACC_BMA150_I2C_SLAVE_ADDRESS, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
}
static int yas_bma222_set_calibration(signed char* data_cal)
{
  // for debug
    printk("[diony] yas_bma222_set_calibration!!!! .\n");
    signed char tmp;
    data_cal[0] = data_cal[1] = 0;
    data_cal[2]=1;
    //yas_bma222_write_reg_byte(YAS_BMA222_SOFT_RESET_REG, YAS_BMA222_SOFT_RESET_VAL);
    //yas_bma222_msleep(1);
     /* Set axes range*/
    //yas_bma222_update_bits(YAS_BMA222_RANGE, YAS_BMA222_RANGE_2G);
    //yas_bma222_set_delay(acc_data.delay);
    //yas_bma222_update_bits(YAS_BMA222_DATA_ENBL, 1);
#ifdef DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
    printk(KERN_INFO "data are %d,%d,%d\n",data_cal[0],data_cal[1],data_cal[2]);
    printk(KERN_INFO "start x axis fast calibration\n");
#endif
    yas_bma222_set_offset_target_x(data_cal[0]);
    tmp=1;//selet x axis in cal_trigger
    yas_bma222_set_cal_trigger(tmp);
    do
    {
        mdelay(2);
       yas_bma222_get_cal_ready(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got cal ready flag is %d\n",tmp);
#endif  
    }while(tmp==0);
	
#ifdef DEBUG
   yas_bma222_get_offset_filt_x(&tmp);
    printk(KERN_INFO "x offset filt is %d\n",tmp);
    printk(KERN_INFO "x axis fast calibration finished\n");
    printk(KERN_INFO "start y axis fast calibration\n");
#endif
    yas_bma222_set_offset_target_y(data_cal[1]);
    //bma222_get_offset_target_y(&tmp);
    //printk(KERN_INFO "y offset is %d\n",tmp);
    //bma222_get_offset_filt_y(&tmp);
    //printk(KERN_INFO "y offset filt is %d\n",tmp);
    tmp=2;//selet y axis in cal_trigger
    yas_bma222_set_cal_trigger(tmp);
    do
    {
        mdelay(2); 
        yas_bma222_get_cal_ready(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got cal ready flag is %d\n",tmp);
#endif  
    }while(tmp==0);
	
#ifdef DEBUG
    yas_bma222_get_offset_filt_y(&tmp);
    printk(KERN_INFO "y offset filt is %d\n",tmp);
    printk(KERN_INFO "y axis fast calibration finished\n");
    printk(KERN_INFO "start z axis fast calibration\n");
#endif
    yas_bma222_set_offset_target_z(data_cal[2]);

    //bma222_get_offset_target_z(&tmp);
    //printk(KERN_INFO "z offset is %d\n",tmp);
    //bma222_get_offset_filt_z(&tmp);
    //printk(KERN_INFO "z offset filt is %d\n",tmp);
    tmp=3;//selet z axis in cal_trigger
    yas_bma222_set_cal_trigger(tmp);
    do
    {
        mdelay(2);   //for test 
        yas_bma222_get_cal_ready(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got cal ready flag is %d\n",tmp);
#endif  
    }while(tmp==0);
	
#ifdef DEBUG
    yas_bma222_get_offset_filt_z(&tmp);
    printk(KERN_INFO "z offset filt is %d\n",tmp);
    printk(KERN_INFO "z axis fast calibration finished\n");
    printk(KERN_INFO "store xyz offset to eeprom\n");
#endif
    tmp=1;//unlock eeprom
    yas_bma222_set_ee_w(tmp);
    yas_bma222_set_ee_prog_trig();//update eeprom
    do
    {
        mdelay(2); 
        yas_bma222_get_eeprom_writing_status(&tmp);
#ifdef DEBUG
        printk(KERN_INFO "wait 2ms and got eeprom writing status is %d\n",tmp);
#endif  
    }while(tmp==0);
	
    tmp=0;//lock eemprom
    yas_bma222_set_ee_w(tmp);
#ifdef DEBUG
    printk(KERN_INFO "eeprom writing is finished\n");
#endif  
    return 0;
}

static int yas_bma150_read_reg_byte(unsigned char adr)
{
    unsigned char buf;
    int err;

    err = yas_bma150_read_reg(adr, &buf, 1);
    if (err == 0) {
        return buf;
    }

    return 0;
}

static int yas_bma150_write_reg_byte(unsigned char adr, unsigned char val)
{
    return yas_bma150_write_reg(adr, &val, 1);
}

#define yas_bma150_read_bits(r) \
    ((yas_bma150_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_bma150_update_bits(r,v) \
    yas_bma150_write_reg_byte(r##_REG, \
                           ((yas_bma150_read_reg_byte(r##_REG) & ~r##_MASK) | ((v) << r##_SHIFT)))

static int yas_bma150_lock(void)
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
int yas_bma222_set_offset_target_x(unsigned char offsettarget)
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
   else
      {
      err = yas_bma222_read_reg(YAS_BMA222_COMP_TARGET_OFFSET_X__REG, &data, 1);
	  printk("[diony] yas_bma222_set_offset_target_data  = %d .  (read)  \n",data);
      data = YAS_BMA222_SET_BITSLICE(data, YAS_BMA222_COMP_TARGET_OFFSET_X, offsettarget );
	  printk("[diony] yas_bma222_set_offset_target_data  = %d .  (after BITSLICE,write)  \n",data);
      err =yas_bma222_write_reg(YAS_BMA222_COMP_TARGET_OFFSET_X__REG, &data, 1);
      }
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100369
   bma222_get_offset_target_x */
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
int yas_bma222_get_offset_target_x(unsigned char *offsettarget )
{
    int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = yas_bma222_read_reg(YAS_BMA222_COMP_TARGET_OFFSET_X__REG, &data, 1);
      data = YAS_BMA222_GET_BITSLICE(data,YAS_BMA222_COMP_TARGET_OFFSET_X);
      *offsettarget = data;
   }
   
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100368
   bma222_set_offset_target_y */
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
int yas_bma222_set_offset_target_y(unsigned char offsettarget)
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = yas_bma222_read_reg(YAS_BMA222_COMP_TARGET_OFFSET_Y__REG, &data, 1);
      data = YAS_BMA222_SET_BITSLICE(data, YAS_BMA222_COMP_TARGET_OFFSET_Y, offsettarget );
      err = yas_bma222_write_reg(YAS_BMA222_COMP_TARGET_OFFSET_Y__REG, &data, 1);
   }
   return err;
}


/* EasyCASE ) */
/* EasyCASE ( 100364
   bma222_get_offset_target_y */
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
int yas_bma222_get_offset_target_y(unsigned char *offsettarget )
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = yas_bma222_read_reg(YAS_BMA222_COMP_TARGET_OFFSET_Y__REG, &data, 1);
      data = YAS_BMA222_GET_BITSLICE(data, YAS_BMA222_COMP_TARGET_OFFSET_Y);
      *offsettarget = data;
   }
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100371
   bma222_set_offset_target_z */
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
int yas_bma222_set_offset_target_z(unsigned char offsettarget)
{
    int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
   else
   {
       err = yas_bma222_read_reg(YAS_BMA222_COMP_TARGET_OFFSET_Z__REG, &data, 1);
      data = YAS_BMA222_SET_BITSLICE(data, YAS_BMA222_COMP_TARGET_OFFSET_Z, offsettarget );
       err = yas_bma222_write_reg(YAS_BMA222_COMP_TARGET_OFFSET_Z__REG, &data, 1);
   }
   return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100372
   bma222_get_offset_target_z */
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
int yas_bma222_get_offset_target_z(unsigned char *offsettarget )
{
   int err=0;
   unsigned char data;
   /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
   else
   {
      err = yas_bma222_read_reg(YAS_BMA222_COMP_TARGET_OFFSET_Z__REG, &data, 1);
      data =YAS_BMA222_GET_BITSLICE(data, YAS_BMA222_COMP_TARGET_OFFSET_Z);
      *offsettarget = data;
   }
   return err;
}



int yas_bma222_set_cal_trigger(unsigned char caltrigger)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err = yas_bma222_read_reg(YAS_BMA222_EN_FAST_COMP__REG, &data, 1);
		printk("[diony] yas_bma222_set_cal_trigger  data = %d .  (read)  \n",data);
	 	data = YAS_BMA222_SET_BITSLICE(data, YAS_BMA222_EN_FAST_COMP, caltrigger );
		 printk("[diony] yas_bma222_set_cal_trigger  data = %d .  (after BITSLICE,write)  \n",data);
		err = yas_bma222_write_reg(YAS_BMA222_EN_FAST_COMP__REG, &data, 1);
	}
	return err;
}


int yas_bma222_get_cal_ready(unsigned char *calrdy )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err = yas_bma222_read_reg(YAS_BMA222_OFFSET_CTRL_REG, &data, 1);
		printk("[diony] yas_bma222_get_cal_ready data= %d (read) \n",data);
		data = YAS_BMA222_GET_BITSLICE(data, YAS_BMA222_FAST_COMP_RDY_S);
		printk("[diony] yas_bma222_get_cal_ready data = %d  (after BITSLICE)\n",data);
		*calrdy = data;
	}
	
	return err;
}


int yas_bma222_set_offset_filt_x(unsigned char offsetfilt)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = yas_bma222_write_reg(YAS_BMA222_OFFSET_FILT_X_REG, &data, 1);
    }
   	return err;
}

/* EasyCASE ) */
/* EasyCASE ( 100377
   bma222_get_offset_filt_x */
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
int yas_bma222_get_offset_filt_x(unsigned char *offsetfilt )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = yas_bma222_read_reg(YAS_BMA222_OFFSET_FILT_X_REG, &data, 1);
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100381
   bma222_set_offset_filt_y */
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
int yas_bma222_set_offset_filt_y(unsigned char offsetfilt)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = yas_bma222_write_reg(YAS_BMA222_OFFSET_FILT_Y_REG, &data, 1);
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100382
   bma222_get_offset_filt_y */
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
int yas_bma222_get_offset_filt_y(unsigned char *offsetfilt )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = yas_bma222_read_reg(YAS_BMA222_OFFSET_FILT_Y_REG, &data, 1);
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100384
   bma222_set_offset_filt_z */
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
int yas_bma222_set_offset_filt_z(unsigned char offsetfilt)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = yas_bma222_write_reg(YAS_BMA222_OFFSET_FILT_Z_REG, &data, 1);
    }
   	return err;
}
/* EasyCASE ) */
/* EasyCASE ( 100385
   bma222_get_offset_filt_z */
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
int yas_bma222_get_offset_filt_z(unsigned char *offsetfilt )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
   	else
    {
    	data =  offsetfilt;
    	err = yas_bma222_read_reg(YAS_BMA222_OFFSET_FILT_Z_REG, &data, 1);
    }
   	return err;
}
int yas_bma222_set_ee_w(unsigned char eew)
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err =  yas_bma222_read_reg(YAS_BMA222_UNLOCK_EE_WRITE_SETTING__REG, &data, 1);
		data = YAS_BMA222_SET_BITSLICE(data, YAS_BMA222_UNLOCK_EE_WRITE_SETTING, eew);
		err =  yas_bma222_write_reg(YAS_BMA222_UNLOCK_EE_WRITE_SETTING__REG, &data, 1);
	}
	return err;
}

int yas_bma222_set_ee_prog_trig(void)
{
	int err=0;
       unsigned char data;
	 unsigned char eeprog;
	eeprog = 0x01;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err =  yas_bma222_read_reg(YAS_BMA222_START_EE_WRITE_SETTING__REG, &data, 1);
		data = YAS_BMA222_SET_BITSLICE(data, YAS_BMA222_START_EE_WRITE_SETTING, eeprog);
		err =  yas_bma222_write_reg(YAS_BMA222_START_EE_WRITE_SETTING__REG, &data, 1);
	}
	return err;
}

int yas_bma222_get_eeprom_writing_status(unsigned char *eewrite )
{
	int err=0;
       unsigned char data;
      /* Check initialize */
       if (acc_data.initialize == 0) {
           return YAS_ERROR_NOT_INITIALIZED;
      }
	else
	{
		err =  yas_bma222_read_reg(YAS_BMA222_EEPROM_CTRL_REG, &data, 1);
		data = YAS_BMA222_GET_BITSLICE(data, YAS_BMA222_EE_WRITE_SETTING_S);
		*eewrite = data;
	}
	return err;
}
static int yas_bma150_unlock(void)
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

static int yas_bma150_i2c_open(void)
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

static int yas_bma150_i2c_close(void)
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

static int yas_bma150_msleep(int msec)
{
    struct yas_acc_driver_callback *cbk = &pcb->callback;

    if (msec <= 0) {
        return YAS_ERROR_ARG;
    }

    cbk->msleep(msec);

    return YAS_NO_ERROR;
}

static int yas_bma150_power_up(void)
{
    yas_bma150_update_bits(YAS_BMA150_SLEEP, 0);
    yas_bma150_msleep(1);

    return YAS_NO_ERROR;
}

static int yas_bma150_power_down(void)
{
    yas_bma150_update_bits(YAS_BMA150_SLEEP, 1);

    return YAS_NO_ERROR;
}

static int yas_bma150_init(void)
{
    struct yas_acc_filter filter;
    int err;
    int id;

    /* Check intialize */
    if (acc_data.initialize == 1) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Init data */
    yas_bma150_init_data();

    /* Open i2c */
    err = yas_bma150_i2c_open();
    if (err != YAS_NO_ERROR) {
        return err;
    }

    /* Check id */
    id = yas_bma150_read_reg_byte(YAS_BMA150_CHIP_ID_REG);
    if (id != YAS_BMA150_CHIP_ID) {
        yas_bma150_i2c_close();
        return YAS_ERROR_CHIP_ID;
    }

    /* Reset chip */
    yas_bma150_power_up();
    yas_bma150_write_reg_byte(YAS_BMA150_SOFT_RESET_REG, YAS_BMA150_SOFT_RESET_MASK);
    yas_bma150_msleep(1); /* wait 10us after soft_reset to start I2C transaction */

    /* Set axes range*/
    yas_bma150_update_bits(YAS_BMA150_RANGE, YAS_BMA150_RANGE_2G);

    acc_data.initialize = 1;

    yas_bma150_set_delay(YAS_BMA150_DEFAULT_DELAY);
    yas_bma150_set_position(YAS_BMA150_DEFAULT_POSITION);
    filter.threshold = YAS_BMA150_DEFAULT_THRESHOLD;
    yas_bma150_set_filter(&filter);
    yas_bma150_power_down();

    return YAS_NO_ERROR;
}

static int yas_bma150_term(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma150_set_enable(0);

    /* Close I2C */
    yas_bma150_i2c_close();

    acc_data.initialize = 0;

    return YAS_NO_ERROR;
}

static int yas_bma150_get_delay(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.delay;
}

static int yas_bma150_set_delay(int delay)
{
    unsigned char odr;
    int i;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Determine optimum odr */
    for (i = 1; i < (int)(sizeof(yas_bma150_odr_tbl) / sizeof(struct yas_bma_odr)) &&
             delay >= (int)yas_bma150_odr_tbl[i].delay; i++)
        ;

    odr = yas_bma150_odr_tbl[i-1].odr;
    acc_data.delay = delay;

    if (yas_bma150_get_enable()) {
        yas_bma150_update_bits(YAS_BMA150_BANDWIDTH, odr);
    } else {
        yas_bma150_power_up();
        yas_bma150_update_bits(YAS_BMA150_BANDWIDTH, odr);
        yas_bma150_power_down();
    }

    return YAS_NO_ERROR;
}

static int yas_bma150_get_offset(struct yas_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    *offset = acc_data.offset;

    return YAS_NO_ERROR;
}

static int yas_bma150_set_offset(struct yas_vector *offset)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.offset = *offset;

    return YAS_NO_ERROR;
}

static int yas_bma150_get_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.enable;
}

static int yas_bma150_set_enable(int enable)
{
    int err;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (yas_bma150_ischg_enable(enable)) {
        if (enable) {
            /* Open i2c */
            err = yas_bma150_i2c_open();
            if (err != YAS_NO_ERROR) {
                return err;
            }
            /* Reset chip */
            yas_bma150_power_up();
            yas_bma150_write_reg_byte(YAS_BMA150_SOFT_RESET_REG, YAS_BMA150_SOFT_RESET_MASK);
            yas_bma150_msleep(1); /* wait 10us after soft_reset to start I2C transaction */
            /* Set axes range*/
            yas_bma150_update_bits(YAS_BMA150_RANGE, YAS_BMA150_RANGE_2G);
            yas_bma150_set_delay(acc_data.delay);
            yas_bma150_power_up();
        } else {
            yas_bma150_power_down();
            err = yas_bma150_i2c_close();
            if (err != YAS_NO_ERROR) {
                return err;
            }
       }
    }

    acc_data.enable = enable;

    return YAS_NO_ERROR;
}

static int yas_bma150_get_filter(struct yas_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    filter->threshold = acc_data.threshold;

    return YAS_NO_ERROR;
}

static int yas_bma150_set_filter(struct yas_acc_filter *filter)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.threshold = filter->threshold;

    return YAS_NO_ERROR;
}

static int yas_bma150_get_filter_enable(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.filter_enable;
}

static int yas_bma150_set_filter_enable(int enable)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.filter_enable = enable;

    return YAS_NO_ERROR;
}

static int yas_bma150_get_position(void)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    return acc_data.position;
}

static int yas_bma150_set_position(int position)
{
    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    acc_data.position = position;

    return YAS_NO_ERROR;
}

static int yas_bma150_data_filter(int data[], int raw[], struct yas_bma_acceleration *accel)
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

static int yas_bma150_measure(int *out_data, int *out_raw)
{
    struct yas_bma_acceleration accel;
    unsigned char buf[6];
    int32_t raw[3], data[3];

    int pos = acc_data.position;
    int i,j;

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Read acceleration data */
    if (yas_bma150_read_reg(YAS_BMA150_ACC_REG, buf, 6) != 0) {
        for (i = 0; i < 3; i++) raw[i] = 0;
    } else {
        for (i = 0; i < 3; i++) raw[i] = ((int16_t)((buf[i*2+1] << 8)) | (buf[i*2] & 0xfe)) >> 6;
    }

    /* for X, Y, Z axis */
    for (i = 0; i < 3; i++) {
        /* coordinate transformation */
        data[i] = 0;
        for (j = 0; j < 3; j++) {
            data[i] += raw[j] * yas_bma_position_map[pos][i][j];
        }
        /* normalization */
        data[i] *= (YAS_BMA150_GRAVITY_EARTH / YAS_BMA150_RESOLUTION);
    }

    yas_bma150_data_filter(data, raw, &accel);

    out_data[0] = accel.x - acc_data.offset.v[0];
    out_data[1] = accel.y - acc_data.offset.v[1];
    out_data[2] = accel.z - acc_data.offset.v[2];
    out_raw[0] = accel.x_raw;
    out_raw[1] = accel.y_raw;
    out_raw[2] = accel.z_raw;
    acc_data.last = accel;

    return YAS_NO_ERROR;
}

/* --------------------------------------------------------------------------- */
static int yas_init(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_init();
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_init();
	    yas_bma150_unlock();
	}
    return err;
}

static int yas_term(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_term();
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_term();
	    yas_bma150_unlock();
	}

    return err;
}

static int yas_get_delay(void)
{
    int ret;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    ret = yas_bma222_get_delay();
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    ret = yas_bma150_get_delay();
	    yas_bma150_unlock();
    }

    return ret;
}

static int yas_set_delay(int delay)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    if (delay < 0 || delay > YAS_BMA222_MAX_DELAY) {
	        return YAS_ERROR_ARG;
	    } else if (delay < YAS_BMA222_MIN_DELAY) {
	        delay = YAS_BMA222_MIN_DELAY;
	    }

	    yas_bma222_lock();
	    err = yas_bma222_set_delay(delay);
	    yas_bma222_unlock();
		}
	else
	{
	    if (delay < 0 || delay > YAS_BMA150_MAX_DELAY) {
	        return YAS_ERROR_ARG;
	    } else if (delay < YAS_BMA150_MIN_DELAY) {
	        delay = YAS_BMA150_MIN_DELAY;
	    }

	    yas_bma150_lock();
	    err = yas_bma150_set_delay(delay);
	    yas_bma150_unlock();
	}

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_get_offset(offset);
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_get_offset(offset);
	    yas_bma150_unlock();
	}

    return err;
}

static int yas_set_offset(struct yas_vector *offset)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    if (offset == NULL ||
	        offset->v[0] < YAS_BMA222_ABSMIN_2G || YAS_BMA222_ABSMAX_2G < offset->v[0] ||
	        offset->v[1] < YAS_BMA222_ABSMIN_2G || YAS_BMA222_ABSMAX_2G < offset->v[1] ||
	        offset->v[2] < YAS_BMA222_ABSMIN_2G || YAS_BMA222_ABSMAX_2G < offset->v[2]) {
	        return YAS_ERROR_ARG;
	    }

	    yas_bma222_lock();
	    err = yas_bma222_set_offset(offset);
	    yas_bma222_unlock();
	}
	else
	{
	    if (offset == NULL ||
	        offset->v[0] < YAS_BMA150_ABSMIN_2G || YAS_BMA150_ABSMAX_2G < offset->v[0] ||
	        offset->v[1] < YAS_BMA150_ABSMIN_2G || YAS_BMA150_ABSMAX_2G < offset->v[1] ||
	        offset->v[2] < YAS_BMA150_ABSMIN_2G || YAS_BMA150_ABSMAX_2G < offset->v[2]) {
	        return YAS_ERROR_ARG;
	    }

	    yas_bma150_lock();
	    err = yas_bma150_set_offset(offset);
	    yas_bma150_unlock();
	}

    return err;
}

static int yas_get_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_get_enable();
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_get_enable();
	    yas_bma150_unlock();
	}

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_set_enable(enable);
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_set_enable(enable);
	    yas_bma150_unlock();
	}

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_get_filter(filter);
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_get_filter(filter);
	    yas_bma150_unlock();
	}

    return err;
}

static int yas_set_filter(struct yas_acc_filter *filter)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    if (filter == NULL || filter->threshold < 0 || filter->threshold > YAS_BMA222_ABSMAX_2G) {
	        return YAS_ERROR_ARG;
	    }

	    yas_bma222_lock();
	    err = yas_bma222_set_filter(filter);
	    yas_bma222_unlock();
	}
	else
	{
	    if (filter == NULL || filter->threshold < 0 || filter->threshold > YAS_BMA150_ABSMAX_2G) {
	        return YAS_ERROR_ARG;
	    }

	    yas_bma150_lock();
	    err = yas_bma150_set_filter(filter);
	    yas_bma150_unlock();
	}

    return err;
}

static int yas_get_filter_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_get_filter_enable();
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_get_filter_enable();
	    yas_bma150_unlock();
	}

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_set_filter_enable(enable);
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_set_filter_enable(enable);
	    yas_bma150_unlock();
	}

    return err;
}

static int yas_get_position(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_get_position();
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_get_position();
	    yas_bma150_unlock();
	}

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_set_position(position);
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_set_position(position);
	    yas_bma150_unlock();
    }

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	{
	    yas_bma222_lock();
	    err = yas_bma222_measure(data->xyz.v, data->raw.v);
	    yas_bma222_unlock();
	}
	else
	{
	    yas_bma150_lock();
	    err = yas_bma150_measure(data->xyz.v, data->raw.v);
	    yas_bma150_unlock();
	}

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

	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	    *val = yas_bma222_read_reg_byte(adr);
	else
	    *val = yas_bma150_read_reg_byte(adr);

    return YAS_NO_ERROR;
}
#endif
static int yas_set_calibration(signed char* data_cal)
{
    int err;
    // for debug
    printk("[diony] yas_set_calibration!!!! .\n");
         if (pcb == NULL) {
          return YAS_ERROR_NOT_INITIALIZED;
    }

    /* Check initialize */
    if (acc_data.initialize == 0) {
        return YAS_ERROR_NOT_INITIALIZED;
    }
    yas_bma222_lock();
    err = yas_bma222_set_calibration(&data_cal);
    yas_bma222_unlock();
    
    return err;
}
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
	if(board_hw_revision >1) /* [HSS][Galaxy S+] REV0.0 : BMA023,  REV0.1 : BMA222 */
	    yas_bma222_term();
	else
	    yas_bma150_term();

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
    f->set_calibration = yas_set_calibration;
#if DEBUG
    f->get_register = yas_get_register;
#endif
    pcb = &cb;

    return YAS_NO_ERROR;
}
