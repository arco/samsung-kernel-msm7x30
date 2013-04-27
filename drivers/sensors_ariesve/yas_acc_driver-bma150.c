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

/* --------------------------------------------------------------------------- */
/*  Structure definition                                                       */
/* --------------------------------------------------------------------------- */
/* Output data rate */
struct yas_bma150_odr {
    unsigned long delay;               /* min delay (msec) in the range of ODR */
    unsigned char odr;                 /* bandwidth register value             */
};

/* Axes data */
struct yas_bma150_acceleration {
    int x;
    int y;
    int z;
    int x_raw;
    int y_raw;
    int z_raw;
};

/* Driver private data */
struct yas_bma150_data {
    int initialize;
    int i2c_open;
    int enable;
    int delay;
    int position;
    int threshold;
    int filter_enable;
    struct yas_vector offset;
    struct yas_bma150_acceleration last;
};

/* --------------------------------------------------------------------------- */
/*  Data                                                                       */
/* --------------------------------------------------------------------------- */
/* Control block */
static struct yas_acc_driver  cb;
static struct yas_acc_driver *pcb = NULL;
static struct yas_bma150_data acc_data;

/* Output data rate */
static const struct yas_bma150_odr yas_bma150_odr_tbl[] = {
    {1,  YAS_BMA150_BANDWIDTH_1500HZ},
    {2,  YAS_BMA150_BANDWIDTH_750HZ},
    {3,  YAS_BMA150_BANDWIDTH_375HZ},
    {6,  YAS_BMA150_BANDWIDTH_190HZ},
    {10, YAS_BMA150_BANDWIDTH_100HZ},
    {20, YAS_BMA150_BANDWIDTH_50HZ},
    {40, YAS_BMA150_BANDWIDTH_25HZ},
};

/* Transformation matrix for chip mounting position */
static const int yas_bma150_position_map[][3][3] = {
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

/* --------------------------------------------------------------------------- */
/*  Local functions                                                            */
/* --------------------------------------------------------------------------- */

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
        err = cbk->i2c_read(YAS_ACC_I2C_SLAVEADDR, adr, buf, len);
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
        err = cbk->i2c_write(YAS_ACC_I2C_SLAVEADDR, adr, buf, len);
        if (err != 0) {
            return err;
        }

        return err;
    }

    return YAS_NO_ERROR;
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
    for (i = 1; i < (int)(sizeof(yas_bma150_odr_tbl) / sizeof(struct yas_bma150_odr)) &&
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

static int yas_bma150_data_filter(int data[], int raw[], struct yas_bma150_acceleration *accel)
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
    struct yas_bma150_acceleration accel;
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
            data[i] += raw[j] * yas_bma150_position_map[pos][i][j];
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

    yas_bma150_lock();
    err = yas_bma150_init();
    yas_bma150_unlock();

    return err;
}

static int yas_term(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma150_lock();
    err = yas_bma150_term();
    yas_bma150_unlock();

    return err;
}

static int yas_get_delay(void)
{
    int ret;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma150_lock();
    ret = yas_bma150_get_delay();
    yas_bma150_unlock();

    return ret;
}

static int yas_set_delay(int delay)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (delay < 0 || delay > YAS_BMA150_MAX_DELAY) {
        return YAS_ERROR_ARG;
    } else if (delay < YAS_BMA150_MIN_DELAY) {
        delay = YAS_BMA150_MIN_DELAY;
    }

    yas_bma150_lock();
    err = yas_bma150_set_delay(delay);
    yas_bma150_unlock();

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

    yas_bma150_lock();
    err = yas_bma150_get_offset(offset);
    yas_bma150_unlock();

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
        offset->v[0] < YAS_BMA150_ABSMIN_2G || YAS_BMA150_ABSMAX_2G < offset->v[0] ||
        offset->v[1] < YAS_BMA150_ABSMIN_2G || YAS_BMA150_ABSMAX_2G < offset->v[1] ||
        offset->v[2] < YAS_BMA150_ABSMIN_2G || YAS_BMA150_ABSMAX_2G < offset->v[2]) {
        return YAS_ERROR_ARG;
    }

    yas_bma150_lock();
    err = yas_bma150_set_offset(offset);
    yas_bma150_unlock();

    return err;
}

static int yas_get_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma150_lock();
    err = yas_bma150_get_enable();
    yas_bma150_unlock();

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

    yas_bma150_lock();
    err = yas_bma150_set_enable(enable);
    yas_bma150_unlock();

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

    yas_bma150_lock();
    err = yas_bma150_get_filter(filter);
    yas_bma150_unlock();

    return err;
}

static int yas_set_filter(struct yas_acc_filter *filter)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    if (filter == NULL || filter->threshold < 0 || filter->threshold > YAS_BMA150_ABSMAX_2G) {
        return YAS_ERROR_ARG;
    }

    yas_bma150_lock();
    err = yas_bma150_set_filter(filter);
    yas_bma150_unlock();

    return err;
}

static int yas_get_filter_enable(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma150_lock();
    err = yas_bma150_get_filter_enable();
    yas_bma150_unlock();

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

    yas_bma150_lock();
    err = yas_bma150_set_filter_enable(enable);
    yas_bma150_unlock();

    return err;
}

static int yas_get_position(void)
{
    int err;

    /* Check intialize */
    if (pcb == NULL) {
        return YAS_ERROR_NOT_INITIALIZED;
    }

    yas_bma150_lock();
    err = yas_bma150_get_position();
    yas_bma150_unlock();

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

    yas_bma150_lock();
    err = yas_bma150_set_position(position);
    yas_bma150_unlock();

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

    yas_bma150_lock();
    err = yas_bma150_measure(data->xyz.v, data->raw.v);
    yas_bma150_unlock();

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

    *val = yas_bma150_read_reg_byte(adr);

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
#if DEBUG
    f->get_register = yas_get_register;
#endif
    pcb = &cb;

    return YAS_NO_ERROR;
}
