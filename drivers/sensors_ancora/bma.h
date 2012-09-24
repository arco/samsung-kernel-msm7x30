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

#ifndef __BMA_H__
#define __BMA_H__

/*----------------------------------------------------------------------------*/
/*                   Acceleration Calibration Configuration                   */
/*----------------------------------------------------------------------------*/
#define BMA_DEFAULT_ACCCALIB_LENGTH         (20)
#define BMA_DEFAULT_ACCCALIB_DISTORTION     (25000)
#define BMA_ACC_I2C_SLAVEADDR               (0x08)

#define BMA_VERSION                        "4.0.1"

/* -------------------------------------------------------------------------- */
/*  Typedef definition                                                        */
/* -------------------------------------------------------------------------- */

#if defined(__LINUX_KERNEL_DRIVER__)
#include <linux/types.h>
#elif defined(__ANDROID__)
#include <stdint.h>
#else
typedef signed char         int8_t;
typedef unsigned char       uint8_t;
typedef signed short        int16_t;
typedef unsigned short      uint16_t;
typedef signed int          int32_t;
typedef unsigned int        uint32_t;
#endif

/* -------------------------------------------------------------------------- */
/*  Macro definition                                                          */
/* -------------------------------------------------------------------------- */

/* Debugging */
#define DEBUG                               (0)

#if DEBUG
#ifdef __LINUX_KERNEL_DRIVER__
#include <linux/kernel.h>
#define YLOGD(args) (printk args )
#define YLOGI(args) (printk args )
#define YLOGE(args) (printk args )
#define YLOGW(args) (printk args )
#elif defined __ANDROID__
#include <cutils/log.h>
#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "bma"
#define YLOGD(args) (LOGD args )
#define YLOGI(args) (LOGI args )
#define YLOGE(args) (LOGE args )
#define YLOGW(args) (LOGW args )
#else /* __ANDROID__ */
#include <stdio.h>
#define YLOGD(args) (printf args )
#define YLOGI(args) (printf args )
#define YLOGE(args) (printf args )
#define YLOGW(args) (printf args )
#endif /* __ANDROID__ */
#else /* DEBUG */
#define YLOGD(args) 
#define YLOGI(args) 
#define YLOGW(args) 
#define YLOGE(args) 
#endif /* DEBUG */

#define BMA_REPORT_DATA                     (0x01)
#define BMA_REPORT_CALIB                    (0x02)
#define BMA_REPORT_OVERFLOW_OCCURED         (0x04)
#define BMA_REPORT_HARD_OFFSET_CHANGED      (0x08)
#define BMA_REPORT_CALIB_OFFSET_CHANGED     (0x10)

#define BMA_HARD_OFFSET_UNKNOWN             (0x7f)
#define BMA_CALIB_OFFSET_UNKNOWN            (0x7fffffff)

#define BMA_NO_ERROR                        (0)
#define BMA_ERROR_ARG                       (-1)
#define BMA_ERROR_NOT_INITIALIZED           (-2)
#define BMA_ERROR_BUSY                      (-3)
#define BMA_ERROR_I2C                       (-4)
#define BMA_ERROR_CHIP_ID                   (-5)
#define BMA_ERROR_NOT_ACTIVE                (-6)
#define BMA_ERROR_RESTARTSYS                (-7)
#define BMA_ERROR_HARDOFFSET_NOT_WRITTEN    (-8)
#define BMA_ERROR_ERROR                     (-128)

#ifndef NULL
#define NULL ((void*)(0))
#endif
#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (!(0))
#endif
#ifndef NELEMS
#define NELEMS(a) ((int)(sizeof(a)/sizeof(a[0])))
#endif
#ifndef ABS
#define ABS(a) ((a) > 0 ? (a) : -(a))
#endif
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/* -------------------------------------------------------------------------- */
/*  Structure definition                                                      */
/* -------------------------------------------------------------------------- */
struct bma_vector {
    int32_t v[3];
};
struct bma_matrix {
    int32_t matrix[9];
};
struct bma_acc_data {
    struct bma_vector xyz;
    struct bma_vector raw;
};
struct bma_acc_filter {
    int threshold; /* um/s^2 */
};

struct bma_acc_driver_callback {
    int (*lock)(void);
    int (*unlock)(void);
    int (*i2c_open)(void);
    int (*i2c_close)(void);
    int (*i2c_write)(uint8_t slave, uint8_t adr, const uint8_t *buf, int len);
    int (*i2c_read) (uint8_t slave, uint8_t adr, uint8_t *buf, int len);
    void (*msleep)(int msec);
};

struct bma_acc_driver {
    int (*init)(void);
    int (*term)(void);
    int (*get_delay)(void);
    int (*set_delay)(int delay);
    int (*get_offset)(struct bma_vector *offset);
    int (*set_offset)(struct bma_vector *offset);
    int (*get_enable)(void);
    int (*set_enable)(int enable);
    int (*get_filter)(struct bma_acc_filter *filter);
    int (*set_filter)(struct bma_acc_filter *filter);
    int (*get_filter_enable)(void);
    int (*set_filter_enable)(int enable);
    int (*get_position)(void);
    int (*set_position)(int position);
    int (*measure)(struct bma_acc_data *data);
    int (*set_calibration)(unsigned char* data_cal, int cal_init);
    int (*get_calibration)(signed char* cal_result);    
#if DEBUG
    int (*get_register)(uint8_t adr, uint8_t *val);
#endif
    struct bma_acc_driver_callback callback;
};

struct bma_acc_calibration_threshold {
    int32_t variation;
};

struct bma_acc_calibration_callback {
    int (*lock)(void);
    int (*unlock)(void);
};

struct bma_acc_calibration {
    int (*init)(void);
    int (*term)(void);
    int (*update)(struct bma_vector *acc);
    int (*get_offset)(struct bma_vector *offset);
    int (*set_offset)(struct bma_vector *offset);
    int (*get_threshold)(struct bma_acc_calibration_threshold *threshold);
    int (*set_threshold)(struct bma_acc_calibration_threshold *threshold);
    struct bma_acc_calibration_callback callback;
};

/* -------------------------------------------------------------------------- */
/*  Global function definition                                                */
/* -------------------------------------------------------------------------- */

int bma_acc_driver_init(struct bma_acc_driver *f);
int bma_acc_calibration_init(struct bma_acc_calibration *f);

#endif /* __BMA_H__ */
