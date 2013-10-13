/*
 * Copyright (c) 2010-2012 Yamaha Corporation
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

struct utimeval {
	int32_t tv_sec;
	int32_t tv_msec;
};

struct utimer {
	struct utimeval prev_time;
	struct utimeval total_time;
	struct utimeval delay_ms;
};

static int utimeval_init(struct utimeval *val);
static int utimeval_is_initial(struct utimeval *val);
static int utimeval_is_overflow(struct utimeval *val);
static struct utimeval utimeval_plus(struct utimeval *first,
		struct utimeval *second);
static struct utimeval utimeval_minus(struct utimeval *first,
		struct utimeval *second);
static int utimeval_greater_than(struct utimeval *first,
		struct utimeval *second);
static int utimeval_greater_or_equal(struct utimeval *first,
		struct utimeval *second);
static int utimeval_greater_than_zero(struct utimeval *val);
static int utimeval_less_than_zero(struct utimeval *val);
static struct utimeval *msec_to_utimeval(struct utimeval *result,
		uint32_t msec);
static uint32_t utimeval_to_msec(struct utimeval *val);

static struct utimeval utimer_calc_next_time(struct utimer *ut,
	struct utimeval *cur);
static struct utimeval utimer_current_time(void);
static int utimer_is_timeout(struct utimer *ut);
static int utimer_clear_timeout(struct utimer *ut);
static uint32_t utimer_get_total_time(struct utimer *ut);
static uint32_t utimer_get_delay(struct utimer *ut);
static int utimer_set_delay(struct utimer *ut, uint32_t delay_ms);
static int utimer_update(struct utimer *ut);
static int utimer_update_with_curtime(struct utimer *ut, struct utimeval *cur);
static uint32_t utimer_sleep_time(struct utimer *ut);
static uint32_t utimer_sleep_time_with_curtime(struct utimer *ut,
		struct utimeval *cur);
static int utimer_init(struct utimer *ut, uint32_t delay_ms);
static int utimer_clear(struct utimer *ut);
static void utimer_lib_init(void (*func)(int *sec, int *msec));

#define YAS_CDRV_CENTER_X  512
#define YAS_CDRV_CENTER_Y1 512
#define YAS_CDRV_CENTER_Y2 512
#define YAS_CDRV_CENTER_T  256
#define YAS_CDRV_CENTER_I1 512
#define YAS_CDRV_CENTER_I2 512
#define YAS_CDRV_CENTER_I3 512

#define YAS_CDRV_HARDOFFSET_MEASURE_OF_VALUE	33
#define YAS_CDRV_HARDOFFSET_MEASURE_UF_VALUE	 0
#define YAS_CDRV_NORMAL_MEASURE_OF_VALUE	1024
#define YAS_CDRV_NORMAL_MEASURE_UF_VALUE	1

#define MS3CDRV_CMD_MEASURE_HARDOFFSET		0x1
#define MS3CDRV_CMD_MEASURE_XY1Y2T		0x2

#define MS3CDRV_RDSEL_MEASURE			0xc0
#define MS3CDRV_RDSEL_CALREGISTER		0xc8

#define MS3CDRV_WAIT_MEASURE_HARDOFFSET		2 /*  1.5[ms] */
#define MS3CDRV_WAIT_MEASURE_XY1Y2T		13 /* 12.3[ms] */

#define MS3CDRV_GSENSOR_INITIALIZED		(0x01)
#define MS3CDRV_MSENSOR_INITIALIZED		(0x02)

#define YAS_CDRV_MEASURE_X_OFUF			0x1
#define YAS_CDRV_MEASURE_Y1_OFUF		0x2
#define YAS_CDRV_MEASURE_Y2_OFUF		0x4

struct yas_machdep_func {
	int (*device_open)(void);
	int (*device_close)(void);
	int (*device_write)(const uint8_t *buf, int len);
	int (*device_read)(uint8_t *buf, int len);
	void (*msleep)(int msec);
};

static int yas_cdrv_actuate_initcoil(void);
static int yas_cdrv_set_offset(const int8_t *offset);
static int yas_cdrv_recalc_calib_offset(int32_t *prev_calib_offset,
		int32_t *new_calib_offset, int8_t *prev_offset,
		int8_t *new_offset);
static int yas_cdrv_set_transformatiom_matrix(const int8_t *transform);
static int yas_cdrv_measure_offset(int8_t *offset);
static int yas_cdrv_measure(int32_t *msens, int32_t *raw, int16_t *t);
static int yas_cdrv_init(const int8_t *transform,
		struct yas_machdep_func *func);
static int yas_cdrv_term(void);

static void (*current_time)(int *sec, int *msec);

static int
utimeval_init(struct utimeval *val)
{
	if (val == NULL)
		return -1;
	val->tv_sec = val->tv_msec = 0;
	return 0;
}

static int
utimeval_is_initial(struct utimeval *val)
{
	if (val == NULL)
		return 0;
	return val->tv_sec == 0 && val->tv_msec == 0;
}

static int
utimeval_is_overflow(struct utimeval *val)
{
	int32_t max;

	if (val == NULL)
		return 0;

	max = (int32_t) ((uint32_t) 0xffffffff / (uint32_t) 1000);
	if (val->tv_sec > max) {
		return 1; /* overflow */
	} else if (val->tv_sec == max) {
		if (val->tv_msec > (int32_t)((uint32_t)0xffffffff
					% (uint32_t)1000))
			return 1; /* overflow */
	}

	return 0;
}

static struct utimeval
utimeval_plus(struct utimeval *first, struct utimeval *second)
{
	struct utimeval result = {0, 0};
	int32_t tmp;

	if (first == NULL || second == NULL)
		return result;

	tmp = first->tv_sec + second->tv_sec;
	if (first->tv_sec >= 0 && second->tv_sec >= 0 && tmp < 0)
		goto overflow;
	if (first->tv_sec < 0 && second->tv_sec < 0 && tmp >= 0)
		goto underflow;

	result.tv_sec = tmp;
	result.tv_msec = first->tv_msec + second->tv_msec;
	if (1000 <= result.tv_msec) {
		tmp = result.tv_sec + result.tv_msec / 1000;
		if (result.tv_sec >= 0 && result.tv_msec >= 0 && tmp < 0)
			goto overflow;
		result.tv_sec = tmp;
		result.tv_msec = result.tv_msec % 1000;
	}
	if (result.tv_msec < 0) {
		tmp = result.tv_sec + result.tv_msec / 1000 - 1;
		if (result.tv_sec < 0 && result.tv_msec < 0 && tmp >= 0)
			goto underflow;
		result.tv_sec = tmp;
		result.tv_msec = result.tv_msec % 1000 + 1000;
	}

	return result;

overflow:
	result.tv_sec = (int32_t)0x7fffffff;
	result.tv_msec = 999;
	return result;

underflow:
	result.tv_sec = (int32_t)0x80000000;
	result.tv_msec = 0;
	return result;
}

static struct utimeval
utimeval_minus(struct utimeval *first, struct utimeval *second)
{
	struct utimeval result = {0, 0}, tmp;

	if (first == NULL || second == NULL
			|| second->tv_sec == (int)0x80000000)
		return result;

	tmp.tv_sec = -second->tv_sec;
	tmp.tv_msec = -second->tv_msec;
	return utimeval_plus(first, &tmp);
}

static int
utimeval_less_than(struct utimeval *first, struct utimeval *second)
{
	if (first == NULL || second == NULL)
		return 0;

	if (first->tv_sec > second->tv_sec) {
		return 1;
	} else if (first->tv_sec < second->tv_sec) {
		return 0;
	} else {
		if (first->tv_msec > second->tv_msec)
			return 1;
		else
			return 0;
	}
}

static int
utimeval_greater_than(struct utimeval *first, struct utimeval *second)
{
	if (first == NULL || second == NULL)
		return 0;

	if (first->tv_sec < second->tv_sec) {
		return 1;
	} else if (first->tv_sec > second->tv_sec) {
		return 0;
	} else {
		if (first->tv_msec < second->tv_msec)
			return 1;
		else
			return 0;
	}
}

static int
utimeval_greater_or_equal(struct utimeval *first,
						 struct utimeval *second)
{
	return !utimeval_less_than(first, second);
}

static int
utimeval_greater_than_zero(struct utimeval *val)
{
	struct utimeval zero = {0, 0};
	return utimeval_greater_than(&zero, val);
}

static int
utimeval_less_than_zero(struct utimeval *val)
{
	struct utimeval zero = {0, 0};
	return utimeval_less_than(&zero, val);
}

static struct utimeval *
msec_to_utimeval(struct utimeval *result, uint32_t msec)
{
	if (result == NULL)
		return result;
	result->tv_sec = (int32_t)(msec / 1000);
	result->tv_msec = (int32_t)(msec % 1000);

	return result;
}

static uint32_t
utimeval_to_msec(struct utimeval *val)
{
	if (val == NULL)
		return 0;
	if (utimeval_less_than_zero(val))
		return 0;
	if (utimeval_is_overflow(val))
		return 0xffffffff;

	return (uint32_t)(val->tv_sec * 1000 + val->tv_msec);
}

static struct utimeval
utimer_calc_next_time(struct utimer *ut, struct utimeval *cur)
{
	struct utimeval result = {0, 0}, delay;

	if (ut == NULL || cur == NULL)
		return result;

	utimer_update_with_curtime(ut, cur);
	if (utimer_is_timeout(ut)) {
		result = *cur;
	} else {
		delay = utimeval_minus(&ut->delay_ms, &ut->total_time);
		result = utimeval_plus(cur, &delay);
	}

	return result;
}

static struct utimeval
utimer_current_time(void)
{
	struct utimeval tv;
	int sec, msec;

	if (current_time != NULL)
		current_time(&sec, &msec);
	else
		sec = 0, msec = 0;

	tv.tv_sec = sec;
	tv.tv_msec = msec;

	return tv;
}

static int
utimer_clear(struct utimer *ut)
{
	if (ut == NULL)
		return -1;
	utimeval_init(&ut->prev_time);
	utimeval_init(&ut->total_time);

	return 0;
}

static int
utimer_update_with_curtime(struct utimer *ut, struct utimeval *cur)
{
	struct utimeval tmp;

	if (ut == NULL || cur == NULL)
		return -1;
	if (utimeval_is_initial(&ut->prev_time))
		ut->prev_time = *cur;
	if (utimeval_greater_than_zero(&ut->delay_ms)) {
		tmp = utimeval_minus(cur, &ut->prev_time);
		if (utimeval_less_than_zero(&tmp)) {
			utimeval_init(&ut->total_time);
		} else {
			ut->total_time = utimeval_plus(&tmp, &ut->total_time);
			if (utimeval_is_overflow(&ut->total_time))
				utimeval_init(&ut->total_time);
		}
		ut->prev_time = *cur;
	}

	return 0;
}

static int
utimer_update(struct utimer *ut)
{
	struct utimeval cur;

	if (ut == NULL)
		return -1;
	cur = utimer_current_time();
	utimer_update_with_curtime(ut, &cur);
	return 0;
}

static int
utimer_is_timeout(struct utimer *ut)
{
	if (ut == NULL)
		return 0;
	if (utimeval_greater_than_zero(&ut->delay_ms))
		return utimeval_greater_or_equal(&ut->delay_ms,
				&ut->total_time);
	else
		return 1;
}

static int
utimer_clear_timeout(struct utimer *ut)
{
	uint32_t delay, total;

	if (ut == NULL)
		return -1;

	delay = utimeval_to_msec(&ut->delay_ms);
	if (delay == 0 || utimeval_is_overflow(&ut->total_time)) {
		total = 0;
	} else {
		if (utimeval_is_overflow(&ut->total_time)) {
			total = 0;
		} else {
			total = utimeval_to_msec(&ut->total_time);
			total = total % delay;
		}
	}
	msec_to_utimeval(&ut->total_time, total);

	return 0;
}

static uint32_t
utimer_sleep_time_with_curtime(struct utimer *ut, struct utimeval *cur)
{
	struct utimeval tv;

	if (ut == NULL || cur == NULL)
		return 0;
	tv = utimer_calc_next_time(ut, cur);
	tv = utimeval_minus(&tv, cur);
	if (utimeval_less_than_zero(&tv))
		return 0;

	return utimeval_to_msec(&tv);
}

static uint32_t
utimer_sleep_time(struct utimer *ut)
{
	struct utimeval cur;

	if (ut == NULL)
		return 0;

	cur = utimer_current_time();
	return utimer_sleep_time_with_curtime(ut, &cur);
}

static int
utimer_init(struct utimer *ut, uint32_t delay_ms)
{
	if (ut == NULL)
		return -1;
	utimer_clear(ut);
	msec_to_utimeval(&ut->delay_ms, delay_ms);

	return 0;
}

static uint32_t
utimer_get_total_time(struct utimer *ut)
{
	return utimeval_to_msec(&ut->total_time);
}

static uint32_t
utimer_get_delay(struct utimer *ut)
{
	if (ut == NULL)
		return 0;
	return utimeval_to_msec(&ut->delay_ms);
}

static int
utimer_set_delay(struct utimer *ut, uint32_t delay_ms)
{
	return utimer_init(ut, delay_ms);
}

static void
utimer_lib_init(void (*func)(int *sec, int *msec))
{
	current_time = func;
}


struct yas_cdriver {
	uint8_t raw_calreg[9];
	int8_t roughoffset_is_set;
	int16_t matrix[9];
	int16_t correction_m[9];
	int8_t temp_coeff[3];
	int8_t initialized;
	int16_t temperature;
	struct yas_machdep_func func;
};

static struct yas_cdriver cdriver = {
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
	0,
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0},
	0,
	-1,
	{NULL, NULL, NULL, NULL, NULL}
};


static int
device_open(void)
{
	if (cdriver.func.device_open == NULL)
		return -1;
	return cdriver.func.device_open();
}

static int
device_close(void)
{
	if (cdriver.func.device_close == NULL)
		return -1;
	return cdriver.func.device_close();
}

static int
device_write(const uint8_t *buf, int len)
{
	if (cdriver.func.device_write == NULL)
		return -1;
	return cdriver.func.device_write(buf, len);
}

static int
device_read(uint8_t *buf, int len)
{
	if (cdriver.func.device_read == NULL)
		return -1;
	return cdriver.func.device_read(buf, len);
}

static void
sleep(int millisec)
{
	if (cdriver.func.msleep == NULL)
		return;
	cdriver.func.msleep(millisec);
}

/*------------------------------------------------------------------------------
 *                      Compact Driver Measure Functions
 *----------------------------------------------------------------------------*/

static int
yas_cdrv_check_transformation_matrix(const int8_t *p)
{
	int i, j;
	int n;

	for (i = 0; i < 3; ++i) {
		n = 0;
		for (j = 0; j < 3; ++j) {
			if (p[i*3 + j] == 1 || p[i*3 + j] == -1)
				n++;
		}
		if (n != 1)
			return -1;
	}

	for (i = 0; i < 3; ++i) {
		n = 0;
		for (j = 0; j < 3; ++j) {
			if (p[j*3 + i] == 1 || p[j*3 + i] == -1)
				n++;
		}
		if (n != 1)
			return -1;
	}

	return YAS_NO_ERROR;
}

static int
yas_cdrv_make_correction_matrix(const int8_t *p, const int16_t *matrix,
		int16_t *ans)
{
	int i, j, k;
	int16_t temp16;

	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j) {
			temp16 = 0;
			for (k = 0; k < 3; ++k)
				temp16 = (int16_t)(temp16 + p[i*3 + k]
						* matrix[k*3 + j]);
			ans[i*3 + j] = temp16;
		}
	}

	return YAS_NO_ERROR;
}

static int
yas_cdrv_transform(const int16_t *matrix, const int32_t *raw, int32_t *data)
{
	int i, j;
	int32_t temp;

	for (i = 0; i < 3; ++i) {
		temp = 0;
		for (j = 0; j < 3; ++j)
			temp += (int32_t)matrix[i*3 + j] * raw[j];
		data[i] = temp;
	}

	return YAS_NO_ERROR;
}

static int
yas_cdrv_msens_correction(const int32_t *raw, uint16_t temperature,
		int32_t *data)
{
	static const int16_t center16[3] = {
		16 * YAS_CDRV_CENTER_X,
		16 * YAS_CDRV_CENTER_Y1,
		16 * YAS_CDRV_CENTER_Y2
	};
	int32_t temp_rawdata[3];
	int32_t raw_xyz[3];
	int32_t temps32;
	int i;

	for (i = 0; i < 3; i++)
		temp_rawdata[i] = raw[i];

	for (i = 0; i < 3; ++i) {
		temp_rawdata[i] <<= 4;
		temp_rawdata[i] -= center16[i];

		/*
		  Memo:
		  The number '3 / 100' is approximated to '0x7ae1 / 2^20'.
		*/

		temps32 = ((int32_t)temperature - YAS_CDRV_CENTER_T)
			* cdriver.temp_coeff[i] * 0x7ae1;
		if (temps32 >= 0)
			temp_rawdata[i] -= (int16_t)(temps32 >> 16);
		else
			temp_rawdata[i] += (int16_t)((-temps32) >> 16);
	}

	raw_xyz[0] = -temp_rawdata[0];
	raw_xyz[1] = temp_rawdata[2] - temp_rawdata[1];
	raw_xyz[2] = temp_rawdata[2] + temp_rawdata[1];

	yas_cdrv_transform(cdriver.correction_m, raw_xyz, data);

	for (i = 0; i < 3; ++i)
		data[i] /= 1600;

	return YAS_NO_ERROR;
}

static int
yas_cdrv_actuate_initcoil(void)
{
	int i;
	static const uint8_t InitCoilTable[16] = {
		0x90, 0x81, 0x91, 0x82, 0x92, 0x83, 0x93, 0x84,
		0x94, 0x85, 0x95, 0x86, 0x96, 0x87, 0x97, 0x80
	};

	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	for (i = 0; i < 16; ++i) {
		if (device_write(&InitCoilTable[i], 1) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	return YAS_NO_ERROR;
}

static int
yas_cdrv_measure_offset(int8_t *offset)
{
	int i;
	uint8_t dat;
	uint8_t buf[6];
	int rv = YAS_NO_ERROR;

	if (offset == NULL)
		return YAS_ERROR_ARG;

	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	dat = MS3CDRV_RDSEL_MEASURE;
	if (device_write(&dat, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

	dat = MS3CDRV_CMD_MEASURE_HARDOFFSET;
	if (device_write(&dat, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

    sleep(MS3CDRV_WAIT_MEASURE_HARDOFFSET);

	if (device_read(buf, 6) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
    
	if (buf[0] & 0x80)
		return YAS_ERROR_BUSY;

	for (i = 0; i < 3; ++i)
		offset[2 - i] = (int8_t)(((buf[i << 1] & 0x7) << 8)
				| buf[(i << 1) | 1]);

	if (offset[0] <= YAS_CDRV_HARDOFFSET_MEASURE_UF_VALUE
		|| offset[0] >= YAS_CDRV_HARDOFFSET_MEASURE_OF_VALUE)
		rv |= YAS_CDRV_MEASURE_X_OFUF;
	if (offset[1] <= YAS_CDRV_HARDOFFSET_MEASURE_UF_VALUE
		|| offset[1] >= YAS_CDRV_HARDOFFSET_MEASURE_OF_VALUE)
		rv |= YAS_CDRV_MEASURE_Y1_OFUF;
	if (offset[2] <= YAS_CDRV_HARDOFFSET_MEASURE_UF_VALUE
		|| offset[2] >= YAS_CDRV_HARDOFFSET_MEASURE_OF_VALUE)
		rv |= YAS_CDRV_MEASURE_Y2_OFUF;

	return rv;
}

static int
yas_cdrv_set_offset(const int8_t *offset)
{
	int i;
	uint8_t dat;
	static const uint8_t addr[3] = { 0x20, 0x40, 0x60};
	uint8_t tmp[3];
	int rv = YAS_NO_ERROR;

	if (offset == NULL)
		return YAS_ERROR_ARG;
	if (offset[0] > 32 || offset[1] > 32 || offset[2] > 32)
		return YAS_ERROR_ARG;

	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	for (i = 0; i < 3; i++)
		tmp[i] = (uint8_t)offset[i];

	for (i = 0; i < 3; ++i) {
		if (tmp[i] <= 5)
			tmp[i] = 0;
		else
			tmp[i] = (uint8_t)(tmp[i] - 5);
	}
	for (i = 0; i < 3; ++i) {
		dat = addr[i] | tmp[i];
		if (device_write(&dat, 1) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	cdriver.roughoffset_is_set = 1;

	return rv;
}

static int
yas_cdrv_measure_preparation(void)
{
	uint8_t dat;

	dat = MS3CDRV_RDSEL_MEASURE;
	if (device_write(&dat, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

	return YAS_NO_ERROR;
}

static int
yas_cdrv_measure_sub(int32_t *msens, int32_t *raw, int16_t *t)
{
	uint8_t dat;
	uint8_t buf[8];
	uint8_t rv = YAS_NO_ERROR;
	int32_t temp_msens[3];
	int i;

	if (cdriver.roughoffset_is_set == 0)
		return YAS_ERROR_HARDOFFSET_NOT_WRITTEN;

	dat = MS3CDRV_CMD_MEASURE_XY1Y2T;
	if (device_write(&dat, 1) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

	for (i = 0; i < MS3CDRV_WAIT_MEASURE_XY1Y2T; i++) {
		sleep(1);

		if (device_read(buf, 8) < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		if (!(buf[0] & 0x80))
			break;
	}

	if (buf[0] & 0x80)
		return YAS_ERROR_BUSY;

	for (i = 0; i < 3; ++i)
		temp_msens[2 - i]
			= ((buf[i << 1] & 0x7) << 8) + buf[(i << 1) | 1];
	cdriver.temperature = (int16_t)(((buf[6] & 0x7) << 8) + buf[7]);

	if (temp_msens[0] <= YAS_CDRV_NORMAL_MEASURE_UF_VALUE
		|| temp_msens[0] >= YAS_CDRV_NORMAL_MEASURE_OF_VALUE) {
		rv |= YAS_CDRV_MEASURE_X_OFUF;
	}
	if (temp_msens[1] <= YAS_CDRV_NORMAL_MEASURE_UF_VALUE
		|| temp_msens[1] >= YAS_CDRV_NORMAL_MEASURE_OF_VALUE) {
		rv |= YAS_CDRV_MEASURE_Y1_OFUF;
	}
	if (temp_msens[2] <= YAS_CDRV_NORMAL_MEASURE_UF_VALUE
		|| temp_msens[2] >= YAS_CDRV_NORMAL_MEASURE_OF_VALUE) {
		rv |= YAS_CDRV_MEASURE_Y2_OFUF;
	}

	yas_cdrv_msens_correction(temp_msens, (uint16_t)cdriver.temperature,
			msens);

	if (raw != NULL) {
		for (i = 0; i < 3; i++)
			raw[i] = temp_msens[i];
	}
	if (t != NULL)
		*t = cdriver.temperature;

	return (int)rv;
}

static int
yas_cdrv_measure(int32_t *msens, int32_t *raw, int16_t *t)
{
	int rv, i;

	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (msens == NULL)
		return YAS_ERROR_ARG;

	rv = yas_cdrv_measure_preparation();
	if (rv < 0)
		return rv;

	rv = yas_cdrv_measure_sub(msens, raw, t);
	for (i = 0; i < 3; i++)
		msens[i] *= 400; /* typically msens * 400 is nT in unit */

	return rv;
}

static int
yas_cdrv_recalc_calib_offset(int32_t *prev_calib_offset,
		int32_t *new_calib_offset, int8_t *prev_offset,
		int8_t *new_offset)
{
	int32_t tmp[3], resolution[9], base[3];
	int32_t raw[3];
	int32_t diff, i;

	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	if (prev_calib_offset == NULL || new_calib_offset == NULL
			|| prev_offset == NULL || new_offset == NULL) {
		return YAS_ERROR_ARG;
	}

	raw[0] = raw[1] = raw[2] = 0;
	yas_cdrv_msens_correction(raw, (uint16_t)cdriver.temperature, base);
	for (i = 0; i < 3; i++) {
		raw[0] = raw[1] = raw[2] = 0;
		raw[i] = 226;
		yas_cdrv_msens_correction(raw, (uint16_t)cdriver.temperature,
				&resolution[i*3]);
		resolution[i*3 + 0] -= base[0];
		resolution[i*3 + 1] -= base[1];
		resolution[i*3 + 2] -= base[2];
	}

	for (i = 0; i < 3; i++)
		tmp[i] = prev_calib_offset[i] / 400; /* nT -> count */
	for (i = 0; i < 3; i++) {
		diff = (int32_t)new_offset[i] - (int32_t)prev_offset[i];
		while (diff > 0) {
			tmp[0] -= resolution[i*3 + 0];
			tmp[1] -= resolution[i*3 + 1];
			tmp[2] -= resolution[i*3 + 2];
			diff--;
		}
		while (diff < 0) {
			tmp[0] += resolution[i*3 + 0];
			tmp[1] += resolution[i*3 + 1];
			tmp[2] += resolution[i*3 + 2];
			diff++;
		}
	}
	for (i = 0; i < 3; i++)
		new_calib_offset[i] = tmp[i] * 400; /* count -> nT */

	return YAS_NO_ERROR;
}

static int
yas_cdrv_set_transformatiom_matrix(const int8_t *transform)
{
	if (transform == NULL)
		return YAS_ERROR_ARG;
	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (yas_cdrv_check_transformation_matrix(transform) < 0)
		return YAS_ERROR_ARG;

	yas_cdrv_make_correction_matrix(transform, cdriver.matrix,
			cdriver.correction_m);

	return YAS_NO_ERROR;
}

static int
yas_cdrv_init(const int8_t *transform, struct yas_machdep_func *func)
{
	int i;
	uint8_t dat;
	uint8_t *buf = cdriver.raw_calreg;
	uint8_t tempu8;

	if (transform == NULL || func == NULL)
		return YAS_ERROR_ARG;
	cdriver.func = *func;
	if (yas_cdrv_check_transformation_matrix(transform) < 0)
		return YAS_ERROR_ARG;

	cdriver.roughoffset_is_set = 0;

	if (!cdriver.initialized) {
		if (device_open() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	/* preparation to read CAL register */
	dat = MS3CDRV_CMD_MEASURE_HARDOFFSET;
	if (device_write(&dat, 1) < 0) {
		device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	sleep(MS3CDRV_WAIT_MEASURE_HARDOFFSET);

	dat = MS3CDRV_RDSEL_CALREGISTER;
	if (device_write(&dat, 1) < 0) {
		device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	/* dummy read */
	if (device_read(buf, 9) < 0) {
		device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	if (device_read(buf, 9) < 0) {
		device_close();
		return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	cdriver.matrix[0] = 100;

	tempu8 = (buf[0] & 0xfc) >> 2;
	cdriver.matrix[1] = (int16_t)(tempu8 - 0x20);

	tempu8 = (uint8_t)(((buf[0] & 0x3) << 2) | ((buf[1] & 0xc0) >> 6));
	cdriver.matrix[2] = (int16_t)(tempu8 - 8);

	tempu8 = (uint8_t)(buf[1] & 0x3f);
	cdriver.matrix[3] = (int16_t)(tempu8 - 0x20);

	tempu8 = (buf[2] & 0xfc) >> 2;
	cdriver.matrix[4] = (int16_t)(tempu8 + 38);

	tempu8 = (uint8_t)(((buf[2] & 0x3) << 4) | (buf[3] & 0xf0) >> 4);
	cdriver.matrix[5] = (int16_t)(tempu8 - 0x20);

	tempu8 = (uint8_t)(((buf[3] & 0xf) << 2) | ((buf[4] & 0xc0) >> 6));
	cdriver.matrix[6] = (int16_t)(tempu8 - 0x20);

	tempu8 = (uint8_t)(buf[4] & 0x3f);
	cdriver.matrix[7] = (int16_t)(tempu8 - 0x20);

	tempu8 = (buf[5] & 0xfe) >> 1;
	cdriver.matrix[8] = (int16_t)(tempu8 + 66);

	yas_cdrv_make_correction_matrix(transform, cdriver.matrix,
			cdriver.correction_m);

	for (i = 0; i < 3; ++i)
		cdriver.temp_coeff[i] = (int8_t)((int16_t)buf[i + 6] - 0x80);

	cdriver.initialized = 1;

	return YAS_NO_ERROR;
}

static int
yas_cdrv_term(void)
{
	if (!cdriver.initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	cdriver.initialized = 0;
	if (device_close() < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;

	return YAS_NO_ERROR;
}

#define MEASURE_RESULT_OVER_UNDER_FLOW		(0x07)

#define YAS_DEFAULT_CALIB_INTERVAL		(50)	/* 50 msecs */
#define YAS_DEFAULT_DATA_INTERVAL		(200)	/* 200 msecs */
#define YAS_INITCOIL_INTERVAL			(3000)	/* 3 seconds */
#define YAS_INITCOIL_GIVEUP_INTERVAL		(180000)/* 180 seconds */
#define YAS_DETECT_OVERFLOW_INTERVAL		(0)	/* 0 second */

#define YAS_MAG_ERROR_DELAY			(200)
#define YAS_MAG_STATE_NORMAL			(0)
#define YAS_MAG_STATE_INIT_COIL			(1)
#define YAS_MAG_STATE_MEASURE_OFFSET		(2)

static const int8_t YAS_TRANSFORMATION[][9] = {
	{ 0,  1,  0, -1,  0,  0,  0,  0,  1 },
	{-1,  0,  0,  0, -1,  0,  0,  0,  1 },
	{ 0, -1,  0,  1,  0,  0,  0,  0,  1 },
	{ 1,  0,  0,  0,  1,  0,  0,  0,  1 },
	{ 0, -1,  0, -1,  0,  0,  0,  0, -1 },
	{ 1,  0,  0,  0, -1,  0,  0,  0, -1 },
	{ 0,  1,  0,  1,  0,  0,  0,  0, -1 },
	{-1,  0,  0,  0,  1,  0,  0,  0, -1 },
};

static const int supported_data_interval[] = {10, 20, 50, 60, 100, 200, 1000};
static const int supported_calib_interval[] = {60, 60, 50, 60, 50, 50, 50};
static const int32_t INVALID_CALIB_OFFSET[]
	= {0x7fffffff, 0x7fffffff, 0x7fffffff};
static const int8_t INVALID_OFFSET[] = {0x7f, 0x7f, 0x7f};

struct yas_adaptive_filter {
	int num;
	int index;
	int filter_len;
	int filter_noise;
	int32_t sequence[YAS_MAG_MAX_FILTER_LEN];
};

struct yas_thresh_filter {
	int32_t threshold;
	int32_t last;
};

struct yas_driver {
	int initialized;
	struct yas_mag_driver_callback callback;
	struct utimer data_timer;
	struct utimer initcoil_timer;
	struct utimer initcoil_giveup_timer;
	struct utimer detect_overflow_timer;
	int32_t prev_mag[3];
	int32_t prev_xy1y2[3];
	int32_t prev_mag_w_offset[3];
	int16_t prev_temperature;
	int measure_state;
	int active;
	int overflow;
	int initcoil_gaveup;
	int position;
	int delay_timer_use_data;
	int delay_timer_interval;
	int delay_timer_counter;
	int filter_enable;
	int filter_len;
	int filter_thresh;
	int filter_noise[3];
	struct yas_adaptive_filter adap_filter[3];
	struct yas_thresh_filter thresh_filter[3];
	struct yas_mag_offset offset;
#ifdef YAS_MAG_MANUAL_OFFSET
	struct yas_vector manual_offset;
#endif
	struct yas_matrix static_matrix;
	struct yas_matrix dynamic_matrix;
};

static struct yas_driver this_driver;

static int
lock(void)
{
	if (this_driver.callback.lock != NULL) {
		if (this_driver.callback.lock() < 0)
			return YAS_ERROR_RESTARTSYS;
	}
	return 0;
}

static int
unlock(void)
{
	if (this_driver.callback.unlock != NULL) {
		if (this_driver.callback.unlock() < 0)
			return YAS_ERROR_RESTARTSYS;
	}
	return 0;
}

static int32_t
square(int32_t data)
{
	return data * data;
}

static void
adaptive_filter_init(struct yas_adaptive_filter *adap_filter, int len,
		int noise)
{
	int i;

	adap_filter->num = 0;
	adap_filter->index = 0;
	adap_filter->filter_noise = noise;
	adap_filter->filter_len = len;

	for (i = 0; i < adap_filter->filter_len; ++i)
		adap_filter->sequence[i] = 0;
}

static int32_t
adaptive_filter_filter(struct yas_adaptive_filter *adap_filter, int32_t in)
{
	int32_t avg, sum;
	int i;

	if (adap_filter->filter_len == 0)
		return in;
	if (adap_filter->num < adap_filter->filter_len) {
		adap_filter->sequence[adap_filter->index++] = in / 100;
		adap_filter->num++;
		return in;
	}
	if (adap_filter->filter_len <= adap_filter->index)
		adap_filter->index = 0;
	adap_filter->sequence[adap_filter->index++] = in / 100;

	avg = 0;
	for (i = 0; i < adap_filter->filter_len; i++)
		avg += adap_filter->sequence[i];
	avg /= adap_filter->filter_len;

	sum = 0;
	for (i = 0; i < adap_filter->filter_len; i++)
		sum += square(avg - adap_filter->sequence[i]);
	sum /= adap_filter->filter_len;

	if (sum <= adap_filter->filter_noise)
		return avg * 100;

	return ((in/100 - avg) * (sum - adap_filter->filter_noise) / sum + avg)
		* 100;
}

static void
thresh_filter_init(struct yas_thresh_filter *thresh_filter, int threshold)
{
	thresh_filter->threshold = threshold;
	thresh_filter->last = 0;
}

static int32_t
thresh_filter_filter(struct yas_thresh_filter *thresh_filter, int32_t in)
{
	if (in < thresh_filter->last - thresh_filter->threshold
			|| thresh_filter->last
			+ thresh_filter->threshold < in) {
		thresh_filter->last = in;
		return in;
	} else {
		return thresh_filter->last;
	}
}

static void
filter_init(struct yas_driver *d)
{
	int i;

	for (i = 0; i < 3; i++) {
		adaptive_filter_init(&d->adap_filter[i], d->filter_len,
				d->filter_noise[i]);
		thresh_filter_init(&d->thresh_filter[i], d->filter_thresh);
	}
}

static void
filter_filter(struct yas_driver *d, int32_t *orig, int32_t *filtered)
{
	int i;

	for (i = 0; i < 3; i++) {
		filtered[i] = adaptive_filter_filter(&d->adap_filter[i],
				orig[i]);
		filtered[i] = thresh_filter_filter(&d->thresh_filter[i],
				filtered[i]);
	}
}

static int
is_valid_offset(const int8_t *p)
{
	return p != NULL && (p[0] < 33) && (p[1] < 33) && (p[2] < 33)
		&& (0 <= p[0] && 0 <= p[1] && 0 <= p[2]);
}

static int
is_valid_calib_offset(const int32_t *p)
{
	int i;
	for (i = 0; i < 3; i++) {
		if (p[i] != INVALID_CALIB_OFFSET[i])
			return 1;
	}
	return 0;
}

static int
is_offset_differ(const int8_t *p0, const int8_t *p1)
{
	return p0[0] != p1[0] || p0[1] != p1[1] || p0[2] != p1[2];
}

static int
is_calib_offset_differ(const int32_t *p0, const int32_t *p1)
{
	return p0[0] != p1[0] || p0[1] != p1[1] || p0[2] != p1[2];
}

static int
get_overflow(struct yas_driver *d)
{
	return d->overflow;
}

static void
set_overflow(struct yas_driver *d, const int overflow)
{
	if (d->overflow != overflow)
		d->overflow = overflow;
}

static int
get_initcoil_gaveup(struct yas_driver *d)
{
	return d->initcoil_gaveup;
}

static void
set_initcoil_gaveup(struct yas_driver *d, const int initcoil_gaveup)
{
	d->initcoil_gaveup = initcoil_gaveup;
}

static int32_t *
get_calib_offset(struct yas_driver *d)
{
	return d->offset.calib_offset.v;
}

static void
set_calib_offset(struct yas_driver *d, const int32_t *offset)
{
	int i;

	if (is_calib_offset_differ(d->offset.calib_offset.v, offset)) {
		for (i = 0; i < 3; i++)
			d->offset.calib_offset.v[i] = offset[i];
	}
}

#ifdef YAS_MAG_MANUAL_OFFSET
static int32_t *
get_manual_offset(struct yas_driver *d)
{
	return d->manual_offset.v;
}

static void
set_manual_offset(struct yas_driver *d, const int32_t *offset)
{
	int i;

	for (i = 0; i < 3; i++)
		d->manual_offset.v[i] = offset[i];
}
#endif

static int32_t *
get_static_matrix(struct yas_driver *d)
{
	return d->static_matrix.matrix;
}

static void
set_static_matrix(struct yas_driver *d, const int32_t *matrix)
{
	int i;

	for (i = 0; i < 9; i++)
		d->static_matrix.matrix[i] = matrix[i];
}

static int32_t *
get_dynamic_matrix(struct yas_driver *d)
{
	return d->dynamic_matrix.matrix;
}

static void
set_dynamic_matrix(struct yas_driver *d, const int32_t *matrix)
{
	int i;

	for (i = 0; i < 9; i++)
		d->dynamic_matrix.matrix[i] = matrix[i];
}

static int8_t *
get_offset(struct yas_driver *d)
{
	return d->offset.hard_offset;
}

static void
set_offset(struct yas_driver *d, const int8_t *offset)
{
	int i;

	if (is_offset_differ(d->offset.hard_offset, offset)) {
		for (i = 0; i < 3; i++)
			d->offset.hard_offset[i] = offset[i];
	}
}

static int
get_active(struct yas_driver *d)
{
	return d->active;
}

static void
set_active(struct yas_driver *d, const int active)
{
	d->active = active;
}

static int
get_position(struct yas_driver *d)
{
	return d->position;
}

static void
set_position(struct yas_driver *d, const int position)
{
	d->position = position;
}

static int
get_measure_state(struct yas_driver *d)
{
	return d->measure_state;
}

static void
set_measure_state(struct yas_driver *d, const int state)
{
	YLOGI(("state(%d)\n", state));
	d->measure_state = state;
}

static struct utimer *
get_data_timer(struct yas_driver *d)
{
	return &d->data_timer;
}

static struct utimer *
get_initcoil_timer(struct yas_driver *d)
{
	return &d->initcoil_timer;
}

static struct utimer *
get_initcoil_giveup_timer(struct yas_driver *d)
{
	return &d->initcoil_giveup_timer;
}

static struct utimer *
get_detect_overflow_timer(struct yas_driver *d)
{
	return &d->detect_overflow_timer;
}

static int
get_delay_timer_use_data(struct yas_driver *d)
{
	return d->delay_timer_use_data;
}

static void
set_delay_timer_use_data(struct yas_driver *d, int flag)
{
	d->delay_timer_use_data = !!flag;
}

static int
get_delay_timer_interval(struct yas_driver *d)
{
	return d->delay_timer_interval;
}

static void
set_delay_timer_interval(struct yas_driver *d, int interval)
{
	d->delay_timer_interval = interval;
}

static int
get_delay_timer_counter(struct yas_driver *d)
{
	return d->delay_timer_counter;
}

static void
set_delay_timer_counter(struct yas_driver *d, int counter)
{
	d->delay_timer_counter = counter;
}

static int
get_filter_enable(struct yas_driver *d)
{
	return d->filter_enable;
}

static void
set_filter_enable(struct yas_driver *d, int enable)
{
	if (enable)
		filter_init(d);
	d->filter_enable = !!enable;
}

static int
get_filter_len(struct yas_driver *d)
{
	return d->filter_len;
}

static void
set_filter_len(struct yas_driver *d, int len)
{
	if (len < 0)
		return;
	if (len > YAS_MAG_MAX_FILTER_LEN)
		return;
	d->filter_len = len;
	filter_init(d);
}

static int
get_filter_noise(struct yas_driver *d, int *noise)
{
	int i;

	for (i = 0; i < 3; i++)
		noise[i] = d->filter_noise[i];
	return 0;
}

static void
set_filter_noise(struct yas_driver *d, int *noise)
{
	int i;

	if (noise == NULL)
		return;
	for (i = 0; i < 3; i++) {
		if (noise[i] < 0)
			noise[i] = 0;
		d->filter_noise[i] = noise[i];
	}
	filter_init(d);
}

static int
get_filter_thresh(struct yas_driver *d)
{
	return d->filter_thresh;
}

static void
set_filter_thresh(struct yas_driver *d, int threshold)
{
	if (threshold < 0)
		return;
	d->filter_thresh = threshold;
	filter_init(d);
}

static int32_t*
get_previous_mag(struct yas_driver *d)
{
	return d->prev_mag;
}

static void
set_previous_mag(struct yas_driver *d, int32_t *data)
{
	int i;
	for (i = 0; i < 3; i++)
		d->prev_mag[i] = data[i];
}

static int32_t*
get_previous_xy1y2(struct yas_driver *d)
{
	return d->prev_xy1y2;
}

static void
set_previous_xy1y2(struct yas_driver *d, int32_t *data)
{
	int i;
	for (i = 0; i < 3; i++)
		d->prev_xy1y2[i] = data[i];
}

static int32_t*
get_previous_mag_w_offset(struct yas_driver *d)
{
	return d->prev_mag_w_offset;
}

static void
set_previous_mag_w_offset(struct yas_driver *d, int32_t *data)
{
	int i;
	for (i = 0; i < 3; i++)
		d->prev_mag_w_offset[i] = data[i];
}

static int16_t
get_previous_temperature(struct yas_driver *d)
{
	return d->prev_temperature;
}

static void
set_previous_temperature(struct yas_driver *d, int16_t temperature)
{
	d->prev_temperature = temperature;
}

static int
init_coil(struct yas_driver *d)
{
	int rt;

	YLOGD(("init_coil IN\n"));

	utimer_update(get_initcoil_timer(d));
	if (!get_initcoil_gaveup(d)) {
		utimer_update(get_initcoil_giveup_timer(d));
		if (utimer_is_timeout(get_initcoil_giveup_timer(d))) {
			utimer_clear_timeout(get_initcoil_giveup_timer(d));
			set_initcoil_gaveup(d, TRUE);
		}
	}
	if (utimer_is_timeout(get_initcoil_timer(d))
			&& !get_initcoil_gaveup(d)) {
		utimer_clear_timeout(get_initcoil_timer(d));
		YLOGI(("init_coil!\n"));
		rt = yas_cdrv_actuate_initcoil();
		if (rt < 0) {
			YLOGE(("yas_cdrv_actuate_initcoil failed[%d]\n", rt));
			return rt;
		}
		if (get_overflow(d) || !is_valid_offset(get_offset(d)))
			set_measure_state(d, YAS_MAG_STATE_MEASURE_OFFSET);
		else
			set_measure_state(d, YAS_MAG_STATE_NORMAL);
	}

	YLOGD(("init_coil OUT\n"));

	return 0;
}

static int
measure_offset(struct yas_driver *d)
{
	int8_t offset[3];
	int32_t moffset[3];
	int rt, result = 0, i;

	YLOGI(("measure_offset IN\n"));

	rt = yas_cdrv_measure_offset(offset);
	if (rt < 0) {
		YLOGE(("yas_cdrv_measure_offset failed[%d]\n", rt));
		return rt;
	}

	YLOGI(("rough offset[%d][%d][%d]\n", offset[0], offset[1], offset[2]));

	if (rt & MEASURE_RESULT_OVER_UNDER_FLOW) {
		YLOGI(("yas_cdrv_measure_offset overflow x[%d] y[%d] z[%d]\n",
				!!(rt&0x01), !!(rt&0x02), !!(rt&0x04)));

		set_overflow(d, TRUE);
		set_measure_state(d, YAS_MAG_STATE_INIT_COIL);
		return YAS_REPORT_OVERFLOW_OCCURED;
	}

	for (i = 0; i < 3; i++)
		moffset[i] = get_calib_offset(d)[i];
	if (is_offset_differ(get_offset(d), offset)) {
		if (is_valid_offset(get_offset(d))
				&& is_valid_calib_offset(get_calib_offset(d))) {
			yas_cdrv_recalc_calib_offset(get_calib_offset(d),
					moffset,
					get_offset(d),
					offset);
			result |= YAS_REPORT_CALIB_OFFSET_CHANGED;
		}
	}

	rt = yas_cdrv_set_offset(offset);
	if (rt < 0) {
		YLOGE(("yas_cdrv_set_offset failed[%d]\n", rt));
		return rt;
	}

	result |= YAS_REPORT_HARD_OFFSET_CHANGED;
	set_offset(d, offset);
	if (is_valid_calib_offset(moffset))
		set_calib_offset(d, moffset);
	set_measure_state(d, YAS_MAG_STATE_NORMAL);

	YLOGI(("measure_offset OUT\n"));

	return result;
}

static int
measure_msensor_normal(struct yas_driver *d, int32_t *magnetic,
		int32_t *mag_w_offset, int32_t *xy1y2, int16_t *temperature)
{
	int rt = 0, result, i;
	int32_t tmp[3];

	YLOGD(("measure_msensor_normal IN\n"));

	result = 0;
	rt = yas_cdrv_measure(mag_w_offset, xy1y2, temperature);
	if (rt < 0) {
		YLOGE(("yas_cdrv_measure failed[%d]\n", rt));
		return rt;
	}
#ifdef YAS_MAG_MANUAL_OFFSET
	for (i = 0; i < 3; i++)
		mag_w_offset[i] -= get_manual_offset(d)[i];
#endif
	if (rt & MEASURE_RESULT_OVER_UNDER_FLOW) {
		YLOGI(("yas_cdrv_measure overflow x[%d] y[%d] z[%d]\n",
				!!(rt&0x01), !!(rt&0x02), !!(rt&0x04)));
		utimer_update(get_detect_overflow_timer(d));
		set_overflow(d, TRUE);
		if (utimer_is_timeout(get_detect_overflow_timer(d))) {
			utimer_clear_timeout(get_detect_overflow_timer(d));
			result |= YAS_REPORT_OVERFLOW_OCCURED;
		}
		if (get_measure_state(d) == YAS_MAG_STATE_NORMAL)
			set_measure_state(d, YAS_MAG_STATE_INIT_COIL);
	} else {
		utimer_clear(get_detect_overflow_timer(d));
		set_overflow(d, FALSE);
		if (get_measure_state(d) == YAS_MAG_STATE_NORMAL) {
			utimer_clear(get_initcoil_timer(d));
			utimer_clear(get_initcoil_giveup_timer(d));
		}
	}

	for (i = 0; i < 3; i++) {
		tmp[i]
			= (get_static_matrix(d)[i*3+0]/10
					* (mag_w_offset[0]/10)) / 100
			+ (get_static_matrix(d)[i*3+1]/10
					* (mag_w_offset[1]/10)) / 100
			+ (get_static_matrix(d)[i*3+2]/10
					* (mag_w_offset[2]/10)) / 100;
	}
	for (i = 0; i < 3; i++)
		magnetic[i] = mag_w_offset[i] = tmp[i];
	if (is_valid_calib_offset(get_calib_offset(d))) {
		for (i = 0; i < 3; i++)
			magnetic[i] -= get_calib_offset(d)[i];
	}
	for (i = 0; i < 3; i++) {
		tmp[i]
			= (get_dynamic_matrix(d)[i*3+0]/10
					* (magnetic[0]/10)) / 100
			+ (get_dynamic_matrix(d)[i*3+1]/10
					* (magnetic[1]/10)) / 100
			+ (get_dynamic_matrix(d)[i*3+2]/10
					* (magnetic[2]/10)) / 100;
	}
	for (i = 0; i < 3; i++)
		magnetic[i] = tmp[i];
	if (get_filter_enable(d))
		filter_filter(d, magnetic, magnetic);

	YLOGD(("measure_msensor_normal OUT\n"));

	return result;
}

static int
measure_msensor(struct yas_driver *d, int32_t *magnetic, int32_t *mag_w_offset,
		int32_t *xy1y2, int16_t *temperature)
{
	int result, i;

	YLOGD(("measure_msensor IN\n"));

	for (i = 0; i < 3; i++) {
		magnetic[i] = get_previous_mag(d)[i];
		mag_w_offset[i] = get_previous_mag_w_offset(d)[i];
		xy1y2[i] = get_previous_xy1y2(d)[i];
		*temperature = get_previous_temperature(d);
	}

	result = 0;
	switch (get_measure_state(d)) {
	case YAS_MAG_STATE_INIT_COIL:
		result = init_coil(d);
		break;
	case YAS_MAG_STATE_MEASURE_OFFSET:
		result = measure_offset(d);
		break;
	case YAS_MAG_STATE_NORMAL:
		result = 0;
		break;
	default:
		result = -1;
		break;
	}

	if (result < 0)
		return result;

	if (!(result & YAS_REPORT_OVERFLOW_OCCURED)) {
		result |= measure_msensor_normal(d, magnetic, mag_w_offset,
				xy1y2, temperature);
	}
	set_previous_mag(d, magnetic);
	set_previous_xy1y2(d, xy1y2);
	set_previous_mag_w_offset(d, mag_w_offset);
	set_previous_temperature(d, *temperature);

	YLOGD(("measure_msensor OUT\n"));

	return result;
}

static int
measure(struct yas_driver *d, int32_t *magnetic, int32_t *mag_w_offset,
		int32_t *xy1y2, int16_t *temperature, uint32_t *time_delay)
{
	int result;
	int counter;
	uint32_t total = 0;

	YLOGD(("measure IN\n"));

	utimer_update(get_data_timer(d));

	result = measure_msensor(d, magnetic, mag_w_offset, xy1y2, temperature);
	if (result < 0)
		return result;

	counter = get_delay_timer_counter(d);
	total = utimer_get_total_time(get_data_timer(d));
	if (utimer_get_delay(get_data_timer(d)) > 0)
		counter = counter - (int)(total
				/ utimer_get_delay(get_data_timer(d)));
	else
		counter = 0;

	if (utimer_is_timeout(get_data_timer(d))) {
		utimer_clear_timeout(get_data_timer(d));

		if (get_delay_timer_use_data(d)) {
			result |= YAS_REPORT_DATA;
			if (counter <= 0)
				result |= YAS_REPORT_CALIB;
		} else {
			result |= YAS_REPORT_CALIB;
			if (counter <= 0)
				result |= YAS_REPORT_DATA;
		}
	}

	if (counter <= 0)
		set_delay_timer_counter(d, get_delay_timer_interval(d));
	else
		set_delay_timer_counter(d, counter);

	*time_delay = utimer_sleep_time(get_data_timer(d));

	YLOGD(("measure OUT [%d]\n", result));

	return result;
}

static int
resume(struct yas_driver *d)
{
	int32_t zero[] = {0, 0, 0};
	struct yas_machdep_func func;
	int rt;

	YLOGI(("resume IN\n"));

	func.device_open = d->callback.device_open;
	func.device_close = d->callback.device_close;
	func.device_write = d->callback.device_write;
	func.device_read = d->callback.device_read;
	func.msleep = d->callback.msleep;

	rt = yas_cdrv_init(YAS_TRANSFORMATION[get_position(d)], &func);
	if (rt < 0) {
		YLOGE(("yas_cdrv_init failed[%d]\n", rt));
		return rt;
	}

	utimer_clear(get_data_timer(d));
	utimer_clear(get_initcoil_giveup_timer(d));
	utimer_clear(get_initcoil_timer(d));
	utimer_clear(get_detect_overflow_timer(d));

	set_previous_mag(d, zero);
	set_previous_xy1y2(d, zero);
	set_previous_mag_w_offset(d, zero);
	set_previous_temperature(d, 0);
	set_overflow(d, FALSE);
	set_initcoil_gaveup(d, FALSE);

	filter_init(d);

	if (is_valid_offset(d->offset.hard_offset)) {
		yas_cdrv_set_offset(d->offset.hard_offset);
/*
		rt = yas_cdrv_actuate_initcoil();
		if (rt < 0) {
			YLOGE(("yas_cdrv_actuate_initcoil failed[%d]\n", rt));
			set_measure_state(d, YAS_MAG_STATE_INIT_COIL);
		} else {
			set_measure_state(d, YAS_MAG_STATE_NORMAL);
		}
*/












	} else {
		rt = yas_cdrv_actuate_initcoil();
		if (rt < 0) {
			YLOGE(("yas_cdrv_actuate_initcoil failed[%d]\n", rt));
			set_measure_state(d, YAS_MAG_STATE_INIT_COIL);
		} else {
			set_measure_state(d, YAS_MAG_STATE_MEASURE_OFFSET);
		}
	}

	YLOGI(("resume OUT\n"));
	return 0;
}

static int
suspend(struct yas_driver *d)
{
	YLOGI(("suspend IN\n"));

	(void) d;
	yas_cdrv_term();

	YLOGI(("suspend OUT\n"));
	return 0;
}

static int
check_interval(int ms)
{
	int index;

	if (ms <= supported_data_interval[0])
		ms = supported_data_interval[0];
	for (index = 0; index < NELEMS(supported_data_interval); index++) {
		if (ms == supported_data_interval[index])
			return index;
	}
	return -1;
}

static int
yas_get_delay_nolock(struct yas_driver *d, int *ms)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (get_delay_timer_use_data(d))
		*ms = (int)utimer_get_delay(get_data_timer(d));
	else
		*ms = (int)utimer_get_delay(get_data_timer(d))
			* get_delay_timer_interval(d);
	return YAS_NO_ERROR;
}

static int
yas_set_delay_nolock(struct yas_driver *d, int ms)
{
	int index;
	uint32_t delay_data, delay_calib;

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	index = check_interval(ms);
	if (index < 0)
		return YAS_ERROR_ARG;
	delay_data = (uint32_t)supported_data_interval[index];
	delay_calib = (uint32_t)supported_calib_interval[index];
	set_delay_timer_use_data(d, delay_data < delay_calib);
	if (delay_data < delay_calib) {
		set_delay_timer_interval(d, (int)(delay_calib / delay_data));
		set_delay_timer_counter(d, (int)(delay_calib / delay_data));
		utimer_set_delay(get_data_timer(d),
				(uint32_t)supported_data_interval[index]);
	} else {
		set_delay_timer_interval(d, (int)(delay_data / delay_calib));
		set_delay_timer_counter(d, (int)(delay_data / delay_calib));
		utimer_set_delay(get_data_timer(d),
				(uint32_t)supported_calib_interval[index]);
	}

	return YAS_NO_ERROR;
}

static int
yas_get_offset_nolock(struct yas_driver *d, struct yas_mag_offset *offset)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	*offset = d->offset;
	return YAS_NO_ERROR;
}

static int
yas_set_offset_nolock(struct yas_driver *d, struct yas_mag_offset *offset)
{
	int32_t zero[] = {0, 0, 0};
	int rt;

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (!get_active(d)) {
		d->offset = *offset;
		return YAS_NO_ERROR;
	}

	if (!is_valid_offset(offset->hard_offset)
			|| is_offset_differ(offset->hard_offset,
				d->offset.hard_offset)) {
		filter_init(d);

		utimer_clear(get_data_timer(d));
		utimer_clear(get_initcoil_giveup_timer(d));
		utimer_clear(get_initcoil_timer(d));
		utimer_clear(get_detect_overflow_timer(d));

		set_previous_mag(d, zero);
		set_previous_xy1y2(d, zero);
		set_previous_mag_w_offset(d, zero);
		set_previous_temperature(d, 0);
		set_overflow(d, FALSE);
		set_initcoil_gaveup(d, FALSE);
	}
	d->offset = *offset;

	if (is_valid_offset(d->offset.hard_offset)) {
		yas_cdrv_set_offset(d->offset.hard_offset);
	} else {
		rt = yas_cdrv_actuate_initcoil();
		if (rt < 0) {
			YLOGE(("yas_cdrv_actuate_initcoil failed[%d]\n", rt));
			set_measure_state(d, YAS_MAG_STATE_INIT_COIL);
		} else {
			set_measure_state(d, YAS_MAG_STATE_MEASURE_OFFSET);
		}
	}

	return YAS_NO_ERROR;
}

#ifdef YAS_MAG_MANUAL_OFFSET

static int
yas_get_manual_offset_nolock(struct yas_driver *d, struct yas_vector *offset)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	*offset = d->manual_offset;

	return YAS_NO_ERROR;
}

static int
yas_set_manual_offset_nolock(struct yas_driver *d, struct yas_vector *offset)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	set_manual_offset(d, offset->v);

	return YAS_NO_ERROR;
}

#endif

static int
yas_get_static_matrix_nolock(struct yas_driver *d, struct yas_matrix *matrix)
{
	int i;

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	for (i = 0; i < 9; i++)
		matrix->matrix[i] = get_static_matrix(d)[i];
	return YAS_NO_ERROR;
}

static int
yas_set_static_matrix_nolock(struct yas_driver *d, struct yas_matrix *matrix)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	set_static_matrix(d, matrix->matrix);
	return YAS_NO_ERROR;
}

static int
yas_get_dynamic_matrix_nolock(struct yas_driver *d, struct yas_matrix *matrix)
{
	int i;

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	for (i = 0; i < 9; i++)
		matrix->matrix[i] = get_dynamic_matrix(d)[i];

	return YAS_NO_ERROR;
}

static int
yas_set_dynamic_matrix_nolock(struct yas_driver *d, struct yas_matrix *matrix)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	set_dynamic_matrix(d, matrix->matrix);

	return YAS_NO_ERROR;
}

static int
yas_get_enable_nolock(struct yas_driver *d)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	return get_active(d);
}

static int
yas_set_enable_nolock(struct yas_driver *d, int active)
{
	int rt;

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (active) {
		if (get_active(d))
			return YAS_NO_ERROR;
		rt = resume(d);
		if (rt < 0)
			return rt;
		set_active(d, TRUE);
	} else {
		if (!get_active(d))
			return YAS_NO_ERROR;
		rt = suspend(d);
		if (rt < 0)
			return rt;
		set_active(d, FALSE);
	}
	return YAS_NO_ERROR;
}

static int
yas_get_filter_nolock(struct yas_driver *d, struct yas_mag_filter *filter)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	filter->len = get_filter_len(d);
	get_filter_noise(d, filter->noise);
	filter->threshold = get_filter_thresh(d);
	return YAS_NO_ERROR;
}

static int
yas_set_filter_nolock(struct yas_driver *d, struct yas_mag_filter *filter)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	set_filter_len(d, filter->len);
	set_filter_noise(d, filter->noise);
	set_filter_thresh(d, filter->threshold);
	return YAS_NO_ERROR;
}

static int
yas_get_filter_enable_nolock(struct yas_driver *d)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	return get_filter_enable(d);
}

static int
yas_set_filter_enable_nolock(struct yas_driver *d, int enable)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	set_filter_enable(d, enable);
	return YAS_NO_ERROR;
}

static int
yas_get_position_nolock(struct yas_driver *d, int *position)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	*position = get_position(d);
	return YAS_NO_ERROR;
}

static int
yas_set_position_nolock(struct yas_driver *d, int position)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (get_active(d))
		yas_cdrv_set_transformatiom_matrix(
				YAS_TRANSFORMATION[position]);
	set_position(d, position);
	filter_init(d);
	return YAS_NO_ERROR;
}

static int
yas_read_reg_nolock(struct yas_driver *d, uint8_t *buf, int len)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (!get_active(d)) {
		if (d->callback.device_open() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (d->callback.device_read(buf, len) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (!get_active(d)) {
		if (d->callback.device_close() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	return YAS_NO_ERROR;
}

static int
yas_write_reg_nolock(struct yas_driver *d, const uint8_t *buf, int len)
{
	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (!get_active(d)) {
		if (d->callback.device_open() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}
	if (d->callback.device_write(buf, len) < 0)
		return YAS_ERROR_DEVICE_COMMUNICATION;
	if (!get_active(d)) {
		if (d->callback.device_close() < 0)
			return YAS_ERROR_DEVICE_COMMUNICATION;
	}

	return YAS_NO_ERROR;
}

static int
yas_measure_nolock(struct yas_driver *d, struct yas_mag_data *data,
		int *time_delay_ms)
{
	uint32_t time_delay = YAS_MAG_ERROR_DELAY;
	int rt, i;

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	*time_delay_ms = YAS_MAG_ERROR_DELAY;

	if (!get_active(d)) {
		for (i = 0; i < 3; i++) {
			data->xyz.v[i] = get_previous_mag(d)[i];
			data->raw.v[i] = get_previous_mag_w_offset(d)[i];
			data->xy1y2.v[i] = get_previous_xy1y2(d)[i];
		}
		data->temperature = get_previous_temperature(d);
		return YAS_NO_ERROR;
	}

	rt = measure(d, data->xyz.v, data->raw.v, data->xy1y2.v,
			&data->temperature, &time_delay);
	if (rt >= 0) {
		*time_delay_ms = (int)time_delay;
		if (*time_delay_ms > 0)
			*time_delay_ms += 1; /* for the system that the time is
						in usec unit */
	}

	return rt;
}

static int
yas_init_nolock(struct yas_driver *d)
{
#ifdef YAS_MAG_MANUAL_OFFSET
	int32_t zero[] = {0, 0, 0};
#endif

    int32_t notransform[] = {10000, 0, 0, 0, 10000, 0, 0, 0, 10000};

	int noise[] = {
		YAS_MAG_DEFAULT_FILTER_NOISE_X,
		YAS_MAG_DEFAULT_FILTER_NOISE_Y,
		YAS_MAG_DEFAULT_FILTER_NOISE_Z
	};

	YLOGI(("yas_init_nolock IN\n"));

	if (d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;

	utimer_lib_init(this_driver.callback.current_time);
	utimer_init(get_data_timer(d), 50);
	utimer_init(get_initcoil_timer(d), YAS_INITCOIL_INTERVAL);
	utimer_init(get_initcoil_giveup_timer(d), YAS_INITCOIL_GIVEUP_INTERVAL);
	utimer_init(get_detect_overflow_timer(d), YAS_DETECT_OVERFLOW_INTERVAL);

	set_delay_timer_use_data(d, 0);
	set_delay_timer_interval(d,
			YAS_DEFAULT_DATA_INTERVAL / YAS_DEFAULT_CALIB_INTERVAL);
	set_delay_timer_counter(d,
			YAS_DEFAULT_DATA_INTERVAL / YAS_DEFAULT_CALIB_INTERVAL);

	set_filter_enable(d, FALSE);
	set_filter_len(d, YAS_MAG_DEFAULT_FILTER_LEN);
	set_filter_thresh(d, YAS_MAG_DEFAULT_FILTER_THRESH);
	set_filter_noise(d, noise);
	filter_init(d);
	set_calib_offset(d, INVALID_CALIB_OFFSET);
#ifdef YAS_MAG_MANUAL_OFFSET
	set_manual_offset(d, zero);
#endif
	set_static_matrix(d, notransform);
	set_dynamic_matrix(d, notransform);
	set_offset(d, INVALID_OFFSET);
	set_active(d, FALSE);
	set_position(d, 0);

	d->initialized = 1;

	YLOGI(("yas_init_nolock OUT\n"));

	return YAS_NO_ERROR;
}

static int
yas_term_nolock(struct yas_driver *d)
{
	YLOGI(("yas_term_nolock\n"));

	if (!d->initialized)
		return YAS_ERROR_NOT_INITIALIZED;
	if (get_active(d))
		suspend(d);
	d->initialized = 0;

	YLOGI(("yas_term_nolock out\n"));
	return YAS_NO_ERROR;
}

static int
yas_get_delay(void)
{
	int ms = 0, rt;

	YLOGI(("yas_get_delay\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_delay_nolock(&this_driver, &ms);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_delay[%d] OUT\n", ms));

	return rt < 0 ? rt : ms;
}

static int
yas_set_delay(int delay)
{
	int rt;

	YLOGI(("yas_set_delay\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_delay_nolock(&this_driver, delay);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_delay OUT\n"));

	return rt;
}

static int
yas_get_offset(struct yas_mag_offset *offset)
{
	int rt;

	YLOGI(("yas_get_offset\n"));

	if (offset == NULL)
		return YAS_ERROR_ARG;
	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_offset_nolock(&this_driver, offset);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_offset[%d] OUT\n", rt));

	return rt;
}

static int
yas_set_offset(struct yas_mag_offset *offset)
{
	int rt;

	YLOGI(("yas_set_offset IN\n"));

	if (offset == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_offset_nolock(&this_driver, offset);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_offset OUT\n"));

	return rt;
}

#ifdef YAS_MAG_MANUAL_OFFSET

static int
yas_get_manual_offset(struct yas_vector *offset)
{
	int rt;

	YLOGI(("yas_get_manual_offset\n"));

	if (offset == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_manual_offset_nolock(&this_driver, offset);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_manual_offset[%d] OUT\n", rt));

	return rt;
}

static int
yas_set_manual_offset(struct yas_vector *offset)
{
	int rt;

	YLOGI(("yas_set_manual_offset IN\n"));

	if (offset == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_manual_offset_nolock(&this_driver, offset);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_manual_offset OUT\n"));

	return rt;
}

#endif

static int
yas_get_static_matrix(struct yas_matrix *matrix)
{
	int rt;

	YLOGI(("yas_get_static_matrix\n"));

	if (matrix == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_static_matrix_nolock(&this_driver, matrix);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_static_matrix[%d] OUT\n", rt));

	return rt;
}

static int
yas_set_static_matrix(struct yas_matrix *matrix)
{
	int rt;

	YLOGI(("yas_set_static_matrix IN\n"));

	if (matrix == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_static_matrix_nolock(&this_driver, matrix);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_static_matrix OUT\n"));

	return rt;
}

static int
yas_get_dynamic_matrix(struct yas_matrix *matrix)
{
	int rt;

	YLOGI(("yas_get_dynamic_matrix\n"));

	if (matrix == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_dynamic_matrix_nolock(&this_driver, matrix);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_dynamic_matrix[%d] OUT\n", rt));

	return rt;
}

static int
yas_set_dynamic_matrix(struct yas_matrix *matrix)
{
	int rt;

	YLOGI(("yas_set_dynamic_matrix IN\n"));

	if (matrix == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_dynamic_matrix_nolock(&this_driver, matrix);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_dynamic_matrix OUT\n"));

	return rt;
}

static int
yas_get_enable(void)
{
	int rt;

	YLOGI(("yas_get_enable\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_enable_nolock(&this_driver);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_enable OUT[%d]\n", rt));

	return rt;
}

static int
yas_set_enable(int enable)
{
	int rt;

	YLOGI(("yas_set_enable IN\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_enable_nolock(&this_driver, enable);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_enable OUT\n"));

	return rt;
}

static int
yas_get_filter(struct yas_mag_filter *filter)
{
	int rt;

	YLOGI(("yas_get_filter\n"));

	if (filter == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_filter_nolock(&this_driver, filter);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_filter[%d] OUT\n", rt));

	return rt;
}

static int
yas_set_filter(struct yas_mag_filter *filter)
{
	int rt, i;

	YLOGI(("yas_set_filter IN\n"));

	if (filter == NULL
			|| filter->len < 0
			|| YAS_MAG_MAX_FILTER_LEN < filter->len
			|| filter->threshold < 0) {
		return YAS_ERROR_ARG;
	}
	for (i = 0; i < 3; i++) {
		if (filter->noise[i] < 0)
			return YAS_ERROR_ARG;
	}

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_filter_nolock(&this_driver, filter);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_filter OUT\n"));

	return rt;
}

static int
yas_get_filter_enable(void)
{
	int rt;

	YLOGI(("yas_get_filter_enable\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_filter_enable_nolock(&this_driver);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_filter_enable OUT[%d]\n", rt));

	return rt;
}

static int
yas_set_filter_enable(int enable)
{
	int rt;

	YLOGI(("yas_set_filter_enable IN\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_filter_enable_nolock(&this_driver, enable);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_filter_enable OUT\n"));

	return rt;
}

static int
yas_get_position(void)
{
	int position = 0;
	int rt;

	YLOGI(("yas_get_position\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_get_position_nolock(&this_driver, &position);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_get_position[%d] OUT\n", position));

	return rt < 0 ? rt : position;
}

static int
yas_set_position(int position)
{
	int rt;

	YLOGI(("yas_set_position\n"));

	if (position < 0 || 7 < position)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_set_position_nolock(&this_driver, position);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_set_position[%d] OUT\n", position));

	return rt;
}

static int
yas_read_reg(uint8_t *buf, int len)
{
	int rt;

	YLOGI(("yas_read_reg\n"));

	if (buf == NULL || len <= 0)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_read_reg_nolock(&this_driver, buf, len);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_read_reg[%d] OUT\n", rt));

	return rt;
}

static int
yas_write_reg(const uint8_t *buf, int len)
{
	int rt;

	YLOGI(("yas_write_reg\n"));

	if (buf == NULL || len <= 0)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_write_reg_nolock(&this_driver, buf, len);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGI(("yas_write_reg[%d] OUT\n", rt));

	return rt;
}

static int
yas_measure(struct yas_mag_data *data, int *time_delay_ms)
{
	int rt;

	YLOGD(("yas_measure IN\n"));

	if (data == NULL || time_delay_ms == NULL)
		return YAS_ERROR_ARG;

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_measure_nolock(&this_driver, data, time_delay_ms);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	YLOGD(("yas_measure OUT[%d]\n", rt));

	return rt;
}

static int
yas_init(void)
{
	int rt;

	YLOGI(("yas_init\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_init_nolock(&this_driver);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	return rt;
}

static int
yas_term(void)
{
	int rt;
	YLOGI(("yas_term\n"));

	if (lock() < 0)
		return YAS_ERROR_RESTARTSYS;

	rt = yas_term_nolock(&this_driver);

	if (unlock() < 0)
		return YAS_ERROR_RESTARTSYS;

	return rt;
}

int
yas_mag_driver_init(struct yas_mag_driver *f)
{
	if (f == NULL)
		return YAS_ERROR_ARG;
	if (f->callback.device_open == NULL
			|| f->callback.device_close == NULL
			|| f->callback.device_read == NULL
			|| f->callback.device_write == NULL
			|| f->callback.msleep == NULL
			|| f->callback.current_time == NULL) {
		return YAS_ERROR_ARG;
	}

	f->init = yas_init;
	f->term = yas_term;
	f->get_delay = yas_get_delay;
	f->set_delay = yas_set_delay;
	f->get_offset = yas_get_offset;
	f->set_offset = yas_set_offset;
#ifdef YAS_MAG_MANUAL_OFFSET
	f->get_manual_offset = yas_get_manual_offset;
	f->set_manual_offset = yas_set_manual_offset;
#endif
	f->get_static_matrix = yas_get_static_matrix;
	f->set_static_matrix = yas_set_static_matrix;
	f->get_dynamic_matrix = yas_get_dynamic_matrix;
	f->set_dynamic_matrix = yas_set_dynamic_matrix;
	f->get_enable = yas_get_enable;
	f->set_enable = yas_set_enable;
	f->get_filter = yas_get_filter;
	f->set_filter = yas_set_filter;
	f->get_filter_enable = yas_get_filter_enable;
	f->set_filter_enable = yas_set_filter_enable;
	f->get_position = yas_get_position;
	f->set_position = yas_set_position;
	f->read_reg = yas_read_reg;
	f->write_reg = yas_write_reg;
	f->measure = yas_measure;

	if ((f->callback.lock == NULL && f->callback.unlock != NULL)
			|| (f->callback.lock != NULL
				&& f->callback.unlock == NULL)) {
		this_driver.callback.lock = NULL;
		this_driver.callback.unlock = NULL;
	} else {
		this_driver.callback.lock = f->callback.lock;
		this_driver.callback.unlock = f->callback.unlock;
	}
	this_driver.callback.device_open = f->callback.device_open;
	this_driver.callback.device_close = f->callback.device_close;
	this_driver.callback.device_write = f->callback.device_write;
	this_driver.callback.device_read = f->callback.device_read;
	this_driver.callback.msleep = f->callback.msleep;
	this_driver.callback.current_time = f->callback.current_time;
	yas_term();

	return YAS_NO_ERROR;
}
