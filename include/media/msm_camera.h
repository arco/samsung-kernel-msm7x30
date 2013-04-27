/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef __LINUX_MSM_CAMERA_H
#define __LINUX_MSM_CAMERA_H

#ifdef MSM_CAMERA_BIONIC
#include <sys/types.h>
#endif
#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>
#ifdef MSM_CAMERA_GCC
#include <time.h>
#else
#include <linux/time.h>
#endif

#define MSM_CAM_IOCTL_MAGIC 'm'

#define MSM_CAM_IOCTL_GET_SENSOR_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 1, struct msm_camsensor_info *)

#define MSM_CAM_IOCTL_REGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 2, struct msm_pmem_info *)

#define MSM_CAM_IOCTL_UNREGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAM_IOCTL_CTRL_COMMAND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 4, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_CONFIG_VFE  \
	_IOW(MSM_CAM_IOCTL_MAGIC, 5, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_STATS \
	_IOR(MSM_CAM_IOCTL_MAGIC, 6, struct msm_camera_stats_event_ctrl *)

#define MSM_CAM_IOCTL_GETFRAME \
	_IOR(MSM_CAM_IOCTL_MAGIC, 7, struct msm_camera_get_frame *)

#define MSM_CAM_IOCTL_ENABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 8, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_CTRL_CMD_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 9, struct camera_cmd *)

#define MSM_CAM_IOCTL_CONFIG_CMD \
	_IOW(MSM_CAM_IOCTL_MAGIC, 10, struct camera_cmd *)

#define MSM_CAM_IOCTL_DISABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 11, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_PAD_REG_RESET2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 12, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_VFE_APPS_RESET \
	_IOW(MSM_CAM_IOCTL_MAGIC, 13, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 14, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_STATS_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 15, struct msm_stats_buf *)

#define MSM_CAM_IOCTL_AXI_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 16, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_PICTURE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 17, struct msm_camera_ctrl_cmd *)

#define MSM_CAM_IOCTL_SET_CROP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 18, struct crop_info *)

#define MSM_CAM_IOCTL_PICT_PP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 19, uint8_t *)

#define MSM_CAM_IOCTL_PICT_PP_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 20, struct msm_snapshot_pp_status *)

#define MSM_CAM_IOCTL_SENSOR_IO_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 21, struct sensor_cfg_data *)

#define MSM_CAM_IOCTL_FLASH_LED_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 22, unsigned *)

#define MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME \
	_IO(MSM_CAM_IOCTL_MAGIC, 23)

#define MSM_CAM_IOCTL_CTRL_COMMAND_2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_AF_CTRL \
	_IOR(MSM_CAM_IOCTL_MAGIC, 25, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_AF_CTRL_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 26, struct msm_ctrl_cmt_t *)

#define MSM_CAM_IOCTL_CONFIG_VPE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 27, struct msm_camera_vpe_cfg_cmd *)

#define MSM_CAM_IOCTL_AXI_VPE_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 28, struct msm_camera_vpe_cfg_cmd *)

#define MSM_CAM_IOCTL_STROBE_FLASH_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 29, uint32_t *)

#define MSM_CAM_IOCTL_STROBE_FLASH_CHARGE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 30, uint32_t *)

#define MSM_CAM_IOCTL_STROBE_FLASH_RELEASE \
	_IO(MSM_CAM_IOCTL_MAGIC, 31)

#define MSM_CAM_IOCTL_FLASH_CTRL \
	_IOW(MSM_CAM_IOCTL_MAGIC, 32, struct flash_ctrl_data *)

#define MSM_CAM_IOCTL_ERROR_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 33, uint32_t *)

#define MSM_CAM_IOCTL_ABORT_CAPTURE \
	_IO(MSM_CAM_IOCTL_MAGIC, 34)

#define MSM_CAM_IOCTL_SET_FD_ROI \
	_IOW(MSM_CAM_IOCTL_MAGIC, 35, struct fd_roi_info *)

#define MSM_CAM_IOCTL_GET_CAMERA_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 36, struct msm_camera_info *)

#define MSM_CAMERA_LED_OFF  0
#define MSM_CAMERA_LED_LOW  1
#define MSM_CAMERA_LED_HIGH 2

#define MSM_CAMERA_STROBE_FLASH_NONE 0
#define MSM_CAMERA_STROBE_FLASH_XENON 1

#define MSM_MAX_CAMERA_SENSORS  5
#define MAX_SENSOR_NAME 32

#define PP_SNAP  0x01
#define PP_RAW_SNAP ((0x01)<<1)
#define PP_PREV  ((0x01)<<2)
#define PP_MASK		(PP_SNAP|PP_RAW_SNAP|PP_PREV)

#define MSM_CAM_CTRL_CMD_DONE  0
#define MSM_CAM_SENSOR_VFE_CMD 1

/*****************************************************
 *  structure
 *****************************************************/

/* define five type of structures for userspace <==> kernel
 * space communication:
 * command 1 - 2 are from userspace ==> kernel
 * command 3 - 4 are from kernel ==> userspace
 *
 * 1. control command: control command(from control thread),
 *                     control status (from config thread);
 */
struct msm_ctrl_cmd {
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
	uint32_t timeout_ms;
	int resp_fd; /* FIXME: to be used by the kernel, pass-through for now */
};

struct msm_vfe_evt_msg {
	unsigned short type;	/* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len;	/* size in, number of bytes out */
	uint32_t frame_id;
	void *data;
};

struct msm_isp_evt_msg {
	unsigned short type;	/* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len;	/* size in, number of bytes out */
	/* maximum possible data size that can be
i	  sent to user space as v4l2 data structure
	  is only of 64 bytes */
	uint8_t data[48];
};
struct msm_vpe_evt_msg {
	unsigned short type; /* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len; /* size in, number of bytes out */
	uint32_t frame_id;
	void *data;
};
struct msm_isp_stats_event_ctrl {
	unsigned short resptype;
	union {
		struct msm_isp_evt_msg isp_msg;
		struct msm_ctrl_cmd ctrl;
	} isp_data;
};

#define MSM_CAM_RESP_CTRL         0
#define MSM_CAM_RESP_STAT_EVT_MSG 1
#define MSM_CAM_RESP_V4L2         2
#define MSM_CAM_RESP_MAX          3

/* this one is used to send ctrl/status up to config thread */
struct msm_stats_event_ctrl {
	/* 0 - ctrl_cmd from control thread,
	 * 1 - stats/event kernel,
	 * 2 - V4L control or read request */
	int resptype;
	int timeout_ms;
	struct msm_ctrl_cmd ctrl_cmd;
	/* struct  vfe_event_t  stats_event; */
	struct msm_vfe_evt_msg stats_event;
};

/* 2. config command: config command(from config thread); */
struct msm_camera_cfg_cmd {
	/* what to config:
	 * 1 - sensor config, 2 - vfe config */
	uint16_t cfg_type;

	/* sensor config type */
	uint16_t cmd_type;
	uint16_t queue;
	uint16_t length;
	void *value;
};

#define CMD_GENERAL			0
#define CMD_AXI_CFG_OUT1		1
#define CMD_AXI_CFG_SNAP_O1_AND_O2	2
#define CMD_AXI_CFG_OUT2		3
#define CMD_PICT_T_AXI_CFG		4
#define CMD_PICT_M_AXI_CFG		5
#define CMD_RAW_PICT_AXI_CFG		6

#define CMD_FRAME_BUF_RELEASE		7
#define CMD_PREV_BUF_CFG		8
#define CMD_SNAP_BUF_RELEASE		9
#define CMD_SNAP_BUF_CFG		10
#define CMD_STATS_DISABLE		11
#define CMD_STATS_AEC_AWB_ENABLE	12
#define CMD_STATS_AF_ENABLE		13
#define CMD_STATS_AEC_ENABLE		14
#define CMD_STATS_AWB_ENABLE		15
#define CMD_STATS_ENABLE  		16

#define CMD_STATS_AXI_CFG		17
#define CMD_STATS_AEC_AXI_CFG		18
#define CMD_STATS_AF_AXI_CFG 		19
#define CMD_STATS_AWB_AXI_CFG		20
#define CMD_STATS_RS_AXI_CFG		21
#define CMD_STATS_CS_AXI_CFG		22
#define CMD_STATS_IHIST_AXI_CFG		23
#define CMD_STATS_SKIN_AXI_CFG		24

#define CMD_STATS_BUF_RELEASE		25
#define CMD_STATS_AEC_BUF_RELEASE	26
#define CMD_STATS_AF_BUF_RELEASE	27
#define CMD_STATS_AWB_BUF_RELEASE	28
#define CMD_STATS_RS_BUF_RELEASE	29
#define CMD_STATS_CS_BUF_RELEASE	30
#define CMD_STATS_IHIST_BUF_RELEASE	31
#define CMD_STATS_SKIN_BUF_RELEASE	32

#define UPDATE_STATS_INVALID		33
#define CMD_AXI_CFG_SNAP_GEMINI		34
#define CMD_AXI_CFG_SNAP		35
#define CMD_AXI_CFG_PREVIEW		36
#define CMD_AXI_CFG_VIDEO		37

#define CMD_STATS_IHIST_ENABLE 38
#define CMD_STATS_RS_ENABLE 39
#define CMD_STATS_CS_ENABLE 40
#define CMD_VPE 41
#define CMD_AXI_CFG_VPE 42

/* vfe config command: config command(from config thread)*/
struct msm_vfe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

struct msm_vpe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

#define MAX_CAMERA_ENABLE_NAME_LEN 32
struct camera_enable_cmd {
	char name[MAX_CAMERA_ENABLE_NAME_LEN];
};

#define MSM_PMEM_OUTPUT1		0
#define MSM_PMEM_OUTPUT2		1
#define MSM_PMEM_OUTPUT1_OUTPUT2	2
#define MSM_PMEM_THUMBNAIL		3
#define MSM_PMEM_MAINIMG		4
#define MSM_PMEM_RAW_MAINIMG		5
#define MSM_PMEM_AEC_AWB		6
#define MSM_PMEM_AF			7
#define MSM_PMEM_AEC			8
#define MSM_PMEM_AWB			9
#define MSM_PMEM_RS		    	10
#define MSM_PMEM_CS	    		11
#define MSM_PMEM_IHIST			12
#define MSM_PMEM_SKIN			13
#define MSM_PMEM_VIDEO			14
#define MSM_PMEM_PREVIEW		15
#define MSM_PMEM_VIDEO_VPE		16
#define MSM_PMEM_MAX			17

#define STAT_AEAW			0
#define STAT_AEC			1
#define STAT_AF				2
#define STAT_AWB			3
#define STAT_RS				4
#define STAT_CS				5
#define STAT_IHIST			6
#define STAT_SKIN			7
#define STAT_MAX			8

#define FRAME_PREVIEW_OUTPUT1		0
#define FRAME_PREVIEW_OUTPUT2		1
#define FRAME_SNAPSHOT			2
#define FRAME_THUMBNAIL			3
#define FRAME_RAW_SNAPSHOT		4
#define FRAME_MAX			5

struct msm_pmem_info {
	int type;
	int fd;
	void *vaddr;
	uint32_t offset;
	uint32_t len;
	uint32_t y_off;
	uint32_t cbcr_off;
	uint8_t active;
};

struct outputCfg {
	uint32_t height;
	uint32_t width;

	uint32_t window_height_firstline;
	uint32_t window_height_lastline;
};

#define OUTPUT_1	0
#define OUTPUT_2	1
#define OUTPUT_1_AND_2            2   /* snapshot only */
#define OUTPUT_1_AND_3            3   /* video */
#define CAMIF_TO_AXI_VIA_OUTPUT_2 4
#define OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2 5
#define OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 6
#define LAST_AXI_OUTPUT_MODE_ENUM = OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 7

#define MSM_FRAME_PREV_1	0
#define MSM_FRAME_PREV_2	1
#define MSM_FRAME_ENC		2

#define OUTPUT_TYPE_P		(1<<0)
#define OUTPUT_TYPE_T		(1<<1)
#define OUTPUT_TYPE_S		(1<<2)
#define OUTPUT_TYPE_V		(1<<3)
#define OUTPUT_TYPE_L		(1<<4)

struct fd_roi_info {
	void *info;
	int info_len;
};

struct msm_frame {
	struct timespec ts;
	int path;
	unsigned long buffer;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;

	void *cropinfo;
	int croplen;
	uint32_t error_code;
	struct fd_roi_info roi_info;
};

#define MSM_CAMERA_ERR_MASK (0xFFFFFFFF & 1)

struct msm_stats_buf {
	int type;
	unsigned long buffer;
	int fd;
};

#define MSM_V4L2_VID_CAP_TYPE	0
#define MSM_V4L2_STREAM_ON	1
#define MSM_V4L2_STREAM_OFF	2
#define MSM_V4L2_SNAPSHOT	3
#define MSM_V4L2_QUERY_CTRL	4
#define MSM_V4L2_GET_CTRL	5
#define MSM_V4L2_SET_CTRL	6
#define MSM_V4L2_QUERY		7
#define MSM_V4L2_GET_CROP	8
#define MSM_V4L2_SET_CROP	9
#define MSM_V4L2_MAX		10

#define V4L2_CAMERA_EXIT 	43
struct crop_info {
	void *info;
	int len;
};

struct msm_postproc {
	int ftnum;
	struct msm_frame fthumnail;
	int fmnum;
	struct msm_frame fmain;
};

struct msm_snapshot_pp_status {
	void *status;
};

#define CFG_SET_MODE			0
#define CFG_SET_EFFECT			1
#define CFG_START			2
#define CFG_PWR_UP			3
#define CFG_PWR_DOWN			4
#define CFG_WRITE_EXPOSURE_GAIN		5
#define CFG_SET_DEFAULT_FOCUS		6
#define CFG_MOVE_FOCUS			7
#define CFG_REGISTER_TO_REAL_GAIN	8
#define CFG_REAL_TO_REGISTER_GAIN	9
#define CFG_SET_FPS			10
#define CFG_SET_PICT_FPS		11
#define CFG_SET_BRIGHTNESS		12
#define CFG_SET_CONTRAST		13
#define CFG_SET_ZOOM			14
#define CFG_SET_EXPOSURE_MODE		15
#define CFG_SET_WB			16
#define CFG_SET_ANTIBANDING		17
#define CFG_SET_EXP_GAIN		18
#define CFG_SET_PICT_EXP_GAIN		19
#define CFG_SET_LENS_SHADING		20
#define CFG_GET_PICT_FPS		21
#define CFG_GET_PREV_L_PF		22
#define CFG_GET_PREV_P_PL		23
#define CFG_GET_PICT_L_PF		24
#define CFG_GET_PICT_P_PL		25
#define CFG_GET_AF_MAX_STEPS		26
#define CFG_GET_PICT_MAX_EXP_LC		27
#define CFG_SEND_WB_INFO    28
#define CFG_MAX 			29

#ifndef CONFIG_MACH_ARIESVE
//pault
#define CFG_SET_EXPOSURE_VALUE 30
#define CFG_SET_ROTATION 31
#define CFG_SET_DATALINE_CHECK 32
#endif

#define MOVE_NEAR	0
#define MOVE_FAR	1

#define SENSOR_PREVIEW_MODE		0
#define SENSOR_SNAPSHOT_MODE		1
#define SENSOR_RAW_SNAPSHOT_MODE	2
#define SENSOR_VIDEO_120FPS_MODE	3
#define SENSOR_SNAPSHOT_TRANSFER	4

#define SENSOR_QTR_SIZE			0
#define SENSOR_FULL_SIZE		1
#define SENSOR_QVGA_SIZE		2
#define SENSOR_INVALID_SIZE		3

#define CAMERA_EFFECT_OFF		0
#define CAMERA_EFFECT_MONO		1
#define CAMERA_EFFECT_NEGATIVE		2
#define CAMERA_EFFECT_SOLARIZE		3
#define CAMERA_EFFECT_SEPIA		4
#define CAMERA_EFFECT_POSTERIZE		5
#define CAMERA_EFFECT_WHITEBOARD	6
#define CAMERA_EFFECT_BLACKBOARD	7
#define CAMERA_EFFECT_AQUA		8
#define CAMERA_EFFECT_MAX		9

struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
};

struct focus_cfg {
	int32_t steps;
	int dir;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};
struct wb_info_cfg {
	uint16_t red_gain;
	uint16_t green_gain;
	uint16_t blue_gain;
};
struct sensor_cfg_data {
	int cfgtype;
	int mode;
	int rs;
	uint8_t max_steps;

	union {
		int8_t effect;
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
#ifndef CONFIG_MACH_ARIESVE
//pault
		int8_t ev;
		int8_t wb;
		int8_t rotation;
		int8_t dataline;
#endif
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg exp_gain;
		struct focus_cfg focus;
		struct fps_cfg fps;
		struct wb_info_cfg wb_info;
	} cfg;
};

enum flash_type {
	LED_FLASH,
	STROBE_FLASH,
};

enum strobe_flash_ctrl_type {
	STROBE_FLASH_CTRL_INIT,
	STROBE_FLASH_CTRL_CHARGE,
	STROBE_FLASH_CTRL_RELEASE
};

struct strobe_flash_ctrl_data {
	enum strobe_flash_ctrl_type type;
	int charge_en;
};

struct msm_camera_info {
	int num_cameras;
	uint8_t has_3d_support[MSM_MAX_CAMERA_SENSORS];
	uint8_t is_internal_cam[MSM_MAX_CAMERA_SENSORS];
	uint32_t s_mount_angle[MSM_MAX_CAMERA_SENSORS];
};

struct flash_ctrl_data {
	int flashtype;
	union {
		int led_state;
		struct strobe_flash_ctrl_data strobe_ctrl;
	} ctrl_data;
};

#define GET_NAME			0
#define GET_PREVIEW_LINE_PER_FRAME	1
#define GET_PREVIEW_PIXELS_PER_LINE	2
#define GET_SNAPSHOT_LINE_PER_FRAME	3
#define GET_SNAPSHOT_PIXELS_PER_LINE	4
#define GET_SNAPSHOT_FPS		5
#define GET_SNAPSHOT_MAX_EP_LINE_CNT	6

struct msm_camsensor_info {
	char name[MAX_SENSOR_NAME];
	uint8_t flash_enabled;
	int8_t total_steps;
};

#if defined (CONFIG_OEM_CAMERA)
typedef struct{
	uint32_t device_id;
	uint32_t cmd;
	uint32_t  value_1;
	uint32_t  value_2;
#ifndef CONFIG_MACH_ARIESVE
    uint32_t  value_3;//pault
#endif
	void* p;
} sensor_ext_cfg_data;

typedef struct {
	char company;
	char module_vesion;
	char year;
	char month;
	char update_times[2];
} sensor_version_info;

typedef struct {
	uint32_t dev_num;
	char module_name[10];
} sensor_name_info;

struct sensor_version {
	unsigned int major;
	unsigned int minor;
};

struct sensor_date_info {
	unsigned int year;
	unsigned int month;
	unsigned int date;
};

struct sensor_sensor_maker{
	unsigned int maker;
	unsigned int optical;
};

struct sensor_version_af{
	unsigned int low;
	unsigned int high;
};

struct sensor_gamma{
	unsigned int rg_low;
	unsigned int rg_high;
	unsigned int bg_low;
	unsigned int bg_high;
};

struct sensor_firmware_info
{
    struct sensor_version fw;
    struct sensor_version prm;
    struct sensor_date_info dateinfo;
    struct sensor_sensor_maker sensor_info;
    struct sensor_version_af af_info;    
    struct sensor_gamma gamma;    
    unsigned int fw_dump_size;
	unsigned  int sensor_version;
};

struct gps_info_common {
	unsigned int 	direction;
	unsigned int 	dgree;
	unsigned int	minute;
	unsigned int	second;
};

enum v4l2_blur
{
	BLUR_LEVEL_0 = 0,
	BLUR_LEVEL_1,
	BLUR_LEVEL_2,
	BLUR_LEVEL_3,
	BLUR_LEVEL_MAX,
};

int mc7_sensor_ext_config(void __user *arg);
int ce147_sensor_ext_config(void __user *arg);
int s5ka3dfx_sensor_ext_config(void __user *arg);
int s5k4ecgx_sensor_ext_config(void __user *arg);
#ifndef CONFIG_MACH_ARIESVE
int s5k4ecgx_sensor_esd_detected();
int s5k5ccaf_sensor_esd_detected(); //ESD
int sr030pc30_sensor_esd_detected(); //ESD
#endif
int ce147_get_fw_data(void __user *arg);
int sr030pc30_sensor_ext_config(void __user *argp);
#ifndef CONFIG_MACH_ARIESVE
int s5k5ccaf_sensor_ext_config(void __user *arg);
#ifdef CONFIG_SENSOR_SR130PC10
int sr130pc10_sensor_ext_config(void __user *argp);
#endif
#endif
#define MSM_CAM_IOCTL_EXT_CONFIG  _IOWR(MSM_CAM_IOCTL_MAGIC, 50, sensor_ext_cfg_data)
#define MSM_CAM_IOCTL_FIRMWARE_UPDATE  _IOWR(MSM_CAM_IOCTL_MAGIC, 51, sensor_ext_cfg_data)
#define MSM_CAM_IOCTL_READ_VERSION_INFO  _IOWR(MSM_CAM_IOCTL_MAGIC, 52, struct sensor_firmware_info)
#define MSM_CAM_IOCTL_READ_MODULE_NAME  _IOWR(MSM_CAM_IOCTL_MAGIC, 53, sensor_name_info)

#define	EXIF_EXPOSURE_TIME		        0
#define	EXIF_TV					1
#define	EXIF_AV					2
#define	EXIF_BV					3
#define	EXIF_EBV				4
#define	EXIF_ISO				5
#define	EXIF_FLASH				6

enum ext_cfg_command
{
    EXT_CFG_SET_FLASH = 0,
    EXT_CFG_SET_FLASH_MODE,
    EXT_CFG_SET_AUTO_CONTRAST,
    EXT_CFG_SET_SCENE,
    EXT_CFG_SET_SHARPNESS,
    EXT_CFG_SET_EFFECT,
    EXT_CFG_SET_SATURATION,
    EXT_CFG_SET_ISO,
    EXT_CFG_SET_WB,
    EXT_CFG_SET_CONTRAST,
    EXT_CFG_SET_BRIGHTNESS,
    EXT_CFG_SET_ZOOM,
    EXT_CFG_SET_FPS,
    EXT_CFG_SET_FPS_MODE,
    EXT_CFG_SET_AF_MODE,
    EXT_CFG_SET_AF_START,
    EXT_CFG_SET_AF_STOP,
    EXT_CFG_SET_AF_OPERATION,
    EXT_CFG_GET_AF_STATUS,
    EXT_CFG_SET_TOUCHAF_POS,
    EXT_CFG_SET_FACE_DETECT,
    EXT_CFG_SET_METERING,
    EXT_CFG_SET_CONTINUOUS_AF,
    EXT_CFG_SET_PREVIEW_SIZE,
    EXT_CFG_SET_PICTURE_SIZE,
    EXT_CFG_SET_JPEG_QUALITY,
    EXT_CFG_SET_ANTISHAKE,
    EXT_CFG_SET_WDR,
    EXT_CFG_SET_EXIF,
    EXT_CFG_SET_DTP,
    EXT_CFG_SET_AE_AWB,
    EXT_CFG_SET_FRONT_CAMERA_MODE,
    EXT_CFG_SET_BEAUTY,
    EXT_CFG_SET_VINTAGEMODE,
    EXT_CFG_SET_BATCH_REFLECTION,
    EXT_CFG_GET_JPEG_SIZE,
    EXT_CFG_UPDATE_FIRMWARE,
    EXT_CFG_DUMP_FIRMWARE,
    EXT_CFG_SET_GPS_LATITUDE,
    EXT_CFG_SET_GPS_LONGITUDE,
    EXT_CFG_SET_GPS_ALTITUDE,
    EXT_CFG_SET_GPS_TIMESTAMP,
    EXT_CFG_SET_EXIF_TIME_INFO,
    EXT_CFG_SET_GPS_PROCESSINGMETHOD,
    EXT_CFG_SET_EXIF_ORIENTATION_INFO,
    EXT_CFG_SET_BLUR,
    EXT_CFG_SET_THUMB_NULL,
    EXT_CFG_SET_CAM_MODE,
    EXT_CFG_GET_EXIF_INFO,
    EXT_CFG_GET_FLASH_INFO,
    EXT_CFG_GET_VGACAM_ROTATED, // kurtlee    
    EXT_CFG_TEST_ESD,
    EXT_CFG_MAX,
#ifndef CONFIG_MACH_ARIESVE
    EXT_CFG_TEMP, //TELECA_BATTERY
#endif
};

enum ext_cfg_command_cammode
{
	EXT_CFG_CAM_MODE_CAMERA,
	EXT_CFG_CAM_MODE_CAMCORDER,
	EXT_CFG_CAM_MODE_FACTORY_TEST,
};

enum ext_cfg_command_jpeq_quality
{
	EXT_CFG_JPEG_QUALITY_SUPERFINE,
	EXT_CFG_JPEG_QUALITY_FINE,
	EXT_CFG_JPEG_QUALITY_NORMAL,
};

enum ext_cfg_command_auto_contrast
{
	EXT_CFG_AUTO_CONTRAST_ON,
	EXT_CFG_AUTO_CONTRAST_OFF,
};

enum ext_cfg_command_framerate
{
	EXT_CFG_FRAME_AUTO = 0,
	EXT_CFG_FRAME_FIX_15 = 15,
	EXT_CFG_FRAME_FIX_20 = 20,
	EXT_CFG_FRAME_FIX_30 = 30,
};

enum ext_cfg_command_effect
{
	EXT_CFG_EFFECT_NORMAL,
	EXT_CFG_EFFECT_NEGATIVE,
	EXT_CFG_EFFECT_MONO,
	EXT_CFG_EFFECT_SEPIA,
};

enum ext_cfg_command_whitebalance
{
	EXT_CFG_WB_AUTO,
	EXT_CFG_WB_DAYLIGHT,
	EXT_CFG_WB_CLOUDY,
	EXT_CFG_WB_FLUORESCENT,
	EXT_CFG_WB_INCANDESCENT,
};

enum ext_cfg_command_brightness
{                   
#ifdef CONFIG_MACH_ARIESVE      
	EXT_CFG_BR_STEP_M_4,
#else
	EXT_CFG_BR_STEP_M_4 = -4,
#endif
	EXT_CFG_BR_STEP_M_3,
	EXT_CFG_BR_STEP_M_2,
	EXT_CFG_BR_STEP_M_1,
	EXT_CFG_BR_STEP_0,
	EXT_CFG_BR_STEP_P_1,
	EXT_CFG_BR_STEP_P_2,
	EXT_CFG_BR_STEP_P_3,
	EXT_CFG_BR_STEP_P_4,
};

enum ext_cfg_command_contrast
{
	EXT_CFG_CR_STEP_M_2,               
	EXT_CFG_CR_STEP_M_1,                
	EXT_CFG_CR_STEP_0,                  
	EXT_CFG_CR_STEP_P_1,                
	EXT_CFG_CR_STEP_P_2,                
};

enum ext_cfg_command_saturation
{
	EXT_CFG_SA_STEP_M_2,                
	EXT_CFG_SA_STEP_M_1,                
	EXT_CFG_SA_STEP_0,                  
	EXT_CFG_SA_STEP_P_1,                
	EXT_CFG_SA_STEP_P_2,                
};

enum ext_cfg_command_sharpness
{
	EXT_CFG_SP_STEP_M_2,                
	EXT_CFG_SP_STEP_M_1,                
	EXT_CFG_SP_STEP_0,                 
	EXT_CFG_SP_STEP_P_1,                
	EXT_CFG_SP_STEP_P_2,                
};

enum ext_cfg_command_iso
{
	EXT_CFG_ISO_AUTO,
	EXT_CFG_ISO_50,
	EXT_CFG_ISO_100,
	EXT_CFG_ISO_200,
	EXT_CFG_ISO_400,
};

enum ext_cfg_command_metering
{
	EXT_CFG_METERING_NORMAL, //matrix
	EXT_CFG_METERING_SPOT,
	EXT_CFG_METERING_CENTER,
};

enum ext_cfg_command_scene
{
	EXT_CFG_SCENE_OFF,
	EXT_CFG_SCENE_PORTRAIT,
	EXT_CFG_SCENE_LANDSCAPE,
	EXT_CFG_SCENE_SPORTS,
	EXT_CFG_SCENE_PARTY,
	EXT_CFG_SCENE_BEACH,
	EXT_CFG_SCENE_SUNSET,
	EXT_CFG_SCENE_DAWN,
	EXT_CFG_SCENE_FALL,
	EXT_CFG_SCENE_NIGHTSHOT,
	EXT_CFG_SCENE_BACKLIGHT,
	EXT_CFG_SCENE_FIREWORK,
	EXT_CFG_SCENE_TEXT,
	EXT_CFG_SCENE_CANDLE,
};

enum ext_cfg_command_af_operation
{
	EXT_CFG_AF_CHECK_STATUS,
	EXT_CFG_AF_OFF,
	EXT_CFG_AF_SET_NORMAL,
	EXT_CFG_AF_SET_MACRO,
	EXT_CFG_AF_DO,
	EXT_CFG_AF_SET_MANUAL,
	EXT_CFG_AF_CHECK_2nd_STATUS,
	EXT_CFG_AF_SET_AE_FOR_FLASH,
	EXT_CFG_AF_BACK_AE_FOR_FLASH,
	EXT_CFG_AF_CHECK_AE_STATUS,
	EXT_CFG_AF_POWEROFF,
};

enum ext_cfg_command_af_status
{
	EXT_CFG_AF_PROGRESS = 1,
	EXT_CFG_AF_SUCCESS,
	EXT_CFG_AF_LOWCONF,//Fail
	EXT_CFG_AF_CANCELED,
	EXT_CFG_AF_TIMEOUT,
	EXT_CFG_AE_STABLE,
	EXT_CFG_AE_UNSTABLE,
};

enum ext_cfg_command_ae_awb
{
	EXT_CFG_AE_LOCK,
	EXT_CFG_AE_UNLOCK,
	EXT_CFG_AWB_LOCK,
	EXT_CFG_AWB_UNLOCK,
};

enum ext_cfg_command_cpu_policy
{
	EXT_CFG_CPU_CONSERVATIVE,
	EXT_CFG_CPU_ONDEMAND,
	EXT_CFG_CPU_PERFORMANCE,
};

enum ext_cfg_command_dtp
{
	EXT_CFG_DTP_OFF,
	EXT_CFG_DTP_ON,
};

enum ext_cfg_command_zoom
{
	EXT_CFG_ZOOM_STEP_0,
	EXT_CFG_ZOOM_STEP_1,
	EXT_CFG_ZOOM_STEP_2,
	EXT_CFG_ZOOM_STEP_3,
	EXT_CFG_ZOOM_STEP_4,
	EXT_CFG_ZOOM_STEP_5,
	EXT_CFG_ZOOM_STEP_6,
	EXT_CFG_ZOOM_STEP_7,
	EXT_CFG_ZOOM_STEP_8,
};

enum ext_cfg_command_picture_size
{
	EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M,
#ifndef CONFIG_MACH_ARIESVE
	EXT_CFG_SNAPSHOT_SIZE_2560x1536_4M_WIDE,
#endif
	EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M,
#ifndef CONFIG_MACH_ARIESVE
	EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE,
#endif
	EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M,
#ifndef CONFIG_MACH_ARIESVE
	EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE,
#endif
	EXT_CFG_SNAPSHOT_SIZE_1280x960_1M,
#ifndef CONFIG_MACH_ARIESVE
	EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE,
#endif
	EXT_CFG_SNAPSHOT_SIZE_640x480_VGA,
	EXT_CFG_SNAPSHOT_SIZE_320x240_QVGA,
};

enum ext_cfg_command_preview_size
{
#ifndef CONFIG_MACH_ARIESVE
      EXT_CFG_PREVIEW_SIZE_1280x720_D1,
	EXT_CFG_PREVIEW_SIZE_800x480_WVGA,
#endif
	EXT_CFG_PREVIEW_SIZE_720x480_D1,
	EXT_CFG_PREVIEW_SIZE_640x480_VGA,
	EXT_CFG_PREVIEW_SIZE_320x240_QVGA,
	EXT_CFG_PREVIEW_SIZE_176x144_QCIF,
};

enum ext_cfg_command_flash
{
	EXT_CFG_FLASH_ON,
	EXT_CFG_FLASH_OFF,
	EXT_CFG_FLASH_AUTO,
	EXT_CFG_FLASH_TURN_ON,
	EXT_CFG_FLASH_TURN_OFF,
};
#ifndef CONFIG_MACH_ARIESVE
//pault
enum ext_cfg_command_exposure
{
  CAMERA_EXPOSURE_NEGATIVE_2,
  CAMERA_EXPOSURE_NEGATIVE_1,
  CAMERA_EXPOSURE_0,
  CAMERA_EXPOSURE_POSITIVE_1,
  CAMERA_EXPOSURE_POSITIVE_2
};

enum ext_cfg_command_wb
{
  CAMERA_WB_MIN_MINUS_1,
  CAMERA_WB_AUTO = 1,  /* This list must match aeecamera.h */
  CAMERA_WB_CUSTOM,
  CAMERA_WB_INCANDESCENT,
  CAMERA_WB_FLUORESCENT,
  CAMERA_WB_DAYLIGHT,
  CAMERA_WB_CLOUDY_DAYLIGHT,
  CAMERA_WB_TWILIGHT,
  CAMERA_WB_SHADE,
  CAMERA_WB_MAX_PLUS_1
};
#endif
#endif /* CONFIG_OEM_CAMERA */

#endif /* __LINUX_MSM_CAMERA_H */
