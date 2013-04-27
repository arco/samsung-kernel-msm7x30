//#define MDNIE_TUNING

#define CE147_DRIVER_NAME	"CE147"
#define CE147_FW_F2_PATH	"/system/firmware/CE147F02.bin"

#define FORMAT_FLAGS_COMPRESSED		0x3
#define SENSOR_JPEG_SNAPSHOT_MEMSIZE	0x360000

#define CE147_DEBUG
#define CE147_INFO
#define CE147_CAM_POWER

#define FIN printk(KERN_INFO "[CAMDRV] ce147.c:%05d: %s\n", __LINE__, __func__)

#define CAMDRV_ENABLE_DEBUG
#ifdef CAMDRV_ENABLE_DEBUG
#define CAMDRV_DEBUG(fmt, arg...)  \
                do{\
                printk("[CAMDRV/CE147] " fmt,##arg);}\
                while(0)
#else
#define CAMDRV_DEBUG(fmt, arg...)  
#endif

#ifdef CE147_DEBUG
#define ce147_msg	printk
#else
#define ce147_msg 	printk
#endif

#ifdef CE147_INFO
#define ce147_info	printk
#else
#define ce147_info	printk
#endif

/* Default resolution & pixelformat. plz ref ce147_platform.h */
#define DEFAULT_PIX_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */
#define DEFUALT_MCLK		24000000
#define POLL_TIME_MS		10

/* Camera ISP command */
#define CMD_VERSION			0x00
#define DATA_VERSION_FW		0x00
#define DATA_VERSION_DATE	0x01
#define CMD_GET_BATCH_REFLECTION_STATUS	0x02
#define DATA_VERSION_SENSOR	0x03
#define CMD_HD_PREVIEW		0x03
#define CMD_SET_WB			0x04
#define DATA_VERSION_AF		0x05
#define CMD_SET_FLASH_MANUAL	0x06
#define CMD_SET_EXIF_CTRL    	0x07//ykh
#define CMD_AE_WB_LOCK		0x11
#define CMD_SET_ANTI_BANDING	0x14
#define CMD_SET_WB_AUTO		0x1A
#define CMD_SET_AUTO_FOCUS_MODE 0x20
#define CMD_START_AUTO_FOCUS_SEARCH 0x23
#define CMD_CHECK_AUTO_FOCUS_SEARCH 0x24
#define CMD_STOP_LENS_MOVEMENT 0x35
#define CMD_SET_EFFECT		0x3D
#define CMD_SET_TOUCH_AUTO_FOCUS 0x4D
#define CMD_START_OT		0x50
#define CMD_CHECK_OT		0x51
#define CMD_PREVIEW_SIZE	0x54
#define CMD_FPS				0x5A
#define CMD_SET_ANTI_SHAKE	0x5B
#define CMD_SET_DATA		0x65
#define CMD_DATA_OUT_REQ	0x66
#define CMD_PREVIEW			0x6B
#define	CMD_PREVIEW_STATUS	0x6C
#define CMD_CAPTURE_SIZE	0x73
#define CMD_BUFFERING_CAPTURE	0x74
#define CMD_SET_SMART_AUTO  0x82
#define CMD_GET_SMART_AUTO_STATUS 0x83
#define CMD_SET_WDR			0x88
#define CMD_JPEG_SIZE		0x8E
#define CMD_JPEG_BUFFERING	0x8F
#define CMD_JPEG_CONFIG		0x90
#define CMD_JPEG_BUFFERING2		0x92//ykh
#define CMD_SET_FACE_DETECTION 0x9A
#define CMD_SET_FACE_LOCK 	0x9C
#define CMD_INFO_EXIF		0xA0
#define CMD_INFO_MODEL		0xA1
#define CMD_INFO_ROT		0xA2
#define CMD_INFO_LONGITUDE_LATITUDE		0xA3
#define CMD_INFO_ALTITUDE		0xA4
#define CMD_SET_FLASH		0xB2
#define CMD_SET_DZOOM		0xB9
#define CMD_GET_DZOOM_LEVEL 0xBA
#define CMD_SET_EFFECT_SHOT 0xC0
#define DATA_VERSION_GAMMA	0x0E0
#define DATA_VERSION_SENSOR_MAKER 0xE0
#define CMD_CHECK_DATALINE	0xEC
#define CMD_INIT			0xF0
#define CMD_FW_INFO			0xF2
#define CMD_FWU_UPDATE		0xF3
#define CMD_FW_UPDATE		0xF4
#define CMD_FW_STATUS		0xF5
#define CMD_FW_DUMP			0xFB
#define CMD_GPS_TIMESTAMP		0xA7

enum ce147_oprmode {
	CE147_OPRMODE_VIDEO = 0,
	CE147_OPRMODE_IMAGE = 1,
};

/* Declare Funtion */
static int ce147_set_ae_awb(int val);
static int ce147_set_iso(int);
static int ce147_set_metering(int);
static int ce147_set_ev(int);
static int ce147_set_slow_ae(int);
static int ce147_set_gamma(int);
static int ce147_set_effect(int);
static int ce147_set_white_balance(int);
static int ce147_set_power(int);

struct ce147_ctrls_t {
	const struct msm_camera_sensor_info *sensordata;
};

enum ce147_frame_size {
	CE147_PREVIEW_VGA = 0,
	CE147_PREVIEW_WVGA,
	CE147_PREVIEW_VERTICAL_QCIF,
	CE147_PREVIEW_720P,
	CE147_PREVIEW_D1,
	CE147_PREVIEW_QCIF,
	CE147_PREVIEW_QVGA,
	CE147_PREVIEW_592x480,	
	CE147_CAPTURE_VGA, /* 640 x 480 */	
	CE147_CAPTURE_WVGA, /* 800 x 480 */
	CE147_CAPTURE_W1MP, /* 1600 x 960 */
	CE147_CAPTURE_2MP, /* UXGA  - 1600 x 1200 */
	CE147_CAPTURE_W2MP, /* 35mm Academy Offset Standard 1.66  - 2048 x 1232, 2.4MP */	
	CE147_CAPTURE_3MP, /* QXGA  - 2048 x 1536 */
	CE147_CAPTURE_W4MP, /* WQXGA - 2560 x 1536 */
	CE147_CAPTURE_5MP, /* 2560 x 1920 */
};

enum v4l2_scene_mode
{
  SCENE_MODE_OFF = 0,
  SCENE_MODE_LANDSCAPE = 1,
  SCENE_MODE_SNOW,
  SCENE_MODE_BEACH,
  SCENE_MODE_SUNSET,
  SCENE_MODE_NIGHT,
  SCENE_MODE_PORTRAIT,
  SCENE_MODE_BACKLIGHT,
  SCENE_MODE_SPORTS,
  SCENE_MODE_ANTISHAKE,
  SCENE_MODE_FLOWERS,
  SCENE_MODE_CANDLELIGHT,
  SCENE_MODE_FIREWORKS,
  SCENE_MODE_PARTY,
  SCENE_MODE_NIGHT_PORTRAIT,
  SCENE_MODE_THEATRE,
  SCENE_MODE_ACTION,
  SCENE_MODE_DAWN,
  SCENE_MODE_TEXT,
  SCENE_MODE_FALL_COLOR,
  SCENE_MODE_MAX,
};

enum v4l2_flash_mode
{
	FLASH_MODE_BASE,
	FLASH_MODE_OFF,
	FLASH_MODE_AUTO,
	FLASH_MODE_ON,
	FLASH_MODE_TORCH,
	FLASH_MODE_MAX,
};

enum v4l2_ev_mode {
	EV_MINUS_4	= -4,
	EV_MINUS_3	= -3,
	EV_MINUS_2	= -2,
	EV_MINUS_1	= -1,
	EV_DEFAULT	= 0,
	EV_PLUS_1	= 1,
	EV_PLUS_2	= 2,
	EV_PLUS_3	= 3,
	EV_PLUS_4	= 4,	
	EV_PLUS_5	= 5,	
};

enum v4l2_wb_mode {	
	WHITE_BALANCE_BASE = 0,
	WHITE_BALANCE_AUTO = 1,
	WHITE_BALANCE_TUNGSTEN = 3,
	WHITE_BALANCE_FLUORESCENT = 4,
	WHITE_BALANCE_SUNNY = 5,
	WHITE_BALANCE_CLOUDY = 6,	
	WHITE_BALANCE_FALLCOLOR,
	WHITE_BALANCE_SUNSET,
	WHITE_BALANCE_DAWN,
	WHITE_BALANCE_MAX,
};

enum v4l2_effect_mode {
	IMAGE_EFFECT_BASE = 0,
	IMAGE_EFFECT_BNW = 1,		
	IMAGE_EFFECT_NEGATIVE = 2,	
	IMAGE_EFFECT_SEPIA = 4,	
	IMAGE_EFFECT_NONE,
	IMAGE_EFFECT_AQUA,
	IMAGE_EFFECT_ANTIQUE,
	IMAGE_EFFECT_SHARPEN,
	IMAGE_EFFECT_MAX,
};

enum v4l2_iso_mode {
	ISO_AUTO = 0,
	ISO_50,
	ISO_100,
	ISO_200,
	ISO_400,
	ISO_800,
	ISO_1600,
	ISO_SPORTS,	
	ISO_NIGHT,
	ISO_MOVIE,
	ISO_FIREWORK,
	ISO_MAX,
};

enum v4l2_metering_mode {
	METERING_MATRIX,
	METERING_CENTER,
	METERING_SPOT,
	METERING_LAND,	
	METERING_HD,	
	METERING_MAX,
};

enum v4l2_contrast_mode {
	CONTRAST_MINUS_2 = 0,
	CONTRAST_MINUS_1,
	CONTRAST_DEFAULT,
	CONTRAST_PLUS_1,
	CONTRAST_PLUS_2,
	CONTRAST_MAX,
};

enum v4l2_saturation_mode {
	SATURATION_MINUS_2 = 0,
	SATURATION_MINUS_1,
	SATURATION_DEFAULT,
	SATURATION_PLUS_1,
	SATURATION_PLUS_2,
	SATURATION_MAX,
};

enum v4l2_sharpness_mode {
	SHARPNESS_MINUS_2 = 0,
	SHARPNESS_MINUS_1,
	SHARPNESS_DEFAULT,
	SHARPNESS_PLUS_1,
	SHARPNESS_PLUS_2,
	SHARPNESS_MAX,
};

enum v4l2_wdr_mode {
	WDR_OFF,
	WDR_ON,	
	WDR_MAX,
};

enum v4l2_anti_shake_mode {
	ANTI_SHAKE_OFF,
	ANTI_SHAKE_STILL_ON,	
	ANTI_SHAKE_MOVIE_ON,
	ANTI_SHAKE_MAX,
};

enum v4l2_touch_af {
	TOUCH_AF_STOP = 0,
	TOUCH_AF_START,
	TOUCH_AF_MAX,
};

enum v4l2_smart_auto {
	SMART_AUTO_OFF = 0,
	SMART_AUTO_ON,
	SMART_AUTO_MAX,
};

enum v4l2_vintage_mode {
	VINTAGE_MODE_BASE,
	VINTAGE_MODE_OFF,		
	VINTAGE_MODE_NORMAL,
	VINTAGE_MODE_WARM,
	VINTAGE_MODE_COOL,
	VINTAGE_MODE_BNW,
	VINTAGE_MODE_MAX,	
};

enum v4l2_zoom_level {
	ZOOM_LEVEL_0 = 0,
	ZOOM_LEVEL_1,
	ZOOM_LEVEL_2,
	ZOOM_LEVEL_3,
	ZOOM_LEVEL_4,
	ZOOM_LEVEL_5,
	ZOOM_LEVEL_6,
	ZOOM_LEVEL_7,
	ZOOM_LEVEL_8,
	ZOOM_LEVEL_9,
	ZOOM_LEVEL_10,
	ZOOM_LEVEL_11,
	ZOOM_LEVEL_12,
	ZOOM_LEVEL_MAX,
};

enum v4l2_face_detection {
	FACE_DETECTION_OFF = 0,
	FACE_DETECTION_ON,
	FACE_DETECTION_NOLINE,
	FACE_DETECTION_ON_BEAUTY,
	FACE_DETECTION_MAX,
};

enum v4l2_smart_auto_status {
	SMART_AUTO_STATUS_AUTO = 0,
	SMART_AUTO_STATUS_LANDSCAPE,
	SMART_AUTO_STATUS_PORTRAIT,
	SMART_AUTO_STATUS_MACRO,
	SMART_AUTO_STATUS_NIGHT,
	SMART_AUTO_STATUS_PORTRAIT_NIGHT,
	SMART_AUTO_STATUS_BACKLIT,
	SMART_AUTO_STATUS_PORTRAIT_BACKLIT,
	SMART_AUTO_STATUS_ANTISHAKE,
	SMART_AUTO_STATUS_PORTRAIT_ANTISHAKE,	
	SMART_AUTO_STATUS_MAX,
};

enum v4l2_auto_focus {
	AUTO_FOCUS_OFF = 0,
	AUTO_FOCUS_ON,
	AUTO_FOCUS_MAX,
};

enum v4l2_beauty_shot {
	BEAUTY_SHOT_OFF = 0,
	BEAUTY_SHOT_ON,
	BEAUTY_SHOT_MAX,
};

enum v4l2_ae_awb_lockunlock {
	AE_UNLOCK_AWB_UNLOCK = 0,
	AE_LOCK_AWB_UNLOCK,
	AE_UNLOCK_AWB_LOCK,
	AE_LOCK_AWB_LOCK,
	AE_AWB_MAX
};

enum v4l2_face_lock {
	FACE_LOCK_OFF = 0,
	FACE_LOCK_ON,
	FIRST_FACE_TRACKING,
	FACE_LOCK_MAX,
};

enum v4l2_focusmode {
	FOCUS_MODE_MACRO = 1,
	FOCUS_MODE_FD = 3,
	FOCUS_MODE_AUTO = 4,
	FOCUS_MODE_TOUCH = 5,
	FOCUS_MODE_MAX,
};

enum v4l2_obj_tracking_status {
	OBJECT_TRACKING_STATUS_BASE,
	OBJECT_TRACKING_STATUS_PROGRESSING,
	OBJECT_TRACKING_STATUS_SUCCESS, 	
	OBJECT_TRACKING_STATUS_FAIL,
	OBJECT_TRACKING_STATUS_MISSING,
	OBJECT_TRACKING_STATUS_MAX,
};

enum v4l2_ot_start_stop {
	OT_STOP = 0,
	OT_START,
	OT_MAX,
};

enum v4l2_caf_start_stop {
	CAF_STOP = 0,
	CAF_START,
	CAF_MAX,
};

enum v4l2_frame_rate {
	FRAME_RATE_AUTO = 0,
	FRAME_RATE_7 = 7,		
	FRAME_RATE_15 = 15,
	FRAME_RATE_30 = 30,
	FRAME_RATE_60 = 60,
	FRAME_RATE_120 = 120,
	FRAME_RATE_MAX
};

enum v4l2_anti_banding{
	ANTI_BANDING_AUTO = 0,
	ANTI_BANDING_50HZ = 1,
	ANTI_BANDING_60HZ = 2,
	ANTI_BANDING_OFF = 3,
};

enum v4l2_gamma_mode{
	GAMMA_OFF = 0,
	GAMMA_ON = 1,
	GAMMA_MAX,
};

enum v4l2_slow_ae_mode{
	SLOW_AE_OFF,
	SLOW_AE_ON,
	SLOW_AE_MAX,
};

enum v4l2_strobe_control {
	/* turn off the flash light */
	V4L2_STROBE_CONTROL_OFF		= 0,
	/* turn on the flash light */
	V4L2_STROBE_CONTROL_ON		= 1,
	/* act guide light before splash */
	V4L2_STROBE_CONTROL_AFGUIDE	= 2,
	/* charge the flash light */
	V4L2_STROBE_CONTROL_CHARGE	= 3,
};

enum v4l2_strobe_conf {
	V4L2_STROBE_OFF			= 0,	/* Always off */
	V4L2_STROBE_ON			= 1,	/* Always splashes */
	/* Auto control presets */
	V4L2_STROBE_AUTO		= 2,
	V4L2_STROBE_REDEYE_REDUCTION	= 3,
	V4L2_STROBE_SLOW_SYNC		= 4,
	V4L2_STROBE_FRONT_CURTAIN	= 5,
	V4L2_STROBE_REAR_CURTAIN	= 6,
	/* Extra manual control presets */
	/* keep turned on until turning off */
	V4L2_STROBE_PERMANENT		= 7,
	V4L2_STROBE_EXTERNAL		= 8,
};

enum v4l2_strobe_status {
	V4L2_STROBE_STATUS_OFF		= 0,
	/* while processing configurations */
	V4L2_STROBE_STATUS_BUSY		= 1,
	V4L2_STROBE_STATUS_ERR		= 2,
	V4L2_STROBE_STATUS_CHARGING	= 3,
	V4L2_STROBE_STATUS_CHARGED	= 4,
};

struct ce147_enum_framesize {
	/* mode is 0 for preview, 1 for capture */
	enum ce147_oprmode mode;
	unsigned int index;
	unsigned int width;
	unsigned int height;	
};

struct ce147_version {
	unsigned int major;
	unsigned int minor;
};

struct ce147_date_info {
	unsigned int year;
	unsigned int month;
	unsigned int date;
};

enum ce147_runmode {
	CE147_RUNMODE_NOTREADY,
	CE147_RUNMODE_IDLE, 
	CE147_RUNMODE_READY,
	CE147_RUNMODE_RUNNING, 
};

struct ce147_firmware {
	unsigned int addr;
	unsigned int size;
};

/* Camera functional setting values configured by user concept */
struct ce147_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;		/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;		/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;		/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;		/* Color FX (AKA Color tone) */
	unsigned int contrast;		/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct ce147_jpeg_param {
	unsigned int enable;
	unsigned int quality;
	unsigned int main_size;  /* Main JPEG file size */
	unsigned int thumb_size; /* Thumbnail file size */
	unsigned int main_offset;
	unsigned int thumb_offset;
	unsigned int postview_offset;
} ; 

struct ce147_position {
	int x;
	int y;
} ; 

struct ce147_gps_info{
	unsigned char ce147_gps_buf[8];
	unsigned char ce147_altitude_buf[4];
	unsigned long gps_timeStamp;//ykh
	char gps_processingmethod[50];//ykh
};

struct ce147_sensor_maker{
	unsigned int maker;
	unsigned int optical;
};

struct ce147_version_af{
	unsigned int low;
	unsigned int high;
};

struct ce147_gamma{
	unsigned int rg_low;
	unsigned int rg_high;
	unsigned int bg_low;
	unsigned int bg_high;
};

#if 0
struct tm {
   int     tm_sec;         /* seconds */
   int     tm_min;         /* minutes */
   int     tm_hour;        /* hours */
   int     tm_mday;        /* day of the month */
   int     tm_mon;         /* month */
   int     tm_year;        /* year */
   int     tm_wday;        /* day of the week */
   int     tm_yday;        /* day in the year */
   int     tm_isdst;       /* daylight saving time */

   long int tm_gmtoff;     /* Seconds east of UTC.  */
   const char *tm_zone;    /* Timezone abbreviation.  */
};
#endif

struct ce147_status_t {
	//struct ce147_platform_data *pdata;
	//struct v4l2_subdev sd;
	//struct v4l2_pix_format pix;
	//struct v4l2_fract timeperframe;
	struct ce147_userset userset;
	struct ce147_jpeg_param jpeg;
	struct ce147_version fw;
	struct ce147_version prm;
	struct ce147_date_info dateinfo;
	struct ce147_firmware fw_info;
	struct ce147_position position;
	struct ce147_sensor_maker sensor_info;	
	struct ce147_version_af af_info;
	struct ce147_gamma gamma;
	//struct v4l2_streamparm strm;
	struct ce147_gps_info gpsInfo;
	enum ce147_runmode runmode;
	enum ce147_oprmode oprmode;
	int framesize_index;
	int sensor_version;
	int freq;	/* MCLK in Hz */
	int fps;
	int fps_mode; //0:auto, 1:fixed
	int ot_status;
	int sa_status;
	int anti_banding;	
	int preview_size;
	int postview_size;
	unsigned int fw_dump_size;
	int hd_preview_on;
	int pre_af_status;
	int cur_af_status;
	struct ce147_version main_sw_fw;
	struct ce147_version main_sw_prm;
	struct ce147_date_info main_sw_dateinfo;
	int exif_orientation_info;
	int check_dataline;
	int hd_slow_ae;
	int hd_gamma;
	int iso;
	int metering;
	int ev;
	int effect;
	int wb;
	int saturation;
	int contrast;
	int sharpness;
	int aeawb;
	int focus_mode;
	struct tm *exifTimeInfo;
	int disable_aeawb_lock;
	int exif_ctrl;//ykh
	int thumb_null;//ykh
};
