/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/ioctl.h>

#include "mdp4_video_enhance.h"
#include "mdp4_video_tuning.h"
#include "msm_fb.h"
#include "mdp.h"
#include "mdp4.h"

#define MDP4_VIDEO_ENHANCE_TUNING
#define VIDEO_ENHANCE_DEBUG

#ifdef VIDEO_ENHANCE_DEBUG
#define DPRINT(x...)	printk(KERN_ERR "mdnie " x)
#else
#define DPRINT(x...)	
#endif

#define MAX_LUT_SIZE	256

unsigned int mDNIe_data[MAX_LUT_SIZE*3];
//unsigned int mDNIe_data_R[MAX_LUT_SIZE];
//unsigned int mDNIe_data_G[MAX_LUT_SIZE];
//unsigned int mDNIe_data_B[MAX_LUT_SIZE];
int mDNIe_data_sharpness;

typedef enum {
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE, 	// Not Support
	mDNIe_VIDEO_COLD_MODE, 	// Not Support
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI,
	mDNIe_GALLERY,
	mDNIe_BYPASS,  
#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
	mDNIe_DMB_MODE = 20,
	mDNIe_DMB_WARM_MODE, 	// Not Support
	mDNIe_DMB_COLD_MODE, 	// Not Support
#endif	
#ifdef CONFIG_TARGET_LOCALE_NTT
	mDNIe_ISDBT_MODE = 30,
	mDNIe_ISDBT_WARM_MODE,
	mDNIe_ISDBT_COLD_MODE,
#endif
#ifdef BROWSER_COLOR_TONE_SET
	mDNIe_BROWSER_TONE1	= 40,
	mDNIe_BROWSER_TONE2,
	mDNIe_BROWSER_TONE3,
#endif
} Lcd_mDNIe_UI;

typedef enum {
	mDNIe_DYNAMIC,
	mDNIe_STANDARD,
	mDNIe_MOVIE,
} Lcd_mDNIe_User_Set;


static struct class *mdnie_class;
struct device *mdnie_dev;
struct class *mdnieset_outdoor_class;
struct device *switch_mdnieset_outdoor_dev;

Lcd_mDNIe_UI current_mDNIe_Mode = mDNIe_UI_MODE; /* mDNIe Set Status Checking Value.*/
//Lcd_mDNIe_User_Set current_mDNIe_user_mode = mDNIe_STANDARD; /*mDNIe_user Set Status Checking Value.*/ //not support by mdp

//u8 current_mDNIe_OutDoor_OnOff = FALSE;  // not support by mdp
static bool g_mdine_enable = 0;

void lut_tune(int num, unsigned int *pLutTable );
void sharpness_tune(int num );
DEFINE_MUTEX(msm_fb_ioctl_lut_sem1);

#ifdef MDP4_VIDEO_ENHANCE_TUNING
#define MAX_FILE_NAME 128
#define TUNING_FILE_PATH "/sdcard/tuning/"

static int tuning_enable = 0;
static char tuning_filename[MAX_FILE_NAME];

static int load_tuning_data(char *filename);
static int parse_text(char *src, int len);
#endif


#ifdef MDP4_VIDEO_ENHANCE_TUNING
static int parse_text(char *src, int len)
{
	int i,count, ret;
	int index=0;
	int j = 0;
	char *str_line[300];
	char *sstart;
	char *c;
	unsigned int data1, data2, data3;
	int	sharpvalue;

	c = src;
	count = 0;
	sstart = c;
	sharpvalue = 0;

	for(i=0; i<len; i++,c++){
		char a = *c;
		if(a=='\r' || a=='\n'){
			if(c > sstart){
				str_line[count] = sstart;
				count++;
			}
			*c='\0';
			sstart = c+1;
		}
	}

	if(c > sstart){
		str_line[count] = sstart;
		count++;
	}

	ret = sscanf(str_line[0], "%d\n", &sharpvalue);
	DPRINT("sharp ret %d, sharpvalue %d\n",ret, sharpvalue);
	mDNIe_data_sharpness = sharpvalue;
	
	for(i=1; i<count; i++){
		DPRINT("line:%d, [start]%s[end]\n", i, str_line[i]);
		ret = sscanf(str_line[i], "0x%x, 0x%x, 0x%x\n", &data1, &data2,&data3);
		DPRINT("Result => [0x%2x 0x%2x 0x%2x] %s\n", data1, data2, data3, (ret==3)?"Ok":"Not available");
		DPRINT("ret => %d\n",ret);
		if(ret == 3) {   
//			mDNIe_data_R[index] = data1;
//			mDNIe_data_G[index] = data2;
//			mDNIe_data_B[index++] = data3;	
			mDNIe_data[j++] = data1;
			mDNIe_data[j++] = data2;
			mDNIe_data[j++] = data3;
			index++;
		}
	}
	return index;
}


static int load_tuning_data(char *filename)
{
    struct file *filp;
	char	*dp;
	long	l ;
	loff_t  pos;
	int     ret, num;
	mm_segment_t fs;

	DPRINT("[CMC623:INFO]:%s called loading file name : [%s]\n",__func__,filename);

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filename, O_RDONLY, 0);
	if(IS_ERR(filp)) 
	{
		printk(KERN_ERR "[CMC623:ERROR]:File open failed\n");
		return -1;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	DPRINT("[CMC623:INFO]: Loading File Size : %ld(bytes)", l);

	dp = kmalloc(l+10, GFP_KERNEL);	
	if(dp == NULL){
		DPRINT("[CMC623:ERROR]:Can't not alloc memory for tuning file load\n");
		filp_close(filp, current->files);
		return -1;
	}
	pos = 0;
	memset(dp, 0, l);
	DPRINT("[CMC623:INFO] : before vfs_read()\n");
	ret = vfs_read(filp, (char __user *)dp, l, &pos);   
	DPRINT("[CMC623:INFO] : after vfs_read()\n");

	if(ret != l) {
        DPRINT("[CMC623:ERROR] : vfs_read() filed ret : %d\n",ret);
        kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);

	set_fs(fs);
	num = parse_text(dp, l);

	if(!num) {
		DPRINT("[CMC623:ERROR]:Nothing to parse\n");
		kfree(dp);
		return -1;
	}

	DPRINT("[CMC623:INFO] : Loading Tuning Value's Count : %d", num);
	lut_tune(num, mDNIe_data);
	sharpness_tune(mDNIe_data_sharpness);

	kfree(dp);
	return num;
}


static ssize_t tuning_show(struct device *dev,
        struct device_attribute *attr, char *buf)

{
    int ret = 0;
    ret = sprintf(buf,"Tunned File Name : %s\n",tuning_filename);

    return ret;
}


static ssize_t tuning_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	char *pt;
    memset(tuning_filename,0,sizeof(tuning_filename));
    sprintf(tuning_filename,"%s%s",TUNING_FILE_PATH,buf);

	pt = tuning_filename;
    while(*pt)
	{
		if(*pt =='\r'|| *pt =='\n')
			{
			*pt = 0;
			break;
			}
		pt++;
	}
    DPRINT("[CMC623:INFO]:%s:%s\n",__func__,tuning_filename);

    if (load_tuning_data(tuning_filename) <= 0) {
        DPRINT("[CMC623:ERROR]:load_tunig_data() failed\n");
        return size;
    }
    tuning_enable = 1;
    return size;
}

static DEVICE_ATTR(tuning, 0664, tuning_show, tuning_store);

#endif

void free_cmap(struct fb_cmap *cmap)
{
	if (cmap->red)
		kfree(cmap->red);
	if (cmap->green)
		kfree(cmap->green);
	if (cmap->blue)
		kfree(cmap->blue);
}

void lut_tune(int num, unsigned int *pLutTable )
{
			printk(KERN_ERR"[mDNIE WARNING] LUT TUNE PASSATO\n");
//	int fb;
	struct fb_info *info;
	struct fb_cmap test_cmap;
	struct fb_cmap *cmap;

	static int mdp_lut_i = 0;
	u16 r_1, g_1, b_1;//for final assignment
//	fb = open("/dev/graphics/fb0", O_RDWR);
	__u16 *r, *g, *b, i;
	int j = 0;
	
	info = registered_fb[0];
	cmap = &test_cmap;
	//=====================================
	// cmap allocation 
	//=====================================
	cmap->red = 0;
	cmap->green = 0;
	cmap->blue = 0;
	cmap->transp = 0;
	cmap->start = 0;
	cmap->len = num; // MAX_LUT_SIZE;//LUT has 256 entries
	cmap->red = kmalloc(cmap->len * sizeof(__u16), GFP_KERNEL);
	if (!cmap->red) {
		printk(KERN_ERR "can't malloc cmap!");
		goto fail_rest;
	}

	cmap->green = kmalloc(cmap->len * sizeof(__u16), GFP_KERNEL);
	if (!cmap->green) {
		printk(KERN_ERR "can't malloc cmap!");
		goto fail_rest;
	}

	cmap->blue = kmalloc(cmap->len * sizeof(__u16), GFP_KERNEL);
	if (!cmap->blue) {
		printk(KERN_ERR "can't malloc cmap!");
		goto fail_rest;
	}
	r = cmap->red;
	g = cmap->green;
	b = cmap->blue;

	j = 0;
	DPRINT("cmap->len %d\n",cmap->len);
	/* Assigning the cmap*/
	for (i = 0; i < cmap->len; i++) {
		*r++ = pLutTable[j++];
		*g++ = pLutTable[j++];
		*b++ = pLutTable[j++];
	}
#if 1
	/*instead of an ioctl*/
	mutex_lock(&msm_fb_ioctl_lut_sem1);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	//ret = mdp_lut_hw_update(cmap);
	j = 0;
	for (i = 0; i < cmap->len; i++) {
//		r_1 = mDNIe_data_R[i];
//		g_1 = mDNIe_data_G[i];
//		b_1 = mDNIe_data_B[i];
		r_1 = pLutTable[j++]; 
		g_1 = pLutTable[j++];
		b_1 = pLutTable[j++];
		

#ifdef CONFIG_FB_MSM_MDP40
		MDP_OUTP(MDP_BASE + 0x94800 +
#else
		MDP_OUTP(MDP_BASE + 0x93800 +
#endif
			(0x400*mdp_lut_i) + cmap->start*4 + i*4,
				((g_1 & 0xff) |
				 ((b_1 & 0xff) << 8) |
				 ((r_1 & 0xff) << 16)));
	}
	MDP_OUTP(MDP_BASE + 0x90070, (mdp_lut_i << 10) | 0x17);
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	mdp_lut_i = (mdp_lut_i + 1)%2;
	mutex_unlock(&msm_fb_ioctl_lut_sem1);
#else	
	info->fbops->fb_open && info->fbops->fb_ioctl(info, MSMFB_SET_LUT, cmap);
#endif	

 fail_rest:
	free_cmap(cmap);
//	close(fb);
}

void sharpness_tune(int num )
{
	char *vg_base;
	vg_base = MDP_BASE + MDP4_VIDEO_BASE;
	outpdw(vg_base + 0x8200, mdp4_ss_table_value((int8_t)num, 0)); 
	outpdw(vg_base + 0x8204, mdp4_ss_table_value((int8_t)num, 1)); 
}


////////////////[
int s3c_mdnie_start(void)
{
	g_mdine_enable = 1;
	return 0;
}

int s3c_mdnie_off(void)
{
	g_mdine_enable = 0;
	return 0;
}

void mDNIe_Set_Mode(Lcd_mDNIe_UI mode)
{
	unsigned int *pLut = NULL;
	int	sharpvalue = 0;
	static int isSetDMBMode = 0;

	DPRINT("[mdnie set] mDNIe_Set_Mode \n");
	if(!g_mdine_enable) {
		printk(KERN_ERR"[mDNIE WARNING] mDNIE engine is OFF. So you cannot set mDnie Mode correctly.\n");
		return;  // return 0;
	}
		switch (mode) {
		case mDNIe_UI_MODE:
#if 1 // QSEED Check save			
			if(isSetDMBMode==1)
			{
				mdp4_vg_qseed_init_VideoPlay(0);
//				mdp4_vg_qseed_init_VideoPlay(1);
				isSetDMBMode = 0;
			}
#endif			
			pLut = UI_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;

		case mDNIe_VIDEO_MODE:
		case mDNIe_VIDEO_WARM_MODE:
		case mDNIe_VIDEO_COLD_MODE:
#if 1 // QSEED Check save			
			if(isSetDMBMode==1)
			{
				mdp4_vg_qseed_init_VideoPlay(0);
//				mdp4_vg_qseed_init_VideoPlay(1);
				isSetDMBMode = 0;
			}
#endif			
			pLut = VIDEO_LUT;
			sharpvalue = SHARPNESS_VIDEO;		
			break;

		case mDNIe_CAMERA_MODE:
			pLut = CAMERA_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;

		case mDNIe_NAVI:
			pLut = NAVI_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;
			
		case mDNIe_GALLERY:
			pLut = GALLERY_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;
		case mDNIe_BYPASS: 
			pLut = BYPASS_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;

#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
		case mDNIe_DMB_MODE:				// warm, clod not distinguish
		case mDNIe_DMB_WARM_MODE:
		case mDNIe_DMB_COLD_MODE:
#if 1  // QSEED Check save			
			if(isSetDMBMode==0)
			{
				mdp4_vg_qseed_init_DMB(0);
//				mdp4_vg_qseed_init_DMB(1);
				isSetDMBMode = 1;
			}
#endif			
			pLut = DMB_LUT;
			sharpvalue = SHARPNESS_DMB;
			break;
#endif 
#ifdef CONFIG_TARGET_LOCALE_NTT
		case mDNIe_ISDBT_MODE:
		case mDNIe_ISDBT_WARM_MODE:
		case mDNIe_ISDBT_COLD_MODE:
			pLut = BYPASS_LUT;
			sharpvalue = SHARPNESS_BYPASS;	
//			mDNIe_txtbuf_to_parsing(ISDBT_COLD_MODE_FILE);
			break;
#endif
#ifdef BROWSER_COLOR_TONE_SET
		case mDNIe_BROWSER_TONE1:
			pLut = BROWSER_TONE1_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;
		case mDNIe_BROWSER_TONE2:
			pLut = BROWSER_TONE2_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;
		case mDNIe_BROWSER_TONE3:
			pLut = BROWSER_TONE3_LUT;
			sharpvalue = SHARPNESS_BYPASS;
			break;			
#endif			
		}

		lut_tune(MAX_LUT_SIZE, pLut);
		sharpness_tune(sharpvalue);

		current_mDNIe_Mode = mode;
//		current_mDNIe_OutDoor_OnOff = FALSE;
//	}

#ifdef CONFIG_FB_S3C_MDNIE_TUNINGMODE_FOR_BACKLIGHT
	pre_val = -1;
#endif	/* CONFIG_FB_S3C_MDNIE_TUNINGMODE_FOR_BACKLIGHT */
	DPRINT("[mDNIe] mDNIe_Set_Mode : Current_mDNIe_mode (%d)  \n", current_mDNIe_Mode);  
}

void mDNIe_User_Select_Mode(Lcd_mDNIe_User_Set mode)
{
/*
	if(!g_mdine_enable) {
		printk(KERN_ERR"[mDNIE WARNING] mDNIE engine is OFF. So you cannot set mDnie Mode correctly.\n");
		return 0;
	}
	switch (mode) {
	case mDNIe_DYNAMIC:  
		mDNIe_txtbuf_to_parsing(DYNAMIC_MODE_FILE);
		break;

	case mDNIe_STANDARD:  
		mDNIe_txtbuf_to_parsing(STANDARD_MODE_FILE);
		break;

	case mDNIe_MOVIE:  
		mDNIe_txtbuf_to_parsing(MOVIE_MODE_FILE);
		break;
	}
	current_mDNIe_user_mode = mode;
	printk(KERN_ERR "[mDNIe] mDNIe_user_select_Mode : User_mDNIe_Setting_Mode (%d), Current_mDNIe_mode(%d) \n", current_mDNIe_user_mode,current_mDNIe_Mode);  
*/
}


static ssize_t scenario_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int mdnie_ui = 0;

	DPRINT("called %s \n", __func__);

	switch (current_mDNIe_Mode) {
	case mDNIe_UI_MODE:
	default:
		mdnie_ui = 0;
		break;

	case mDNIe_VIDEO_MODE:
		mdnie_ui = 1;
		break;

	case mDNIe_VIDEO_WARM_MODE:
		mdnie_ui = 2;
		break;

	case mDNIe_VIDEO_COLD_MODE:
		mdnie_ui = 3;
		break;

	case mDNIe_CAMERA_MODE:
		mdnie_ui = 4;
		break;

	case mDNIe_NAVI:
		mdnie_ui = 5;
		break;

	case mDNIe_GALLERY:
		mdnie_ui = 6;
		break;

	case mDNIe_BYPASS:  
		mdnie_ui = 7;
		break;
		
#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
	case mDNIe_DMB_MODE:
		mdnie_ui = mDNIe_DMB_MODE;
		break;

	case mDNIe_DMB_WARM_MODE:
		mdnie_ui = mDNIe_DMB_WARM_MODE;
		break;

	case mDNIe_DMB_COLD_MODE:
		mdnie_ui = mDNIe_DMB_COLD_MODE;
		break;
#endif
#ifdef CONFIG_TARGET_LOCALE_NTT
	case mDNIe_ISDBT_MODE:
		mdnie_ui = mDNIe_ISDBT_MODE;
		break;

	case mDNIe_ISDBT_WARM_MODE:
		mdnie_ui = mDNIe_ISDBT_WARM_MODE;
		break;

	case mDNIe_ISDBT_COLD_MODE:
		mdnie_ui = mDNIe_ISDBT_COLD_MODE;
		break;
#endif
#ifdef BROWSER_COLOR_TONE_SET
	case mDNIe_BROWSER_TONE1:
		mdnie_ui = mDNIe_BROWSER_TONE1;
		break;
	case mDNIe_BROWSER_TONE2:
		mdnie_ui = mDNIe_BROWSER_TONE2;
		break;
	case mDNIe_BROWSER_TONE3:
		mdnie_ui = mDNIe_BROWSER_TONE3;
		break;
#endif		
	}
	return sprintf(buf, "%u\n", mdnie_ui);
}

static ssize_t scenario_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	DPRINT("[mdnie set] in scenario_store, input value = %d \n",value);

	switch (value) {
	case SIG_MDNIE_UI_MODE:
		current_mDNIe_Mode = mDNIe_UI_MODE;
		break;

	case SIG_MDNIE_VIDEO_MODE:
		current_mDNIe_Mode = mDNIe_VIDEO_MODE;
		break;

	case SIG_MDNIE_VIDEO_WARM_MODE:
		current_mDNIe_Mode = mDNIe_VIDEO_WARM_MODE;
		break;

	case SIG_MDNIE_VIDEO_COLD_MODE:
		current_mDNIe_Mode = mDNIe_VIDEO_COLD_MODE;
		break;

	case SIG_MDNIE_CAMERA_MODE:
		current_mDNIe_Mode = mDNIe_CAMERA_MODE;
		break;

	case SIG_MDNIE_NAVI:
		current_mDNIe_Mode = mDNIe_NAVI;
		break;

	case SIG_MDNIE_GALLERY:
		current_mDNIe_Mode = mDNIe_GALLERY;
		break;

	case SIG_MDNIE_BYPASS:  
		current_mDNIe_Mode = mDNIe_BYPASS;
		break;

#if defined(CONFIG_TDMB) || defined(CONFIG_TDMB_MODULE)
	case SIG_MDNIE_DMB_MODE:
		current_mDNIe_Mode = mDNIe_DMB_MODE;
		break;

	case SIG_MDNIE_DMB_WARM_MODE:
		current_mDNIe_Mode = mDNIe_DMB_WARM_MODE;
		break;

	case SIG_MDNIE_DMB_COLD_MODE:
		current_mDNIe_Mode = mDNIe_DMB_COLD_MODE;
		break;
#endif 
#ifdef CONFIG_TARGET_LOCALE_NTT
	case SIG_MDNIE_ISDBT_MODE:
		current_mDNIe_Mode = mDNIe_ISDBT_MODE;
		break;

	case SIG_MDNIE_ISDBT_WARM_MODE:
		current_mDNIe_Mode = mDNIe_ISDBT_WARM_MODE;
		break;

	case SIG_MDNIE_ISDBT_COLD_MODE:
		current_mDNIe_Mode = mDNIe_ISDBT_COLD_MODE;
		break;
#endif
#ifdef BROWSER_COLOR_TONE_SET
	case SIG_MDNIE_BROWSER_TONE1:
		current_mDNIe_Mode = mDNIe_BROWSER_TONE1;
		break;
	case SIG_MDNIE_BROWSER_TONE2:
		current_mDNIe_Mode = mDNIe_BROWSER_TONE2;
		break;
	case SIG_MDNIE_BROWSER_TONE3:
		current_mDNIe_Mode = mDNIe_BROWSER_TONE3;
		break;		
#endif

	default:
		printk(KERN_ERR "\nscenario_store value is wrong : value(%d)\n", value);
		break;
	}

	mDNIe_Set_Mode(current_mDNIe_Mode);

	return size;
}

static DEVICE_ATTR(scenario, 0664, scenario_show, scenario_store);

static ssize_t mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int mdnie_ui = 0;

	DPRINT("called %s \n", __func__);

	return sprintf(buf, "%u\n", mdnie_ui);
	
}

static ssize_t mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	DPRINT("[mdnie set] in mode_store, input value = %d \n",value);

	return size;
}

static DEVICE_ATTR(mode, 0664, mode_show, mode_store);


static ssize_t mdnieset_init_file_cmd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char temp[15];
	
	DPRINT("called %s \n", __func__);
	sprintf(temp, "mdnieset_init_file_cmd_show \n");
	strcat(buf, temp);
	
	return strlen(buf);
}

static ssize_t mdnieset_init_file_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	DPRINT("mdnieset_init_file_cmd_store  : value(%d)\n", value);	
	switch (value) {
	case 0:  
		current_mDNIe_Mode =mDNIe_UI_MODE;
		break;
		
	default:
		printk(KERN_ERR "mdnieset_init_file_cmd_store value is wrong : value(%d)\n", value);
		break;
	}
	mDNIe_Set_Mode(current_mDNIe_Mode);

	return size;
}

static DEVICE_ATTR(mdnieset_init_file_cmd, 0664, mdnieset_init_file_cmd_show, mdnieset_init_file_cmd_store);


static ssize_t outdoor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	DPRINT("called %s \n", __func__);

	return sprintf(buf, "0\n" );
}

static ssize_t outdoor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);


	DPRINT("[mdnie set] in outdoor_store, input value = %d \n",value);


//	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);

	return size;
}

static DEVICE_ATTR(outdoor, 0664, outdoor_show, outdoor_store);
////////////////]


void init_mdnie_class(void)
{
	mdnie_class = class_create(THIS_MODULE, "mdnie");
	if (IS_ERR(mdnie_class))
		pr_err("Failed to create class(mdnie_class)!\n");

	mdnie_dev = device_create(mdnie_class, NULL, 0, NULL, "mdnie");
	if (IS_ERR(mdnie_dev))
		pr_err("Failed to create device(switch_mdnieset_ui_dev)!\n");

	if (device_create_file(mdnie_dev, &dev_attr_scenario) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_scenario.attr.name);

	if (device_create_file(mdnie_dev, &dev_attr_mode) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_mode.attr.name);

	if (device_create_file(mdnie_dev, &dev_attr_mdnieset_init_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_mdnieset_init_file_cmd.attr.name);

//	mdnieset_outdoor_class = class_create(THIS_MODULE, "mdnieset_outdoor");
//	if (IS_ERR(mdnieset_outdoor_class))
//		pr_err("Failed to create class(mdnieset_outdoor_class)!\n");

//	switch_mdnieset_outdoor_dev = device_create(mdnieset_outdoor_class, NULL, 0, NULL, "switch_mdnieset_outdoor");
//	if (IS_ERR(switch_mdnieset_outdoor_dev))
//		pr_err("Failed to create device(switch_mdnieset_outdoor_dev)!\n");

	if (device_create_file(mdnie_dev, &dev_attr_outdoor) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_outdoor.attr.name);
	
#ifdef MDP4_VIDEO_ENHANCE_TUNING    
    if (device_create_file(mdnie_dev, &dev_attr_tuning) < 0) {
        pr_err("Failed to create device file(%s)!\n",dev_attr_tuning.attr.name);
    }
#endif	
	s3c_mdnie_start();
	sharpness_tune(0);	
#ifdef CONFIG_FB_MSM_MIPI_S6E8AA0_WXGA_Q1_PANEL
	lut_tune(MAX_LUT_SIZE, UI_LUT);
#endif
}


