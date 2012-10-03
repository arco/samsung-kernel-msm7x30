/* drivers/video/msm/logo.c
 *
 * Show Logo in RLE 565 format
 *
 * Copyright (C) 2008 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 2)

static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}

/* convert RGB565 to RBG8888 */
static void memset16_rgb8888(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	unsigned short red;
	unsigned short green;
	unsigned short blue;

	red = ( val & 0xF800) >> 8;
	green = (val & 0x7E0) >> 3;
	blue = (val & 0x1F) << 3;

	count >>= 1;
	while (count--)
	{
		*ptr++ = (red<<8) | green;
		*ptr++ = blue;
	}
}

/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
#ifdef CONFIG_FB_MSM_SEC_BOOTLOGO
int load_565rle_image_onfb( char *filename, int start_x, int start_y, bool bf_supported)
{
	int fd, err = 0;
	unsigned count, max;
	unsigned short *data, *bits, *ptr;
	struct fb_info *info;
	unsigned short *p_line;
	unsigned short sp, c_pixel;
	unsigned draw_pixel;
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
	struct module *owner;
#endif
	info = registered_fb[0];

	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
	owner = info->fbops->owner;
	if (!try_module_get(owner))
		return NULL;
	if (info->fbops->fb_open && info->fbops->fb_open(info, 0)) {
		module_put(owner);
		return NULL;
	}
#endif
	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}

	max = fb_width(info) * fb_height(info);
	ptr = data;
        if (bf_supported && (info->node == 1 || info->node == 2)) {
                err = -EPERM;
                pr_err("%s:%d no info->creen_base on fb%d!\n",
                       __func__, __LINE__, info->node);
                goto err_logo_free_data;
        }
	bits = (unsigned short *)(info->screen_base+(info->fix.line_length*start_y) );

	p_line = bits;
	sp = start_x; // start pixel in Line
	while (count > 3) {
		unsigned n = ptr[0];  // pixel count

		if (n > max)
			break;
		draw_pixel = n;
		while(draw_pixel > 0)
		{
			if(sp == 480)
			{
				// move next-line
				p_line += (info->fix.line_length / 2);
				sp = 0;
			}

			if( (sp + draw_pixel) >= 480)
				c_pixel = 480 - sp;
			else
				c_pixel = draw_pixel;

			memset16_rgb8888(&p_line[sp*2], ptr[1], c_pixel<<1);

			draw_pixel -= c_pixel; // decrease drawed-pixel-count
			sp += c_pixel;
		}

		max -= n;
		ptr += 2;
		count -= 4;
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565rle_image_onfb);
#endif

/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
int load_565rle_image(char *filename, bool bf_supported)
{
	struct fb_info *info;
	int fd, count, err = 0;
	unsigned max;
	unsigned short *data, *bits, *ptr;

	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if (sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}

	max = fb_width(info) * fb_height(info);
	ptr = data;
	if (bf_supported && (info->node == 1 || info->node == 2)) {
		err = -EPERM;
		pr_err("%s:%d no info->creen_base on fb%d!\n",
		       __func__, __LINE__, info->node);
		goto err_logo_free_data;
	}
	if (info->screen_base) {
		bits = (unsigned short *)(info->screen_base);
		while (count > 3) {
			unsigned n = ptr[0];
			if (n > max)
				break;
			memset16(bits, ptr[1], n << 1);
			bits += n;
			max -= n;
			ptr += 2;
			count -= 4;
		}
	}

err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565rle_image);
