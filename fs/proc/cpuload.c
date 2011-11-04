/*
 * fs/proc/cpuload.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *      Global GSM S/W 'jaecheol kim'
 *      email : jc22.kim@samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define CPU_LOAD_LOG_BUF_LEN    (0x10000)
#define CPU_LOAD_LOG_BUF_MASK   (CPU_LOAD_LOG_BUF_LEN-1)

DECLARE_WAIT_QUEUE_HEAD(cpuload_wait);

static u32 log_buf_len = CPU_LOAD_LOG_BUF_LEN;
static u8 *cpuload_buf;
static u32 log_start = 0;
static u32 log_end = 0;
static struct timeval cpuload_systime;

/* default cpuload not permitted! */
static int log_permitted = 0;
static int in_use = 0;

static int load_old = 0;
static int targetclk_old = 0;
static int newclk_old = 0;

#define CPU_LOG_BUF(idx)    (cpuload_buf[(idx) & CPU_LOAD_LOG_BUF_MASK])

static void cpuload_emit_log_char(char c)
{
    CPU_LOG_BUF(log_end) = c;
    log_end++;
    if (unlikely(log_end - log_start > log_buf_len))
        log_start = log_end - log_buf_len;
}

int cpuload_log(int load, int targetclk, int newclk,
        const char *buf, size_t count)
{
    int i;
    u32 len;
    u8 tmp_buf[1024];

    if (likely(!log_permitted))
        return 0;

    do_gettimeofday(&cpuload_systime);

    len = sprintf(tmp_buf, "%ld.%06ld", cpuload_systime.tv_sec, cpuload_systime.tv_usec);

    /* update load */
    if (load >= 0) {
        len += sprintf(tmp_buf + len, "\t%d", load);
        load_old = load;
        /* code */
    } else
        len += sprintf(tmp_buf + len, "\t0%d", load_old);

    /* update target clk */
    if (targetclk >= 0) {
        len += sprintf(tmp_buf + len, "\t%d", targetclk);
        targetclk_old = targetclk;
    } else
        len += sprintf(tmp_buf + len, "\t0%d", targetclk_old);

    /* update new clk */
    if (newclk >= 0) {
        len += sprintf(tmp_buf + len, "\t%d", newclk);
        newclk_old = newclk;
    } else
        len += sprintf(tmp_buf + len, "\t0%d", newclk_old);

	for (i=0; i < len; i++)
		cpuload_emit_log_char(tmp_buf[i]);

	if (unlikely(buf)) {
		cpuload_emit_log_char('\t');
		for (i=0; i < count ; i++)
			cpuload_emit_log_char(buf[i]);
	}

	if (CPU_LOG_BUF(log_end-1) != '\n')
		cpuload_emit_log_char('\n');

    wake_up_interruptible(&cpuload_wait);

    return 0;
}
EXPORT_SYMBOL(cpuload_log);

static int cpuload_open(struct inode * inode, struct file * file)
{
    if (!in_use) {
        cpuload_buf = (char *)kmalloc(log_buf_len, GFP_KERNEL);
        if (!cpuload_buf) {
            printk("%s: kmalloc failed\n", __func__);
            return -ENOMEM;
        }
    }

    in_use++;

    if (in_use <= 0) {
        printk("%s: open count error! (%d)\n", __func__, in_use);
        log_permitted = 0;
        return -EAGAIN;
    }

    log_permitted = 1;

    /* get current clock */
    newclk_old = targetclk_old = cpufreq_quick_get(0);

    return 0;
}

static int cpuload_release(struct inode * inode, struct file * file)
{
    in_use--;

    if (in_use <= 0) {
        in_use = 0;
        log_permitted = 0;
        kfree(cpuload_buf);
    }

    return 0;
}

static ssize_t cpuload_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
    int len;
    int error;
    int ret;

    if ((file->f_flags & O_NONBLOCK) && (log_start == log_end))
        return -EAGAIN;

    if (!access_ok(VERIFY_WRITE, buf, count))
        return -EFAULT;

    error = wait_event_interruptible(cpuload_wait, (log_start - log_end));

    if (error)
        return error;

    len = log_end - log_start;

    if (count < len)
        len = count;

    ret = len;

    while (len) {
        __put_user(CPU_LOG_BUF(log_start), buf);
        log_start++;
        buf++;
        len--;
    }

    return ret;
}

static ssize_t cpuload_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
    cpuload_log(-1, -1, -1, buf, count);

    wake_up_interruptible(&cpuload_wait);

    return count;
}

static unsigned int cpuload_poll(struct file *file, poll_table *wait)
{
    poll_wait(file, &cpuload_wait, wait);

    if (log_start != log_end)
        return POLLIN | POLLRDNORM;

    return 0;
}

static const struct file_operations proc_cpuload_operations = {
    .read       = cpuload_read,
    .write      = cpuload_write,
    .poll       = cpuload_poll,
    .open       = cpuload_open,
    .release    = cpuload_release,
};

static int __init proc_cpuload_init(void)
{
    proc_create("cpuload", S_IRUSR|S_IWUSR, NULL, &proc_cpuload_operations);
    return 0;
}
module_init(proc_cpuload_init);
