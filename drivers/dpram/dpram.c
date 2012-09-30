/****************************************************************************

**

** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2010 ALL RIGHTS RESERVED

**

****************************************************************************/
#define	NO_TTY_DPRAM	1
#define	NO_TTY_TX_RETRY	1

#define _ENABLE_ERROR_DEVICE

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mm.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/irq.h>

#ifdef _ENABLE_ERROR_DEVICE
#include <linux/poll.h>
#include <linux/cdev.h>
#endif	/* _ENABLE_ERROR_DEVICE */

#include <asm/irq.h>
#include <asm/io.h>

#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>

#include "dpram.h"
#include "../../arch/arm/mach-msm/smd_private.h"
#include "../../arch/arm/mach-msm/proc_comm.h"

#define DRIVER_NAME 		"DPRAM"
#define DRIVER_MAJOR_NUM	255

#undef _DEBUG
#ifdef _DEBUG
#define dprintk(s, args...) printk("[DPRAM] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif	/* _DEBUG */

#define WRITE_TO_DPRAM(dest, src, size) \
	_memcpy((void *)(SmemBase + dest), src, size)

#define READ_FROM_DPRAM(dest, src, size) \
	_memcpy(dest, (void *)(SmemBase + src), size)

#ifdef _ENABLE_ERROR_DEVICE
#define DPRAM_ERR_MSG_LEN			65
#define DPRAM_ERR_DEVICE			"dpramerr"
#endif	/* _ENABLE_ERROR_DEVICE */

//#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)
#define MSM_TRIG_A2M_DPRAM_INT     (writel(1 << 4, MSM_GCC_BASE + 0x8))

static volatile unsigned char *SmemBase;
static int DpramInited = 0;

static struct tty_driver *dpram_tty_driver;
static dpram_tasklet_data_t dpram_tasklet_data[MAX_INDEX];
static dpram_device_t dpram_table[MAX_INDEX] = {
	{
		.in_head_addr = DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_FORMATTED_SIZE,

		.out_head_addr = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_FORMATTED_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_FORMATTED_SIZE,

		.mask_req_ack = INT_MASK_REQ_ACK_F,
		.mask_res_ack = INT_MASK_RES_ACK_F,
		.mask_send = INT_MASK_SEND_F,
	},
	{
		.in_head_addr = DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_RAW_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_RAW_SIZE,

		.out_head_addr = DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_RAW_SIZE,

		.mask_req_ack = INT_MASK_REQ_ACK_R,
		.mask_res_ack = INT_MASK_RES_ACK_R,
		.mask_send = INT_MASK_SEND_R,
	},
};

static struct tty_struct *dpram_tty[MAX_INDEX];
static struct ktermios *dpram_termios[MAX_INDEX];
static struct ktermios *dpram_termios_locked[MAX_INDEX];

extern void *smem_alloc(unsigned, unsigned);

//Get charging status & charger Connect value!!!
//extern void get_charger_type(void);
//extern void msm_batt_check_event(void);
//extern int get_charging_status(void);

static void print_smem(void);
static void dpram_ramdump(void);

static void res_ack_tasklet_handler(unsigned long data);
static void send_tasklet_handler(unsigned long data);

static DECLARE_TASKLET(fmt_send_tasklet, send_tasklet_handler, 0);
static DECLARE_TASKLET(raw_send_tasklet, send_tasklet_handler, 0);

static DECLARE_TASKLET(fmt_res_ack_tasklet, res_ack_tasklet_handler,
		(unsigned long)&dpram_table[FORMATTED_INDEX]);
static DECLARE_TASKLET(raw_res_ack_tasklet, res_ack_tasklet_handler,
		(unsigned long)&dpram_table[RAW_INDEX]);

#ifdef _ENABLE_ERROR_DEVICE
static unsigned int dpram_err_len;
static char dpram_err_buf[DPRAM_ERR_MSG_LEN];

struct class *dpram_class;

static DECLARE_WAIT_QUEUE_HEAD(dpram_err_wait_q);
static struct fasync_struct *dpram_err_async_q;
#endif	/* _ENABLE_ERROR_DEVICE */

static DECLARE_MUTEX(write_mutex);
struct wake_lock imei_wake_lock;
struct wake_lock dpram_wake_lock;
struct wake_lock silent_wake_lock;

extern struct class *sec_class;

/* tty related functions. */
static inline void byte_align(unsigned long dest, unsigned long src)
{
	u16 *p_src;
	volatile u16 *p_dest;

	if (!(dest % 2) && !(src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0xFF00) | (*p_src & 0x00FF);
	}

	else if ((dest % 2) && (src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0x00FF) | (*p_src & 0xFF00);
	}

	else if (!(dest % 2) && (src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0xFF00) | ((*p_src >> 8) & 0x00FF);
	}

	else if ((dest % 2) && !(src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0x00FF) | ((*p_src << 8) & 0xFF00);
	}

	else {
		dprintk("oops.~\n");
	}
}

static inline void _memcpy(void *p_dest, const void *p_src, int size)
{
	unsigned long dest = (unsigned long)p_dest;
	unsigned long src = (unsigned long)p_src;

	if (size <= 0) {
		return;
	}

	if (dest & 1) {
		byte_align(dest, src);
		dest++, src++;
		size--;
	}

	if (size & 1) {
		byte_align(dest + size - 1, src + size - 1);
		size--;
	}

	if (src & 1) {
		unsigned char *s = (unsigned char *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) {
			*d++ = s[0] | (s[1] << 8);
			s += 2;
		}
	}

	else {
		u16 *s = (u16 *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) { *d++ = *s++; }
	}
}

static inline int _memcmp(u8 *dest, u8 *src, int size)
{
	int i = 0;

	while (i++ < size) {
		if (*(dest + i) != *(src + i)) {
			return 1;
		}
	}

	return 0;
}

static inline int WRITE_TO_DPRAM_VERIFY(u32 dest, void *src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)(SmemBase + dest), (void *)src, size);

		if (!_memcmp((u8 *)(SmemBase + dest), (u8 *)src, size))
			return 0;
	}

	return -1;
}

static inline int READ_FROM_DPRAM_VERIFY(void *dest, u32 src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)dest, (void *)(SmemBase + src), size);

		if (!_memcmp((u8 *)dest, (u8 *)(SmemBase + src), size))
			return 0;
	}

	return -1;
}

static void send_interrupt_to_phone(u16 irq_mask)
{
	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, &irq_mask,
			DPRAM_INTERRUPT_PORT_SIZE);
	//writel(1, MSM_A2M_INT(4));
	MSM_TRIG_A2M_DPRAM_INT;
	dprintk("PDA -> Phone interrupt!\n");
}

#ifdef	NO_TTY_DPRAM

#define yisprint(c)	((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
void yhexdump(const char *buf, int len)
{
	char str[80], octet[10];
	int ofs, i, l;

//	printk("<yhexdump()> : ADDR - [0x%08x], len -[%d]\n", buf, len);
	for (ofs = 0; ofs < len; ofs += 16) {
		sprintf( str, "%03d: ", ofs );

		for (i = 0; i < 16; i++) {
			if ((i + ofs) < len)
				sprintf( octet, "%02x ", buf[ofs + i] );
			else
				strcpy( octet, "   " );

			strcat( str, octet );
		}
			strcat( str, "  " );
			l = strlen( str );

		for (i = 0; (i < 16) && ((i + ofs) < len); i++)
			str[l++] = yisprint( buf[ofs + i] ) ? buf[ofs + i] : '.';

		str[l] = '\0';
		printk( "%s\n", str );
	}
}

EXPORT_SYMBOL(yhexdump);

char	multipdp_rbuf[128 * 1024];
static int dpram_write(dpram_device_t *device, const unsigned char *buf, int len);
static int  (*multipdp_rx_noti_func)(char *, int);
static inline int dpram_tty_insert_data(dpram_device_t *device, const u8 *psrc, u16 size);

int  multipdp_buf_copy(int index, char *dpram, int size)
{
//	int i;

	if( index < 0 || index > sizeof(multipdp_rbuf) || (index + size) > sizeof(multipdp_rbuf))
		return -1;

	dprintk("multipdp_buf_copy:index=%d size=%d\n", index, size);
	memcpy( (void *)&multipdp_rbuf[index], (void *)dpram, size);
	return( size);

}
EXPORT_SYMBOL(multipdp_buf_copy);

int	multipdp_rx_noti_regi( int (*rx_cfunc)(char *, int))
{
	multipdp_rx_noti_func =  rx_cfunc;
	return 0;
}
EXPORT_SYMBOL(multipdp_rx_noti_regi);

int multipdp_rx_datalen;
int	multipdp_rx_data(dpram_device_t *device, int len)
{
	static int inuse_flag = 0;
	
	int ret = 0;	
	
	if( len == 0 )
		return 0;
		
	if( inuse_flag )
		printk("***** inuse_flag = %d\n", inuse_flag);	
		
	inuse_flag ++;
	
	//yhexdump(multipdp_rbuf, len);	
	//multipdp_rbuf
	if( multipdp_rx_noti_func)
	{
		dprintk("multipdp_rx_data Before(noti_func) : len=%d\n",len);
		multipdp_rx_datalen = len;

		ret = multipdp_rx_noti_func(multipdp_rbuf, len);
		//memset(multipdp_rbuf, 0x00, len);
		dprintk("multipdp_rx_data After(noti_func) : ret=%d\n",ret);
	}

	inuse_flag --;
	
	return(ret);
}

int	multipdp_dump(void)
{
	yhexdump(multipdp_rbuf, multipdp_rx_datalen);
	return 0;
}
EXPORT_SYMBOL(multipdp_dump);

int multipdp_write(const unsigned char *buf, int len)
{
	int i, ret;
	// FORMATTED_INDEX : dpram0, RAW_INDEX : dpram1
	dpram_device_t *device = &dpram_table[RAW_INDEX];

#ifdef	NO_TTY_TX_RETRY
	for(i =0; i<10; i++)
	{
		ret = dpram_write(device, buf, len);
		if( ret > 0 )
		{
			break;
		}
		printk(KERN_DEBUG "dpram_write() failed: %d, i(%d)\n", ret, i);
	}
	if ( i>=10) {
		printk(KERN_DEBUG "dpram_write() failed: %d\n", ret);
	}
	
	return ret;
#endif
}
EXPORT_SYMBOL(multipdp_write);
#endif

//struct class *sec_class;
struct device *dpram_dev;

#define POWER_DOWN_TIME ( 30 * HZ )
struct device *pm_dev;
void power_down_registertimer(struct timer_list* ptimer, unsigned long timeover );
void power_down_timeout(unsigned long arg);
struct timer_list power_down_timer;
bool power_down;

static ssize_t show_info(struct device *d,
		struct device_attribute *attr, char *buf)
{

	char *p = buf;

	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;
	u16 in_interrupt = 0, out_interrupt = 0;

#ifdef _ENABLE_ERROR_DEVICE
	char err_buf[DPRAM_ERR_MSG_LEN];
#endif	/* _ENABLE_ERROR_DEVICE */
	
	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));

	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, 
			sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, 
		    sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, 
		    sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, 
		    sizeof(fmt_out_tail));

	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, 
		    sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, 
		    sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, 
		    sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, 
		    sizeof(raw_out_tail));

	READ_FROM_DPRAM((void *)&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, 
		    DPRAM_INTERRUPT_PORT_SIZE);
	READ_FROM_DPRAM((void *)&out_interrupt, DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, 
		    DPRAM_INTERRUPT_PORT_SIZE);


#ifdef _ENABLE_ERROR_DEVICE
	memset((void *)err_buf, '\0', DPRAM_ERR_MSG_LEN);
	memcpy(err_buf, dpram_err_buf, DPRAM_ERR_MSG_LEN - 1);
#endif	/* _ENABLE_ERROR_DEVICE */

	p += sprintf(p,
			"-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"| MAGIC CODE\t\t| 0x%04x\n"
			"| ENABLE CODE\t\t| 0x%04x\n"
			"| PHONE->PDA FMT HEAD\t| %u\n"
			"| PHONE->PDA FMT TAIL\t| %u\n"
			"| PDA->PHONE FMT HEAD\t| %u\n"
			"| PDA->PHONE FMT TAIL\t| %u\n"
			"| PHONE->PDA RAW HEAD\t| %u\n"
			"| PHONE->PDA RAW TAIL\t| %u\n"
			"| PDA->PHONE RAW HEAD\t| %u\n"
			"| PDA->PHONE RAW TAIL\t| %u\n"
			"| PHONE->PDA INT.\t| 0x%04x\n"
			"| PDA->PHONE INT.\t| 0x%04x\n"
#ifdef _ENABLE_ERROR_DEVICE
			"| LAST PHONE ERR MSG\t| %s\n"
#endif	/* _ENABLE_ERROR_DEVICE */
			"-------------------------------------\n",
			magic, enable,
			fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail,
			raw_in_head, raw_in_tail, raw_out_head, raw_out_tail,
			in_interrupt, out_interrupt,

#ifdef _ENABLE_ERROR_DEVICE
			(buf[0] != '\0' ? buf : "NONE")
#endif	/* _ENABLE_ERROR_DEVICE */
		);

	return p - buf;
}
static DEVICE_ATTR(info, S_IRUGO|S_IWUSR, show_info, NULL);

#ifdef CONFIG_DPRAM_WHITELIST
static ssize_t store_whitelist(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int i;

	switch (buf[0]) {
	case 0x7F:
#if 1	//1
		printk("SEND WHITELIST USING DPARM FMT COMMAND\n");
		for(i=0; i<count; i++)
			printk("%02x", buf[i]);
		printk("\n");
#endif
		dpram_write(&dpram_table[0], buf, count);
		break;
	default:

		break;
	}

	return count;
}

static DEVICE_ATTR(whitelist, S_IRUGO|S_IWUSR, NULL, store_whitelist);
#endif


static ssize_t store_power_down(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);

	if (value == 1)
	{
		printk("[dpram] %s(%d)\n", __func__, __LINE__);
		power_down_registertimer(&power_down_timer, POWER_DOWN_TIME);	
	}	

	return count;
}

static DEVICE_ATTR(power_down, S_IRUGO|S_IWUSR, NULL, store_power_down);


static int dpram_write(dpram_device_t *device,
		const unsigned char *buf, int len)
{
	int retval = 0;
	int size = 0;

	u16 head, tail;
	u16 irq_mask = 0;
	

	down(&write_mutex);

	READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));


	// +++++++++ head ---------- tail ++++++++++ //
	if (head < tail) {
		size = tail - head - 1;
		size = (len > size) ? size : len;

		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;
	}

	// tail +++++++++++++++ head --------------- //
	else if (tail == 0) {
		size = device->out_buff_size - head - 1;
		size = (len > size) ? size : len;

		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;
	}

	// ------ tail +++++++++++ head ------------ //
	else {
		size = device->out_buff_size - head;
		size = (len > size) ? size : len;
		
		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;

		if (len > retval) {
			size = (len - retval > tail - 1) ? tail - 1 : len - retval;
			
			WRITE_TO_DPRAM(device->out_buff_addr, buf + retval, size);
			retval += size;
		}
	}

	/* @LDK@ calculate new head */
	head = (u16)((head + retval) % device->out_buff_size);
	WRITE_TO_DPRAM_VERIFY(device->out_head_addr, &head, sizeof(head));

	/* @LDK@ send interrupt to the phone, if.. */
	irq_mask = INT_MASK_VALID;

	if (retval > 0)
		irq_mask |= device->mask_send;

	if (len > retval)
		irq_mask |= device->mask_req_ack;

	send_interrupt_to_phone(irq_mask);

	up(&write_mutex);
	return retval;
}

static inline
int dpram_tty_insert_data(dpram_device_t *device, const u8 *psrc, u16 size)
{
#define CLUSTER_SEGMENT 1550
	u16 copied_size = 0;
	int retval = 0;
	// ... ..... multipdp. .... raw data. ....
		
	if (size > CLUSTER_SEGMENT){
		while (size) {
			copied_size = (size > CLUSTER_SEGMENT) ? CLUSTER_SEGMENT : size;
			tty_insert_flip_string(device->serial.tty, psrc + retval, copied_size);

			size = size - copied_size;
			retval += copied_size;
		}

		return retval;
	}

	return tty_insert_flip_string(device->serial.tty, psrc, size);
}

static int dpram_read(dpram_device_t *device, const u16 non_cmd)
{
	int retval = 0;
	int size = 0;

	u16 head, tail;

#ifdef	NO_TTY_DPRAM
	struct tty_struct *tty = device->serial.tty;
#endif

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	if (head != tail) {
		u16 up_tail = 0;

		// ------- tail ++++++++++++ head -------- //
		if (head > tail) {
			size = head - tail;
#ifdef	NO_TTY_DPRAM
      		if( tty->index != 1) {	//index : 0=dpram0, 1=dpram1	
				retval = dpram_tty_insert_data(device, (unsigned char *)(SmemBase + (device->in_buff_addr + tail)), size);
			}else {	//2: dpram1
		    	retval = multipdp_buf_copy( 0, (unsigned char *)(SmemBase + (device->in_buff_addr + tail)), size);
			}
#endif
			if (retval != size)
				dprintk("Size Mismatch : Real Size = %d, Returned Size = %d\n", size, retval);
		}

		// +++++++ head ------------ tail ++++++++ //
		else {
			int tmp_size = 0;

			// Total Size.
			size = device->in_buff_size - tail + head;

			// 1. tail -> buffer end.
			tmp_size = device->in_buff_size - tail;
#ifdef	NO_TTY_DPRAM
      			if( tty->index != 1) {	//index : 0=dpram0, 1=dpram1	
					retval = dpram_tty_insert_data(device, (unsigned char *)(SmemBase + (device->in_buff_addr + tail)), tmp_size);
				}
				else {
		    		retval = multipdp_buf_copy( 0, (unsigned char *)(SmemBase + (device->in_buff_addr + tail)), tmp_size);
				}
#endif

			// 2. buffer start -> head.
			if (size > tmp_size) {
#ifdef	NO_TTY_DPRAM
      			if( tty->index != 1) {	//index : 0=dpram0, 1=dpram1	
					dpram_tty_insert_data(device, (unsigned char *)(SmemBase + device->in_buff_addr), size - tmp_size);
			}
			else {
		       		multipdp_buf_copy( tmp_size, (unsigned char *)(SmemBase + device->in_buff_addr), size - tmp_size);
			}
#endif
				retval += (size - tmp_size);
			}
		}

		/* new tail */
		up_tail = (u16)((tail + retval) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}

	if (non_cmd & device->mask_req_ack)
		send_interrupt_to_phone(INT_NON_COMMAND(device->mask_res_ack));

#ifdef	NO_TTY_DPRAM
	if( tty->index == 1)
		multipdp_rx_data(device, retval);
#endif

	return retval;
}

static void dpram_clear(void)
{
	long i = 0;
	unsigned long flags;
	
	u16 value = 0;

	dprintk("SmemBase = %x\n", (unsigned int)SmemBase);
	/* @LDK@ clear DPRAM except interrupt area */
	local_irq_save(flags);

	for (i=DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS; i<DPRAM_SIZE - (DPRAM_INTERRUPT_PORT_SIZE * 2); i+=2)
	{
		*((u16 *)(SmemBase + i)) = 0;
	}

	//for LPM mode booting
	*((u16 *)(SmemBase + DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS)) = 0x01;

	local_irq_restore(flags);

	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
}

static void dpram_init_and_report(void)
{
	const u16 magic_code = 0x00aa;
//	const u16 init_start = INT_COMMAND(INT_MASK_CMD_INIT_START);
//	const u16 init_end = INT_COMMAND(INT_MASK_CMD_INIT_END);
	const u16 init_end = INT_COMMAND(INT_MASK_CMD_INIT_END|INT_MASK_CP_AIRPLANE_BOOT|INT_MASK_CP_AP_ANDROID);

	u16 ac_code = 0;

	dprintk("start\n");

#if 0
	/* @LDK@ send init start code to phone */
	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			&init_start, DPRAM_INTERRUPT_PORT_SIZE);
	//writel(1, MSM_A2M_INT(4));
	MSM_TRIG_A2M_DPRAM_INT;

	/* @LDK@ write DPRAM disable code */
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));
#endif

	/* @LDK@ dpram clear */
	dpram_clear();


	/* @LDK@ write magic code */
	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &magic_code, sizeof(magic_code));

	/* @LDK@ write DPRAM enable code */
	ac_code = 0x0001;
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));
	
	/* @LDK@ send init end code to phone */
	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			&init_end, DPRAM_INTERRUPT_PORT_SIZE);

	//writel(1, MSM_A2M_INT(4));
	MSM_TRIG_A2M_DPRAM_INT;
	dprintk("finish\n");
}

static inline int dpram_get_read_available(dpram_device_t *device)
{
	u16 head, tail;

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	dprintk("%s : head = 0x%x\n", __func__, head);
	dprintk("%s : tail = 0x%x\n", __func__, tail);
	return head - tail;
}

static void dpram_drop_data(dpram_device_t *device)
{
	u16 head;

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &head, sizeof(head));
}

static void dpram_phone_on(void)
{
	dprintk("\n");
	dpram_init_and_report();
}

static void dpram_phone_off(void)
{
}

static int dpram_phone_getstatus(void)
{
	return 0;
}

static void dpram_phone_reset(void)
{
	const u16 reboot_magic_code = 0x3569;
	u16 magic_read;

	dprintk("[RAM DUMP] REBOOT_MAGIC_CODE\n");

	READ_FROM_DPRAM((void *)&magic_read, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic_read));
	dprintk("[RAM DUMP] Prev Magic Code : 0x%x\n", magic_read); 

	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &reboot_magic_code, sizeof(reboot_magic_code));
	dprintk("[RAM DUMP] SMSM WRITE\n");	
	
	READ_FROM_DPRAM((void *)&magic_read, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic_read));
	dprintk("[RAM DUMP] Cur Magic Code : 0x%x\n", magic_read); 
	
	msleep(100);
	smsm_reset_modem(SMSM_SYSTEM_DOWNLOAD);

}

static void dpram_mem_rw(struct _mem_param *param)
{
	/* @LDK@ write */
	if (param->dir) {
		WRITE_TO_DPRAM(param->addr, (void *)&param->data, sizeof(param->data));
	}

	/* @LDK@ read */
	else {
		READ_FROM_DPRAM((void *)&param->data, param->addr, sizeof(param->data));
	}
}

static void dpram_ramdump(void)
{
	printk("[DPRAM] RAMDUMP MODE START!\n");
//	printk("[DPRAM] write 0xCCCC flag at MSM_SHARED_RAM_BASE + 0x30 address\n");
//	printk("[DPRAM] MSM_SHARED_RAM_BASE: 0x%08x\n", (unsigned int) MSM_SHARED_RAM_BASE);
	writel(0xCCCC, MSM_SHARED_RAM_BASE + 0x30); 
	printk("[DPRAM] call msm_proc_comm_reset_modem_now func\n");
	msm_proc_comm_reset_modem_now();
}

static void print_smem(void)
{
	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;
	u16 in_interrupt = 0, out_interrupt = 0;
	u8 raw_out_buf;
//	u16 dump_data = 0;
#if 1
	struct file *filp;
//  int i;
	int writelen;
	mm_segment_t old_fs;
	static char buf[1024*32];
//	static int buf[1024];
	int count, chr_count;
//	char *src;
//	fl_owner_t id;
#endif

	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));
	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fmt_out_tail));
	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(raw_out_tail));
	READ_FROM_DPRAM((void *)&raw_out_buf, DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS, sizeof(raw_out_buf));
	READ_FROM_DPRAM((void *)&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, DPRAM_INTERRUPT_PORT_SIZE);
	READ_FROM_DPRAM((void *)&out_interrupt, DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, DPRAM_INTERRUPT_PORT_SIZE);
	dprintk("\n");
	printk("#####################################\n");
	printk("#########  DPRAM DUMP DATA (0604)  #########\n");
	printk("#####################################\n");
	printk("-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"| MAGIC CODE\t\t| 0x%04x\n"
			"| ENABLE CODE\t\t| 0x%04x\n"
			"| PHONE->PDA FMT HEAD\t| %u\n"
			"| PHONE->PDA FMT TAIL\t| %u\n"
			"| PDA->PHONE FMT HEAD\t| %u\n"
			"| PDA->PHONE FMT TAIL\t| %u\n"
			"| PHONE->PDA RAW HEAD\t| %u\n"
			"| PHONE->PDA RAW TAIL\t| %u\n"
			"| PDA->PHONE RAW HEAD\t| %u\n"
			"| PDA->PHONE RAW TAIL\t| %u\n"
			"| PDA->PHONE RAW BUFF\t| %u\n"
			"| PHONE->PDA INT.\t| 0x%04x\n"
			"| PDA->PHONE INT.\t| 0x%04x\n"
			"-------------------------------------\n",
			magic, enable,
			fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail,
			raw_in_head, raw_in_tail, raw_out_head, raw_out_tail, raw_out_buf,
			in_interrupt, out_interrupt
		);
#if 1
//		id = current->files;

		count = 1024 * 8;
		chr_count = 0;
		old_fs = get_fs();
        set_fs(KERNEL_DS);

		filp = filp_open("/sdcard/dpram_dump",O_CREAT|O_WRONLY,0666);
		if(!filp)
    		printk("Can't creat /sdcard/dpram_dump file\n");
		else 
		{
			memcpy((void *)buf, (void *)SmemBase, DPRAM_SIZE/*1024*32*/);
			writelen = filp->f_op->write(filp,(void *)buf,DPRAM_SIZE/*1024*32*/,&filp->f_pos);
		}
		set_fs(old_fs);
#if 0
		printk("\n");	
		printk("#####################################\n");
		printk("# PDA2PHONE FORMATTED BUFFER DUMP   #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
		
		for (i=DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS; i<DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("|\t0x%04x\t|\t0x%04x\t\t\n", i, dump_data);
		}    
																															   
		printk("\n");	
		printk("#####################################\n");
		printk("# PDA2PHONE RAW BUFFER DUMP         #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
			
		for (i=DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS; i<DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("| 0x%04x\t\t| 0x%04x\t\n", i, dump_data);
		}    
																																											  
		printk("\n");	
		printk("#####################################\n");
		printk("# PHONE2PDA FORMATTED BUFFER DUMP   #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
			
		for (i=DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS; i<DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("| 0x%04x\t\t| 0x%04x\t\n", i, dump_data);
		}    
		
		printk("\n");	
		printk("#####################################\n");
		printk("# PHONE2PDA RAW BUFFER DUMP         #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
			
		for (i=DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS; i<DPRAM_PDA2PHONE_INTERRUPT_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("| 0x%04x\t\t| 0x%04x\t\n", i, dump_data);
		}
#endif		
#endif
}

void request_phone_power_off_reset(int flag);


/* dpram tty file operations. */
static int dpram_tty_open(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = &dpram_table[tty->index];

	dprintk("tty->index = %d\n", tty->index);
	device->serial.tty = tty;
	device->serial.open_count++;

	if (device->serial.open_count > 1) {
		device->serial.open_count--;
		return -EBUSY;
	}

	#if 1	// hobac.

	if (tty->index == 1)	// dpram1
	{
		struct termios termios;
		mm_segment_t oldfs;

		oldfs = get_fs(); set_fs(get_ds());

                /* use kernel up to 2.6.38, if you use this line */
                if (file->f_op->unlocked_ioctl)
                        file->f_op->unlocked_ioctl(file, TCGETA, (unsigned long)&termios);

		set_fs(oldfs);

		termios.c_cflag = CS8 | CREAD | HUPCL | CLOCAL | B115200;
		termios.c_iflag = IGNBRK | IGNPAR;
		termios.c_lflag = 0;
		termios.c_oflag = 0;
		termios.c_cc[VMIN] = 1;
		termios.c_cc[VTIME] = 1;

		oldfs = get_fs(); set_fs(get_ds());

                /* use kernel up to 2.6.38, if you use this line */
                if (file->f_op->unlocked_ioctl)
                        file->f_op->unlocked_ioctl(file, TCSETA, (unsigned long)&termios);

		set_fs(oldfs);
	}

	#endif
	tty->driver_data = (void *)device;
	tty->low_latency = 1;

	return 0;
}

static void dpram_tty_close(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device && (device == &dpram_table[tty->index])) {
		down(&device->serial.sem);
		device->serial.open_count--;
		device->serial.tty = NULL;
		up(&device->serial.sem);
	}
}

static int dpram_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (!device) {
		return 0;
	}

	return dpram_write(device, buffer, count);
}

static int dpram_tty_write_room(struct tty_struct *tty)
{
	int avail;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
		READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

		avail = (head < tail) ? tail - head - 1 :
			device->out_buff_size + tail - head - 1;

		return avail;
	}

	return 0;
}

static int dpram_tty_ioctl(struct tty_struct *tty, unsigned int cmd,
				unsigned long arg)
{
	unsigned int val;

	switch (cmd) {
		case HN_DPRAM_PHONE_ON:
			if (DpramInited) 
			{
				dprintk("Doubled Phone On Cmd : do nothing\n");
				return 0;
			}
			dprintk("[Version 3] HN_DPRAM_PHONE_ON\n");
			dpram_phone_on();
			DpramInited = 1;
			return 0;

		case HN_DPRAM_PHONE_OFF:
			dprintk("HN_DPRAM_PHONE_OFF\n");
			dpram_phone_off();
			return 0;

		case HN_DPRAM_PHONE_GETSTATUS:
			dprintk("HN_DPRAM_PHONE_GETSTATUS\n");
			val = dpram_phone_getstatus();
			return copy_to_user((unsigned int *)arg, &val, sizeof(val));

		case HN_DPRAM_PHONE_RESET:
			dprintk("[RAM DUMP]HN_DPRAM_PHONE_RESET\n");
			dpram_phone_reset();
			return 0;

		case HN_DPRAM_MEM_RW:
		{
			struct _mem_param param;

			dprintk("HN_DPRAM_MEM_RW\n");
			val = copy_from_user((void *)&param, (void *)arg, sizeof(param));
			dpram_mem_rw(&param);

			if (!param.dir) {
				return copy_to_user((unsigned long *)arg, &param, sizeof(param));
			}

			return 0;
		}

		case HN_DPRAM_DUMP:
			print_smem();
			return 0;

		case HN_DPRAM_WAKELOCK:
			wake_lock(&imei_wake_lock);
			return 0;

		case HN_DPRAM_WAKEUNLOCK:
			wake_unlock(&imei_wake_lock);
			return 0;

		case HN_DPRAM_RAMDUMP:
			dpram_ramdump();
			return 0;
		default:
			dprintk("default\n");
			break;
	}

	return -ENOIOCTLCMD;
}

static int dpram_tty_chars_in_buffer(struct tty_struct *tty)
{
	int data;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
		READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

		data = (head > tail) ? head - tail - 1 :
			device->out_buff_size - tail + head;

		return data;
	}

	return 0;
}

#ifdef _ENABLE_ERROR_DEVICE
static int dpram_err_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);

	unsigned long flags;
	ssize_t ret;
	size_t ncopy;

	add_wait_queue(&dpram_err_wait_q, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	while (1) {
		local_irq_save(flags);

		if (dpram_err_len) {
			ncopy = min(count, dpram_err_len);

			if (copy_to_user(buf, dpram_err_buf, ncopy)) {
				ret = -EFAULT;
			}

			else {
				ret = ncopy;
			}

			dpram_err_len = 0;
			
			local_irq_restore(flags);
			break;
		}

		local_irq_restore(flags);

		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		schedule();
	}

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&dpram_err_wait_q, &wait);

	return ret;
}

static int dpram_err_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &dpram_err_async_q);
}

static unsigned int dpram_err_poll(struct file *filp,
		struct poll_table_struct *wait)
{
	poll_wait(filp, &dpram_err_wait_q, wait);
	return ((dpram_err_len) ? (POLLIN | POLLRDNORM) : 0);
}
#endif	/* _ENABLE_ERROR_DEVICE */

/* handlers. */
static void res_ack_tasklet_handler(unsigned long data)
{
	dpram_device_t *device = (dpram_device_t *)data;

	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
				tty->ldisc->ops->write_wakeup) {
			(tty->ldisc->ops->write_wakeup)(tty);
		}

		wake_up_interruptible(&tty->write_wait);
	}
}

static void send_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;

	if (device != NULL && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;
		ret = dpram_read(device, non_cmd);
		if (ret == -EAGAIN) {
			if (non_cmd & INT_MASK_SEND_F) tasklet_schedule(&fmt_send_tasklet);
			if (non_cmd & INT_MASK_SEND_R) tasklet_schedule(&raw_send_tasklet);
			return ;
		}
#ifdef	NO_TTY_DPRAM
      if( tty->index != 1)	//index : 0=dpram0, 1=dpram1	
#endif
		tty_flip_buffer_push(tty);
	}

	else {
		dpram_drop_data(device);
	}
}

static void cmd_req_active_handler(void)
{
	send_interrupt_to_phone(INT_COMMAND(INT_MASK_CMD_RES_ACTIVE));
}

static void cmd_error_display_handler(void)
{
	char buf[DPRAM_ERR_MSG_LEN];

#ifdef _ENABLE_ERROR_DEVICE
	unsigned long flags;
#endif	/* _ENABLE_ERROR_DEVICE */

	//for silent reset
	printk("[DPRAM] %s : silent reset,\n", __func__);
	wake_lock(&silent_wake_lock);
	memset((void *)buf, 0, sizeof (buf));
	buf[0] = '1';
	buf[1] = ' ';
	READ_FROM_DPRAM((buf + 2), DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS,
			sizeof (buf) - 3);

	dprintk("[PHONE ERROR] ->> %s\n", buf);

#ifdef _ENABLE_ERROR_DEVICE
	local_irq_save(flags);
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);
	dpram_err_len = 64;
	local_irq_restore(flags);

	wake_up_interruptible(&dpram_err_wait_q);
	kill_fasync(&dpram_err_async_q, SIGIO, POLL_IN);
#endif	/* _ENABLE_ERROR_DEVICE */
}

static void cmd_phone_start_handler(void)
{
	dprintk("\n");
	dpram_init_and_report();
}

static void cmd_req_time_sync_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_phone_deep_sleep_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_nv_rebuilding_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_emer_down_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_chg_detect_noti(void)
{
	u16 value;
	u16 irq_clear = 0x0000;

//	get_charger_type();
//	get_charging_status();
//	msm_batt_check_event();

	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
	if(value == 0x40C0) {
		WRITE_TO_DPRAM(DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, &irq_clear, DPRAM_INTERRUPT_PORT_SIZE);
		printk("[DPRAM:%s] chg_detect irq: 0x%x cleared.\n", __func__, value);
	}
	else {
		READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
		printk("[DPRAM:%s] changed irq: 0x%x detected.\n", __func__, value);
	}

}

static void cmd_chg_state_changed(void)
{
	u16 value;
	u16 irq_clear = 0x0000;

//	get_charging_status();
//	msm_batt_check_event();
	
	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
	if(value == 0x50C0) {
		WRITE_TO_DPRAM(DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, &irq_clear, DPRAM_INTERRUPT_PORT_SIZE);
		printk("[DPRAM:%s] chg_state irq: 0x%x cleared.\n", __func__, value);
	}
	else {
		READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
		printk("[DPRAM:%s] changed irq: 0x%x detected.\n", __func__, value);
	}

}
static void command_handler(u16 cmd)
{
	switch (cmd) {
		case INT_MASK_CMD_REQ_ACTIVE:
			cmd_req_active_handler();
			break;

		case INT_MASK_CMD_ERR_DISPLAY:
			cmd_error_display_handler();
			break;

		case INT_MASK_CMD_PHONE_START:
			cmd_phone_start_handler();
			break;

		case INT_MASK_CMD_REQ_TIME_SYNC:
			cmd_req_time_sync_handler();
			break;

		case INT_MASK_CMD_PHONE_DEEP_SLEEP:
			cmd_phone_deep_sleep_handler();
			break;

		case INT_MASK_CMD_NV_REBUILDING:
			cmd_nv_rebuilding_handler();
			break;

		case INT_MASK_CMD_EMER_DOWN:
			cmd_emer_down_handler();
			break;
		case INT_MASK_CMD_CHG_DETECT_NOTI:
			cmd_chg_detect_noti();
			break;

		case INT_MASK_CMD_CHG_STATE_CHANGED:
			cmd_chg_state_changed();
			break;

		default:
			dprintk("Unknown command..\n");
	}
}

static void non_command_handler(u16 non_cmd)
{
	u16 head, tail;

	/* @LDK@ formatted check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(tail));

	if (head != tail)
		non_cmd |= INT_MASK_SEND_F;

	/* @LDK@ raw check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(tail));

	if (head != tail)
		non_cmd |= INT_MASK_SEND_R;

	/* @LDK@ +++ scheduling.. +++ */
	if (non_cmd & INT_MASK_SEND_F) {
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		dpram_tasklet_data[FORMATTED_INDEX].device = &dpram_table[FORMATTED_INDEX];
		dpram_tasklet_data[FORMATTED_INDEX].non_cmd = non_cmd;
		
		fmt_send_tasklet.data = (unsigned long)&dpram_tasklet_data[FORMATTED_INDEX];
		tasklet_schedule(&fmt_send_tasklet);
	}

	if (non_cmd & INT_MASK_SEND_R) {
		wake_lock_timeout(&dpram_wake_lock, 6*HZ);
		dpram_tasklet_data[RAW_INDEX].device = &dpram_table[RAW_INDEX];
		dpram_tasklet_data[RAW_INDEX].non_cmd = non_cmd;

		raw_send_tasklet.data = (unsigned long)&dpram_tasklet_data[RAW_INDEX];
		/* @LDK@ raw buffer op. -> soft irq level. */
		tasklet_hi_schedule(&raw_send_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_F) {
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		tasklet_schedule(&fmt_res_ack_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_R) {
		wake_lock_timeout(&dpram_wake_lock, 6*HZ);
		tasklet_hi_schedule(&raw_res_ack_tasklet);
	}
}

static inline
void check_int_pin_level(void)
{
}

/* @LDK@ interrupt handlers. */
static irqreturn_t dpram_interrupt(int irq, void *dev_id)
{
	u16 irq_mask = 0;
#if 0
	u16 fih, fit, foh, fot;
	u16 rih, rit, roh, rot;
#endif

	dprintk("%s : interrupt handler\n", __func__);	
	
//	wake_lock_timeout(&dpram_wake_lock, HZ/2);

	READ_FROM_DPRAM(&irq_mask, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(irq_mask));
#if 0
	printk("=====>[%s,%d] irq_mask: %x\n", __func__, __LINE__, irq_mask);	
	READ_FROM_DPRAM_VERIFY(&fih, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fih));
	READ_FROM_DPRAM_VERIFY(&fit, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fit));
	READ_FROM_DPRAM_VERIFY(&foh, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(foh));
	READ_FROM_DPRAM_VERIFY(&fot, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fot));
	READ_FROM_DPRAM_VERIFY(&rih, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(rih));
	READ_FROM_DPRAM_VERIFY(&rit, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(rit));
	READ_FROM_DPRAM_VERIFY(&roh, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(roh));
	READ_FROM_DPRAM_VERIFY(&rot, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(rot));
	printk("\n fmt_in  H:%4d, T:%4d\n fmt_out H:%4d, T:%4d\n raw_in  H:%4d, T:%4d\n raw out H:%4d, T:%4d\n", fih, fit, foh, fot, rih, rit, roh, rot);
#endif

	/* valid bit verification. @LDK@ */
	if (!(irq_mask & INT_MASK_VALID)) {
		dprintk("Invalid interrupt mask: 0x%04x\n", irq_mask);
		return IRQ_NONE;
	}

	/* command or non-command? @LDK@ */
	if (irq_mask & INT_MASK_COMMAND) {
		irq_mask &= ~(INT_MASK_VALID | INT_MASK_COMMAND);
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		command_handler(irq_mask);
	}

	else {
		irq_mask &= ~INT_MASK_VALID;
		non_command_handler(irq_mask);
		//wake_lock_timeout(&dpram_wake_lock, 6*HZ);
	}

	return IRQ_HANDLED;
}
#if 0
static irqreturn_t phone_active_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
#endif
/* basic functions. */
#ifdef _ENABLE_ERROR_DEVICE
static struct file_operations dpram_err_ops = {
	.owner = THIS_MODULE,
	.read = dpram_err_read,
	.fasync = dpram_err_fasync,
	.poll = dpram_err_poll,
	.llseek = no_llseek,

	/* TODO: add more operations */
};
#endif	/* _ENABLE_ERROR_DEVICE */


static struct tty_operations dpram_tty_ops = {
	.open = dpram_tty_open,
	.close = dpram_tty_close,
	.write = dpram_tty_write,
	.write_room = dpram_tty_write_room,
	.ioctl = dpram_tty_ioctl,
	.chars_in_buffer = dpram_tty_chars_in_buffer,

	/* TODO: add more operations */
};

#ifdef _ENABLE_ERROR_DEVICE
static void unregister_dpram_err_device(void)
{
	unregister_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE);
	class_destroy(dpram_class);
}

static int register_dpram_err_device(void)
{
	/* @LDK@ 1 = formatted, 2 = raw, so error device is '0' */
	struct device *dpram_err_dev_t;
	int ret = register_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE, &dpram_err_ops);

	if ( ret < 0 )
	{
		return ret;
	}

	dpram_class = class_create(THIS_MODULE, "err");

	if (IS_ERR(dpram_class))
	{
		unregister_dpram_err_device();
		return -EFAULT;
	}

	dpram_err_dev_t = device_create(dpram_class, NULL,
			MKDEV(DRIVER_MAJOR_NUM, 0), NULL, DPRAM_ERR_DEVICE);

	if (IS_ERR(dpram_err_dev_t))
	{
		unregister_dpram_err_device();
		return -EFAULT;
	}

	return 0;
}
#endif	/* _ENABLE_ERROR_DEVICE */


static int register_dpram_driver(void)
{
	int retval = 0;

	/* @LDK@ allocate tty driver */
	dpram_tty_driver = alloc_tty_driver(MAX_INDEX);

	if (!dpram_tty_driver) {
		return -ENOMEM;
	}

	/* @LDK@ initialize tty driver */
	dpram_tty_driver->owner = THIS_MODULE;
	dpram_tty_driver->magic = TTY_DRIVER_MAGIC;
	dpram_tty_driver->driver_name = DRIVER_NAME;
	dpram_tty_driver->name = "dpram";
	dpram_tty_driver->major = DRIVER_MAJOR_NUM;
	dpram_tty_driver->minor_start = 1;
	dpram_tty_driver->num = 2;	// original
	dpram_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	dpram_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	dpram_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	dpram_tty_driver->init_termios = tty_std_termios;
	dpram_tty_driver->init_termios.c_cflag =
		(B115200 | CS8 | CREAD | CLOCAL | HUPCL);

	tty_set_operations(dpram_tty_driver, &dpram_tty_ops);

	dpram_tty_driver->ttys = dpram_tty;
	dpram_tty_driver->termios = dpram_termios;
	dpram_tty_driver->termios_locked = dpram_termios_locked;

	/* @LDK@ register tty driver */
	retval = tty_register_driver(dpram_tty_driver);

	if (retval) {
		dprintk("tty_register_driver error\n");
		put_tty_driver(dpram_tty_driver);
		return retval;
	}

	return 0;
}

static void unregister_dpram_driver(void)
{
	tty_unregister_driver(dpram_tty_driver);
}

static void init_devices(void)
{
	int i;

	for (i = 0; i < MAX_INDEX; i++) {
		init_MUTEX(&dpram_table[i].serial.sem);

		dpram_table[i].serial.open_count = 0;
		dpram_table[i].serial.tty = NULL;
	}
}

static void kill_tasklets(void)
{
	tasklet_kill(&fmt_res_ack_tasklet);
	tasklet_kill(&raw_res_ack_tasklet);

	tasklet_kill(&fmt_send_tasklet);
	tasklet_kill(&raw_send_tasklet);
}

static int register_interrupt_handler(void)
{
	int retval = 0;


	/* @LDK@ interrupt area read - pin level will be driven high. */
	dprintk("Dpram clear start\n");
	dpram_clear();

	/* @LDK@ Phone active INT. */
	
	/* @LDK@ dpram interrupt */
	retval = request_irq(INT_A9_M2A_4, dpram_interrupt, IRQF_TRIGGER_RISING, DRIVER_NAME, NULL);

	if (retval) {
		dprintk("DPRAM interrupt handler failed.\n");
		unregister_dpram_driver();
		return -1;
	}
	dprintk("INT_A9_M2A_4 interrupt handler success\n");
#if 0
	/* @LDK@ phone active interrupt */
	retval = request_irq(phone_active_irq, phone_active_interrupt,
			IRQF_DISABLED, "Phone Active", NULL);

	if (retval) {
		dprintk("Phone active interrupt handler failed.\n");
		free_irq(dpram_irq, NULL);
		unregister_dpram_driver();
		return -1;
	}
#endif

	return 0;
}

static void check_miss_interrupt(void)
{
	unsigned long flags;
	u16 head, tail;
	u16 value;

	dprintk("%s\n", __func__);
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(tail));

	dprintk("%s : head = 0x%x\n", __func__, head);
	dprintk("%s : tail = 0x%x\n", __func__, tail);

	if (head != tail) 
	{
		dprintk("there is a missed interrupt. try to read it!\n");
		printk("[DPRAM:%s] there is a missed interrupt. try to read it!\n", __func__);

		local_irq_save(flags);
		dpram_interrupt(INT_A9_M2A_4, NULL);
		local_irq_restore(flags);
	}
	
	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
	if(value == 0x40C0 || value == 0x50C0) {
		printk("[DPRAM:%s] there is a missed battery interrupt. try to read it!\n", __func__);

		local_irq_save(flags);
		dpram_interrupt(INT_A9_M2A_4, NULL);
		local_irq_restore(flags);
	}
}

static int dpram_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int dpram_resume(struct platform_device *dev)
{
	check_miss_interrupt();
	return 0;
}

enum {
	RESET,
	POWEROFF,
};

void request_phone_power_off_reset(int flag)
{
	unsigned char fmt_cmd_reset[12] = {0x7F, 0x0A, 0x00, 0x5C, 0x07, 0x00, 0x5C, 0x00, 0x01, 0x03, 0x05, 0x7E};
	unsigned char fmt_cmd_powoff[12] = {0x7F, 0x0A, 0x00, 0x5D, 0x07, 0x00, 0x5D, 0x00, 0x01, 0x02, 0x01, 0x7E};

	switch(flag) {
		case RESET :
			printk("Dpram Reset's called\n");
			dpram_write(&dpram_table[0], fmt_cmd_reset, sizeof(fmt_cmd_reset));
			break;
		case POWEROFF : 
			printk("Dpram Poweroff's called\n");
			dpram_write(&dpram_table[0], fmt_cmd_powoff, sizeof(fmt_cmd_powoff));
			break;
	}
}

static int __devinit dpram_probe(struct platform_device *dev)
{
	int retval;

	/* allocate smem dpram area */
	dprintk("SMEM_DPRAM allocation\n");
	SmemBase = (volatile unsigned char *)(smem_alloc(SMEM_ID_VENDOR0, DPRAM_SIZE/*0x4000*2*/));
	if (!SmemBase)
	{
		dprintk("smem_alloc failed : SmemBase = 0x%x\n", (unsigned int)SmemBase);
		return -1;
	}
	dprintk("SmemBase = 0x%x\n", (unsigned int)SmemBase);

	/* @LDK@ register dpram (tty) driver */
	retval = register_dpram_driver();

	if (retval) {
		dprintk("Failed to register dpram (tty) driver.\n");
		return -1;
	}

#ifdef _ENABLE_ERROR_DEVICE
	/* @LDK@ register dpram error device */
	retval = register_dpram_err_device();

	if (retval) {
		dprintk("Failed to register dpram error device.\n");

		unregister_dpram_driver();
		return -1;
	}

	memset((void *)dpram_err_buf, '\0', sizeof dpram_err_buf);
#endif /* _ENABLE_ERROR_DEVICE */

	/* @LDK@ register interrupt handler */
	dprintk("Register interrupt handler\n");
	if ((retval = register_interrupt_handler()) < 0) {
		return -1;
	}

	/* @LDK@ initialize device table */
	init_devices();

	/* @LDK@ check out missing interrupt from the phone */
	//check_miss_interrupt();

	return 0;
}

static int __devexit dpram_remove(struct platform_device *dev)
{
	/* @LDK@ unregister dpram (tty) driver */
	unregister_dpram_driver();

#ifdef _ENABLE_ERROR_DEVICE
	/* @LDK@ unregister dpram error device */
	unregister_dpram_err_device();
#endif

	kill_tasklets();

	return 0;
}

static struct platform_driver platform_dpram_driver = {
	.probe = dpram_probe,
	.remove = __devexit_p(dpram_remove),
	.suspend = dpram_suspend,
	.resume = dpram_resume,
	.driver = {
		.name = "dpram",
	},
};

struct smem_info {
	unsigned int info;
};

struct smem_info *smem_flag;
int silent_value = 0;
int dump_enable_flag = 0;

EXPORT_SYMBOL(dump_enable_flag);

void power_down_registertimer(struct timer_list* ptimer, unsigned long timeover )
{
        printk("%s\n",__func__);
        init_timer(ptimer);
        ptimer->expires = get_jiffies_64() + timeover;
        ptimer->data = (long) NULL;
        ptimer->function = power_down_timeout;
        add_timer(ptimer);
}
 
void power_down_timeout(unsigned long arg)
{
        printk("%s\n",__func__);
        //smem_flag->info = 0xAEAEAEAE;
        //msm_proc_comm_reset_modem_now();
        power_down = true;
        pm_power_off();
}

static int silent_read_proc_debug(char *page, char **start, off_t offset,
		                    int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "%u\n", silent_value);
}

static int silent_write_proc_debug(struct file *file, const char *buffer,
		                    unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		silent_value = 0;
		printk("Set silent : %d\n", silent_value);
	} else if (buf[0] == '1') {
		silent_value = 1;
		printk("Set silent : %d\n", silent_value);
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

static int dump_read_proc_debug(char *page, char **start, off_t offset,
		                    int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "%u\n", dump_enable_flag);
}

static int dump_write_proc_debug(struct file *file, const char *buffer,
		                    unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {			// low (no RAM dump)
		dump_enable_flag = 0;		
		smem_flag->info = 0xAEAEAEAE;
	} else if (buf[0] == '1') {		// middle (kernel fault)
		dump_enable_flag = 1;
		smem_flag->info = 0xA9A9A9A9;
	} else if (buf[0] == '2') {		// high (user fault)
		dump_enable_flag = 2;
		smem_flag->info = 0xA9A9A9A9;
	} else {
		kfree(buf);
		return -EINVAL;
	}

	printk("dump_enable_flag : %d, smem_flag : 0x%08x\n", dump_enable_flag, smem_flag->info);

	kfree(buf);
	return count;
}

static int nosim_proc_read(char *page, char **start, off_t offset,
		                    int count, int *eof, void *data)
{
	return 0;
}

static int nosim_proc_write(struct file *file, const char *buffer,
		                    unsigned long count, void *data)
{
	char *buf;

	if (count < 1 || count > 6)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (!strncmp(buf,"PANIC",5))
		BUG();

	kfree(buf);
	return count;
}
//#define AUTO_POWER_ON_OFF_FLAG //for auto Reset Test Sexykyu

#ifdef AUTO_POWER_ON_OFF_FLAG
/* init & cleanup. */
#define RESET_ALL       0xab800200
#define RESET_ALL_VAL   0x1
#endif
static int __init dpram_init(void)
{
	int ret;
	struct proc_dir_entry *ent;
	
#ifdef AUTO_POWER_ON_OFF_FLAG
	void __iomem *reset_base;

	reset_base = ioremap(RESET_ALL, PAGE_SIZE);

	writel(RESET_ALL_VAL, reset_base);

	while(1);
#endif
	ret = platform_driver_register(&platform_dpram_driver);
	if (ret) {
		goto error_return;
	}

	platform_device_register_simple("dpram", -1, NULL, 0);
	wake_lock_init(&imei_wake_lock, WAKE_LOCK_SUSPEND, "IEMI");
	wake_lock_init(&dpram_wake_lock, WAKE_LOCK_SUSPEND, "DPRAM");
	wake_lock_init(&silent_wake_lock, WAKE_LOCK_SUSPEND, "SILENT_RESET");

	// For silent ram dump mode
	
	ent = create_proc_entry("silent", S_IRWXUGO, NULL);
	ent->read_proc = silent_read_proc_debug;
	ent->write_proc = silent_write_proc_debug;
	smem_flag = (struct smem_info *) smem_alloc(SMEM_ID_VENDOR2, sizeof(struct smem_info));
	if(smem_flag->info == 0xAEAEAEAE)
		silent_value = 1;

	ent = create_proc_entry("dump_enable", S_IRWXUGO, NULL);
	ent->read_proc = dump_read_proc_debug;
	ent->write_proc = dump_write_proc_debug;
	smem_flag->info = 0xAEAEAEAE;
	printk("[Silent Value] : %d\n", silent_value);

	ent = create_proc_entry("nosim_handler", S_IRWXUGO, NULL);
	ent->read_proc = nosim_proc_read;
	ent->write_proc = nosim_proc_write;
	
	//sec_class = class_create(THIS_MODULE, "sec");
	if(IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");
	
	dpram_dev = device_create(sec_class, NULL, 0, NULL, "dpram");
	if(IS_ERR(dpram_dev))
		pr_err("Failed to create device(dpram)!\n");
	
	if(device_create_file(dpram_dev, &dev_attr_info) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_info.attr.name);
#ifdef CONFIG_DPRAM_WHITELIST
	if(device_create_file(dpram_dev, &dev_attr_whitelist) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_whitelist.attr.name);
#endif
	pm_dev = device_create(sec_class, NULL, 0, NULL, "pm");
	if(IS_ERR(pm_dev))
		pr_err("Failed to create device(pm)!\n");
	if(device_create_file(pm_dev, &dev_attr_info) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_info.attr.name);
	if(device_create_file(pm_dev, &dev_attr_power_down) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_power_down.attr.name);
error_return:

	return ret;
}

static void __exit dpram_exit(void)
{
	wake_lock_destroy(&dpram_wake_lock);
	wake_lock_destroy(&imei_wake_lock);
	wake_lock_destroy(&silent_wake_lock);
	platform_driver_unregister(&platform_dpram_driver);
}

module_init(dpram_init);
module_exit(dpram_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");
MODULE_DESCRIPTION("DPRAM Device Driver for Linux MITs.");
MODULE_LICENSE("GPL");


