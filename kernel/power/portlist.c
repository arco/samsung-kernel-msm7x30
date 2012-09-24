#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>

#undef DEBUG_PORT_INFO
//#define DEBUG_PORT_INFO

#ifdef DEBUG_PORT_INFO
#define dprintk		printk
#else
#define dprintk(s,args...)
#endif

#define MAKE_HEADER(PTR, MAIN_CMD, SUB_CMD, CMD_TYPE, PKT_SIZE) \
					(PTR)->hdr.len = (unsigned short)PKT_SIZE; \
					(PTR)->hdr.main_cmd = (unsigned char)MAIN_CMD; \
					(PTR)->hdr.sub_cmd = (unsigned char)SUB_CMD;\
					(PTR)->hdr.cmd_type = (unsigned char)CMD_TYPE;

#define MAX_GPRS_PORT_LIST_LEN 200
#define IPC_DELIMITER_START 0x7F
#define IPC_DELIMITER_END 0x7E
#define IPC_GPRS_CMD 0xD
#define IPC_GPRS_PORT_LIST 0x11
#define IPC_CMD_SET 0x3

typedef struct {
   unsigned short len;
   unsigned char msg_seq;
   unsigned char ack_seq;
   unsigned char main_cmd;
   unsigned char sub_cmd;
   unsigned char cmd_type;
} __attribute__((packed)) ipc_hdr_type;


typedef struct {
   ipc_hdr_type hdr;
   unsigned char tcp_list_type;
   unsigned char tcp_list_len;
   unsigned short tcp_port_list[MAX_GPRS_PORT_LIST_LEN];
   unsigned char udp_list_type;
   unsigned char udp_list_len;
   unsigned short udp_port_list[MAX_GPRS_PORT_LIST_LEN];
} __attribute__((packed)) ipc_pda_gprs_port_list_set_type ;

typedef struct tagHDLCFrame
{
	unsigned char	m_StartMagicCode;
	unsigned short m_Length;
	unsigned char	m_ChannelID;	// RAW data frame format
	unsigned char	m_CtrlInfo;		// control information id
	void			*m_pValue;
	unsigned char	m_EndMagicCode;
} HDLCFrame_t;

unsigned char get_control_infoid(void)
{
	unsigned char s_infoid = 0;

#if 0	// useless code
	if (s_infoid >= 128)
		s_infoid = 0;
#endif	

	++s_infoid;

	return s_infoid;
}

static int readPortFromFile(const char* path,unsigned short* portList)
{
	int lineNum = 0;
//	int index = 0;
	int read_byte = 0;
	struct file* fp = NULL;
	char* readline = NULL;
	int port = 0;
	int port_index = 0;
	unsigned int ip = 0;
	char ipv6_path[16];
	char eachline[500];
	int readline_offset = 0;
	int eachline_offset = 0;

	// First, read from tcp or udp

	fp = filp_open(path, O_RDONLY , 00644);

	if (fp == NULL)
	{
	    printk("\n %s open failed",path);
	}

	readline = (char*)kmalloc(4096, GFP_KERNEL);

	read_byte = fp->f_op->read(fp, readline, 4096, &fp->f_pos);

	dprintk("read byte = %d\n",read_byte);

#ifdef DEBUG_PORT_INFO
	for ( index = 0; index < read_byte ; index++) {
		printk("%c",*(readline+index));
		}
#endif	
	// parse buffer and update port list

	while (readline_offset < read_byte) {
		if ( *(readline+readline_offset) == '\n')
			{
			sscanf(eachline,"%d: %x:%x",&lineNum, &ip, &port);		
			dprintk("Read IP, Port : %x:%x\n",ip,port);	

			if (ip != 0x0100007F && ip != 0x00000000) 
			{
				if ( port != 0  && ip != 0x00000000)
				{
					portList[port_index++] = (unsigned short)port;
					printk("WHITELIST : add port %4x\n", port);
				}
			}

			//clear line buffer
			memset(eachline, 0 , sizeof(eachline));
			eachline_offset = 0;
			}
		eachline[eachline_offset++] = *(readline+readline_offset);
		readline_offset++;
	}
	
	kfree(readline);

	if( fp != NULL ) {
		filp_close(fp, NULL);
		fp = NULL;
	}

	// Next, read from tcp6 or udp6
	sprintf(ipv6_path, "%s6", path);

	fp = filp_open(ipv6_path, O_RDONLY , 00644);

	if (fp == NULL)
	{
	    printk("\n %s open failed",path);
	    return 0;
	}

	readline_offset = 0;
	eachline_offset = 0;
	ip = 0; port = 0; lineNum = 0;

	readline = (char*)kmalloc(4096, GFP_KERNEL);

	read_byte = fp->f_op->read(fp, readline, 4096, &fp->f_pos);

	dprintk("read byte = %d\n",read_byte);

#ifdef DEBUG_PORT_INFO
	for ( index = 0; index < read_byte ; index++) {
		printk("%c",*(readline+index));
	}
#endif

	// parse buffer and update port list

	while (readline_offset < read_byte) {
		if ( *(readline+readline_offset) == '\n')
			{
			sscanf(eachline,"%d: %32x:%x",&lineNum, &ip, &port);
			//sscanf(readLine,"%*d: %*64[0-9a-fA-F]:%X", &port);
			dprintk("Read Port : %x:%x\n", ip, port);	

			if ( port != 0  && ip != 0x00000000)
			{
				portList[port_index++] = (unsigned short)port;
				printk("WHITELIST : add port %4x\n", port);
			}

			//clear line buffer
			memset(eachline, 0 , sizeof(eachline));
			eachline_offset = 0;
			}
		eachline[eachline_offset++] = *(readline+readline_offset);
		readline_offset++;
	}
	
	kfree(readline);

	if( fp != NULL ) {
		filp_close(fp, NULL);
		fp = NULL;
	}

	return port_index;
}

static int activePortGet(int portType, unsigned short* portList)
{
	int portLen = 0;

	if(portType == 1){ /* Read Tcp Port*/	
		portLen = readPortFromFile("/proc/net/tcp",portList);
		//LOGD("read from tcp : %d", portLen);
	}
	else if(portType == 2){ /* Read Udp Port*/
		portLen = readPortFromFile("/proc/net/udp",portList);	
		//LOGD("read from udp : %d", portLen);
	}
	
	return portLen;
}

static ipc_pda_gprs_port_list_set_type port_list_packet;
char* hdlc_frame = NULL;

int TxGPRS_SetPortList(void)
{
	int hdlc_size;
	char *rawdata = NULL;
	ipc_pda_gprs_port_list_set_type *pipc;
	HDLCFrame_t frame_message;

	dprintk("[%s][line:%d]\n",__func__, __LINE__);

	memset(&port_list_packet, 0, sizeof(port_list_packet));

	// tcp_list_type 0x1 menas whitelist
	port_list_packet.tcp_list_type = 0x1;
	port_list_packet.udp_list_type = 0x1;

	//TODO: You should re-implement below using kernel structure instead of /proc
	port_list_packet.tcp_list_len = activePortGet(1, port_list_packet.tcp_port_list);
	port_list_packet.udp_list_len = activePortGet(2, port_list_packet.udp_port_list);

	MAKE_HEADER(&port_list_packet, IPC_GPRS_CMD, IPC_GPRS_PORT_LIST, IPC_CMD_SET, sizeof(port_list_packet));

	pipc = &port_list_packet;

	//TODO: ??
	pipc->hdr.msg_seq = 0x0;
	pipc->hdr.ack_seq = 0x0;

	// 7F(1) + HDLC header length + ipc packet length + 7E(1)
	hdlc_size = 1 + 3 + pipc->hdr.len + 1;
	hdlc_frame = (char *)kmalloc(hdlc_size, GFP_KERNEL);
	if (hdlc_frame == NULL)
		return -1;

	// length = HDLC header length(3) + IPC packet length
	frame_message.m_Length = (unsigned short)(pipc->hdr.len+ 3);
	frame_message.m_CtrlInfo = get_control_infoid();
	// save target
	rawdata = (char *)(hdlc_frame);
	// start packet(1byte)
	rawdata[0] = IPC_DELIMITER_START;
	// frame length(2byte)
	rawdata[1] = frame_message.m_Length & 0x00ff;

	rawdata[2] = (frame_message.m_Length >> 8) & 0x00ff;
	// frame control

	// update last sent HDLC frame
	//SetLastSentHDLCFrameInfo(&frame_message);

	rawdata[3] = frame_message.m_CtrlInfo;

	memcpy(rawdata + 4, (char *)pipc, pipc->hdr.len);

	// end packet(1byte)
	rawdata[4 + pipc->hdr.len] = IPC_DELIMITER_END;

#ifdef DEBUG_PORT_INFO
	int i = 0;
	for ( ; i< hdlc_size; i++) {
		if(i %16 == 0) printk("\n");

		printk("%02x ",*(rawdata+i));
		}
#endif	

	dprintk("[%s][line:%d]\n",__func__, __LINE__);
	return hdlc_size;
}

void clear_portlist(void)
{
	dprintk("[%s][line:%d]\n",__func__, __LINE__);
	
	if(hdlc_frame !=  NULL) {
		kfree(hdlc_frame);
		hdlc_frame = NULL;
	}
}

#define WHITE_LIST_PATH	"/sys/class/sec/dpram/whitelist"

int writePacketToFile(char *abuf, int length)
{
	struct file *fp = NULL;
	int ret = 0;

	fp = filp_open(WHITE_LIST_PATH, O_WRONLY , 00666);
	if (fp == NULL) {
		printk("failed to open file %s\n", WHITE_LIST_PATH);
		return 0;
	}

	 ret = fp->f_op->write(fp, abuf, length, &fp->f_pos);

	 printk("request byte length = %d,  return byte = %d\n",length, ret);

	if( fp != NULL ) {
		filp_close(fp, NULL);
		fp = NULL;
	}

	return ret;
}

int process_white_list(void)
{
	int length = 0;
	int ret = 0;
	
	length = TxGPRS_SetPortList();
	ret = writePacketToFile( hdlc_frame, length);
	clear_portlist();

	if ( length == ret )
		return 0;

	return 1;
}
