/*
** =========================================================================
** File:
**     tspdrv.h
**
** Description: 
**     Constants and type definitions for the TouchSense Kernel Module.
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/
#include <mach/msm_iomap.h>

#ifndef _TSPDRV_H
#define _TSPDRV_H

/* Constants */
#define MODULE_NAME                         "tspdrv"
#define TSPDRV                              "/dev/"MODULE_NAME
#define TSPDRV_MAGIC_NUMBER                 0x494D4D52
#define TSPDRV_STOP_KERNEL_TIMER            _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 1)
//#define VIBE_DEBUG

/* 
** Obsolete IOCTL command 
** #define TSPDRV_IDENTIFY_CALLER           _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 2)
*/
#define TSPDRV_ENABLE_AMP                   _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 3)
#define TSPDRV_DISABLE_AMP                  _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 4)
#define TSPDRV_GET_NUM_ACTUATORS            _IO(TSPDRV_MAGIC_NUMBER & 0xFF, 5)
#define VIBE_MAX_DEVICE_NAME_LENGTH			64
#define SPI_HEADER_SIZE                     3   /* DO NOT CHANGE - SPI buffer header size */
#define VIBE_OUTPUT_SAMPLE_SIZE             50  /* DO NOT CHANGE - maximum number of samples */

#define GPIO_VIBTONE_EN1	163
#define GPIO_LEVEL_HIGH	1
#define GPIO_LEVEL_LOW	0
#define GPIO_VIBTONE_PWM	16

//#define VIBE_TUNING

/* Type definitions */
#ifdef __KERNEL__
typedef int8_t		VibeInt8;
typedef u_int8_t	VibeUInt8;
typedef int16_t		VibeInt16;
typedef u_int16_t	VibeUInt16;
typedef int32_t		VibeInt32;
typedef u_int32_t	VibeUInt32;
typedef u_int8_t	VibeBool;
typedef VibeInt32	VibeStatus;

typedef struct
{
    VibeUInt8 nActuatorIndex;  /* 1st byte is actuator index */
    VibeUInt8 nBitDepth;       /* 2nd byte is bit depth */
    VibeUInt8 nBufferSize;     /* 3rd byte is data size */
    VibeUInt8 dataBuffer[VIBE_OUTPUT_SAMPLE_SIZE];
} samples_buffer;

typedef struct
{
    VibeInt8 nIndexPlayingBuffer;
    VibeUInt8 nIndexOutputValue;
    samples_buffer actuatorSamples[2]; /* Use 2 buffers to receive samples from user mode */
} actuator_samples_buffer;

#endif

/* Error and Return value codes */
#define VIBE_S_SUCCESS                      0	/* Success */
#define VIBE_E_FAIL						    -4	/* Generic error */

#if defined(VIBE_RECORD) && defined(VIBE_DEBUG)
    void _RecorderInit(void);
    void _RecorderTerminate(void);
    void _RecorderReset(int nActuator);
    void _Record(int actuatorIndex, const char *format,...);
#endif

/* Kernel Debug Macros */
#ifdef __KERNEL__
    #ifdef VIBE_DEBUG
        #define DbgOut(_x_) printk _x_
    #else   /* VIBE_DEBUG */
        #define DbgOut(_x_)
    #endif  /* VIBE_DEBUG */

    #if defined(VIBE_RECORD) && defined(VIBE_DEBUG)
        #define DbgRecorderInit(_x_) _RecorderInit _x_
        #define DbgRecorderTerminate(_x_) _RecorderTerminate _x_
        #define DbgRecorderReset(_x_) _RecorderReset _x_
        #define DbgRecord(_x_) _Record _x_
    #else /* defined(VIBE_RECORD) && defined(VIBE_DEBUG) */
        #define DbgRecorderInit(_x_)
        #define DbgRecorderTerminate(_x_)
        #define DbgRecorderReset(_x_)
        #define DbgRecord(_x_)
    #endif /* defined(VIBE_RECORD) && defined(VIBE_DEBUG) */
#endif  /* __KERNEL__ */

#define __inp(port)         (*((volatile unsigned char *) (port)))
#define __inpw(port)        (*((volatile unsigned short *) (port)))
#define __inpdw(port)       (*((volatile unsigned int *) (port)))

#define __outp(port, val)   (*((volatile unsigned char *) (port)) = ((unsigned char) (val)))
#define __outpw(port, val)  (*((volatile unsigned short *) (port)) = ((unsigned short) (val)))
#define __outpdw(port, val) (*((volatile unsigned int *) (port)) = ((unsigned int) (val)))


#define in_dword(addr)              (__inpdw(addr))
#define in_dword_masked(addr, mask) (__inpdw(addr) & (mask))
#define out_dword(addr, val)        __outpdw(addr,val)
#define out_dword_masked(io, mask, val, shadow)  \
   shadow = (shadow & (unsigned int)(~(mask))) | ((unsigned int)((val) & (mask))); \
   (void) out_dword( io, shadow); 
#define out_dword_masked_ns(io, mask, val, current_reg_content) \
  (void) out_dword( io, ((current_reg_content & (unsigned int)(~(mask))) | ((unsigned int)((val) & (mask)))) )

#define HWIO_GP_MD_REG_ADDR                                                 (MSM_CLK_CTL_BASE      + 0x00000058)
#define HWIO_GP_MD_REG_PHYS                                                 (MSM_CLK_CTL_PHYS + 0x00000058)
#define HWIO_GP_MD_REG_RMSK                                                 0xffffffff
#define HWIO_GP_MD_REG_SHFT                                                          0
#define HWIO_GP_MD_REG_IN                                                   \
        in_dword_masked(HWIO_GP_MD_REG_ADDR, HWIO_GP_MD_REG_RMSK)
#define HWIO_GP_MD_REG_INM(m)                                               \
        in_dword_masked(HWIO_GP_MD_REG_ADDR, m)
#define HWIO_GP_MD_REG_OUT(v)                                               \
        out_dword(HWIO_GP_MD_REG_ADDR,v)
#define HWIO_GP_MD_REG_OUTM(m,v)                                            \
	out_dword_masked_ns(HWIO_GP_MD_REG_ADDR,m,v,HWIO_GP_MD_REG_IN)

#define HWIO_GP_MD_REG_M_VAL_BMSK                                           0xffff0000
#define HWIO_GP_MD_REG_M_VAL_SHFT                                                 0x10
#define HWIO_GP_MD_REG_D_VAL_BMSK                                               0xffff
#define HWIO_GP_MD_REG_D_VAL_SHFT                                                    0

#define HWIO_GP_NS_REG_ADDR                                                 (MSM_CLK_CTL_BASE      + 0x0000005C)
#define HWIO_GP_NS_REG_PHYS                                                 (MSM_CLK_CTL_PHYS + 0x0000005C)
#define HWIO_GP_NS_REG_RMSK                                                 0xffffffff
#define HWIO_GP_NS_REG_SHFT                                                          0
#define HWIO_GP_NS_REG_IN                                                   \
        in_dword_masked(HWIO_GP_NS_REG_ADDR, HWIO_GP_NS_REG_RMSK)
#define HWIO_GP_NS_REG_INM(m)                                               \
        in_dword_masked(HWIO_GP_NS_REG_ADDR, m)
#define HWIO_GP_NS_REG_OUT(v)                                               \
        out_dword(HWIO_GP_NS_REG_ADDR,v)
#define HWIO_GP_NS_REG_OUTM(m,v)                                            \
	out_dword_masked_ns(HWIO_GP_NS_REG_ADDR,m,v,HWIO_GP_NS_REG_IN)
#define HWIO_GP_NS_REG_GP_N_VAL_BMSK                                        0xffff0000
#define HWIO_GP_NS_REG_GP_N_VAL_SHFT                                              0x10
#define HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK                                          0x800
#define HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT                                            0xb
#define HWIO_GP_NS_REG_GP_CLK_INV_BMSK                                           0x400
#define HWIO_GP_NS_REG_GP_CLK_INV_SHFT                                             0xa
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK                                    0x200
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT                                      0x9
#define HWIO_GP_NS_REG_MNCNTR_EN_BMSK                                            0x100
#define HWIO_GP_NS_REG_MNCNTR_EN_SHFT                                              0x8
#define HWIO_GP_NS_REG_MNCNTR_RST_BMSK                                            0x80
#define HWIO_GP_NS_REG_MNCNTR_RST_SHFT                                             0x7
#define HWIO_GP_NS_REG_MNCNTR_MODE_BMSK                                           0x60
#define HWIO_GP_NS_REG_MNCNTR_MODE_SHFT                                            0x5
#define HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK                                           0x18
#define HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT                                            0x3
#define HWIO_GP_NS_REG_SRC_SEL_BMSK                                                0x7
#define HWIO_GP_NS_REG_SRC_SEL_SHFT                                                  0

#define __msmhwio_outm(hwiosym, mask, val)  HWIO_##hwiosym##_OUTM(mask, val)
#define HWIO_OUTM(hwiosym, mask, val)                    __msmhwio_outm(hwiosym, mask, val)
#endif  /* _TSPDRV_H */
