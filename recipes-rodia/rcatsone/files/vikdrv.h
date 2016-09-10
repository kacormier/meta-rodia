/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
#ifndef _VIK_MEMORY_DBG_DRV_H_
#define _VIK_MEMORY_DBG_DRV_H_

/*
 */
#ifdef __KERNEL__
	#include <linux/types.h>		// linux KERNEL type definitions for uint16_t
#else
	#include <stdint.h>				// linux USER type definitions for uint16_t
#endif
#include <linux/ioctl.h>


#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif

#ifndef KERNEL_VERSION /* pre-2.1.90 didn't have it */
#  define KERNEL_VERSION(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

/* only allow 2.0.x  2.2.y and 2.4.z */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0) /* not < 2.4 */
#  error "This kernel is too old: not supported by this file"
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// #  error "This kernel is too recent: not supported by this file"
#endif

/* remember about the current version */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#  define LINUX_24
#else 			//LINUX_VERSION_CODE < KERNEL_VERSION(2,7,0)
#  define LINUX_26
#endif

#if defined(LINUX_24)
	#define ioread8 readb
	#define ioread16 readw
	#define ioread32 readl

	#define iowrite8 writeb
	#define iowrite16 writew
	#define iowrite32 writel
#endif

/*
 * Ioctl definitions
 */
/*
 *	read/write structure
 */
typedef struct CPU_Register {
	unsigned int	addr;
    #if defined(PHOENIX_PROBE)
    unsigned short  res;
    #else
	unsigned int	res;
    #endif
} CPU_Register;

/* Use 'c' as magic number */
#define VIKDRV_IOC_MAGIC  'c'

#define VIKDRV_MEM_READ    		_IOWR(VIKDRV_IOC_MAGIC, 100, CPU_Register)	 /* read memory */
#define VIKDRV_MEM_WRITE   		_IOWR(VIKDRV_IOC_MAGIC, 101, CPU_Register)	 /* write memory*/


#define VIKDRV_IOC_MINNR 100
#define VIKDRV_IOC_MAXNR 101
#endif	/*_VIK_MEMORY_DBG_DRV_H_*/
