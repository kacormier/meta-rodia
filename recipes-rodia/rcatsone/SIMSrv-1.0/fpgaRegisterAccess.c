/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */

#include <linux/version.h>

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

// If not development board...
#ifndef SIM_DEV_BOARD
#include <sys/ioctl.h>
#include <fcntl.h>	// O_RDWR, O_NONBLOCK
#include <errno.h>
//#include <string.h>	// strerror()
#include "ks.h"		// KS comat includes
#include "cres.h"	// FPGA_POKE
#include "util.h"	// error()
#include "vikdrv.h"	// peek/poke ioctl params
#else
#include "stubs.h"
#include <sys/ioctl.h>
#include <fcntl.h>	// O_RDWR, O_NONBLOCK
#include <errno.h>
// #include "ks.h"		// KS comat includes
#include "cres.h"	// FPGA_POKE
// #include "util.h"	// error()
#include "vikdrv.h"	// peek/poke ioctl params
#endif

static int fppfd = -1;  // FPGA Peek/Poke File Descriptor for examining and setting FPGA registers
static RESOURCE fpga_poke = FPGA_POKE;  // lock poke of FPGA registers
					// would be more efficient to hava a lock for each register

int openFPGARegisterAccess(void)
{
   KS_lockw(fpga_poke);
   // don't open the file if it's already open
   if (fppfd < 0)
   {
      if ((fppfd = open("/dev/vik0", O_RDWR | O_NONBLOCK)) == -1)
         error(eFATAL, "Cannot open FPGA peek/poke device /dev/vik0: errno %i", errno);
   }
   KS_unlock(fpga_poke);
   return fppfd;
}


int peekFPGA(unsigned int addr, unsigned char *value)
{
   CPU_Register   fpgaReg;

   // hack: open the FPGA access file if it isn't already (openFPGA.. call is
   //  thread-safe).  This open call really belongs in the hardware init layer,
   //  but for some reason it is SIMClientStartup, which doesn't run on the
   //  CDMA probes.
   if (fppfd < 0)
      if (openFPGARegisterAccess() < 0)
         return -1;

   fpgaReg.addr = addr;
   if (ioctl(fppfd, VIKDRV_MEM_READ, &fpgaReg) < 0)
   {
      error(eWARN, "Cannot peek FPGA register %x: errno %i", addr, errno);
      return -1;
   }
   *value = fpgaReg.res;
   return 0;
}


int pokeFPGA(unsigned int addr, unsigned char value)
{
	CPU_Register	fpgaReg;
	int		rtnVal;

   // hack: open the FPGA access file if it isn't already (openFPGA.. call is
   //  thread-safe).  This open call really belongs in the hardware init layer,
   //  but for some reason it is SIMClientStartup, which doesn't run on the
   //  CDMA probes.
   // celder added 2007Feb13 bugfix #2706 - preload SIM command failed because
   //  openFPGARegisterAccess() has not yet been called.
   if (fppfd < 0)
      if (openFPGARegisterAccess() < 0)
         return -1;

	fpgaReg.addr = addr;
	fpgaReg.res = value;
	KS_lockw(fpga_poke);
	if ((rtnVal = ioctl(fppfd, VIKDRV_MEM_WRITE, &fpgaReg)) < 0)
		error(eWARN, "Cannot poke FPGA register %x: errno %i", addr, errno);
	KS_unlock(fpga_poke);
	return rtnVal;
}

int updateBitsFPGA(unsigned int addr, unsigned char bits, int setUnset)
{
	CPU_Register	fpgaReg;
	int rtnVal;

   // hack: open the FPGA access file if it isn't already (openFPGA.. call is
   //  thread-safe).  This open call really belongs in the hardware init layer,
   //  but for some reason it is SIMClientStartup, which doesn't run on the
   //  CDMA probes.
   // celder added 2007Feb13 bugfix #2706 - preload SIM command failed because
   //  openFPGARegisterAccess() has not yet been called.
   if (fppfd < 0)
      if (openFPGARegisterAccess() < 0)
         return -1;

	fpgaReg.addr = addr;
	KS_lockw(fpga_poke);
	if ((rtnVal = ioctl(fppfd, VIKDRV_MEM_READ, &fpgaReg)) == 0) {
		if (setUnset)
			fpgaReg.res |= bits;
		else
			fpgaReg.res &= ~bits;
		if ((rtnVal = ioctl(fppfd, VIKDRV_MEM_WRITE, &fpgaReg)) < 0)
			error(eWARN, "Cannot poke FPGA register %x: errno %i", addr, errno);
	} else
		error(eWARN, "Cannot peek FPGA register %x: errno %i", addr, errno);
	KS_unlock(fpga_poke);
	return rtnVal;
}
