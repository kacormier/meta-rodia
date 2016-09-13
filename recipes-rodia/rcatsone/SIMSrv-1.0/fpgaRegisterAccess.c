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
#include <stdint.h>
#include "fpga.h"
#include "../../LinuxDrivers/phoenix/sim/simdrv.h"
#include "fpgaRegisterAccess.h"
#else
#include "stubs.h"
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>	// O_RDWR, O_NONBLOCK
#include <errno.h>
// #include "ks.h"		// KS comat includes
#include "cres.h"	// FPGA_POKE
// #include "util.h"	// error()
#include "vikdrv.h"	// peek/poke ioctl params
#include <stdint.h>
#include "fpga.h"
#include "simdrv.h"	// peek/poke ioctl params
#include "fpgaRegisterAccess.h"
#endif

static int fppfd = -1;  // FPGA Peek/Poke File Descriptor for examining and setting FPGA registers
static RESOURCE fpga_poke = FPGA_POKE;  // lock poke of FPGA registers
					// would be more efficient to hava a lock for each register

// AMP prototypes
int peekAMP(unsigned int ph, unsigned int addr, unsigned char *value);
int pokeAMP(unsigned int ph, unsigned int addr, unsigned char value);
int updateBitsAMP(unsigned int ph, unsigned int addr, unsigned char bits, int setUnset);

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


int peekFPGA(unsigned int ph, unsigned int addr, unsigned char *value)
{
   CPU_Register   fpgaReg;

  // If AMP bound...
  if (ph >= AMP_PHONE_MIN &&
      ph <= AMP_PHONE_MAX)
  {
    // Do the work
    return(peekAMP(ph, addr, value));
  }

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
      error(eWARN,
            "Cannot peek FPGA probe (ph %d) register %x: errno %i",
            ph, addr, errno);
      return -1;
   }
   *value = fpgaReg.res;
   return 0;
}


int pokeFPGA(unsigned int ph, unsigned int addr, unsigned char value)
{
	CPU_Register	fpgaReg;
	int		rtnVal;

  // If AMP bound...
  if (ph >= AMP_PHONE_MIN &&
      ph <= AMP_PHONE_MAX)
  {
    // Do the work
    return(pokeAMP(ph, addr, value));
  }

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
      error(eWARN,
            "Cannot poke FPGA probe (ph %d) register %x: errno %i",
            ph, addr, errno);
	KS_unlock(fpga_poke);
	return rtnVal;
}

int updateBitsFPGA(unsigned int ph, unsigned int addr, unsigned char bits, int setUnset)
{
	CPU_Register	fpgaReg;
	int rtnVal;

  // If AMP bound...
  if (ph >= AMP_PHONE_MIN &&
      ph <= AMP_PHONE_MAX)
  {
    // Do the work
    return(updateBitsAMP(ph, addr, bits, setUnset));
  }

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
      error(eWARN,
            "Cannot poke FPGA probe (ph %d) register %x: errno %i",
            ph, addr, errno);
	} else
      error(eWARN,
            "Cannot peek FPGA probe (ph %d) register %x: errno %i",
            ph, addr, errno);
	KS_unlock(fpga_poke);
	return rtnVal;
}


int
openAMPRegisterAccess(
  unsigned int ph)
{
  // Locals
  char devname[20];
  int simdrvFd = -1;

  // Lock
  KS_lockw(fpga_poke);

  // Format device name
  snprintf(&devname, sizeof(devname), "/dev/simphone%d", ph);

  // Access
  simdrvFd = open(devname, O_RDWR | O_NONBLOCK);

  // If failed to open...
  if (simdrvFd < 0)
  {
    // Log
    error(eFATAL, "Cannot open AMP peek/poke device %s: errno %i",
          devname, errno);
  }

  // Unlock
  KS_unlock(fpga_poke);

  // Return result
  return(simdrvFd);
}

int
peekAMP(
  unsigned int ph,
  unsigned int addr,
  unsigned char *value)
{
  AMP_Register fpgaReg;
  int simdrvFd = -1;
  int ret = 0;

  // Open access
  simdrvFd = openAMPRegisterAccess(ph);

  // If failed...
  if (simdrvFd < 0)
    return -1;

  // Configure
  fpgaReg.addr = addr;

  // Disatch
  if (ioctl(simdrvFd, SIMDRV_MEM_READ, &fpgaReg) < 0)
  {
    // Failed
    error(eWARN,
          "Cannot peek FPGA AMP (ph %d) register %x: errno %i",
          ph, addr, errno);
    ret = -1;
  }

  // Set value
  *value = fpgaReg.res;

   // Close
   close(simdrvFd);

   // Return result
   return(ret);
}

int
pokeAMP(
  unsigned int ph,
  unsigned int addr,
  unsigned char value)
{
  AMP_Register fpgaReg;
  int simdrvFd = -1;
  int ret = 0;

  // Open access
  simdrvFd = openAMPRegisterAccess(ph);

  // If failed...
  if (simdrvFd < 0)
    return -1;

  // Configure
	fpgaReg.addr = addr;
	fpgaReg.res = value;

  // Lock
	KS_lockw(fpga_poke);

  // Disatch
  if (ioctl(simdrvFd, SIMDRV_MEM_WRITE, &fpgaReg) < 0)
  {
    // Failed
    error(eWARN,
          "Cannot poke FPGA AMP (ph %d) register %x: errno %i",
          ph, addr, errno);
    ret = -1;
  }

  // Unlock
	KS_unlock(fpga_poke);

 // Close
 close(simdrvFd);

 // Return result
 return(ret);
}

int
updateBitsAMP(
  unsigned int ph,
  unsigned int addr,
  unsigned char bits,
  int setUnset)
{
  AMP_Register fpgaReg;
  int simdrvFd = -1;
  int ret = 0;

  // Open access
  simdrvFd = openAMPRegisterAccess(ph);

  // If failed...
  if (simdrvFd < 0)
    return -1;

  // Configure
	fpgaReg.addr = addr;

  // Lock
	KS_lockw(fpga_poke);

	if ((ret = ioctl(simdrvFd, SIMDRV_MEM_READ, &fpgaReg)) == 0) {
		if (setUnset)
			fpgaReg.res |= bits;
		else
			fpgaReg.res &= ~bits;
		if ((ret = ioctl(simdrvFd, SIMDRV_MEM_WRITE, &fpgaReg)) < 0)
      // Failed
      error(eWARN,
            "Cannot poke FPGA AMP (ph %d) register %x: errno %i",
            ph, addr, errno);
	} else
    // Failed
    error(eWARN,
          "Cannot peek FPGA AMP (ph %d) register %x: errno %i",
          ph, addr, errno);

  // Unlock
	KS_unlock(fpga_poke);

 // Close
 close(simdrvFd);

 // Return result
 return(ret);
}