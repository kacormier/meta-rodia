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
#include <stdio.h>
#include "ks.h"		// KS comat includes
#include "cres.h"	// FPGA_POKE
#include "util.h"	// error()
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
#include <stdio.h>
#include <stdint.h>
#include "fpga.h"
#include "simdrv.h"	// peek/poke ioctl params
#include "fpgaRegisterAccess.h"
#endif

static RESOURCE fpga_poke = FPGA_POKE;  // lock poke of FPGA registers
					// would be more efficient to hava a lock for each register

// SIM Driver prototypes
int peekSIMDriver(unsigned int ph, unsigned int addr, unsigned char *value);
int pokeSIMDriver(unsigned int ph, unsigned int addr, unsigned char value);
int updateBitsSIMDriver(unsigned int ph, unsigned int addr, unsigned char bits, int setUnset);


int peekFPGA(unsigned int ph, unsigned int addr, unsigned char *value)
{
  // Do the work
  return(peekSIMDriver(ph, addr, value));
}


int pokeFPGA(unsigned int ph, unsigned int addr, unsigned char value)
{
  // Do the work
  return(pokeSIMDriver(ph, addr, value));
}

int updateBitsFPGA(unsigned int ph, unsigned int addr, unsigned char bits, int setUnset)
{
  // Do the work
  return(updateBitsSIMDriver(ph, addr, bits, setUnset));
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
  snprintf(&devname[0], sizeof(devname), "/dev/simphone%d", ph);

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
peekSIMDriver(
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
          "Cannot peek SIM Driver (ph %d) register %x: errno %i",
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
pokeSIMDriver(
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
          "Cannot poke SIM Driver (ph %d) register %x: errno %i",
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
updateBitsSIMDriver(
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
            "Cannot poke SIM Driver (ph %d) register %x: errno %i",
            ph, addr, errno);
	} else
    // Failed
    error(eWARN,
          "Cannot peek SIM Driver (ph %d) register %x: errno %i",
          ph, addr, errno);

  // Unlock
	KS_unlock(fpga_poke);

 // Close
 close(simdrvFd);

 // Return result
 return(ret);
}