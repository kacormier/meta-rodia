#ifndef _SIMDRV_USBSUP_H_
#define _SIMDRV_USBSUP_H_

// USB support routines

// Include support
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <asm/types.h>

// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

int 
usbio(
  int thePhoneId, 
  char *theBuffer, 
  size_t theLength);

#endif  // _SIMDRV_USBSUP_H_