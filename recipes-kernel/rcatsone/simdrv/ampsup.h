#ifndef _SIMDRV_AMPSUP_H_
#define _SIMDRV_AMPSUP_H_

// AMP support routines

// Include support
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <asm/types.h>

// ----------------------------------------------------------------------------
// Definitions
// ----------------------------------------------------------------------------
#define SIMDRV_AMP_FPGA_BASE_ADDRESS   0x5000;

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------
extern uint8_t * amp_mapped_address;

// ----------------------------------------------------------------------------
// Structures
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

int amp_AttachDevice(int thePhoneId, 
                     struct FpgaRegs *p_Fpga,
                     uint16_t p_DeviceType, 
                     uint16_t p_DeviceId);
                       
int amp_ClockSim(int thePhoneId,
                 struct FpgaRegs *p_Fpga, 
                 StateChange p_NewState);

int amp_PowerSim(int thePhoneId,
                 struct FpgaRegs *p_Fpga, 
                 StateChange p_NewState);

void amp_GetFpgaRegisters(struct FpgaRegs *p_Fpga, int p_PhoneId);

void amp_GetUartRegisters(struct UartRegs *p_Uart, int p_PhoneId);

int amp_WarmResetSim(int thePhoneId,
                     struct FpgaRegs *p_Fpga);

uint8_t amp_IsSimPresent(int thePhoneId,
                         struct FpgaRegs *p_Fpga);

uint8_t amp_IsSimReady(int thePhoneId,
                       struct FpgaRegs *p_Fpga);

unsigned int amp_ioread8(int thePhoneId, 
                         void *theAddress);

int amp_iowrite8(int thePhoneId, 
                 uint8_t theValue, 
                 void *theAddress);

// ----------------------------------------------------------------------------
// Inlines
// ----------------------------------------------------------------------------



#endif  // _SIMDRV_AMPSUP_H_