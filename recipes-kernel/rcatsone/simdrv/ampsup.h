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

int amp_AttachDevice( struct FpgaRegs *p_Fpga,
                      uint16_t p_DeviceType, uint16_t p_DeviceId);
                       
int amp_ClockSim(struct FpgaRegs *p_Fpga, StateChange p_NewState);

int amp_PowerSim  (struct FpgaRegs *p_Fpga, StateChange p_NewState);

void amp_GetFpgaRegisters(struct FpgaRegs *p_Fpga, int p_PhoneId);

void amp_GetUartRegisters(struct UartRegs *p_Uart, int p_PhoneId);

int amp_WarmResetSim(struct FpgaRegs *p_Fpga);

unsigned int amp_ioread8(void *theAddress);

void amp_iowrite8(uint8_t tjeValue, void *theAddress);

// ----------------------------------------------------------------------------
// Inlines
// ----------------------------------------------------------------------------

//
//  low level Status Byte functions
//  return == 0 - SIM not present;
//         != 0 - SIM present
//
static inline uint8_t amp_IsSimPresent(struct FpgaRegs *p_Fpga)
{
    return ioread8(p_Fpga->sim_status) & FPGA_SIM_STAT_SIM_PRESENT;
}
//
static inline uint8_t amp_IsSimReady(struct FpgaRegs *p_Fpga)
{
    return ioread8(p_Fpga->sim_status) & FPGA_SIM_STAT_SIM_PRESENT;
}

//
// set SIM reset line On/Off
//      state == 0 - Reset line off
//      state == 1 - Reset line on
//
static inline
int amp_ResetSim(struct FpgaRegs *p_Fpga, StateChange p_NewState)
{
    uint8_t     byRconCsr = ioread8(p_Fpga->sim_rcon_csr);

    if ( p_NewState == STATE_ON && ((byRconCsr & RCON_CSR_RST) == 0) )
    {   // Reset line off - Set it on
        if ((byRconCsr & RCON_CSR_CLK) == 0)    // if clock disable => enable the clock first
        {       // enable clock
            int retval = 0;
            if ((retval = fpga_ClockSim(p_Fpga,STATE_ON)) != 0)
                return retval;
        }
        // set reset line
        byRconCsr = ioread8(p_Fpga->sim_rcon_csr);
        iowrite8((byRconCsr | RCON_CSR_RST), p_Fpga->sim_rcon_csr);
        // how can I check that Reset line is set ????
        //???????
    }
    else if ( p_NewState == STATE_OFF && ((byRconCsr & RCON_CSR_RST) != 0) )
    {       // clear reset line
        iowrite8((byRconCsr & ~RCON_CSR_RST), p_Fpga->sim_rcon_csr);
    }
    return 0;
}

#endif  // _SIMDRV_AMPSUP_H_