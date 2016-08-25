#ifndef _SIMDRV_FPGASUP_H_
#define _SIMDRV_FPGASUP_H_

// Include support
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <asm/types.h>

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------
extern uint8_t * fpga_mapped_address;

// ----------------------------------------------------------------------------
// Structures
// ----------------------------------------------------------------------------

//----- SIM/Phone control registers
typedef struct FpgaRegs
{
    // SIM control registers
    uint8_t     *sim_status;
    // UART control registers
    uint8_t     *sim_rcon_csr;
    uint8_t     *sim_rcon_addr;
    // phone control registers
    uint8_t     *ph_ssr;
    uint8_t     *ph_scr;
    uint8_t     *ph_imr;
} FpgaRegs;

//
// ----- FPGA register precalculated addresses - to speed up IRQ
//
typedef struct UartRegs
{
    uint8_t     *rbr;
    uint8_t     *ier;
    uint8_t     *iir;
    uint8_t     *fcr;       // FIFO control register
    uint8_t     *lcr;       // Line Control Register
    uint8_t     *lsr;       // Line Status  Register
    uint8_t     *msr;
    uint8_t     *dll;
    uint8_t     *dlh;
    uint8_t     *spc;
    uint8_t     *t0ctr;
    uint8_t     *rtl;
    uint8_t     *tfs;
    uint8_t     *tcd;
} UartRegs;

// SSW remote connection control/status register:
#define RCON_CSR_RST      0x08
#define RCON_CSR_CLK      0x04
#define RCON_CSR_VCC      0x03      // mask for VCC
#define RCON_CSR_VCC5     0x03
#define RCON_CSR_VCC3     0x02
#define RCON_CSR_VCC18    0x01

#define SST_VCC_READY       (1<<4)
#define SST_VCC_FAULT       (1<<3)

// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

int fpga_AttachDevice( struct FpgaRegs *p_Fpga,
                       uint16_t p_DeviceType, uint16_t p_DeviceId);
                       
int fpga_ClockSim(struct FpgaRegs *p_Fpga, StateChange p_NewState);

int fpga_PowerSim  (struct FpgaRegs *p_Fpga, StateChange p_NewState);

void fpga_GetFpgaRegisters(int p_PhoneId, struct FpgaRegs *p_Fpga);

int fpga_WarmResetSim(struct FpgaRegs *p_Fpga);

// ----------------------------------------------------------------------------
// Inlines
// ----------------------------------------------------------------------------

//
// function fills up struct UartRegs with FPGA adresses
//
static inline void fpga_GetUartRegisters(int p_PhoneId, struct UartRegs *p_Uart)
{
    // Precompute register addresses.
    p_Uart->rbr = fpga_mapped_address + UART_RHR(p_PhoneId);
    p_Uart->ier = fpga_mapped_address + UART_IER(p_PhoneId);
    p_Uart->iir = fpga_mapped_address + UART_IIR(p_PhoneId);
    p_Uart->fcr = p_Uart->iir;
    p_Uart->lcr = fpga_mapped_address + UART_LCR(p_PhoneId);
    p_Uart->lsr = fpga_mapped_address + UART_LSR(p_PhoneId);
    p_Uart->msr = fpga_mapped_address + UART_MSR(p_PhoneId);
    p_Uart->dll = fpga_mapped_address + UART_DLLA(p_PhoneId);
    p_Uart->dlh = fpga_mapped_address + UART_DLHA(p_PhoneId);
    p_Uart->spc = fpga_mapped_address + UART_SPC(p_PhoneId);

    p_Uart->t0ctr = fpga_mapped_address + UART_T0_CTR(p_PhoneId);
    p_Uart->rtl = fpga_mapped_address + UART_RTL(p_PhoneId);
    p_Uart->tfs = fpga_mapped_address + UART_TFS(p_PhoneId);
    p_Uart->tcd = fpga_mapped_address + UART_TCD(p_PhoneId);
}

//
//  low level Status Byte functions
//  return == 0 - SIM not present;
//         != 0 - SIM present
//
static inline uint8_t fpga_IsSimPresent(struct FpgaRegs *p_Fpga)
{
    return ioread8(p_Fpga->sim_status) & FPGA_SIM_STAT_SIM_PRESENT;
}
//
static inline uint8_t fpga_IsSimReady(struct FpgaRegs *p_Fpga)
{
    return ioread8(p_Fpga->sim_status) & FPGA_SIM_STAT_SIM_PRESENT;
}

//
// set SIM reset line On/Off
//      state == 0 - Reset line off
//      state == 1 - Reset line on
//
static inline
int fpga_ResetSim(struct FpgaRegs *p_Fpga, StateChange p_NewState)
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

#endif