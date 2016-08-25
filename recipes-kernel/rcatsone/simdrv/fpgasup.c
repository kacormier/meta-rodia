// Include support
#include <asm/io.h>
#include <linux/delay.h>    /* udelay */
#include "fpga.h"
#include "simdrv.h"
#include "fpgasup.h" 

// Globals
uint8_t * fpga_mapped_address;

//
// attach/detach phone/SIM
//
int fpga_AttachDevice( struct FpgaRegs *p_Fpga,
                       uint16_t p_DeviceType, uint16_t p_DeviceId)
{
    switch( p_DeviceType)     // 1 - phone (0 to 2); 2 - SIM (0 to 3)
    {
        case DEVTYPE_NODEVICE:          // 0 - detach any device
            iowrite8(0, p_Fpga->sim_rcon_addr);
            iowrite8(0, p_Fpga->sim_rcon_csr);
            iowrite8(0x00, p_Fpga->ph_ssr);      // phone not connected to the SIM
            iowrite8(0x00, p_Fpga->ph_scr);      // SIM present
            //iowrite8(0x00, p_Dev->ph_imr);    // disable interrupts
        break;
        case DEVTYPE_PHONE:     // phone
            if ( p_DeviceId > 2 )
                return -EFAULT;                 // wrong device Id
            // set phone address
            iowrite8(0x80 | p_DeviceId, p_Fpga->sim_rcon_addr);
            // attach to phone
            iowrite8(0x00, p_Fpga->sim_rcon_csr);

            // phone is not connected to local SIM
            iowrite8(0x00, p_Fpga->ph_ssr);
            // tell to phone that SIM present
            iowrite8(0x01, p_Fpga->ph_scr);
            //iowrite8(0x03, p_Dev->ph_imr);    // enable interrupts
        break;
        case DEVTYPE_SIM:                   // attach SIM slot to UART
            if ( p_DeviceId > 3 )
                return -EFAULT;                 // wrong device Id
            // set SIM address
            iowrite8(0x80 | p_DeviceId, p_Fpga->sim_rcon_addr);
            // attach to SIM
            iowrite8(0x80,p_Fpga->sim_rcon_csr);
            // recalculate SIM status register address
            p_Fpga->sim_status = fpga_mapped_address + SIM_STATUS(p_DeviceId);
        break;
        case DEVTYPE_DIRECT:    // attach phone to SIM directly disconnec UART
            if ( p_DeviceId > 3 )
                return -EFAULT;         // wrong device Id
            // disconnect SIM from UART
            iowrite8(0x00, p_Fpga->sim_rcon_addr);
            //
            iowrite8(0x00,p_Fpga->sim_rcon_csr);

            // connect phone to the local SIM
            iowrite8(0x80 | p_DeviceId, p_Fpga->ph_ssr);
            // local SIM connected
            iowrite8(0x00, p_Fpga->ph_scr);
            //iowrite8(0x03, p_Dev->ph_imr);    // enable interrupts
        break;
        case DEVTYPE_MONITOR:
            if ( p_DeviceId > 2 )               // should be attache dto UART connected to the phone
                return -EFAULT;                 // wrong device Id
            // set SIM address
            iowrite8(0x80 | p_DeviceId, p_Fpga->sim_rcon_addr);
            // attach to SIM
            iowrite8(0x00,p_Fpga->sim_rcon_csr);
        break;
        default:
            return -EFAULT;             // device type is not allowed
    }
    return 0;
}

//
// enable / disable SIM clock
//      state == 0 - disable clock
//      state == 1 - enable clock
//
int fpga_ClockSim(struct FpgaRegs *p_Fpga, StateChange p_NewState)
{
    uint8_t     byRconCsr = ioread8(p_Fpga->sim_rcon_csr);

    if ( p_NewState == STATE_ON && ((byRconCsr & RCON_CSR_CLK) == 0) )
    {   // SIM clock is off - enable clock
        if ((byRconCsr & RCON_CSR_VCC) == 0)    // if power off => switch it on
        {       // power ON
            int retval;
            if ((retval = fpga_PowerSim(p_Fpga,STATE_ON)) != 0)
                return retval;
        }
        byRconCsr = ioread8(p_Fpga->sim_rcon_csr);
        // enable clock
        iowrite8((byRconCsr | RCON_CSR_CLK), p_Fpga->sim_rcon_csr);
        udelay(120);                // Wait at least 200 clock cycles for the SIM to tri-state the I/O line.

        // how can I check that clock is ON ????
        //???????
    }
    else if ( p_NewState == STATE_OFF && ((byRconCsr & RCON_CSR_CLK) != 0) )
    {       // SIM clock is on - disable clock
        iowrite8((byRconCsr & ~RCON_CSR_CLK), p_Fpga->sim_rcon_csr);
    }
    return 0;
}

//
// enable / disable SIM power (VCC)
//      state == 0 - power off
//      state == 1 - power on
//  The problem with power is that there are 3 VCC power levels :
//      1.8, 3.0, 5.0 volts - we should be smatrt about voltages - TODO
//  FOR NOW we always use class B - 3.0 volts
//

int fpga_PowerSim(struct FpgaRegs *p_Fpga, StateChange p_NewState)
{
    uint8_t     byRconCsr = ioread8(p_Fpga->sim_rcon_csr);

    if ( p_NewState == STATE_ON && ((byRconCsr & RCON_CSR_VCC) == 0) )
    {       // SIM power is off - switch it ON
        uint8_t     bySst;
        int         i = 0;
        iowrite8((byRconCsr | RCON_CSR_VCC3), p_Fpga->sim_rcon_csr);
        // busy wait untill VCC is ready

        for (i = 0; i < 10000; i++)
        {
            bySst = ioread8(p_Fpga->sim_status);
            if ( (bySst & SST_VCC_READY) != 0 )
                return 0;
            if ( (bySst & SST_VCC_FAULT) != 0 )
                return -EFAULT;
        }

        printk("=============VCC ON TIMEOUT====== bySst=0x%X\n", bySst);
        return -ETIMEDOUT;
    }
    else if ( p_NewState == STATE_OFF && ((byRconCsr & RCON_CSR_VCC) != 0) )
    {       // SIM power is on - switch it OFF
        byRconCsr &= ~RCON_CSR_RST;
        iowrite8(byRconCsr, p_Fpga->sim_rcon_csr);
        //usleep(240);              // Match the cold reset delay (400 cycles min).
        byRconCsr &= ~RCON_CSR_CLK;
        iowrite8(byRconCsr, p_Fpga->sim_rcon_csr);
        //usleep(20);
        iowrite8((byRconCsr & ~RCON_CSR_VCC), p_Fpga->sim_rcon_csr);
        //usleep(20000);            // A 10 mS minimum delay is required
                                    //  before applying another class.
    }
    return 0;
}

//
// function fills up struct UartRegs with FPGA adresses
//
void fpga_GetFpgaRegisters(int p_PhoneId, struct FpgaRegs *p_Fpga)
{
    // Precompute phone/UART control/SIM FPGA registers
    p_Fpga->sim_status = fpga_mapped_address + SIM_STATUS(p_PhoneId);

    p_Fpga->sim_rcon_csr  = fpga_mapped_address + RCON_SCR(p_PhoneId);
    p_Fpga->sim_rcon_addr = fpga_mapped_address + RCON_ADDR(p_PhoneId);

    p_Fpga->ph_ssr = fpga_mapped_address + LPH_SSR(p_PhoneId);
    p_Fpga->ph_scr = fpga_mapped_address + LPH_SCR(p_PhoneId);
    p_Fpga->ph_imr = fpga_mapped_address + LPH_IER(p_PhoneId);
}


//****************************  SSW_WARM_RESET  ********************************
// SSW_Warm_Reset():  Perform a warm reset of the SIM.  The SIM should generate
//  an ATR within 40,000 clock cycles.
//
int fpga_WarmResetSim(struct FpgaRegs *p_Fpga)
{
    uint8_t     byRconCsr ;
    // be shure that power and clock is ON
    int         retval = fpga_ClockSim(p_Fpga, STATE_ON);

    byRconCsr = ioread8(p_Fpga->sim_rcon_csr);
    iowrite8((byRconCsr & ~RCON_CSR_RST), p_Fpga->sim_rcon_csr);

    udelay(240);                // Match the cold reset delay (400 cycles min).
    iowrite8((byRconCsr | RCON_CSR_RST), p_Fpga->sim_rcon_csr);
    return retval;
}