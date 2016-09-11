/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
// FPGA_SERIAL.C - support code for the UARTs in the FPGA
//

// From FPGA's point of view, "phone" or "ph" index means SIM Link Slot index,
// or, if a SIM link doesn't exist, the cell phone index.  In either case, it is
// 0, 1 or 2 in the current probes.

#include <linux/version.h>

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

// If not development board...
#ifndef SIM_DEV_BOARD
#include "fpga.h"
#include "fpga_serial.h"

#include "util.h"
#include "phone.h" // ph_Phone
#include "fpga_ph.h"
#include "fpgaRegisterAccess.h"
#else
#include "stubs.h"
#include "fpga.h"
#include "fpga_serial.h"

// #include "util.h"
// #include "phone.h" // ph_Phone
#include "fpga_ph.h"
#include "fpgaRegisterAccess.h"
#endif

//extern "C" {
   //extern const struct sim_control sim_control_struct[NUM_T0_PORTS];
//}

// #define FOSC                           33330000.0   // Processor clock oscillator frequency.


void fpga_enable_phone_interrupts(unsigned char ph, bool enable)
{
   ddebug2(DBG2_SIM_SIMU, ("%s interrupts for SIM slot link %d", enable ? "Enable" : "Disable", ph));
   updateBitsFPGA(ph, sim_control_struct[ph].phone_interrupt_enable_register, FPGA_LPH_VCC_INT_EN | FPGA_LPH_RST_INT_EN, enable);
   // updateBitsFPGA(FPGA_UART_IER(ph), IER_RX_INT_ENABLE /* | IER_ERR_INT_ENABLE */, enable); N.B.: now done in simdrvmain.c
   pokeFPGA(ph, FPGA_UART_IIR(ph), ((enable) ? IIR_RCV_FIFO_ENABLE : 0));
}


