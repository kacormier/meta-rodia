#ifndef CASA_FPGA_PH_H__
#define CASA_FPGA_PH_H__
/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
// FPGA_Ph.h
//

#include <linux/version.h>

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

// If not development board...
#ifndef SIM_DEV_BOARD
#include "ResultCode.h"
#endif
#include "fpga.h"

#define VER_TEST_MSB_CODE   0x01
#if defined(PHOENIX_PROBE)
#define VER_TEST_LSB_CODE   0x01
#else
#define VER_TEST_LSB_CODE   0x00
#endif

struct sim_control
{
    unsigned int sim_selection_register;
    unsigned int phone_status_control_register;
    unsigned int phone_interrupt_enable_register;
    unsigned int remote_connection_adr_register;
    unsigned int remote_connection_status_control_register;
};

//**************************  FUNCTION PROTOTYPES  *****************************
// Public:
int  fpga_bld_num(long *bldnum);

/**
Maps a phone to a SIM or a UART.

@arg ph - the phone index (0 - 2) [i.e., 0 - (NUM_PHONES - 1)]
@arg sim - the sim index, (0, 1 - 4 for the SIM slots where 0 means disconnected) and (-1 - -3) for the UARTS
@returns ERROR_OK on success.
*/
ResultCode fpga_map_phone_sim(unsigned char ph, char sim);
char fpga_phone_sim_present(char ph, char enable);

//********************************  MACROS  ************************************

// shared control structure (what does this do?)
extern const struct sim_control sim_control_struct[];

#endif      // #ifndef CASA_FPGA_PH_H__
