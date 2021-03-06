/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
// FPGA_ph.c
//

// From FPGA's point of view, "phone" or "ph" index means SIM Link Slot index,
// or, if a SIM link doesn't exist, the cell phone index.  In either case, it is
// 0, 1 or 2 in the current probes.

//*****************************  INCLUDE FILES  ********************************

#include <linux/version.h>

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

// If not development board...
#ifndef SIM_DEV_BOARD
#include "nonvol.h"
#include "fpgaRegisterAccess.h"
#include <cproject.h>
#include <sstream>         // uses ostringstream
#include "fpga.h"
#include "fpga_amp.h"
#include "util.h"   // if XScale, RCATS util.h, else SIM Switch util.h
#include "fpga_serial.h"
#include "fpga_ph.h"
#else
#include "stubs.h"
// #include "nonvol.h"
#include "fpgaRegisterAccess.h"
// #include <cproject.h>
#include <sstream>         // uses ostringstream
#include "fpga.h"
#include "fpga_amp.h"
// #include "util.h"   // if XScale, RCATS util.h, else SIM Switch util.h
#include "fpga_serial.h"
#include "fpga_ph.h"
#endif

#define DEBUG(_x_) {}

using namespace std;

// External:
extern void phone_sim_assignment(unsigned char phone, char sim);

// AMP instances have ph=3,4,5
#define AMP_PHONE_MIN 3
#define AMP_PHONE_MAX 5

//***************************  GLOBAL VARIABLES  *******************************
const struct sim_control sim_control_struct[] =
{
  // Probe support
   { FPGA_LPH0_SIM, FPGA_LPH0_SCR, FPGA_LPH0_EVENT_IER, FPGA_RC0_ADR, FPGA_RC0_SCR },
   { FPGA_LPH1_SIM, FPGA_LPH1_SCR, FPGA_LPH1_EVENT_IER, FPGA_RC1_ADR, FPGA_RC1_SCR },
   { FPGA_LPH2_SIM, FPGA_LPH2_SCR, FPGA_LPH2_EVENT_IER, FPGA_RC2_ADR, FPGA_RC2_SCR },
  // AMP support
   { FPGA_AMP_PH_SIM, FPGA_AMP_PH_SCR, FPGA_AMP_PH_EVENT_IER, FPGA_AMP_RCON_ADR, FPGA_AMP_RCON_SCR },  
   { FPGA_AMP_PH_SIM, FPGA_AMP_PH_SCR, FPGA_AMP_PH_EVENT_IER, FPGA_AMP_RCON_ADR, FPGA_AMP_RCON_SCR },  
   { FPGA_AMP_PH_SIM, FPGA_AMP_PH_SCR, FPGA_AMP_PH_EVENT_IER, FPGA_AMP_RCON_ADR, FPGA_AMP_RCON_SCR },        
};

//*****************************  FPGA_BLD_NUM  *********************************
int fpga_bld_num(long *bldnum)
{
#if 0
   unsigned char msb, lsb;

   // Initialize the return data to a default value.
   *bldnum = 0;

   // Obtain the FPGA version/test registers
   if(peekFPGA(FPGA_VER_TEST_MSB, &msb) < 0)
      return -1;
   peekFPGA(FPGA_VER_TEST_LSB, &lsb);

   // If the contents of the FPGA version/test registers does not match a known
   // constants then assume that the FPGA image has not been down loaded.
   if(msb != VER_TEST_MSB_CODE || lsb != VER_TEST_LSB_CODE)
      return -4;

   peekFPGA(FPGA_BUILDM, &msb);
   peekFPGA(FPGA_BUILDL, &lsb);
   *bldnum = (msb<<8) | lsb;
#endif
   return 0;
}

void map_uart(unsigned char ph, char sim)
{
   updateBitsFPGA(ph, sim_control_struct[ph].remote_connection_status_control_register, FPGA_RC_SCR_CLK_EN, 0);
   pokeFPGA(ph, sim_control_struct[ph].remote_connection_adr_register, sim | FPGA_PH_SIM_ENABLE);
   updateBitsFPGA(ph, sim_control_struct[ph].remote_connection_status_control_register, FPGA_RC_SCR_CLK_EN, 1);
}


ResultCode fpga_map_phone_sim(unsigned char ph, char sim)
{
    // AMP-based instance?
    int isAmp = (ph >= AMP_PHONE_MIN && ph <= AMP_PHONE_MAX);
    
   // verify the parameters
   if (ph >= NUM_PHONES + NUM_AMP_PHONES)
   {
      ostringstream ostr;
      ostr << "invalid SIM link slot parameter " << int(ph);
      return MAKE_RCATS_ERROR(ERROR_INVALID_CMD_PARM, ostr.str());
   }
   if (sim < -NUM_SIM_UARTS || sim > NUM_SIM_SLOTS)
   {
      ostringstream ostr;
      ostr << "invalid SIM slot parameter " << int(sim);
      return MAKE_RCATS_ERROR(ERROR_INVALID_CMD_PARM, ostr.str());
   }

   // note assignment of sim to phone in PhoneSession structure
   phone_sim_assignment((isAmp ? (ph - 3) : ph), sim);

   // in all cases we disconnect first
   fpga_enable_phone_interrupts(ph, false);           // disable phone interrupts
   fpga_phone_sim_present(ph, FALSE);                   // de-assert SIM present line
   
   pokeFPGA(ph, sim_control_struct[ph].sim_selection_register, 0);      // disable local SIM selection
   pokeFPGA(ph, sim_control_struct[ph].remote_connection_adr_register, 0);  // map UART to nothing

   if (sim != 0) // going to connect either a SIM or a UART
   {
      // make sure the phone sees the SIM switch by dropping SIM present for at least 200 ms
      // (dge: assume above comment is wrong and it was suppose to say 20 ms)
      KS_delay(SELFTASK, 20/CLKTICK);

      if (sim > 0) { // connect to a SIM slot
         pokeFPGA(ph, sim_control_struct[ph].sim_selection_register, (sim-1) | FPGA_PH_SIM_ENABLE);
         fpga_phone_sim_present(ph, TRUE);
      } else // connect to a UART
      {
         map_uart(ph, -sim - 1);
         fpga_phone_sim_present(ph, TRUE);
         fpga_enable_phone_interrupts(ph, true);
      }
   }

   DEBUG(("SIM link slot %d connected to SIM %d", (int)ph, (int)sim));
   return ERROR_OK;
}

//
// fpga_phone_sim_present - set the SIM present signal when phone is mapped to a UART
//

char fpga_phone_sim_present(char ph, char enable)
{  
   ddebug2(DBG2_SIM_SIMU, ("SIM Link Slot %d: %s SIM present", ph, enable ? "set" : "unset"));
   return (char)updateBitsFPGA((int)ph, sim_control_struct[(int)ph].phone_status_control_register, FPGA_LPH_SCR_SIM_PRESENT, (int)enable);
}

