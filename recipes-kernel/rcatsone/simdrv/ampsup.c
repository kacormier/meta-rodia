// Include support
#include <linux/version.h>
#include <linux/delay.h>    /* udelay */
#include "fpga.h"
#include "simdrv.h"
#include "fpgasup.h" 
#include "ampsup.h"
#include "rcats_msgs.h"
#include "usbsup.h"

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

// Globals
uint8_t * amp_mapped_address;

//
// attach/detach phone/SIM
//
int amp_AttachDevice(int thePhoneId, 
                     struct FpgaRegs *p_Fpga,
                     uint16_t p_DeviceType, 
                     uint16_t p_DeviceId)
{
  switch( p_DeviceType)     // 1 - phone (0 to 2); 2 - SIM (0 to 3)
  {
      case DEVTYPE_NODEVICE:          // 0 - detach any device
          amp_iowrite8(thePhoneId, 0, p_Fpga->sim_rcon_addr);
          amp_iowrite8(thePhoneId, 0, p_Fpga->sim_rcon_csr);
          amp_iowrite8(thePhoneId, 0x00, p_Fpga->ph_ssr);      // phone not connected to the SIM
          amp_iowrite8(thePhoneId, 0x00, p_Fpga->ph_scr);      // SIM present
          //amp_iowrite8(0x00, p_Dev->ph_imr);    // disable interrupts
      break;
      case DEVTYPE_PHONE:     // phone
          if ( p_DeviceId > 2 )
              return -EFAULT;                 // wrong device Id
          // set phone address
          amp_iowrite8(thePhoneId, 0x80 | p_DeviceId, p_Fpga->sim_rcon_addr);
          // attach to phone
          amp_iowrite8(thePhoneId, 0x00, p_Fpga->sim_rcon_csr);

          // phone is not connected to local SIM
          amp_iowrite8(thePhoneId, 0x00, p_Fpga->ph_ssr);
          // tell to phone that SIM present
          amp_iowrite8(thePhoneId, 0x01, p_Fpga->ph_scr);
          //amp_iowrite8(0x03, p_Dev->ph_imr);    // enable interrupts
      break;
      case DEVTYPE_SIM:                   // attach SIM slot to UART
          if ( p_DeviceId > 3 )
              return -EFAULT;                 // wrong device Id
          // set SIM address
          amp_iowrite8(thePhoneId, 0x80 | p_DeviceId, p_Fpga->sim_rcon_addr);
          // attach to SIM
          amp_iowrite8(thePhoneId, 0x80,p_Fpga->sim_rcon_csr);
          // recalculate SIM status register address
          p_Fpga->sim_status = amp_mapped_address + SIM_STATUS(p_DeviceId);
      break;
      case DEVTYPE_DIRECT:    // attach phone to SIM directly disconnec UART
          if ( p_DeviceId > 3 )
              return -EFAULT;         // wrong device Id
          // disconnect SIM from UART
          amp_iowrite8(thePhoneId, 0x00, p_Fpga->sim_rcon_addr);
          //
          amp_iowrite8(thePhoneId, 0x00,p_Fpga->sim_rcon_csr);

          // connect phone to the local SIM
          amp_iowrite8(thePhoneId, 0x80 | p_DeviceId, p_Fpga->ph_ssr);
          // local SIM connected
          amp_iowrite8(thePhoneId, 0x00, p_Fpga->ph_scr);
          //amp_iowrite8(0x03, p_Dev->ph_imr);    // enable interrupts
      break;
      case DEVTYPE_MONITOR:
          if ( p_DeviceId > 2 )               // should be attache dto UART connected to the phone
              return -EFAULT;                 // wrong device Id
          // set SIM address
          amp_iowrite8(thePhoneId, 0x80 | p_DeviceId, p_Fpga->sim_rcon_addr);
          // attach to SIM
          amp_iowrite8(thePhoneId, 0x00,p_Fpga->sim_rcon_csr);
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
int amp_ClockSim(int thePhoneId,
                 struct FpgaRegs *p_Fpga, 
                 StateChange p_NewState)
{
  uint8_t     byRconCsr = amp_ioread8(thePhoneId, p_Fpga->sim_rcon_csr);

  if ( p_NewState == STATE_ON && ((byRconCsr & RCON_CSR_CLK) == 0) )
  {   // SIM clock is off - enable clock
      if ((byRconCsr & RCON_CSR_VCC) == 0)    // if power off => switch it on
      {       // power ON
          int retval;
          if ((retval = amp_PowerSim(thePhoneId, p_Fpga,STATE_ON)) != 0)
              return retval;
      }
      byRconCsr = amp_ioread8(thePhoneId, p_Fpga->sim_rcon_csr);
      // enable clock
      amp_iowrite8(thePhoneId, (byRconCsr | RCON_CSR_CLK), p_Fpga->sim_rcon_csr);
      udelay(120);                // Wait at least 200 clock cycles for the SIM to tri-state the I/O line.

      // how can I check that clock is ON ????
      //???????
  }
  else if ( p_NewState == STATE_OFF && ((byRconCsr & RCON_CSR_CLK) != 0) )
  {       // SIM clock is on - disable clock
      amp_iowrite8(thePhoneId, (byRconCsr & ~RCON_CSR_CLK), p_Fpga->sim_rcon_csr);
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

int amp_PowerSim(int thePhoneId, 
                 struct FpgaRegs *p_Fpga, 
                 StateChange p_NewState)
{
/*    uint8_t     byRconCsr = amp_ioread8(p_Fpga->sim_rcon_csr);

    if ( p_NewState == STATE_ON && ((byRconCsr & RCON_CSR_VCC) == 0) )
    {       // SIM power is off - switch it ON
        uint8_t     bySst;
        int         i = 0;
        amp_iowrite8((byRconCsr | RCON_CSR_VCC3), p_Fpga->sim_rcon_csr);
        // busy wait untill VCC is ready

        for (i = 0; i < 10000; i++)
        {
            bySst = amp_ioread8(p_Fpga->sim_status);
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
        amp_iowrite8(byRconCsr, p_Fpga->sim_rcon_csr);
        //usleep(240);              // Match the cold reset delay (400 cycles min).
        byRconCsr &= ~RCON_CSR_CLK;
        amp_iowrite8(byRconCsr, p_Fpga->sim_rcon_csr);
        //usleep(20);
        amp_iowrite8((byRconCsr & ~RCON_CSR_VCC), p_Fpga->sim_rcon_csr);
        //usleep(20000);            // A 10 mS minimum delay is required
                                    //  before applying another class.
    }
    return 0;*/

    // TBD
  return(0);
}

//
// function fills up struct UartRegs with FPGA adresses
//
void amp_GetFpgaRegisters(struct FpgaRegs *p_Fpga, int p_PhoneId)
{
    // Each AMP has its own phone so only ever phone 0 for amp
    p_PhoneId = 0;
    
    // Precompute phone/UART control/SIM FPGA registers
    p_Fpga->sim_status = amp_mapped_address + SIM_STATUS(p_PhoneId);

    p_Fpga->sim_rcon_csr  = amp_mapped_address + RCON_SCR(p_PhoneId);
    p_Fpga->sim_rcon_addr = amp_mapped_address + RCON_ADDR(p_PhoneId);

    p_Fpga->ph_ssr = amp_mapped_address + LPH_SSR(p_PhoneId);
    p_Fpga->ph_scr = amp_mapped_address + LPH_SCR(p_PhoneId);
    p_Fpga->ph_imr = amp_mapped_address + LPH_IER(p_PhoneId);
}


//
// function fills up struct UartRegs with FPGA adresses
//
void amp_GetUartRegisters(struct UartRegs *p_Uart, int p_PhoneId)
{
    // Each AMP has its own phone so only ever phone 0 for amp
    p_PhoneId = 0;
    
    // Precompute register addresses.
    p_Uart->rbr = amp_mapped_address + UART_RHR(p_PhoneId);
    p_Uart->ier = amp_mapped_address + UART_IER(p_PhoneId);
    p_Uart->iir = amp_mapped_address + UART_IIR(p_PhoneId);
    p_Uart->fcr = p_Uart->iir;
    p_Uart->lcr = amp_mapped_address + UART_LCR(p_PhoneId);
    p_Uart->lsr = amp_mapped_address + UART_LSR(p_PhoneId);
    p_Uart->msr = amp_mapped_address + UART_MSR(p_PhoneId);
    p_Uart->dll = amp_mapped_address + UART_DLLA(p_PhoneId);
    p_Uart->dlh = amp_mapped_address + UART_DLHA(p_PhoneId);
    p_Uart->spc = amp_mapped_address + UART_SPC(p_PhoneId);

    p_Uart->t0ctr = amp_mapped_address + UART_T0_CTR(p_PhoneId);
    p_Uart->rtl = amp_mapped_address + UART_RTL(p_PhoneId);
    p_Uart->tfs = amp_mapped_address + UART_TFS(p_PhoneId);
    p_Uart->tcd = amp_mapped_address + UART_TCD(p_PhoneId);
}


//****************************  SSW_WARM_RESET  ********************************
// SSW_Warm_Reset():  Perform a warm reset of the SIM.  The SIM should generate
//  an ATR within 40,000 clock cycles.
//
int 
amp_WarmResetSim(
  int thePhoneId, 
  struct FpgaRegs *p_Fpga)
{
    uint8_t     byRconCsr ;
    // be shure that power and clock is ON
    int         retval = amp_ClockSim(thePhoneId, p_Fpga, STATE_ON);

    byRconCsr = amp_ioread8(thePhoneId, p_Fpga->sim_rcon_csr);
    amp_iowrite8(thePhoneId, (byRconCsr & ~RCON_CSR_RST), p_Fpga->sim_rcon_csr);

    udelay(240);                // Match the cold reset delay (400 cycles min).
    amp_iowrite8(thePhoneId, (byRconCsr | RCON_CSR_RST), p_Fpga->sim_rcon_csr);
    return retval;
}

//
//  low level Status Byte functions
//  return == 0 - SIM not present;
//         != 0 - SIM present
//
uint8_t 
amp_IsSimPresent(
  int thePhoneId,
  struct FpgaRegs *p_Fpga)
{
    return amp_ioread8(thePhoneId, p_Fpga->sim_status) & FPGA_SIM_STAT_SIM_PRESENT;
}
//
uint8_t 
amp_IsSimReady(
  int thePhoneId,
  struct FpgaRegs *p_Fpga)
{
    return amp_ioread8(thePhoneId, p_Fpga->sim_status) & FPGA_SIM_STAT_SIM_PRESENT;
}

//
// set SIM reset line On/Off
//      state == 0 - Reset line off
//      state == 1 - Reset line on
//
int 
amp_ResetSim(
  int thePhoneId,
  struct FpgaRegs *p_Fpga, 
  StateChange p_NewState)
{
    uint8_t     byRconCsr = ioread8(p_Fpga->sim_rcon_csr);

    if ( p_NewState == STATE_ON && ((byRconCsr & RCON_CSR_RST) == 0) )
    {   // Reset line off - Set it on
        if ((byRconCsr & RCON_CSR_CLK) == 0)    // if clock disable => enable the clock first
        {       // enable clock
            int retval = 0;
            if ((retval = amp_ClockSim(thePhoneId, p_Fpga,STATE_ON)) != 0)
                return retval;
        }
        // set reset line
        byRconCsr = amp_ioread8(thePhoneId, p_Fpga->sim_rcon_csr);
        amp_iowrite8(thePhoneId, (byRconCsr | RCON_CSR_RST), p_Fpga->sim_rcon_csr);
        // how can I check that Reset line is set ????
        //???????
    }
    else if ( p_NewState == STATE_OFF && ((byRconCsr & RCON_CSR_RST) != 0) )
    {       // clear reset line
        amp_iowrite8(thePhoneId, (byRconCsr & ~RCON_CSR_RST), p_Fpga->sim_rcon_csr);
    }
    return 0;
}

unsigned int 
amp_ioread8(
  int thePhoneId,
  void *theAddress)
{
  // Buffer to spped things up
  static char myCommandBuffer[64];
  static char * myAddressPointer = NULL;
  
  char * myInsertionPointer = NULL;
  size_t myLength = 0;
  char * myRequest = &myCommandBuffer[0];
  char * myResponse = NULL;
  int ret = 0;
  long theResult;
#ifndef SIM_DEV_BOARD      
  char *myLeftoverPointer = 0;
#endif
  
  // If buffer not primed...
  if (!myAddressPointer)
  {
    // Initialize
    *myRequest = 0;
        
    // Configure
    myAddressPointer = myRequest;
    myAddressPointer += sprintf(myAddressPointer, "%s", RCTN_CMD_FPGA);
    *myAddressPointer++ = RCTN_DELIM;
    myAddressPointer += sprintf(myAddressPointer, "%s", RCTN_CMD_GET);
    *myAddressPointer++ = RCTN_DELIM;  
    *myAddressPointer = 0;
  }
  
  // Set insertion pointer
  myInsertionPointer = myAddressPointer;
  
  // Format
  myInsertionPointer += sprintf(myInsertionPointer, "%0x%x%c", (int) theAddress, RCTN_TERM);
  
  // Determine length
  myLength = (myInsertionPointer - myRequest) + 1;
  
  // Place to look for response
  // FPGA|GET|<address>|value
  myResponse = myInsertionPointer;
  
  // Log for now (already terminated)
  printk(KERN_ALERT "simdrv: phonesim%d: %s", thePhoneId, myRequest);
  
  // Invoke USB call
  ret = usbio(thePhoneId, myRequest, myLength);
  
  // If successful call...
  if (ret == 0)
  {    
    // Pleace to look for response
    // FPGA|GET|<address>|OK|value\n
    // -------------------^
    myResponse = myInsertionPointer;
  
    // If delimiter not present...
    if (*myResponse++ != RCTN_DELIM)
    {
      // Failed
      ret = 1;
    }
    
    // If okay so far...
    if (ret == 0)
    {
      // If not matched OK...
      if (strncmp(myResponse, 
                  RCTN_RESULT_SUCCESS, 
                  strlen(RCTN_RESULT_SUCCESS)) != 0)
      {
        // Failed
        ret = 1;        
      }
    }
    
    // If okay so far...
    if (ret == 0)
    {
#ifdef SIM_DEV_BOARD      
      // Convert to value
      ret = kstrtol(myResponse, 16, &theResult);
#else
      // Convert to value
      theResult = simple_strtoull(myResponse, &myLeftoverPointer, 16);
#endif
    }
  }

  // If failed...
  if (ret != 0)
  {
    // Log for now (already terminated)
    printk(KERN_ALERT "simdrv: phonesim%d: failed %s", thePhoneId, myRequest);
  }    
  
  // Return result
  return(ret == 0 ? theResult : 0);
}
  
int
amp_iowrite8(int thePhoneId, uint8_t theValue, void *theAddress)
{
  // Buffer to spped things up
  static char myCommandBuffer[64];
  static char * myAddressPointer = NULL;
  char * myInsertionPointer = NULL;
  size_t myLength = 0;
  char * myRequest = &myCommandBuffer[0];
  char * myResponse = NULL;  
  int ret = 0;

  // If buffer not primed...
  if (!myAddressPointer)
  {
    // Initialize
    *myRequest = 0;
      
    // Configure
    myAddressPointer = myRequest;
    myAddressPointer += sprintf(myAddressPointer, "%s", RCTN_CMD_FPGA);
    *myAddressPointer++ = RCTN_DELIM;
    myAddressPointer += sprintf(myAddressPointer, "%s", RCTN_CMD_SET);
    *myAddressPointer++ = RCTN_DELIM;  
    *myAddressPointer = 0;
  }
  
  // Set insertion pointer
  myInsertionPointer = myAddressPointer;
  
  // Format
  myInsertionPointer += 
    sprintf(myInsertionPointer, 
            "0x%x%c0x%x%c", 
            (int) theAddress, RCTN_DELIM,
            theValue, RCTN_TERM);
  
  // Determine length
  myLength = (myInsertionPointer - myRequest) + 1;
  
  // Log for now (already terminated)
  printk(KERN_ALERT "simdrv: phonesim%d: %s", thePhoneId, myRequest);
  
  // Invoke USB call
  ret = usbio(thePhoneId, myRequest, myLength);
  
  // If successful call...
  if (ret == 0)
  {    
    // Pleace to look for response
    // FPGA|SET|<address>|<value>|OK\n
    // --------------------------^
    myResponse = myInsertionPointer;
  
    // If delimiter not present...
    if (*myResponse++ != RCTN_DELIM)
    {
      // Failed
      ret = 1;
    }
    
    // If okay so far...
    if (ret == 0)
    {
      // If not matched OK...
      if (strncmp(myResponse, 
                  RCTN_RESULT_SUCCESS, 
                  strlen(RCTN_RESULT_SUCCESS)) != 0)
      {
        // Failed
        ret = 1;        
      }
    }
  }
  
  // If failed...
  if (ret != 0)
  {
    // Log for now (already terminated)
    printk(KERN_ALERT "simdrv: phonesim%d: failed %s", thePhoneId, myRequest);
  }    
  
  // Return result
  return(ret);  
}