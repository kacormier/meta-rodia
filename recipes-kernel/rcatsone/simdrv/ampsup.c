// Include support
#include <linux/version.h>
#include <linux/delay.h>    /* udelay */
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
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

// ----------------------------------------------------------------------------
// Definitions
// ----------------------------------------------------------------------------
#define SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS   0x20
#define SIMDRV_AMP_FPGA_UART_BASE_ADDRESS   0x30

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
          p_Fpga->sim_status = SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + SIM_STATUS(p_DeviceId);
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
#define SIM_STATUS_OFFSET     0xa   // 0x2a
#define SIM_RCON_CSR_OFFSET   0x9   // 0x29
#define SIM_RCON_ADDR_OFFSET  0x8   // 0x28
#define PH_SSR_OFFSET         0x4   // 0x24
#define PH_SCR_OFFSET         0x5   // 0x25
#define PH_IMR_OFFSET         0x6   // 0x26

void amp_GetFpgaRegisters(struct FpgaRegs *p_Fpga, int p_PhoneId)
{
    // Precompute phone/UART control/SIM FPGA registers
    p_Fpga->sim_status = (char *) (SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + SIM_STATUS_OFFSET);
    p_Fpga->sim_rcon_csr  = (char *) (SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + SIM_RCON_CSR_OFFSET);
    p_Fpga->sim_rcon_addr = (char *) (SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + SIM_RCON_ADDR_OFFSET);
    p_Fpga->ph_ssr = (char *) (SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + PH_SSR_OFFSET);
    p_Fpga->ph_scr = (char *) (SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + PH_SCR_OFFSET);
    p_Fpga->ph_imr = (char *) (SIMDRV_AMP_FPGA_CTRL_BASE_ADDRESS + PH_IMR_OFFSET);
    
    // Log mappings
    printk(KERN_ALERT "simdrv: phonesim%d: p_Fpga->sim_status 0x%x\n", p_PhoneId, (int) p_Fpga->sim_status);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Fpga->sim_rcon_csr 0x%x\n", p_PhoneId, (int) p_Fpga->sim_rcon_csr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Fpga->sim_rcon_addr 0x%x\n", p_PhoneId, (int) p_Fpga->sim_rcon_addr);            
    printk(KERN_ALERT "simdrv: phonesim%d: p_Fpga->ph_ssr 0x%x\n", p_PhoneId, (int) p_Fpga->ph_ssr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Fpga->ph_scr 0x%x\n", p_PhoneId, (int) p_Fpga->ph_scr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Fpga->ph_imr 0x%x\n", p_PhoneId, (int) p_Fpga->ph_imr);    
}


//
// function fills up struct UartRegs with FPGA adresses
//
void amp_GetUartRegisters(struct UartRegs *p_Uart, int p_PhoneId)
{
    int zeroPhoneId;
    
    // Each AMP has its own phone so only ever phone 0 for amp
    zeroPhoneId = 0;

    // All UART registers offset by UART_BASE

    // Precompute register addresses.
    p_Uart->rbr = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_RHR(zeroPhoneId) - UART_BASE);
    p_Uart->ier = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_IER(zeroPhoneId) - UART_BASE);
    p_Uart->iir = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_IIR(zeroPhoneId) - UART_BASE);
    p_Uart->fcr = p_Uart->iir;
    p_Uart->lcr = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_LCR(zeroPhoneId) - UART_BASE);
    p_Uart->lsr = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_LSR(zeroPhoneId) - UART_BASE);
    p_Uart->msr = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_MSR(zeroPhoneId) - UART_BASE);
    p_Uart->dll = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_DLLA(zeroPhoneId) - UART_BASE);
    p_Uart->dlh = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_DLHA(zeroPhoneId) - UART_BASE);
    p_Uart->spc = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_SPC(zeroPhoneId) - UART_BASE);

    p_Uart->t0ctr = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_T0_CTR(zeroPhoneId) - UART_BASE);
    p_Uart->rtl = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_RTL(zeroPhoneId) - UART_BASE);
    p_Uart->tfs = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_TFS(zeroPhoneId) - UART_BASE);
    p_Uart->tcd = (char *) (SIMDRV_AMP_FPGA_UART_BASE_ADDRESS + UART_TCD(zeroPhoneId) - UART_BASE);
    
    // Log mappings
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->rbr 0x%x\n", p_PhoneId, (int) p_Uart->rbr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->ier 0x%x\n", p_PhoneId, (int) p_Uart->ier);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->iir 0x%x\n", p_PhoneId, (int) p_Uart->iir);      
          
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->lcr 0x%x\n", p_PhoneId, (int) p_Uart->lcr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->lsr 0x%x\n", p_PhoneId, (int) p_Uart->lsr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->msr 0x%x\n", p_PhoneId, (int) p_Uart->msr);       
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->dll 0x%x\n", p_PhoneId, (int) p_Uart->dll);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->dlh 0x%x\n", p_PhoneId, (int) p_Uart->dlh);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->spc 0x%x\n", p_PhoneId, (int) p_Uart->spc);
    
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->t0ctr 0x%x\n", p_PhoneId, (int) p_Uart->t0ctr);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->rtl 0x%x\n", p_PhoneId, (int) p_Uart->rtl);       
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->tfs 0x%x\n", p_PhoneId, (int) p_Uart->tfs);
    printk(KERN_ALERT "simdrv: phonesim%d: p_Uart->tcd 0x%x\n", p_PhoneId, (int) p_Uart->tcd);      
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
// Get control device for AMP instance
//

#ifdef SIM_DEV_BOARD
// Define path to control interface
#define SIM_DRV_AMP_CTRL_PATH "/tmp/extusb"
#else
// Define path to control interface
#define SIM_DRV_AMP_CTRL_PATH "/var/run/dev/extusb"
#endif

char *
amp_getCtrlDevice(
  int thePhoneId)
{
 // Buffer to spped things up
  static char myFileNameBuffer[64];
  static char * myInstancePointer = NULL;
  char * myFileName = &myFileNameBuffer[0];
  char * myInsertionPointer = NULL;
  int theCtrlId = 0;

  // If buffer not primed...
  if (!myInstancePointer)
  {
    // Initialize
    *myFileName = 0;

    // Configure
    myInstancePointer = myFileName;
    myInstancePointer += sprintf(myInstancePointer, "%s", SIM_DRV_AMP_CTRL_PATH);
    sprintf(myInstancePointer, "X_ctrl");

    // myInstancePointer left pointing at X
  }

  // Set insertion pointer
  myInsertionPointer = myInstancePointer;

  // Add instance (phonesim 3,4,5 to extusb 0,1,2)
  theCtrlId = thePhoneId;
  theCtrlId -= 3;
  *myInsertionPointer++ = '0' + theCtrlId;

  // Return result
  return(myFileName);
}

uint8_t
amp_IsPhonePresent(
  int thePhoneId)
{
  mm_segment_t old_fs;
  struct file *file;
  uint8_t isPresent = 0;
  char * myControlDevice = 0;

  // Set kernel context
  old_fs = get_fs();
  set_fs(get_ds());

  // AMP presence is only known by presence of the ctrl device
  myControlDevice = amp_getCtrlDevice(thePhoneId);

  // Open control path
  file = filp_open(myControlDevice, O_RDONLY | O_NOCTTY | O_NONBLOCK, 0);

  // If errored...
  if (IS_ERR(file))
  {
    // Not present
    isPresent = 0;
  }
  else
  {
    // Is present
    isPresent = 1;

    // Close
    filp_close(file, 0);
  }

  // Restore context
  set_fs(old_fs);

  // Return result
  return(isPresent);
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

//
//  amp_ioread8
//
#define AMP_IO_BUFFER_SIZE  64

unsigned int
amp_ioread8(
  int thePhoneId,
  void *theAddress)
{
  // Buffer to spped things up
  static char myCommandBuffer[AMP_IO_BUFFER_SIZE];
  static char * myAddressPointer = NULL;
  extern struct mutex gUsbMutex;

  char * myInsertionPointer = NULL;
  size_t myLength = 0;
  char * myRequest = &myCommandBuffer[0];
  char * myResponse = NULL;
  int ret = 0;
  long theResult;
  char *myLeftoverPointer = 0;
  
  // Lock
  mutex_lock(&gUsbMutex);
  
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
  else
  {
    // Clear out trash
    memset(myAddressPointer,
           0,
           (AMP_IO_BUFFER_SIZE - (myAddressPointer - myRequest)));
  }

  // Set insertion pointer
  myInsertionPointer = myAddressPointer;

  // Format
  myInsertionPointer += sprintf(myInsertionPointer, "0x%x", (int) theAddress);

  // Determine length
  myLength = (myInsertionPointer - myRequest);

  // Place to look for response
  // FPGA|GET|<address>|value
  myResponse = myInsertionPointer;

  // Log
#ifdef SIM_DEBUG_TRACE
  printk(KERN_ALERT "simdrv: phonesim%d: %s", thePhoneId, myRequest);
#endif

  // Invoke USB call
  ret = usbio(thePhoneId, myRequest, myLength);

  // If outright failure...
  if (ret != 0)
  {
    // Stuff failed into request
    sprintf(myInsertionPointer, " (fail)\n");

    // Log for now
    printk(KERN_ALERT "simdrv: phonesim%d: %s", thePhoneId, myRequest);

    // Unlock
    mutex_unlock(&gUsbMutex);
  
    // Return result
    return(ret);
  }

  // If successful call...
  if (ret == 0)
  {
    // Pleace to look for response
    // FPGA|GET|<address>|value|OK\n
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
#ifdef SIM_DEV_BOARD
      // Convert to value
      ret = kstrtol(myResponse, 16, &theResult);

      // Set leftover pointer
      myLeftoverPointer = strchr(myResponse, '|');
#else
      // Convert to value
      theResult = simple_strtoull(myResponse, &myLeftoverPointer, 16);
#endif
    }

    // Advance response
    myResponse = myLeftoverPointer;

    // If delimiter not present...
    if (myResponse && *myResponse++ != RCTN_DELIM)
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

  // Log result
  if (ret == 0)
  {
#ifdef SIM_DEBUG_TRACE
    // Log (ok)
    printk(KERN_ALERT "simdrv: phonesim%d: %s (ok)",
          thePhoneId, myRequest);
#endif
  }
  else
  {
    // Log (fail)
    printk(KERN_ALERT "simdrv: phonesim%d: %s (fail)",
          thePhoneId, myRequest);
  }


  // Unlock
  mutex_unlock(&gUsbMutex);
    
  // Return result
  return(ret == 0 ? theResult : 0);
}

//
//  amp_iowrite8
//

int
amp_iowrite8(int thePhoneId, uint8_t theValue, void *theAddress)
{
  // Buffer to spped things up
  static char myCommandBuffer[AMP_IO_BUFFER_SIZE];
  static char * myAddressPointer = NULL;
  char * myInsertionPointer = NULL;
  size_t myLength = 0;
  char * myRequest = &myCommandBuffer[0];
  char * myResponse = NULL;
  int ret = 0;
  extern struct mutex gUsbMutex;
  
  // Lock
  mutex_lock(&gUsbMutex);
    
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
  else
  {
    // Clear out trash
    memset(myAddressPointer,
           0,
           (AMP_IO_BUFFER_SIZE - (myAddressPointer - myRequest)));
  }

  // Set insertion pointer
  myInsertionPointer = myAddressPointer;

  // Format
  myInsertionPointer +=
    sprintf(myInsertionPointer,
            "0x%x%c0x%x",
            (int) theAddress, RCTN_DELIM,
            theValue);

  // Determine length
  myLength = (myInsertionPointer - myRequest);

  // Log
#ifdef SIM_DEBUG_TRACE  
  printk(KERN_ALERT "simdrv: phonesim%d: %s", thePhoneId, myRequest);
#endif

  // Invoke USB call
  ret = usbio(thePhoneId, myRequest, myLength);

  // If outright failure...
  if (ret != 0)
  {
    // Stuff failed into request
    sprintf(myInsertionPointer, " (fail)\n");

    // Log for now
    printk(KERN_ALERT "simdrv: phonesim%d: %s", thePhoneId, myRequest);

    // Unlock
    mutex_unlock(&gUsbMutex);
    
    // Return result
    return(ret);
  }

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

  // Log result
  if (ret == 0)
  {
#ifdef SIM_DEBUG_TRACE
    // Log (ok)
    printk(KERN_ALERT "simdrv: phonesim%d: %s (ok)",
          thePhoneId, myRequest);
#endif
  }
  else
  {
    // Log (fail)
    printk(KERN_ALERT "simdrv: phonesim%d: %s (fail)",
          thePhoneId, myRequest);
  }

  // Unlock
  mutex_unlock(&gUsbMutex);
    
  // Return result
  return(ret);
}