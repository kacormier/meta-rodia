/*
 * Copyright (c) 2005 Casabyte, Inc.  All Rights Reserved
 */


#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

//#define SIM_DEBUG_TRACE
//#define SIM_DEBUG_INTERRUPT_TRACE
//#define SIM_DEBUG_PHONE_INTERRUPT_TRACE

#define MINIMUM_FPGA_BUILD_VERSION      (6)
#define DRIVER_VERSION_STRING   "SimDrv ver 5.000"
#define DRIVER_VERSION_INT      (5000)      //5.000

#include <linux/version.h>

// Option for dev board handling

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

// Enable AMP support (or not)
#define SIM_DEV_ENABLE_AMP_SUPPORT
#define SIM_DEV_ENABLE_WORKAROUNDS

#ifndef KERNEL_VERSION /* pre-2.1.90 didn't have it */
#  define KERNEL_VERSION(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0) /* not < 2.4 */
#  error "This kernel is too old: not supported by this file"
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
#ifndef SIM_DEV_BOARD
#  error "This kernel is too recent: not supported by this file"
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#  define LINUX_24
#else                   //LINUX_VERSION_CODE < KERNEL_VERSION(2,7,0)
#  define LINUX_26
#endif

#include <linux/module.h>   // MOD_INC_USE_COUNT
#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>    /* O_ACCMODE */
#include <linux/init.h>     /* module_init and module_exit */
#include <linux/delay.h>    /* udelay */
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/timer.h>    /* init_timer, add_timer etc. */
#include <linux/ioport.h>
#include <linux/interrupt.h>    // from 2.6.16
#include <asm/atomic.h>     /* atomic integer operations */
#include <linux/semaphore.h>  /* semaphores */
#include <asm/uaccess.h>    /* access_ok */
#include <asm/system.h>     /* cli(), *_flags */
#include <asm/io.h>     /* ioremap, iounmap */
#include <asm/byteorder.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/sched.h>
// obsolete in ver 2.6
#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
#define EXPORT_NO_SYMBOLS

#include "simdrv.h"
#include "circ_buffer.h"
#include "fpga.h"       // NUM_PHONES, FPGA Register Map for Phone-SIM interface
#include "fpgasup.h"
#include "ampsup.h"

/*=======================INITIALIZATION STEPS DEFINITIONS==================
 */
enum    {
    None,
    Mem,
    IOmap,
    RegisterMajor,
    MajorDev,
    FPGA,
    RxTasklet,
    IRQ,
    Statistic
}           DriverState;
/*=======================HARDWARE REGISTER DEFINITIONS==================
 */
// Phone IER bit masks:(IMR)
#define PH_IMR_IRQ_DISABLE  0x00
#define PH_IMR_VCC_IRQ_EN   0x01
#define PH_IMR_RST_IRQ_EN   0x02

// UART IER bit masks:
#define IER_IRQ_DISABLE  0x00
#define IER_RX_IRQ_EN    0x01
#define IER_TX_IRQ_EN    0x02
#define IER_LS_IRQ_EN    0x04
#define IER_MSR_IRQ_EN   0x08
#define IER_GLBL_IRQ_EN  0x80       // new UART - 0 - all UART int disabled
                                    //            1 - all UART int enabled
// UART IIR (read only) bit masks:
#define IIR_MSR_IRQ      0x00
#define IIR_NO_IRQ       0x01
#define IIR_TX_IRQ       0x02
#define IIR_RX_IRQ       0x04
#define IIR_LS_IRQ       0x06
#define IIR_RX_TIME_IRQ  0x0C

// UART FCR (write only) bit masks:
#define FCR_FIFO_ENABLE  0x01
#define FCR_RX_FIFO_RST  0x02
#define FCR_TX_FIFO_RST  0x04

#define UART_FREQ           (33333333L)
#define SIM_FREQ            (3333333)
#define DEFAUL_BAUD_RATE    (8737)

/*=======================SOFTWARE PARAMETERS DEFINITIONS===========
 */
//#define READ_BUF_SIZE       (500)
#define IRQ_BUF_SIZE        (512)   // buffer size should be power of 2
#define IOCTL_BUF_SIZE      (2048)  // buffer size should be power of 2
#define MAX_CMD_SIZE        (260)   /* maximum length of the CMD/Response data block */
#define MAX_LOGGED_CMDS     (10)    /* maximun phone command buffer reads to log. */

#define READ_TIMEOUT_MS     (70000) /* default if not set as module parameter */

/* Vasily Kaliniouk found in Section 13.5.1 (pp. 403-404) of IXP4XX_IXC1100_Developer_Man_252480-3.pdf
 * a table that shows the source of Int21 is GPIO[4], so that is the interrupt number that must be
 * used by this driver to process FPGA SSW interrupts
 */
#define FPGA_INTERRUPT_NUMBER   (10)    /* default if not set as module parameter */
#define SIMPHONE_MAJOR_DEV_NUM  (209)
#define SIMPHONE_MINOR_DEV_NUM  (0)

#define ATR_BUFFER_SIZE (50)

// SIM thread activity status
#define SIM_THREAD_STOP         (0)
#define SIM_THREAD_ACTIVE       (1)

#define SIM_IO_STOP             (0)
#define SIM_IO_ACTIVE           (1)

//m_T0Mode values
#define T0_WAIT_COMMAND         (0)
#define T0_WAIT_RESPONSE        (1)

//FileStruct.m_MonitorFile allowed values
#define EXCLUSIVE_ACCESS_OFF    (0)
#define EXCLUSIVE_ACCESS_ON     (1)

// From Phone Buffer Values
#define BUF_VALUE_PART(_x_)     (uint8_t)(_x_ & 0xFF)
#define BUF_EVENT_PART(_x_)     (_x_ & 0xFF00)
#define BUF_LINE_STATUS_EVENT   (LINE_STATUS_EVENT<<8)
#define BUF_SIM_RESET_EVENT     (SIM_RESET_EVENT<<8)
#define BUF_POWER_ON_EVENT      (POWER_ON_EVENT<<8)
#define BUF_POWER_OFF_EVENT     (POWER_OFF_EVENT<<8)
#define BUF_POWER_FAULT_EVENT   (POWER_FAULT_EVENT<<8)
// Find new from-phone-buffer pointer with buffer wrap handled using module (%)
#define BUF_PTR(_BUF, _newptr)  (_BUF->Base + ((_newptr - _BUF->Base) % (_BUF->Extent - _BUF->Base)))

/*=======================CONTROL STRUCTURES DEFINITIONS=================
 */

typedef struct T0Parser
{
    int         nExpectedLength;    // command header without a Length field
    int         nNextTdIndex;
    int         m_TckInAtr;         // if != 0 ATR needs TCK byte
// next 2 variables was added to switch to MI server to support ATR
    int         m_nHistLen;
    int         m_nTckPresent;      // if > 0 - TCK read TCK byte after histor..
    uint8_t     m_Event;            // space for event if event happened during parsing
} T0Parser;

// next enum and structure control TX buffer
typedef enum TxDataType
{
    TX_ATR,             // send ATR
    TX_COMMAND          // send regular command/response with or without baudrate change
} TxDataType;

typedef struct TxBuffer
{
    struct semaphore    m_TxSemaphore;      // buffer access guard
    int         m_nTxPtr;                   // tells IRQ where to take next byte to send
    uint16_t    m_BaudRateDivisor;          // new baudrate after sent is done
    TxDataType  m_DataType;                 // ATR or Regular command
    uint8_t     m_Buffer[CMD_BUFFER_SIZE];  // transmit buffer
} TxBuffer;

typedef struct tagCmdRead
{
    int         m_len;                    // command length
    uint8_t     m_Buffer[MAX_CMD_SIZE];   // command bytes read
} CmdRead;

typedef
struct  phoneSIMStruct
{
    uint8_t *       ATRptr;
    uint8_t         ATRsize;
    uint16_t        BogusInterrupts;
    uint16_t        SpuriousUartInterrupts;
    uint16_t        SpuriousPhoneInterrupts;

    // T=0 UART ISR related variables:
    //wait_queue_head_t   tx_wake_queue;      // UART Tx event: ready to send or error
    //wait_queue_head_t   rx_wake_queue;
    wait_queue_head_t   ioctl_wake_queue;   // read data is ready for ioctl level

    UartRegs    m_Uart;
    FpgaRegs    m_Fpga;

    // ----- statistics data
    uint16_t ioctl_cnt_loss;        // ioctl circular buffer overflow counter
    uint16_t bogus_uart_irq;
    uint16_t spurious_uart_irq;
    uint16_t lsr_error_cnt_irq;
    uint16_t tx_error_cnt_irq;
    uint16_t spurious_phone_irq;
    uint32_t tx_cnt;                // transmited bytes
    uint32_t rx_cnt;                // received bytes
    uint16_t rx_cnt_loss;           // Lost character count due to a full
                                    // circular buffer.
    // ----- flags
    uint8_t  tx_err_flg;            // Set when the transmitter is unable to
                                    // successfully transmit a character.

    int         m_PhoneId;                  /* phone ID - information field */
    struct phoneSIMStruct  *m_PhoneForCopyEvents;  /* for monitoring - copy event from this phone */
                                            /* NULL - if no copy requested erased when file is closed*/
    int         m_EventSrc;
    int         m_RxThreadStatus;
    int         m_TxStatus;                 /* UART open for Tx */
    int         m_RxStatus;                 /* UART open for Rx */
    struct tasklet_struct   m_RxTasklet;    /* IRQ bottom half */

    CircBuffer  m_RxBuffer;                 /* receive buffer transfers received data and events to */
                                            /* the processing thread - initiated by open */
    CircBuffer  m_IoctlBuffer;              /* Rx proc. thread put processed data to this buffer */
                                            /* ioctl will pickup data from here          */
    atomic_t    m_PhoneTxTrigger;           /* Tx interrupt will set this flag to 0      */
                                            /* when there is no more bytes to send and   */
                                            /* Tx interrupt is disabled. And to 1 all    */
                                            /* time. Used together with tx_wake_queue    */
    atomic_t    m_EventCnt;                 /* event counter to wake up blocking ioctl   */
    int8_t      m_CommandMap[256];          /* each element of the array corresponds to one SIM command */
                                            /* stored value corresponds to type of the command */
                                            /*                          or SIMCMD_NOT_DEFINED  */

    uint8_t     m_LastClass;                /* in "Waiting Response" mode - save ACK byte */
    uint8_t     m_LastInstruction;          /* in "Waiting Response" mode - save ACK byte */
    uint16_t    m_T0Mode;                   /* T=0 mode - could be wait command and wait response */
    uint16_t    m_SendNull;                 /* 1 - send T=0 NULL in Waiting response mode; 0 - do NOT send. */
    uint16_t    m_AckMode;                  /* 1 - send ACK before response; 0 - do NOT send. */
    struct timer_list   m_SendNullTimer;    /* timer - for sending T=0 NULL */

#ifdef SEND_DEBUG_MESSAGES
    //  debug data
    int                 m_DebugOn;          // 1 - send debug messages
    struct timeval      m_ResetTime;        // time of the last reset
    struct timeval      m_LastTime;
    long                m_AtrFirstDelta;    // time between reset and send first ATR byte
    long                m_AtrDelta;         // longest time between bytes in ATR
    char                m_DebugMsg[DEBUG_BIF_SIZE]; // debug message buffer for the UART
#endif
    struct TxBuffer     m_TxBuffer;         // transmit buffer structure
    spinlock_t          m_ConfigLock;       // protects UART configuration operations

    struct semaphore    m_RxSemaphore;
    int                 m_ExclusiveFlag;    // 0 - all opened files share UART
                                            // 1 - UART is used exclusevly by THIS file
// timer to simulate Insert/Remove interrupts
    uint8_t             m_SimPresent;
// moved from tequal0 driver
    volatile uint32_t   m_ActualIrEvents;
    volatile uint32_t   m_AcknowledgedIrEvents;
    int                 m_NeedsReset;       // to protect against transaction executing in case of some errors
    struct fasync_struct *  NotifyList;     // is used in fasync
    struct SimConfigUart    m_DefaultUartConfig;  // default UART configuration

    // CmdReadLog is a circular array holding most recent phone commands read
    // that are dumped when a unexpected class byte is detected. A mod of
    // m_numCmdRead identifies which array entry to store the next command in.
    struct tagCmdRead   m_CmdReadLog[MAX_LOGGED_CMDS];
    uint32_t            m_numCmdReads;       // number of phone command buffer reads


}   phoneSIMStruct;


typedef struct  FileStruct
{
    phoneSIMStruct      *m_pUartStruct;
    int                 m_ExclusiveAccess;  // 0 - regular file; 1 - monitor file
} FileStruct;


/*
 *  NOTE: pointers uart_rbr, uart_iir, uart_lsr, uart_msr, ph_scr and some other
 *        do not have volatile modifier, as they should, because Linux
 *        ioread8/iowrite8 moacros do the volatile casting for us
 *
 *  #define ioread8(p)      ({ unsigned int __v = __raw_readb(p); __v; })
 *  #define __raw_readb(a)  (__chk_io_ptr(a), *(volatile unsigned char __force  *)(a))
 */

/*=========================== AMP SUPPORT DEFINITIONS =================
 */

#ifdef SIM_DEV_ENABLE_AMP_SUPPORT
#define NUM_AMPS  3   // Max per probe
#else
#define NUM_AMPS  0   // No support
#endif

/*=======================GLOBAL VARIABLES==========================
 * Global Data (static not needed because EXPORT_NO_SYMBOLS is used)
 */

// Devices include probe + AMP
#define NUM_DEVICES  (NUM_PHONES + NUM_AMPS)

uint32_t        g_FpgaVersion = 0;
uint32_t        read_timeout_ms = READ_TIMEOUT_MS;
int32_t         fpga_irq = FPGA_INTERRUPT_NUMBER;
uint32_t        character_retransmit = 3;
int32_t         simdrv_major = SIMPHONE_MAJOR_DEV_NUM;
char *          simdrv_dev = "simphone";


struct cdev         simdrv_cdev;
struct class_simple *sim_class;         //Linux 2.6 class_simple Interface

phoneSIMStruct      phoneSIMData[NUM_DEVICES];
int32_t             g_uartDivisor[NUM_DEVICES]; // Current UART Divisor for identifying baud change.
uint8_t             ATR[NUM_DEVICES][ATR_BUFFER_SIZE];

// Precompute register addresses.
uint8_t *ssw_irq_reg = NULL;

// timer to simulate Insert/Remove interrupts
struct timer_list   timer_IrCheck;

/*===================WORK QUEUE SUPPORT==========================
 */
struct work_cont {
	struct work_struct real_work;
	int arg;
} work_cont;

// Our work queue functions
static void simdrv_TimerFunctionWorker(struct work_struct *work);

// Our work queues
struct work_cont simdrv_timer_wq;


uint8_t     g_LsrTxError;       // different UART type have this error bit in different places
//  function pointers ("virtual functions)
void (*simdrv_SendAtr)              (phoneSIMStruct * p_Dev);
int  (*simdrv_SendResponse)         (phoneSIMStruct * p_Dev, uint8_t *p_strBuf,
                                     int p_nBufLen, uint16_t p_BaudRateDivisor );
void (*simdrv_ClearRx)              (phoneSIMStruct *p_Dev);
int  (*simdrv_CalculateDivisor)     (int p_nBaudRate);
int  (*simdrv_DivisorToBaudrate)    (int p_nDivisor);

/*=========================== FUNCTIONS DEFINITIONS=====================
 */
static int simdrv_ReadRawDataBlock(phoneSIMStruct * p_Dev, BufferElement *p_strBuf, int p_nLen);
static int simdrv_ReadPhoneData(phoneSIMStruct * p_Dev, BufferElement *p_strBuf, int p_nCmdLen);
static int simdrv_ReadData(phoneSIMStruct * p_Dev, uint8_t *p_Buff, int p_nReadLen);
static inline uint8_t simdrv_translateEvent(phoneSIMStruct * p_Dev, BufferElement *p_usValue);
static int simdrv_UartConfig(phoneSIMStruct *p_Dev, struct SimConfigUart *p_UartConfig);
static int simdrv_ReadCommand(phoneSIMStruct * p_Dev, uint8_t *p_ResBuf, uint8_t p_byCmdSz);
static int simdrv_ReadCommandFull(phoneSIMStruct * p_Dev, struct SimCommand *p_Command);
static int simdrv_ReadMonitor(phoneSIMStruct * p_Dev, struct SimMonitor *p_Command);
static void simdrv_SendNull(unsigned long ptr);
static uint16_t simdrv_IsEvent(phoneSIMStruct *p_Dev, BufferElement *p_strBuf, int p_nBufCnt);
static inline int simdrv_Atr( phoneSIMStruct *p_Dev, struct SimCommand *p_Command,
                               struct T0Parser *p_Parser);
static inline int simdrv_Pps( phoneSIMStruct *p_Dev, struct SimCommand *p_Command,
                               struct T0Parser *p_Parser);

static int simdrv_init_phone(int ph);

#ifdef SEND_DEBUG_MESSAGES
    static inline void simdrv_AddDebugMsg(phoneSIMStruct * p_Dev, struct SimCommand *p_Command);
#endif
static void simdrv_SetUartDivisor(phoneSIMStruct *p_Dev, int32_t p_nDivisor);
static inline int  simdrv_FillTxFifoIrq(phoneSIMStruct * p_Dev);

// Device-agnostic dispatch routines
int dev_AttachDevice(phoneSIMStruct *p_Dev, uint16_t p_DeviceType, uint16_t p_DeviceId);
int dev_ClockSim(phoneSIMStruct *p_Dev, StateChange p_NewState);
int dev_PowerSim (phoneSIMStruct *p_Dev, StateChange p_NewState);
void dev_GetFpgaRegisters(phoneSIMStruct *p_Dev, int p_PhoneId);
void dev_GetUartRegisters(phoneSIMStruct *p_Dev, int p_PhoneId);
int dev_WarmResetSim(phoneSIMStruct *p_Dev);
uint8_t dev_IsSimPresent(phoneSIMStruct *p_Dev);
uint8_t dev_IsSimReady(phoneSIMStruct *p_Dev);
uint8_t dev_IsPhonePresent(int p_PhoneId);
unsigned int dev_ioread8(phoneSIMStruct *p_Dev, void *theAddress);
void dev_iowrite8(phoneSIMStruct *p_Dev, uint8_t theValue, void *theAddress);

/*==============================FUNCTIONS===============================
 */
/*==============================DEBUG===================================
 */
//#ifdef SIM_DEBUG_TRACE
void simdrv_DebugPrint(const char *p_pszLabel, int p_nPhoneId, uint8_t *p_Data, int p_DataLen)
{
    int i;
    printk("%s Ph%d. Len = %d: ", p_pszLabel, p_nPhoneId, p_DataLen);
    for ( i=0; i < p_DataLen; i++)
      printk("%2.2X ", p_Data[i]);
    printk("\n");
}
//
// debug function
//
void simdrv_DumpUartRegs(phoneSIMStruct *p_Dev)
{
printk("RBR=0x%2.2X; IER=0x%2.2X; IIR=0x%2.2X; LCR=0x%2.2X; MCR=0x??\n",
        dev_ioread8(p_Dev, p_Dev->m_Uart.rbr), dev_ioread8(p_Dev, p_Dev->m_Uart.ier),
        dev_ioread8(p_Dev, p_Dev->m_Uart.iir), dev_ioread8(p_Dev, p_Dev->m_Uart.lcr));
printk("LSR=0x%2.2X; MSR=0x%2.2X; SCR=0x00; DLL=0x%2.2X; DLH=0x%2.2X\n",
        dev_ioread8(p_Dev, p_Dev->m_Uart.lsr), dev_ioread8(p_Dev, p_Dev->m_Uart.msr),
        dev_ioread8(p_Dev, p_Dev->m_Uart.dll), dev_ioread8(p_Dev, p_Dev->m_Uart.dlh));

printk("TCF=0x??; TCT=0x%2.2X; RTL=0x%2.2X; TFS=0x%2.2X; TCD=0x%2.2X\n",
        dev_ioread8(p_Dev, p_Dev->m_Uart.t0ctr), dev_ioread8(p_Dev, p_Dev->m_Uart.rtl),
        dev_ioread8(p_Dev, p_Dev->m_Uart.tfs), dev_ioread8(p_Dev, p_Dev->m_Uart.tcd));
}
//#endif
//
//  service function - calculates 2 timeval delta
//
static inline long deltaTime(struct timeval *tv_before, struct timeval *tv_after)
{
    long deltaSec = tv_after->tv_sec - tv_before->tv_sec;
    return ((tv_after->tv_usec + deltaSec*1000000) - tv_before->tv_usec);
}

/*====================Device Dispatch Functions=========================
 */
int dev_AttachDevice(phoneSIMStruct *theDevice,
                     uint16_t p_DeviceType,
                     uint16_t p_DeviceId)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    return(fpga_AttachDevice(&theDevice->m_Fpga,
                             p_DeviceType,
                             p_DeviceId));
  else
    return(amp_AttachDevice(theDevice->m_PhoneId,
                            &theDevice->m_Fpga,
                            p_DeviceType,
                            p_DeviceId));
}

int dev_ClockSim(phoneSIMStruct *theDevice,
                 StateChange p_NewState)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    return(fpga_ClockSim(&theDevice->m_Fpga,
                         p_NewState));
  else
    return(amp_ClockSim(theDevice->m_PhoneId,
                        &theDevice->m_Fpga,
                        p_NewState));
}

int dev_PowerSim(phoneSIMStruct *theDevice,
                 StateChange p_NewState)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    return(fpga_PowerSim(&theDevice->m_Fpga,
                         p_NewState));
  else
    return(amp_PowerSim(theDevice->m_PhoneId,
                        &theDevice->m_Fpga,
                        p_NewState));
}

void dev_GetFpgaRegisters(phoneSIMStruct *theDevice,
                          int p_PhoneId)
{
  // If probe...
  if (p_PhoneId < NUM_PHONES)
    fpga_GetFpgaRegisters(&theDevice->m_Fpga, p_PhoneId);
  else
    amp_GetFpgaRegisters(&theDevice->m_Fpga, p_PhoneId);
}

void dev_GetUartRegisters(phoneSIMStruct *theDevice,
                          int p_PhoneId)
{
  // If probe...
  if (p_PhoneId < NUM_PHONES)
    fpga_GetUartRegisters(&theDevice->m_Uart, p_PhoneId);
  else
    amp_GetUartRegisters(&theDevice->m_Uart, p_PhoneId);
}

int dev_WarmResetSim(phoneSIMStruct *theDevice)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    return(fpga_WarmResetSim(&theDevice->m_Fpga));
  else
    return(amp_WarmResetSim(theDevice->m_PhoneId, &theDevice->m_Fpga));
}

uint8_t dev_IsSimPresent(phoneSIMStruct *theDevice)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    return(fpga_IsSimPresent(&theDevice->m_Fpga));
  else
    return(amp_IsSimPresent(theDevice->m_PhoneId, &theDevice->m_Fpga));
}

uint8_t dev_IsSimReady(phoneSIMStruct *theDevice)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    return(fpga_IsSimReady(&theDevice->m_Fpga));
  else
    return(amp_IsSimReady(theDevice->m_PhoneId, &theDevice->m_Fpga));
}

uint8_t dev_IsPhonePresent(int p_PhoneId)
{
  // If probe...
  if (p_PhoneId < NUM_PHONES)
    // By definition ready...
    return(1);
  else
    return(amp_IsPhonePresent(p_PhoneId));
}

unsigned int dev_ioread8(phoneSIMStruct *theDevice, void *theAddress)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
    // Avoid another call - go diect
    return(ioread8(theAddress));
  else
    return(amp_ioread8(theDevice->m_PhoneId, theAddress));
}

void dev_iowrite8(phoneSIMStruct *theDevice, uint8_t theValue, void *theAddress)
{
  // If probe...
  if (theDevice->m_PhoneId < NUM_PHONES)
     // Avoid another call - go diect
    iowrite8(theValue, theAddress);
  else
    amp_iowrite8(theDevice->m_PhoneId, theValue, theAddress);
}

/*==============================HW Functions============================
 */
//
//
static void simdrv_Uart2ClearRx(phoneSIMStruct *p_Dev)
{
   // Clear rx data, line error, msr, and phone status interrupts.
    dev_iowrite8(p_Dev, FCR_FIFO_ENABLE | FCR_RX_FIFO_RST, p_Dev->m_Uart.fcr);
    dev_ioread8(p_Dev, p_Dev->m_Uart.msr);
    dev_ioread8(p_Dev, p_Dev->m_Fpga.ph_scr);
}
//
//
static int simdrv_Uart2CalculateDivisor(int p_nBaudRate)
{
    return ((UART_FREQ + (p_nBaudRate/2L)) / p_nBaudRate) - 1L;
}
//
//
//
static int simdrv_Uart2DivisorToBaudrate(int p_nDivisor)
{
    return (UART_FREQ + ((p_nDivisor+1)/2L)) / (p_nDivisor+1);
}
//
//  function enables UART Rx and Lsr interrupts as well
//  as phone interrupts (reset, VCC)
//  Function should be used when OPEN file ONLY
//
static inline void simdrv_isr_rx_enable(phoneSIMStruct *p_Dev)
{
    uint8_t  byUartIer  = IER_RX_IRQ_EN     | IER_LS_IRQ_EN;   //| IER_TX_IRQ_EN;
    uint8_t  byPhoneIer = PH_IMR_VCC_IRQ_EN | PH_IMR_RST_IRQ_EN;
    if ( g_FpgaVersion >= 7 )    // new UART
        byUartIer |= IER_GLBL_IRQ_EN;   // UART-2 got global int disable bit
    simdrv_ClearRx(p_Dev);
    dev_iowrite8(p_Dev, byPhoneIer, p_Dev->m_Fpga.ph_imr);
    dev_iowrite8(p_Dev, byUartIer,  p_Dev->m_Uart.ier);
}
//
//  disable ALL UART and phone interrupts
//  Function should be used when CLOSE file and init driver ONLY
//           in all other casess spinlock_irqsafe is used
//
static inline void simdrv_isr_all_disable(phoneSIMStruct *p_Dev)
{
    dev_iowrite8(p_Dev, 0x00, p_Dev->m_Fpga.ph_imr);
    dev_iowrite8(p_Dev, 0x00, p_Dev->m_Uart.ier);      //???? IER_GLBL_IRQ_EN
}
//
// enable only UART Tx interrupt
//
static inline void simdrv_isr_tx_enable(phoneSIMStruct *p_Dev)
{
    uint8_t  byUartIer = dev_ioread8(p_Dev, p_Dev->m_Uart.ier);
    dev_iowrite8(p_Dev, byUartIer | IER_TX_IRQ_EN,  p_Dev->m_Uart.ier);
}
//
// disable only UART Tx interrupt
//
static inline void simdrv_isr_tx_disable(phoneSIMStruct *p_Dev)
{
    uint8_t  byUartIer = dev_ioread8(p_Dev, p_Dev->m_Uart.ier);
    dev_iowrite8(p_Dev, byUartIer & (~IER_TX_IRQ_EN),  p_Dev->m_Uart.ier);
}

//****************************  SIMDRV_UART_ISR  *******************************
// SIMdrv_UART_ISR(): Service T=0 UART interrupts.
// 1) Only service 1 type of UART interrupt at a time to spread out the service
//    to other UARTs.
// 2) Service tx, rx, and line status interrupts.  Normally the rx interrupt can
//    handle the line status interrupts, but the T=0 transmit error indicator
//    does not necessarily occur with characters in the receive buffer.
//
static inline void simdrv_uart_isr(phoneSIMStruct *PD)
{
    volatile uint8_t        byLsr;
    uint16_t                rx_word = 0;
    uint16_t                tmpWrd = 0;
    uint8_t                 byIir = dev_ioread8(PD, PD->m_Uart.iir);
    uint8_t                 lsr_mask = (0x0F | g_LsrTxError); // Uart Rx error/dataready bits and Tx error bit.

#ifdef SIM_DEBUG_INTERRUPT_TRACE
static int cnt = 0;
if ( cnt++ < 160 ) printk(KERN_ALERT "-uart-%d; iir=0x%X\n", PD->m_PhoneId, byIir);
#endif

    switch(byIir & 0x0F)
    {
      case 0x06:  // Interrupt: Receive line status error (PE, FE, OR).
#ifdef SIM_DEBUG_INTERRUPT_TRACE
            printk("IIR - line status error will reset PE/FE/OR: %d %X\n", PD->m_PhoneId, (byIir & 0x0F));
#endif
            PD->m_NeedsReset = 1;
      case 0x0C:  // Interrupt: Stale date in FIFO.
      case 0x04:  // Interrupt: Receive buffer full.
        while ((byLsr = dev_ioread8(PD, PD->m_Uart.lsr)) != 0)
        {
#ifdef SIM_DEBUG_INTERRUPT_TRACE
            printk("Loop LSR data: %d %X \n", PD->m_PhoneId, byLsr);
#endif
            // We ignore some status bits here.
            if ((byLsr & lsr_mask) == 0) {
                break;
            }

            // receive error statistic
            if ( byLsr & 0x0E) {
#ifdef SIM_DEBUG_INTERRUPT_TRACE
                printk("LSR data: receive errors, statistics update ..\n");
#endif
                PD->lsr_error_cnt_irq++;

            }

            // Check for a T=0 transmitter error.
            if(byLsr & g_LsrTxError)
            {
                PD->tx_err_flg = 1;       // Inform driver of tx error.
                // wake up transmiter funtions
                //wake_up_interruptible(&PD->tx_wake_queue);
                // collect statistic
                PD->tx_error_cnt_irq++;
#ifdef SIM_DEBUG_INTERRUPT_TRACE
                printk("LSR data: receive - protocol (unrecoverable) tx error.. \n");
#endif
            }

            // Check for receive data and receive data line errors.
            // All receive errors will also have RBR data available.
            if(byLsr & 0x01)
            {
               rx_word = (((uint8_t)dev_ioread8(PD, (uint8_t*)(PD->m_Uart.rbr)) << 8) | (byLsr & 0x0E));
               tmpWrd = cpu_to_le16(rx_word);
#ifdef SIM_DEBUG_INTERRUPT_TRACE
 // if ( PD->m_DebugFlag )
               printk("LSR data - rcv buffer full %d data: %X \n", PD->m_PhoneId, rx_word);
#endif

                //NOTE - for LSR error buffer overrun - error flag
                //       will disaper only after buffer is empty.

                // Place the received character and the line status in
                // circular buffer.  If the circular buffer is full, then
                // increment a lost character count.

                // for WRITE ONLY files - errors coould be received here
                // operator if - will ignore these errors
                if ( PD->m_RxStatus == SIM_IO_ACTIVE)
                {
                    if ( cb_PutElement(&PD->m_RxBuffer, rx_word ) == 0 )
                        PD->rx_cnt_loss++;      // TODO: process conditions - return ???
                    else
                        tasklet_hi_schedule(&PD->m_RxTasklet);

                    PD->rx_cnt++;        // collect statistics
                }
            }
        }
        break;

      case 0x02:  // Interrupt: Transmit holding register empty.

#ifdef SIM_DEBUG_INTERRUPT_TRACE
            printk("IIR - Transmit holding register empty : %d %X\n", PD->m_PhoneId, (byIir & 0x0F));
#endif
        if ( g_FpgaVersion >= 7 )    // new UART with hardware Tx FIFO
        {
            if ( PD->m_TxBuffer.m_nTxPtr < sizeof(PD->m_TxBuffer.m_Buffer))
            {
                if ( simdrv_FillTxFifoIrq(PD) == 0 )
                {   // if all data from buffer is moved to FIFO - it is
                    // safe to allow next "send response" ioctl.
                    // This next "send response" will APPEND bytes to the FIFO -
                    // so all bytes will reach destination
                    PD->m_TxBuffer.m_DataType = TX_COMMAND;
                    // allow ioctl to process next "send response"
                    // semaphore up is allowed in IRQ - as not sleeping function
                    up (&PD->m_TxBuffer.m_TxSemaphore);
                }
            }
        }
        else        // old UART
        {
         // The interrupt was cleared by reading the IIR.  Signal the output
         // driver.
         if ( PD->m_TxBuffer.m_nTxPtr < sizeof(PD->m_TxBuffer.m_Buffer))
         {
            if ( PD->m_TxBuffer.m_nTxPtr+1 >= sizeof(PD->m_TxBuffer.m_Buffer))
                simdrv_isr_tx_disable(PD);       // disable Tx interrupt

#ifdef SIM_DEBUG_INTERRUPT_TRACE
printk("%d#%X ", PD->m_PhoneId, PD->m_TxBuffer.m_Buffer[PD->m_TxBuffer.m_nTxPtr]);
#endif

#ifdef SEND_DEBUG_MESSAGES
            // if debug messages
            if ( (PD->m_TxBuffer.m_DataType == TX_ATR) && PD->m_DebugOn)
            {
                struct timeval      timeStamp;
                long                delta;

                do_gettimeofday (&timeStamp);           //byte sent time
                delta = deltaTime(&PD->m_LastTime, &timeStamp);

                // time when first ATR byte was sent
                if ( PD->m_AtrDelta == 0 )
                {
                    PD->m_AtrFirstDelta = delta;
                    PD->m_AtrDelta = 1;
                }
                else
                if ( PD->m_AtrDelta < delta )
                    PD->m_AtrDelta = delta;

                PD->m_LastTime = timeStamp;
            }
#endif
            // send next byte and move pointer
            dev_iowrite8(PD, PD->m_TxBuffer.m_Buffer[PD->m_TxBuffer.m_nTxPtr], PD->m_Uart.rbr);
            PD->tx_cnt++;               // statistic

            PD->m_TxBuffer.m_nTxPtr++;

            // reach the end of the buffer
            if ( PD->m_TxBuffer.m_nTxPtr >= sizeof(PD->m_TxBuffer.m_Buffer))
            {   // all bytes of the buffer was sent - can send next portion
#ifdef SEND_DEBUG_MESSAGES
                // send debug message
                if ( (PD->m_TxBuffer.m_DataType == TX_ATR) && PD->m_DebugOn)
                {    //send debug info
                    snprintf(PD->m_DebugMsg + strlen(PD->m_DebugMsg),           // write to debug buffer
                             sizeof(PD->m_DebugMsg) - strlen(PD->m_DebugMsg),   // not more than free bytes exists
                            "Reset %3.3ld.%6.6ld sec; ATR after=%ld usec\n",
                            PD->m_ResetTime.tv_sec%1000, PD->m_ResetTime.tv_usec,
                            PD->m_AtrFirstDelta);


                    snprintf(PD->m_DebugMsg + strlen(PD->m_DebugMsg),           // write to debug buffer
                             sizeof(PD->m_DebugMsg) - strlen(PD->m_DebugMsg),   // not more than free bytes exists
                            "ATR sent %3.3ld.%6.6ld sec; ATR max delta=%ld usec\n",
                            PD->m_ResetTime.tv_sec%1000, PD->m_ResetTime.tv_usec, PD->m_AtrDelta);
                }
#endif
                if (  PD->m_TxBuffer.m_BaudRateDivisor != 0
                   && PD->m_TxBuffer.m_DataType == TX_COMMAND )
                {
                    // set new baudrate
                    // TODO - i know it takes some time (1.4 msec to send byte )
                    //        so everyone in system will wait on this loop
                    while ((dev_ioread8(PD, PD->m_Uart.lsr) & (LSR_TRANSMIT_EMPTY)) == 0);

                    simdrv_SetUartDivisor(PD, PD->m_TxBuffer.m_BaudRateDivisor);

                }
                PD->m_TxBuffer.m_DataType = TX_COMMAND;
                // allow ioctl to process next "send response"
                // semaphore up is allowed in IRQ - as not sleeping function
                up (&PD->m_TxBuffer.m_TxSemaphore);
            }
         }
         else
         {
            printk("ERROR: - TX TRYING TO SEND NO BYTES(%d) %d\n", PD->m_PhoneId, PD->m_TxBuffer.m_nTxPtr);
            //fix the damage - reset Tx interrupt by sending dummy byte
            simdrv_isr_tx_disable(PD);       // disable Tx interrupt
            dev_iowrite8(PD, 0, PD->m_Uart.rbr);
         }
         }
         break;


      case 0x01:  // No interrupt is pending within the IIR.
        PD->spurious_uart_irq++;
        break;

      case 0x00:  // Interrupt: Modem status register signal change.
        dev_ioread8(PD, PD->m_Uart.msr);      // Clear the modem status interrupt.
        PD->bogus_uart_irq++;
        break;

      default:
        PD->bogus_uart_irq++;
   }
// if ( cnt++ < 160 ) printk("+uart isr complete+%d; iir=0x%X\n", PD->m_PhoneId, byIir);
}
//
// phone IRQ helper function -
// put event to Circular Buffer
//
inline static void PutEventToBuffer (phoneSIMStruct *p_Dev, uint16_t p_usEvent)
{
    if ( cb_PutElement(&p_Dev->m_RxBuffer, (p_usEvent >> 8)) != 0 )
        tasklet_hi_schedule(&p_Dev->m_RxTasklet);   // bottom half
    else
        p_Dev->rx_cnt_loss++;
}
//***************************  SIMDRV_PHONE_ISR  *******************************
// SIMdrv_Phone_ISR(): Service T=0 VCC/RST signal change interrupts.
//
static inline void simdrv_phone_isr(phoneSIMStruct *PD)
{
    register uint8_t    tval = dev_ioread8(PD, PD->m_Fpga.ph_scr);
    uint16_t            rx_word;
    int8_t              chg_flg = 0;

#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
static int cnt = 0;
if ( cnt++ < 100 ) printk(KERN_ALERT "^phone^%d val=0x%X\n", PD->m_PhoneId, tval);
#endif


    // Check for RST changes.
    if(tval & FPGA_LPH_SCR_DELTA_RST)
    {
#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
printk(KERN_ALERT "^phone isr !!! delta reset\n");
#endif
        chg_flg = -1;
        if(tval & FPGA_LPH_SCR_RST_STATUS)
        {
            chg_flg = 1;
#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
printk(KERN_ALERT "^phone isr !!! reset\n");
#endif
            // RX CIRCULAR BUFFER INSERTION(0x8100) or PD->rx_cnt_loss++ ]]]
            rx_word = IRQ_EVENT_RESET << 8;
            // rx_word = IRQ_EVENT_RESET ;
#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
{
printk("Rst%d 0x%X\n", PD->m_PhoneId, rx_word);
    struct timeval      timeStamp;
    do_gettimeofday (&timeStamp);
    printk("%2.2ld.%6.6ld :Rst%d 0x%X\n", timeStamp.tv_sec%100, timeStamp.tv_usec,
                                         PD->m_PhoneId, rx_word);
}
#endif

#ifdef SEND_DEBUG_MESSAGES
            // ATR debug messages initialization
            do_gettimeofday (&PD->m_ResetTime);
            PD->m_LastTime = PD->m_ResetTime;
            PD->m_AtrDelta  = 0;
#endif
            // copy event to another phone
            if ( PD->m_PhoneForCopyEvents != NULL )
                PutEventToBuffer (PD->m_PhoneForCopyEvents, rx_word);

            PutEventToBuffer (PD, rx_word);
        }
    }

    // Check for VCC level changes.
    if(tval & FPGA_LPH_SCR_DELTA_VCC)
    {
#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
        printk(KERN_ALERT "^phone isr !!! delta vcc\n");
#endif
        chg_flg = 1;
        // RX CIRCULAR BUFFER INSERTION((tval & 0x8E) << 8) or PD->rx_cnt_loss++
        rx_word = (tval & 0x4E);
        rx_word = rx_word << 8;
#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
{
printk("EV%d 0x%X\n", PD->m_PhoneId, rx_word);
    struct timeval      timeStamp;
    do_gettimeofday (&timeStamp);
    printk("%2.2ld.%6.6ld :EV%d 0x%X\n", timeStamp.tv_sec%100, timeStamp.tv_usec,
                                         PD->m_PhoneId, rx_word);

}
#endif
        // copy event to another phone
        if ( PD->m_PhoneForCopyEvents != NULL )
            PutEventToBuffer (PD->m_PhoneForCopyEvents, rx_word);

        PutEventToBuffer (PD, rx_word);
    }

    // The change flag determines how the interrupt was handled.
    // chg_flg < 0 : Interrupt event was ignored.
    // chg_flg = 0 : Interrupt had no associated event.
    // chg_flg > 0 : Reportable interrupt event.
#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
    if(chg_flg == 0) {
        PD->spurious_phone_irq++;
        printk(KERN_ALERT "^phone isr !!! spurious interrupt \n");
    }
    if (chg_flg < 0)
        printk(KERN_ALERT "^phone isr !!! interrupt ignored\n");
    if (chg_flg > 0)
        printk(KERN_ALERT "^phone isr !!! reportable interrupt event\n");
#endif

}

//******************************  SIMDRV_ISR  **********************************
// SIMDrv_ISR():  Service SSW interrupts.  SSW interrupts are on a dedicated
//  X-Scale interrupt line, so this interrupt is not shared.
//
// Each 16 bit word within a receive buffer is encoded as follows:
//
//  0000 xxx0 xxxx xxxx = Receive character event
//  0100 xxx0 0000 0000 = VCC event
//  1000 000x 0000 0000 = RST event (only report rising edge events).
//
//  Receive character events:
//  0000 xxx0 xxxx xxxx = Receive character event
//  0000 0000 xxxx xxxx =  Received character is in the LSB w/o error.
//  0000 0010 xxxx xxxx =  A UART over-run has been detected.  The character in
//                         the LSB was received w/o error.
//  0000 0100 xxxx xxxx =  The char in the LSB was received with a parity error.
//  0000 1000 xxxx xxxx =  The char in the LSB was received with a framing error.
//  0000 1100 xxxx xxxx =  The char in the LSB was received with a parity and
//                         framing error.
//  0000 1110 xxxx xxxx =  The char in the LSB was received with a parity and
//                         framing error.  In addition, a UART over-run occurred.

//  VCC events:
//  0100 xxx0 0000 0000 = VCC event
//  0100 1xx0 0000 0000 =  VCC fault event.
//  0100 0000 0000 0000 =  VCC off event.
//  0100 0010 0000 0000 =  VCC 1.8V on event.
//  0100 0100 0000 0000 =  VCC 3.0V on event.
//  0100 0110 0000 0000 =  VCC 5.0V on event.

//  RST events:
//  1000 000x 0000 0000 = RST event (only report rising edge events).
//  1000 0000 0000 0000 =  RST falling edge event.
//  1000 0001 0000 0000 =  RST rising edge event.
//
#ifndef SIM_DEV_BOARD
static irqreturn_t simdrv_isr(int32_t irq, void *dev_id)
{
    register uint8_t fpga_ssw_irq;

#ifdef SIM_DEBUG_PHONE_INTERRUPT_TRACE
    static int cnt = 0;
    if ( cnt++ < 160 ) printk("%d^simdrv isr^%X\n", cnt, fpga_ssw_irq);
#endif

   // disable all ssw interrupts

   iowrite8(0x01, ssw_irq_reg);

   fpga_ssw_irq = ioread8(ssw_irq_reg);

   if(fpga_ssw_irq & FPGA_SSW_IRQ_PH0)  simdrv_phone_isr(&phoneSIMData[0]);
   if(fpga_ssw_irq & FPGA_SSW_IRQ_PH1)  simdrv_phone_isr(&phoneSIMData[1]);
   if(fpga_ssw_irq & FPGA_SSW_IRQ_PH2)  simdrv_phone_isr(&phoneSIMData[2]);
   if(fpga_ssw_irq & FPGA_SSW_IRQ_UART0) simdrv_uart_isr(&phoneSIMData[0]);
   if(fpga_ssw_irq & FPGA_SSW_IRQ_UART1) simdrv_uart_isr(&phoneSIMData[1]);
   if(fpga_ssw_irq & FPGA_SSW_IRQ_UART2) simdrv_uart_isr(&phoneSIMData[2]);

   // enable all interrupts
   iowrite8(0x00, ssw_irq_reg);
   return IRQ_HANDLED;

}
#endif

/*
 *  starts/restarts send T=0 NULL byte timer
 */
static void simdrv_StartTimer(phoneSIMStruct *p_Dev)
{
    if ( p_Dev->m_SendNull == SIMCONFIG_SEND_NULL )
    {
        /* start blink timer */
        p_Dev->m_SendNullTimer.function = simdrv_SendNull;
        p_Dev->m_SendNullTimer.data = (unsigned long)p_Dev;
        p_Dev->m_SendNullTimer.expires = jiffies + HZ;

        add_timer(&p_Dev->m_SendNullTimer);
    }
}
//
//  function stop all Waiting response activities
//
static inline void simdrv_StopWaitingResponse(phoneSIMStruct *p_Dev)
{
    p_Dev->m_T0Mode = T0_WAIT_COMMAND;
    // stop scheduled timer
    del_timer_sync(&p_Dev->m_SendNullTimer);
}

//***************************  UART_CONFIG_PORT  ******************************
// SIMDrv_Config():  Configures a different aspects of the driver functionality
//
int simdrv_Config(phoneSIMStruct *p_Dev, struct SimConfigHeader *p_Config, uint8_t *arg)
{

    switch (p_Config->m_ConfigType )
    {
        case SIMCONFIG_COMMAND_LIST:
        {
            struct SimConfigCommandList cmdList;
            int     i = 0;

            if ( p_Config->m_Length > sizeof(struct SimConfigCommandList) )
                return -EFAULT;         // driver does not have enough space

            if ( copy_from_user(&cmdList, arg, p_Config->m_Length))
                return -EFAULT;

            // clear phone command table
            memset ( p_Dev->m_CommandMap, SIMCMD_NOT_DEFINED, sizeof (p_Dev->m_CommandMap));

            while ( cmdList.m_CommandList[i] != 0 )
            {
                p_Dev->m_CommandMap[cmdList.m_CommandList[i] & 0xFF] =
                        (cmdList.m_CommandList[i] >> 8) & 0xFF;
                i++;
            }

            return 0;
        }

        case SIMCONFIG_UART:
        {
            struct SimConfigUart    uartCfg;
            //uint16_t                usIers;
            int                     retval = 0;

            if ( p_Config->m_Length != sizeof(struct SimConfigUart) )
                return -EFAULT;         // driver does not have enough space
            if ( copy_from_user(&uartCfg, arg, p_Config->m_Length))
                return -EFAULT;

            p_Dev->m_DefaultUartConfig = uartCfg;
            //usIers = simdrv_isr_disable(p_Dev, FMODE_READ | FMODE_WRITE);
            // reconfigure UART
            retval =  simdrv_UartConfig(p_Dev, &uartCfg);
            // enable interrupts

            //simdrv_isr_enable(p_Dev,(uint8_t)((usIers>>8) & 0xFF),
            //                      (uint8_t)(usIers & 0xFF));
            return retval;
        }
        case SIMCONFIG_WAITING_MODE:
        {
            struct SimConfigWaitingMode waitingCfg;
            int                         retval = 0;

            if ( p_Config->m_Length != sizeof(struct SimConfigWaitingMode) )
                return -EFAULT;         // driver does not have enough space
            if ( copy_from_user(&waitingCfg, arg, p_Config->m_Length))
                return -EFAULT;

            p_Dev->m_SendNull = (waitingCfg.m_SendNull == 0 )? SIMCONFIG_DISMISS_NULL : SIMCONFIG_SEND_NULL;
            return retval;
        }
        case SIMCONFIG_ACK_MODE:
        {
            struct SimConfigAckMode     ackCfg;
            int                         retval = 0;

            if ( p_Config->m_Length != sizeof(struct SimConfigAckMode) )
                return -EFAULT;         // driver does not have enough space
            if ( copy_from_user(&ackCfg, arg, p_Config->m_Length))
                return -EFAULT;

            p_Dev->m_AckMode = (ackCfg.m_SendAck == 0 )? SIMCONFIG_DISMISS_ACK : SIMCONFIG_SEND_ACK;
            return retval;
        }
        case SIMCONFIG_ATTACH_DEVICE:
        {
            struct SimConfigAttachDevice    attachCfg;

            if ( p_Config->m_Length != sizeof(struct SimConfigAttachDevice) )
                return -EFAULT;         // driver does not have enough space
            if ( copy_from_user(&attachCfg, arg, p_Config->m_Length))
                return -EFAULT;

            return dev_AttachDevice(p_Dev, attachCfg.m_DeviceType, attachCfg.m_DeviceId);
        }
        case SIMCONFIG_DEBUG:
        {
            struct SimConfigDebug   debugCfg;
            int                     retval = 0;

            if ( p_Config->m_Length != sizeof(struct SimConfigDebug) )
                return -EFAULT;         // driver does not have enough space
            if ( copy_from_user(&debugCfg, arg, p_Config->m_Length))
                return -EFAULT;

#ifdef SEND_DEBUG_MESSAGES
            p_Dev->m_DebugOn = (debugCfg.m_Debug == 0 )? SIMCONFIG_DEBUG_OFF : SIMCONFIG_DEBUG_ON;
#endif
            return retval;
        }
    }

    return -ENOSYS;
}

//
//  function sets new UART baudrate
//  Funtion disables interrupts prior to executing this function.
//
static void simdrv_SetUartDivisor(phoneSIMStruct *p_Dev, int32_t p_nDivisor)
{
    unsigned long   ulFlags;

    // No change in baud, so we're done.
    if (g_uartDivisor[p_Dev->m_PhoneId] == p_nDivisor)
        return;
    g_uartDivisor[p_Dev->m_PhoneId] = p_nDivisor;

 //   spin_lock_irqsave(&p_Dev->m_ConfigLock, ulFlags);

    // Set the baud rate.
    dev_iowrite8(p_Dev, dev_ioread8(p_Dev, p_Dev->m_Uart.lcr) | 0x80, p_Dev->m_Uart.lcr);  // Access divisor latch.
    dev_iowrite8(p_Dev, (int8_t)p_nDivisor, p_Dev->m_Uart.dll);
    dev_iowrite8(p_Dev, (int8_t)(p_nDivisor >> 8), p_Dev->m_Uart.dlh);
    //
    dev_iowrite8(p_Dev, dev_ioread8(p_Dev, p_Dev->m_Uart.lcr) & 0x7F, p_Dev->m_Uart.lcr);  // Access divisor unlatch.

    // clear Rx FIFO and stuff
    simdrv_ClearRx(p_Dev);

//    spin_unlock_irqrestore(&p_Dev->m_ConfigLock, ulFlags);
#ifdef SIM_DEBUG_TRACE
    printk("Set uartDivisor[%d]: Baudrate = %d\n", p_Dev->m_PhoneId, simdrv_DivisorToBaudrate(p_nDivisor));
#endif
}

//***************************  UART_CONFIG_PORT  ******************************
// SIMDrv_UART_Cfg():  Configures a UART port.
//  Funtion disables interrupts prior to executing this function.
//
static int simdrv_UartConfig(phoneSIMStruct *p_Dev, struct SimConfigUart *p_UartConfig)
{
    int32_t         nDivisor;
    unsigned long   ulFlags;

// printk("simdrv_UartConfig - enter\n");
    nDivisor = simdrv_CalculateDivisor(p_UartConfig->m_nBaudRate);
    g_uartDivisor[p_Dev->m_PhoneId] = nDivisor;

// printk("simdrv_UartConfig - before spinlock\n");
//    spin_lock_irqsave(&p_Dev->m_ConfigLock, ulFlags);
// printk("simdrv_UartConfig - after spinlock\n");
    // Set the baud rate.
    dev_iowrite8(p_Dev, dev_ioread8(p_Dev, p_Dev->m_Uart.lcr) | 0x80, p_Dev->m_Uart.lcr);  // Access divisor latch.
    dev_iowrite8(p_Dev, (int8_t)nDivisor, p_Dev->m_Uart.dll);
    dev_iowrite8(p_Dev, (int8_t)(nDivisor >> 8), p_Dev->m_Uart.dlh);

    // Disable access to the divisor latch and set the LCR (data bits, parity,
    // and stop bits).
    dev_iowrite8(p_Dev, p_UartConfig->m_chLcr & 0x3F, p_Dev->m_Uart.lcr);
    // write SPC register
    dev_iowrite8(p_Dev, p_UartConfig->m_chSpc, p_Dev->m_Uart.spc);

    // clear Rx FIFO and stuff
    simdrv_ClearRx(p_Dev);

//    spin_unlock_irqrestore(&p_Dev->m_ConfigLock, ulFlags);

    // Log insertion change
    printk(KERN_ALERT
           "simdrv: phonesim%d: baudrate %d\n",
           p_Dev->m_PhoneId, simdrv_DivisorToBaudrate(nDivisor));

    return 0;
}

//=============================TASKLET =========================================
// =============================================================================
//  IRQ tasklet
// =============================================================================
void simdrv_tasklet (unsigned long p_Param)
{
    //int       rv;
    phoneSIMStruct *dev = (phoneSIMStruct*)p_Param;
    BufferElement   usValue;
    BufferElement   usValue_t;

    // get the all characters from RX circular buffer one by one
    while ( cb_GetElement(&dev->m_RxBuffer, &usValue) != 0 )
    {
        uint8_t    byEvent;

    //-----------------------------------------------------------------------
    //          here is a place to add extra processing for the data stream
    //          f.ex implement T=0 protocol
    //-----------------------------------------------------------------------
        byEvent = simdrv_translateEvent(dev, &usValue);
#ifdef SIM_DEBUG_INTERRUPT_TRACE
                    printk("simdrv: event %d %x\n",byEvent,usValue);
#endif
        // queue_work signature for Linux 2.6 only
        if (byEvent == SIM_RESET_EVENT)
        {
#ifdef SIM_DEBUG_INTERRUPT_TRACE
                    printk("simdrv: ATR for phone RESET_EVENT \n");
#endif
            simdrv_StopWaitingResponse(dev);

            // restore default UART configuration/speeed after reset
            simdrv_UartConfig(dev, &dev->m_DefaultUartConfig);

            // if we do not steal events - send ATR
            if ( dev->m_EventSrc == -1 )
            {
                if ( dev->ATRsize > 0 )
                {
                    // we know that User Context and Tasklets
                    // are serialized (by disable_tasklet)
                    // disable Tx interrupts
                    simdrv_isr_tx_disable(dev);

                    // if semaphore was not down - get it
                    // if it was - leave as it was - in both cases
                    // driver should free semaphore afetr sending ATR
                    down_trylock (&dev->m_TxBuffer.m_TxSemaphore);

                    // set flag - ATR is sent;
                    //simdrv_send_atr(dev);
#ifdef SIM_DEBUG_INTERRUPT_TRACE
                    printk("simdrv: sending ATR -  RESET_EVENT\n");
#endif
                    simdrv_SendAtr(dev);
                }
                else
                    printk("ERROR simdrv: ATR for phone %d NOT LOADED. Reset ignored\n", dev->m_PhoneId);

            }
        }


//-----------------------------------------------------------------------
        // put processed character to ioctl buffer

        if (byEvent == IRQ_CMD_OR_PAYLOAD) {
             usValue_t = (usValue >> 8);
        } else  {
             usValue_t = (usValue << 8);
        }

#ifdef SIM_DEBUG_INTERRUPT_TRACE
    printk("IOCTL Thr(%d) 0x%X\n", dev->m_PhoneId, usValue_t);
#endif
        if ( cb_PutElement(&dev->m_IoctlBuffer, usValue_t) != 0 )
        // if ( cb_PutElement(&dev->m_IoctlBuffer, (usValue << 8)) != 0 )
        {
            // for any event - wakeup blocking ioctl
            if ( byEvent != CMD_OR_PAYLOAD )
                atomic_inc(&dev->m_EventCnt);
        }
        else
            dev->ioctl_cnt_loss++;
    }

    if ( dev->tx_err_flg != 0)       // tx error happens
    {
        dev->tx_err_flg = 0;
        usValue = (LINE_STATUS_EVENT + TRANSMIT_ERROR_EVENT) << 8;
        // usValue = (LINE_STATUS_EVENT + TRANSMIT_ERROR_EVENT) ;
        if ( cb_PutElement(&dev->m_IoctlBuffer, usValue) == 0 )
            dev->ioctl_cnt_loss++;
    }
    // wake up ioctl
    wake_up_interruptible(&dev->ioctl_wake_queue);
}
//==========================================================================
//
//  translate FPGA registers event representation to RCATS event
//
static inline uint8_t simdrv_translateEvent(phoneSIMStruct * p_Dev, BufferElement *p_usValue)
{
    uint8_t     *byWork = (uint8_t*)p_usValue;
    uint8_t     byEvent;

#ifdef SIM_DEBUG_INTERRUPT_TRACE
    printk("simdrv: translate %x %x \n",byWork[0], byWork[1]);
#endif

    if ( byWork[0] == IRQ_CMD_OR_PAYLOAD )
        return IRQ_CMD_OR_PAYLOAD;


    byEvent = byWork[0];

    if ( byEvent == IRQ_EVENT_RESET )
    {
            byWork[0] = SIM_RESET_EVENT;
            return SIM_RESET_EVENT;
    }
//  VCC events:
//  0100 xxx0 0000 0000 = VCC event
//  0100 1xx0 0000 0000 =  VCC fault event.
//  0100 0000 0000 0000 =  VCC off event.
//  0100 0010 0000 0000 =  VCC 1.8V on event.
//  0100 0100 0000 0000 =  VCC 3.0V on event.
//  0100 0110 0000 0000 =  VCC 5.0V on event.
    if ( byEvent & IRQ_EVENT_VCC )
    {
        if ( (byEvent & IRQ_EVENT_VCC_FAULT) == IRQ_EVENT_VCC_FAULT )
        {
            byWork[0] = POWER_FAULT_EVENT;
            return POWER_FAULT_EVENT;
        }
        if ( byEvent == IRQ_EVENT_VCC_POWER_OFF )
        {
            byWork[0] = POWER_OFF_EVENT;
            return POWER_OFF_EVENT;
        }
        byWork[0] = POWER_ON_EVENT;
//        byWork[0] = POWER_ON_EVENT | ((byEvent<<3) & 0xF0);

        return POWER_ON_EVENT;
    }
    // only line status event left

    byWork[0] = LINE_STATUS_EVENT | (byEvent << 3);
    return byWork[0];
}

//
//
//
static int 
simdrv_UartEnable(
  phoneSIMStruct *pDev,
  struct file * file)
{
  if (file->f_mode & FMODE_READ)
      pDev->m_RxStatus = SIM_IO_ACTIVE;
  if (file->f_mode & FMODE_WRITE)
  {
      pDev->m_TxStatus = SIM_IO_ACTIVE;
      // reset Tx subsystem
      pDev->m_TxBuffer.m_nTxPtr = sizeof(pDev->m_TxBuffer.m_Buffer);
      pDev->m_TxBuffer.m_DataType = TX_COMMAND;
  }
  pDev->m_T0Mode   = T0_WAIT_COMMAND;
  pDev->m_SendNull = SIMCONFIG_SEND_NULL;

  pDev->m_NeedsReset = 1;
  
  dev_iowrite8(pDev, 1, pDev->m_Uart.t0ctr);  // clear UART Rx/Tx error counter
  dev_iowrite8(pDev, 1, pDev->m_Uart.tcd);    // set UART Tx inter-character delay (G20 needs it to be at least 1)
  
  // configure UART to default configuration - at that point interrupts are disabled
  simdrv_UartConfig(pDev, &pDev->m_DefaultUartConfig);

  MOD_INC_USE_COUNT;  /* Before we maybe sleep */
  // enable interrupts - in special ioctl
  simdrv_isr_rx_enable(pDev);

  dev_PowerSim(pDev, STATE_OFF);        // power off the SIM

  return 0;
}

//
//
//
static int simdrv_open(struct inode * inode, struct file * file)
{
    phoneSIMStruct  *pDev;
    FileStruct      *pFile = (FileStruct *)file->private_data;
    int ph = 0;

    // Access phone instance
    ph = (MINOR(inode->i_rdev) % NUM_DEVICES);
    pDev = phoneSIMData + ph;

    // If not initialized...
    if (pDev->m_PhoneId == NUM_DEVICES)
    {
      // Initialize
      simdrv_init_phone(ph);
    }

    if (!pFile)                        /* non-devfs open support */
        file->private_data = pFile = kmalloc(sizeof(FileStruct), GFP_KERNEL);
    if (!pFile)
        return -ENOMEM;

    // attach UART structure to the file structure
    pFile->m_pUartStruct = phoneSIMData + (MINOR(inode->i_rdev) % NUM_DEVICES);
    pDev = pFile->m_pUartStruct;

    if ( file->f_flags & O_EXCL )
    {           // exclusive access
        if ( pDev->m_ExclusiveFlag == EXCLUSIVE_ACCESS_ON )
        {
            kfree((void*)pFile);
            file->private_data = NULL;
            return -EBUSY;                  // allowed only one file with exclusive access
        }
        printk("Exclusive file access UART %d\n", pDev->m_PhoneId);
        pFile->m_ExclusiveAccess = EXCLUSIVE_ACCESS_ON;        // Exclusive access file
        pDev->m_ExclusiveFlag = EXCLUSIVE_ACCESS_ON;          // Exclusive access file
    }
    else
    {
        pFile->m_ExclusiveAccess = EXCLUSIVE_ACCESS_OFF;        // regular file
        pDev->m_ExclusiveFlag  = EXCLUSIVE_ACCESS_OFF;          // regular file
    }

    return(0);
}

//
//
//
static int 
simdrv_UartDisable(
  phoneSIMStruct *pDev,
  struct file * file)
{
  FileStruct      *pFile = (FileStruct *)file->private_data;
    
  // stop scheduled timer
  del_timer_sync(&pDev->m_SendNullTimer);

  mdelay(2);  // 1.3 character-times to allow quesence

  // disable interrupts
  simdrv_isr_all_disable(pDev);

  mdelay(2);  // 1.3 character-times to allow quesence
  //
  // Stop transmit operation if any runs
  if (file->f_mode & FMODE_READ)
  {
      pDev->m_RxStatus = SIM_IO_STOP;
      atomic_set(&pDev->m_EventCnt, 0);
      cb_ClearBuffer(&pDev->m_RxBuffer);
      cb_ClearBuffer(&pDev->m_IoctlBuffer);
  }
  if (file->f_mode & FMODE_WRITE)
  {
      // wake up transmitting "thread" and stop current
      // transmittion as well as all waiting transmittions
      pDev->m_TxStatus = SIM_IO_STOP;
      // reset Tx subsystem
      if ( pDev->m_TxBuffer.m_nTxPtr < sizeof(pDev->m_TxBuffer.m_Buffer))
          up (&pDev->m_TxBuffer.m_TxSemaphore);
      pDev->m_TxBuffer.m_nTxPtr = sizeof(pDev->m_TxBuffer.m_Buffer);
      pDev->m_TxBuffer.m_DataType = TX_COMMAND;
  }

  MOD_DEC_USE_COUNT;

  // switch off copying events from another phone
  if ( pDev->m_EventSrc != -1 )
      phoneSIMData[pDev->m_EventSrc].m_PhoneForCopyEvents = NULL;

  if ( pFile->m_ExclusiveAccess == EXCLUSIVE_ACCESS_ON )
      pDev->m_ExclusiveFlag  = EXCLUSIVE_ACCESS_OFF;       // unlock files on this UAR

  dev_PowerSim(pDev, STATE_OFF);        // power off the SIM

  dev_AttachDevice(pDev, DEVTYPE_NODEVICE, DEVTYPE_NODEVICE);

  return 0;
}

static int simdrv_release(struct inode * inode, struct file * file)
{
    FileStruct      *pFile = (FileStruct *)file->private_data;
    phoneSIMStruct  *pDev = pFile->m_pUartStruct;

    // free file structure allocated in the open
    kfree((void*)pFile);
    file->private_data = NULL;

    return 0;
}
//
// Function to process SIMDRV_RESPONSE_NEGOTIATION(old) and SIMDRV_RESPONSE_BAUDRATE (new)
// sends response to the phone and wait untill last byte of response is left UART.
// Calling program can change baud rate without any extra waiting
//
static int simdrv_SendResponseEx(phoneSIMStruct *p_Dev, uint16_t p_Length,
                                 uint8_t *p_Response, uint16_t p_BaudRateDivisor)
{
    uint8_t     lsr;

    if (p_Length > RSP_BUFFER_SIZE)
        return -E2BIG;

#ifdef SIM_DEBUG_TRACE
    simdrv_DebugPrint("RSP_Ex", p_Dev->m_PhoneId, p_Response, (int)p_Length);
#endif
    // function can be used to set new UART baudrate
    // in this case data length = 0;
    if ( p_Length > 0 )
    {
        int         nRetval;
        // change status to wait command (blocking function)
        simdrv_StopWaitingResponse(p_Dev);
        // analyze response to diffrentiate NAK/ERROR/NULL situations
        if ( p_Dev->m_LastClass != 0xFF && p_Dev->m_LastClass != 0x3B
          && p_Length > 2 )         // data. (error situation - always return 2 bytes )
        {           // send ACK
            // protection against unknown commands
            if ( p_Dev->m_AckMode == SIMCONFIG_SEND_ACK)
            {
                nRetval = simdrv_SendResponse(p_Dev, &(p_Dev->m_LastInstruction), 1, 0);
                if ( nRetval )
                    return nRetval;
            }
        }
        // blocking send data to the phone
        nRetval = simdrv_SendResponse(p_Dev, p_Response, p_Length, p_BaudRateDivisor);
        if ( nRetval )
            return nRetval;
    }
    // function will return after it put LAST response byte to the UART register
    // now we will wait untill this byte will leave the register == transmitting is finished
    // and then change the UART baud rate
    while (((lsr = dev_ioread8(p_Dev, p_Dev->m_Uart.lsr))
            & (LSR_TRANSMIT_EMPTY/* | LSR_TRANSMIT_HOLDING_EMPTY*/)) == 0 );

    return 0;
}
//
//
//
#ifndef SIM_DEV_BOARD
static int simdrv_ioctl(struct inode * inode, struct file * file, unsigned int command, unsigned long arg)
#else
static long simdrv_ioctl(struct file * file, unsigned int command, unsigned long arg)
#endif
{
    int retval = 0;

//static int dbg_cnt = 0;
    FileStruct      *pFile = (FileStruct *)file->private_data;
    phoneSIMStruct  *pDev = pFile->m_pUartStruct;
    // if ((retval = down_interruptible(&PD->Serialize)) == 0)  {   // only allow one ioctl at a time for a given /dev/simphoneX
    if (signal_pending(current))
    {
        retval = -ERESTARTSYS;
    }
    else
    {
        switch (command)
        {
        case SIMDRV_VERSION:         // return driver version
            return  DRIVER_VERSION_INT;

        case SIMDRV_COMMAND_FULL:
            {
                struct SimCommand   cmd;
/*
    Later When it would be possible to test add support for exclusive access (monitoring)
    It is possible that this should be done in simdrv_ReadRawDataBlock function - not here
                if ( pDev->m_ExclusiveFlag == EXCLUSIVE_ACCESS_ON
                  && pFile->m_ExclusiveAccess != EXCLUSIVE_ACCESS_ON)
                    return -EBUSY;
*/
                // serialize command reading by Rx semaphore
                if (down_interruptible (&pDev->m_RxSemaphore))
                    return -ERESTARTSYS;
                // next call is blocking
                retval = simdrv_ReadCommandFull(pDev, &cmd);

                // free Rx semaphore
                up (&pDev->m_RxSemaphore);

                if ( retval != 0 )
                    return retval;

#ifdef SIM_DEBUG_TRACE
                simdrv_DebugPrint("CMD_FULL", pDev->m_PhoneId, cmd.m_Command, (int)cmd.m_Length);
                if (cmd.m_Event != CMD_OR_PAYLOAD)
                    printk("simdrv: Event = 0x%X\n", cmd.m_Event);
#endif
#ifdef SEND_DEBUG_MESSAGES
                // add debug messages if any exist
                simdrv_AddDebugMsg(PD, &cmd);
#endif
                if ( copy_to_user((void*)arg, &cmd, sizeof(struct SimCommand)) )
                    retval = -EFAULT;
            }
            break;

        case SIMDRV_COMMAND:
            {
            uint8_t     byCmdSz;

            if ((retval = get_user(byCmdSz, (uint8_t *)arg)) == 0)
            {
                uint8_t         strResBuf[MAX_CMD_SIZE];

                // serialize command reading by Rx semaphore
                if (down_interruptible (&pDev->m_RxSemaphore))
                    return -ERESTARTSYS;

                retval = simdrv_ReadCommand(pDev, strResBuf, byCmdSz);

                // free Rx semaphore
                up (&pDev->m_RxSemaphore);

                if ( retval >= 0 )
                {
#ifdef SIM_DEBUG_TRACE
                    simdrv_DebugPrint("CMD", pDev->m_PhoneId, strResBuf, (int)byCmdSz+1);
#endif
                    if ( copy_to_user((void*)arg, strResBuf, byCmdSz+1) )
                        retval = -EFAULT;
                }
            }
            else
                retval = -EFAULT;
            }
            break;
        case SIMDRV_ATR:
            {
            uint8_t ATRhistSz;
            if ((retval = get_user(ATRhistSz, (uint8_t *)arg)) == 0) {
                if (ATRhistSz > ATR_BUFFER_SIZE - 5) {
                    retval = -E2BIG;
                    break;
                }
                __copy_from_user((pDev->ATRptr)+5,(uint8_t *)(arg+1),(unsigned long)ATRhistSz);
                // reinitialize ATR to prevent override from SIMDRV_ATR_FULL
                *((pDev->ATRptr)+0) = 0x3B;
                *((pDev->ATRptr)+1) = 0x90;  // This byte is modified when size known
                *((pDev->ATRptr)+2) = 0x11;
                *((pDev->ATRptr)+3) = 0x40;
                *((pDev->ATRptr)+4) = 0xFF;

                *((pDev->ATRptr)+1) = 0x90 | ATRhistSz; // ATR_BUFFER_SIZE==20 and above test imply ATRhistSz fits in 4 bits
                pDev->ATRsize = ATRhistSz + 5;
            }
            }
            break;

        case SIMDRV_ATR_FULL:
            {
            uint8_t ATRhistSz;
                            
            if ((retval = get_user(ATRhistSz, (uint8_t *)arg)) == 0) {
                if (ATRhistSz > ATR_BUFFER_SIZE) {
                    retval = -E2BIG;
                    break;
                }
                __copy_from_user((pDev->ATRptr),(uint8_t *)(arg+1),(unsigned long)ATRhistSz);
                pDev->ATRsize = ATRhistSz;
                
              // Log insertion change
              printk(KERN_ALERT
                   "simdrv: phonesim%d: SIMDRV_ATR_FULL (ATRsize %d)\n",
                   pDev->m_PhoneId, pDev->ATRsize);                
            }
            }
            break;
        //
        // ioctl sends response to the phone
        //
        case SIMDRV_RESPONSE:
            {
                uint16_t    usRspSz;
                uint8_t     strBuf[RSP_BUFFER_SIZE];
                if ((retval = get_user(usRspSz, (uint16_t *)arg)) == 0)
                {
                    if (usRspSz > RSP_BUFFER_SIZE)
                    {
                        retval = -E2BIG;
                        break;
                    }
                    __copy_from_user(strBuf,(uint8_t *)(arg+2),(unsigned long)usRspSz);
                    // blocking send data to the phone
#ifdef SIM_DEBUG_TRACE
                    simdrv_DebugPrint("RSP", pDev->m_PhoneId, strBuf, (int)usRspSz);
#endif
                    // change status to wait command (blocking function)
                    simdrv_StopWaitingResponse(pDev);
                    // analyze response to diffrentiate NAK/ERROR/NULL situations
                    if ( pDev->m_LastClass != 0xFF && pDev->m_LastClass != 0x3B
                      && usRspSz > 2 )         // data. (error situation - always return 2 bytes )
                    {           // send ACK
                        // protection against unknown commands
                        if ( pDev->m_AckMode == SIMCONFIG_SEND_ACK)
                            retval = simdrv_SendResponse(pDev, &(pDev->m_LastInstruction), 1, 0);
                    }
                    retval = simdrv_SendResponse(pDev, strBuf, (int)usRspSz, 0);
                }
            }
            break;
        //
        // two ioctl (old and new) sends response to the phone and after last byte of response
        // is sent - sets new baudrate (using prepared baud rate divisor - old or baudrate - new )
        //
        case SIMDRV_RESPONSE_BAUDRATE:
            {
                struct SimResponseBaudRate    respBuff;
                if ( copy_from_user(&respBuff, (uint8_t *)arg, sizeof(struct SimResponseBaudRate)) )
                    retval = -EFAULT;
                else
                {
                    uint16_t    divisor = simdrv_CalculateDivisor(respBuff.m_BaudRate);

                    retval = simdrv_SendResponseEx(pDev, respBuff.m_Length, respBuff.m_Response, divisor);
                    if ( retval != 0 )
                        break;

                    // reconfigure UART
                    simdrv_SetUartDivisor(pDev, divisor);                    // enable interrupts

                    retval = 0;
                }
            }
            break;

        case SIMDRV_RESPONSE_NEGOTIATION:
            {
                struct SimResponseEx    respBuff;
                if ( copy_from_user(&respBuff, (uint8_t *)arg, sizeof(struct SimResponseEx)) )
                    retval = -EFAULT;
                else
                {
                    retval = simdrv_SendResponseEx(pDev, respBuff.m_Length, respBuff.m_Response, respBuff.m_BaudRateDivisor);
                    if ( retval != 0 )
                        break;
                    // reconfigure UART
                    simdrv_SetUartDivisor(pDev, respBuff.m_BaudRateDivisor);
                    retval = 0;
                }

            }
            break;

        case SIMDRV_CONFIG:
            {
                struct SimConfigHeader     config;
                if ( copy_from_user(&config, (uint8_t *)arg, sizeof(struct SimConfigHeader)))
                    retval = -EFAULT;
                else
                    retval = simdrv_Config(pDev, &config, (uint8_t *)arg);
            }
            break;
        case SIMDRV_ENABLE:
            {
                retval = 0;
                retval = simdrv_UartEnable(pDev, file);
            }
            break;
        case SIMDRV_DISABLE:
            {
                retval = 0;
                retval = simdrv_UartDisable(pDev, file);
            }
            break;            

            case SIMDRV_COPY_EVENTS:
            {
                int     nPhone = (int)arg;
                if ( nPhone >= 0 || nPhone < NUM_DEVICES)
                {
                    pDev->m_EventSrc = nPhone;
                    phoneSIMData[nPhone].m_PhoneForCopyEvents = pDev;
                }
                else
                {       // stop copying
                    if ( pDev->m_EventSrc != 1 )
                        phoneSIMData[pDev->m_EventSrc].m_PhoneForCopyEvents = NULL;
                }
            }
            break;
            case SIMDRV_MONITOR:
            {
                struct SimMonitor   cmd;

//                int tmp=PD->m_IoctlBuffer.cnt;
/*
if ( tmp > 50 )
{
    printk("In cnt=%d ",tmp);
{
    struct timeval      timeStamp;
    do_gettimeofday (&timeStamp);
    printk("%2.2ld.%6.6ld :MonitorOut=%d\n",timeStamp.tv_sec%100, timeStamp.tv_usec, cmd.m_Length);
}
}*/
                if (get_user(cmd.m_Length, (uint16_t *)arg) != 0)
                    return -EFAULT;

                // next call is blocking - at least one element will be returned
                if ( (retval = simdrv_ReadMonitor(pDev, &cmd)) != 0 )
                    return retval;
/*if ( tmp > 50 )
{
    printk("Out=%d cnt=%d ",cmd.m_Length, tmp);
{
    struct timeval      timeStamp;
    do_gettimeofday (&timeStamp);
    printk("%2.2ld.%6.6ld :MonitorOut=%d\n",timeStamp.tv_sec%100, timeStamp.tv_usec, cmd.m_Length);
}
}*/
                // copy to user m_Length elements
                if ( copy_to_user((void*)arg, &cmd, (cmd.m_Length + 1) * sizeof(uint16_t)) )
                    retval = -EFAULT;
            }
            break;

      case SIMDRV_MEM_READ:
      {
        AMP_Register    reg;
        if ( copy_from_user(&reg, (AMP_Register *)arg, sizeof(AMP_Register)))
            return -EFAULT;
        else
        {
          // No visibility of failure
          reg.res = dev_ioread8(pDev, reg.addr);
          retval = 0;

          // Log insertion change
          printk(KERN_ALERT
                 "simdrv: phonesim%d: SIMDRV_MEM_READ (addr 0x%x val 0x%x)\n",
                 pDev->m_PhoneId, reg.addr, reg.res);
        }
        if ( retval == 0 )
            if (copy_to_user((void*)arg, &reg, sizeof(reg)))
                return -EFAULT;
      }
      break;

      case SIMDRV_MEM_WRITE:
      {
        AMP_Register    reg;
            
        if ( copy_from_user(&reg, (AMP_Register *)arg, sizeof(AMP_Register)))
            return -EFAULT;
        else
        {
          // No visibility of failure
          dev_iowrite8(pDev, reg.res, reg.addr);
          retval = 0;

          // Log insertion change
          printk(KERN_ALERT
                 "simdrv: phonesim%d: SIMDRV_MEM_WRITE (addr 0x%x val 0x%x)\n",
                 pDev->m_PhoneId, reg.addr, reg.res);
        }
      }
      break;



        default:
            retval = -ENOSYS;
            break;
        }
    }
    return retval;
}
#ifndef SIM_DEV_BOARD
static int simdrv_proc_page(char *buf, char **start, off_t offset,
                        int count, int *eof, void *data)
{
    int         len = 0;
    int         ph;

    len += sprintf(buf+len,"%s\n", DRIVER_VERSION_STRING);
    for (ph = 0; ph < NUM_DEVICES; ++ph)
    {
        phoneSIMStruct * dev = phoneSIMData + ph;
        uint8_t     dll = dev_ioread8(dev, dev->m_Uart.dll);
        uint8_t     dlh = dev_ioread8(dev, dev->m_Uart.dlh);
        uint16_t    divisor = dll + (((uint16_t)dlh) << 8);

        len += sprintf(buf+len,"\nPhone %d.  Tx=%u bytes.  Rx=%u bytes, Baud Rate=%d\n",
                        dev->m_PhoneId, dev->tx_cnt, dev->rx_cnt,
                        simdrv_DivisorToBaudrate(divisor));

        len += sprintf(buf+len,"         rx_cnt_loss = %d\n", dev->rx_cnt_loss);
        len += sprintf(buf+len,"      ioctl_cnt_loss = %d\n", dev->ioctl_cnt_loss);
        len += sprintf(buf+len,"      bogus_uart_irq = %d\n", dev->bogus_uart_irq);
        len += sprintf(buf+len,"    tx_error_cnt_irq = %d\n", dev->tx_error_cnt_irq);
        len += sprintf(buf+len,"   lsr_error_cnt_irq = %d\n", dev->lsr_error_cnt_irq);
        len += sprintf(buf+len,"   spurious_uart_irq = %d\n", dev->spurious_uart_irq);
    }
    return len;
}
#endif

//
// UART specific function for new UART with hardware Tx FIFO
//  This function is used by IRQ
//
static inline int simdrv_FillTxFifoIrq(phoneSIMStruct * p_Dev)
{
    uint8_t tfs;
    // copy to Tx FIFO while it has space

    while ( (tfs = dev_ioread8(p_Dev, p_Dev->m_Uart.tfs)) > 0 )
    {
        while ( tfs-- > 0 )
        {

#ifdef SIM_DEBUG_INTERRUPT_TRACE
  printk("FillTxFifo IRQ %d#%X ", p_Dev->m_PhoneId, p_Dev->m_TxBuffer.m_Buffer[p_Dev->m_TxBuffer.m_nTxPtr]);
#endif
            // put next byte in Tx FIFO
            dev_iowrite8(p_Dev, p_Dev->m_TxBuffer.m_Buffer[p_Dev->m_TxBuffer.m_nTxPtr], p_Dev->m_Uart.rbr);
            p_Dev->tx_cnt++;               // statistic
            p_Dev->m_TxBuffer.m_nTxPtr++;
            // if we sent all bytes from buffer
            if ( p_Dev->m_TxBuffer.m_nTxPtr >= sizeof(p_Dev->m_TxBuffer.m_Buffer) )
            {
                if ( p_Dev->m_TxBuffer.m_BaudRateDivisor != 0 )  // set baudrate after all bytes is sent
                    simdrv_SetUartDivisor(p_Dev, p_Dev->m_TxBuffer.m_BaudRateDivisor);
                // tell IRQ Tx is done
                //p_Dev->m_TxBuffer.m_nTxPtr = sizeof(p_Dev->m_TxBuffer.m_Buffer);
                return 0;       // transmit finished
            }
        }
    }
    return 1;       // NOTE: different from simdrv_FillTxFifo
}

//
// UART specific function for new UART with hardware Tx FIFO
//
static inline int simdrv_FillTxFifo(phoneSIMStruct * p_Dev, uint8_t *p_strBuf, int p_nBufLen)
{
#ifdef SIM_DEBUG_INTERRUPT_TRACE
    uint8_t     *ptr = p_strBuf;
#endif
    uint8_t     tfs;
    uint8_t     nfs;

    // copy to Tx FIFO while it has space

    tfs = dev_ioread8(p_Dev, p_Dev->m_Uart.tfs);
    while ( (tfs = dev_ioread8(p_Dev, p_Dev->m_Uart.tfs)) > 0 )
    {
        while ( tfs-- > 0 )
        {
            nfs = dev_ioread8(p_Dev, p_Dev->m_Uart.tfs);
            // put next byte in Tx FIFO
            dev_iowrite8(p_Dev, *p_strBuf++, p_Dev->m_Uart.rbr);
            p_Dev->tx_cnt++;               // statistic
            // if we do not have more bytes to send
            if ( --p_nBufLen <= 0 )
            {
#ifdef SIM_DEBUG_INTERRUPT_TRACE
    while ( ptr != p_strBuf )
        printk("%dx%X ", p_Dev->m_PhoneId, *ptr++);
#endif
    nfs = dev_ioread8(p_Dev, p_Dev->m_Uart.tfs);
                return 0;
            }
        }
    }
#ifdef SIM_DEBUG_INTERRUPT_TRACE
    while ( ptr != p_strBuf )
        printk("contd .. %dT%X ", p_Dev->m_PhoneId, *ptr++);
#endif

    return p_nBufLen;
}
//
//  send response implementation for UART with Tx FIFO
//
static int simdrv_Uart2SendResponse(phoneSIMStruct * p_Dev, uint8_t *p_strBuf,
                                    int p_nBufLen, uint16_t p_BaudRateDivisor)
{
    //protection against sending when file is already closed
    if ( p_Dev->m_TxStatus == SIM_IO_STOP )
        return 0;

    // wait untill buffer is in use
    if (down_interruptible (&p_Dev->m_TxBuffer.m_TxSemaphore))
        return -ERESTARTSYS;

    // enable Tx interrupt - initiate transmit
    // TODO for UART2 Tx should be always enabled
    simdrv_isr_tx_enable(p_Dev);

#ifdef SIM_DEBUG_TRACE
    simdrv_DebugPrint("send2 in", p_Dev->m_PhoneId, p_strBuf, (int)p_nBufLen);
#endif
    // do not execute tasklet(bottom half) for a while to prevent racing
    tasklet_disable(&p_Dev->m_RxTasklet);

    // check if ATR in process. If so flush the requesed data
    if ( p_Dev->m_TxBuffer.m_DataType == TX_COMMAND )
    {
        //
        int leftCnt = simdrv_FillTxFifo(p_Dev, p_strBuf, p_nBufLen);
        if ( leftCnt == 0 )
        {   // FIFO is big enough for all Tx bytes
            if ( p_BaudRateDivisor != 0 )  // set baudrate after all bytes is sent
                simdrv_SetUartDivisor(p_Dev, p_BaudRateDivisor);
            // tell IRQ Tx is done
            p_Dev->m_TxBuffer.m_nTxPtr = sizeof(p_Dev->m_TxBuffer.m_Buffer);

            p_Dev->m_TxBuffer.m_DataType = TX_COMMAND;
            // allow ioctl to process next "send response"
            // semaphore up is allowed in IRQ - as not sleeping function
            up (&p_Dev->m_TxBuffer.m_TxSemaphore);
            tasklet_enable(&p_Dev->m_RxTasklet);
            return 0;       // transmit finished
        }
        //printk("send2 out - Tx FIFO/Buffered %d/%d.\n", (p_nBufLen - leftCnt), leftCnt);

        p_Dev->m_TxBuffer.m_BaudRateDivisor = p_BaudRateDivisor;
        p_Dev->m_TxBuffer.m_nTxPtr = sizeof(p_Dev->m_TxBuffer.m_Buffer) - leftCnt;

        // copy to the end of the transmit buffer
        memcpy(&p_Dev->m_TxBuffer.m_Buffer[p_Dev->m_TxBuffer.m_nTxPtr],
               &p_strBuf[p_nBufLen - leftCnt], leftCnt);
    }

    // IRQ will set semaphore UP after last byte from Tx buffer will be sent to Tx FIFO

    // allow taskled(IRQ bottom half) to be executed
    tasklet_enable(&p_Dev->m_RxTasklet);

    return 0;
}

//
//
//
static void simdrv_Uart2SendAtr(phoneSIMStruct * p_Dev)
{
    // NOTE tasklet will enable Tx interrupt
    uint8_t     *atrBuf = p_Dev->ATRptr;
    int         atrSize = (int)(p_Dev->ATRsize);
    int leftCnt;

    // printk("ATR Command -send through uart2\n");
    //protection against sending when file is already closed
    if ( p_Dev->m_TxStatus == SIM_IO_STOP )
        return;

    simdrv_isr_tx_enable(p_Dev);      //always enabled for UART2 - for noe

    p_Dev->m_TxBuffer.m_DataType = TX_ATR;
    p_Dev->m_TxBuffer.m_BaudRateDivisor = 0;


    // first - clear Tx FIFO
    dev_iowrite8(p_Dev, FCR_FIFO_ENABLE | FCR_TX_FIFO_RST, p_Dev->m_Uart.fcr);

    leftCnt = simdrv_FillTxFifo(p_Dev, atrBuf, atrSize);
    // whole ATR sequence fit in Tx FIFO - sendng done and no IRQ needed
    if ( leftCnt == 0 )
    {   // FIFO is big enough for all Tx bytes
        // tell IRQ Tx is done
        p_Dev->m_TxBuffer.m_nTxPtr = sizeof(p_Dev->m_TxBuffer.m_Buffer);

        p_Dev->m_TxBuffer.m_DataType = TX_COMMAND;
        // allow ioctl to process next "send response"
        // semaphore up is allowed in IRQ - as not sleeping function
        up (&p_Dev->m_TxBuffer.m_TxSemaphore);
        return;       // transmit finished
    }

    p_Dev->m_TxBuffer.m_nTxPtr = sizeof(p_Dev->m_TxBuffer.m_Buffer) - leftCnt;
    // copy to the end of the transmit buffer
    memcpy(&p_Dev->m_TxBuffer.m_Buffer[p_Dev->m_TxBuffer.m_nTxPtr],
           &atrBuf[atrSize - leftCnt], leftCnt);
}

/*
 *  schedued task - is runned by timer in case we need to send T=0 NULL byte
 */
static void simdrv_SendNull(unsigned long ptr)
{
    struct phoneSIMStruct   *dev = (struct phoneSIMStruct*)ptr;
    uint8_t                 bufNull[2];

    bufNull[0] = 0x60;

    // send response is protected against racing by semaphore
    simdrv_SendResponse(dev, bufNull, 1, 0);
    // and restart timer
    simdrv_StartTimer(dev);
}
//
// function does blocking read from UART.
// Function processing events as well as received bytes.
// If event happens - function return array with single element - event
//  and flush circular buffer
// Return - read buffer length ( element count)
//
// NOTES
//   - p_strBuf size == Circular buffer size - no if needed
//   - separate processing for RESET event - continue to wait ?????
//

//
//  returns to user "raw" data from UART
//  IOCTL is used for phone <=> SIM communication monitoring
//

static int simdrv_ReadMonitor(phoneSIMStruct * p_Dev, struct SimMonitor *p_Command)
{
    int         nRetval = 0;
    int         nMaxLength = p_Command->m_Length;
    register int nCnt;

    //p_Command->m_Length  already contains Length
    if ( p_Command->m_Length  <= 0 )
        return -EINVAL;

    // wait at least one "raw" element
    if ( (nRetval = wait_event_interruptible_timeout(p_Dev->ioctl_wake_queue,
            (   (cb_GetElementCount(&p_Dev->m_IoctlBuffer) > 0)     // at least one element was read
             || (atomic_read(&p_Dev->m_EventCnt) > 0)),         // or event happened
            read_timeout_ms*HZ/1000)) < 0 )
        return -ERESTARTSYS;                //sleep was interrupted by some sort of a signal

    //  process timeout
    if (nRetval == 0)
    {   // timeout BUT
        // create timeout event
        p_Command->m_Length = 1;
        p_Command->m_Command[0] = TIMEOUT_EVENT << 8;
        return 0;
    }

    nCnt = cb_GetElementCount(&p_Dev->m_IoctlBuffer);

    if ( nCnt > nMaxLength )
        nCnt = nMaxLength;

    p_Command->m_Length = cb_GetBlock(&p_Dev->m_IoctlBuffer, p_Command->m_Command, nCnt);

    // adjust event count
    if ( atomic_read(&p_Dev->m_EventCnt) > 0 )
    {
        int     i;

        for ( i = 0; i < p_Command->m_Length; i++)
        {
            if ( BUF_EVENT_PART(p_Command->m_Command[i]) != 0 )
                atomic_dec(&p_Dev->m_EventCnt);             // -- event flag
        }
    }

    return 0;
}
//
//  parses TA or PPS1 byte and calculates baud rate
//  p_PhoneFreq - should be in Hz
//
static inline int simdrv_CalculateBaudRate (uint8_t  p_byFdValue, int p_PhoneFreq)
{
    static int   ClockRateConversion[16] =
                    {  372,  372,  558,  744, 1116, 1488, 1860, -1,
                        -1,  512,  768, 1024, 1536, 2048,   -1, -1};
    static int   BaudRateAdjustment[16]  =
                    { -1,  1,  2,  4,  8, 16, 32, 64,
                      12, 20, -1, -1, -1, -1, -1, -1};

    int     F = (p_byFdValue >> 4) & 0x0F;
    int     D =  p_byFdValue & 0x0F;
    if ( ClockRateConversion[F] == -1 || BaudRateAdjustment[D] == -1 )
    {
        printk( "Baud rate calculation error: Wrong F or D value\n");
        return -1;
    }
    return  (p_PhoneFreq * BaudRateAdjustment[D] + ClockRateConversion[F]/2)/ClockRateConversion[F];
}
//
//  function takes PPS request and PPS response
//  and calculates negotiated UART speed or error negotiation
//      NOTE: ISO does not define PS2 and PS3 bytes -
//            function does not process these bytes
//
static inline int simdrv_ComparePpsMessages(
                            uint8_t *p_PpsRequest,  uint16_t p_ReqLength,
                            uint8_t *p_PpsResponse, uint16_t p_RespLength)
{
    uint16_t        i;

    // check if SIM wants use default baud rate
    if ( p_RespLength == 3
      && p_PpsResponse[0] == 0xFF
      && p_PpsResponse[1] == 0x00
      && p_PpsResponse[2] == 0xFF )
        return 1;                   // default baud rate

    // current version ISO defines that in PPS response
    // PS2 and PS3 bytes should be exactly the same as in request
    // either non present or have the same value
    // This definition leaves 2 possible response lengths 3 and 4
    // We processed 3 in the previouse if statment
    if ( p_ReqLength != p_RespLength )
    {
        printk("ERROR: PPS Request and Response have different length\n");
        return -EPROTO;                  // unsuccessful PPS exchange
    }

    for ( i = 0; i < p_ReqLength; i++ )
    {
        if ( p_PpsRequest[i] != p_PpsResponse[i])
        {
            printk("ERROR: PPS Request and Response not identical. Byte # %d\n", i);
            return -EPROTO;             // unsuccessful PPS exchange
        }
    }

    return 0;       // p_PpsResponse[2] has agreed F and D for new baud rate
}
//
//  function is used to calculate Control byte for ATR and PPS records
//  Function does Exclusive OR of p_Buffer bytes 0 to p_nCkPosition-1
//  and put result to p_Buffer[p_nCkPosition]
//
static inline void simdrv_CalculateCheckByte(uint8_t *p_Buffer, int p_nCkPosition)
{
    int     i = 0 ;
    p_Buffer[p_nCkPosition] = 0;
    while ( i != p_nCkPosition )
        p_Buffer[p_nCkPosition] ^= p_Buffer[i++];
}
//
//  reads ATR from SIM
//      returns < 0 - Linux error
//              > 0 - Event (event code)
//              == 0 - OK;
//
static inline
int simdrv_ReadAtr(phoneSIMStruct *p_Dev, struct SimAtr * p_Atr)
{
    int                 nRetval = 0;
    struct T0Parser     parser = {4, 0, 0};
    BufferElement       strBuf[MAX_CMD_SIZE];
    struct SimCommand   command;
    struct SimCommand   *p_Command = &command;      // for now

    // initialize return structure fields
    p_Command->m_Length = 0;
    p_Command->m_Event  = CMD_OR_PAYLOAD;

    // read class byte
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, 1)) < 0 )
        return nRetval;                 // Linux Error

    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
#ifdef SIM_DEBUG_TRACE
        printk("simdrv: m_Event = %d \n", p_Command->m_Event);
#endif
        p_Command->m_Length = 0;
        return p_Command->m_Event;       // event
    }
    p_Command->m_Command[p_Command->m_Length++] = BUF_VALUE_PART(strBuf[0]);

    // process first ATR byte
    switch  (p_Command->m_Command[0])
    {
        case 0x3B:              //ATR direct  coding convention
        case 0x3F:              //ATR inverse coding convention
            nRetval = simdrv_Atr(p_Dev, p_Command, &parser);
            if ( nRetval != 0 )                 // Linux error
                return nRetval;
            if ( p_Command->m_Event != 0 )      // T=0 event
                return p_Command->m_Event;
        break;
        case 0x03:              // ATR in inverse coding convention WHEN read in direct coding convention
                                // reconfigure coding convention and reset SIM
            printk("Got Inverse Encoding in Direct Read Mode!.\n");
            nRetval = -EPROTO;
        break;
        default:
            printk("Error: ATR Command 0x%2.2X is not supported\n", p_Command->m_Command[0]);
            nRetval = -EPROTO;
    }
    //transfer result from simdrv ioctl structure to tequal0 ioctl structure - fix later
    p_Atr->m_TckPresent = (parser.m_nTckPresent != 0 ) ? 1 : 0;
    p_Atr->m_AtrSize = p_Command->m_Length - parser.m_nHistLen;  // parser.m_nHistLen => history length + TCK if present
    memcpy(p_Atr->m_Atr, p_Command->m_Command, p_Atr->m_AtrSize);
    p_Atr->m_HistorySize = parser.m_nHistLen - p_Atr->m_TckPresent;
    memcpy(p_Atr->m_History, &p_Command->m_Command[p_Atr->m_AtrSize], p_Atr->m_HistorySize);
    p_Atr->m_TckValue = p_Command->m_Command[p_Command->m_Length-1];
    return nRetval;
}


//
//  Sends PPS message to the SIM process SIM response set up UART baud rate
//      returns < 0 - Linux error
//              > 0 - Event (event code)
//              == 0 - OK;
//
static inline
int simdrv_ExecutePps(phoneSIMStruct *p_Dev, uint8_t p_byBaudRate)
{
    int                 nRetval = 0;
    struct T0Parser     parser = {4, 0, 0};
    BufferElement       strBuf[MAX_CMD_SIZE];
    struct SimCommand   command;
    struct SimCommand   *p_Command = &command;      // for now ????
    uint8_t     pps[] = {0xFF, 0x10, 0x94, 0x7B};

    // prepare pps
    pps[2] = p_byBaudRate;
    // recalculate PCK - check character
    simdrv_CalculateCheckByte(pps, 3);
    // Send PPS to the SIM
    nRetval = simdrv_SendResponse(p_Dev, pps, sizeof(pps), 0);
    if ( nRetval != 0 )
        return nRetval;     // error while send

    // get SIM response
    // initialize return structure fields
    p_Command->m_Length = 0;
    p_Command->m_Event  = CMD_OR_PAYLOAD;

    // read class byte
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, 1)) < 0 )
    {
        //printk("PPS class read ERROR %d\n", nRetval);
        return nRetval;                 // Linux Error
    }
    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        //printk("PPS class read event 0x%X\n", strBuf[0]);
        return p_Command->m_Event;      // T=0 event
    }
    p_Command->m_Command[p_Command->m_Length++] = BUF_VALUE_PART(strBuf[0]);

    // process first PPS byte
    if (p_Command->m_Command[0] != 0xFF)
        return -EPROTO;                 // protocol error - SIM should send PPS message

    // get left PPS response bytes
    nRetval = simdrv_Pps( p_Dev, p_Command, &parser);
    if ( nRetval != 0 )                 // linux error
        return nRetval;
    if ( p_Command->m_Event != 0 )      // T=0 event
        return p_Command->m_Event;

    nRetval = simdrv_ComparePpsMessages(pps, sizeof (pps), p_Command->m_Command, p_Command->m_Length);
    if ( nRetval == 0 )                 // use a new baud rate
    {
        // calculate new baud from F and D byte
        int nNewBaud = simdrv_CalculateBaudRate (pps[2], SIM_FREQ);
        if ( nNewBaud < 0 )
            return -EDOM;               // problems with baudrate calculation

        // Reset current divisor to insure new baud rate is applied.
        g_uartDivisor[p_Dev->m_PhoneId] = 0;

        printk("New Baudrate(%d) = %d\n", p_Dev->m_PhoneId, nNewBaud);
        // reconfigure UART
        simdrv_SetUartDivisor(p_Dev, simdrv_CalculateDivisor(nNewBaud));
    }
    if ( nRetval == 1 )
        nRetval = 0;                    // stay on default baudrate

    return nRetval;                     // < 0 - Linux error; 0 - OK
}

//
//  Timer function - simulates SIM Insert/Remove IRQ
//                   for all phones
//
#define SIMDRV_CHECK_INTERVAL 30  // every 15 seconds

static void
simdrv_TimerFunctionWorker(
struct work_struct *work_arg
)
{
  int     sim;
  unsigned long p_nJifs;
  struct work_cont *c_ptr;
  static int logCount = 0;
  
  c_ptr = container_of(work_arg, struct work_cont, real_work);

  // Increment log count
  logCount++;
  
  // If matched inerval...
  if (logCount >= SIMDRV_CHECK_INTERVAL)
  {
    // Log insertion change
    printk(KERN_ALERT
           "simdrv: checking SIM insertion/removal\n");
    
    // Reset
    logCount = 0;
  }

  for ( sim = 0; sim < NUM_DEVICES; sim++ )
  {
      phoneSIMStruct      *dev = &phoneSIMData[sim];

// For now skip AMP logic untill details figured out
#ifdef SIM_DEV_ENABLE_WORKAROUNDS
      if (sim >= NUM_PHONES)
      {
        continue;
      }
#endif

      volatile uint8_t    bySimPresent = dev_IsSimPresent(dev);
      if ( bySimPresent != dev->m_SimPresent)
      {
          // Log insertion change
          printk(KERN_ALERT
                 "simdrv: phonesim%d: insertion (%d -> %d)\n",
                 sim, dev->m_SimPresent, bySimPresent);

          dev->m_NeedsReset = 1;
          dev_PowerSim(dev, STATE_OFF);

          dev->m_ActualIrEvents++;

          // IRQ simulation
          dev->m_SimPresent = bySimPresent;
      }
  }

  // Access interval
  p_nJifs = (unsigned long) (c_ptr->arg);
  
  // Restart timer
  timer_IrCheck.expires = jiffies + p_nJifs;
  add_timer(&timer_IrCheck);
}


static void
simdrv_TimerFunction(unsigned long p_nJifs)
{
  // Acknowledge interrupt
  iowrite8(0x00, ssw_irq_reg);
  iowrite8(0x01, ssw_irq_reg);
    
  // Schedule work queue
  simdrv_timer_wq.arg = (int) p_nJifs;
  schedule_work(&simdrv_timer_wq.real_work);
}

//
//
//
static inline void simdrv_StartInsertTimer(const unsigned long p_nJifs)
{
    /* initialize LCD init timer structure*/
    init_timer(&timer_IrCheck);
    timer_IrCheck.function = simdrv_TimerFunction;
    timer_IrCheck.data = (unsigned long)p_nJifs;    // 500 msec
    timer_IrCheck.expires = jiffies + p_nJifs;
    add_timer(&timer_IrCheck);
}

//
// Function should be called after simdrv_ReadRawDataBlock
//          to check if event happened and if so - correctly
//          flush ioctl buffer
//
static uint16_t simdrv_IsEvent(phoneSIMStruct * p_Dev, //struct SimCommand *p_Command,
            BufferElement   *p_strBuf, int p_nBufCnt)
{
    uint16_t    usEvent;
    int         nEventCnt;

    //int tmpCnt = cb_GetElementCount(&p_Dev->m_IoctlBuffer);
    // check if there is an event in the buffer
    // and flush the buffer
    if ( (nEventCnt = atomic_read(&p_Dev->m_EventCnt)) > 0 )
    {
        // we have data in both p_strBuf and ioctl circular buffer
        uint16_t        usStoreEvent = 0;
        BufferElement   usValue;
        int             i;

        for ( i = 0; i < p_nBufCnt; i++ )
        {
            usEvent = BUF_EVENT_PART(p_strBuf[i]) >> 8;
            if ( usEvent != 0 )
            {
                atomic_dec(&p_Dev->m_EventCnt);             // -- event flag
                usStoreEvent = usEvent;

                // after reset event phone will send
                // correct command ==> DO NOT flush collected bytes/events
                if ( usEvent == SIM_RESET_EVENT )
                {
                    //p_Command->m_Length = 0;    // event flag
                    //p_Command->m_Event  = usEvent;
                    return usEvent;
                }
            }
        }
        // event happens - flush the buffer
        while ( cb_GetElement(&p_Dev->m_IoctlBuffer, &usValue) != 0 )
        {
            usEvent = BUF_EVENT_PART(usValue) >> 8;
            if ( usEvent != 0 )
            {
                atomic_dec(&p_Dev->m_EventCnt);             // -- event flag
                usStoreEvent = usEvent;

                // after reset event phone will send
                // correct command ==> DO NOT flush collected bytes/events
                if ( usEvent == SIM_RESET_EVENT )
                    break;
            }

        }
        //p_Command->m_Length = 0;    // event flag
        //p_Command->m_Event  = usStoreEvent;
        return usStoreEvent;
    }
    return 0;
}
//
//
//
static inline void simdrv_CopyRawToCommand(struct SimCommand *p_Command,
                                  BufferElement *p_strBuf, int p_nBufCnt)
{
    int     i;
    // store command header bytes to p_Command
    for ( i = 0; i < p_nBufCnt; i++)
        p_Command->m_Command[p_Command->m_Length++] = BUF_VALUE_PART(p_strBuf[i]);

}
//
//  process ATR sequence
//
static inline int simdrv_Atr( phoneSIMStruct * p_Dev, struct SimCommand *p_Command,
                              struct T0Parser *p_Parser)
{
    int             nRetval = 0;
    uint8_t         byYn;
    BufferElement   strBuf[10];
    int             nLength;
    int             nNextTdFlag = 1;        // if 1 expect another interface bytes block

    p_Parser->m_nTckPresent = 0;
    //read T0 byte
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, 1)) < 0 )
        return nRetval;         // Linux Error

    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        return 0;               // event is packed to the p_Command to be deliverd to application
    }
    // store T0 byte to p_Command
    p_Command->m_Command[p_Command->m_Length++] = BUF_VALUE_PART(strBuf[0]);

    byYn = p_Command->m_Command[1];     //get next Y(n) byte
    p_Parser->m_nHistLen = byYn & 0x0F; // add number of historical bytes

    do
    {
        nLength  = 0;               // length of the next chank of the interface bytes

        if ( byYn & (1<<4) )
            nLength++;              // TA included
        if ( byYn & (1<<5) )
            nLength++;              // TB include
        if ( byYn & (1<<6) )
            nLength++;              // TC included

        if ( byYn & (1<<7) )
            nLength++;              // TD included
        else            // do not expect another TD byte
            nNextTdFlag = 0;

        // process T - to figure out if we have TCK byte ( if protocol != 0 )
        if ( p_Command->m_Length != 2 )
            p_Parser->m_nTckPresent += (byYn & 0xF);

        // read next portion of interface bytes
        if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, nLength)) < 0 )
            return nRetval;         // Linux Error

        // event processing
        if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
        {
            p_Command->m_Length = 0;
            return 0;               // event is packed to the p_Command to be deliverd to application
        }
        // store command header bytes to p_Command
        simdrv_CopyRawToCommand(p_Command, strBuf, nRetval);

        // set new value to byYn
        byYn = BUF_VALUE_PART(strBuf[nLength-1]);
    } while ( nNextTdFlag != 0 );

    if (p_Parser->m_nTckPresent)
        p_Parser->m_nHistLen++;

    // read history bytes ( plus TCK byte if requested)
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, p_Parser->m_nHistLen)) < 0 )
        return nRetval;         // Linux Error

    // event processing
    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        return 0;               // event is packed to the p_Command to be deliverd to application
    }
    // store command header bytes to p_Command
    simdrv_CopyRawToCommand(p_Command, strBuf, nRetval);

    return 0;
}
//
//  process next byte from PPS sequence
//      returns < 0 - Linux error
//              == 0 - OK or Event;
//
static inline int simdrv_Pps( phoneSIMStruct * p_Dev, struct SimCommand *p_Command,
                               struct T0Parser *p_Parser)
{
    int             nRetval = 0;
    uint8_t         byIns;
    BufferElement   strBuf[10];
    int             nLength;

    //read PPS0 byte
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, 1)) < 0 )
        return nRetval;         // Linux Error

    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        return 0;               // event is packed to the p_Command to be deliverd to application
    }
    // store PPS0 byte to p_Command
    p_Command->m_Command[p_Command->m_Length++] = BUF_VALUE_PART(strBuf[0]);

    byIns = p_Command->m_Command[1];

    // we already get two first bytes PPSS and PPS0
    nLength = 1;                        //PPSS + PPS0 .... + PCK
    if ( byIns & (1<<4) )               // PPS1 included
        nLength++;
    if ( byIns & (1<<5) )               // PPS2 included
        nLength++;
    if ( byIns & (1<<6) )               // PPS3 included
        nLength++;

    //read tail of PPS string
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, nLength)) < 0 )
        return nRetval;         // Linux Error

    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        return 0;
    }
    // store command header bytes to p_Command
    simdrv_CopyRawToCommand(p_Command, strBuf, nRetval);

    return 0;
}

//
// continue command input + processing
//  we already have command class read and checked
//
static inline int simdrv_PhoneCommand( phoneSIMStruct * p_Dev, struct SimCommand *p_Command,
                                       struct T0Parser *p_Parser)
{
    int             nRetval = 0;
    uint8_t         byIns;
    BufferElement   strBuf[MAX_CMD_SIZE];
    uint8_t         byCommandType;
    int             nLength;

    //read four next/last command header bytes : INS, P1, P2, P3(length)
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, 4)) < 0 )
        return nRetval;         // Linux Error

    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        return 0;               // event is packed to the p_Command to be deliverd to application
    }
    // store command header bytes to p_Command
    simdrv_CopyRawToCommand(p_Command, strBuf, nRetval);

    // collect some usfull values to the variables
    byIns   = p_Command->m_Command[1];              // instruction code
    nLength = p_Command->m_Command[4];              // length byte
    byCommandType = p_Dev->m_CommandMap[byIns];     // command type
    // check if command is listed in the list of allowed commands
    if ( byCommandType == SIMCMD_NOT_DEFINED ) // command not in the list
    {
        printk("Error: Command 0x%2.2X is not allowed\n", byIns);
        return 0;
    }
    if ( byCommandType == SIMCMD_ONLY_HEADER )
    {
        printk("Error: Command 0x%2.2X does not expects length\n", byIns);
        return 0;
    }

    // send ACK - simdrv_send_response - synch by semaphore

    if (p_Dev->m_CommandMap[byIns] == SIMCMD_PAYLOAD )
    {
        // NOTE: I am not shure that next "if" is always correct
        if ( nLength != 0 )
            nRetval = simdrv_SendResponse(p_Dev, &byIns, 1, 0);

        //read/get command payload
        if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, nLength)) < 0 )
            return nRetval;         // Linux Error

        if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
        {
            p_Command->m_Length = 0;
            return 0;               // event is packed to the p_Command to be deliverd to application
        }
        // store command header bytes to p_Command
        simdrv_CopyRawToCommand(p_Command, strBuf, nRetval);


    }
    else
    {       //"request SIM data" command
        // change to "wait response" mode
        p_Dev->m_T0Mode = T0_WAIT_RESPONSE;
        // and remember INS byte (command code)
        p_Dev->m_LastInstruction = byIns;
        // activate sending each second NULL byte(0x60)
        simdrv_StartTimer(p_Dev);
    }

    return 0;
}
#ifdef SEND_DEBUG_MESSAGES
//
//
//
static inline void simdrv_AddDebugMsg(phoneSIMStruct * p_Dev, struct SimCommand *p_Command)
{
    if ( strlen(p_Dev->m_DebugMsg) != 0 )
    {
        p_Command->m_DebugLength = strlen(p_Dev->m_DebugMsg);
        strncpy(p_Command->m_DebugMsg, p_Dev->m_DebugMsg, p_Command->m_DebugLength+1);
        //clear buffer
        p_Dev->m_DebugMsg[0] = '\0';
    }
    else
        p_Command->m_DebugLength = 0;
}
#endif
//
// simdrv_ReadFullCommand for UART with "receive trigger level"
//
static int simdrv_ReadCommandFull(phoneSIMStruct * p_Dev, struct SimCommand *p_Command)
{
    int                 nRetval = 0;
    struct T0Parser     parser = {4, 0, 0};
    BufferElement       strBuf[MAX_CMD_SIZE];
    int                 count = 0;
    int                 index = 0;

    // initialize return structure fields
    p_Command->m_Length = 0;
    p_Command->m_Event  = CMD_OR_PAYLOAD;

    // read class byte
    if ( (nRetval = simdrv_ReadRawDataBlock(p_Dev, strBuf, 1)) < 0 )
        return nRetval;         // Linux Error

    if ( (p_Command->m_Event = simdrv_IsEvent(p_Dev, strBuf, nRetval)) != 0 )
    {
        p_Command->m_Length = 0;
        return 0;               // event is packed to the p_Command to be deliverd to application
    }
    p_Command->m_Command[p_Command->m_Length++] = BUF_VALUE_PART(strBuf[0]);

    switch  (p_Command->m_Command[0])
    {
        case 0xA0:
        case 0x00:
        case 0x80:
            nRetval = simdrv_PhoneCommand( p_Dev, p_Command, &parser);
        break;
        case 0xFF:              //PPS protocol
            nRetval = simdrv_Pps(p_Dev, p_Command, &parser);
        break;
        case 0x3B:              //ATR direct  coding convention
        case 0x3F:              //ATR inverse coding convention
            nRetval = simdrv_Atr(p_Dev, p_Command, &parser);
        break;
        case 0x03:              // ATR in inverse coding convention WHEN read in direct coding convention
                                // reconfigure coding convention and reset SIM
            printk("Got Inverse Encoding in Direct Read Mode!.\n");
            nRetval = -2;       // ??????
        break;
        default:
            printk("simdrv: Error: Command Class 0x%2.2X is not supported\n", p_Command->m_Command[0]);
            printk("simdrv: Ph%d Cmd read backtrace, ReadCnt=%d\n", p_Dev->m_PhoneId, p_Dev->m_numCmdReads);
            if (p_Dev->m_numCmdReads > 0)
            {
                // Log last CMD read and work backwards.
                index = (p_Dev->m_numCmdReads - 1) % MAX_LOGGED_CMDS;
                for (count = 0; count < MAX_LOGGED_CMDS; count++)
                {
                    if (p_Dev->m_CmdReadLog[index].m_len >= 0) {
                        simdrv_DebugPrint("simdrv: Cmd buf", p_Dev->m_PhoneId, &(p_Dev->m_CmdReadLog[index].m_Buffer[0]), p_Dev->m_CmdReadLog[index].m_len);
                    } else {
                        printk("simdrv: Cmd buf Ph%d read status=%d\n", p_Dev->m_PhoneId, p_Dev->m_CmdReadLog[index].m_len);
                    }

                    if (index > 0) {
                        index--;
                    } else {
                        index = MAX_LOGGED_CMDS - 1;
                    }
                }
            }
            simdrv_DumpUartRegs(p_Dev);
            nRetval = -1;       //??????
    }
    // remember for the next send response function
    p_Dev->m_LastClass = p_Command->m_Command[0];
    return nRetval;
}
//
//
//
static int simdrv_ReadCommand(phoneSIMStruct * p_Dev, uint8_t *p_ResBuf, uint8_t p_byCmdSz)
{
    BufferElement   Buf[MAX_CMD_SIZE];
    int             nResetCnt = -1;
    int             nReadLen;
    int             i;
    int             nRetVal = 0;

    do
    {
        nResetCnt++;
        // block waiting for incoming data
        nReadLen = simdrv_ReadPhoneData(p_Dev, Buf+1, p_byCmdSz);
        if ( nReadLen < 0 )
            return nReadLen;            // Linux Error

        p_ResBuf[0] = 0;
        // nReadLen or == cmdSz or 1 - if event was returned
        for ( i = 1; i <= nReadLen; i++)
        {
            p_ResBuf[i] = BUF_VALUE_PART(Buf[i]);
            if ( BUF_EVENT_PART(Buf[i]) != 0 )
            {   // TODO - copy could be done to the same buffer
                p_ResBuf[0] = (uint8_t)(BUF_EVENT_PART(Buf[i]) >> 8);
                break;
            }
        }
        nRetVal = nReadLen;
    } while ( (nResetCnt < 2) && (p_ResBuf[0] == SIM_RESET_EVENT) );

    if (nResetCnt >= 2)
        p_ResBuf[0] = SIM_RESET_EVENT | (1<<7);

    // special case - payload after reset
    if ( p_ResBuf[0] == CMD_OR_PAYLOAD &&  nResetCnt != 0)
        p_ResBuf[0] = SIM_RESET_EVENT;

    return nRetVal;
}

static int simdrv_ReadPhoneData(phoneSIMStruct * p_Dev, BufferElement *p_strBuf, int p_nCmdLen)
{
    uint16_t    usEvent;
    int         nLen;

    // Block read p_nCmdLen elements or less
    nLen = simdrv_ReadRawDataBlock(p_Dev, p_strBuf, p_nCmdLen);

    if ( ( usEvent = simdrv_IsEvent(p_Dev, p_strBuf, nLen)) != 0 )
    {
        p_strBuf[0] = usEvent<<8 | BUF_VALUE_PART(p_strBuf[0]);
        return p_nCmdLen;               // event is packed to the p_Command to be deliverd to application
    }
    return nLen;
}
//
//
// Return - read buffer length ( element count) or Linux negative error
//
//  NOTE: - this function does not process an event.
//          If event happened function returns requested
//          number of bytes or less. Calling function
//          should check return value and p_Dev->m_EventCnt
//          and react correspondently to events
//        - function can generate it's own event - TIMEOUT event
//          this event will be put into the p_strBuf
//
static int simdrv_ReadRawDataBlock(phoneSIMStruct * p_Dev, BufferElement *p_strBuf, int p_nLen)
{
    int         nRetval;
    int         nLeft = p_nLen;     //should be unsigned ???
    int         nReadCnt;
    int         nChunk;
    int         i = 0;
    int         nIndex;
    //struct timeval  tv_before, tv_after;

#ifdef SIM_DEBUG_INTERRUPT_TRACE
if ( p_nLen == 4 )
printk("simdrv_ReadRawDataBlock  len=0x%X\n", p_nLen);
#endif

    do
    {
        // read data from the circular buffer
        nReadCnt = cb_GetBlock(&p_Dev->m_IoctlBuffer, &p_strBuf[p_nLen - nLeft], nLeft);

        // Maintain last few phone commands for debugging failures.
        nIndex = (p_Dev->m_numCmdReads % MAX_LOGGED_CMDS);
        if (nReadCnt >= 0) {
            for (i = 0; i < nReadCnt; i++)
            {
                p_Dev->m_CmdReadLog[nIndex].m_Buffer[i] = BUF_VALUE_PART(p_strBuf[i]);
#ifdef SIM_DEBUG_INTERRUPT_TRACE
printk("simdrv_ReadRawDataBlock  bufvalue part=0x%X\n", BUF_VALUE_PART(p_strBuf[i]));
printk("simdrv_ReadRawDataBlock  value=0x%X\n", p_strBuf[i]);
#endif
            }
            //simdrv_DebugPrint("ReadRaw", p_Dev->m_PhoneId, &(p_Dev->m_CmdReadLog[nIndex].m_Buffer[0]), nReadCnt);
        } else {
            //printk("ReadRaw nReadCnt=%d\n", nReadCnt);
        }
        p_Dev->m_CmdReadLog[nIndex].m_len = nReadCnt;
        ++p_Dev->m_numCmdReads;

        if ( nReadCnt == nLeft )
        {
            // set UART Receive trigger level to maximum
            dev_iowrite8(p_Dev, 0x7F, p_Dev->m_Uart.rtl);     //0x7F
            return p_nLen;
        }

        if ( atomic_read(&p_Dev->m_EventCnt) > 0 )
        {   // in case of event - we can not expect to receive all
            // requested/needed bytes - so return what we've got
            // set UART Receive trigger level to maximum
            dev_iowrite8(p_Dev, 0x7F, p_Dev->m_Uart.rtl);     //0x7F
#ifdef SIM_DEBUG_TRACE
            printk("simdrv: ReadRaw Event detected! req len=0x%X nReadCnt=0x%X\n", p_nLen, nReadCnt);
#endif
            return p_nLen - nLeft + nReadCnt;
        }
        // we need more data
        nLeft -= nReadCnt;
        nChunk = nLeft;
        // set new UART Receive trigger level
        //nChunk = (nLeft < 150) ? nLeft : 150;
        dev_iowrite8(p_Dev, (uint8_t)(nChunk-1), p_Dev->m_Uart.rtl);

        //do_gettimeofday (&tv_before);
        // and wait for data to arrive
        if ( (nRetval = wait_event_interruptible_timeout(p_Dev->ioctl_wake_queue,
                (   (cb_GetElementCount(&p_Dev->m_IoctlBuffer) >= 1)    //nChunk
                 || (atomic_read(&p_Dev->m_EventCnt) > 0)),
                read_timeout_ms*HZ/1000)) < 0 )
        {
#ifdef SIM_DEBUG_TRACE
            printk("simdrv: ReadRaw Interrupted! req len=0x%X nReadCnt=0x%X\n", p_nLen, nReadCnt);
#endif
            return -ERESTARTSYS;                //sleep was interrupted by some sort of a signal
        }
        //do_gettimeofday (&tv_after);

        if (nRetval == 0)
        {   // timeout BUT
            // buffer could be not empty  !!! f.ex. - lost character - though it should be event in this case
            // flush the buffer
            cb_ClearBufferLock(&p_Dev->m_IoctlBuffer);     // flush buffer and all events in it
//            atomic_set(&p_Dev->m_EventCnt,0);         // buffer is empty - no events
            atomic_set(&p_Dev->m_EventCnt,1);           // buffer is empty - only timeout event
            p_strBuf[0] = TIMEOUT_EVENT << 8;
            //printk("Bf %3.3ld.%6.6ld; Af %3.3ld.%6.6ld \n",tv_before.tv_sec%1000, tv_before.tv_usec, tv_after.tv_sec%1000, tv_after.tv_usec);
            //printk("simdrv: Read Timeout(%d)\n", p_Dev->m_PhoneId);
            return 1;       // return one element with timeout event
        }
    } while ( nLeft > 0 );

    // function should return from inside the loop -
    // and never reach this point
    dev_iowrite8(p_Dev, 0x7F, p_Dev->m_Uart.rtl);     //0x7F
    return p_nLen;
}
//
//  simdrv_read_command wrapper for communication with SIM only
//      return  0 => OK p_nReadLen bytes was read
//              <0 -  Linux error
//              >0 - Communication error happend
//
static int simdrv_ReadData(phoneSIMStruct *p_Dev, uint8_t *p_Buff, int p_nReadLen)
{
    BufferElement   strBuf[MAX_CMD_SIZE];
    int             nReadCnt;
    int             nRetVal = 0;
    int             i;

    if ( (nReadCnt = simdrv_ReadRawDataBlock(p_Dev, strBuf, p_nReadLen)) < 0 )
        return nReadCnt;         // Linux Error

    // copy received data to the structure
    for ( i = 0; i < nReadCnt; i++)
    {
        p_Buff[i] = BUF_VALUE_PART(strBuf[i]);
        if ( BUF_EVENT_PART(strBuf[i]) != 0 )
        {
            atomic_dec(&p_Dev->m_EventCnt);         // decrement event flag
            nRetVal = BUF_EVENT_PART(strBuf[i]);    // Communication error  happened
        }
    }
    return nRetVal;
}
//#endif
/*
 * structure with pointers to driver functions
 */
struct file_operations simdrv_fops =
{
    owner:      THIS_MODULE,
#ifndef SIM_DEV_BOARD
    ioctl:      simdrv_ioctl,
#else
    unlocked_ioctl:  simdrv_ioctl,
#endif
    open:       simdrv_open,
    release:    simdrv_release,
//    fasync:     simdrv_fasync,
};

/*
 * module initialization and cleanup
 */

static void simdrv_cleanup_module(void)
{
    int ph;

    switch (DriverState)
    {
    case Statistic:
#ifndef SIM_DEV_BOARD
        remove_proc_entry("sim", NULL /* parent dir */);
#endif
    case IRQ:
#ifndef SIM_DEV_BOARD
        free_irq(fpga_irq, &simdrv_dev);
#endif
    case RxTasklet:
    {
      // Logs
      printk(KERN_ALERT "simdrv: cleanup(RxTasklet)\n");

      //  kill tasklets
      for (ph = 0; ph < NUM_DEVICES; ++ph)
      {
        // If tasklet running...
        if (phoneSIMData[ph].m_RxTasklet.func)
        {
          // Kill it
          tasklet_kill(&phoneSIMData[ph].m_RxTasklet);
        }
      }
    }
    case FPGA:          // fall through from previous case
    {
      // Logs
      printk(KERN_ALERT "simdrv: cleanup(FPGA)\n");

      for (ph = 0; ph < NUM_DEVICES; ++ph)
      {
          // stop send T=0 NULL byte timer, in case it still sceduled/runnig
          phoneSIMData[ph].m_T0Mode = T0_WAIT_COMMAND;


          del_timer_sync(&phoneSIMData[ph].m_SendNullTimer);

          //phoneSIMData[ph].Flags.Valid = 0;
          // deallocate read buffer
          cb_DisconnectBuffer(&phoneSIMData[ph].m_RxBuffer);
          cb_DisconnectBuffer(&phoneSIMData[ph].m_IoctlBuffer);
      }
    }
    case MajorDev:          // fall through from previous case
        cdev_del(&simdrv_cdev);

    case RegisterMajor:
        unregister_chrdev_region(MKDEV(simdrv_major, SIMPHONE_MINOR_DEV_NUM),NUM_DEVICES);
        // in case a timer still working
        del_timer_sync(&timer_IrCheck);

    case IOmap:         // fall through from previous case
#ifndef SIM_DEV_BOARD
        ioport_unmap(fpga_mapped_address);
#endif
    case Mem:           // fall through from previous case
        // release_mem_region(FPGA_SSW_IRQ, FPGA_REG_SIZE - SSW_IRQ);
    case None:          // fall through from previous case
    default:
        break;
    }

    // Flush work queues
	flush_work(&simdrv_timer_wq.real_work);

    printk("simdrv: Module Unloading...\n");
    return;
}
//
//  Driver supports 2 kinds of UARTs -"old" and "new"
//  "old" UART is implemented in FPGA 6
//  "new" UART is implemented in FPGA 7 and higher
//  some functions should be implemented differently for differen typs of UARTs
//  This functions initialize "virtual UART functions
//  These is one exception from this rule - IRQ routine checks FPGA version
//  instead of calling "virtual function".
//
static inline void simdrv_InitUartVirtualFunctions(void)
{                        // new UART with Tx FIFO
        g_LsrTxError = LSR_TX_ERROR;
        simdrv_SendAtr           = simdrv_Uart2SendAtr;
        simdrv_SendResponse      = simdrv_Uart2SendResponse;
        simdrv_ClearRx           = simdrv_Uart2ClearRx;
        simdrv_CalculateDivisor  = simdrv_Uart2CalculateDivisor;
        simdrv_DivisorToBaudrate = simdrv_Uart2DivisorToBaudrate;
}

//
// simdrv_init_phone
//
static int
simdrv_init_phone(int ph)
{
  int i = 0;

  // Log
  printk(KERN_ALERT "simdrv: Initializing phonesim%d\n",ph);

  phoneSIMStruct * PD = phoneSIMData + ph;

  spin_lock_init(&PD->m_ConfigLock);

  // Set phone ID early - needed for access
  PD->m_PhoneId = ph;

  // Precompute UART registers addresses.
  dev_GetUartRegisters(PD, ph);
  // Precompute phone/SIM control FPGA registers addresses.
  dev_GetFpgaRegisters(PD, ph);

  // prepare default UART configuration for T=0 protocol
  PD->m_DefaultUartConfig.m_nBaudRate = DEFAUL_BAUD_RATE;
  PD->m_DefaultUartConfig.m_chLcr = LCR_DATA_BITS_8 | LCR_PARITY_EVEN | LCR_STOP_BITS_2;
  PD->m_DefaultUartConfig.m_chSpc = SPC_T0_ENABLE; // plus direct coding convention

  // set statistics variables
  PD->tx_err_flg = 0;             // Set when the transmitter is unable to
                                  // successfully transmit a character.
  PD->rx_cnt_loss = 0;            // Lost character count due to a full
                                  // circular buffer.
  PD->ioctl_cnt_loss = 0;

  PD->bogus_uart_irq    = 0;
  PD->spurious_uart_irq = 0;
  PD->lsr_error_cnt_irq = 0;
  PD->tx_error_cnt_irq  = 0;
  PD->rx_cnt            = 0;
  PD->tx_cnt            = 0;

  // allocate IRQ<=>processing thread circular buffer
  if ( cb_AllocBuffer(&PD->m_RxBuffer, IRQ_BUF_SIZE, GFP_KERNEL) != 0 )
  {
      printk("Error In Memory Allocation for Ioctl Buffer\n");
      return  -ENOMEM;
  }
  // allocate processing thread<=>ioctl circular buffer
  if ( cb_AllocBuffer(&PD->m_IoctlBuffer, IOCTL_BUF_SIZE, GFP_KERNEL) != 0 )
  {
      printk("Error In Memory Allocation for Ioctl Buffer\n");
      return  -ENOMEM;
  }

  PD->m_PhoneForCopyEvents = NULL;
  PD->m_EventSrc = -1;
  PD->m_RxThreadStatus = SIM_THREAD_STOP;
  //init_completion(&PD->m_RxThreadCompletion);

  tasklet_init(&PD->m_RxTasklet, simdrv_tasklet, (unsigned long)PD);

  atomic_set(&PD->m_PhoneTxTrigger, 0);
  atomic_set(&PD->m_EventCnt, 0);

  // initialize ioctl wait queue
  init_waitqueue_head(&PD->ioctl_wake_queue);

  // initilize Transmit semaphore
  sema_init(&PD->m_TxBuffer.m_TxSemaphore, 1);       // 1 - means semaphore available
  // initilize Receive semaphore
  sema_init(&PD->m_RxSemaphore, 1);       // 1 - means semaphore available

  PD->ATRptr = ATR[ph];
  ATR[ph][0] = 0x3B;
  ATR[ph][1] = 0x90;  // This byte is modified when size known
  ATR[ph][2] = 0x11;
  ATR[ph][3] = 0x40;
  ATR[ph][4] = 0xFF;
  PD->BogusInterrupts = 0;
  PD->SpuriousUartInterrupts = 0;
  PD->SpuriousPhoneInterrupts = 0;

  PD->m_SendNull  = SIMCONFIG_SEND_NULL;
  PD->m_AckMode   = SIMCONFIG_SEND_ACK;

  PD->m_ExclusiveFlag  = EXCLUSIVE_ACCESS_OFF;  // shared access

  PD->m_SendNull  = SIMCONFIG_SEND_NULL;
  // initialize timer for sending T=0 NULL byte
  init_timer(&PD->m_SendNullTimer);

  // init_timer(&timer_IrCheck);

  // ---- from tequal0 driver
  PD->m_ActualIrEvents = 0;
  PD->m_AcknowledgedIrEvents = 0;
  PD->m_SimPresent = 0xFF;

#ifdef SEND_DEBUG_MESSAGES
  do_gettimeofday (&PD->m_ResetTime);     // to assign some time
  PD->m_DebugMsg[0] = '\0';               // empty debug buffer
  PD->m_DebugOn = SIMCONFIG_DEBUG_ON;
  PD->m_TxBuffer.m_DataType = TX_COMMAND;
#endif

  PD->m_numCmdReads = 0;
  for (i = 0; i < MAX_LOGGED_CMDS; i++)
  {
      PD->m_CmdReadLog[i].m_len = 0;
  }
  g_uartDivisor[ph] = 0;

  // Success
  return(0);
}

//
// simdrv_init
//
static inline int simdrv_init(void)
{
    int ph;
    phoneSIMStruct * PD;

    // Precompute SSW register addresse.
    ssw_irq_reg = fpga_mapped_address + SSW_IRQ;

    // Original probe-only support always initialized
    // and ran all phones therefore data for each phone
    // was always initialized.
    //
    // With AMP, some instances may never initialize.
    //
    // Initializae any data that may need cleanup logic.

    // Initialize any data for all phone instances.
    for (ph = 0; ph < NUM_DEVICES; ++ph)
    {
      // Access phone
      PD = phoneSIMData + ph;

      // Initialize phone ID to max devices
      PD->m_PhoneId = NUM_DEVICES;

      // Initialize empty tasklet.
      // On initialization it will be redone.
      tasklet_init(&PD->m_RxTasklet, NULL, 0L);

      // Initialize timer
      init_timer(&PD->m_SendNullTimer);
      
      // Perform mappings
      dev_GetFpgaRegisters(PD, ph);
      dev_GetUartRegisters(PD, ph);
    }

    // Initialize all probe instances
    for (ph = 0; ph < NUM_PHONES; ++ph)
    {
      // If phone present...
      if (dev_IsPhonePresent(ph))
      {
        // Initialize it
        simdrv_init_phone(ph);
      }
    }

    return 0;
}

static int __init simdrv_init_module(void)
{
    int result = 0;

    DriverState = None;         /* start initialization */

    printk(KERN_ALERT "SIMDRV start\n");

#ifndef SIM_DEV_BOARD
    // Perform FPGA mapping
    fpga_mapped_address = ioport_map((unsigned long)FPGA_REG_BASE, (size_t)FPGA_REG_SIZE);
#endif

    // If mapped...
    if (fpga_mapped_address != NULL)
    {
      DriverState = IOmap;
    } else {
      printk("simdrv Couldn't Map Region [%x:%x]\n",FPGA_REG_BASE,FPGA_REG_BASE+FPGA_REG_SIZE-1);
      result = -EFAULT;
      goto fail;
    }

    //
    // check if we have correct FPGA version
    //

    // FPGA version is now hard-coded
    g_FpgaVersion = 8; // default to new

    simdrv_InitUartVirtualFunctions();

    result = register_chrdev_region(MKDEV(simdrv_major, SIMPHONE_MINOR_DEV_NUM),
                                          NUM_DEVICES, simdrv_dev);
    if (result < 0)
    {
        printk(KERN_ALERT "simdrv: can't get major %d;%d\n",simdrv_major, result);
        goto fail;
    }
    DriverState = RegisterMajor;

    // initialize Insert/Remove timer - each 500 msec
    simdrv_StartInsertTimer((500 * HZ)/1000);

    //simdrv_cdev = cdev_alloc();
    simdrv_cdev.ops = &simdrv_fops;
    cdev_init(&simdrv_cdev, &simdrv_fops);
    result = cdev_add(&simdrv_cdev, MKDEV(simdrv_major,SIMPHONE_MINOR_DEV_NUM), NUM_DEVICES);
    if (result < 0)
    {
        printk(KERN_ALERT "simdrv: can't allocate device structure %d\n", result);
        goto fail;
    }
    DriverState = MajorDev;

    if (simdrv_major == 0)
      simdrv_major = result; /* dynamic */
    printk(KERN_ALERT "simdrv: MAJOR %d  FPGA version %d\n", simdrv_major, g_FpgaVersion);

    /* initialize FPGA phone and UART registers */
    if ( simdrv_init() != 0 )
        goto fail;
    DriverState = FPGA;

    EXPORT_NO_SYMBOLS; /* otherwise, leave global symbols visible */

    DriverState = RxTasklet; // even if function failed - it is possible that
                             // some tasklets were started - so we need to kill them
    if ( result != 0 )
        goto fail;

    // Assume success
    result = 0;

#ifndef SIM_DEV_BOARD
    /* install SIM interrupt service routine */
    result = request_irq(fpga_irq, simdrv_isr, IRQF_SHARED, "phoneRemoteSIM", &simdrv_dev);
#endif

    if (result)
    {
      printk(KERN_ALERT "simdrv: can't get assigned irq %i\n", fpga_irq);
      goto fail;
    } else {
      DriverState = IRQ;
    }
#ifndef SIM_DEV_BOARD
    create_proc_read_entry( "sim",
                            0 /* default mode */,
                            NULL /* parent dir */,
                            simdrv_proc_page,
                            NULL /* client data */);
#endif
    DriverState = Statistic;

    // Initialize work queues
  	INIT_WORK(&simdrv_timer_wq.real_work, simdrv_TimerFunctionWorker);

    printk("simdrv: Loaded Successfully, irq = %d.\n", fpga_irq);
    return 0; /* succeed */

fail:
    simdrv_cleanup_module();
    return result;
}

module_param(read_timeout_ms, int, 0);
MODULE_PARM_DESC(read_timeout_ms, "Timeout in milliseconds for reading from phone");
module_param(fpga_irq, int, 0);
MODULE_PARM_DESC(fpga_irq, "Interrupt number used for interrupts from FPGA");
module_param(character_retransmit, int, 0);
MODULE_PARM_DESC(character_retransmit,"Set SIM error-signal character retransmit count Range [0-3]");

module_init(simdrv_init_module);
module_exit(simdrv_cleanup_module);
MODULE_LICENSE("GPL");
//MODULE_LICENSE("Proprietary. Copyright 2005, 2006 Casabyte Inc.  Send bug reports to support@casbayte.com");

