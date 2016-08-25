#ifndef _SIMDRV_H_
#define _SIMDRV_H_

/*
 * Copyright (c) 2005 Casabyte, Inc.  All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <asm/types.h>

//#define SIMDRV_DEBUG_IOCTL          // if defined debug ioctl will be active

// LCR bit masks:
#define LCR_DATA_BITS_5     0x00
#define LCR_DATA_BITS_6     0x01
#define LCR_DATA_BITS_7     0x02
#define LCR_DATA_BITS_8     0x03
#define LCR_STOP_BITS_1     0x00
#define LCR_STOP_BITS_2     0x04
#define LCR_PARITY_NONE     0x00
#define LCR_PARITY_ODD      0x08
#define LCR_PARITY_EVEN     0x18
#define LCR_PARITY_ONE      0x28
#define LCR_PARITY_ZERO     0x38

// LSR bit masks:
#define LSR_TRANSMIT_HOLDING_EMPTY  (1<<5)
#define LSR_TRANSMIT_EMPTY          (1<<6)

// SPC bit masks:
#define SPC_T0_ENABLE       0x01
#define SPC_TX_RETRY_NONE   0x00
#define SPC_TX_RETRY_ONE    0x02
#define SPC_TX_RETRY_TWO    0x04
#define SPC_TX_RETRY_THREE  0x06
#define SPC_T0_DIRECT       0x00
#define SPC_TO_INVERSE      0x08

/*
 * ioctl definitions
 */

// SIMDRV_COMMAND:  Reads commands &/or payload sent from the phone to the SIM.
//          First write 1 byte containing size to read.  But if command
//          (always 5 bytes) is SELECT, also read 2 bytes payload,
//          since this is very common, so add 2 bytes to buffer.
//          But also allow 1 byte at beginning, which is 0 for commands
//          or payload, and non-zero for an event (see defines below).
//          Also reads command &/or response trace from local SIM.
// SIMDRV_ATR:      Writes the ATR, which is Answer To RESET, to the driver, so
//                  that when the phone sends a RESET, the ATR can be sent
//                  without system call latency.  Note just the ATR history
//                  bytes are written, since the rest of the ATR is fixed.
//                  The ATR history is preceded by 1 byte containing its size.
// SIMDRV_RESPONSE: Writes response to a command from the SIM to the phone.  The
//                  response is preceded by the op code and followed by the
//                  status word.  These are preceded by the size of response
//                  that follows; this has 2 bytes (the 1st byte contains the
//          MSBs and the 2nd contains the LSBs).
/*
struct uart_config {
   int32_t  baudrate;
   int8_t   lcr;
   int8_t   spc;
} uart_config;
*/
// definitions for SIMDRV_ENABLE
#define UART_DISABLE    (0)
#define UART_ENABLE     (1)

#define SIMCONFIG_COMMAND_LIST      (1)
#define SIMCONFIG_UART              (2)
#define SIMCONFIG_WAITING_MODE      (3)
#define SIMCONFIG_ACK_MODE          (4)
#define SIMCONFIG_ATTACH_DEVICE     (5)
#define SIMCONFIG_DEBUG             (6)

// definitions for SIMCONFIG_COMMAND_LIST
#define SIMCMD_LIST_ELEMENT_CNT     (50)

#define SIMCMD_NOT_DEFINED          (0xFF)
#define SIMCMD_ONLY_HEADER          (0)
#define SIMCMD_PAYLOAD              (1)
#define SIMCMD_RESPONSE             (2)
//#define SIMCMD_PAYLOAD_RESPONSE     SIMCMD_PAYLOAD | SIMCMD_RESPONSE
//definitions for SIMCONFIG_WAITING_MODE

#define SIMCONFIG_DISMISS_NULL      (0)
#define SIMCONFIG_SEND_NULL         (1)

#define SIMCONFIG_DISMISS_ACK       (0)
#define SIMCONFIG_SEND_ACK          (1)

#define DEVTYPE_NODEVICE            (0) // detach all from UART or broke direct connection
#define DEVTYPE_PHONE               (1) // attach phone to UART
#define DEVTYPE_SIM                 (2) // attach SIM to UART
#define DEVTYPE_DIRECT              (3) // attach phone to SIM directly no UART involved
#define DEVTYPE_MONITOR             (4) // attach UART to another UART to monitor existing communication

// copied from tequal0 driver
#define TEQUAL0_STATUS_PRESENT  (1<<0)
#define TEQUAL0_STATUS_READY    (1<<1)

// Command Buffer Size:  Command (0 or 5 bytes) + Payload (0 to 256 bytes, but have never seen more than 19 bytes) + Events
#define CMD_BUFFER_SIZE (300)
// Response Buffer Size:  Payload (0 to 256 bytes) + Status Word (2 bytes)
#define RSP_BUFFER_SIZE (260)

typedef struct SimConfigHeader
{
    uint16_t    m_ConfigType;       // defines requested configuration
    uint16_t    m_Length;           // configuration structure size
                                    // ...
                                    // definitions needed to configure simdriver
} SimConfigHeader;

typedef struct SimConfigCommandList
{
    uint16_t    m_ConfigType;       // defines requested configuration
    uint16_t    m_Length;           // configuration structure size

    uint16_t    m_CommandList[SIMCMD_LIST_ELEMENT_CNT];
                                    // each element of the array is a single
                                    // command definition
                                    // MS Byte - command type
                                    //                    Payload/Response/Both
                                    // LS Byte - command code (INS in the specs)
} SimConfigCommandList;

typedef struct SimConfigUart
{
    uint16_t    m_ConfigType;       // defines requested configuration
    uint16_t    m_Length;           // configuration structure size

    int32_t     m_nBaudRate;
    int8_t      m_chLcr;
    int8_t      m_chSpc;
} SimConfigUart;


typedef struct SimConfigWaitingMode
{
    uint16_t    m_ConfigType;       // defines requested configuration
    uint16_t    m_Length;           // configuration structure size
    uint16_t    m_SendNull;         //  1 - send NULL 0 - do NOT send.
} SimConfigWaitingMode;

typedef struct SimConfigAckMode
{
    uint16_t    m_ConfigType;       // defines requested configuration
    uint16_t    m_Length;           // configuration structure size
    uint16_t    m_SendAck;          //  1 - send ACK in responses 0 - do NOT send.
} SimConfigAckMode;

typedef struct SimConfigAttachDevice
{
    uint16_t    m_ConfigType;       // defines requested configuration
    uint16_t    m_Length;           // configuration structure size
    uint16_t    m_DeviceType;       // 0 - phone (0 to 2); 1 - SIM (0 to 3)
    uint16_t    m_DeviceId;         // slot/phone number
} SimConfigAttachDevice;

typedef struct SimConfigDebug
{
    uint16_t    m_ConfigType;       // defines requested configuration == SIMCONFIG_DEBUG
    uint16_t    m_Length;           // configuration structure size
    uint16_t    m_Debug;            //  1 - send debug messages. 0 - do NOT send. - Could be extended later
} SimConfigDebug;

typedef struct SimCommand
{
    uint16_t    m_Length;       // length of the data in the m_Command
                                // or 0 - if only event is returned.
    uint8_t     m_Event;
    uint8_t     m_Command[CMD_BUFFER_SIZE];
} SimCommand;

typedef struct SimMonitor
{
    uint16_t    m_Length;       // IN - requested length
                                // OUT - read element count of read data in the m_Command
                                // or 0 - if only event is returned.
    uint16_t     m_Command[CMD_BUFFER_SIZE];
} SimMonitor;

typedef  struct SimResponse
{
    uint16_t    m_Length;           // length of the data to be sent in the m_Response
    uint8_t     m_Response[RSP_BUFFER_SIZE];
} SimResponse;

typedef  struct SimResponseEx
{
    uint16_t    m_BaudRateDivisor;  // baud rate divisor
    uint16_t    m_Length;           // length of the data to be sent in the m_Response
    uint8_t     m_Response[RSP_BUFFER_SIZE];
} SimResponseEx;

typedef  struct SimResponseBaudRate
{
    uint32_t    m_BaudRate;         // Baud Rate
    uint16_t    m_Length;           // length of the data to be sent in the m_Response
    uint8_t     m_Response[260];
} SimResponseBaudRate;

typedef struct SimCommunication
{
    uint16_t    m_Length;           // length of the data in the m_Command
    uint8_t     m_Event;
    uint8_t     m_Command[CMD_BUFFER_SIZE];
} SimCommunication;



//
// structures for IOCTLs from tequal0 driver
//
typedef struct  SimInsert               // old tequal0_insertion definition
{
    uint32_t    m_IrEventCount; // Number of Insert/Remove Events Since Open
    uint16_t    m_SimPresent;   // SIM Currently Inserted
    uint16_t    m_Flags;
    long        m_Timeout;      // Max number of HZ (jiffies) to wait before timeout
    uint8_t     m_Slot[2];      // (read only) Slot Code one Letter [A-L], one digit [1-3]
}   SimInsert;

typedef struct  SimAtr
{
    uint8_t     m_AtrSize;
    uint8_t     m_HistorySize;
    uint8_t     m_Atr[39];
    uint8_t     m_History[15];
    uint8_t     m_TckPresent;       // 0 - TCK not present in ATR; 1 - present
    uint8_t     m_TckValue;
} SimAtr;


typedef struct  SimPdu
{
    uint8_t     m_CommandBlock[5];
    uint8_t     m_StatusBlock[2];
    uint8_t     m_Payload[256];
unsigned int    m_PayloadBytes;     // Actual Transfer Size
    uint8_t     m_PayloadReceive;   // 0 - send payload to SIM
                                    // 1 - get payload from SIM
}   SimPdu;

// definitions for Flags member
#define BLOCK_INSERTED  (1 << 0)    /* Block until SIM Present is true */
#define BLOCK_IR_EVENT  (1 << 1)    /* Block until any Insert/Remove Event  */



// Typical SIM communication UART configuration.
//  uart_cfg.baudrate = 8737;
//  uart_cfg.lcr = LCR_DATA_BITS_8 | LCR_PARITY_EVEN | LCR_STOP_BITS_2;
//  uart_cfg.spc = SPC_T0_ENABLE | SPC_TX_RETRY_THREE;
//  simdrv_uart_cfg(PD, &uart_cfg);


#define SIMDRV_IOC_MAGIC    0xDE
#define SIMDRV_IOC_MIN_NR   110

enum ioc_nr
{
    command_nr = SIMDRV_IOC_MIN_NR,
    atr_nr,
    response_nr,
    config_nr,
    enable_nr,
    version_nr,
    command_full_nr,
    response_negotiation_nr,
    atr_full_nr,
    atr_sim_comm_nr,
    phone_copy_events,
    get_raw_stream,
    sim_insert,
    sim_reset,
    sim_transact,
    sim_power,
    sim_clock,
    sim_status,
    response_negotiation_baud
    };

#define SIMDRV_COMMAND      _IOWR(SIMDRV_IOC_MAGIC, command_nr, uint8_t)
#define SIMDRV_ATR          _IOW(SIMDRV_IOC_MAGIC, atr_nr, uint8_t)
#define SIMDRV_RESPONSE     _IOW(SIMDRV_IOC_MAGIC, response_nr, uint8_t)
#define SIMDRV_SET          _IOW(SIMDRV_IOC_MAGIC, set_nr, uart_config)
#define SIMDRV_ENABLE       _IO (SIMDRV_IOC_MAGIC, enable_nr)
#define SIMDRV_VERSION      _IO (SIMDRV_IOC_MAGIC, version_nr)
#define SIMDRV_CONFIG       _IOW(SIMDRV_IOC_MAGIC, config_nr, SimConfigHeader)
#define SIMDRV_COMMAND_FULL _IOWR(SIMDRV_IOC_MAGIC, command_full_nr, SimCommand)
#define SIMDRV_RESPONSE_NEGOTIATION _IOW(SIMDRV_IOC_MAGIC, response_negotiation_nr, SimResponseEx)
#define SIMDRV_RESPONSE_BAUDRATE    _IOW(SIMDRV_IOC_MAGIC, response_negotiation_baud, SimResponseBaudRate)
#define SIMDRV_ATR_FULL             _IOW(SIMDRV_IOC_MAGIC, atr_full_nr, uint8_t)
#ifdef REMOVED_CODE
#define SIMDRV_SIM_COMMUNICATION    _IOW(SIMDRV_IOC_MAGIC, atr_sim_comm_nr, SimCommunication)
#endif
// copy (steal) events from other phone. Expects signed int parameter - phone from copy event from
#define SIMDRV_COPY_EVENTS  _IO (SIMDRV_IOC_MAGIC, phone_copy_events)
#define SIMDRV_MONITOR      _IOWR(SIMDRV_IOC_MAGIC, get_raw_stream, SimMonitor)

//
// IOCTLs from tequal0 driver
//
#define TEQUAL0_INSERT      _IOWR(SIMDRV_IOC_MAGIC, sim_insert, SimInsert)  // Block until Inserted/Removed
#define TEQUAL0_RESET       _IOR (SIMDRV_IOC_MAGIC, sim_reset,  SimAtr)     // Reset SIM + ATR + PPS(if needed), return results in structure
#define TEQUAL0_TRANSACT    _IOWR(SIMDRV_IOC_MAGIC, sim_transact, SimPdu)   // Perform with SIM Complete PDU Transaction
#define TEQUAL0_POWER       _IO(SIMDRV_IOC_MAGIC,  sim_power)               // Set SIM Power to arg==0 -> off; arg!=0 -> on
#define TEQUAL0_CLOCK       _IO(SIMDRV_IOC_MAGIC,  sim_clock)               // Set SIM Clock to arg==0 -> off; arg!=0 -> on
#define TEQUAL0_STATUS      _IOR(SIMDRV_IOC_MAGIC, sim_status, uint32_t)    // Fetch Status word from driver


// CMD AND EVENT BITs of Byte 0 of array returned for SIMDRV_COMMAND
#define CMD_OR_PAYLOAD      (0)
#define LINE_STATUS_EVENT   (1)
#define SIM_RESET_EVENT     (2)
#define POWER_ON_EVENT      (3)
#define POWER_OFF_EVENT     (4)
#define POWER_FAULT_EVENT   (5)
#define TIMEOUT_EVENT       (6)
#define LOCAL_SIM_TRACE     (7)

// LINE_STATUS_EVENT bits
#define TRANSMIT_ERROR_EVENT  (1<<7)
#define FRAMING_ERROR_EVENT   (1<<6)
#define PARITY_ERROR_EVENT    (1<<5)
#define OVERRUN_ERROR_EVENT   (1<<4)

//
//  IRQ reports generic events. processing thread will convert them
//      to the EVENT codes defined above
//
#define IRQ_CMD_OR_PAYLOAD      (0)
#define IRQ_EVENT_RESET         (0x81)
#define IRQ_EVENT_VCC           (0x40)
#define IRQ_EVENT_VCC_POWER_OFF (0x40)
#define IRQ_EVENT_VCC_FAULT     (0x48)

#define CMD_AND_EVENT_BIT_MASK  (0xF)

// used in all SIM On/Off functions as a parameter

typedef enum
{
    STATE_OFF,
    STATE_ON
} StateChange;

#endif  /*_SIMDRV_H_*/
