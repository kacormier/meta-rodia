/*
 *
 *  Copyright (c) 2015 RCATSOne Incorporated.  All rights reserved.
 *
 */
// FPGA.h
//
// #######################
// ## Before XScale RTP ##
// #######################
//
//------------------------------  ADDRESS MAP  ---------------------------------
// 0xC00x - FPGA registers:
// 0xC000 -  Version register (RO).
// 0xC002 -  SSW board HW revision register (RO).
//
// 0xC02x - Local phone 0 interface.  Also decoded at 0xC10x for backward
//          compatibility.
//      0 -  Phone 0 SIM selection register (RW).
//      1 -  Phone 0 status/control register (RW).
// 0xC03x - Local phone 1 interface.  Also decoded at 0xC11x for backward
//          compatibility.
//      0 -  Phone 1 SIM selection register (RW).
//      1 -  Phone 1 status/control register (RW).
// 0xC04x - Local phone 2 interface.
//      0 -  Phone 2 SIM selection register (RW).
//      1 -  Phone 2 status/control register (RW).
//
// 0xC10x - Local phone 0 interface:
// 0xC100 -  Phone 0 SIM selection register (RW).
// 0xC101 -  Phone 0 status/control register (RW).
// 0xC11x - Local phone 1 interface:
// 0xC110 -  Phone 1 SIM selection register (RW).
// 0xC111 -  Phone 1 status/control register (RW).
//
// 0xC12x - Remote phone 0 interface
// 0xC120 -  Remote phone 0 SIM selection register (RW).
// 0xC121 -  Remote phone 0 SIM status/control register (RW).
// 0xC13x - Remote phone 1 interface
// 0xC130 -  Remote phone 1 SIM selection register (RW).
// 0xC131 -  Remote phone 1 SIM status/control register (RW).
// 0xC14x - Remote phone 2 interface
// 0xC140 -  Remote phone 2 SIM selection register (RW).
// 0xC141 -  Remote phone 2 SIM status/control register (RW).
//
// 0xC20x - SIM card interface:
// 0xC200 -  SIM 0 status control register (RW).
// 0xC201 -  SIM 1 status control register (RW).
// 0xC202 -  SIM 2 status control register (RW).
// 0xC203 -  SIM 3 status control register (RW).
//
// 0xD00x - SIM UART 1.
// 0xD000 - RBR: Receive buffer register (RO).
//          THR: Transmit holding register (WO).
// 0xD001 - IER: Interrupt enable register (RW).
// 0xD002 - IIR: Interrupt identification register (RO).
// 0xD003 - LCR: Line control register (RW).
// 0xD004 - MCR:
// 0xD005 - LSR: Line status register (RO).
// 0xD006 - MSR:
// 0xD007 - SCR:
// 0xD008 - DLL: Divisor latch LSB (RW).
// 0xD009 - DLM: Divisor latch MSB (RW).
// 0xD00A - SPC: SIM/T0 protocol control register (RW).
//
// 0xD01x - SIM UART 2.
// 0xD02x - SIM UART 3.
//
// 0xE00x - Debugging UART.
// 0xE000 - RBR: Receive buffer register (RO).
//          THR: Transmit holding register (WO).
// 0xE001 - IER: Interrupt enable register (RW).
// 0xE002 - IIR: Interrupt identification register (RO).
// 0xE003 - LCR: Line control register (RW).
// 0xE004 - MCR:
// 0xE005 - LSR: Line status register (RO).
// 0xE006 - MSR:
// 0xE007 - SCR:
// 0xE008 - DLL: Divisor latch LSB (RW).
// 0xE009 - DLM: Divisor latch MSB (RW).

//-------------------------  LOCAL PHONE INTERFACE  ----------------------------
// Phone's SIM selection register:
//  MSB   7: Enable/Disable (1/0) SIM selection.
//      6-2: Reserved (set to 0).
//      1-0: SIM address.
//
// Local phone's status/control register:
//  MSB   7: RST signal change interrupt enable.
//        6: VCC signal change interrupt enable.
//        5: Delta phone's RST SIM signal.
//        4: Delta phone's VCC SIM signal.
//        3: Status of the phone's general purpose output.
//        2: Phone's RST SIM signal status.
//        1: Phone's VCC SIM signal status.
//  LSB   0: SIM present control/status bit. When a SIM address has been
//           enabled, this bit is read only.  When SIM address is disabled
//           this is a control bit, even if UART is disabled as well.

//------------------------  REMOTE PHONE INTERFACE  ----------------------------
// Remote phone's SIM selection register:
//  MSB   7: Enable/Disable (1/0) UART selection.  Connects/disconnects the
//           UARTS from a phone or SIM.
//      6-2: Reserved (set to 0).
//      1-0: Phone/SIM address.  Whether it is a phone or SIM address depends
//           upon bit 7 in SIM status/control register.
//
// Remote phone's SIM status/control register:
//  MSB   7: Phone/SIM (0/1) connection control.  Selects the phone or SIM;
//           see bits 0-1 in the SIM selection register above.
//      6-4: Reserved (set to 0).
//        3: SIM VCC signal control.
//        2: SIM RST signal control.
//        1: SIM CLK enable/disable.
//        0: SIM CLK stop level.

//--------------------------  SIM CARD INTERFACE  ------------------------------
// SIM status/control register:
//  MSB 7-6: SIM LED control (11 = off; 10 = green; 01 = red; 00 = red/green).
//        5: LED control source: 0/1 = SIM signals/reg.
//        4: SIM VCC voltage select (0/1 = 3V/5V).
//        3: SIM VCC signal status.
//        2: SIM RST signal status.
//        1: SIM ready signal generated by the SIM interface chip.
//        0: SIM present

//-----------------------------  UART SUPPORT  ---------------------------------
// The FPGA contains UARTs that have a 450 compatible interface with with extensions
// to support the SIM T0 protocol.  The following registers and bits are supported:
//
//  0 - RBR(7-0): Receive buffer register (RO).
//      THR(7-0): Transmit holding register (WO).
//  1 - IER(3-0): Interrupt enable register (RW).
//  2 - IIR(3-0): Interrupt identification register (RO).
//  3 - LCR(7,5-0): Line control register (RW).
//      - The transmission of 1.5 stop bits is not supported.
//  4 - MCR(4,0): Modem control register (RW).
//      - The DTR signal is only available to support loop-back to the DSR input.
//  5 - LSR(6-5,3-0): Line status register (RO).
//      - The break interrupt flag (bit 4) is not supported.  When the SIM/T0
//        protocol is active, this bit indicates an unrecoverable transmit
//        error.  It is set when a character has been re-transmitted, according
//        to the SPC register, and an error signal was returned each time.  It
//        is cleared when the LSR is read.
//  6 - MSR(5,1): Modem status register (RO).
//  7 - SCR: Not supported.
//  8 - DLL(7-0): Divisor latch LSB (RW).
//      - This register is also accessible at offset 0 when LCR(7) is set.
//  9 - DLM(1-0): Divisor latch MSB (RW).
//      - This register is also accessible at offset 1 when LCR(7) is set.
//  A - SPC(3-0): SIM/T0 protocol control register (RW).
//         0: T0 error signal generation/detection: 0/1 = disabled/enabled.
//            When T0 error signal generation/detection is enabled, then the
//            number of stop bits is 2 regardless of LCR[2].
//       2-1: Error signal re-transmit count (0 - 3).
//         3: T0 coding convention: 0 = direct  (Tx LSB first. Z = 1).
//                                  1 = inverse (Tx MSB first. Z = 0).
//       7-4: Reserved (set to 0).
//
//
// Besides not supporting all the UART registers and bits, the UART varies from
// the 450 specification in the following:
// - The addressable memory space is 16 bytes, not 8.  Extra registers and
//   functionality are added through the upper 8 addressable memory locations.
// - When an over-run is detected, the RBR, LSR[PE], and LSR[FE] are NOT over
//   written.
//
// For more information on the UART registers, refer to the 16450 data sheet.

//**************************  FUNCTION PROTOTYPES  *****************************
#ifndef __KERNEL__
// Public:
char fpga_init(void);
void fpga_reset(void);
int  fpga_ram_test(void);

   #ifdef __cplusplus
   extern "C" {
   #endif
      int  fpga_bld_num(long *bldnum);
   #ifdef __cplusplus
   }        // end extern "C" {
   #endif

#endif

//********************************  MACROS  ************************************

#define NUM_T0_PORTS                3

#define FPGA_PH_SIM_ENABLE          0x80

//              UART BITS
// IER (offset 1) bit masks:
#define IER_INT_DISABLE             0x00
#define IER_RX_INT_ENABLE           0x01
#define IER_TX_INT_ENABLE           0x02
#define IER_ERR_INT_ENABLE          0x04
#define IER_MSR_INT_ENABLE          0x08

// LCR (offset 3) bit masks:
#define LCR_DATA_BITS_5             0x00
#define LCR_DATA_BITS_6             0x01
#define LCR_DATA_BITS_7             0x02
#define LCR_DATA_BITS_8             0x03
#define LCR_STOP_BITS_1             0x00
#define LCR_STOP_BITS_2             0x04
#define LCR_PARITY_NONE             0x00
#define LCR_PARITY_ODD              0x08
#define LCR_PARITY_EVEN             0x18
#define LCR_PARITY_ONE              0x28
#define LCR_PARITY_ZERO             0x38
#define LCR_DLAB                    0x80

// LSR (offset 5) bits masks:
#define LSR_FIFO_HAS_CHAR           0x01
// NOTE: bit 0 must be set for any of these errors to be valid.
#define LSR_OVERRUN_ERROR           0x03
#define LSR_PARITY_ERROR            0x05
#define LSR_FRAMING_ERROR           0x09
#define LSR_THR_EMPTY               0x20
#define LSR_TSR_EMPTY               0x40

#define SPC_T0_ENABLE               0x01
#define SPC_T0_0_RETRANSMITS        0x00
#define SPC_T0_1_RETRANSMITS        0x02
#define SPC_T0_2_RETRANSMITS        0x04
#define SPC_T0_3_RETRANSMITS        0x06
#define SPC_T0_DIRECT_CODING        0x00
#define SPC_T0_INDIRECT_CODING      0x08

//
// ################
// ## XScale RTP ##
// ################
//
//                         X-SCALE RTP FPGA REGISTERS
//                             14 September 2005

//***********************  FPGA IMAGE FILE FORMAT  *****************************
//Header file format:
//Offset Length  Description
//  0      2     FPGA image file ID.                            3A 29        :)
//  2      2     Image file header version.                     00 00
//  4      4     FPGA data offset from start of header (bytes). __ __ __ __
//  8      4     FPGA image length (bytes).                     __ __ __ __
// 12      4     FPGA build number.                             __ __ __ __

//***************************   REGISTER  MAP  *********************************
// -- The XS's expansion bus occupies 256 MB starting a base address of 0x0000_
// -- 0000 or 0x5000_0000 (configurable).  Each of the eight expansion bus chip
// -- selects occupies up to 16 MB for a total of 128 MB, which is mirrored
// -- once within the 256 MB expansion space.
// -- The FPGA uses EX_CS7n which is mapped to address space 0x0700_0000
// -- (base 0 + 16 MB * 7) at reset and then re-configured to 0x5700_0000
// -- after reset.
// -- A1 and A0 are not decoded.  The lower bits will determine the byte lane.
// -- Therefore each 8 bit register occupies 32 bits in the memory map.
// --
// -- CS7n A11 A10  16-bit addr   SIZE/REGS   ADDR SPACE CONTENTS
// --   0   0   0  0x0000-0x03FF  1024/256 B  General purpose regs.
// --   0   0   1  0x0400-0x07FF  1024/256 B  UARTs.
// --   0   1   0  0x0800-0x0BFF  1024/256 B  SSW interface.
// --   0   1   1  0x0C00-0x0FFF  1024/256 B  Reserved.
// --
// -- 0x0000 - 0x0007: Reserved: FPGA version/test registers.
// -- 0x0008 - 0x000B: Build number (MSB).
// -- 0x000C - 0x000F: Build number (LSB).
// -- 0x0010 - 0x0013: FPGA IRQ 0 status register (RO).
// -- 0x0014 - 0x0017: Reserved: Mobile phone UART IRQ status register (RO).
// -- 0x0018 - 0x001F: Reserved.
// -- 0x0020 - 0x003F: Power board interface registers.
// --          0x0020:  Power board I/O register (RW).
// --          0x0024:  LCD bus control/status register (RW).
// --          0x0028:  LED bus control/status register (RW).
// -- 0x0040 - 0x005F: LCD interface registers.
// --          0x0040:  LCD control register (RW).
// --          0x0044:  LCD FIFO space register (RO).
// --          0x0048:  LCD instruction register (WO).
// --          0x004C:  LCD data register (WO).
// -- 0x0060 - 0x007F: LED (I2C) interface registers.
// --          0x0060:  I2C control/status register (RW).
// --          0x0064:  I2C error code register (RO).
// --          0x0068:  I2C transmit/receive data register (RW).
// --          0x006C:  Reserved.
// --          0x0070:  LED line control register (RW).
// -- 0x0080 - 0x008F: LCD push button interface registers.
// --          0x0080:  LCD button control/status register (RW).
// --          0x0084:  LCD button event register (RO):
// -- 0x0090 - 0x009F: Telco interface registers.
// --          0x0090:  Telco DSP control/status register (RW).
// --          0x0094:  Telco CPLD JTAG register (RW).
// -- 0x00A0 - 0x00BF: Reserved - Ethernet EEPROM (I2C) interface registers.
// -- 0x00C0 - 0x00FF: Reserved - PCM link registers.
// -- 0x0100 - 0x01FF: Reserved.
// -- 0x0200 - 0x021F: Phone 1 I/O registers.
// --          0x0200:  Phone 1 I/O register 0.
// --          0x0204:  Phone 1 I/O direction register 0.
// --          0x0208:  Phone 1 I/O register 1.
// --          0x020C:  Phone 1 I/O direction register 1.
// --          0x0210:  Phone 1 I/O function register 1.
// --          0x0214:  Phone 1 I/O register 2.
// --          0x0218:  Phone 1 I/O direction register 2.
// --          0x021C:  Phone 1 I/O function register 2.
// -- 0x0220 - 0x023F: Phone 2 I/O registers.
// -- 0x0240 - 0x025F: Phone 3 I/O registers.
// -- 0x0260 - 0x02FF: Reserved - Phone 4-8 I/O registers.
// -- 0x0300 - 0x03FF: Reserved.

// -- 0x0400 - 0x043F: Reserved - Mobile phone 1 - UART 1.
// -- 0x0440 - 0x047F: Reserved - Mobile phone 1 - UART 2.
// -- 0x0480 - 0x04BF: Reserved - Mobile phone 2 - UART 1.
// -- 0x04C0 - 0x04FF: Reserved - Mobile phone 2 - UART 2.
// -- 0x0500 - 0x053F: Reserved - Mobile phone 3 - UART 1.
// -- 0x0540 - 0x057F: Reserved - Mobile phone 3 - UART 2.
// -- 0x0580 - 0x07BF: Reserved.
// -- 0x07C0 - 0x07FF: Reserved - Power board UART.

// -- 0x0800 - 0x083F: SSW general registers
// --          0x0800: SSW version register (RO).
// --          0x0804: Reserved.
// --          0x0808: SSW IRQ register (RO).
// --          0x080C: Reserved: SSW IRQ register 2 (RO).
// -- 0x0840 - 0x08BF: SSW local phone interface registers.
// --          0x0840: SSW phone 0 interface.
// --          0x0850: SSW phone 1 interface.
// --          0x0860: SSW phone 2 interface.
// -- 0x08C0 - 0x093F: SSW remote connection interface registers.
// --          0x08C0: SSW remote connection 0 interface.
// --          0x08D0: SSW remote connection 1 interface.
// --          0x08E0: SSW remote connection 2 interface.
// -- 0x0940 - 0x09BF: SSW SIM card interface registers.
// --          0x0940: SIM 1 interface.
// --          0x0950: SIM 2 interface.
// --          0x0960: SIM 3 interface.
// --          0x0970: SIM 4 interface.
// -- 0x09C0 - 0x0BFF: SSW T=0 UARTs.
// --          0x09C0: T=0 UART 1.
// --          0x0A00: T=0 UART 2.
// --          0x0A40: T=0 UART 3.

// -- 0x0C00 - 0x0FFF: Reserved.

// ----------------------  GENERAL PURPOSE REGISTERS  --------------------------
// ------------------------  FPGA IRQ 0 STATUS REG  ----------------------------
// -- The FPGA IRQ from the XSAB is connected to the X-Scale on GPIO-2.  GPIO-2
// -- should be configured as an active low interrupt.  When any bit within the
// -- FPGA IRQ 0 status register is set then an interrupt is pending.
// --
// -- FPGA IRQ 0 status register (R0):
// -- MSB 7: Power board LCD/LED bus IRQ.
// --     6: LCD push button IRQ.
// --     5: LED (I2C) IRQ.
// --     4: DSP IRQ (HINTn or XF).
// --   3-2: Reserved (set to 0).
// --     1: Power board UART IRQ.
// -- LSB 0: Reserved: Mobile phone UART IRQ.
// --

// ---------------------------  BUILD NUM REGISTER  ----------------------------
// -- A 16 bit build number is contained within 2 read-only registers.  The
// -- register at the lower address contains the MSB and the register at the
// -- higher address contains the LSB.
// --

//***********************  POWER BOARD INTERFACE  ******************************
//Driver level notes:
//- The power board may take control of the LCD and LED buses at any time.  The
//  power board interface registers provide a mechanism for the LCD and LED
//  drivers to recover after the LCD and LED buses become available again.
//- At this time, the LCD is a write only device.  The LCD driver may continue
//  to dump messages to the LCD interface without a problem even when the LCD
//  is not present.
//- When the LCD becomes present, the bus remains disabled until the bus is
//  enabled through a control register.  An interrupt may be generated
//  indicating that the LCD needs to be re-initialized.  At which point the
//  driver can reset the LCD interface, enable the LCD bus, initialize the
//  LCD, and indicate to the application layer to refresh the LCD.
//- The LED interface will return a "bus unavailable" error when the I2C bus
//  is not present for use.  The LED driver should continue to keep state
//  information even when the bus is unavailable.  Then when the bus becomes
//  available, the current LED state may be re-established.
//- When the LED bus becomes present, the bus remains disabled until the bus
//  is enabled through a control register.  An interrupt may be generated
//  indicating that the LED interface needs to be re-initialized.  At which
//  point, the driver can reset the I2C interface, enable the I2C bus,
//  and re-establish the LED state.
//- The loss and gain of the LED bus should be completely transparent to the
//  application layer.

//FPGA interface:
// -- Power board I/O register (RW).
// -- MSB 7-4: Reserved (set to 0).
// --       3: Power board GPIO status.
// --       2: Power board GPIO status.
// --       1: Power board bus request ctrl (1 = request bus).
// -- LSB   0: Power board bus grant status (1 = bus available).
// --
// -- LCD bus control/status register (RW).
// -- MSB 7-4: Reserved (set to 0).
// --       3: LCD init flag IRQ enable.   When set, an interrupt is generated
// --          when the LCD init flag is set.  The interrupt may be cleared by
// --          writing a '1' to the LCD init flag.
// --       2: LCD bus enable ctrl/status (1 = Enable bus).  This bit should
// --          be set by the driver when it is ready to initialize the bus
// --          device.  It may be cleared by the driver to disconnect the bus
// --          device.  This bit will also clear automatically when the power
// --          board takes control of the bus.
// --       1: LCD init flag (1 = LCD initialization required).  Set by HW when
// --          the bus goes from an unavailable state to an available state.
// --          The init flag is cleared by writing a '1' to this bit.
// -- LSB   0: LCD bus available status (1 = bus available).  This status flag
// --          indicates that the bus is available for use, but it still may
// --          not be initialized for use.

// -- LED bus control/status register (RW).
// -- MSB 7-4: Reserved (set to 0).
// --       3: LED init flag IRQ enable.   When set, an interrupt is generated
// --          when the LED init flag is set.  The interrupt may be cleared by
// --          writing a '1' to the LED init flag.
// --       2: LED bus enable ctrl/status (1 = Enable bus).  This bit should
// --          be set by the driver when it is ready to initialize the bus
// --          device.  It may be cleared by the driver to disconnect the bus
// --          device.  This bit will also clear automatically when the power
// --          board takes control of the bus.
// --       1: LED init flag (1 = LED initialization required).  Set by HW when
// --          the bus goes from an unavailable state to an available state.
// --          The init flag is cleared by writing a '1' to this bit.
// -- LSB   0: LED bus available status (1 = bus available).  This status flag
// --          indicates that the bus is available for use, but it still may
// --          not be initialized for use.


//*****************************  LCD INTERFACE  ********************************
//Supports writing to an Optrex LCD module in 4 bit mode.
//
//Driver level notes:
//- Possible driver level interface:
//  The power-board uses fprintf() to write access the LCD.  The string may
//  contain escape sequences with pre-defined meanings for the LCD.  The
//  following have been defined for the power board:
//    \f = Form feed => Clear display and home cursor.
//    \n = New line => Advance 1 line feed (modulo 1) and set "x" position at 0.
//    \r = Carriage return => Set the "x" position at 0.
//  Other escape sequences can be defined to support the LCD capabilities.

//FPGA interface:
// ---------------------------  REGISTER DECODE  -------------------------------
// -- Offset | Register
// --   0    | LCD control register (RW).
// --   1    | LCD FIFO space register (RO).
// --   2    | LCD instruction register (WO).
// --   3    | LCD data register (WO).

// ---------------------------  LCD CONTROL REG  -------------------------------
// -- LCD control/status register (R/W):
// -- MSB   7: LCD FIFO full flag (1 = full).
// --       6: LCD FIFO empty flag (1 = empty).
// --       5: LCD busy flag (1 = busy).  For diagnostic information.  The LCD
// --          interface is still operational when this bit is set.
// --     4-2: Reserved (set to 0).
// --       1: LCD clear FIFO control.  Writing a '1' to this bit clears the
// --          LCD FIFO.  A read of this bit will return 0.
// -- LSB   0: LCD initialization mode control.
// --          0 = Write only the most significant nibble from the FIFO.  The
// --              user interface enforces the inter-nibble delay.  The LCD
// --              busy signal is not monitored.  This mode is meant for
// --              initializing the LCD.
// --          1 = Write the most significant nibble and then the least
// --              signifcant nibble.  The LCD busy signal is monitored to
// --              enforce an inter-character delay.
// --          The initialization mode bit should not be changed back to 0
// --          until at least 15.2 mS have passed since the FIFO emptied.

// -------------------------  LCD FIFO SPACE REG  ------------------------------
// -- LCD FIFO space register (RO):
// -- MSB   7: Reserved (set to 0).
// -- LSB 6-0: FIFO available space.
// --

// -------------------------  LCD INSTRUCTION REG  -----------------------------
// ----------------------------  LCD DATA REG  ---------------------------------
// -- The LCD instruction and data registers are write only registers that fill
// -- the same FIFO.  The FIFO is 9 bits wide, with the 9th bit being the LCD's
// -- RS signal.  A write to the instruction register clears the LCD's RS bit,
// -- while a write to the data register sets the LCD's RS bit within the FIFO.
// --


//*************************  LCD BUTTON INTERFACE  *****************************
//Driver level notes:
//- For interrupt operations, the ISR can clear the push button interrupt by
//  writing a 1 to the push button IRQ flag and then setting a semaphore for the
//  push button task.  Since the push button interrupt is shared, this allows
//  other interrupts to be serviced more quickly.
//- Possible driver level interface:
//  fgetc() could be used as an appropriate interface for the buttons.  It would
//  return the button code or EOF if no buttons codes are available.
//
//FPGA interface:
// -------------------  LCD BUTTON CONTROL/STATUS REG  -------------------------
// -- LCD button control/status register (RW):
// -- MSB   7: Push button IRQ enable (1 = enabled push button events will
// --          cause an interrupt).  The interrupt may be cleared by reading
// --          all the pending button events or by writing a 1 to the push
// --          button IRQ flag bit.  When a 1 is written to the IRQ flag to
// --          clear the interrupt, then another interrupt will not occur
// --          until all the pending events have been read.
// --       6: Push button IRQ flag.  Set when an enabled push button event
// --          has occured.  This bit may be cleared by writing '1' to this bit
// --          or by reading all of the pending push button events.
// --       5: Push button 2 release event enable.
// --       4: Push button 2 down event enable.
// --       3: Push button 2 status.
// --       2: Push button 1 release event enable.
// --       1: Push button 1 down event enable.
// -- LSB   0: Push button 1 status.
// --

// ------------------------  LCD BUTTON EVENT REG  -----------------------------
// -- LCD button event register (RO):
// -- MSB   7: Valid event (1 = the event code is valid).  This bit means that
// --          the button event FIFO was not empty when this register was read.
// --     6-2: Reserved (set to 0).
// -- LSB 1-0: Push button event code.  The code consists of an event bit (0/1
// --          = down/up) and the button number (zero based).
// --          11 = Button 2 release event.
// --          10 = Button 2 push event.
// --          01 = Button 1 release event.
// --          00 = Button 1 push event.


//*****************************  LED INTERFACE  ********************************
//Driver level notes:
//- The FPGA presents a high level I2C interface to the driver for manipulating
//  the LEDs.
//- For interrupt operations, the ISR can clear the I2C interrupt by writing a 1
//  to the I2C IRQ flag and then setting a semaphore for the I2C task.  Since
//  the interrupt is shared, this allows other interrupts to be serviced more
//  quickly.
//- It may be advisable to keep a set of shadow registers for the LED chips.
//  1) It will save significant time and interrupts while manipulating the LEDs.
//  2) In case the power board takes control and then releases the I2C bus, then
//     the LED state may be returned to what the RTP application expects without
//     any RTP application intervention.
//  It may also be advisable to keep the higher level state of each LED (on/off/
//  color etc) for the same reasons.
//- Possible driver level interface:
//
//  int led(int mask, int ctrl);
//  mask = A mask indicating the LEDs that should be affected by the passed
//         control word (eg. 0x05 => LED 1 and 3 should be modified).
//  ctrl = A word that indicates the operation to be performed (set/clear/
//         toggle/blink) along with a color code (if necessary) on the LEDs
//         indicated by the passed mask word.
//  return value - Driver error conditions.   But it is expected, that most
//         applications will not error check LED operations because LED
//         operations have been typically begnign with no status feedback
//         available.
//
//  The actual PWM value for the color should be determined by the driver so
//  that, for example, the color red is the same from application task to
//  application task.  An alternate entry point could also be created to allow
//  an application to set the PWM code for an LED.
//
//
//FPGA interface:
//  The FPGA is connected to 3 I2C LED chips (MAX6965).  Within the FPGA, the
//  LED interface is composed of an I2C interface and an LED line control
//  register.  The LED line control register handles the non-I2C signals to
//  the LED chips.

// -----------------------  I2C CONTROL/STATUS REG  ----------------------------
// -- I2C control/status register (RW):
// -- MSB   7: I2C IRQ enable (1 = Generate an interrupt when the I2C busy
// --          status goes inactive.  The interrupt may be cleared by writing
// --          a 1 to the I2C interrupt flag.
// --       6: I2C IRQ flag.  Set when the I2C busy status goes inactive.  It
// --          is cleared by writing a 1 to this bit or by writing a 1 to the
// --          start control bit.
// --       5: I2C error code flag.  This flag is set when an error occurs and
// --          is reset when the I2C start control is set.  When set, an error
// --          code is available within the I2C error code register.
// --       4: Transmit/Receive FIFO clear control.  Writing a '1' to this bit
// --          clears the transmit and receive FIFOs.  This bit does not clear
// --          the internal transmit buffer.  A read will always return '0'.
// --       3: Receive data ready bit.  Received data is available within the
// --          I2C receive FIFO when this bit is set.  This bit is cleared by
// --          reading the receive FIFO contents or by a reset.
// --       2: Transmit FIFO full bit.  The contents of the transmit FIFO are
// --          transferred to the 63 byte transmit command buffer.  The FIFO
// --          may fill temporarily during the transfer.
// --       1: Reset control and status bit.  Writing a 1 to this bit resets
// --          the I2C interface.  This bit is cleared by the I2C interface
// --          when the reset process has completed.  Reset affects the
// --          following:
// --          - Pending IRQs are cleared.
// --          - The transmit and receive FIFOs are cleared.
// --          - The I2C error code register is cleared.
// --          - I2C processing is halted and the I2C bus is returned to the
// --            idle state.
// -- LSB   0: Start control and busy status.  Writing a 1 to this bit starts
// --          the processing of the I2C command buffer.  When read, this bit
// --          returns a 1 if the I2C interface is busy.
// --

// -------------------------  I2C ERROR CODE REG  ------------------------------
// -- I2C error code register (RO):  The error code register is non-zero when
// --  an I2C processing error occurs.  The error code is reset when the I2C
// --  start control is set.  When an error occurs, the processing of the
// --  command buffer is halted and the command buffer is flushed.
// --
// -- MSB 7-4: Reserved (set to 0).
// -- LSB 3-0: I2C error code.
// --
// -- The following is a list of I2C error codes:
// --   0 - No command buffer processing errors were detected.
// --   1 - The internal command buffer overflowed.  None of the commands
// --       within the buffer were executed.
// --   2 - Invalid command buffer.  None of the commands within the buffer
// --       were executed.
// --   3 - The I2C command address was not acknowledged by any slave on the
// --       bus.
// --   4 - A transmitted data byte was not acknowledged by the slave.
// --   5 - The receive FIFO over-flowed.
// --

// -------------------------  I2C TX/RX DATA REG  ------------------------------
// -- I2C transmit/receive data register (R/W):
// --
// -- A write to this register transfers the data to the I2C transmit FIFO.
// -- The 15 byte transmit FIFO may fill temporarily as the FPGA interface
// -- transfers the FIFO contents to an internal transmit buffer.  The
// -- internal transmit buffer may contain command strings that are at most
// -- 63 bytes long.
// --
// -- A read returns a byte from the I2C receive FIFO, which is 15 bytes deep.
// -- Command processing will abort with an error if the receive FIFO over-
// -- flows.
// --
// -- The transmit commands have the following syntax:
// --  Cmd Addr {data} {Cmd Addr {Data}} ... {Cmd Addr {Data}}
// -- where
// --  Cmd = The msb indicates a write or read command and the remainder of
// --        the byte indicates the command byte length.
// --        Write: msb = 0.  The length includes the address byte and all the
// --               data bytes within the write command.
// --        Read:  msb = 1.  The length bits indicate how many bytes to read
// --               from the I2C device plus 1.  The length does not include
// --               the address byte.
// --  Addr = The slave device address byte.
// --  Data = Write: The data to be written to the I2C device.
// --         Read: A read command does not contain data.
// --
// -- Example: Write and then read 0xA5 to the scratch pad register (offset 3)
// -- within the LED chip at address 0x40.  The LED chip's internal address
// -- pointer does not change after the write operation.
// -- The transmit buffer should be loaded with the following sequence:
// --    0x03 0x40 0x03 0xA5 0x80 0x40
// --

// --------------------  LED INTERFACE LINE CONTROL REG  -----------------------
// -- LED line control register (RW):
// -- MSB 7-2: Reserved (set to 0).
// --       1: LED blink control.  The blink control is connected to all the
// --          LED chips.
// -- LSB   0: LED I2C interface chip reset control (1 = reset).  Reset must be
// --          asserted for at least 1 uS.  The reset control is connected to
// --          all the LED chips.
// --

//****************************  TELCO INTERFACE  *******************************
// ------------------  TELCO DSP BOARD STATUS/CONTROL REGISTER  ----------------
// -- MSB   7: DSP reset control (1 = DSP in reset).
// --       6: DSP MP/MCn control (1/0 = DSP in uP/uC mode).
// --       5: DSP INT2 control (1 = DSP interrupt active).
// --       4: DSP BIO control.
// --       3: DSP XF status signal.
// --       2: DSP XF IRQ mask (1 = XF IRQ is not masked).
// --       1: DSP HINT status (1 = HPI IRQ active).
// -- LSB   0: DSP HINT mask (1 = HPI IRQ is not masked).


// -------------------------  TELCO CPLD JTAG REGISTER  ------------------------
// -- MSB 7: Telco CPLD JTAG enable (0/1 = tri-state / enabled).
// --   6-4: Reserved (set to 0).
// --     3: TCK - CPLD test clock input.
// --     2: TMS - CPLD test master strobe input.
// --     1: TDI - CPLD test data input.
// -- LSB 0: TDO - CPLD test data output.

//************************  MOBILE PHONE INTERFACE  ****************************
// -- 0x0200 - 0x021F: Phone 1 I/O registers.
// --          0x0200:  Phone 1 I/O register 0.
// --          0x0204:  Phone 1 I/O direction register 0.
// --          0x0208:  Phone 1 I/O register 1.
// --          0x020C:  Phone 1 I/O direction register 1.
// --          0x0210:  Phone 1 I/O function register 1.
// --          0x0214:  Phone 1 I/O register 2.
// --          0x0218:  Phone 1 I/O direction register 2.
// --          0x021C:  Phone 1 I/O function register 2.
// -- 0x0220 - 0x023F: Phone 2 I/O registers.
// -- 0x0240 - 0x025F: Phone 3 I/O registers.

// ---------------------------  PHONE I/O REG 0  -------------------------------
// -- Phone I/O register 0 (RW):
// -- Reading the phone I/O register always returns the status at the GPIO pin.
// -- Writing the contents of the I/O register only affects the GPIO pin if
// -- the GPIO is configured as an output within I/O direction register 0.
// --
// -- MSB   7: Phone power control (0/1 = off/on).
// --     6-1: Reserved (set to 0).
// -- LSB   0: Phone GPIO 0.

// -------------------------  PHONE I/O DIR REG 0  -----------------------------
// -- Phone I/O direction register 0 (RW):
// -- Specifies whether a GPIO pin is an output or an input pin.
// --
// -- MSB 7-4: Reserved (set to 0).
// -- LSB 4-0: Phone GPIO 0 directional control (0 = input, 1 = output).

// ---------------------------  PHONE I/O REG 1  -------------------------------
// -- Phone I/O register 1 (RW):
// -- Reading the phone I/O register always returns the status at the DPIO pin.
// -- Writing the contents of the I/O register only affects the DPIO pin if
// -- the corresponding bit within the function I/O register is 0, and the
// -- bit is configured as an output within the I/O direction register.
// --
// --                                      I/O Reg 1 Pin Name
// -- MSB 7: DPIO 7 / UART 1 TXD status.   ph_io_txd(PH)(0)
// --     6: DPIO 6 / UART 1 RXD status.   ph_io_rxd(PH)(0)
// --     5: DPIO 5 / UART 1 RTSn status.  ph_io_rtsn(PH)(0)
// --     4: DPIO 4 / UART 1 CTSn status.  ph_io_ctsn(PH)(0)
// --     3: DPIO 3 / UART 1 DTRn status.  ph_io_dtrn(PH)(0)
// --     2: DPIO 2 / UART 1 DSRn status.  ph_io_dsrn(PH)(0)
// --     1: DPIO 1 / UART 1 DCDn status.  ph_io_dcdn(PH)(0)
// -- LSB 0: DPIO 0 / UART 1 RIn status.   ph_io_rin(PH)(0)

// -------------------------  PHONE I/O DIR REG 1  -----------------------------
// -- Phone I/O direction register 1 (RW):
// -- Specifies whether a DPIO pin is an output or an input pin.  If a DPIO's
// -- pin is set for an alternate function, then the corresponding directional
// -- control bit is ignored.  Reading the direction register always returns the
// -- contents of the direction register irrespective of the DPIO pin function.
// --
// -- MSB 7-4: Phone DPIO 7-4 directional control (0 = input, 1 = output).
// -- LSB 3-0: Phone DPIO 3-0 directional control (0 = input, 1 = output).
// --

// ----------------------  PHONE I/O FUNCTION REG 1 ----------------------------
// -- Phone I/O function register 1 (RW):
// -- Specifies whether a phone's dual purpose I/O (DPIO) pin functions as a
// -- general purpose I/O pin or as a phone UART I/O signal.  Setting a bit
// -- within this register connects the corresponding UART signal to the
// -- phone's UART.  In this case, the I/O direction register is ignored.
// -- Reading the I/O register always returns the I/O pin status.
// --
// --  Function  Direction    I/O    | I/O
// --  register  register   register | Pin
// --     0         0          x     | GP input
// --     0         1          0     | GP output low.
// --     0         1          1     | GP output high.
// --     1         x          x     | UART I/O.
// --
// --                                  I/O Reg 1 Pin Name
// -- MSB   7: DPIO 7 / UART 1 TXD.    ph_io_txd(PH)(0)
// --       6: DPIO 6 / UART 1 RXD.    ph_io_rxd(PH)(0)
// --       5: DPIO 5 / UART 1 RTSn.   ph_io_rtsn(PH)(0)
// --       4: DPIO 4 / UART 1 CTSn.   ph_io_ctsn(PH)(0)
// --       3: DPIO 3 / UART 1 DTRn.   ph_io_dtrn(PH)(0)
// --       2: DPIO 2 / UART 1 DSRn.   ph_io_dsrn(PH)(0)
// --       1: DPIO 1 / UART 1 DCDn.   ph_io_dcdn(PH)(0)
// -- LSB   0: DPIO 0 / UART 1 RIn.    ph_io_rin(PH)(0)


// ---------------------------  PHONE I/O REG 2  -------------------------------
// -- Phone I/O register 2 (RW):
// -- Reading the phone I/O register always returns the status at the DPIO pin.
// -- Writing the contents of the I/O register only affects the DPIO pin if
// -- the corresponding bit within the function I/O register is 0, and the
// -- bit is configured as an output within the I/O direction register.
// --
// --                                       I/O Reg 2 Pin Name
// -- MSB 7: DPIO 15 / UART 2 TXD status.   ph_io_txd(PH)(1)
// --     6: DPIO 14 / UART 2 RXD status.   ph_io_rxd(PH)(1)
// --     5: DPIO 13 / UART 2 RTSn status.  ph_io_rtsn(PH)(1)
// --     4: DPIO 12 / UART 2 CTSn status.  ph_io_ctsn(PH)(1)
// --     3: DPIO 11 / UART 2 DTRn status.  ph_io_dtrn(PH)(1)
// --     2: DPIO 10 / UART 2 DSRn status.  ph_io_dsrn(PH)(1)
// --     1: DPIO  9 / UART 2 DCDn status.  ph_io_dcdn(PH)(1)
// -- LSB 0: DPIO  8 / UART 2 RIn status.   ph_io_rin(PH)(1)

// -------------------------  PHONE I/O DIR REG 2  -----------------------------
// -- Phone I/O direction register 2 (RW):
// -- Specifies whether a DPIO pin is an output or an input pin.  If a DPIO's
// -- pin is set for an alternate function, then the corresponding directional
// -- control bit is ignored.  Reading direction register always returns the
// -- contents of the direction register irrespective of the DPIO pin function.
// --
// -- MSB 7-4: Phone DPIO 15-12 directional control (0 = input, 1 = output).
// -- LSB 3-0: Phone DPIO 11-8  directional control (0 = input, 1 = output).
// --

// ----------------------  PHONE I/O FUNCTION REG 2 ----------------------------
// -- Phone I/O function register 2 (RW):
// -- Specifies whether a phone's dual purpose I/O (DPIO) pin functions as a
// -- general purpose I/O pin or as a phone UART I/O signal.  Setting a bit
// -- within this register connects the corresponding UART signal to the
// -- phone's UART.  In this case, the I/O direction register is ignored.
// -- Reading the I/O register always returns the I/O pin status.
// --
// --  Function  Direction    I/O    | I/O
// --  register  register   register | Pin
// --     0         0          x     | GP input
// --     0         1          0     | GP output low.
// --     0         1          1     | GP output high.
// --     1         x          x     | UART I/O.
// --
// --                                   I/O Reg 1 Pin Name
// -- MSB   7: DPIO 15 / UART 2 TXD.    ph_io_txd(PH)(1)
// --       6: DPIO 14 / UART 2 RXD.    ph_io_rxd(PH)(1)
// --       5: DPIO 13 / UART 2 RTSn.   ph_io_rtsn(PH)(1)
// --       4: DPIO 12 / UART 2 CTSn.   ph_io_ctsn(PH)(1)
// --       3: DPIO 11 / UART 2 DTRn.   ph_io_dtrn(PH)(1)
// --       2: DPIO 10 / UART 2 DSRn.   ph_io_dsrn(PH)(1)
// --       1: DPIO  9 / UART 2 DCDn.   ph_io_dcdn(PH)(1)
// -- LSB   0: DPIO  8 / UART 2 RIn.    ph_io_rin(PH)(1)
// --

// ------------------------  PHONE CONFIGURATION  ------------------------------
// To avoid electrical contention when configuring the phone I/O, initialize
// each I/O set in the following order:
// 1) Set all I/O signals as inputs (zero the direction register).
// 2) Configure the function register.
// 3) Configure the I/O register.
// 4) Configure the direction register.

// HD1065 - MC75 Phone board.
// MC75 configuration board configuration:
//    Phone GPIO[0] = Not used.
//    Phone communication port 0 is a complete 8 wire interface that is
//     connected to UART 1.
//    Phone communication port 1 is connected to UART 2 and uses RXD, TXD,
//     CTSn, and RTSn.
//    The following UART 2 signals are connected as I/O to control the phone:
//       RIn  = IO_REG2[0] = Awake status (0/1 = on/off).
//       DCDn = IO_REG2[1] = Emergency reset control.  Toggle high at least 10
//                           mS to turn phone off.
//       DSRn = IO_REG2[2] = Ignition control.  Toggle high at least 400 mS to
//                           turn phone on.
//       DTRn = IO_REG2[3] = Not used.

// MC75 phone 'A' I/O register configuration
//    0x57000204: I/O direction reg 0: 0x00
//    0x57000200: I/O reg 0:           0x00

//    0x5700020C: I/O Direction reg 1: 0x00
//    0x57000210: I/O Function reg 1:  0xFF
//    0x57000208: I/O reg 1:           0x00
//    0x5700020C: I/O Direction reg 1: 0x00

//    0x57000218: I/O direction reg 2: 0x00
//    0x5700021C: I/O function reg 2:  0xF0
//    0x57000214: I/O reg 2:           0x00
//    0x57000218: I/O direction reg 2: 0x06

//
//*****************************  SSW INTERFACE  ********************************

// 0x0800 - 0x083F: SSW general registers
//          0x0800: SSW version register (RO).
//          0x0804: Reserved.
//          0x0808: SSW IRQ register (RO).
//          0x080C: Reserved: SSW IRQ register 2 (RO).
// 0x0840 - 0x08BF: SSW local phone interface registers.
//          0x0840: Phone 0 SIM selection register (RW).
//          0x0844: Phone 0 status/control register (RW).
//          0x0848: Phone 0 interrupt mask register (RW).
//          0x0850: Phone 1 SIM selection register (RW).
//          0x0854: Phone 1 status/control register (RW).
//          0x0858: Phone 1 interrupt mask register (RW).
//          0x0860: Phone 2 SIM selection register (RW).
//          0x0864: Phone 2 status/control register (RW).
//          0x0868: Phone 2 interrupt mask register (RW).
// 0x08C0 - 0x093F: SSW remote connection interface registers.
//          0x08C0: Remote connection 0 address register (RW).
//          0x08C4: Remote connection 0 status/control register (RW).
//          0x08D0: Remote connection 1 address register (RW).
//          0x08D4: Remote connection 1 status/control register (RW).
//          0x08E0: Remote connection 2 address register (RW).
//          0x08E4: Remote connection 2 status/control register (RW).
// 0x0940 - 0x09BF: SSW SIM card interface registers.
//          0x0940: SIM 1 status register (RO).
//          0x0944: SIM 1 LED register (RW).
//          0x0950: SIM 2 status register (RO).
//          0x0954: SIM 2 LED register (RW).
//          0x0960: SIM 3 status register (RO).
//          0x0964: SIM 3 LED register (RW).
//          0x0970: SIM 4 status register (RO).
//          0x0974: SIM 4 LED register (RW).
// 0x09C0 - 0x0BFF: SSW T=0 UARTs.
//          0x09C0: T=0 UART 1.
//          0x0A00: T=0 UART 2.
//          0x0A40: T=0 UART 3.

// -----------------------  SSW IRQ STATUS REGISTER  ---------------------------
// -- SSW IRQ status register (R0):  When any bit within the SSW IRQ status
// -- register is set then an interrupt is pending.
// --
// -- MSB 7: Reserved (set to 0).
// --     6: T=0 UART 2 IRQ.
// --     5: T=0 UART 1 IRQ.
// --     4: T=0 UART 0 IRQ.
// --     3: Reserved (set to 0).
// --     2: Phone 2 IRQ.
// --     1: Phone 1 IRQ.
// -- LSB 0: Phone 0 IRQ.

// ------------------------  PHONE SIM SELECTION REG  --------------------------
// -- Phone's SIM selection register (RW):
// --  MSB  7: Enable/Disable (1/0) SIM selection.
// --     6-2: Reserved (set to 0).
// --     1-0: SIM address.

// ------------------------  PHONE STATUS/CONTROL REG  -------------------------
// -- Phone's status/control register (RW):
// -- MSB   7: Delta phone's RSTn SIM signal status.
// --       6: Delta phone's VCC SIM/fault signal status.
// --       5: Phone's RSTn SIM signal status.
// --       4: Reserved (set to 0).
// --       3: Phone SIM VCC level fault status (1 = fault).
// --     2-1: Phone's SIM VCC level status:
// --          00 = off, 01 = 1.8V, 10 = 3.0V, 11 = 5.0V.
// -- LSB   0: SIM present control/status bit. When a SIM address has been
// --          enabled, this bit is read only.

// ------------------------  PHONE INTERRUPT MASK REG  -------------------------
// -- Phone's interrupt mask register (RW):
// -- MSB 7-2: Reserved (set to 0).
// --       1: RST signal change interrupt enable.
// -- LSB   0: VCC/fault signal change interrupt enable.

// ----------------------  REMOTE CONNECTION ADDRESS REG  ----------------------
// -- Remote connection address register (RW):  Determines the address of the
// -- destination device to which a T=0 UART is connected.  When connecting to
// -- a SIM, ensure that the SIM clock is disabled prior to enabling the SIM
// -- address to avoid SIM clock glitches.
// --
// --  MSB  7: Address enable/disable (1/0).
// --     6-2: Reserved (set to 0).
// --     1-0: Phone/SIM address.

// ------------------  REMOTE CONNECTION STATUS/CONTROL REG  -------------------
// -- Remote connection status/control register (RW):
// -- MSB   7: Device connection control: Phone/SIM (0/1).
// --       6: Monitor enable.  Monitor one-side of a local connection.  When
// --          monitoring one side of a local phone-SIM connection, the remote
// --          connection address register should contain the phone's address
// --          and not the SIM's address.
// --          Effectively, bits 7 and 6 mean the following:
// --          11 = Monitor the SIM side of a local phone-SIM connection.
// --          01 = Monitor the phone side of a local phone-SIM connection.
// --          10 = Communicate with a local SIM or monitor both sides of the
// --               connection.
// --          00 = Communicate with a local phone or monitor both sides of
// --               the connection.
// --     5-4: Reserved (set to 0).
// --       3: SIM RST signal control.
// --       2: SIM CLK enable control (0/1 = disable/enable).
// --     1-0: SIM VCC control (00 = off, 01 = 1.8V, 10 = 3.0V, 11 = 5.0V).

// ---------------------------  SIM STATUS REG  --------------------------------
// -- SIM status register (RO):
// -- MSB   7: ATR fault detection.
// --       6: SIM card electrical faults: VCC under voltage, VCC over current,
// --          clk/RST signal fault, chip over temp, and SIM removal during a
// --          transaction.  All these faults initiate a deactivation sequence.
// --       5: SIM RST signal status.
// --       4: SIM VCC ready.
// --       3: Phone's SIM VCC level fault (1 = fault).
// --     2-1: Requested SIM VCC level (00 = off, 01 = 1.8V, 10 = 3.0V, 11 = 5.0V)
// -- LSB   0: SIM present

// --------------------------  SIM LED REGISTER  -------------------------------
// -- SIM LED register (RW):  Each SIM socket has a dual color LED.  The LED
// -- may be controlled by the firmware or directly by the FPGA (default).
// -- The follow color definitions are used when the FPGA controls the LED:
// --
// --  OFF   = No SIM.
// --  GRN   = SIM present (VCC inactive).
// --  RED   = In-use (SIM present, VCC active).
// --  AMBER = SIM interface chip fault or a VCC control circuit fault.
// --
// -- MSB 7-4: Reserved (set to 0).
// --       3: Reserved: SIM blue LED control (0/1 = off/on).
// --       2: SIM green LED control (0/1 = off/on).
// --       1: SIM red LED control (0/1 = off/on).
// -- LSB   0: LED control source: 0/1 = SIM signals/reg.

//-------------------------  SSW T=0 UART SUPPORT  -----------------------------
//The FPGA SSW logic contains UARTs that have a 16450 compatible interface with
//extensions to to support the SIM T0 protocol.  The following registers and
//bits are supported:
//
//  0 - RBR(7-0): Receive buffer register (RO).
//      THR(7-0): Transmit holding register (WO).
//  1 - IER(3-0): Interrupt enable register (RW).
//  2 - IIR(3-0): Interrupt identification register (RO).
//  3 - LCR(7,5-0): Line control register (RW).
//    - The transmission of 1.5 stop bits is not supported.
//  4 - MCR(4,0): Modem control register (RW).
//    - The DTR signal is only available to support loop-back to the DSR input.
//  5 - LSR(6-5,3-0): Line status register (RO).
//      - The break interrupt flag (bit 4) is not supported.  When the SIM/T0
//        protocol is active, this bit indicates an unrecoverable transmit
//        error.  It is set when a character has been re-transmitted, according
//        to the SPC register, and an error signal was returned each time.  It
//        is cleared when the LSR is read.
//  6 - MSR(5,1): Modem status register (RO).
//  7 - SCR: Not supported.
//  8 - DLL(7-0): Divisor latch LSB (RW).
//      - This register is also accessible at offset 0 when LCR(7) is set.
//  9 - DLM(1-0): Divisor latch MSB (RW).
//      - This register is also accessible at offset 1 when LCR(7) is set.
//  A - SPC(3-0): SIM/T0 protocol control register (RW).
//         0: T0 error signal generation/detection: 0/1 = disabled/enabled.
//            When T0 error signal generation/detection is enabled, then the
//            number of stop bits is 2 regardless of LCR[2].
//       2-1: Error signal re-transmit count (0 - 3).
//         3: T0 coding convention: 0 = direct  (Tx LSB first. Z = 1).
//                                  1 = inverse (Tx MSB first. Z = 0).
//       7-4: Reserved (set to 0).
//
//
//Besides not supporting all the UART registers and bits, the UART varies from
//the 450 specification in the following:
//- The addressable memory space is 16 bytes, not 8.  Extra registers and
//  functionality are added through the upper 8 addressable memory locations.
//  Note: A1 and A0 are not decoded on the XScale FPGA interface.  Therefore
//  each 8 bit register occupies 32 bits in the memory map and 1 UART occupies
//  64 bytes.
//- When an over-run is detected, the RBR, LSR[PE], and LSR[FE] are NOT over
//  written.
//- Each T=0 UART has a 512 byte receive FIFO.
//
//For more information on the UART registers, refer to the 16450 data sheet.

#define NUM_PHONES  NUM_T0_PORTS
#define NUM_SIM_UARTS   NUM_T0_PORTS

#ifdef __KERNEL__
#ifdef LINUX_24
#define FPGA_REG_BASE               IXP425_EXP_BUS_CS7_BASE_PHYS
#else
#if defined(PHOENIX_PROBE)
#define FPGA_REG_BASE               (0x5000)
#else
#define FPGA_REG_BASE               IXP4XX_EXP_BUS_CS7_BASE_PHYS
#endif
#endif
#else
#if defined(PHOENIX_PROBE)
#define FPGA_REG_BASE               (0x5000)
#else
#define FPGA_REG_BASE               (0x57000000)
#endif
#endif
#define FPGA_REG_SIZE               (0xC00)

#if defined(PHOENIX_PROBE)
#define VER_TEST_MSB                (0x000)
#define VER_TEST_LSB                (0x001)
#define BUILDM                      (0x004)
#define BUILDL                      (0x005)
#define PHONE_BASE                  (0x020)
#define PHONE_OFFSET_MULT           (1)
#else
#define VER_TEST_MSB                (0x000)
#define VER_TEST_LSB                (0x004)
#define BUILDM                      (0x008)
#define BUILDL                      (0x00C)
#define PHONE_BASE                  (0x200)
#define PHONE_OFFSET_MULT           (4)
#endif

#define FPGA_VER_TEST_MSB           (FPGA_REG_BASE + VER_TEST_MSB)
#define FPGA_VER_TEST_LSB           (FPGA_REG_BASE + VER_TEST_LSB)
#define FPGA_BUILDM                 (FPGA_REG_BASE + BUILDM)
#define FPGA_BUILDL                 (FPGA_REG_BASE + BUILDL)

#define FPGA_PHONE_BASE             (FPGA_REG_BASE + PHONE_BASE)

#define PHONE_BASEx(_x_)            (PHONE_BASE + (0x8 * (_x_))* PHONE_OFFSET_MULT)
#define FPGA_PHONE_BASEx(_x_)       (FPGA_REG_BASE + PHONE_BASEx(_x_))
#define PHONE_REG_0(_x_)            (PHONE_BASE + (0x8 * (_x_) + 0)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_REG_0(_x_)       (FPGA_REG_BASE + PHONE_REG_0(_x_))
#define PHONE_DIR_REG_0(_x_)        (PHONE_BASE + (0x8 * (_x_) + 1)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_DIR_REG_0(_x_)   (FPGA_REG_BASE + PHONE_DIR_REG_0(_x_))
#define PHONE_REG_1(_x_)            (PHONE_BASE + (0x8 * (_x_) + 2)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_REG_1(_x_)       (FPGA_REG_BASE + PHONE_REG_1(_x_))
#define PHONE_DIR_REG_1(_x_)        (PHONE_BASE + (0x8 * (_x_) + 3)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_DIR_REG_1(_x_)   (FPGA_REG_BASE + PHONE_DIR_REG_1(_x_))
#define PHONE_FUNC_REG_1(_x_)       (PHONE_BASE + (0x8 * (_x_) + 4)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_FUNC_REG_1(_x_)  (FPGA_REG_BASE + PHONE_FUNC_REG_1(_x_))
#define PHONE_REG_2(_x_)            (PHONE_BASE + (0x8 * (_x_) + 5)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_REG_2(_x_)       (FPGA_REG_BASE + PHONE_REG_2(_x_))
#define PHONE_DIR_REG_2(_x_)        (PHONE_BASE + (0x8 * (_x_) + 6)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_DIR_REG_2(_x_)   (FPGA_REG_BASE + PHONE_DIR_REG_2(_x_))
#define PHONE_FUNC_REG_2(_x_)       (PHONE_BASE + (0x8 * (_x_) + 7)* PHONE_OFFSET_MULT)
#define FPGA_PHONE_FUNC_REG_2(_x_)  (FPGA_REG_BASE + PHONE_FUNC_REG_2(_x_))

#if defined(PHOENIX_PROBE)
#define SSW_IRQ                     (0x0A2)
#define LPH0_SIM                    (0x0A4)
#define LPH0_SCR                    (0x0A5)
#define LPH0_EVENT_IER              (0x0A6)
#define LPH1_SIM                    (0x0A8)
#define LPH1_SCR                    (0x0A9)
#define LPH1_EVENT_IER              (0x0AA)
#define LPH2_SIM                    (0x0AC)
#define LPH2_SCR                    (0x0AD)
#define LPH2_EVENT_IER              (0x0AE)
#define RC0_ADR                     (0x0B0)
#define RC0_SCR                     (0x0B1)
#define RC1_ADR                     (0x0B2)
#define RC1_SCR                     (0x0B3)
#define RC2_ADR                     (0x0B4)
#define RC2_SCR                     (0x0B5)
#define UART_BASE                   (0x0C0)
#define UART_OFFSET_MULT            (0x01)

#else
#define SSW_IRQ                     (0x808)
#define LPH0_SIM                    (0x840)
#define LPH0_SCR                    (0x844)
#define LPH0_EVENT_IER              (0x848)
#define LPH1_SIM                    (0x850)
#define LPH1_SCR                    (0x854)
#define LPH1_EVENT_IER              (0x858)
#define LPH2_SIM                    (0x860)
#define LPH2_SCR                    (0x864)
#define LPH2_EVENT_IER              (0x868)
#define RC0_ADR                     (0x8C0)
#define RC0_SCR                     (0x8C4)
#define RC1_ADR                     (0x8D0)
#define RC1_SCR                     (0x8D4)
#define RC2_ADR                     (0x8E0)
#define RC2_SCR                     (0x8E4)
#define UART_BASE                   (0x9C0)
#define UART_OFFSET_MULT            (0x04)
#endif

#define FPGA_SSW_IRQ                (FPGA_REG_BASE + SSW_IRQ)

#define FPGA_LPH0_SIM               (FPGA_REG_BASE + LPH0_SIM)
#define FPGA_LPH0_SCR               (FPGA_REG_BASE + LPH0_SCR)
#define FPGA_LPH0_EVENT_IER         (FPGA_REG_BASE + LPH0_EVENT_IER)
#define FPGA_LPH1_SIM               (FPGA_REG_BASE + LPH1_SIM)
#define FPGA_LPH1_SCR               (FPGA_REG_BASE + LPH1_SCR)
#define FPGA_LPH1_EVENT_IER         (FPGA_REG_BASE + LPH1_EVENT_IER)
#define FPGA_LPH2_SIM               (FPGA_REG_BASE + LPH2_SIM)
#define FPGA_LPH2_SCR               (FPGA_REG_BASE + LPH2_SCR)
#define FPGA_LPH2_EVENT_IER         (FPGA_REG_BASE + LPH2_EVENT_IER)

#define FPGA_RC0_ADR                (FPGA_REG_BASE + RC0_ADR)
#define FPGA_RC0_SCR                (FPGA_REG_BASE + RC0_SCR)
#define FPGA_RC1_ADR                (FPGA_REG_BASE + RC1_ADR)
#define FPGA_RC1_SCR                (FPGA_REG_BASE + RC1_SCR)
#define FPGA_RC2_ADR                (FPGA_REG_BASE + RC2_ADR)
#define FPGA_RC2_SCR                (FPGA_REG_BASE + RC2_SCR)

#define FPGA_UART_BASE              (FPGA_REG_BASE + UART_BASE)

#define UART_BASEx(_x_)             (UART_BASE + (0x10 * (_x_)) * UART_OFFSET_MULT)
#define FPGA_UART_BASEx(_x_)        (FPGA_REG_BASE + UART_BASEx(_x_))
#define UART_RHR(_x_)               (UART_BASE + (0x10 * (_x_) + 0) * UART_OFFSET_MULT)
#define FPGA_UART_RHR(_x_)          (FPGA_REG_BASE + UART_RHR(_x_))
#define UART_THR(_x_)               (UART_BASE + (0x10 * (_x_) + 0) * UART_OFFSET_MULT)
#define FPGA_UART_THR(_x_)          (FPGA_REG_BASE + UART_THR(_x_))
#define UART_IER(_x_)               (UART_BASE + (0x10 * (_x_) + 1) * UART_OFFSET_MULT)
#define FPGA_UART_IER(_x_)          (FPGA_REG_BASE + UART_IER(_x_))
#define UART_IIR(_x_)               (UART_BASE + (0x10 * (_x_) + 2) * UART_OFFSET_MULT)
#define FPGA_UART_IIR(_x_)          (FPGA_REG_BASE + UART_IIR(_x_))
#define UART_LCR(_x_)               (UART_BASE + (0x10 * (_x_) + 3) * UART_OFFSET_MULT)
#define FPGA_UART_LCR(_x_)          (FPGA_REG_BASE + UART_LCR(_x_))
#define UART_MCR(_x_)               (UART_BASE + (0x10 * (_x_) + 4) * UART_OFFSET_MULT)
#define FPGA_UART_MCR(_x_)          (FPGA_REG_BASE + UART_MCR(_x_))
#define UART_LSR(_x_)               (UART_BASE + (0x10 * (_x_) + 5) * UART_OFFSET_MULT)
#define FPGA_UART_LSR(_x_)          (FPGA_REG_BASE + UART_LSR(_x_))
#define UART_DLL(_x_)               (UART_BASE + (0x10 * (_x_) + 0) * UART_OFFSET_MULT)
#define FPGA_UART_DLL(_x_)          (FPGA_REG_BASE + UART_DLL(_x_))       // Use only in special cases - use FPGA_UART_DLLA instead!!
#define UART_DLH(_x_)               (UART_BASE + (0x10 * (_x_) + 1) * UART_OFFSET_MULT)
#define FPGA_UART_DLH(_x_)          (FPGA_REG_BASE + UART_DLH(_x_))       // Use only in special cases - use FPGA_UART_DLHA instead!!
#define UART_DLLA(_x_)              (UART_BASE + (0x10 * (_x_) + 8) * UART_OFFSET_MULT)
#define FPGA_UART_DLLA(_x_)         (FPGA_REG_BASE + UART_DLLA(_x_))
#define UART_DLHA(_x_)              (UART_BASE + (0x10 * (_x_) + 9) * UART_OFFSET_MULT)
#define FPGA_UART_DLHA(_x_)         (FPGA_REG_BASE + UART_DLHA(_x_))
#define UART_SPC(_x_)               (UART_BASE + (0x10 * (_x_) + 10) * UART_OFFSET_MULT)
#define FPGA_UART_SPC(_x_)          (FPGA_REG_BASE + UART_SPC(_x_))

#define FPGA_SSW_IRQ_PH0            (1<<0)
#define FPGA_SSW_IRQ_PH1            (1<<1)
#define FPGA_SSW_IRQ_PH2            (1<<2)
#define FPGA_SSW_IRQ_UART0          (1<<4)
#define FPGA_SSW_IRQ_UART1          (1<<5)
#define FPGA_SSW_IRQ_UART2          (1<<6)

#define FPGA_LPH_SCR_SIM_PRESENT    (1<<0)
#define FPGA_LPH_SCR_VCC_1_8V       (1<<1)
#define FPGA_LPH_SCR_VCC_3_0V       (1<<2)
#define FPGA_LPH_SCR_VCC_5_0V       (FPGA_LPH_SCR_VCC_1_8V|FPGA_LPH_SCR_VCC_3_0V)
#define FPGA_LPH_SCR_VCC_STATUS     FPGA_LPH_SCR_VCC_5_0V
#define FPGA_LPH_SCR_VCC_LVL_ERR    (1<<3)
#define FPGA_LPH_SCR_RST_STATUS     (1<<5)
#define FPGA_LPH_SCR_DELTA_VCC      (1<<6)
#define FPGA_LPH_SCR_DELTA_RST      (1<<7)

#define FPGA_LPH_VCC_INT_EN         (1<<0)
#define FPGA_LPH_RST_INT_EN         (1<<1)

#define FPGA_RC_SCR_CLK_EN          (1<<2)
#define FPGA_RC_SCR_MON_EN          (1<<6)
#define FPGA_RC_SCR_SIM_SIDE        (1<<7)

#define IIR_RCV_FIFO_ENABLE         (0x01)

