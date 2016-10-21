// Include support
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <asm/termios.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <asm/ioctls.h>
#include <linux/tty.h>
#include <linux/poll.h>
#include "usbsup.h"
#include "ampsup.h"
#include "rcats_msgs.h"

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif


// Configure terminal for target
#if 0

  // From application code com port configuration
/*
   tio.c_iflag = 0;  //eventually becomes 0x80000000, this bit is undefined in bits\termios.h, but cfmakeraw sets it.
   tio.c_oflag = ONLCR; //0x04
   tio.c_cflag = HUPCL | CLOCAL |CREAD | CS8;   //0x1CB1(octal=16261), but we're not setting baud rate here.
   tio.c_lflag = ECHOE | ECHOK | ECHOCTL | ECHOKE;  //0xA30(octal=5060)
*/

  // Access terminal
  tty=(struct tty_struct*)file->private_data;

  tty->termios->c_iflag = 0;
  tty->termios->c_oflag = ONLCR;        // Map NL to CR-NL on output

  tty->termios->tio.c_cflag = 0;
  tty->termios->tio.c_cflag != HUPCL;   // Lower modem control lines after last process closes the device (hang up).
  tty->termios->tio.c_cflag != CLOCAL;  // Ignore modem control lines.
  tty->termios->tio.c_cflag != CREAD;   // Enable receiver
  tty->termios->tio.c_cflag != CS8;     // Character size mask

  tty->termios->tio.c_iflag = 0;
  tty->termios->tio.c_iflag != ECHOE;   // If ICANON is also set, the ERASE character erases the preceding input character
                                        //  and WERASE erases the preceding word
  tty->termios->tio.c_iflag != ECHOK;   // If ICANON is also set, the KILL character erases the current line.
  tty->termios->tio.c_iflag != ECHOCTL; // (not in POSIX) If ECHO is also set, terminal special characters other than TAB,
                                        // NL, START, and STOP are echoed as ^X, where X is the character with ASCII code
                                        // 0x40 greater than the special character. For example, character 0x08 (BS) is
                                        // echoed as ^H. [requires _BSD_SOURCE or _SVID_SOURCE]
  tty->termios->tio.c_iflag != ECHOKE;  // (not in POSIX) If ICANON is also set, KILL is echoed by erasing each character on
                                        // the line, as specified by ECHOE and ECHOPRT. [requires _BSD_SOURCE or _SVID_SOURCE]

  // From application code com port configuration
/*
  if (setup_serial_port(m_fd, m_nBaudrate, 0, 1) != 0 )
*/
//  cfsetispeed(options, baud_mask);
//  cfsetospeed(options, baud_mask);
  tty->termios->options->c_cflag |= B115200;

  // From application code com port configuration
/*
  cfmakeraw(&tio);
*/

  // This seems to be of no use since we do not set any of
  // the following flags anyways
  tty->termios->tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                  | INLCR | IGNCR | ICRNL | IXON);
  tty->termios->tio.c_oflag &= ~OPOST;
  tty->termios->tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty->termios->tio.c_cflag &= ~(CSIZE | PARENB);
  tty->termios->tio.c_cflag |= CS8;

  // From application code com port configuration
/*
   tio.c_cc[VTIME] = vtime;
   tio.c_cc[VMIN] = vmin;
*/
  tty->termios->c_cc[VTIME] = 1 ;       // Timeout in deciseconds for noncanonical read (TIME).
  tty->termios->c_cc[VMIN]  = 0;        // Minimum number of characters for noncanonical read (MIN).

#endif

//
//  tty_setspeed
//
static void
tty_setspeed(struct file *file, int speed)
{
  // If not development board...
#ifndef SIM_DEV_BOARD

  // Locals
	mm_segment_t oldfs;
	struct tty_struct *tty;

  // Set kernel context
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	/*  Set speed */
	struct termios settings;

  // Access terminal
  tty=(struct tty_struct*)file->private_data;

	tty->termios->c_iflag = 0;
	tty->termios->c_oflag = 0;
	tty->termios->c_lflag = 0;
	tty->termios->c_cflag = CLOCAL | CS8 | CREAD;
	tty->termios->c_cc[VMIN] = 1;
	tty->termios->c_cc[VTIME] = 0;
	switch (speed) {
    case 115200:{
		tty->termios->c_cflag |= B2400;
	  }
	  break;
    default:{
    		tty->termios->c_cflag |= B9600;
    	}
    	break;
  }

  // Restore context
  set_fs(oldfs);

#endif  // SIM_DEV_BOARD
}

//
//  tty_read
//
#define BUFF_SIZE 64
static int
tty_read(struct file *f, int timeout, unsigned char *string)
{
  int result;
  unsigned char buff[BUFF_SIZE + 1];
  int length;
  int index;

  result = -1;

  // If not errored...
  if (!IS_ERR(f)) {

    mm_segment_t oldfs;

    oldfs = get_fs();
    set_fs(KERNEL_DS);

    if (f->f_op->poll)
    {
      struct poll_wqueues table;
      struct timeval start, now;

      do_gettimeofday(&start);
      poll_initwait(&table);

      while (1)
      {
        long elapsed;
        int mask;

        mask = f->f_op->poll(f, &table.pt);

        if (mask & (POLLRDNORM | POLLRDBAND | POLLIN |
          POLLHUP | POLLERR))
        {
            break;
        }

        do_gettimeofday(&now);
        elapsed =
        (1000000 * (now.tv_sec - start.tv_sec) +
        now.tv_usec - start.tv_usec);

        if (elapsed > timeout)
        {
          break;
        }

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(((timeout -
        elapsed) * HZ) / 10000);
      }

      poll_freewait(&table);
    }

    f->f_pos = 0;

    if ((length = f->f_op->read(f, buff, BUFF_SIZE, &f->f_pos)) > 0)
    {
#if 0
      for (index = 0; index < length; index++)
        printk(KERN_ALERT "simdrv: tty_read: data[%d] = 0x%x\n",
               index, buff[index]);
#endif

      // Argh - AMP prepends \n\r to response buffer.
      // We do not want this or it will corrupt our
      // response buffer.

      // Terminate
      buff[length] = '\0';

      // Copy skipping for two characters
      strcpy(string, &buff[2]);

      // Success
      result = 0;
    }

    // Restore context
    set_fs(oldfs);
  }

  // Return result
  return(result);
}

// Keep file open or not?
#define USB_KEEP_FILE_OPEN

// Our array of filename pointers for AMP control devices
// Use full array so we can use phone ID as index;
static int filePointersInitiaized = 0;
static struct file *filePointers[NUM_DEVICES];

void
closeUsbCtrDevices(
  void
)
{
  // Locals
  int index = 0;

  // For all pointers...
  for (index = 0; index < NUM_DEVICES; index++)
  {
    // if opened...
    if (filePointers[index])
    {
      // Close and zero
      filp_close(filePointers[index], 0);
      filePointers[index] = 0;
    }
  }  
}

//
//  tty_request
//
int
tty_request(
  int thePhoneId,
  char *filename,
  char *data,
  ssize_t len)
{
  int ret = 0;
  mm_segment_t old_fs;
  u32 pos = 0;
  struct file *file;
  char * response = 0;

  // If file pointers not initialized...
  if (!filePointersInitiaized)
  {
    // For all pointers...
    for (ret = 0; ret < NUM_DEVICES; ret++)
    {
      // Initialize
      filePointers[ret] = NULL;
    }

    // Initialized
    filePointersInitiaized = 1;
  }

  // Set kernel context
  old_fs = get_fs();
  set_fs(get_ds());

#if 0
  for (ret = 0; ret < len; ret++)
    printk(KERN_ALERT "simdrv: phonesim%d: data[%d] = 0x%x\n",
           thePhoneId, ret, data[ret]);
#endif

  // Reset ret (maybe used above)
  ret = 0;

  // Access file pointer
  file = filePointers[thePhoneId];

  // If file not already opened...
  if (!file)
  {
    // Open control path
    file = filp_open(filename, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);

    // If failed to open...
    if (IS_ERR(file))
    {
      // Log
      printk(KERN_ALERT "simdrv: phonesim%d: failed (open %d) %s\n",
            thePhoneId, PTR_ERR(file), filename);

      // Failed
      ret = -EIO;

      // Exit
      goto out;
    }

    // Configure tty
    tty_setspeed(file, 115200);

    // Update entry
    filePointers[thePhoneId] = file;
  }

  // Send command
  ret = vfs_write(file, (char *)data + pos, len - pos, &file->f_pos);

  // Log
#ifdef SIM_DEBUG_TRACE  
  printk(KERN_ALERT
         "simdrv: phonesim%d: vfs_write pos %d len %d ret %d\n",
           thePhoneId, pos, len - pos, ret);
#endif

  if (ret < 0)
        goto out;

  if (ret == 0)
  {
    ret = -EIO;
    goto out;
  }

// If development board we cannot get a response
// back so let's fake it
#ifdef SIM_DEV_BOARD
  // request:     FPGA|GET|<address>
  // response:    FPGA|GET|<address>|<value>|OK

  // request:     FPGA|SET|<address>|<value>
  // response:    FPGA|GET|<address>|<value>|OK

  // Kill \r
  data[len - 1] = 0;

  // Set response pointe
  response = &data[len - 1];

  // If GET...
  if (strstr(data, "GET") != NULL)
  {
    // Dummy value
    response += sprintf(response, "0x%x", 0x00);
  }

  // Terminate
  response += sprintf(response,
                      "%c%s",
                      RCTN_DELIM,
                      RCTN_RESULT_SUCCESS);

  // Force success
  ret = 0;

  // Done
  goto out;

#endif

  // Read response
  ret = tty_read(file, 1000, &data[0]);

out:

  // If not errored...
  if (!IS_ERR(file))
  {
    // If not keeping file open...
#ifndef USB_KEEP_FILE_OPEN
    // Forget file pointer
    filePointers[ret] = NULL;

    // Close
    filp_close(file, 0);
#endif  
  }

  // Restore context
  set_fs(old_fs);

  // Return result
  return(ret);
}

//
//  usbio
//
int
usbio(
  int thePhoneId,
  char *theBuffer,
  size_t theLength)
{
  // Locals
  char * myFileName = NULL;
  char * myInsertionPointer = NULL;
  int ret = 0;

  // Access AMP control device
  myFileName = amp_getCtrlDevice(thePhoneId);

  // Terminate the message with linefeed - needed for AMP
  theBuffer[theLength++] = '\n';

  // Do the work
  ret = tty_request(thePhoneId, myFileName, theBuffer, theLength);

  // Transactions to AMP puts control characters at end of message.
  // Get rid of them.
  myInsertionPointer = theBuffer;
  while (*myInsertionPointer != 0)
  {
    // If carriage return or line feed...
    if (*myInsertionPointer == '\r' ||
        *myInsertionPointer == '\n')
    {
      // Kill it
      *myInsertionPointer = 0;
    }
    // Onwards
    myInsertionPointer++;
  }

  // Return result
  return(ret);
}