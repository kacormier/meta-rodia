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
#include "usbsup.h"

// If new(er) kernel than Phoenix...
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,7,0) /* not > 2.6, by now */
// Enable dev board support
#define SIM_DEV_BOARD
#endif

#ifdef SIM_DEV_BOARD
// Define path to control interface
#define SIM_DRV_AMP_CTRL_PATH "/tmp/extusb"
#else
// Define path to control interface
#define SIM_DRV_AMP_CTRL_PATH "/var/run/dev/extusb"
#endif

int 
write_file(
  int thePhoneId,
  char *filename, 
  char *data,
  ssize_t len)
{
  int ret = 0;
  mm_segment_t old_fs;
  u32 pos = 0;
  struct file *file;
  char * myData = 0;

  old_fs = get_fs();
  set_fs(get_ds());

// If development board open as regular file in /tmp.
// This will allow us appending of commands to visually
// validate but we cannot receive response data.
//
// If target board open real device file. 
#ifdef SIM_DEV_BOARD
  // Open control path
  file = filp_open(filename, O_RDWR | O_NOCTTY | O_NONBLOCK |O_CREAT, 0777);
#else
  // Open control path
  file = filp_open(filename, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
#endif

  // If failed to open...
  if (IS_ERR(file))
  {
    // Log
    printk(KERN_ALERT "simdrv: phonesim%d: failed (open %d) %s", 
           thePhoneId, PTR_ERR(file), filename);
           
    ret = -EIO;
    goto out;
  }

  file->f_pos = 0;

  while (pos < len) {
           
    // Log
    printk(KERN_ALERT "simdrv: phonesim%d: vfs_write pos %d len %d fpos %d", 
           thePhoneId, pos, len, file->f_pos);
    
    ret = vfs_write(file, (char *)data + pos, len - pos, &file->f_pos);
    
    /* TODO handle that correctly */
    /*if (ret == -ERESTARTSYS) {
            continue;
    }*/
    if (ret < 0)
            goto out;
    if (ret == 0) {
            ret = -EIO;
            goto out;
    }
    pos += ret;
    file->f_pos += ret;
  }

  ret = 0;

  filp_close(file, 0);

out:
  
  // If failed...
  if (ret)
  {
    // Log
    printk(KERN_ALERT "simdrv: phonesim%d: failed (write) %s", 
           thePhoneId , data);
  }
  
  set_fs(old_fs);
  return ret;
}

int
usbio(
  int thePhoneId, 
  char *theBuffer, 
  size_t theLength)
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
  }
  
  // Set insertion pointer
  myInsertionPointer = myInstancePointer;  
  
  // Add instance (phonesim 3,4,5 to extusb 0,1,2)
  theCtrlId = thePhoneId;
  theCtrlId -= 3;
  *myInsertionPointer++ = '0' + theCtrlId;
  *myInsertionPointer = 0;
  myInsertionPointer += sprintf(myInsertionPointer, "_ctrl");
  
  // Do the work
  return(write_file(thePhoneId, myFileName, theBuffer, theLength));
}