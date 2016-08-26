// Include support
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include "usbsup.h"


int 
write_file(
  char *filename, 
  char *data,
  ssize_t len)
{
  int ret;
  mm_segment_t old_fs;
  u32 pos = 0;
  struct file *file;

  old_fs = get_fs();
  set_fs(KERNEL_DS);

  file = filp_open(filename, O_WRONLY|O_CREAT, 0777);
  
  if (file == -1)
    goto out;

  while (pos < len) {
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
  }

  ret = 0;

  filp_close(file, 0);

out:
  set_fs(old_fs);
  return ret;
}

int
usbio(
  int thePhoneId, 
  char *theBuffer, 
  size_t theLength)
{
  // TBD
  return(0);
}