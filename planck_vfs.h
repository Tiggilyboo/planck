#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

struct file *file_open(const char *path, int flags, int rights) 
{
  struct file *filp = NULL;
  int err = 0;

  filp = filp_open(path, flags, rights);
  if (IS_ERR(filp)) {
    err = PTR_ERR(filp);
    return NULL;
  }
  return filp;
}

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
  int status;
  status = kernel_write(file, data, size, &offset);

  return status;
}
