#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

struct file *file_open(const char *path, int flags, int rights) 
{
  struct file *filp = NULL;
  mm_segment_t oldfs;
  int err = 0;

  oldfs = get_fs();
  set_fs(get_ds());
  filp = filp_open(path, flags, rights);
  set_fs(oldfs);
  if (IS_ERR(filp)) {
    err = PTR_ERR(filp);
    return NULL;
  }
  return filp;
}

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
  int status;
  mm_segment_t oldfs;

  oldfs = get_fs();
  set_fs(get_ds());

  status = vfs_write(file, data, size, &offset);
  set_fs(oldfs);

  return status;
}
