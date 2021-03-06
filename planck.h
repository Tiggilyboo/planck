#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include "planck_i2c.h"
#include "planck_keycodes.h"
#include "planck_hid.h"
#include "planck_vfs.h"
#include "f_hid.h"

#define DEVICE_NAME           "planck"
#define GPIO_JIFFY_DELAY      3
#define PLANCK_BOOT_INTERNAL  1

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Willshire");
MODULE_DESCRIPTION("Planck internal i2c and external HID keyboard driver");
MODULE_VERSION("0.1");

MODULE_SOFTDEP("pre: libcomposite");
MODULE_SOFTDEP("pre: usb_f_hid");

struct planck_device {
  struct i2c_client *i2c;
  struct workqueue_struct* write_wq;
  struct planck_row_work* row_worker;
  bool internal;
  struct input_dev *input;
};

struct planck_i2c_work {
  struct work_struct work;
  struct planck_device *device;
};
struct planck_row_work {
  struct delayed_work work;
  struct planck_device *device;
  uint16_t last_state[MAX_Y];
  unsigned int layer;
};

static struct of_device_id planck_ids[] = {{.compatible = DEVICE_NAME},{}};
static const struct i2c_device_id planck_id[] = { {DEVICE_NAME, 0}, {}};
static void planck_row_work_handler(struct delayed_work *w);

MODULE_DEVICE_TABLE(i2c, planck_id);
