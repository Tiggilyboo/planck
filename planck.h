#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include "planck_i2c.h"
#include "planck_keycodes.h"
#include "planck_hid.h"
#include "f_hid.h"

#define DEVICE_NAME       "planck"
#define GPIO_INTERRUPT    32
#define GPIO_DEBOUNCE     50

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Willshire");
MODULE_DESCRIPTION("Planck internal i2c and external HID keyboard driver");
MODULE_VERSION("0.1");
MODULE_INFO(intree, "Y");

struct planck_device {
  struct workqueue_struct* read_wq;
  
  struct i2c_client *i2c;
  spinlock_t irq_lock;
  unsigned int irq_number;
  uint16_t state;

  bool internal;
  struct input_dev *input;
};

struct planck_i2c_work {
  struct work_struct work;
  struct planck_device *device;
};

static struct of_device_id planck_ids[] = {{.compatible = DEVICE_NAME},{}};
static const struct i2c_device_id planck_id[] = { {DEVICE_NAME, 0}, {}};
static irq_handler_t planck_gpio_interrupt(unsigned int irq, void *dev_id, struct pt_regs *regs);
static void planck_work_handler(struct work_struct *w);

DECLARE_WORK(workqueue, planck_work_handler);

MODULE_DEVICE_TABLE(i2c, planck_id);
