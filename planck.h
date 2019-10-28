#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include "planck_keycodes.h"
#include "planck_hid.h"

#define DEVICE_NAME       "planck"

#define GPIO_INTERRUPT    32
#define GPIO_DEBOUNCE     50

#define MCP23017_IODIRA   0x00
#define MCP23017_IPOLA    0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA  0x06
#define MCP23017_INTCONA  0x08
#define MCP23017_IOCONA   0x0A
#define MCP23017_GPPUA    0x0C
#define MCP23017_INTFA    0x0E
#define MCP23017_INTCAPA  0x10
#define MCP23017_GPIOA    0x12
#define MCP23017_OLATA    0x14  
#define MCP23017_IODIRB   0x01
#define MCP23017_IPOLB    0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB  0x07
#define MCP23017_INTCONB  0x09
#define MCP23017_IOCONB   0x0B
#define MCP23017_GPPUB    0x0D
#define MCP23017_INTFB    0x0F
#define MCP23017_INTCAPB  0x11
#define MCP23017_GPIOB    0x13
#define MCP23017_OLATB    0x15

#define BYTE_TO_BIN(b)\
(b & 0x80 ? '1' : '0'),\ 
(b & 0x40 ? '1' : '0'),\ 
(b & 0x20 ? '1' : '0'),\ 
(b & 0x10 ? '1' : '0'),\ 
(b & 0x08 ? '1' : '0'),\ 
(b & 0x04 ? '1' : '0'),\ 
(b & 0x02 ? '1' : '0'),\ 
(b & 0x01 ? '1' : '0')
#define BYTE_TO_BIN_PAT "%c%c%c%c%c%c%c%c"

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
