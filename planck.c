#include "planck.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Willshire");
MODULE_DESCRIPTION("Planck i2c keyboard driver for mcp23017");
MODULE_VERSION("0.1");
MODULE_INFO(intree, "Y");

static unsigned int irq_number;
static struct planck_device *device;

static int i2c_read_byte(struct i2c_client *client, unsigned char command) {
  int ret;
  ret = i2c_smbus_read_byte_data(client, command);
  printk(KERN_DEBUG "planck: reading reg %d returned value %d\n", command, ret);

  return ret;
}

static int i2c_write_byte(struct i2c_client *client, int reg, int value) {
  int ret;
  ret = i2c_smbus_write_byte_data(client, reg, value);
  printk(KERN_DEBUG "planck: writing reg %d with value %d", reg, value);

  return ret;
}

static int planck_probe(struct i2c_client *client, const struct i2c_device_id *id) {
  int ret;
  printk("planck: probe!!!");

  if(!i2c_check_functionality(client->adapter, 
        I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
        I2C_FUNC_SMBUS_I2C_BLOCK))
  {
    printk(KERN_ERR "planck: %s needed i2c functionality is not supported\n", __func__);
    return -ENODEV;
  }
  device = kzalloc(sizeof(struct planck_device), GFP_KERNEL);
  if(device == NULL)
  {
    printk(KERN_ERR "planck: %s: no memory\n", __func__);
    return -ENOMEM;
  }
  printk(KERN_DEBUG "planck: probed, set device client\n");
  device->client = client;

  printk(KERN_DEBUG "planck: setting up inputs and pullups");
  // Setup all inputs
  ret = i2c_write_byte(client, MCP23017_IODIRA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_IODIRB, 0xff);
  if(ret != 0) goto i2c_err;

  // Setup pullups
  ret = i2c_write_byte(client, MCP23017_GPPUA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_GPPUB, 0xff);
  if(ret != 0) goto i2c_err;

  // Setup interrupts (all handled/high)
  ret = i2c_write_byte(client, MCP23017_GPINTENA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_GPINTENB, 0xff);

  // Setup interrupt mirrorring
  ret = i2c_write_byte(client, MCP23017_IOCONA, 0x40);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_IOCONB, 0x40);


  goto i2c_ok;
i2c_err:
    printk(KERN_ERR "planck: unable to write to config register, returned error code %d\n", ret);
    return ret;

i2c_ok:
  return 0;
}

static int planck_remove(struct i2c_client *client) {
  struct planck_device *dev;
  dev = i2c_get_clientdata(client);
  kfree(dev);
  if(device != NULL){
    kfree(device);
  }

  printk(KERN_DEBUG "planck: removed\n"); 
  return 0;  
}

MODULE_DEVICE_TABLE(i2c, planck_id);

static struct i2c_driver planck_driver = {
  .driver = {
    .name = "planck",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(planck_ids),
  },
  .probe = planck_probe,
  .remove = planck_remove,
  .id_table = planck_id,
};

static int __init planck_init(void) {
  printk(KERN_DEBUG "planck: initialising...");

  // i2c
  int res;
  res = i2c_add_driver(&planck_driver);
  if(res != 0) {
    printk(KERN_DEBUG "planck: driver registration failed, module not inserted.\n");
    return res;
  }

  // GPIO
  res = gpio_is_valid(GPIO_INTERRUPT);
  if(!res)
  {
    printk(KERN_INFO "planck: invalid interrupt GPIO pin %d\n", GPIO_INTERRUPT);
    return -ENODEV;
  }
  gpio_request(GPIO_INTERRUPT, "gpio_interrupt");
  gpio_direction_input(GPIO_INTERRUPT);
  //gpio_set_debounce(GPIO_INTERRUPT, 50);
  gpio_export(GPIO_INTERRUPT, false);

  // irq
  irq_number = gpio_to_irq(GPIO_INTERRUPT);
  printk(KERN_DEBUG "planck: the interrupt is mapped to irq %d\n", irq_number);

  res = request_irq(irq_number, (irq_handler_t) planck_interrupt, IRQF_TRIGGER_RISING, "planck_interrupt", "planck");
  printk(KERN_DEBUG "planck: the interrupt request result is %d\n", res);

  printk(KERN_DEBUG "planck: inited\n");
  return res;
}

static void __exit planck_exit(void) {
  printk(KERN_DEBUG "planck: exiting...");
  
  // i2c
  i2c_del_driver(&planck_driver);

  // GPIO & irq
  gpio_unexport(GPIO_INTERRUPT);
  free_irq(irq_number, "planck");
  gpio_free(GPIO_INTERRUPT);

  printk("planck: exited");
}

static irq_handler_t planck_interrupt(unsigned int irq, void *dev_id, struct pt_regs *regs){
  printk(KERN_DEBUG "planck: interrupted with irq %d for device '%p'\n", irq, dev_id);
  if(device == NULL || device->client == NULL)
  {
    printk(KERN_ERR "planck: interrupt received, but no i2c device has been set up!\n");
    return (irq_handler_t)IRQ_NONE;
  }
  uint16_t ba = 0;
  uint8_t a;

  // read the i2c
  a = i2c_read_byte(device->client, MCP23017_GPIOA); 
  printk(KERN_DEBUG "planck: read GPIO A: "BYTE_TO_BIN_PAT"\n", BYTE_TO_BIN(a));

  ba = i2c_read_byte(device->client, MCP23017_GPIOB);
  printk(KERN_DEBUG "planck: read GPIO B: "BYTE_TO_BIN_PAT"\n", BYTE_TO_BIN(ba));

  ba <<= 8;
  ba |= a;
  printk(KERN_DEBUG "planck: read u16 as: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"\n", BYTE_TO_BIN(ba>>8), BYTE_TO_BIN(ba));

  device->state = ba;

  return (irq_handler_t)IRQ_HANDLED;
}

module_init(planck_init);
module_exit(planck_exit);
