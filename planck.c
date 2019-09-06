#include "planck.h"

static int i2c_read_byte(struct i2c_client *client, unsigned char command) {
  return i2c_smbus_read_byte_data(client, command);
}
static int i2c_write_byte(struct i2c_client *client, int reg, int value) {
  return i2c_smbus_write_byte_data(client, reg, value);
}
static uint16_t planck_read_state(struct planck_device *device, int reg)
{
  uint16_t ba;
  uint8_t a;

  a = i2c_read_byte(device->client, reg); 
  ba = i2c_read_byte(device->client, reg+1);
  ba <<= 8;
  ba |= a;
  printk(KERN_DEBUG "planck: state is now: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"\n", BYTE_TO_BIN(ba>>8), BYTE_TO_BIN(ba));

  return ba;
}

static void planck_work_handler(struct work_struct *w)
{
  struct planck_i2c_work *work = (struct planck_i2c_work*)w;
  struct planck_device *dev = work->device;
  unsigned long irq_flags = work->irq_flags;
 
  printk("planck: work handler called with flags %d and device %d", irq_flags, dev == NULL ? 1 : 0);
  
  dev->state = planck_read_state(dev, MCP23017_INTCAPA);
  spin_unlock_irqrestore(&dev->irq_lock, irq_flags);

  kfree(w);

  return;
}
static int planck_queue_i2c_work(struct planck_device *device, unsigned long irq_flags)
{
  struct planck_i2c_work* work = (struct planck_i2c_work*)kmalloc(sizeof(struct planck_i2c_work), GFP_KERNEL);
  if(!work) 
    return -ENOMEM;

  INIT_WORK((struct work_struct*)work, planck_work_handler);
  work->device = device;
  work->irq_flags = irq_flags;

  return queue_work(device->read_wq, (struct work_struct*)work);
}

static int planck_configure_gpio(struct planck_device *device)
{
  int res;
  printk("planck: configuring GPIO...\n");

  res = gpio_is_valid(GPIO_INTERRUPT);
  if(!res)
  {
    printk(KERN_INFO "planck: invalid interrupt GPIO pin %d\n", GPIO_INTERRUPT);
    return -ENODEV;
  }
  gpio_request(GPIO_INTERRUPT, "gpio_interrupt");
  gpio_direction_input(GPIO_INTERRUPT);
  gpio_set_value(GPIO_INTERRUPT, 0);
  gpio_set_debounce(GPIO_INTERRUPT, GPIO_DEBOUNCE);

  // irq
  device->irq_number = gpio_to_irq(GPIO_INTERRUPT);
  printk(KERN_DEBUG "planck: the interrupt gpio pin %d is mapped to irq %d\n", GPIO_INTERRUPT, device->irq_number);

  res = request_irq(device->irq_number, (irq_handler_t)planck_gpio_interrupt, IRQF_TRIGGER_RISING, "planck_interrupt", device);
  printk(KERN_DEBUG "planck: the interrupt request result is %d\n", res);
  
  return res;
}

static int planck_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
  struct planck_device *device;
  unsigned long flags;
  int ret;

  printk(KERN_DEBUG "planck: probe!!!");
  if(!i2c_check_functionality(client->adapter, 
        I2C_FUNC_SMBUS_BYTE_DATA | 
        I2C_FUNC_SMBUS_WORD_DATA |
        I2C_FUNC_SMBUS_I2C_BLOCK))
  {
    printk(KERN_ERR "planck: %s needed i2c functionality is not supported\n", __func__);
    return -ENODEV;
  }
  printk(KERN_DEBUG "planck: configuring i2c");

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
 
  // Setup interrupt mirrorring disable SEQOP, polarity HIGH 
  ret = i2c_write_byte(client, MCP23017_IOCONA, 0x62);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_IOCONB, 0x62);
  if(ret != 0) goto i2c_err;
 
  // Setup interrupt compare register
  ret = i2c_write_byte(client, MCP23017_INTCONA, 0x00);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_INTCONB, 0x00);
  if(ret != 0) goto i2c_err;

  // Setup interrupt default value register
  ret = i2c_write_byte(client, MCP23017_DEFVALA, 0x00);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_DEFVALB, 0x00);
  if(ret != 0) goto i2c_err;
  
  // Setup interrupt on change
  ret = i2c_write_byte(client, MCP23017_GPINTENA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_GPINTENB, 0xff);
  if(ret != 0) goto i2c_err;

  printk(KERN_DEBUG "planck: i2c configured\n");
  goto i2c_ok;

i2c_err:
  printk(KERN_ERR "planck: unable to write to i2c register, returned error code %d\n", ret);
  return ret;

i2c_ok:
  printk(KERN_DEBUG "planck: setting up device...\n");
  device = devm_kzalloc(&client->dev, sizeof(struct planck_device), GFP_KERNEL);
  if(device == NULL)
    return -ENOMEM; 

  device->client = client;
  device->read_wq = create_workqueue("planck_workqueue");
  if(device->read_wq == NULL)
    return -ENOMEM;

  spin_lock_init(&device->irq_lock); 

  ret = planck_configure_gpio(device);
  if(ret != 0)
  {
    printk(KERN_ERR "planck: unable to configure gpio, returned: %d\n", ret);
    return ret;
  }
  spin_lock_irqsave(&device->irq_lock, flags);

  i2c_set_clientdata(client, device);
  device->state = planck_read_state(device, MCP23017_INTCAPA);

  spin_unlock_irqrestore(&device->irq_lock, flags);

  printk(KERN_DEBUG "planck: probed.\n");

  return 0;
}

static int planck_remove(struct i2c_client *client) {
  struct planck_device *dev;
  dev = i2c_get_clientdata(client);

  free_irq(dev->irq_number, dev);
  flush_workqueue(dev->read_wq);
  destroy_workqueue(dev->read_wq);
  gpio_free(GPIO_INTERRUPT);
  kfree(dev);

  printk(KERN_DEBUG "planck: removed\n"); 
  return 0;  
}

static struct i2c_driver planck_driver = {
  .driver = {
    .name = DEVICE_NAME,
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(planck_ids),
  },
  .probe = planck_probe,
  .remove = planck_remove,
  .id_table = planck_id,
};

static int __init planck_init(void) {
  int res;
  printk(KERN_DEBUG "planck: initialising...");

  res = i2c_add_driver(&planck_driver);
  if(res != 0) {
    printk(KERN_DEBUG "planck: driver registration failed, module not inserted.\n");
    return res;
  }

  printk(KERN_DEBUG "planck: inited\n");
  return res;
}

static void __exit planck_exit(void) 
{
  printk(KERN_DEBUG "planck: exiting...");
  i2c_del_driver(&planck_driver);
  printk("planck: exited");
}

static irq_handler_t planck_gpio_interrupt(unsigned int irq, void *dev_id, struct pt_regs *regs)
{  
  struct planck_device *device = dev_id;
  unsigned long flags;
  int ret;

  printk(KERN_DEBUG "planck: gpio interrupted with irq %d\n", irq); 

  if(device == NULL)
    return (irq_handler_t)IRQ_HANDLED;

  spin_lock_irqsave(&device->irq_lock, flags);

  ret = planck_queue_i2c_work(device, flags);
  printk(KERN_DEBUG "planck: queued i2c work returned %d", ret);

  return (irq_handler_t)IRQ_HANDLED;
}

module_init(planck_init);
module_exit(planck_exit);
