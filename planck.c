#include "planck.h"

static int i2c_read_byte(struct i2c_client *client, unsigned char command) {
  return i2c_smbus_read_byte_data(client, command);
}
static int i2c_write_byte(struct i2c_client *client, int reg, int value) {
  return i2c_smbus_write_byte_data(client, reg, value);
}
static uint16_t planck_read_i2c_state(struct planck_device *device, int reg)
{
  uint8_t a = i2c_read_byte(device->i2c, reg); 
  uint16_t ba = i2c_read_byte(device->i2c, reg+1);
  ba <<= 8;
  ba |= a;
  
  printk(KERN_DEBUG "planck: state is now: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"\n", BYTE_TO_BIN(ba>>8), BYTE_TO_BIN(ba));

  return ba;
}
static void planck_process_input(struct planck_device *dev, unsigned short keycode, int layer, int pressed)
{
  if(!dev)
    return;

  // Switch input modes from internal to external USB mode
  if(layer == 3 && keycode == KEY_CONNECT && pressed == 0)
  {
    if(dev->internal)
      printk(KERN_DEBUG "planck: switching input mode to usb hid mode!");
    else 
      printk(KERN_DEBUG "planck: switching input mode to internal mode!");

    dev->internal = !dev->internal;
    return;
  }

  if(dev->internal){
    input_event(dev->input, EV_KEY, keycode, pressed);
  } else {
    //dev->hid 
  }
}
static int planck_layer_handler(struct planck_device* dev, uint16_t prev, uint16_t curr)
{
  // y = MAX_Y
  // lower: x == 5 (4 when 0 indexed)
  int lower = (prev & (1 << MAX_Y)) && (prev & (1 << (MAX_Y + 4)));
  // upper: x == 8 (7 when 0 indexed)
  int upper = (prev & (1 << MAX_Y)) && (prev & (1 << (MAX_Y + 7)));

  printk(KERN_DEBUG "planck: upper = %d, lower = %d", upper, lower); 

  if(upper && lower)
    return 3;
  else if (upper)
    return 2;
  else if (lower)
    return 1;
  
  return 0;
}

static void planck_work_handler(struct work_struct *w)
{
  int x, y;
  uint16_t state, last_state;
  unsigned short keycode;
  struct planck_i2c_work *work = container_of(w, struct planck_i2c_work, work);
  struct planck_device *dev = work->device;

  if(dev == NULL){
    printk(KERN_ERR "planck: work_struct has no device!");
    goto finish;
  }

  last_state = dev->state;
  dev->state = ~planck_read_i2c_state(dev, MCP23017_INTCAPA);
  state = dev->state;

  // Nothing changed
  if(state == last_state || ~last_state == 0)
    goto finish;

  int layer = planck_layer_handler(dev, last_state, state);
  int layerOffset = layer * (MAX_X * MAX_Y);
  for(y = 0; y < MAX_Y; y++) {
    int currY = (state & (1 << y));
    int lastY = (last_state & (1 << y));

    for(x = 0; x < MAX_X; x++) {
      int currX = (state & (1 << (x + MAX_Y)));
      int lastX = (last_state & (1 << (x + MAX_Y)));

      // Currently pressed, was not pressed before (Pressed)
      if(!!currX && !!currY && (!lastY || !lastX)){
        keycode = planck_keycodes[layerOffset + (y * MAX_X) + x];
        printk(KERN_DEBUG "planck: pressed (%d, %d), keymap = %d, state = %d, last = %d", x, y, keycode, state, last_state);
        planck_process_input(dev, keycode, layer, 1);
      } 
      // No longer pressed, pressed before (Released)
      else if((!currX || !currY) && (!!lastY && !!lastX)){
        keycode = planck_keycodes[layerOffset + (y * MAX_X) + x];
        printk(KERN_DEBUG "planck: released (%d, %d), keymap = %d, state = %d, last = %d", x, y, keycode, state, last_state);
        planck_process_input(dev, keycode, layer, 0);
      }
    }
  }
  if(dev->internal){
    input_sync(dev->input);
  }

  // cleanup
finish:
  printk(KERN_DEBUG "planck: cleaning up work_handler");
  kfree((void*)w);
}

static int planck_queue_i2c_work(struct planck_device *device)
{
  struct planck_i2c_work* work = (struct planck_i2c_work*)kmalloc(sizeof(struct planck_i2c_work), GFP_KERNEL);
  if(!work){
    return -ENOMEM;
  }

  INIT_WORK((struct work_struct*)work, planck_work_handler);
  work->device = device;

  return queue_work(device->read_wq, (struct work_struct*)work);
}

static int planck_init_gpio(struct planck_device *device)
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
  
  return 0;
}
static int planck_init_hid(void)
{
  int ret;

  ret = platform_device_register(&planck_hid);
  if (ret < 0){
    printk(KERN_ERR "planck: unable to add planck_hid device: %d\n", ret);
    return ret;
  }

  ret = platform_driver_probe(&hidg_plat_driver, planck_hid_plat_driver_probe);
  if (ret < 0){
    printk(KERN_ERR "planck: unable to probe platform driver: %d\n", ret);
    platform_device_unregister(&planck_hid);
    return ret;
  }

  ret = usb_composite_probe(&planck_hidg_driver);
  if (ret < 0){
    printk(KERN_ERR "planck: unable to probe usb composite driver: %d\n", ret);
    platform_driver_unregister(&hidg_plat_driver);
    platform_device_unregister(&planck_hid);
    return ret;
  } 
  
  return ret;
}

static int planck_init_internal_input(struct planck_device* device)
{
  struct input_dev* input;
  int ret;
  const int num_keycodes = ARRAY_SIZE(planck_keycodes);

  printk(KERN_DEBUG "planck: initialising internal input...");
  input = input_allocate_device();
  if(input == NULL)
    return -ENOMEM;
  
  input->evbit[0] = BIT_MASK(EV_KEY);
  input->keycode = planck_keycodes;
  input->keycodesize = sizeof(unsigned short); 
  input->keycodemax = num_keycodes;

  for(ret = 0; ret < num_keycodes; ret++)
  {
    if(planck_keycodes[ret] != KEY_RESERVED)
      set_bit(planck_keycodes[ret], input->keybit);
  }

  ret = input_register_device(input);
  if(ret != 0)
  {
    printk(KERN_ERR "planck: unable to register input device, register returned %d\n", ret);
    goto input_err;
  }

  printk(KERN_DEBUG "planck: initialised input device with %d keycodes", num_keycodes);
  device->input = input;

  return ret;

input_err:
  input_unregister_device(input);
  return -ENODEV;
}

static int planck_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) 
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
  device->i2c = client;

  ret = planck_init_internal_input(device);
  if(ret != 0){
    printk(KERN_ERR "planck: unable to initialise input device, returned %d\n", ret);
    goto free_input;
  }
  
  device->read_wq = create_workqueue("planck_workqueue");
  if(device->read_wq == NULL)
    goto free_queue;

  ret = planck_init_gpio(device);
  if(ret != 0){
    printk(KERN_ERR "planck: unable to configure gpio, returned: %d\n", ret);
    goto free_gpio;
  }

  // Read the initial state
  spin_lock_init(&device->irq_lock); 
  spin_lock_irqsave(&device->irq_lock, flags);
  i2c_set_clientdata(client, device);
  device->state = planck_read_i2c_state(device, MCP23017_INTCAPA);
  spin_unlock_irqrestore(&device->irq_lock, flags);

  goto probe_ok;

free_gpio:
free_queue:
  destroy_workqueue(device->read_wq);
free_input:
  input_unregister_device(device->input);
  return -ENODEV;

probe_ok:
  printk(KERN_DEBUG "planck: probed.\n");
  return 0;
}

static int planck_i2c_remove(struct i2c_client *client) {
  struct planck_device *dev = i2c_get_clientdata(client);
 
  printk(KERN_DEBUG "planck: disable_irq");
  disable_irq(dev->irq_number);
  printk(KERN_DEBUG "planck: free_irq");
  free_irq(dev->irq_number, dev);
  printk(KERN_DEBUG "planck: gpio_free");
  gpio_free(GPIO_INTERRUPT);
  printk(KERN_DEBUG "planck: flush_workqueue");
  flush_workqueue(dev->read_wq);
  printk(KERN_DEBUG "planck: destroy_workqueue");
  destroy_workqueue(dev->read_wq);
  printk(KERN_DEBUG "planck: input_unregister_device");
  input_unregister_device(dev->input);
  printk(KERN_DEBUG "planck: freeing device memory");
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
  .probe = planck_i2c_probe,
  .remove = planck_i2c_remove,
  .id_table = planck_id,
};

static int __init planck_init(void) {
  int res;
  printk(KERN_DEBUG "planck: initialising i2c...");

  printk(KERN_DEBUG "planck: initialising hid...");

  res = planck_init_hid();
  if (res != 0) {
    printk(KERN_DEBUG "planck: hid driver initialisation failed.\n");
    return res;
  }

  res = i2c_add_driver(&planck_driver);
  if(res != 0) {
    printk(KERN_DEBUG "planck: i2c driver registration failed, module not inserted.\n");
    return res;
  }

  printk(KERN_DEBUG "planck: inited\n");
  return res;
}

static void __exit planck_exit(void) 
{
  printk(KERN_DEBUG "planck: exiting...");

  printk(KERN_DEBUG "planck: unregistering hidg plat_driver");
  platform_driver_unregister(&hidg_plat_driver);
  printk(KERN_DEBUG "planck: unregistering usb composite driver");
  usb_composite_unregister(&planck_hidg_driver);

  printk(KERN_DEBUG "planck: deleting i2c driver...");
  i2c_del_driver(&planck_driver);


  printk("planck: exited");
}

static irq_handler_t planck_gpio_interrupt(unsigned int irq, void *dev_id, struct pt_regs *regs)
{  
  struct planck_device *device = dev_id;
  unsigned long flags;
  int ret;

  if(device == NULL)
    return (irq_handler_t)IRQ_HANDLED;

  spin_lock_irqsave(&device->irq_lock, flags);
  ret = planck_queue_i2c_work(device);
  spin_unlock_irqrestore(&device->irq_lock, flags);

  return (irq_handler_t)IRQ_HANDLED;
}

module_init(planck_init);
module_exit(planck_exit);
