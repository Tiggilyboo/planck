#include "planck.h"

static uint16_t planck_read_i2c_state(struct planck_device *device, int reg)
{
  uint8_t a = i2c_read_byte(device->i2c, reg); 
  uint16_t ba = i2c_read_byte(device->i2c, reg+1);
  ba <<= 8;
  ba |= a;
  
  return ba;
}

static void planck_process_input(struct planck_device *dev, int x, int y, int layer, int pressed)
{
  struct hidg_func_node* hfn;
  struct f_hidg* hidg;
  struct file* hidg0;
  unsigned short keycode;
  unsigned short coord;
  int status;

  if(!dev) 
    return;

  coord = (layer * (MAX_X * MAX_Y)) + (y * MAX_X) + x;

  // Switch input modes from internal to external USB mode
  if(layer == 3 && planck_keycodes[coord] == KEY_CONNECT && pressed == 0)
  {
    if(dev->internal)
      printk(KERN_DEBUG "planck: switching input mode to usb hid mode!");
    else 
      printk(KERN_DEBUG "planck: switching input mode to internal mode!");

    dev->internal = !dev->internal;

    // Reset modifiers when switching modes
    planck_hid_report[0] = 0x00;
    return;
  }

  // Internal input device
  if(dev->internal){
    keycode = planck_keycodes[coord];
    if(keycode){
      printk(KERN_DEBUG "planck: input_event for keycode %d in state: %d.\n", keycode, pressed);
      input_event(dev->input, EV_KEY, keycode, pressed);
    }
    return;
  }

  // External HID input
  hfn = list_first_entry_or_null(&hidg_func_list, struct hidg_func_node, node);
  if(hfn == NULL){
    printk(KERN_ERR "planck: unable to process keystroke, hidg_func_node could not be found in hidg_func_list!");
    return;
  }
  if(hfn->f == NULL){
    printk(KERN_ERR "planck: hidg_func_node contained null usb_function!");
    return;
  }
  hidg = func_to_hidg(hfn->f);
  if(hidg == NULL){
    printk(KERN_ERR "planck: usb_function was not a f_hidg struct?!");
    return;
  }

  mutex_unlock(&hidg->lock);

  // Assemble HID request
  keycode = planck_hid_keymap[coord];

  // Handle all modifiers are 0xE[0-7]
  if(keycode >= 0xE0 && keycode <= 0xE7)
  {
    if(pressed)
      planck_hid_report[0] |= (1 << (keycode - 0xE0));
    else
      planck_hid_report[0] &= ~(1 << (keycode - 0xE0));
  }
  // Are we in lower? (Hold shift)
  if(layer == 1){
    if(pressed)
      planck_hid_report[0] |= 0x02;
    else
      planck_hid_report[0] &= ~0x02;
  }

  // Add any new keycode in the report
  if(pressed)
    planck_hid_report[2] = (char)keycode;
  else
    planck_hid_report[2] = 0;

  printk(KERN_INFO "planck: trying to send usb hid report: %x, %x, %x, %x, %x, %x, %x, %x", 
      planck_hid_report[0],planck_hid_report[1],planck_hid_report[2],planck_hid_report[3],
      planck_hid_report[4],planck_hid_report[5],planck_hid_report[6],planck_hid_report[7]);

  hidg0 = file_open("/dev/hidg0", O_RDWR | O_DSYNC, 570);
  if(!hidg0){
    printk(KERN_ERR "planck: unable to open /dev/hidg0\n");
    goto finished;
  }
  status = file_write(hidg0, 0, planck_hid_report, 8);
  if(status < 0){
    printk(KERN_ERR "planck: unable to write to hidg0: %d\n", status);
    goto finished;
  }
  status = vfs_fsync(hidg0, 0);
  if(!status){
    printk(KERN_ERR "planck: unable to fsync hidg0: %d\n", status);
    goto finished;
  }

finished:
  printk(KERN_DEBUG "planck: hid report for keycode %d in state: %d.\n", keycode, pressed);
  mutex_lock(&hidg->lock);
}

static void planck_layer_internal_modifiers(struct planck_device* dev, uint16_t curr, int layer)
{
  // HID handled directly in HID input processing
  if(!dev->internal)
    return;
  
  int clower = (curr & (1 << (3 + MAX_X))) && (curr & (1 << 4));
  int cupper = (curr & (1 << (3 + MAX_X))) && (curr & (1 << 7));

  if(clower){
    if(layer != 1) {
      // ie. last lower == false
      printk(KERN_DEBUG "planck: input_event for keycode %d in state: %d.\n", KEY_LEFTSHIFT, 1);
      input_event(dev->input, EV_KEY, KEY_LEFTSHIFT, 1);
    }
  } else if(!cupper && layer == 1) {
    // ie. last lower == true, curr lower == false
    printk(KERN_DEBUG "planck: input_event for keycode %d in state: %d.\n", KEY_LEFTSHIFT, 0);
    input_event(dev->input, EV_KEY, KEY_LEFTSHIFT, 0);
  }
}

static int planck_layer_handler(struct planck_device* dev, uint16_t prev, uint16_t curr)
{
  int layer = 0;
  int lower = (prev & (1 << (3 + MAX_X))) && (prev & (1 << 4));
  int upper = (prev & (1 << (3 + MAX_X))) && (prev & (1 << 7));

  if(upper && lower)
    layer = 3; 
  else if (upper)
    layer = 2;
  else if (lower)
    layer = 1;

  planck_layer_internal_modifiers(dev, curr, layer);

  return layer;
}

static int planck_queue_row_work(struct planck_device* device)
{
  int y;
  struct planck_row_work* rw = (struct planck_row_work*)kmalloc(sizeof(struct planck_row_work), GFP_KERNEL);
  INIT_DELAYED_WORK((struct delayed_work*)rw, planck_row_work_handler);

  rw->layer = 0;
  rw->device = device;
  for(y = 0; y < MAX_Y; y++){
    rw->last_state[y] = 0;
    rw->last_state[y] |= (1 << (y + MAX_X));
  }
  device->row_worker = rw;

  // Queue the initial work
  return queue_delayed_work(device->write_wq, &rw->work, GPIO_JIFFY_DELAY);
}

static inline int planck_row_state(uint16_t state, int row) {
  int value = 0;

  // Determine the last 4 columns state and the new row state
  value |= (state >> 8);
  // Set all rows HIGH
  value |= (0xf << 4);
  // Set active row LOW
  value &= ~(1 << (4 + row)); 

  return value;
}

static void planck_row_work_handler(struct work_struct* w)
{
  uint16_t state, last_state;
  int value, x, y, layer;
  struct delayed_work *dw = (struct delayed_work*)w;
  struct planck_row_work *work = container_of(dw, struct planck_row_work, work);

  layer = work->layer;

  for(y = 0; y < MAX_Y; y++){
    // Write i2c rows YYYY XXXX
    last_state = work->last_state[y];

    // Get our intended row aka y state to write to i2c
    value = planck_row_state(0xffff, y);
    i2c_write_byte(work->device->i2c, MCP23017_GPIOB, value); 

    // Check the new state now that the row is active
    state = ~planck_read_i2c_state(work->device, MCP23017_GPIOA); 

    // Only set the layer where y == 3, as both layer buttons reside there
    // Otherwise, use the stored layer state in the worker
    if(y == 3){
      layer = planck_layer_handler(work->device, last_state, state);
      work->layer = layer;
    }

    for(x = 0; x < MAX_X; x++){
      int lastX = (last_state & (1 << x));
      int curX = (state & (1 << x));

      if(lastX && !curX){
        planck_process_input(work->device, x, y, layer, 0);
        work->last_state[y] = state; 
      }
      if(curX && !lastX){
        planck_process_input(work->device, x, y, layer, 1);
        work->last_state[y] = state; 
      }
    }
  }
  // Make sure to sync the internal report!
  if(work->device->internal)
    input_sync(work->device->input);

  // We then queue the next row! Yes, this means forever.
  queue_delayed_work(work->device->write_wq, (struct delayed_work*)w, GPIO_JIFFY_DELAY);
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
 
  input->name = "planck-internal-kbd";
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

  // Setup all columns to inputs
  ret = i2c_write_byte(client, MCP23017_IODIRA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_IODIRB, 0x0f);
  if(ret != 0) goto i2c_err;

  // Setup pullups
  ret = i2c_write_byte(client, MCP23017_GPPUA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_GPPUB, 0x0f);
  if(ret != 0) goto i2c_err;
 
  // Setup interrupt mirrorring disable SEQOP, polarity HIGH 
  ret = i2c_write_byte(client, MCP23017_IOCONA, 0x62);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_IOCONB, 0x62);
  if(ret != 0) goto i2c_err;

  // Setup interrupt on change
  ret = i2c_write_byte(client, MCP23017_GPINTENA, 0x00); //0xff);
  if(ret != 0) goto i2c_err;
  // Only fire interrupt for columns, not rows
  ret = i2c_write_byte(client, MCP23017_GPINTENB, 0x00); //0x0f);
  if(ret != 0) goto i2c_err;

  // Set initial GPIO state
  ret = i2c_write_byte(client, MCP23017_GPIOA, 0xff);
  if(ret != 0) goto i2c_err;
  ret = i2c_write_byte(client, MCP23017_GPIOB, 0xff);
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
  device->internal = PLANCK_BOOT_INTERNAL;

  ret = planck_init_internal_input(device);
  if(ret != 0){
    printk(KERN_ERR "planck: unable to initialise input device, returned %d\n", ret);
    goto badstate;
  }
  
  device->write_wq = create_singlethread_workqueue("planck_row_workqueue");
  if(device->write_wq == NULL)
    goto free_input;

  // Device setup complete, set the client's device data
  i2c_set_clientdata(client, device);
  
  // Read the initial state (Makes the MCP23017 happy)
  planck_read_i2c_state(device, MCP23017_INTCAPA);
  
  // Queue the initial row workers! (They maintain their own lifecycle / queue themselves after initial queue)
  planck_queue_row_work(device);

  goto probe_ok;

free_input:
  input_unregister_device(device->input);
badstate:
  return -ENODEV;

probe_ok:
  printk(KERN_DEBUG "planck: probed.\n");
  return 0;
}

static int planck_i2c_remove(struct i2c_client *client) {
  struct planck_device *dev = i2c_get_clientdata(client);
 
  printk(KERN_DEBUG "planck: flushing delayed work and destroying write_wq");
  if(dev->row_worker){
    cancel_delayed_work((struct delayed_work*)dev->row_worker);
  }
  flush_workqueue(dev->write_wq);
  destroy_workqueue(dev->write_wq);
  printk(KERN_DEBUG "planck: input_unregister_device");
  input_unregister_device(dev->input);
  printk(KERN_DEBUG "planck: freeing device memory");
  kfree(dev);

  printk(KERN_DEBUG "planck: removed\n"); 
  return 0;  
}

static struct i2c_driver planck_i2c_driver = {
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

  printk(KERN_DEBUG "planck: initialising hid...");
  res = planck_init_hid();
  if (res != 0) {
    printk(KERN_DEBUG "planck: hid driver initialisation failed.\n");
    return res;
  }

  printk(KERN_DEBUG "planck: initialising i2c...");
  res = i2c_add_driver(&planck_i2c_driver);
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
  printk(KERN_DEBUG "planck: unregistering platform device planck_hid");
  platform_device_unregister(&planck_hid);

  printk(KERN_DEBUG "planck: deleting i2c driver...");
  i2c_del_driver(&planck_i2c_driver);

  printk("planck: exited");
}

module_init(planck_init);
module_exit(planck_exit);
