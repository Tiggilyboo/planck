#include "planck.h"
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

static uint16_t planck_read_i2c_state(struct planck_device *device, int reg)
{
  uint8_t a = i2c_read_byte(device->i2c, reg); 
  uint16_t ba = i2c_read_byte(device->i2c, reg+1);
  ba <<= 8;
  ba |= a;
  
  printk(KERN_DEBUG "planck: state is now: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"\n", BYTE_TO_BIN(ba>>8), BYTE_TO_BIN(ba));

  return ba;
}
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
mm_segment_t oldfs;
int ret;

oldfs = get_fs();
set_fs(get_ds());

ret = vfs_write(file, data, size, &offset);

set_fs(oldfs);
return ret;
}
static void planck_process_input(struct planck_device *dev, unsigned short coord, int layer, int pressed)
{
  struct hidg_func_node* hfn;
  struct f_hidg* hidg;
  unsigned short keycode;
  int status;

  printk(KERN_INFO "planck: planck_process_input invoked, coord: %d, layer: %d, pressed: %d\n", coord, layer, pressed);

  if(!dev) return;

  // Switch input modes from internal to external USB mode
  if(layer == 3 && coord == KEY_CONNECT && pressed == 0)
  {
    if(dev->internal)
      printk(KERN_DEBUG "planck: switching input mode to usb hid mode!");
    else 
      printk(KERN_DEBUG "planck: switching input mode to internal mode!");

    dev->internal = !dev->internal;
    return;
  }

  if(dev->internal){
    keycode = planck_keycodes[coord];
    printk(KERN_DEBUG "planck: input_event for keycode %d in state: %d.\n", keycode, pressed);
    input_event(dev->input, EV_KEY, keycode, pressed);
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

  // Accumulate modifiers
  if(keycode == 0x01 || keycode == 0x10 
      || keycode == 0x02 || keycode == 0x20 
      || keycode == 0x04 || keycode == 0x40 
      || keycode == 0x08 || keycode == 0x80){
    
    if(!(planck_hid_report[0] & keycode)) {
      planck_hid_report[0] = planck_hid_report[0] | keycode;
    }

    // Don't send the mods yet, only with other inputs values!
    printk(KERN_DEBUG "planck: not sending modifier, no other key pressed yet...");
    return;
  }

  // Add any new keycode in the report
  planck_hid_report[2] = (char)keycode;

  printk(KERN_INFO "planck: trying to send usb hid report: %x, %x, %x, %x, %x, %x, %x, %x", 
      planck_hid_report[0],
      planck_hid_report[1],
      planck_hid_report[2],
      planck_hid_report[3],
      planck_hid_report[4],
      planck_hid_report[5],
      planck_hid_report[6],
      planck_hid_report[7]);

  struct file* hidg0 = file_open("/dev/hidg0", O_RDWR | O_DSYNC, 570);
  if(!hidg0){
    printk(KERN_ERR "planck: unable to open /dev/hidg0\n");
    return;
  }
  status = file_write(hidg0, 0, planck_hid_report, 8);
  if(status < 0){
    printk(KERN_ERR "planck: unable to write to hidg0: %d\n", status);
    return;
  }
  status = vfs_fsync(hidg0, 0);
  if(!status){
    printk(KERN_ERR "planck: unable to fsync hidg0: %d\n", status);
    return;
  }

  printk(KERN_DEBUG "planck: wrote planck_hid_report to /dev/hidg0\n");
}

static int planck_layer_handler(struct planck_device* dev, uint16_t prev, uint16_t curr)
{
  // y = MAX_Y
  // lower: x == 5 (4 when 0 indexed)
  int lower = (prev & (1 << MAX_Y)) && (prev & (1 << (MAX_Y + 4)));
  // upper: x == 8 (7 when 0 indexed)
  int upper = (prev & (1 << MAX_Y)) && (prev & (1 << (MAX_Y + 7)));

  //printk(KERN_DEBUG "planck: upper = %d, lower = %d", upper, lower); 
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
  int x, y, layer, layerOffset;
  uint16_t state, last_state;
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

  layer = planck_layer_handler(dev, last_state, state);
  layerOffset = layer * (MAX_X * MAX_Y);
  for(y = 0; y < MAX_Y; y++) {
    int currY = (state & (1 << y));
    int lastY = (last_state & (1 << y));

    for(x = 0; x < MAX_X; x++) {
      int currX = (state & (1 << (x + MAX_Y)));
      int lastX = (last_state & (1 << (x + MAX_Y)));

      // Currently pressed, was not pressed before (Pressed)
      if(!!currX && !!currY && (!lastY || !lastX)){
        planck_process_input(dev, layerOffset + (y * MAX_X) + x, layer, 1);
      } 
      // No longer pressed, pressed before (Released)
      else if((!currX || !currY) && (!!lastY && !!lastX)){
        planck_process_input(dev, layerOffset + (y * MAX_X) + x, layer, 0);
      }
    }
  }
  if(dev->internal){
    input_sync(dev->input);
  }

  // cleanup
finish:
  //printk(KERN_DEBUG "planck: cleaning up work_handler");
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

  printk(KERN_DEBUG "planck: deleting i2c driver...");
  i2c_del_driver(&planck_i2c_driver);


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
