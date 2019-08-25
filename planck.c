#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>

#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14

#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Willshire");
MODULE_DESCRIPTION("Planck i2c keyboard driver for mcp23017");
MODULE_VERSION("0.1");
MODULE_INFO(intree, "Y");

struct planck_device {
  struct i2c_client *client;
};

static struct of_device_id planck_ids[] = {{.compatible = "planck"},{}};
static const struct i2c_device_id planck_id[] = { {"planck", 0}, {}};

static int i2c_read_byte(struct i2c_client *client, unsigned char command) {
  int ret = -1;
  ret = i2c_smbus_read_byte_data(client, command);

  printk(KERN_DEBUG "planck: reading reg %d returned value %d\n", command, ret);
  return ret;
}

static int i2c_write_byte(struct i2c_client *client, int reg, int value) {
  int ret = -1;
  ret = i2c_smbus_write_byte_data(client, reg, value);
  printk(KERN_DEBUG "planck: writing reg %d with value %d", reg, value);
  return ret;
}

static int planck_probe(struct i2c_client *client, const struct i2c_device_id *id) {
  printk("planck: probe!!!");

  struct planck_device *dev;
  int ret = -1;
  if(!i2c_check_functionality(client->adapter, 
        I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
        I2C_FUNC_SMBUS_I2C_BLOCK))
  {
    printk(KERN_ERR "planck: %s needed i2c functionality is not supported\n", __func__);
    return -ENODEV;
  }
  dev = kzalloc(sizeof(struct planck_device), GFP_KERNEL);
  if(dev == NULL)
  {
    printk(KERN_ERR "planck: %s: no memory\n", __func__);
    return -ENOMEM;
  }
  printk(KERN_DEBUG "planck: probed\n");
  dev->client = client;

  // Setup all inputs
  i2c_write_byte(client, MCP23017_IODIRA, 0xff);
  i2c_write_byte(client, MCP23017_IODIRB, 0xff);
  // Setup pullups
  i2c_write_byte(client, MCP23017_GPPUA, 0xff);
  i2c_write_byte(client, MCP23017_GPPUB, 0xff);

  return 0;
}

static int planck_remove(struct i2c_client *client) {
  struct planck_device *dev = i2c_get_clientdata(client);
  kfree(dev);

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
  printk("planck: init");
  int res;
  if((res = i2c_add_driver(&planck_driver))) {
    printk(KERN_DEBUG "planck: driver registration failed, module not inserted.\n");
    return res;
  }

  printk(KERN_DEBUG "planck: inited\n");
  return 0;
}
static void __exit planck_exit(void) {
  printk("planck: exited...");
  i2c_del_driver(&planck_driver);
}

module_init(planck_init);
module_exit(planck_exit);
