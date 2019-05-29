#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>

#include "planck.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Willshire");
MODULE_DESCRIPTION("Planck i2c keyboard driver for mcp23017");
MODULE_VERSION("0.1");

struct planck_device {
  struct i2c_client *client;
};

static int planck_i2c_probe(
    struct i2c_client *client, 
    const struct i2c_device_id *id) 
{
  struct planck_device *dev;

  if(i2c_check_functionality(client->adapter,
    I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)){
    printk(KERN_ERR "%s: needed i2c functionality is not supported\n", __func__);
    return -ENODEV;
  }

  dev = kzalloc(sizeof(struct planck_device), GFP_KERNEL);
  if(dev == NULL){
    printk(KERN_ERR "%s: no memory", __func__);
    return -ENOMEM;
  }

  dev->client = client;
  i2c_set_clientdata(client, dev);

  return 0;
}

static int planck_i2c_remove(struct i2c_client *client)
{
  struct planck_client *dev = i2c_get_clientdata(client);

  kfree(dev);

  return 0;
}

static const struct i2c_device_id planck_i2c_id[] = {
  { "planck_i2c_client", 0x20 },
  { }
};
static struct i2c_driver planck_i2c_driver = {
  .probe = planck_i2c_probe,
  .remove = planck_i2c_remove,
  .id_table = planck_i2c_id,
  .driver = {
    .name = "planck_i2c_client",
    .owner = THIS_MODULE,
  },
};

static int __init planck_init(void)
{
  printk("Initialising planck kernel driver...\n");

  return i2c_add_driver(&planck_i2c_driver);
}

static void __exit planck_exit(void)
{
  printk("Exiting planck kernel driver...\n");
  i2c_del_driver(&planck_i2c_driver);
}

module_init(planck_init);
module_exit(planck_exit);
