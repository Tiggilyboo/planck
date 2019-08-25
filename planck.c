#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>

// MCP23017 Operating address
#define I2C_BASE      0xFF110000

#define MPC23017_GPIOA_MODE		      0x00
#define MPC23017_GPIOB_MODE		      0x01
#define MPC23017_GPIOA_PULLUPS_MODE	0x0c
#define MPC23017_GPIOB_PULLUPS_MODE	0x0d
#define MPC23017_GPIOA_READ         0x12
#define MPC23017_GPIOB_READ         0x13

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Willshire");
MODULE_DESCRIPTION("Planck i2c keyboard driver for mcp23017");
MODULE_VERSION("0.1");
MODULE_INFO(intree, "Y");

static struct of_device_id planck_ids[] = {{.compatible = "planck"},{}};
static const struct i2c_device_id planck_id[] = { {"planck", 0}, {}};

static int planck_probe(struct i2c_client *client, const struct i2c_device_id *id) {
  printk("planck: probe!!!");
  return 0;
}

static int planck_remove(struct i2c_client *client) {
  printk("planck: remove called"); 
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
  i2c_add_driver(&planck_driver);
  return 0;
}
static void __exit planck_exit(void) {
  printk("planck: exited...");
  i2c_del_driver(&planck_driver);
}

module_init(planck_init);
module_exit(planck_exit);
