#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include <linux/usb/g_hid.h>

#include "u_hid.h"
#include "u_f.h"

#define DRIVER_DESC		    "HID Gadget"
#define DRIVER_VERSION		"2019/10/16"
#define HIDG_VENDOR_NUM		0x0525	/* XXX NetChip */
#define HIDG_PRODUCT_NUM	0xa4ac	/* Linux-USB HID gadget */

struct hidg_func_node {
  struct usb_function_instance *fi;
  struct usb_function *f;
  struct list_head node;
  struct hidg_func_descriptor *func;
};
static LIST_HEAD(hidg_func_list);

USB_GADGET_COMPOSITE_OPTIONS();

static struct usb_device_descriptor device_desc = {
  .bLength = sizeof device_desc,
  .bDescriptorType = USB_DT_DEVICE,
  .bDeviceClass = USB_CLASS_PER_INTERFACE,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .idVendor = cpu_to_le16(HIDG_VENDOR_NUM),
  .idProduct = cpu_to_le16(HIDG_PRODUCT_NUM),
  .bNumConfigurations = 1,
};

static const struct usb_descriptor_header *otg_desc[2];

static struct usb_string strings_dev[] = {
  [USB_GADGET_MANUFACTURER_IDX].s = "",
  [USB_GADGET_PRODUCT_IDX].s = DRIVER_DESC,
  [USB_GADGET_SERIAL_IDX].s = "",
  { }
};

static struct usb_gadget_strings stringtab_dev = {
  .language = 0x0409, /* en-us */ 
  .strings = strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = { &stringtab_dev, NULL };

static int planck_config_hid(struct usb_configuration *c)
{
  struct hidg_func_node *e, *n;
  int status = 0;

  printk(KERN_DEBUG "planck_config_hid");

  if(gadget_is_otg(c->cdev->gadget)){
    c->descriptors = otg_desc;
    c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
  }

  list_for_each_entry(e, &hidg_func_list, node) {
    e->f = usb_get_function(e->fi);
    if(IS_ERR(e->f))
      goto put;
    status = usb_add_function(c, e->f);
    if(status < 0) {
      usb_put_function(e->f);
      goto put;
    }
  }

  return 0;

put:
  list_for_each_entry(n, &hidg_func_list, node) {
    if(n == e)
      break;
    usb_remove_function(c, n->f);
    usb_put_function(n->f);
  }
  return status;
}

static struct usb_configuration config_driver = {
  .label = DRIVER_DESC,
  .bConfigurationValue = 1,
  .bmAttributes = USB_CONFIG_ATT_SELFPOWER,
};

static int planck_hid_bind(struct usb_composite_dev *cdev){
  struct usb_gadget *gadget = cdev->gadget;
  struct list_head *tmp;
  struct hidg_func_node *n, *m;
  struct f_hid_opts *hid_opts;
  int status, funcs = 0;

  printk(KERN_DEBUG "planck: hid bind.");

  list_for_each(tmp, &hidg_func_list)
    funcs++;

  if(!funcs)
    return -ENODEV;

  list_for_each_entry(n, &hidg_func_list, node) {
    n->fi = usb_get_function_instance("hid");
    if(IS_ERR(n->fi)) {
      status = PTR_ERR(n->fi);
      goto put;
    }
    hid_opts = container_of(n->fi, struct f_hid_opts, func_inst);
    hid_opts->subclass = n->func->subclass;
    hid_opts->protocol = n->func->protocol;
    hid_opts->report_length = n->func->report_length;
    hid_opts->report_desc_length = n->func->report_desc_length;
    hid_opts->report_desc = n->func->report_desc;
  }

  status = usb_string_ids_tab(cdev, strings_dev);
  if(status < 0)
    goto put;
  device_desc.iManufacturer = strings_dev[USB_GADGET_MANUFACTURER_IDX].id;
  device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;

  if(gadget_is_otg(gadget) && !otg_desc[0]) {
    struct usb_descriptor_header *usb_desc = usb_otg_descriptor_alloc(gadget);
    if(!usb_desc)
      goto put;

    usb_otg_descriptor_init(gadget, usb_desc);
    otg_desc[0] = usb_desc;
    otg_desc[1] = NULL;
  }

  status = usb_add_config(cdev, &config_driver, planck_config_hid);
  if(status < 0)
    goto free_otg_desc;

  usb_composite_overwrite_options(cdev, &coverwrite);
  printk(KERN_DEBUG DRIVER_DESC ", version " DRIVER_VERSION "\n");

  return 0;

free_otg_desc:
  kfree(otg_desc[0]);
  otg_desc[0] = NULL;
put:
  list_for_each_entry(m, &hidg_func_list, node) {
    if(m == n)
      break;
    usb_put_function_instance(m->fi);
  }
  return status;
}

static int planck_hid_unbind(struct usb_composite_dev *cdev){
  struct hidg_func_node *n;
  
  printk(KERN_DEBUG "planck: hid unbind.");

  list_for_each_entry(n , &hidg_func_list, node) {
    usb_put_function(n->f);
    usb_put_function_instance(n->fi);
  }

  kfree(otg_desc[0]);
  otg_desc[0] = NULL;

  return 0;
}

static int planck_hid_plat_driver_probe(struct platform_device *pdev){
  struct hidg_func_descriptor *func; 
  struct hidg_func_node *entry;

  printk(KERN_DEBUG "planck: hid probe!");

  func = dev_get_platdata(&pdev->dev);

  if(!func){
    printk(KERN_ERR "planck: Platform data missing\n");
    return -ENODEV;
  }

  entry = kzalloc(sizeof(*entry), GFP_KERNEL);
  if(!entry)
    return -ENOMEM;

  entry->func = func;
  list_add_tail(&entry->node, &hidg_func_list);

  return 0;
}

static int planck_hid_plat_driver_remove(struct platform_device *pdev){
  struct hidg_func_node *e, *n;

  printk(KERN_DEBUG "planck: planck_hid_plat_driver_remove");

  list_for_each_entry_safe(e, n, &hidg_func_list, node) {
    list_del(&e->node);
    kfree(e);
  }

  return 0;
}
static void planck_platform_device_release(struct device *dev){
  printk(KERN_DEBUG "planck: planck_platform_device_release\n");
}

static struct usb_composite_driver planck_hidg_driver = {
  .name = "planck_hidg_driver",
  .dev = &device_desc,
  .strings = dev_strings,
  .max_speed = USB_SPEED_HIGH,
  .bind = planck_hid_bind,
  .unbind = planck_hid_unbind,
};

static struct platform_driver hidg_plat_driver = {
  .remove = planck_hid_plat_driver_remove,
  .driver = {
    .owner = THIS_MODULE,
    .name = "planck_hidg",
  },
};

static struct hidg_func_descriptor planck_hid_data = {
  .subclass               = 0, /* No subclass */
  .protocol               = 1, /* Keyboard */
  .report_length          = 8,
  .report_desc_length     = 63,
  .report_desc            = {
    0x05, 0x01,     /* USAGE_PAGE (Generic Desktop)           */
    0x09, 0x06,     /* USAGE (Keyboard)                       */
    0xa1, 0x01,     /* COLLECTION (Application)               */
    0x05, 0x07,     /*   USAGE_PAGE (Keyboard)                */
    0x19, 0xe0,     /*   USAGE_MINIMUM (Keyboard LeftControl) */
    0x29, 0xe7,     /*   USAGE_MAXIMUM (Keyboard Right GUI)   */
    0x15, 0x00,     /*   LOGICAL_MINIMUM (0)                  */
    0x25, 0x01,     /*   LOGICAL_MAXIMUM (1)                  */
    0x75, 0x01,     /*   REPORT_SIZE (1)                      */
    0x95, 0x08,     /*   REPORT_COUNT (8)                     */
    0x81, 0x02,     /*   INPUT (Data,Var,Abs)                 */
    0x95, 0x01,     /*   REPORT_COUNT (1)                     */
    0x75, 0x08,     /*   REPORT_SIZE (8)                      */
    0x81, 0x03,     /*   INPUT (Cnst,Var,Abs)                 */
    0x95, 0x05,     /*   REPORT_COUNT (5)                     */
    0x75, 0x01,     /*   REPORT_SIZE (1)                      */
    0x05, 0x08,     /*   USAGE_PAGE (LEDs)                    */
    0x19, 0x01,     /*   USAGE_MINIMUM (Num Lock)             */
    0x29, 0x05,     /*   USAGE_MAXIMUM (Kana)                 */
    0x91, 0x02,     /*   OUTPUT (Data,Var,Abs)                */
    0x95, 0x01,     /*   REPORT_COUNT (1)                     */
    0x75, 0x03,     /*   REPORT_SIZE (3)                      */
    0x91, 0x03,     /*   OUTPUT (Cnst,Var,Abs)                */
    0x95, 0x06,     /*   REPORT_COUNT (6)                     */
    0x75, 0x08,     /*   REPORT_SIZE (8)                      */
    0x15, 0x00,     /*   LOGICAL_MINIMUM (0)                  */
    0x25, 0x65,     /*   LOGICAL_MAXIMUM (101)                */
    0x05, 0x07,     /*   USAGE_PAGE (Keyboard)                */
    0x19, 0x00,     /*   USAGE_MINIMUM (Reserved)             */
    0x29, 0x65,     /*   USAGE_MAXIMUM (Keyboard Application) */
    0x81, 0x00,     /*   INPUT (Data,Ary,Abs)                 */
    0xc0            /* END_COLLECTION                         */
  }
};

static struct platform_device planck_hid = {
  .name = "planck_hidg",
  .id = 0,
  .num_resources = 0,
  .resource = 0,
  .dev.platform_data = &planck_hid_data,
  .dev.release = planck_platform_device_release,
};
