# Planck Linux i2c keyboard driver

A Linux GPIO i2c keyboard driver for the Planck which provides two modes of operation; an internal input device and external via USB HID output.

Currently the i2c is hardcoded for the MCP23017 GPIO multiplexer, it uses registers only for this board, and interrupt patterns just for this board.

## Planck?

I use this keyboard layout daily, and wanted to make a sort of portable computer version. So I took a NanoPi m4, an i2c input multiplexer to provide enough inputs (Need 16 inputs), and wrote this Linux Kernel Module.

## Setup

You will need to pull in linux headers, currently I am building off of version 4.4.196 of the linux kernel. Newer versions should work maybe with slight coaxing...

All bash code is assuming you are using Armbian, you may cater it to your distro of choice:

Fetch and build the kernel module
```sh
$ git clone https://github.com/tiggilyboo/planck
$ cd planck
$ sudo apt update
$ sudo apt install linux-headers-$(uname -a)
$ make
```

Add the device tree overlay for MCP23017 i2c extender,
Without this, we cant probe and setupt the i2c device:
```sh
$ sudo armbian-add-overlay planck-io.dts
$ sudo reboot 
```

Load libcomposite for external HID usb support, and insert the module
```sh
$ sudo modprobe libcomposite
$ sudo insmod planck.ko
$ dmesg
```

## Modes

When the kernel module has been loaded and you've set up the board, you can switch modes via the configured `KEY_CONNECT` key combination. Currently this is defaulted to pressing: `UPPER + LOWER + Q` buttons (Equivilent of layer 3, x = 1, y = 0). Have a look at where KEY_CONNECT has been set to in `planck_keycodes.h`.

### Internal Mode

When operating in internal mode, the driver will output characters to `planck_device->input`, which is an input device on the local machine (The NanoPi M4 in this case). This means we can happily type while connected to HDMI without an external keyboard. 

### External Mode

When operating in external mode, the driver will output to `/dev/hidg0`, which is a configured slave USB HID device. 


