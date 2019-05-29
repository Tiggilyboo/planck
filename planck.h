// I2C Access
#define I2C_ACCESS    0x0720
#define I2C_READ      1
#define I2C_WRITE     0
#define I2C_BLOCK_MAX 32

// MCP23017 Operating address
#define I2C_ADDR      0x27

// MCP23017 A/B registers
#define I2C_IODIRA    0x00
#define I2C_IPOLA     0x02
#define I2C_GPINTENA  0x04
#define I2C_DEFVALA   0x06
#define I2C_INTCONA   0x08
#define I2C_IOCONA    0x0A
#define I2C_GPPUA     0x0C
#define I2C_INTFA     0x0E
#define I2C_INTCAPA   0x10
#define I2C_GPIOA     0x12
#define I2C_OLATA     0x14
#define I2C_IODIRB    0x01
#define I2C_IPOLB     0x03
#define I2C_GPINTENB  0x05
#define I2C_DEFVALB   0x07
#define I2C_INTCONB   0x09
#define I2C_IOCONB    0x0B
#define I2C_GPPUB     0x0D
#define I2C_INTFB     0x0F
#define I2C_INTCAPB   0x11
#define I2C_GPIOB     0x13
#define I2C_OLATB     0x15

#define I2C_INTERR    255
