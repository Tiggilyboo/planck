#include <linux/i2c.h>

#define MCP23017_IODIRA   0x00
#define MCP23017_IPOLA    0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA  0x06
#define MCP23017_INTCONA  0x08
#define MCP23017_IOCONA   0x0A
#define MCP23017_GPPUA    0x0C
#define MCP23017_INTFA    0x0E
#define MCP23017_INTCAPA  0x10
#define MCP23017_GPIOA    0x12
#define MCP23017_OLATA    0x14  
#define MCP23017_IODIRB   0x01
#define MCP23017_IPOLB    0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB  0x07
#define MCP23017_INTCONB  0x09
#define MCP23017_IOCONB   0x0B
#define MCP23017_GPPUB    0x0D
#define MCP23017_INTFB    0x0F
#define MCP23017_INTCAPB  0x11
#define MCP23017_GPIOB    0x13
#define MCP23017_OLATB    0x15

#define BYTE_TO_BIN(b)\
(b & 0x80 ? '1' : '0'),(b & 0x40 ? '1' : '0'),(b & 0x20 ? '1' : '0'),(b & 0x10 ? '1' : '0'),\ 
(b & 0x08 ? '1' : '0'),(b & 0x04 ? '1' : '0'),(b & 0x02 ? '1' : '0'),(b & 0x01 ? '1' : '0')
#define BYTE_TO_BIN_PAT "%c%c%c%c%c%c%c%c"

static int i2c_read_byte(struct i2c_client *client, unsigned char command) {
  return i2c_smbus_read_byte_data(client, command);
}
static int i2c_write_byte(struct i2c_client *client, int reg, int value) {
  return i2c_smbus_write_byte_data(client, reg, value);
}


