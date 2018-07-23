#include <xc.h>
#include "mcp9808.h"
#include "i2c1.h"

uint8_t initMCP(){
    if (i2c_receive_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_MANUF_ID) != 0x0054) return 0;
    if (i2c_receive_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_DEVICE_ID) != 0x0400) return 0;
    
    i2c_write_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_CONFIG, 0x0);
    return 1;
}

uint16_t readTemp( void )
{
  uint16_t t = i2c_receive_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_AMBIENT_TEMP);
  return t;
}

void mcpShutdown(void)
{
  mcpShutdown_wake(1);
}

void mcpShutdown_wake(uint8_t sw_ID)
{
    uint16_t conf_shutdown ;
    uint16_t conf_register = i2c_receive_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_CONFIG);
    if (sw_ID == 1)
    {
       conf_shutdown = conf_register | MCP9808_REG_CONFIG_SHUTDOWN ;
       i2c_write_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_CONFIG, conf_shutdown);
    } 
    else if (sw_ID == 0)
    {
       conf_shutdown = conf_register & ~MCP9808_REG_CONFIG_SHUTDOWN ;
       i2c_write_16bit_data(MCP9808_I2CADDR_DEFAULT, MCP9808_REG_CONFIG, conf_shutdown);
    }
}

void mcpWake(void)
{
  mcpShutdown_wake(0);
  //__delay_ms(250);
}

