
#include "ACC.h"

#define ID_REG (0x0F) // ID Register
#define ACC_ADR_W     (0xD6)
#define ACC_ADR_R     (0xD7)

static BufferedSerial* _uart;
static I2C* _i2c;

int ACC_init(BufferedSerial* uart, I2C *i2c)
{
  _uart = uart;
  _i2c = i2c;

  return 1;
}



int ACC_Config(uint8_t reg, uint8_t val)
{
    char data_write[2];

    data_write[0] = reg;
    data_write[1] = val;

    int status = _i2c->write(ACC_ADR_W, data_write, 2, 0);
    if (status != 0) { // Error
        _uart->printf("ERROR I2C\n");
    }

    return status;

}

uint8_t ACC_Read_Status(void)
{
    char data;
    data = 0x17;

    int status = _i2c->write(ACC_ADR_W, &data, 1);
    if (status != 0) { // Error
        _uart->printf("ERROR I2C\n");
    }

    _i2c->read(ACC_ADR_R, &data, 1);
    return (uint8_t) data;

}


int ACC_Read_Axis(uint8_t* buffer)
{

    char localBuff[6];
    char cmd;
    int i;

    cmd = 0x28;

    int status = _i2c->write(ACC_ADR_W, &cmd, 1);
    if (status != 0) { // Error
        _uart->printf("ERROR I2C\n");
    }

    _i2c->read(ACC_ADR_R, localBuff, 6);

    for(i=0;i<6; i++)
    {
      buffer[i] = localBuff[i];
    }

    return  status;

}
