#include "mbed.h"
#include "lib_crc.h"

//#include "arm_math.h"

#define ID_REG (0x0F) // ID Register
#define ACC_ADR_W     (0xD6)
#define ACC_ADR_R     (0xD7)
#define SAMPLE_SIZE     (512)
#define BYTES_PER_SAMPLE  (2)
#define NUM_OF_AXIS     (3)
#define BUFFER_SIZE     SAMPLE_SIZE * BYTES_PER_SAMPLE * NUM_OF_AXIS
#define CRC_SIZE    (4) //number of bytes for CRC32
#define ACC_NEW_DATA_EVNT 0x00000001

I2C i2c(I2C_SDA, I2C_SCL);

DigitalOut myled(LED1);

InterruptIn _Interrup(D4);

Serial pc(SERIAL_TX, SERIAL_RX);

volatile uint32_t e =0;

int ACC_Config(uint8_t reg, uint8_t val)
{
    char data_write[2];

    data_write[0] = reg;
    data_write[1] = val;

    int status = i2c.write(ACC_ADR_W, data_write, 2, 0);
    if (status != 0) { // Error
        pc.printf("ERROR I2C\n");
    }

    return status;

}

uint8_t ACC_Read_Status(void)
{
    char data;
    data = 0x17;

    int status = i2c.write(ACC_ADR_W, &data, 1);
    if (status != 0) { // Error
        pc.printf("ERROR I2C\n");
    }

    i2c.read(ACC_ADR_R, &data, 1);
    return (uint8_t) data;

}


int ACC_Read_Axis(uint8_t* buffer)
{

    char localBuff[6];
    char cmd;
    int i;

    cmd = 0x28;

    int status = i2c.write(ACC_ADR_W, &cmd, 1);
    if (status != 0) { // Error
        pc.printf("ERROR I2C\n");
    }

    i2c.read(ACC_ADR_R, localBuff, 6);

    for(i=0;i<6; i++)
    {
      buffer[i] = localBuff[i];
    }

    return  status;

}


void isr(void)
{
    e |= ACC_NEW_DATA_EVNT;
			myled = 1;
}

int main()
{

    uint8_t sampleBuffer[BUFFER_SIZE];
    int32_t index = 0;
    uint8_t intStatus;
    uint32_t crc;
    uint8_t buff[6];
    uint32_t i;

    uint16_t txBufferSize = BUFFER_SIZE + CRC_SIZE;

    uint8_t lengthLSB =(uint8_t) (txBufferSize & 0x00FF);
    uint8_t lengthMSB = (uint8_t) ((txBufferSize>>8) & 0x00FF);

    pc.baud (115200);
    i2c.frequency(400000);

     _Interrup.fall(&isr);

		 myled = 0;

    pc.printf("Starting\n");

    ACC_Config(0x0C, 0x01);
    ACC_Config(0x1E, 0x00);
    ACC_Config(0x1F, 0x38);
    ACC_Config(0x20, 0xC0);
    ACC_Config(0x21, 0x00);
    ACC_Config(0x22, 0x64);
    ACC_Config(0x23, 0x08);

		// read the status and axis data to clear interrupts
  	intStatus = ACC_Read_Status();

		ACC_Read_Axis(buff);

    while(1)
    {

        if((e & ACC_NEW_DATA_EVNT)>0)
        {
            //clear event
            __disable_irq();
            e &= (~ACC_NEW_DATA_EVNT);
						myled = 0;
            __enable_irq();

            intStatus = ACC_Read_Status();
            if((intStatus & 0x01)>0)
            {
                ACC_Read_Axis(buff);

                if(index<(BUFFER_SIZE))
                {
                  //Copy new samples into buffer
                  for(i=0;i<6;i++)
                  {
                    sampleBuffer[index] = buff[i];
                    index++;
                  }
                }
                else
                {
                    index = 0;
                    crc = calculate_crc32((char*) sampleBuffer,BUFFER_SIZE);

                    //<HEADER>
                    pc.putc(0xAA);
                    pc.putc(0xAA);
                    pc.putc(0xAA);
                    pc.putc(0xAA);

                    //SYNC
                    pc.putc(0xFF);
                    pc.putc(0x00);


                    //<LENGTH>
                    pc.putc(lengthLSB);
                    pc.putc(lengthMSB);

                    //<PAYLOAD>
                    for(int i=0;i<BUFFER_SIZE; i++)
                    {
                        pc.putc(sampleBuffer[i]);
                    }

                    //<CRC>
                    pc.putc(crc & 0x000000FF);
                    pc.putc((crc>>8) & 0x000000FF);
                    pc.putc((crc>>16) & 0x000000FF);
                    pc.putc((crc>>24) & 0x000000FF);

                }
            }
        }
    }
}
