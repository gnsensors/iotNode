#include "mbed.h"
//#include "arm_math.h"



#define ID_REG (0x0F) // ID Register

#define ACC_ADR_W     (0xD6)
#define ACC_ADR_R     (0xD7)
#define SAMPLE_SIZE     (512)
#define ACC_NEW_DATA_EVNT 0x00000001


I2C i2c(I2C_SDA, I2C_SCL);

DigitalOut myled(LED1);

InterruptIn _Interrup(D4);

Serial pc(SERIAL_TX, SERIAL_RX);


typedef struct _vector{

    int16_t x;
    int16_t y;
    int16_t z;
} vector_t;

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


int ACC_Read_Axis(int16_t* x, int16_t* y, int16_t* z)
{

    char data[6];
    char cmd;



      cmd = 0x28;

    int status = i2c.write(ACC_ADR_W, &cmd, 1);
    if (status != 0) { // Error
        pc.printf("ERROR I2C\n");
    }

    i2c.read(ACC_ADR_R, data, 6);

    *x = (int16_t) (data[0] | (data[1]>>8 & 0xFF00));
    *y = (int16_t) (data[2] | (data[3]>>8 & 0xFF00));
    *z = (int16_t) (data[4] | (data[5]>>8 & 0xFF00));


    return  status;

}


void isr(void)
{
    e |= ACC_NEW_DATA_EVNT;
			myled = 1;
}

int main()
{

    vector_t v[SAMPLE_SIZE];
    int32_t index = 0;

    uint8_t intStatus;

    int16_t x, y, z;

    x = 0;
    y=0;
    z=0;

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
		ACC_Read_Axis(&x, &y, &z);

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
                ACC_Read_Axis(&x, &y, &z);

                if(index<SAMPLE_SIZE)
                {
                    v[index].x = x;
                    v[index].y = y;
                    v[index].z = z;
                    index++;

                    
                }
                else
                {
                    index = 0;
                    pc.printf("AAAA\n");
                    for(int i=0;i<SAMPLE_SIZE; i++)
                    {
                        pc.printf("%d, %d, %d\n", v[i].x, v[i].y, v[i].z);
                    }
                    pc.printf("BBBB\n");
                }
            }
        }
    }
}
