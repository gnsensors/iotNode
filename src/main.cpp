
#include "mbed.h"

#include "BufferedSerial.h"
#include "ACC.h"
#include "comms.h"


#define SAMPLE_SIZE     (512)
#define BYTES_PER_SAMPLE  (2)
#define NUM_OF_AXIS     (3)
#define BUFFER_SIZE     SAMPLE_SIZE * BYTES_PER_SAMPLE * NUM_OF_AXIS

#define ACC_NEW_DATA_EVNT 0x00000001
#define TIMER_OVERFLOW_EVENT 0x00000002


typedef enum _appState
{
  IDLE,
  SAMPLING,
  SENDING,
  PAUSED
} appState;


I2C i2c(I2C_SDA, I2C_SCL);

Ticker ticker;


DigitalOut myled(LED1);

InterruptIn _Interrup(D4);

BufferedSerial debugPort(D8, D2);

BufferedSerial uart(SERIAL_TX, SERIAL_RX);

volatile uint32_t e =0;


void isr(void)
{
    e |= ACC_NEW_DATA_EVNT;
			myled = 1;
}

void timerCB(void)
{
  __disable_irq();
    e |= TIMER_OVERFLOW_EVENT;
  __enable_irq();

}

int main()
{

    uint8_t sampleBuffer[BUFFER_SIZE];
    int32_t index = 0;
    uint8_t intStatus;

    uint8_t buff[6];
    uint32_t i;
    char inputChar;

    appState state = IDLE;


    uart.baud (115200);
    debugPort.baud(115200);
    i2c.frequency(400000);

    ticker.attach(&timerCB,1);

     _Interrup.fall(&isr);

		 myled = 0;

    debugPort.printf("Starting\n");

    ACC_init(&uart, &i2c);
    comms_init(&uart);


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
      switch (state)
      {
        case IDLE:

            if(0<(e & TIMER_OVERFLOW_EVENT))
            {
              __disable_irq();
                e &= (~TIMER_OVERFLOW_EVENT);
                state = SAMPLING;
              __enable_irq();
              debugPort.printf("Start Sampling\n\r");
            }
            break;
        case SAMPLING:
          if ((e & ACC_NEW_DATA_EVNT)>0)
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
                 state = SENDING;
                 debugPort.printf("Done Sampling\n\r");

               }
             }
           }
          break;
        case SENDING:
         comms_sendPacket(sampleBuffer, BUFFER_SIZE);
         state = IDLE;
         break;

        case PAUSED:

          if(0<(e & TIMER_OVERFLOW_EVENT))
          {
            __disable_irq();
              e &= (~TIMER_OVERFLOW_EVENT);

            __enable_irq();
            debugPort.printf("Paused press R to run\n\r");
          }
          break;

        default:
          debugPort.printf("Bad State\n");
        break;

      }

      while(debugPort.readable()> 0)
      {
        inputChar = debugPort.getc();

        debugPort.putc(inputChar);
        switch (inputChar)
        {
            case 'p':
            debugPort.printf(" --Pausing\n\r");
             state = PAUSED;
             break;
            case 'r':
              debugPort.printf(" --End Pause\n\r");
              state = IDLE;
              break;
            default:
              debugPort.printf(" --Bad Command\n\r");
            break;
        }

      }

    }
  }
