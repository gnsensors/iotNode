#include "comms.h"

#define CRC_SIZE    (4) //number of bytes for CRC32


static uint8_t header[]={0xAA,0xAA,0xAA,0xAA};

static uint8_t sync[] = {0xFF, 0x00};

static BufferedSerial* _uart;


int comms_init(BufferedSerial *uart)
{
  _uart = uart;

  return 1;
}

int comms_sendPacket(uint8_t *packet, uint16_t length)
{

    uint32_t crc;
    uint16_t payloadSize;

    payloadSize = length + CRC_SIZE;

    crc = calculate_crc32((char*) packet,length);

    //<HEADER>
    _uart->write(header, 4);

    //SYNC
    _uart->write(sync,2);

    //<LENGTH>
    _uart->write( &payloadSize, 2);

    //<PAYLOAD>
    _uart->write(packet, length);

    //<CRC>
    _uart->write(&crc, 4);

    return 1;
}
