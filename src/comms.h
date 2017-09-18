
#ifndef _COMMS_H
#define _COMMS_H
#include "mbed.h"
#include "BufferedSerial.h"
#include "lib_crc.h"


int comms_init(BufferedSerial *uart);

int comms_sendPacket(uint8_t *packet, uint16_t length);


#endif //_COMMS_H
