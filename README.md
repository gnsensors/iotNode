# iotNode

This is a simple project to serially send accelerometer data over the serial port.

# Hardware

STM32 Nucleo-L152RE  http://www.st.com/en/evaluation-tools/nucleo-l152re.html

ST X-NUCLEO-IKS01A1 http://www.st.com/en/ecosystems/x-nucleo-iks01a1.html

# IDE

Uses the MBED platform as well as Platform.io running on Ubuntu 16.04 with Atom IDE


# Usage
The device will initialize the Accelerometer to sample at it fastest rate with an automatic BW setting.  It will store a buffer of 3 axis 16 bit data (SAMPLE_SIZE* BYTES_PER_SAMPLE * NUM_OF_AXIS) then send all samples over the serial port.  A 32 bit CRC is added to the message

## Serial port settings
115200 1N8 no flow control

## Message format  <HEADER><SYNC><LENGTH><PAYLOAD><CRC>
HEADER = 0xAA 0xAA 0xAA 0xAA (this looks nice on an Oscope so it is easy to debug and see baudrate)
SYNC = two bytes to show start of length 0xFF 0x00 (also nice on an oscope)
LENGTH = 16 bit unsigned integer of the number of bytes being sent little Endian including CRC
PAYLOAD = 8 bit Bytes of data
CRC = CRC32 - 32 bit unsigned integer sent Little endian
