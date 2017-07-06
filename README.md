# iotNode

This is a simple project to serially send accelerometer data over the serial port. 

# Hardware

STM32 Nucleo-L152RE  http://www.st.com/en/evaluation-tools/nucleo-l152re.html

ST X-NUCLEO-IKS01A1 http://www.st.com/en/ecosystems/x-nucleo-iks01a1.html

# IDE

Uses the MBED platform as well as Platform.io running on Ubuntu 16.04 with Atom IDE


# Usage
The device will initialize the Accelerometer to sample at it fastest rate with an automatic BW setting.  It will store a buffer of 3 axis data (SAMPLE_SIZE) then send all samples over the serial port

## Serial port settings
115200 1N8 no flow control


