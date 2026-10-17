# JY-ME02 Magnetic Rotary Encoder Test

The JY-ME02 encoder is an absolute position encoder. This is available in several forms. This repository is specifically for the RS485 interface version. 
The included Arduino Test file allows it to be used with a RP2040 processor. The current angle and velocity of the sensor are printed on the USB Serial output of the RP2040.
The .h and .cpp files can be used in other Arduino sketches as required.

## Connection
The encoder output uses RS485 for the data transmission. A suitable RS485 to TTL Serial converter module can be found on Amazon for a few pounds. 


