# JY-ME02 Magnetic Rotary Encoder Test

The JY-ME02 encoder is an absolute position encoder. This is available in several forms. This repository is specifically for the RS485 interface version. 
The included Arduino Test file allows it to be used with a RP2040 processor. The current angle and velocity of the sensor are printed on the USB Serial output of the RP2040.
The .h and .cpp files can be used in other Arduino sketches as required.

<img width="200" height="200" alt="JY-ME02" src="https://github.com/user-attachments/assets/ce76ce7a-e00f-477a-9609-3d1aa4ce9aea" />

## Connection
The encoder output uses RS485 for the data transmission. A suitable RS485 to TTL Serial converter module can be found on Amazon for a few pounds. 

<img width="200" height="200" alt="RS485" src="https://github.com/user-attachments/assets/5b02b72e-781b-4dd9-9801-f2ff472d88f2" />


| Encoder | Converter| RP2040 |
| :---:   | :---:    |  :---: |
| VCC     | VCC      |5V   |
| A       | A+       |     |
| B       | B-       |     |
|         | RXD      | GPIO 0 |
|         | TXD      | GPIO 1 |
| GND     | GND      | GND |
