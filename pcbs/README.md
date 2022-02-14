# PCBs
 
PCBs created in KiCad 6.0

https://www.kicad.org/download/


All works in progress, PCBs on the way for testing.

### Expansion Board

RPi expansion board to control servo motors and other peripherals without using a microcontroller.

### Motor Controller

Controls 12 motion RC servos using a Teensy 4.0 microcontroller. 

Connects to and powers RPi.

Requires external DC-DC converter to supply 5vDC to board and RPi.

Contains peripheral support for I2C IMU, touch down switches, LED, Neopixel (ws2812b) led strips, current sense, voltage sense, auxillary RC servos, and extra I2C devices.

Fabrication specifications: 4 layer, 1oz (2oz preferred and may be required, testing still in progress).

### Battery Board

Mounts 8x 18650 LiPo batteries and 4x HY2120 (or similar) battery managment systems.

Fabrication specifications: 2 layer, 1oz (2oz preferred and may be required, testing still in progress).
