#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port

# Initialize the EV3
ev3 = EV3Brick()

# Initialize I2C Sensor
C1 = I2CDevice(Port.S2, 0x50 >> 1)

while 1:
    print(C1.read(reg=hex(0), length=2)) # refrection mode, 2 bytes will be sent
    wait(100)