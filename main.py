#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize the EV3
ev3 = EV3Brick()

# Initialize I2C Sensor
C1 = I2CDevice(Port.S1, 0x04)

C1.write(0x04)

while 1:
    tuplea = C1.read(reg=int(hex(4)), length=2)
    print(str(tuplea[0])+", "+str(tuplea[1])) # refrection mode, 2 bytes will be sent
    wait(100)