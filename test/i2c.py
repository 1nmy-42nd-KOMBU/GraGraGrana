#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize the EV3
ev3 = EV3Brick()

# Initialize I2C Sensor
device = I2CDevice(Port.S1, 0x04 >> 1)

print(device.read(reg=hex(4), length=1))