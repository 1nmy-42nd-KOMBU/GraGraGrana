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
    tuple__ = C1.read(reg=int(hex(4)), length=8)
    tuple_ = tuple__.encode('utf-8')
    print(str(tuple_[0])+","+str(tuple_[1])+","+str(tuple_[2])+","+str(tuple_[3])+","+str(tuple_[4])+","+str(tuple_[5])+","+str(tuple_[6])+","+str(tuple_[7])) # refrection mode, 2 bytes will be sent
    wait(100)