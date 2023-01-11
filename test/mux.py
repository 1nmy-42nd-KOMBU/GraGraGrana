#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize the EV3
ev3 = EV3Brick()
wait(100)
# Initialize I2C Sensor
C1 = I2CDevice(Port.S1, 0x50)
C2 = I2CDevice(Port.S1, 0x51)

C1.write(0x52, bytes(0))
C2.write(0x52, bytes(0))
wait(100)

while 1:
    mux = C1.read(reg=0x54 , length=2)
    print(str(mux[0])+","+str(mux[1]))
    mux = C2.read(reg=0x54 , length=2)
    print(str(mux[0])+","+str(mux[1]))
