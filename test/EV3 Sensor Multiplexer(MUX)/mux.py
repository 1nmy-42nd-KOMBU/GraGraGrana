#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# --------------------------------------------------
# This program shows how to use MUX in Pybricks
# For details, just see the user guide of MUX
# --------------------------------------------------

# Initialize the EV3
ev3 = EV3Brick()

# Initialize I2C Sensor
# Color Sensor is  pulled in port C1
# I2C address of C1 of MUX is 0x50(C2=>0x51,C3=>0x52)
# This will be run only on Port 1
C1 = I2CDevice(Port.S1, 0x50)

# set mode to Refrection Mode
# a register of sensor mode is 0x52
# there is not a register of sensor type
# For details, see the user guide of MUX
C1.write(reg=0x52,data=bytes(0))
wait(100)

while True:
    # this code order MUX to send a refrection value,which is 2 bytes
    # A sesnor read data will be sent with 0x54
    C1_value = C1.read(reg=0x54, length=2)
    # display 2 bytes as one value by bit shift
    print(str((C1_value[0] << 8) + C1_value[1]))
    wait(100)
