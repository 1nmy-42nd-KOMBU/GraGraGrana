#!/usr/bin/env pybricks-micropython
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.tools import wait

arduino = UARTDevice(Port.S1, 38400,100)
arduino.clear()

for i in range(1000):
    arduino.write((10).to_bytes(1,'big'))
    while arduino.waiting() < 1:
        wait(1)
    whatread = arduino.read()
    print(whatread[0])