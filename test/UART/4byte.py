#!/usr/bin/env pybricks-micropython
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.tools import wait

arduino = UARTDevice(Port.S1, 115200,100)
arduino.clear()
wait(1000)
print("start")

while 1:
    arduino.write((10).to_bytes(1,'big'))
    while arduino.waiting() < 4:
        wait(10)
        print("error")
        arduino.write((10).to_bytes(1,'big'))
    whatread = arduino.read(4)
    print(str(whatread[0])+", "+str(whatread[1])+", "+str(whatread[2])+", "+str(whatread[3]))
    wait(3)