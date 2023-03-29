#!/usr/bin/env pybricks-micropython
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.tools import wait

pico = UARTDevice(Port.S2, 19200,100)
pico.clear()
esp = UARTDevice(Port.S1, 115200,100)
esp.clear()
wait(1000)
print("start")

for i in range(1000):
    pico.write((10).to_bytes(1,'big'))
    while pico.waiting() < 1:
        wait(10)
        print("error"+str(i))
        pico.write((10).to_bytes(1,'big'))
    whatread = pico.read()
    if whatread[0] != 10:
        print(whatread[0])
    wait(3)

for i in range(1000):
    esp.write((10).to_bytes(1,'big'))
    while esp.waiting() < 1:
        wait(10)
        print("error"+str(i))
        esp.write((10).to_bytes(1,'big'))
    whatread = esp.read()
    if whatread[0] != 10:
        print(whatread[0])
    wait(3)

print("fin")