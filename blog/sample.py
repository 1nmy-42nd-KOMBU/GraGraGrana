#!/usr/bin/env python3
from time import sleep
from ev3dev.ev3 import *

muxC1port = LegoPort("in2:i2c80:mux1") # タッチセンサー
muxC2port = LegoPort("in2:i2c82:mux3") # 超音波センサー

muxC1port.mode = "analog"
sleep(1)

muxC1port.set_device = "lego-ev3-touch"
muxC2port.set_device = "lego-ev3-us"

touchsensor = TouchSensor("in2:i2c80:mux1")
ultrasonicsensor = UltrasonicSensor("in2:i2c81:mux2")
colorsensor = ColorSensor("in2:i2c82:mux3")
sleep(1)

ultrasonicsensor.mode = "US-DIST-CM"
colorsensor.mode = "COL-COLOR"

print(touchsensor.value())
print(ultrasonicsensor.value())
print(colorsensor.value())