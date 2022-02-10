#!/usr/bin/env python3
# 読み込んでいく！
from time import sleep, time
import sys

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering
from ev3dev2.sensor import INPUT_1, INPUT_B, INPUT_C, INPUT_D
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor

from ev3dev2.button import Button
from ev3dev2.sound import Sound

# mux sensor port
muxC1port = LegoPort("in4:i2c80:mux1")  # TouchSensor
muxC2port = LegoPort("in4:i2c81:mux2")  # TouchSensor
muxC3port = LegoPort("in4:i2c82:mux3")    # (driver_name="ms-ev3-smux")

# setting the 1st port on SensorMUX to analogue mode, to be used for touchsensor
muxC1port.mode = "analog"
muxC2port.mode = "analog"
sleep(1) # need to wait for analog mode to be set

# loading the devices for each port
muxC1port.set_device="lego-ev3-touch"
muxC2port.set_device="lego-ev3-touch"
muxC3port.set_device="lego-ev3-color"

touch_sensor_left = TouchSensor("in4:i2c80:mux1")
touch_sensor_right = TouchSensor("in4:i2c81:mux2")
colorsensor4 = ColorSensor("in4:i2c82:mux3")
sleep(1) # need to wait for sensors to be loaded. 0.5 seconds is not enough.

class Sensors_color:
    def __init__(self,port):
        self.port = port
    
    def color(self):
        pass
    
    def refrect(self):
        pass

class Sensors_touch:
    def __init__(self,port):
        self.port = port
    
    def pressed(self):
        pass

botton = Button() # bottons of the brick
sound = Sound()

botton.wait_for_pressed(['enter']) # wait for "enter" botton to be pressed