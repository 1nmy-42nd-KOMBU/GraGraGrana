#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4, Sensor
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.port import LegoPort
from ev3dev2.button import Button
from ev3dev2.sound import Sound

# from ev3dev.brickpi import *
from ev3dev.ev3 import *
from ev3dev.auto import *

from time import sleep, time
import sys

from ev3dev2.button import Button
from ev3dev2.sound import Sound
sleep_time = 1

# Mux-wise--------------------------------------------------------------------------------------------
# mux sensor port
muxC1port = LegoPort("in4:i2c80:mux1")  # TouchSensor
muxC2port = LegoPort("in4:i2c81:mux2")  # TouchSensor
muxC3port = LegoPort("in4:i2c82:mux3")    # (driver_name="ms-ev3-smux")

# setting the 1st port on SensorMUX to analogue mode, to be used for touchsensor
muxC2port.mode = "analog"
muxC3port.mode = "analog"
sleep(sleep_time) # need to wait for analog mode to be set

# loading the devices for each port
muxC1port.set_device = "lego-ev3-color"
muxC2port.set_device = "lego-ev3-touch"
muxC3port.set_device = "lego-ev3-touch"

colorsensor4 = ColorSensor("in4:i2c80:mux1")
touch_sensor_left = TouchSensor("in4:i2c81:mux2")
touch_sensor_right = TouchSensor("in4:i2c82:mux3")
sleep(sleep_time) # need to wait for sensors to be loaded. 0.5 seconds is not enough.

colorsensor4.mode = "COL-REFLECT" #
# ----------------------------------------------------------------------------------------------------

colorsensor1 = ColorSensor(INPUT_1)
colorsensor2 = ColorSensor(INPUT_2)
colorsensor3 = ColorSensor(INPUT_3)

motor_left = LargeMotor(OUTPUT_A)
motor_right = LargeMotor(OUTPUT_B)
tank = MoveTank(OUTPUT_A, OUTPUT_B)
# ----------------------------------------------------------------------------------------------------

button = Button() # bottons of the brick
sound = Sound()

class Sensors_color:
    def __init__(self,port):
        self.port = port
    
    def color(self):
        if self.port == 1:
            return colorsensor1.color
        elif self.port == 2:
            return colorsensor2.color
        elif self.port == 3:
            return colorsensor3.color
        elif self.port == 4:
            sound.speak("Load color sensor")
            colorsensor4.mode = "COL-COLOR"
            sleep(sleep_time)
            return colorsensor4.value()
            colorsensor4.mode = "COL-REFLECT"
            sleep(sleep_time)

    def refrect(self):
        if self.port == 1:
            return colorsensor1.reflected_light_intensity
        elif self.port == 2:
            return colorsensor2.reflected_light_intensity
        elif self.port == 3:
            return colorsensor3.reflected_light_intensity
        elif self.port == 4:
            return colorsensor4.reflected_light_intensity

CS1 = Sensors_color(1)
CS2 = Sensors_color(2)
CS3 = Sensors_color(3)
CS4 = Sensors_color(4)

class Sensors_touch:
    def __init__(self,port):
        self.port = port
    
    def pressed(self):
        if self.port == 5:
            return True if touch_sensor_left.value() == 257 else False
        else:
            return True if touch_sensor_right.value() == 257 else False

TS_left = Sensors_touch(5)
TS_right = Sensors_touch(6)
# ----------------------------------------------------------------------------------------------------

class Motor:
    pass

class Turn:
    pass

while not button.enter(): #wait while all buttons arent pressed
    button.wait_for_bump("left") # start button

    while True:
        pass