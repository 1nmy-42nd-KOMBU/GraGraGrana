#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

colorLeft = ColorSensor(Port.S3)
colorRight = ColorSensor(Port.S4)

motorLeft = Motor(Port.A)
motorRight = Motor(Port.D)

Kp = 0.8
Ki = 0.08
Kd = 0.2
last_error = 0
error = 0
basic_speed = 30

while 1:
    rgb_left = colorLeft.rgb()
    print(str(rgb_left[0])+", "+str(rgb_left[1])+", "+str(rgb_left[2]))

    rgb_right = colorRight.rgb()
    print(str(rgb_right[0])+", "+str(rgb_right[1])+", "+str(rgb_right[2]))
  
