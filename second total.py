#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import I2CDevice
import time

# センサのインスタンスの生成
CS1 = ColorSensor(PortS1)
CS2 = ColorSensor(PortS2)
CS3 = ColorSensor(PortS3)
CS4 = ColorSensor(PortS4)

TS_left = TouchSensor(portS)
TS_right = TouchSensor(portS)

# モーターのインスタンスの生成
left_motor = Motor(port)
right_motor = Motor(port)
SYAKKE = DriveBase(left_motor, right_motor, wheel_diameter=tmp, axle_track=tmp)

def sensor_color():
    part_color
    return