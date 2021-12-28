#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

left_motor = Motor(Port.A) #ロボットに合わせて変える
right_motor = Motor(Port.B)

left_color_sensor = ColorSensor(Port.S1)
right_color_sensor = ColorSensor(Port.S2)

Kp = 1.5
Ki = 0.5
Kd = 1.3
individual_difference = 0 #cs2-cs3個体差

errors = [0,0,0,0,0]

def pid_control(base_power):
    error = CS2.refrection() - CS3.refrection() - individual_difference
    #偏差の累積を操作
    errors.append(error)
    del errors[0]

    u = Kp * error + Ki * sum(errors) + Kd * (error - errors[-2]) #操作量Kp*e+Ki∫e*dt+Kd*dt

    left_motor.run(base_power + u)
    right_motor.run(base_power - u)