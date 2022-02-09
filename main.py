#!/usr/bin/env python3
# 読み込んでいく！
from time import sleep

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering
from ev3dev2.sensor import INPUT_1, INPUT_B, INPUT_C, INPUT_D
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor