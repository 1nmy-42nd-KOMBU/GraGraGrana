#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, Sensor

from ev3dev2.display import Display 
from time import sleep

dsp = Display()

col = ColorSensor(INPUT_1)
tou = TouchSensor(UNPUT_2)

while True:
    dsp.update()
    dsp.text_pixels(int(col.value()) + "_" + int(tou.value()),True,0,52,font = 'courB' + '24')

# text_pixels(text, clear_screen=True, x=0, y=0, text_color=’black’, font=None)