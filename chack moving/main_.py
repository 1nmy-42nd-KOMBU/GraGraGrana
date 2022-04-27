#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()
watch = StopWatch()

col = ColorSensor(Port.S1)

while True:
    rgb = col.rgb()

    if rgb[1] < rgb[0] > rgb[2]: # Red
        hue = 60 * ((rgb[1] - rgb[2]) / (max(rgb) - min(rgb)))
    elif rgb[0] < rgb[1] > rgb[2]: # Green
        hue = 60 * ((rgb[2] - rgb[0]) / (max(rgb) - min(rgb))) + 120
    else: # Blue
        hue = 60 * ((rgb[0] - rgb[1]) / (max(rgb) - min(rgb))) + 240

    if hue < 0:
        hue += 360

    ev3.screen.clear()
    ev3.screen.print(str(rgb) +"_" + str(hue))
