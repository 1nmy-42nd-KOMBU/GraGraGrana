#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()
watch = StopWatch()

col = ColorSensor(Port.S1)

class ColorValue:
    def hsv(self, rgb_):
        try:
            rgb = rgb_[0] * (255 / 100), rgb_[1] * (255 / 100), rgb_[2] * (255 / 100)

            # Hue
            if rgb[1] < rgb[0] > rgb[2]: # Red
                hue = 60 * ((rgb[1] - rgb[2]) / (max(rgb) - min(rgb)))
            elif rgb[0] < rgb[1] > rgb[2]: # Green
                hue = 60 * ((rgb[2] - rgb[0]) / (max(rgb) - min(rgb))) + 120
            else: # Blue
                hue = 60 * ((rgb[0] - rgb[1]) / (max(rgb) - min(rgb))) + 240

            if hue < 0:
                hue += 360

            # Saturation
            saturation = (max(rgb) - min(rgb)) / max(rgb) * 100

            # Value
            value_brightness = max(rgb_)

            return hue, saturation, value_brightness

        except TypeError as error:
            ev3.speaker.say(error)
    
    def reflection(self, rgb_):
        return rgb_[0]

color = ColorValue()

rgb_ = col.rgb()

hsv_ = color.hsv(rgb_)

if 90 < hsv_[0] < 150 and 40 < hsv_[1] and 40 < hsv_[2]:
    ev3.speaker.say("green")
else:
    ev3.speaker.say("others")