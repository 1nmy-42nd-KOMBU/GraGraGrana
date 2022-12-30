#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
timer = StopWatch()

colorLeft = ColorSensor(Port.S3)
colorRight = ColorSensor(Port.S4)

motorLeft = Motor(Port.A)
motorRight = Motor(Port.D)

@micropython.native
def changeRGBtoHSV(rgb):
    rgb0_255 = rgb[0] * 255 / 100, rgb[1] * 255 / 100, rgb[2] * 255 / 200
    maxRGB, minRGB = max(rgb0_255), min(rgb0_255)
    diff = maxRGB - minRGB

    # Hue
    if maxRGB == minRGB : hue = 0
    elif maxRGB == rgb0_255[0] : hue = 60 * ((rgb0_255[1]-rgb0_255[2])/diff)
    elif maxRGB == rgb0_255[1] : hue = 60 * ((rgb0_255[2]-rgb0_255[0])/diff) + 120
    elif maxRGB == rgb0_255[2] : hue = 60 * ((rgb0_255[0]-rgb0_255[1])/diff) + 240
    if hue < 0 : hue += 360

    # Sqturation
    if maxRGB != 0:
        saturation = diff / maxRGB * 100
    else:
        saturation = 0

    # Value(Brightness)
    value = maxRGB

    return hue,saturation,value

@micropython.native
def powertodegs(power):
    return 1050 * power /100

def main():
    Kp = 1.5
    Ki = 0.2
    Kd = 0.4
    last_error = 0
    error = 0
    basic_speed = 30
    count = 0
    print("start")
    start_time = timer.time()
    while timer.time() <= start_time + 10000:
        rgb_left = colorLeft.rgb()
        rgb_right = colorRight.rgb()
        error = rgb_left[1] - rgb_right[1]
        u = Kp * error + Ki * (error + last_error) + Kd * (error - last_error)
        motorLeft.run(powertodegs(basic_speed + u))
        motorRight.run(powertodegs(basic_speed - u))
        hsv_left = changeRGBtoHSV(rgb_left)
        if 120 < hsv_left[0] < 160 and hsv_left[1] > 60 and hsv_left[2] > 20:
            print("Left sensor is over green")
        # print("left hsv:  "+str(hsv_left[0])+", "+str(hsv_left[1])+", "+str(hsv_left[2]))
        # print("left rgb:  "+str(rgb_left[0])+", "+str(rgb_left[1])+", "+str(rgb_left[2]))
        # wait(100)

        hsv_right = changeRGBtoHSV(rgb_right)
        if 120 < hsv_right[0] < 160 and hsv_right[1] > 60 and hsv_right[2] > 20:
            print("Right sensor is over green")
        # print("right hsv: "+str(hsv_right[0])+", "+str(hsv_right[1])+", "+str(hsv_right[2]))
        # print("right rgb: "+str(rgb_right[0])+", "+str(rgb_right[1])+", "+str(rgb_right[2]))
        # wait(100)
        count += 1

    print(str(10/count*1000))

main()
