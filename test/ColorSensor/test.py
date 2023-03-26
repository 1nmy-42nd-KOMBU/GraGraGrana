#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.iodevices import UARTDevice

# 設定コーナー ====================================================
ev3 = EV3Brick()
timer = StopWatch()

# センサーとモーターをEV3と接続していくぅ↑---------------------------
# 失敗した時はその内容を吐露しながら停止するよ

# カラーセンサを接続
try:
    colorLeft = ColorSensor(Port.S4)
    colorRight = ColorSensor(Port.S3)
except OSError as oserror:
    while True:
        ev3.speaker.say("color")
        wait(1000)

@micropython.native
def changeRGBtoHSV(rgb):
    """
    RGBをHSVに変換して返す(タプル型)
    色相(H)は 0~360
    彩度(S)は 0~100
    明度(V)は 0~255 の範囲
    """
    # RGBのしきい値を0~100から0~255に修正 Blueの値が異様に大きい問題があるので÷2して実際に見える色に寄せている
    rgb0_255 = rgb[0] * 255 / 100, rgb[1] * 255 / 100, rgb[2] * 255 / 200
    maxRGB, minRGB = max(rgb0_255), min(rgb0_255)
    diff = maxRGB - minRGB

    # Hue
    if maxRGB == minRGB : hue = 0
    elif maxRGB == rgb0_255[0] : hue = 60 * ((rgb0_255[1]-rgb0_255[2])/diff)
    elif maxRGB == rgb0_255[1] : hue = 60 * ((rgb0_255[2]-rgb0_255[0])/diff) + 120
    elif maxRGB == rgb0_255[2] : hue = 60 * ((rgb0_255[0]-rgb0_255[1])/diff) + 240
    if hue < 0 : hue += 360

    # Saturation
    if maxRGB != 0:
        saturation = diff / maxRGB * 100
    else:
        saturation = 0

    # Value(Brightness)
    value = maxRGB

    return hue,saturation,value

while 1:
    rgb_left = colorLeft.rgb() # 左のRGBをゲットだぜ
    rgb_right = colorRight.rgb() # 右のRGｂをゲットだぜ

    # 左の緑判定
    hsv_left = changeRGBtoHSV(rgb_left) # HSVの値をゲット
    if 140 < hsv_left[0] < 180 and hsv_left[1] > 50 and hsv_left[2] > 20:
        print("L")
    #print("left hsv:  "+str(hsv_left[0])+", "+str(hsv_left[1])+", "+str(hsv_left[2]))
    print("left rgb:  "+str(rgb_left[0])+", "+str(rgb_left[1])+", "+str(rgb_left[2]))

    # 右の緑判定
    hsv_right = changeRGBtoHSV(rgb_right) # HSVの値をゲット
    if 140 < hsv_right[0] < 180 and hsv_right[1] > 50 and hsv_right[2] > 20:
        print("R")
    #print("right hsv: "+str(hsv_right[0])+", "+str(hsv_right[1])+", "+str(hsv_right[2]))
    print("right rgb: "+str(rgb_right[0])+", "+str(rgb_right[1])+", "+str(rgb_right[2]))
    wait(500)