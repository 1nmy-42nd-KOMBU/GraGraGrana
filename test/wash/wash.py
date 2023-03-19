#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.iodevices import UARTDevice

ev3 = EV3Brick()

# Lモーターを接続
try:
    motorLeft = Motor(Port.D)
    motorRight = Motor(Port.A)
except OSError as oserror:
    while True:
        ev3.speaker.say("motor")
        wait(1000)

@micropython.native
def powertodegs(power):
    """スピード(%)をdeg/sに変換する"""
    return 950 * power /100

motorLeft.run(powertodegs(50))
motorRight.run(powertodegs(50))

# ボタンが押されるのを待つ
while not any(ev3.buttons.pressed()):
    pass