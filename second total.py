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
# ----------------------------------------------------------------------------------------------------
# センサのインスタンスの生成
ColorSensor1 = ColorSensor(PortS1)
ColorSensor2 = ColorSensor(PortS2)
ColorSensor3 = ColorSensor(PortS3)
ColorSensor4 = ColorSensor(PortS4)

TS_left = TouchSensor(portS)
TS_right = TouchSensor(portS)

# ----------------------------------------------------------------------------------------------------
# モーターのインスタンスの生成
left_motor = Motor(port)
right_motor = Motor(port)
SYAKKE = DriveBase(left_motor, right_motor, wheel_diameter=tmp, axle_track=tmp)
# ----------------------------------------------------------------------------------------------------
# 変数の定義
value_sensor = []
black_hightest_refrection = 10
silber_lowest_refrection = 40
refrection_last = (0,0)

#PID制御
Kp = 1.5
Ki = 0.5
Kd = 1.3
individual_difference = 0 #ColorSensor2-ColorSensor3　個体差
errors = [0,0,0,0,0]
# ----------------------------------------------------------------------------------------------------
class Sensor:
    def color_aggregation(self,list):
        value_sensor.clear()
        if 1 in list:
            value_sensor.append(ColorSensor1.color())
        if 2 in list:
            value_sensor.append(ColorSensor2.color())
        if 3 in list:
            value_sensor.append(ColorSensor3.color())
        if 4 in list:
            value_sensor.append(ColorSensor4.color())
        return tuple(value_sensor)
    
    def color_all(self):
        return ColorSensor1.color(), ColorSensor2.color(), ColorSensor3.color(), ColorSensor4.color()
    
    def refrection_aggregation(self,list):
        value_sensor.clear()
        if 1 in list:
            value_sensor.append(ColorSensor1.refrection())
        if 2 in list:
            value_sensor.append(ColorSensor2.refrection())
        if 3 in list:
            value_sensor.append(ColorSensor3.refrection())
        if 4 in list:
            value_sensor.append(ColorSensor4.refrection())
        return tuple(value_sensor)
    
    def refrection_all(self):
        return ColorSensor1.refrection(), ColorSensor2.refrection(), ColorSensor3.refrection(), ColorSensor4.refrection()
    
    def touch(self):
        return TS_left.pressed(), TS_right.pressed()

sensor = Sensor()
# ----------------------------------------------------------------------------------------------------
def speedL(self, rotational_speed):
    return rotational_speed * 1050
        
def speedM(self, rotational_speed):
    return rotational_speed * 1560

class Motor:
    def pid_control(self, base_power = 30):
        value_CS_color = sensor.color_part((2,3))
        error = value_CS_color[1] - value_CS_color[2]
        errors.append(error)
        del errors[0]
        u = Kp * error + Ki * sum(errors) + Kd * (error - errors[-2]) #操作量Kp*e+Ki∫e*dt+Kd*dt

        SYAKKE.drive(speedL(base_power), u)

    def speed(self, base_power, steering):
        SYAKKE.drive(speedL(base_power), steering)
        
    def angle(self, base_power, steering, target_angle, stop_type = Stop.BREAK):
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)

        SYAKKE.drive(speedL(base_power), steering)
        while abs(left_motor.angle()) < target_angle or abs(right_motor.angle()) < target_angle:
            wait(10)
        SYAKKE.stop(stop_type)
        
    def time(self, base_power, steering, run_time, stop_type = Stop.BREAK):
        SYAKKE.drive_time(speedL(base_power), steering, run_time)
        SYAKKE.stop(stop_type)

motor = Motor()
tank = motor.Tank()
# ----------------------------------------------------------------------------------------------------
class Turn:
    pass

    def rescue(self):
        pass

turn = Turn()
# ----------------------------------------------------------------------------------------------------

#ボタン入力があるまで待機
while not any(ev3.bottons.pressed()):
    wait(10)

while True:
    #silber
    if sum([i > silber_lowest_refrection for i in sensor.refrection_aggregation()]) >= 2:
        turn.rescue()
    
    # black
    refrection_now = sensor.refrection_aggregation((1,4))
    if 10 < refrection_last[0] - refrection_now[0] or CS1.color() == 1:
        turn.turn_black("left")
    if 10 < refrection_last[1] - refrection_now[1] or CS4.color() == 1:
        turn.turn_black("right")