#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

# センサーのインスタンスの生成
CS1 = ColorSensor(PortS1)
CS2 = ColorSensor(PortS2)
CS3 = ColorSensor(PortS3)
CS4 = ColorSensor(PortS4)

TS_left = TouchSensor(portS)
TS_right = TouchSensor(portS)

# モーターのインスタンスの生成
left_motor = Motor(port)
right_motor = Motor(port)

turn = Turn()
motor = Motor()
tank = motor.Tank()

sensors = Sensors()
# ---------------------------------------------------------------------------------
# 定義
black_hightest_refrection = 10
silber_lowest_refrection = 40
Kp = 1.5
Ki = 0.5
Kd = 1.3
individual_difference = 0 #cs2-cs3　個体差

turn90 = 0
turn180 = 0
#ここまでは事前にやっとく
errors = [0,0,0,0,0]

informations_return = []
refrection_last = sensors.part_refrection((1,4))

# ---------------------------------------------------------------------------------
#ボタン入力があるまで待機
while not any(ev3.buttons.pressed()):
    wait(10)

while True:
    #silber
    if any(i > silber_lowest_refrection for i in sensors.all_refrection()):
        rescue()
    #black
    refrection_now = sensors.part_refrection((1,4))
    if 10 < refrection_last[1] - refrection_now[1] or CS1.color() == "Color.BLACK":
        turn.turn_black("left")
    if 10 < refrection_last[2] - refrection_now[2] or CS4.color() == "Color.BLACK":
        turn.turn_black("right")
    #green
    if "Color.GREEN" in sensors.part_color(2,3):
        turn.turn_green()
    #red
    if any(i == "Color.RED" for i in sensors.all_color()):
        break

    motor.pid_control(40)

# ---------------------------------------------------------------------------------

def rescue():
    run=1

# ---------------------------------------------------------------------------------

class Turn:

    def turn_black(self,direction):
        tank.angle(30,30,90) # 黒の上に乗る
        # 十字路のジャッジ
        if CS1.refrection() < black_hightest_refrection and CS4.refrection() < black_hightest_refrection:
            limit = time.time() + 2.0
            while limit >= time.time():
                motor.pid_control(40) # 黒線上を脱する

            if direction == "left": # left
                tank.angle(40,20,360) # 軌道修正&位置調整
                while (CS1.color() != "Color.BLACK") or (CS3.color() != "Color.BLACK" or CS4.color() != "Color.BLACK"):
                    tank.speed(-30,30)
                if CS1.color() == "Color.BLACK": # 90
                    while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                        tank.speed(-30,30)
                else:
                    while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                        tank.speed(30,-30)

        else: # right
            tank.angle(20,40,360)
            while (CS4.color() != "Color.BLACK") or (CS2.color() != "Color.BLACK" or CS1.color() != "Color.BLACK"):
                tank.speed(30,-30)
            if CS4.color() == "Color.BLACK": # 90
                while not CS3.color() == "Color.WHITE" and CS2.color() == "Color.WHITE":
                    tank.speed(30,-30)
            else:
                while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                    tank.speed(-30,30)

    def turn_green(self):
        tank.angle(30,30,90) # go a bit
        if CS2.color() == "Color.GREEN" and CS3.color() == "Color.GREEN":
            while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                tank.speed(30,-30)
            while not CS3.color() == "Color.BLACK":
                tank.speed(30,-30)
            while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                tank.speed(30,-30)

        elif CS2.color() == "Color.BLACK": # left
            tank.angle(30,30,360) # 線の中央に乗る
            while not CS4.color() == "Color.BLACK":
                tank.speed(-30,30)
            while not CS2.color() == "Color.BLACK":
                tank.speed(-30,30)
            while not CS2.color() == "Color.BLACK" and CS3.color() == "Color.BLACK":
                tank.speed(-30,30)
        else: #right
            tank.angle(30,30,360) # 線の中央に乗る
            while not CS1.color() == "Color.BLACK":
                tank.speed(30,-30)
            while not CS3.color() == "Color.BLACK":
                tank.speed(30,-30)
            while not CS2.color() == "Color.BLACK" and CS3.color() == "Color.BLACK":
                tank.speed(30,-30)

    def avoid():
        run=1
        
        tank.angle(25,-25,tmp)
        tank.angle(25,25,tmp)
        tank.angle(-25,25,tmp)
        tank.angle(25,25,tmp)
        tank.angle(-25,25,tmp)
        while mid_left_color.color() != "Color.BLACK" or mid_right_color.color() != "Color.BLACK":
            tank.speed(25,25)
        tank.angle(25,25,360)
        while mid_right_color.color() != "Color.BLACK":
            tank.speed(25,-25)
        while mid_left_color.color() != "Color.WHITE" or mid_right_color.color() != "Color.WHITE":
            tank.speed(25,-25)


class Motor:

    def pid_control(self,base_power):
        error = CS2.refrection() - CS3.refrection() - individual_difference
        #偏差の累積を操作
        errors.append(error)
        del errors[0]
        u = Kp * error + Ki * sum(errors) + Kd * (error - errors[-2]) #操作量Kp*e+Ki∫e*dt+Kd*dt

        left_motor.run(base_power + u)
        right_motor.run(base_power - u)
    
    class Tank:

        def speed(self, left_speed, right_speed):
            left_motor.run(left_speed)
            right_motor.run(right_speed)
        
        def angle(self, left_speed, right_speed,angle):# 要検討
            left_motor.run_angle(left_speed, angle, stop_type = Stop.BLAKE, wait_type = False)
            right_motor.run_angle(right_speed, angle, stop_type = Stop.BRAKE, wait_type = False)
            if not wait_type:
                wait(run_time)

        def time(self, left_speed, right_speed, run_time, stop_type = Stop.BLAKE, wait_type = False):
            left_motor.run_time(left_speed, run_time, stop_type,wait_type)
            right_motor.run_time(right_speed, run_time, stop_type, wait_type)
            if not wait_type:
                wait(run_time)

class Sensors:
    def all_refrection(self):
        return CS1.refrection(), CS2.refrection(), CS3.refrection(), CS4.refrection()
    
    def all_color(self):
        return CS1.color(), CS2.color(), CS3.color(), CS4.color()
    
    def part_refrection(self,which_return):
        if 1 in which_return:
            informations_return.append(CS1.refrection())
        if 2 in which_return:
            informations_return.append(CS2.refrection())
        if 3 in which_return:
            informations_return.append(CS3.refrection())
        if 4 in which_return:
            informations_return.append(CS4.refrection())
        return informations_return
    
    def part_color(self,which_return):
        if 1 in which_return:
            informations_return.append(CS1.color())
        if 2 in which_return:
            informations_return.append(CS2.color())
        if 3 in which_return:
            informations_return.append(CS3.color())
        if 4 in which_return:
            informations_return.append(CS4.color())
        return informations_return