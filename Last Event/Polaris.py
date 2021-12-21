#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

#インスタンスの生成
left_color = ColorSensor(Port.S1)
mid_left_color = ColorSensor(Port.S2)
mid_right_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S4)

dist1 = UltrasonicSensor(Port.S23)
dist2 = UltrasonicSensor(Port.S24)

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
bucketleft = Motor(Port.D)
bucketright = Motor(Port.A)

door = Motor(Port.C2)

motor = Motor()
tank = motor.Tank()
bucketmove = Motor().Bucket()
boxmove = Motor().Boxforballs()

#ボタン入力があるまで待機
while not any(ev3.buttons.pressed()):
    wait(10)

if Button.CENTER in ev3.buttons.pressed():
    Silber_time = 3
    line()
elif Button.LEFT in ev3.buttons.pressed():
    Silber_time = 2
    line()
elif Button.RIGHT in ev3.buttons.pressed():
    Silber_time = 1

rescue()

def line():
    Kp = 1.7
    Kd = 1.4
    last_error = 0
    Silber_refrection = 40
    Black_refrection = 5

    while True:
        #GREEN
        if ((mid_left_color.color() == "Color.GREEN" or mid_right_color.color() == "Color.GREEN") 
            and (left_color.color() == "Color.WHITE" and right_color.color() == "Color.WHITE")):
            if mid_left_color.color() == "Color.GREEN" and mid_right_color.color() == "Color.GREEN":
                Turn().Green_line_Uturn()
            else:
                Turn().Green_line_right_left()

        #BLACK
        if ((left_color.color() == "Color.BLACK" and mid_left_color.color() == "Color.BLACK") or
            (right_color.color() == "Color.BLACK" and mid_right_color.color() == "Color.BLACK")):
            #もしすべての反射光が5以下(黒)なら
            if ((left_color.refrection() < Black_refrection and mid_left_color.refrection() < Black_refrection) and 
                right_color.refrection() < Black_refrection and mid_right_color.refrection() < Black_refrection):
                #2.5秒間はしる 後の緑を回避するため
                tank.time(40, 40, 2.5, Stop.COAST, False)
            else:
                #片方だけならどっちかに曲がれ
                Turn().Black_line_right_left()

        #DISTANCE
        if 15 < dist1.distance():
            Turn.avoid()

        #SILBER
        if mid_left_color.refrection() > Silber_refrection and mid_right_color.refrection() > Silber_refrection:
            if Silber_time == 0:
                break
            else:
                Silber_time -= 1
                limit = time.time() + 2.5
                while limit <= time.time():
                    Motor().PD_control(40)
        
        Motor().PD_control(40)
    
    class Turn:
        def Green_line_right_left(self):
            if mid_left_color.color() == "Color.GREEN":
                tank.angle(25,25,360)
                while mid_left_color.color() != "Color.BLACK":
                    tank.speed(-25,25)
                while mid_left_color.color() != "Color.WHITE" or mid_right_color.color() != "Color.WHITE":
                    tank.speed(-25,25)
            else:
                tank.angle(25,25,360)
                while mid_right_color.color() != "Color.BLACK":
                    tank.speed(25,-25)
                while mid_left_color.color() != "Color.WHITE" or mid_right_color.color() != "Color.WHITE":
                    tank.speed(25,-25)
    
        def Green_line_Uturn(self):
            while mid_right_color.color() != "Color.BLACK":
                tank.speed(25, -25)
            while mid_left_color.color() != "Color.WHITE" or mid_right_color.color() != "Color.WHITE":
                tank.speed(25, -25)

        def Black_line_right_left(self):
            if mid_left_color.color() == "Color.GREEN":
                tank.angle(25,25,360)
                while mid_left_color.color() != "Color.BLACK":
                    tank.speed(-25,25)
                while mid_left_color.color() != "Color.WHITE" or mid_right_color.color() != "Color.WHITE":
                    tank.speed(-25,25)
            else:
                tank.angle(25,25,360)
                while mid_right_color.color() != "Color.BLACK":
                    tank.speed(25,-25)
                while mid_left_color.color() != "Color.WHITE" or mid_right_color.color() != "Color.WHITE":
                    tank.speed(25,-25)
    
        def avoid(self):
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

def rescue():
    #go back
    tank.angle(-25,-25,tmp)
    bucketmove.open()
    while 3.2 < dist2.distance():
        tank.speed(40.40)
    bucketmove.close()

class Motor:

    def PD_control(self, base_power):
        error = mid_left_color.refrection() - mid_right_color.refrection()
        u = Kp * error + Kd * (error - last_error)
        left_motor.run(100 + u)
        right_motor.run(100 -u)
        last_error = error

    class Tank:
        def speed(self, left_speed, right_speed):
            left_motor.run(left_speed)
            right_motor.run(right_speed)
        
        def angle(self, left_speed, right_speed,angle):
            left_motor.run_angle(left_speed, angle, stop_type = Stop.BLAKE, wait_type = False)
            right_motor.run_angle(right_speed, angle, stop_type = Stop.BRAKE, wait_type = False)
            if not wait_type:
                wait(run_time)

        def time(self, left_speed, right_speed, run_time, stop_type = Stop.BLAKE, wait_type = False):
            left_motor.run_time(left_speed, run_time, stop_type,wait_type)
            right_motor.run_time(right_speed, run_time, stop_type, wait_type)
            if not wait_type:
                wait(run_time)
    
    class Bucket:
        def opening(self):
            bucketleft.run_angle(30, -180, Stop.COAST, True)
            bucketright.run_angle(30, -180, Stop.COAST, True)
        
        def closing(self):
            bucketleft.run_angle(30, 180, Stop.COAST, True)
            bucketright.run_angle(30, 180, Stop.COAST, True)

    class Boxforballs:
        def opening(self):
            door.run_angle(30, 190, Stop.BRAKE, False)

        def closing(self):
            door.run_angle(30, 190, Stop.BRAKE, False)