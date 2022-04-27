#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Direction, Color, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

watch = StopWatch()

color1 = ColorSensor(Poet.S1)
color2 = ColorSensor(Poet.S2)
color3 = ColorSensor(Poet.S3)
color4 = ColorSensor(Poet.S4)

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

class Sensors:
    def __init__(self):
        pass

class Tank:
    Kp = 1.2
    Ki = 0.5
    Kd = 1.0
    individual_difference = 0 #cs2-cs3
    errors = [0,0,0,0,0]

    def drive_pid(self, base_speed):
        error = color2.refrection() - color3.refrection() - Tank.individual_difference
        Tank.errors.append(error)
        del Tank.errors[0]
        u = Tank.Kp * error + Tank.Ki * sum(Tank.errors) + Tank.Kd * (error - Tank.errors[-1])
        left_motor.run(base_speed + u)
        right_motor.run(base_speed - u)
    
    def drive_pid_for_seconds(self, base_speed, time):
        time_run = watch.time() 
        while watch.time() <= time_run:
            self.drive_pid
    
    def drive_pid_for_degrees(self, base_speed):
        pass
    
    def drive_pid_for_rotations(self, base_speed):
        pass
    
    def drive(self, left_speed, right_speed):
        left_motor.run(left_speed)
        right_motor.run(right_speed)

    def drive_for_seconds(self, left_speed, right_speed, time, stop_type = Stop.HOLD, wait=True):
        left_motor.run_time(left_speed, time, stop_type, False)
        left_motor.run_time(left_speed, time, stop_type, False)
        if wait:
            wait(time)
    
    def drive_for_degrees(self, left_speed, right_speed, degrees):
        left_angle = left_motor.angle()
        right_angle = right_motor.angle()
        while not abs(left_angle - left_motor.angle()) > degrees or abs(right_angle - right_motor.angle()) > degrees:
            left_motor.run(left_speed)
            right_motor.run(right_speed)
    
    def drive_for_rotations(self, left_speed, right_speed, rotations):
        degrees = rotations * 360
        left_angle = left_motor.angle()
        right_angle = right_motor.angle()
        while not abs(left_angle - left_motor.angle()) > degrees or abs(right_angle - right_motor.angle()) > degrees:
            left_motor.run(left_speed)
            right_motor.run(right_speed)
    
    def avoid(self):
        pass

tank = Tank()

base_speed = 40

while 1:
    # PID-control
    tank.drive_pid(base_speed)
    
    # touch
    if base_speed - left_motor.speed() > 40 or base_speed - left_motor.speed() < -40:
        pass