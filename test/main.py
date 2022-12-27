#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port, Direction, Color, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

watch = StopWatch()

colorLeft = ColorSensor(Poet.S3)
colorRight = ColorSensor(Poet.S4)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

class Sensors:
    def __init__(self):
        pass

class Tank:
    Kp = 0.8
    Ki = 0.1
    Kd = 0.5
    individual_difference = 0 #cs2-cs3
    last_error = 0

    @micropython.viper
    def drive_pid(self, base_speed):
        leftRGB = colorLeft.rgb()
        rightRGB = colorRight.rgb()
        if leftRGB[0] is None:
            leftRGB[0] = 0
        if leftRGB[1] is None:
            leftRGB[1] = 0
        if leftRGB[2] is None:
            leftRGB[2] = 0
        if rightRGB[0] is None:
            rightRGB[0] = 0
        if rightRGB[1] is None:
            rightRGB[1] = 0
        if rightRGB[2] is None:
            rightRGB[2] = 0

        error = leftRGB[1] - rightRGB[1] - Tank.individual_difference
        u = Tank.Kp * error + Tank.Ki * (Tank.last_error + error) + Tank.Kd * (error - Tank.last_error)
        left_motor.run(base_speed + u)
        right_motor.run(base_speed - u)
        Tank.last_error = error

    def drive_pid_for_seconds(self, base_speed, time):
        time_run = watch.time() 
        pid = self.drive_pid()
        while watch.time() <= time_run:
            pid()

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
        left_run = left_motor.run(left_speed)
        right_run = right_motor.run(right_speed)
        while not abs(left_angle - left_motor.angle()) > degrees or abs(right_angle - right_motor.angle()) > degrees:
            left_run()
            right_run()
    
    def drive_for_rotations(self, left_speed, right_speed, rotations):
        degrees = rotations * 360
        left_angle = left_motor.angle()
        right_angle = right_motor.angle()
        left_run = left_motor.run(left_speed)
        right_run = right_motor.run(right_speed)
        while not abs(left_angle - left_motor.angle()) > degrees or abs(right_angle - right_motor.angle()) > degrees:
            left_run()
            right_run()

    # # left------------------------------------------------------------------------------
    # rgb = leftRGB[0] * (255 / 100), leftRGB[1] * (255 / 100), leftRGB[2] * (255 / 100)
    # maxRBG = max(rgb)
    # minRGB = min(rgb)

    # # Hue
    # if maxRGB == rgb[0]: # Red 
    #     left_hue = 60 * ((rgb[1] - rgb[2]) / (maxRGB - minRGB))
    # elif maxRBG == rgb[1]: # Green
    #     left_hue = 60 * ((rgb[2] - rgb[0]) / (maxRGB - minRGB)) + 120
    # else: # Blue
    #     left_hue = 60 * ((rgb[0] - rgb[1]) / (maxRGB - minRGB)) + 240

    # if left_hue < 0:
    #     left_hue += 360

    # # Saturation
    # left_saturation = (maxRGB - minRGB) / maxRGB * 100

    # # Value
    # left_value_brightness = max(leftRGB)

    # # right--------------------------------------------------------------------------------
    # rgb = rightRGB[0] * (255 / 100), rightRGB[1] * (255 / 100), rightRGB[2] * (255 / 100)
    # maxRBG = max(rgb)
    # minRGB = min(rgb)

    # # Hue
    # if maxRGB == rgb[0]: # Red 
    #     right_hue = 60 * ((rgb[1] - rgb[2]) / (maxRGB - minRGB))
    # elif maxRBG == rgb[1]: # Green
    #     right_hue = 60 * ((rgb[2] - rgb[0]) / (maxRGB - minRGB)) + 120
    # else: # Blue
    #     right_hue = 60 * ((rgb[0] - rgb[1]) / (maxRGB - minRGB)) + 240

    # if right_hue < 0:
    #     right_hue += 360

    # # Saturation
    # right_saturation = (maxRGB - minRGB) / maxRGB * 100

    # # Value
    # right_value_brightness = max(rightRGB)

tank = Tank()

pid_40 = tank.drive_pid(40)

while 1:
    # PID-control
    pid_40()