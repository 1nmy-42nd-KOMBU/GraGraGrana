#!/usr/bin/env python3

# from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering
# from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
# from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, Sensor
# from ev3dev2.port import LegoPort
# from ev3dev2.button import Button
# from ev3dev2.sound import Sound

# from ev3dev.brickpi import *
# from ev3dev.ev3 import *
from ev3dev.auto import *

import ev3dev.ev3 as ev3
import ev3fast

from time import sleep, time
import sys

sleep_time = 1

# Mux-wise--------------------------------------------------------------------------------------------
# mux sensor port
muxC1port = ev3.LegoPort("in4:i2c80:mux1")  # TouchSensor
muxC2port = ev3.LegoPort("in4:i2c81:mux2")  # TouchSensor
muxC3port = ev3.LegoPort("in4:i2c82:mux3")    # (driver_name="ms-ev3-smux")

# setting the 1st port on SensorMUX to analogue mode, to be used for touchsensor
muxC2port.mode = "analog"
muxC3port.mode = "analog"
sleep(sleep_time) # need to wait for analog mode to be set

# loading the devices for each port
muxC1port.set_device = "lego-ev3-color"
muxC2port.set_device = "lego-ev3-touch"
muxC3port.set_device = "lego-ev3-touch"

colorsensor4 = ev3.ColorSensor("in4:i2c80:mux1")
touch_sensor_left = ev3.TouchSensor("in4:i2c81:mux2")
touch_sensor_right = ev3.TouchSensor("in4:i2c82:mux3")
sleep(sleep_time) # need to wait for sensors to be loaded. 0.5 seconds is not enough.

colorsensor4.mode = "COL-REFLECT" #
# instance--------------------------------------------------------------------------------------------

colorsensor1 = ev3fast.ColorSensor(INPUT_1)
colorsensor2 = ev3fast.ColorSensor(INPUT_2)
colorsensor3 = ev3fast.ColorSensor(INPUT_3)

motor_left = ev3fast.LargeMotor(OUTPUT_A)
motor_right = ev3fast.LargeMotor(OUTPUT_B)
movetank = ev3fast.MoveTank(OUTPUT_A, OUTPUT_B)
movesteering = ev3fast.MoveSteering(OUTPUT_A, OUTPUT_B)
# values----------------------------------------------------------------------------------------------

black_highset_refrect = None
silber_lowset_refrect = None
whites = [2,4,6]
avoider = True
last_refrect = 0,0
# Sensors---------------------------------------------------------------------------------------------

button = Button() # bottons of the brick
sound = Sound()

class Sensors_color:
    def __init__(self,port):
        self.port = port
    
    def color(self):
        if self.port == 1:
            return colorsensor1.color
        elif self.port == 2:
            return colorsensor2.color
        elif self.port == 3:
            return colorsensor3.color
        elif self.port == 4:
            sound.speak("Load color sensor")
            colorsensor4.mode = "COL-COLOR"
            sleep(sleep_time)
            return colorsensor4.value()
            colorsensor4.mode = "COL-REFLECT"
            sleep(sleep_time)

    def refrect(self):
        if self.port == 1:
            return colorsensor1.reflected_light_intensity
        elif self.port == 2:
            return colorsensor2.reflected_light_intensity
        elif self.port == 3:
            return colorsensor3.reflected_light_intensity
        elif self.port == 4:
            return colorsensor4.reflected_light_intensity

CS1 = Sensors_color(1)
CS2 = Sensors_color(2)
CS3 = Sensors_color(3)
CS4 = Sensors_color(4)

class Sensors_touch:
    def __init__(self,port):
        self.port = port
    
    def pressed(self):
        if self.port == 5:
            return True if touch_sensor_left.value() == 257 else False
        else:
            return True if touch_sensor_right.value() == 257 else False

TS_left = Sensors_touch(5)
TS_right = Sensors_touch(6)
# Motors----------------------------------------------------------------------------------------------
class Motor:
    def __init__(self,port):
        self.port = port
    
    def position(self):
        return motor_l.position() if self.port == "left" else motor_r.position()

motor_l = Motor("left")
motor_r = Motor("right")

class Motors:
    Kp = 1.5
    Ki = 0.5
    Kd = 1.3
    individual_difference = 0 #cs2-cs3
    errors = [0,0,0,0,0]

    angle_90 = None
    angle_180 = None

    def on_pid(self,base_power):
        error = CS2.refrect - CS3.refrect() - Motors.individual_difference
        Motors.errors.append(error)
        del Motors.errors[0]

        u = (Motors.Kp * error) + (Motors.Ki * sum(Motors.errors)) + (Motors.Kd * (error - Motors.errors[-1]))
        movetank.on(base_power + u,base_power - u)
    
    def on_pid_for_seconds(self,base_power,second,stop_type = True):
        limit = time.time() + second
        while limit > time.time():
            self.on_pid(base_power)
        movetank.off(stop_type)
    
    def on_pid_for_degrees(self,base_power,degree,stop_type = True):
        initial_degree_left = motor_l.position()
        initial_degree_right = motor_r.position()
        while degree > ((abs(initial_degree_left - motor_l.position()) + abs(initial_degree_right - motor_r.position()))) / 2:
            self.on_pid(base_power)
        movetank.off(stop_type)
    
    def on_pid_for_rotations(self,base_power,rotations,stop_type = True):
        initial_degree_left = motor_l.position()
        initial_degree_right = motor_r.position()
        while rotations > ((abs(initial_degree_left - motor_l.position()) + abs(initial_degree_right - motor_r.position()))) / 2 / 360:
            self.on_pid(base_power)
        movetank.off(stop_type)
    
    def stop(stop_type = True):
        movetank.off(stop_type)

    def on(self,left_speed,right_speed):
        movetank.on(left_speed,right_speed)

    def on_for_degrees(self,left_speed,right_speed,degrees,stop_type = True):
        movetank.on_for_degrees(left_speed,right_speed,degrees,stop_type)
            #on_for_degrees(left_speed, right_speed, degrees, brake=True, block=True)

    def on_for_rotations(self,left_speed,right_speed,rotations,stop_type = True):
        movetank.on_for_rotations(left_speed,right_speed,rotations,stop_type)
            #on_for_rotations(left_speed, right_speed, rotations, brake=True, block=True)

    def on_for_seconds(self,left_speed,right_speed,seconds,stop_type = True):
        movetank.on_for_seconds(left_speed,right_speed,seconds,stop_type)
            #on_for_seconds(left_speed, right_speed, seconds, brake=True, block=True)
    
    def turn_left(self,base_power):
        movetank.on_for_degrees(-1 * base_power,base_power,None)

    def turn_right(self,base_power):
        movetank.on_for_degrees(base_power,-1 * base_power,None)
    
    def on_steering(speed,steering):
        movesteering.on(steering,speed)
    
    def on_for_degrees_steering(speed,steering,degrees,stop_type = True):
        movesteering.on_for_degrees(steering,speed,degrees,stop_type)
    
    def on_for_rotations_steering(speed,steering,rotations,stop_type = True):
        movesteering.on_for_rotations(steering,speed,rotations,stop_type)
    
    def on_for_seconds_steering(speed,steering,seconds,stop_type = True):
        movesteering.on_for_seconds(steering,speed,seconds,stop_type)

    def black_quarter(self):
        if CS1.refrect() < black_highset_refrect and CS4.refrect() < black_highset_refrect:
            # go away
            self.on_pid_for_degrees(None,None,False)
        elif CS1.refrect() < CS4.refrect():
            # left
            self.on_for_degrees(30 + 10,30,None)
            while CS1.color() != 1 and CS4.color() != 1:
                self.on(-30,30)
            if CS1.refrect() < black_highset_refrect:
                while not CS2.color() == 1:
                    self.on(-30,30)
                while not CS2.color == 6:
                    self.on(-30,30)
            else:
                while not CS3.color() == 1:
                    self.on(30,-30)
                while not CS3.color() == 1 and CS3.color() == 1:
                    self.on(30,-30)
        else:
            # right
            self.on_for_degrees(30,30 + 10,None)
            while CS4.color() != 1 and CS4.color() != 1:
                self.on(30,-30)
            if CS4.refrect() < black_highset_refrect:
                while not CS3.color() == 1:
                    self.on(30,-30)
                while not CS3.color == 6:
                    self.on(30,-30)
            else:
                while not CS2.color() == 1:
                    self.on(-30,30)
                while not CS2.color() in whites and CS3.color() in whites:
                    self.on(-30,30)
        self.stop()

    def green(self,direction):
        first_position = motor_l.position()
        while abs(motor_l.position() - first_position) < None and (not CS2.color() == 3 and CS3.color() == 3):
            self.on(30,30)
        if CS2.color() == 3 and CS3.color() == 3:
            while CS2.color() in (1,3):
                self.on(30,-30)
            while not CS3 == 1:
                self.on(30,-30)
            while not CS2.color() in whites and CS3.color() in whites:
                self.on(30,-30)
        elif direction == "left":
            while not CS2 == 1:
                self.on(-30,30)
            while not CS2.color() in whites and CS3.color() in whites:
                self.on(-30,30)
        else:
            while not CS3 == 1:
                self.on(30,-30)
            while not CS2.color() in whites and CS3.color() in whites:
                self.on(30,-30)
        self.stop()

    def avoid(self):
        self.turn_right()
        first_position = motor_l.position()
        while abs(first_position - motor_l.position()) > None and TS_left.pressed() + TS_right.pressed() == 256 * 2:
            self.on(30,30)
        turn_direction = -1 if TS_left.pressed() + TS_right.pressed() != 256 * 2 else 1

        self.on_for_degrees(-30,-30,None)
        if turn_direction == -1:
            self.turn_right()
            self.turn_right()

    def save(self):
        pass

tank = Motors()
# ----------------------------------------------------------------------------------------------------

while not button.enter(): #wait while all buttons arent pressed
    
    button.wait_for_pressed(['up', 'left']) # start button
    
    if button.up():
        try:
            pass
        except:
            Sound.speak("a error has been found")
        else:
            Sound.speak("No error has been found")
    else:
        while True:
            # silber----------------------------------------------------------------------------------------------
            if 2 <= sum(i > silber_lowset_refrect for i in (CS1.refrect(),CS2.refrect(),CS3.refrect(),CS4.refrect())) and avoider:
                avoider = False
                tank.avoid()
            # black-----------------------------------------------------------------------------------------------
            if last_refrect[0] - CS1.refrect() > None or last_refrect[1] - CS4.refrect() > None:
                tank.black_quarter()
            last_refrect = CS1.refrect(),CS4.refrect()
            # Green-----------------------------------------------------------------------------------------------
            if (CS1.color() in whites and CS2.color() == 3) or (CS4.color() in whites and CS3.color() == 3):
                tank.green("left" if CS2.color() == 3 else "right")
            # Red-------------------------------------------------------------------------------------------------
            if 2 <= sum(i == 5 for i in (CS1.color(),CS2.color(),CS3.color(),CS4.color())):
                break
            # Touch-----------------------------------------------------------------------------------------------
            if TS_left.pressed() + TS_right.pressed() != 256 * 2:
                tank.avoid()
            # PID-Control-----------------------------------------------------------------------------------------
            tank.on_pid(40)