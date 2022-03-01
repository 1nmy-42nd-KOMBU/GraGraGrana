#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4, Sensor
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.port import LegoPort
from ev3dev2.button import Button
from ev3dev2.sound import Sound

# from ev3dev.brickpi import *
from ev3dev.ev3 import *
from ev3dev.auto import *

from time import sleep, time
import sys

from ev3dev2.button import Button
from ev3dev2.sound import Sound
sleep_time = 1

# Mux-wise--------------------------------------------------------------------------------------------
# mux sensor port
muxC1port = LegoPort("in4:i2c80:mux1")  # TouchSensor
muxC2port = LegoPort("in4:i2c81:mux2")  # TouchSensor
muxC3port = LegoPort("in4:i2c82:mux3")    # (driver_name="ms-ev3-smux")

# setting the 1st port on SensorMUX to analogue mode, to be used for touchsensor
muxC2port.mode = "analog"
muxC3port.mode = "analog"
sleep(sleep_time) # need to wait for analog mode to be set

# loading the devices for each port
muxC1port.set_device = "lego-ev3-color"
muxC2port.set_device = "lego-ev3-touch"
muxC3port.set_device = "lego-ev3-touch"

colorsensor4 = ColorSensor("in4:i2c80:mux1")
touch_sensor_left = TouchSensor("in4:i2c81:mux2")
touch_sensor_right = TouchSensor("in4:i2c82:mux3")
sleep(sleep_time) # need to wait for sensors to be loaded. 0.5 seconds is not enough.

colorsensor4.mode = "COL-REFLECT" #
# instance--------------------------------------------------------------------------------------------

colorsensor1 = ColorSensor(INPUT_1)
colorsensor2 = ColorSensor(INPUT_2)
colorsensor3 = ColorSensor(INPUT_3)

motor_left = LargeMotor(OUTPUT_A)
motor_right = LargeMotor(OUTPUT_B)
movetank = MoveTank(OUTPUT_A, OUTPUT_B)
movesteering = MoveSteering(OUTPUT_A, OUTPUT_B)
# values----------------------------------------------------------------------------------------------

black_highset_refrect = None
silber_lowset_refrect = None
whites = [2,4,6]
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
    Kp = 1.5
    Ki = 0.5
    Kd = 1.3
    individual_difference = 0 #cs2-cs3
    errors = [0,0,0,0,0]

    angle_90 = None
    angle_180 = None

    def position(self):
        return movetank.position

    def on_pid(self,base_power):
        error = CS2.refrect - CS3.refrect()
        Motor.errors.append(error)
        del Motor.errors[-1]

        u = (Motor.Kp * error) + (Motor.Ki * sum(Motor.errors)) + (Motor.Kd * (error - Motor.errors[-1]))
        movetank.on(base_power + u,base_power - u)
    
    def on_pid_for_seconds(self,base_power,second,stop_type = True):
        limit = time.time() + second
        while limit > time.time():
            self.on_pid(base_power)
        movetank.off(stop_type)
    
    def on_pid_for_degrees(self,base_power,degree,stop_type = True):
        initial_degree_left = motor_left.position
        initial_degree_right = motor_right.position
        while degree > ((abs(initial_degree_left - motor_left.position) + abs(initial_degree_right - motor_right.position))) / 2:
            self.on_pid(base_power)
        movetank.off(stop_type)
    
    def on_pid_for_rotations(self,base_power,rotations,stop_type = True):
        initial_degree_left = motor_left.position
        initial_degree_right = motor_right.position
        while rotations > ((abs(initial_degree_left - motor_left.position) + abs(initial_degree_right - motor_right.position))) / 2 / 360:
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
        first_position = self.position()
        while abs(self.position() - first_position) < None or (not CS2.color() == 3 and CS3.color() == 3):
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
        pass
    
    def save(self):
        pass

tank = Motor()
# ----------------------------------------------------------------------------------------------------

avoider = True
last_refrect = 0,0

while not button.enter(): #wait while all buttons arent pressed
    
    button.wait_for_bump("left") # start button

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