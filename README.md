# Goal
To get first place at OKAYAMA Block Tournament of RCJ2023

# Issues at the last event
About Hardware
1. A robot which we made was not able to drive up a hill road.
2. When it turns by a wall, it sometimes collides with the wall.
3. It mistakes not a silber line for a silber line(due to 5th issue).
4. When it drives down in a hill road, it rolls down.
5. The distance between color sensors and flloor is too close.

About Software

1. It often fail to turn corners which consist of black lines.
2. It was not able to save people(collect balls).
3. It was not able to decide to turn left or right to avoid a obstacle.

etc...

# To solve them
About hardware

1. Increase friction of a robot
2. Create a round robot
3. Same as 5th
4. Same as 1st
5. Install color sensors between wheels.

About software

1. Use values of refrection of color sensors to decide black lines
2. Do our bsst
3. Turn right and measure distance to a wall. Second, if there is a wall, turn 180°

# PID-Control
```python.pid-control.py
#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

left_motor = Motor(Port.A) # ロボットに合わせて変える
right_motor = Motor(Port.B)

left_color_sensor = ColorSensor(Port.S1)
right_color_sensor = ColorSensor(Port.S2)

Kp = 1.5 # P制御
Ki = 0.5 # I制御
Kd = 1.3 # D制御
individual_difference = 0 # 個体差※１

errors = [0,0,0,0,0] # ※２

# こいつをループさせる
def pid_control(base_power):
    error = left_color_sensor.refrection() - right_color_sensor.refrection() - individual_difference
    	# 現在の偏差を取得
    errors.append(error)
    del errors[0]

    u = Kp * error + Ki * sum(errors) + Kd * (error - errors[-2]) # 操作量=Kp*e+Ki∫e*dt+Kd*dt

    left_motor.run(base_power + u) # 走らせる
    right_motor.run(base_power - u)
```