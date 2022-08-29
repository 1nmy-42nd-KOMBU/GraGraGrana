#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()
watch = StopWatch()

col = ColorSensor(Port.S1)

time_start = watch.time()

for i in range(2000):
    ev3.screen.clear()
    ev3.screen.print(str(col.color()) + "_" + str(i) + "_" + str(watch.time() - time_start))

wait(10000)