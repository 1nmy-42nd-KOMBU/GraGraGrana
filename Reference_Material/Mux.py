#!/usr/bin/env python3
# imports
from ev3dev.ev3 import *
from time import sleep, time
import sys

# -------------------------------
# CONNECTING SENSORS
# -------------------------------

# setting LegoPort interface
muxC1port = LegoPort("in2:i2c80:mux1")
muxC2port = LegoPort("in2:i2c81:mux2")    # (driver_name="ms-ev3-smux")
muxC3port = LegoPort("in2:i2c82:mux3")

# setting the 1st port on SensorMUX to analogue mode, to be used for touchsensor
muxC1port.mode = "analog"
sleep(1) # need to wait for analog mode to be set

# loading the devices for each port
muxC1port.set_device="lego-ev3-touch"
muxC2port.set_device="lego-ev3-us"
muxC3port.set_device="lego-ev3-ir"

ts = TouchSensor("in2:i2c80:mux1")
us = UltrasonicSensor("in2:i2c81:mux2")
ir = InfraredSensor("in2:i2c82:mux3")
sleep(1) # need to wait for sensors to be loaded. 0.5 seconds is not enough.

us.mode = "US-DIST-CM" # setting to measure distance in centimeters
ir.mode = "IR-PROX" # setting to measure proximity.

def debug_print(*args, **kwargs):
    # Print debug messages to stderr. This shows up in the output panel in VS Code.
    print(*args, **kwargs, file=sys.stderr)

def main():
    # writing initial sensor values
    debug_print("TS: " + str(ts.value()) + " IR: " + str(ir.value()) + " US: " + str(us.value()))
    while True:
        if ts.value() == 257: # 256 == not pressed, 257 == pressed
            Sound.speak("Touch sensor")
        if ir.value() <= 10:
            Sound.speak("Infrared sensor")
        if us.value() <= 50:
            Sound.speak("Ultrasonic sensor")

if __name__ == '__main__':
    main()