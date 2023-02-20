#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

ev3 = EV3Brick()
timer = StopWatch()

try:
    colorLeft = ColorSensor(Port.S3)
    colorRight = ColorSensor(Port.S4)

    motorLeft = Motor(Port.A)
    motorRight = Motor(Port.D)
except OSError as oserror:
    while True:
        ev3.speaker.say("os error")
        wait(1000)
else:
    ev3.speaker.say("all sensors and motors have found")

highest_refrection_of_Black = 15

# ============================================================

class Tank:
    """
    ・モーター関連のクラス
    ・Mindstormsのタンクとステアリングの機能+アルファ
    ・このクラスは実行速度を一切考慮していないので注意
    ・回転方向の指定はパワーですることを想定している
    """
    def drive(self, left_speed, right_speed):
        motorLeft.run(powertodegs(left_speed))
        motorRight.run(powertodegs(right_speed))

    def drive_for_seconds(self, left_speed, right_speed, time, stop_type = "hold", wait=True):
        motorLeft.run_time(powertodegs(left_speed), time, stop_type, False)
        motorRight.run_time(powertodegs(right_speed), time, stop_type, False)
        if wait:
            wait(time)
        
        self.stop(stop_type)

    def drive_for_degrees(self, left_speed, right_speed, degrees, stop_type = "hold"):
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        motorLeft.run(powertodegs(left_speed))
        motorRight.run(powertodegs(right_speed))
        while not abs(left_angle - motorLeft.angle()) > degrees or abs(right_angle - motorRight.angle()) > degrees:
            pass
        self.stop(stop_type)

    def drive_for_rotations(self, left_speed, right_speed, rotations, stop_type = "hold"):
        self.drive_for_degrees(left_speed,right_speed,rotations* 360,stop_type)

    def steering(self,speed,steering):
        if -100 > speed or 100 > speed:
            raise ValueError
        if -100 <= steering < 0:
            motorLeft.run(powertodegs((speed / 50) * steering + speed))
            motorRight.run(powertodegs(speed))
        elif 0 <= steering <= 100:
            motorLeft.run(powertodegs(speed))
            motorRight.run(powertodegs(-1 * (speed / 50) * steering + speed))
        else:
            raise ValueError

    def steering_for_seconds(self,speed,steering,seconds,stop_type = "hold"):
        if seconds <= 0:
            raise ValueError
        time_run = timer.time() + seconds
        while timer.time() <= time_run:
            self.steering(speed,steering)
        self.stop(stop_type)

    def steeing_for_degrees(self,power,steering,degrees,stop_type = "hold"):
        if degrees < 0:
            degrees *= -1
            steering *= -1
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        self.steering(power,steering)
        while not abs(left_angle - motorLeft.angle()) > degrees or abs(right_angle - motorRight.angle()) > degrees:
            pass
        self.stop(stop_type)

    def steering_for_rotations(self,power,steering,rotations,stop_type = "hold"):
        self.steeing_for_degrees(power,steering,rotations * 360,stop_type)

    def stop_option(self,stop_type):
        if stop_type == "stop":
            return Stop.COAST
        elif stop_type == "brake":
            return Stop.BRAKE
        else:
            return Stop.HOLD

    def stop(self, stop_type):
        if stop_type == "stop":
            motorLeft.stop()
            motorRight.stop()
        elif stop_type == "brake":
            motorLeft.brake()
            motorRight.brake()
        else:
            motorLeft.hold()
            motorRight.hold()

tank = Tank()
# ------------------------------------------------------------
@micropython.native
def changeRGBtoHSV(rgb):
    rgb0_255 = rgb[0] * 255 / 100, rgb[1] * 255 / 100, rgb[2] * 255 / 200
    maxRGB, minRGB = max(rgb0_255), min(rgb0_255)
    diff = maxRGB - minRGB

    # Hue
    if maxRGB == minRGB : hue = 0
    elif maxRGB == rgb0_255[0] : hue = 60 * ((rgb0_255[1]-rgb0_255[2])/diff)
    elif maxRGB == rgb0_255[1] : hue = 60 * ((rgb0_255[2]-rgb0_255[0])/diff) + 120
    elif maxRGB == rgb0_255[2] : hue = 60 * ((rgb0_255[0]-rgb0_255[1])/diff) + 240
    if hue < 0 : hue += 360

    # Sqturation
    if maxRGB != 0:
        saturation = diff / maxRGB * 100
    else:
        saturation = 0

    # Value(Brightness)
    value = maxRGB

    return hue,saturation,value

@micropython.native
def powertodegs(power):
    return 950 * power /100
# ------------------------------------------------------------
def onGreenMarker(direction):
    if direction == "l":
        start_angle_deg = motorRight.angle()
        isRightGreen = False

        motorLeft.run(powertodegs(basic_speed))
        motorRight.run(powertodegs(basic_speed))

        while abs(motorRight.angle() - start_angle_deg) <= 50:
            if isGreen('r'):
                isRightGreen = True
        motorLeft.brake()
        motorRight.brake()

        if isRightGreen:
            u_turn()
        else:
            print('turn left')
            tank.drive_for_degrees(30,30,230) # go down 230 deg
            tank.drive_for_degrees(-30,30,180) # spin turn "left" 180 deg
            tank.drive(-30,30)
            while colorLeft.rgb()[1] > highest_refrection_of_Black: # spin turn "left" until left color sensor finds black or green
                pass
            tank.drive_for_degrees(-30,30,110) # spin turn "left" 110 deg to be over line
            motorLeft.brake()
            motorRight.brake()
    else: # "r" --------------------------------------------------------------------------
        start_angle_deg = motorLeft.angle()
        isLeftGreen = False

        motorLeft.run(powertodegs(30))
        motorRight.run(powertodegs(30))

        while abs(motorLeft.angle() - start_angle_deg) <= 50:
            if isGreen('r'):
                isLeftGreen = True
        motorLeft.brake()
        motorRight.brake()

        if isLeftGreen:
            u_turn()
        else:
            print('turn right')
            tank.drive_for_degrees(30,30,230) # go down 230 deg
            tank.drive_for_degrees(30,-30,180) # spin turn "right" 180 deg
            tank.drive(30,-30)
            while colorRight.rgb()[1] > highest_refrection_of_Black: # spin turn "right" until left color sensor finds black or green
                pass
            tank.drive_for_degrees(30,-30,110) # spin turn "right" 110 deg to be over line
            motorLeft.brake()
            motorRight.brake()

def isGreen(direction):
    if direction == "l":
        rgb = colorLeft.rgb()
    else:
        rgb = colorRight.rgb()
    hsv = changeRGBtoHSV(rgb)
    return (120 < hsv[0] < 160 and hsv[1] > 60 and hsv[2] > 20)

def u_turn():
    tank.drive_for_degrees(30,30,230) # go down 230 deg
    tank.drive_for_degrees(30,-30,240) # spin turn "right" 240 deg reqired to be optimized
    tank.drive(30,-30)
    while colorRight.rgb()[1] > highest_refrection_of_Black: # spin turn "right" until left color sensor finds black or green
        pass
    tank.drive_for_degrees(30,-30,110) # spin turn "right" 110 deg to be over line
    motorLeft.brake()
    motorRight.brake()
# ============================================================
def main():
    while 1:
        # init values
        Kp = 2.2
        Ki = 0.1
        Kd = 0.8
        last_error = 0
        error = 0
        basic_speed = 25
        start_time = timer.time()
        print("start")
        # Get ready!!
        ev3.speaker.beep()

        # wait until any button is pressed
        while not any(ev3.buttons.pressed()):
            pass
        while any(ev3.buttons.pressed()):
            pass
        
        # ここでESPとPicoにスタート信号を送る

        while not any(ev3.buttons.pressed()):
            rgb_left = colorLeft.rgb()
            rgb_right = colorRight.rgb()
            error = rgb_left[1] - rgb_right[1]
            u = Kp * error + Ki * (error + last_error) + Kd * (error - last_error)
            motorLeft.run(powertodegs(basic_speed + u))
            motorRight.run(powertodegs(basic_speed - u))
            hsv_left = changeRGBtoHSV(rgb_left)
            if 120 < hsv_left[0] < 160 and hsv_left[1] > 60 and hsv_left[2] > 20:
                print("Left sensor is over green")
            # print("left hsv:  "+str(hsv_left[0])+", "+str(hsv_left[1])+", "+str(hsv_left[2]))
            # print("left rgb:  "+str(rgb_left[0])+", "+str(rgb_left[1])+", "+str(rgb_left[2]))
            # wait(100)

            hsv_right = changeRGBtoHSV(rgb_right)
            if 120 < hsv_right[0] < 160 and hsv_right[1] > 60 and hsv_right[2] > 20:
                print("Right sensor is over green")
            # print("right hsv: "+str(hsv_right[0])+", "+str(hsv_right[1])+", "+str(hsv_right[2]))
            # print("right rgb: "+str(rgb_right[0])+", "+str(rgb_right[1])+", "+str(rgb_right[2]))
            # wait(100)
            wait(20)
        
        motorLeft.brake()
        motorRight.brake()

        # ここでESPとPicoにストップ&リセット信号を送る

main()