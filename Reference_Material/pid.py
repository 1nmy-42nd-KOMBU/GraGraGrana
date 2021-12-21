from ev3dev2.motor import OUTPUT_B, OUTPUT_C, LargeMotor, MoveSteering, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
import time

touch = TouchSensor()
color = ColorSensor()
lm_b = LargeMotor(OUTPUT_B)
lm_c = LargeMotor(OUTPUT_C)
stm = MoveSteering(OUTPUT_B, OUTPUT_C)


#カラーセンサーによる白黒の反射光取得
def get_bw():
    global black, white
    print('push for white')
    while not touch.is_pressed:
        pass
    white = color.reflected_light_intensity
    print('white intensity: {}'.format(white))
    while touch.is_pressed:
        pass

    print('push for black')
    while not touch.is_pressed:
        pass
    black = color.reflected_light_intensity
    print('black intensity: {}'.format(black))
    while touch.is_pressed:
        pass
    return black, white



#PID走行をする関数
def pid_run(kp_value, ki_value, kd_value, b, w):
    global last_error
    pid_value = 0
    midpoint = (b + w) / 2
    last_error = 0
    cs = color.reflected_light_intensity
    error = cs - midpoint

    pid_value = kp_value * error + ki_value * (error + last_error) + kd_value * (error - last_error)
    stm.on(pid_value, SpeedPercent(20))

    last_error = error



def main():
    get_bw()
    while not touch.is_pressed:
        pid_run(0.5, 0.3, 0.5, black, white)
    lm_b.stop(stop_action='brake')
    lm_c.stop(stop_action='brake')



if __name__ == '__main__':
    main()
