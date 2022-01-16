#定義
Kp = 1.5
Ki = 0.5
Kd = 1.3
individual_difference = 0 #cs2-cs3
#ここまでは事前にやっとく
errors = [0,0,0,0,0]
SYAKKE = DriveBase(left_motor, right_motor, wheel_diameter=tmp, axle_track=tmp)

def pid_control(base_power):
    error = CS2.refrection() - CS3.refrection() - individual_difference
    #偏差の累積を操作
    errors.append(error)
    del errors[0]

    u = Kp * error + Ki * sum(errors) + Kd * (error - errors[-2]) #操作量Kp*e+Ki∫e*dt+Kd*dt

    SYAKKE.drive(base_power, u)