#定義
Kp = 1.5
Ki = 0.5
Kd = 1.3
individual_difference = 0 #cs2-cs3
#ここまでは事前にやっとく
last_error = 0
errors = [0,0,0,0,0]

def pid_control(base_power):
    error = CS2.refrection() - CS3.refrection() - individual_difference
    errors.append(error)
    del errors[0]

    u = Kp * error + Ki * sum(errors) + Kd * (error - last_error)

    left_motor.run(base_power + u)
    right_motor.run(base_power - u)

    last_error = error