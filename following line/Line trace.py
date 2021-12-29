# 定義
black_hightest_refrection = 10
silber_lowest_refrection = 40
Kp = 1.5
Ki = 0.5
Kd = 1.3
individual_difference = 0 #cs2-cs3　個体差

turn90 = 
turn180 = 
#ここまでは事前にやっとく
errors = [0,0,0,0,0]

def line_trace():
    while True:
        
    class Turn:

        def turn_black_90(self,direction): #direction 1なら左、-1なら右
            tank.angle(30,30,90) # 黒の上に乗る
            if CS1.refrection() < black_hightest_refrection and CS4.refrection() < black_hightest_refrection:
                #黒線上を脱する
                limit = time.time() + 2.5
                while limit <= time.time():

            while 
            
            tank.angle(25,25,360)

        def turn_green(self):

class Motor:

    def pid_control(self,base_power):
        error = CS2.refrection() - CS3.refrection() - individual_difference
        #偏差の累積を操作
        errors.append(error)
        del errors[0]
        u = Kp * error + Ki * sum(errors) + Kd * (error - errors[-2]) #操作量Kp*e+Ki∫e*dt+Kd*dt

        left_motor.run(base_power + u)
        right_motor.run(base_power - u)
    
    class Tank:

        def speed(self, left_speed, right_speed):
            left_motor.run(left_speed)
            right_motor.run(right_speed)
        
        def angle(self, left_speed, right_speed,angle):
            left_motor.run_angle(left_speed, angle, stop_type = Stop.BLAKE, wait_type = False)
            right_motor.run_angle(right_speed, angle, stop_type = Stop.BRAKE, wait_type = False)
            if not wait_type:
                wait(run_time)

        def time(self, left_speed, right_speed, run_time, stop_type = Stop.BLAKE, wait_type = False):
            left_motor.run_time(left_speed, run_time, stop_type,wait_type)
            right_motor.run_time(right_speed, run_time, stop_type, wait_type)
            if not wait_type:
                wait(run_time)
