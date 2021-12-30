def line_trace():
    while True:
        #silber
        run=1

    class Turn:

        def turn_black(self,direction): # direction 1なら左、-1なら右
            tank.angle(30,30,90) # 黒の上に乗る
            # 十字路のジャッジ
            if CS1.refrection() < black_hightest_refrection and CS4.refrection() < black_hightest_refrection:
                limit = time.time() + 2.0
                while limit >= time.time():
                    pid_control() # 黒線上を脱する

            if direction == 1: # left
                tank.angle(40,20,360) # 軌道修正&位置調整
                while (CS1.color() != "Color.BLACK") or (CS3.color() != "Color.BLACK" or CS4.color() != "Color.BLACK"):
                    tank.speed(-30,30)
                if CS1.color() == "Color.BLACK": # 90
                    while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                        tank.speed(-30,30)
                else:
                    while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                        tank.speed(30,-30)

            else: # right
                tank.angle(20,40,360)
                while (CS4.color() != "Color.BLACK") or (CS2.color() != "Color.BLACK" or CS1.color() != "Color.BLACK"):
                    tank.speed(30,-30)
                if CS4.color() == "Color.BLACK": # 90
                    while not CS3.color() == "Color.WHITE" and CS2.color() == "Color.WHITE":
                        tank.speed(30,-30)
                else:
                    while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                        tank.speed(-30,30)

        def turn_green(self):
            tank.angle(30,30,90) # go a bit
            if CS2.color() == "Color.GREEN" and CS3.color() == "Color.GREEN":
                while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                    tank.speed(30,-30)
                while not CS3.color() == "Color.BLACK":
                    tank.speed(30,-30)
                while not CS2.color() == "Color.WHITE" and CS3.color() == "Color.WHITE":
                    tank.speed(30,-30)

            elif CS2.color() == "Color.BLACK": # left
                tank.angle(30,30,360) # 線の中央に乗る
                while not CS4.color() == "Color.BLACK":
                    tank.speed(-30,30)
                while not CS2.color() == "Color.BLACK":
                    tank.speed(-30,30)
                while not CS2.color() == "Color.BLACK" and CS3.color() == "Color.BLACK":
                    tank.speed(-30,30)
            else: #right
                tank.angle(30,30,360) # 線の中央に乗る
                while not CS1.color() == "Color.BLACK":
                    tank.speed(30,-30)
                while not CS3.color() == "Color.BLACK":
                    tank.speed(30,-30)
                while not CS2.color() == "Color.BLACK" and CS3.color() == "Color.BLACK":
                    tank.speed(30,-30)

        def avoid():
            run=1


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
        
        def angle(self, left_speed, right_speed,angle):# 要検討
            left_motor.run_angle(left_speed, angle, stop_type = Stop.BLAKE, wait_type = False)
            right_motor.run_angle(right_speed, angle, stop_type = Stop.BRAKE, wait_type = False)
            if not wait_type:
                wait(run_time)

        def time(self, left_speed, right_speed, run_time, stop_type = Stop.BLAKE, wait_type = False):
            left_motor.run_time(left_speed, run_time, stop_type,wait_type)
            right_motor.run_time(right_speed, run_time, stop_type, wait_type)
            if not wait_type:
                wait(run_time)
