#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.iodevices import UARTDevice

# 設定コーナー ====================================================
ev3 = EV3Brick()
timer = StopWatch()

# センサーとモーターをEV3と接続していくぅ↑---------------------------
# 失敗した時はその内容を吐露しながら停止するよ

# カラーセンサを接続
try:
    colorLeft = ColorSensor(Port.S4)
    colorRight = ColorSensor(Port.S3)
except OSError as oserror:
    while True:
        ev3.speaker.say("color")
        wait(1000)

# Lモーターを接続
try:
    motorLeft = Motor(Port.D)
    motorRight = Motor(Port.A)
except OSError as oserror:
    while True:
        ev3.speaker.say("motor")
        wait(1000)

# Mモーターを接続
try:
    arm_bucket = Motor(Port.B)
    arm_rotate = Motor(Port.C)
except OSError as oserror:
    while True:
        ev3.speaker.say("m motor")
        wait(1000)

# Raspberry Pi Picoを接続
try:
    pico = UARTDevice(Port.S2, 9600,100)
except OSError as oserror:
    while True:
        ev3.speaker.say("pico")
        wait(1000)
else:
    pico.clear()

# ESP32を接続
try:
    esp = UARTDevice(Port.S1, 115200,100)
except OSError as oserror:
    while True:
        ev3.speaker.say("E S P 32")
        wait(1000)
else:
    esp.clear() # 受信バッファをクリア
# ----------------------------------------------------------------
highest_refrection_of_Black = const(15)
basic_speed = 30
# 設定コーナー終わり ===============================================

# 足回りのクラス ===================================================
class Tank:
    """
    ・モーター関連のクラス
    ・Mindstormsのタンクとステアリングの機能+アルファ
    ・このクラスは実行速度を一切考慮していないので注意
    ・回転方向の指定はパワーですることを想定している
    """
    Kp = 2.2
    Ki = 0.1
    Kd = 0.8
    last_error = 0
    error = 0

    def drive(self, left_speed, right_speed):
        """
        機体を進める
        ストップするまで止まらんで
        """
        motorLeft.run(powertodegs(left_speed))
        motorRight.run(powertodegs(right_speed))

    def drive_for_seconds(self, left_speed, right_speed, time, stop_type = "brake", wait=True):
        """
        指定されたスピード(%)と時間(ms)で進む
        こいつにだけwait機能がある 他は非同期処理を習得したら考える
        """
        motorLeft.run_time(powertodegs(left_speed), time, stop_type, False)
        motorRight.run_time(powertodegs(right_speed), time, stop_type, False)
        if wait:
            wait(time)
        
        self.stop(stop_type)

    def drive_for_degrees(self, left_speed, right_speed, degrees, stop_type = "brake"):
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        motorLeft.run(powertodegs(left_speed))
        motorRight.run(powertodegs(right_speed))
        while abs(left_angle - motorLeft.angle()) <= degrees and abs(right_angle - motorRight.angle()) <= degrees:
            pass
        self.stop(stop_type)

    def drive_for_rotations(self, left_speed, right_speed, rotations, stop_type = "brake"):
        self.drive_for_degrees(left_speed,right_speed,rotations* 360,stop_type)

    def steering(self,speed,steering):
        """
        ステアリング機能
        デバッグしてないYO
        """
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

    def steering_for_seconds(self,speed,steering,seconds,stop_type = "brake"):
        if seconds <= 0:
            raise ValueError
        time_run = timer.time() + seconds
        while timer.time() <= time_run:
            self.steering(speed,steering)
        self.stop(stop_type)

    def steeing_for_degrees(self,power,steering,degrees,stop_type = "brake"):
        if degrees < 0:
            degrees *= -1
            steering *= -1
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        self.steering(power,steering)
        while not abs(left_angle - motorLeft.angle()) > degrees or abs(right_angle - motorRight.angle()) > degrees:
            pass
        self.stop(stop_type)

    def steering_for_rotations(self,power,steering,rotations,stop_type = "brake"):
        self.steeing_for_degrees(power,steering,rotations * 360,stop_type)

    def stop_option(self,stop_type):
        """
        単一モーターを標準関数でストップするときのオプションを返す
        他の関数と同じ感覚でストップできるように
        """
        if stop_type == "stop":
            return Stop.COAST
        elif stop_type == "brake":
            return Stop.BRAKE
        else:
            return Stop.HOLD

    def stop(self, stop_type):
        """ 止まる """
        if stop_type == "stop":
            motorLeft.stop()
            motorRight.stop()
        elif stop_type == "brake":
            motorLeft.brake()
            motorRight.brake()
        else:
            motorLeft.hold()
            motorRight.hold()
    
    def drive_pid_for_degrees(self,speed,degrees,stop_type):
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        while abs(left_angle - motorLeft.angle()) <= degrees and abs(right_angle - motorRight.angle()) <= degrees:
            rgb_left = colorLeft.rgb() # 左のRGBをゲットだぜ
            rgb_right = colorRight.rgb() # 右のRGｂをゲットだぜ

            # PID制御
            Tank.error = rgb_left[1] - rgb_right[1] # 偏差をゲットだぜ
            u = Tank.Kp * Tank.error + Tank.Ki * (Tank.error + Tank.last_error) + Tank.Kd * (Tank.error - Tank.last_error) # 操作量をゲットだぜ
            motorLeft.run(powertodegs(speed + u)) # 動かす
            motorRight.run(powertodegs(speed - u))
        self.stop(stop_type)

    def turn_right(self,degree):
        """degreeには180,90のどれか"""
        esp.clear()
        print(degree)
        esp.clear()
        esp.write(degree.to_bytes(1,'big'))
        wait(100)
        hoge = esp.read(1) # read 18 or 9 

        tank.drive(30,-30)
        esp.clear()
        while esp.waiting() == 0:
            print("turning")
            wait(100)
        motorLeft.brake()
        motorRight.brake()
        hoge = esp.read(1) # read 180 or 90
        esp.write(degree.to_bytes(1,'big'))
        esp.clear()

    def turn_left(self,degree):
        """degreeには180,27のどれか"""
        esp.clear()
        print(degree)
        esp.clear()
        esp.write(degree.to_bytes(1,'big'))
        wait(100)
        hoge = esp.read(1) # read 18 or 27

        tank.drive(-30,30)
        esp.clear()
        while esp.waiting() == 0:
            print("turning")
            wait(100)
        motorLeft.brake()
        motorRight.brake()
        hoge = esp.read(1) # read 180 or 270
        esp.write(degree.to_bytes(1,'big'))
        esp.clear()

tank = Tank()

# アーム関連のクラス ====================================================
class Arm:
    """アーム関係の動作"""
    def open_arm(self):
        """アームを展開"""
        arm_rotate.run_angle(powertodegs(40),250,Stop.COAST,True) 
    def close_arm(self):
        """アームをしまう"""
        arm_rotate.run_angle(powertodegs(-40),250,Stop.BRAKE,True)

    def open_bucket(self):
        """回るやつを開放状態にする"""
        arm_bucket.run_angle(powertodegs(-40),150,Stop.BRAKE,True)

    def close_bucket(self):
        """回るやつを閉じる"""
        arm_bucket.run_angle(powertodegs(40),150,Stop.BRAKE,True)

arm = Arm()

# ライントーレスのグッズ ==================================================
# ただの計算だからネイティブコードエミッタで高速化してる
@micropython.native
def powertodegs(power):
    """スピード(%)をdeg/sに変換する"""
    return 950 * power /100

@micropython.native
def changeRGBtoHSV(rgb):
    """
    RGBをHSVに変換して返す(タプル型)
    色相(H)は 0~360
    彩度(S)は 0~100
    明度(V)は 0~255 の範囲
    """
    # RGBのしきい値を0~100から0~255に修正 Blueの値が異様に大きい問題があるので÷2して実際に見える色に寄せている
    rgb0_255 = rgb[0] * 255 / 100, rgb[1] * 255 / 100, rgb[2] * 255 / 200
    maxRGB, minRGB = max(rgb0_255), min(rgb0_255)
    diff = maxRGB - minRGB

    # Hue
    if maxRGB == minRGB : hue = 0
    elif maxRGB == rgb0_255[0] : hue = 60 * ((rgb0_255[1]-rgb0_255[2])/diff)
    elif maxRGB == rgb0_255[1] : hue = 60 * ((rgb0_255[2]-rgb0_255[0])/diff) + 120
    elif maxRGB == rgb0_255[2] : hue = 60 * ((rgb0_255[0]-rgb0_255[1])/diff) + 240
    if hue < 0 : hue += 360

    # Saturation
    if maxRGB != 0:
        saturation = diff / maxRGB * 100
    else:
        saturation = 0

    # Value(Brightness)
    value = maxRGB

    return hue,saturation,value

# 緑マーカー ============================================================
def onGreenMarker(direction):
    if direction == "l": # 左のカラーセンサーが緑を見つけた
        # 50°進みつつ右にも緑がないか確認 結果はisRightGreenに格納
        start_angle_deg = motorRight.angle()
        isRightGreen = False
        motorLeft.run(powertodegs(basic_speed))
        motorRight.run(powertodegs(basic_speed))
        while abs(motorRight.angle() - start_angle_deg) <= 50:
            if isGreen('r'):
                isRightGreen = True
        motorLeft.brake()
        motorRight.brake()

        if isRightGreen: # 反対にもマーカーがあったらUターン
            u_turn()
        else:
            # ほんまもんの左折
            print_pico(111)
            print('turn left')
            tank.drive_for_degrees(30,30,180) # 180°前に進んで機体を交差点の中心に持ってく
            tank.drive_for_degrees(-30,30,160,"stop") # 180°回転して左のカラーセンサー下にラインがないようにする
            tank.drive(-30,30)
            while colorLeft.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を左のセンサが見つけるまで回る
                pass
            tank.drive_for_degrees(-30,30,110) # 機体をラインに沿わせる
            motorLeft.brake()
            motorRight.brake()
            tank.drive_for_degrees(30,30,50) # 緑マーカーかぶりを回避したい
    else: # # 左のカラーセンサーが緑を見つけた(消去法的にね) -------------------------------------------------------------------
        # 50°進みつつ左にも緑がないか確認 結果はisRightGreenに格納
        start_angle_deg = motorLeft.angle()
        isLeftGreen = False

        motorLeft.run(powertodegs(30))
        motorRight.run(powertodegs(30))

        while abs(motorLeft.angle() - start_angle_deg) <= 50:
            if isGreen('l'):
                isLeftGreen = True
        motorLeft.brake()
        motorRight.brake()

        if isLeftGreen: # 反対にもマーカーがあったらUターン
            u_turn()
        else:
            print('turn right')
            print_pico(112)
            tank.drive_for_degrees(30,30,180) # 180°前に進んで機体を交差点の中心に持ってく
            tank.drive_for_degrees(30,-30,180,"stop") # 180°回転して左のカラーセンサー下にラインがないようにする
            tank.drive(30,-30)
            while colorRight.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を右のセンサが見つけるまで回る
                pass
            tank.drive_for_degrees(30,-30,110) # 機体をラインに沿わせる
            motorLeft.brake() # 回転方向の運動を止める
            motorRight.brake()
            tank.drive_for_degrees(30,30,50) # 緑マーカーかぶりを回避したい
    print_pico(100)

def isGreen(direction):
    """
    onGreenMarkerの付属品
    directionで渡された方向のカラーセンサが緑を見ているか判定
    """
    # RGBを取得して
    if direction == "l":
        rgb = colorLeft.rgb()
    else:
        rgb = colorRight.rgb()
    # HSVに変換して
    hsv = changeRGBtoHSV(rgb)
    # 緑かどうかの真偽値を返す
    return (140 < hsv[0] < 180 and hsv[1] > 50 and hsv[2] > 20)

def u_turn():
    # 柱にぶつからん為にタイルの中心で回転しようとしたら回転軸がぶれて、センサがいい感じにラインの上に乗ってくんない
    # もう考えたくないからジャイロで180°ぶん回すんじゃー
    print_pico(113)
    #tank.drive_for_degrees(30,30,45) # 回転の中心の調整
    gyro_range11(-30,30,180)
    tank.drive_for_degrees(30,30,100,"stop")

# 黒線①============================================================

def black(direction):
    """左or右のラインセンサが黒を感知したときの"""
    if direction == "l":
        # 左のラインセンサが黒を感知
        # 50°進みつつ右にも黒がないか確認する
        # この時点ではトの字、十字、左折が考えられる
        print("l")
        isRightBlack = False
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        motorLeft.run(powertodegs(basic_speed))
        motorRight.run(powertodegs(basic_speed))
        while abs(left_angle - motorLeft.angle()) <= 50 and abs(right_angle - motorRight.angle()) <= 50:
            line_statue = UARTwithESP32_LineMode(10,4)[0]
            if line_statue == 2 or line_statue == 3:
                isRightBlack = True
            wait(10)
        
        if isRightBlack:
            print_pico(123)
            # 無印の十字路 無視して突き進むんや
            tank.drive_for_degrees(30,30,120,"stop")
            tank.drive_pid_for_degrees(30,90,"stop")
        else:
            tank.drive_for_degrees(30,30,110) # 180°前に進んで機体を交差点の中心に持ってく
            esp.clear()
            central_line_sensor = UARTwithESP32_LineMode(10,4)[2]
            if central_line_sensor == 0 or colorLeft.rgb()[1] <= highest_refrection_of_Black or colorRight.rgb()[1] <= highest_refrection_of_Black:
                # 真ん中ら辺のセンサが黒を読んでるからトの字やねぇ もはやPicoに表示する瞬間も与えない
                pass # 無視!
                print("t")
            else:
                # 本当の左折
                print("l")
                print_pico(121)
                tank.drive_for_degrees(-30,30,160,"stop") # 180°回転して左のカラーセンサー下にラインがないようにする
                tank.drive(-30,30)
                while colorLeft.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を左のセンサが見つけるまで回る
                    pass
                tank.drive_for_degrees(-30,30,180) # 機体をラインに沿わせる
                motorLeft.brake()
                motorRight.brake()
                tank.drive_for_degrees(30,30,50,"stop") # ラインかぶりを回避したい
    elif direction == "r":
        # 右のラインセンサが黒を感知
        # 50°進みつつ左にも黒がないか確認する
        # この時点ではトの字、十字、右折が考えられる
        isLeftBlack = False
        left_angle = motorLeft.angle()
        right_angle = motorRight.angle()
        motorLeft.run(powertodegs(basic_speed))
        motorRight.run(powertodegs(basic_speed))
        while abs(left_angle - motorLeft.angle()) <= 50 and abs(right_angle - motorRight.angle()) <= 50:
            line_statue = UARTwithESP32_LineMode(10,4)[0]
            if line_statue == 1 or line_statue == 3:
                isLeftBlack = True
            wait(10)
        
        if isLeftBlack:
            print_pico(123)
            # 無印の十字路 無視して突き進むんや
            tank.drive_for_degrees(30,30,120,"stop")
            tank.drive_pid_for_degrees(30,90,"stop")
        else:
            tank.drive_for_degrees(30,30,100) # 180°前に進んで機体を交差点の中心に持ってく
            esp.clear()
            central_line_sensor = UARTwithESP32_LineMode(10,4)[2]
            if central_line_sensor == 0 or colorLeft.rgb()[1] <= highest_refrection_of_Black or colorRight.rgb()[1] <= highest_refrection_of_Black:
                # 真ん中ら辺のセンサが黒を読んでるからトの字やねぇ
                pass # 無視!
            else:
                # 本当の右折
                print_pico(122)
                tank.drive_for_degrees(30,-30,160,"stop") # 180°回転して左のカラーセンサー下にラインがないようにする
                tank.drive(30,-30)
                while colorRight.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を右のセンサが見つけるまで回る
                    pass
                tank.drive_for_degrees(30,-30,180) # 機体をラインに沿わせる
                motorLeft.brake() # 回転方向の運動を止める
                motorRight.brake()
                tank.drive_for_degrees(30,30,50) # ラインかぶりを回避したい
    else: #"both"が来る
        # 無印の十字路 無視して突き進むんや
        print_pico(123)
        tank.drive_for_degrees(30,30,120,"stop")
        tank.drive_pid_for_degrees(30,90,"stop")
    print_pico(100)

# ラインを見失った ============================================================

def lost_line():
    """
    中央のラインセンサが黒を見失ったとき
    -ラインからずれた
    -キャップ
    の2つが考えられるよね
    """
    print("lost line")
    tank.stop("brake")
    # その場の左右にラインがないかを確認
    if colorLeft.rgb()[0] <= highest_refrection_of_Black:
        # 脱線 右にずれている 左に回れ
        recover_to_line("l")
    elif colorRight.rgb()[0] <= highest_refrection_of_Black:
        # 脱線 左にずれている 右に回れ
        recover_to_line("r")
    else:
        tank.drive_for_degrees(30,30,50) # 外れてるならちゃんと外れてもらう
        tank.drive_for_degrees(-42,30,100) # 左に回転
        esp.clear()
        line_sensors = UARTwithESP32_LineMode(10,4)
        if line_sensors[2] == 0:
            pass
        elif line_sensors[0] == 1:
            # 右にずれている 左に回れ
            recover_to_line("l")
        elif line_sensors[0] == 2:
            # 左にずれいている 右に回れ
            recover_to_line("r")
        else:
            tank.drive_for_degrees(42,-30,200) # 右に回転
            esp.clear()
            line_sensors = UARTwithESP32_LineMode(10,4)
            if line_sensors[2] == 0:
                pass
            elif line_sensors[0] == 1:
                # 右にずれている 左に回れ
                recover_to_line("l")
            elif line_sensors[0] == 2:
                # 左にずれいている 右に回れ
                recover_to_line("r")
            else:
                # ギャップ
                tank.drive_for_degrees(-35,30,110) # 元の位置に回転
                esp.clear()
                tank.drive(30,30)
                while 1:
                    line_sensors = UARTwithESP32_LineMode(10,4)
                    if line_sensors[0] == 1 or colorLeft.rgb()[1] <= highest_refrection_of_Black:
                        # 右にずれている 左に回れ
                        tank.drive_for_degrees(30,30,50)
                        recover_to_line("l")
                        print(str(line_sensors[0])+" "+str(colorLeft.rgb()[1]))
                        break
                    elif line_sensors[0] == 2 or colorRight.rgb()[1] <= highest_refrection_of_Black:
                        # 左にずれいている 右に回れ
                        tank.drive_for_degrees(30,30,50)
                        recover_to_line("r")
                        print(str(line_sensors[0])+" "+str(colorRight.rgb()[1]))
                        break
                    elif line_sensors[2] == 0:
                        print("central"+str(line_sensors[2]))
                        # ちょうど真ん中のセンサに乗った
                        break
                    wait(10)

def recover_to_line(direction):
    esp.clear()
    wait(10)
    if direction == "l":
        print("l")
        central_line_sensor = 0
        tank.drive(-30,30)
        while central_line_sensor != 0:
            line_sensors = UARTwithESP32_LineMode(10,4)
            central_line_sensor = line_sensors[2]
            wait(10)
        tank.drive_for_degrees(-30,30,50)
    else: # == "r"
        print("r")
        central_line_sensor = 0
        tank.drive(30,-30)
        while central_line_sensor != 0:
            UARTwithESP32_LineMode(10,4)
            central_line_sensor = line_sensors[2]
            wait(10)
        tank.drive_for_degrees(30,-30,50)

# レスキューキット============================================================

def rescuekit():
    """レスキューキット検知後の動作"""
    tank.drive_for_degrees(-1*basic_speed,-1*basic_speed,350) # 一歩下がる
    arm_rotate.run_angle(powertodegs(40),250,Stop.COAST,True) # アームを下ろす
    tank.drive_for_degrees(basic_speed,basic_speed,350) # 前に進む
    arm_bucket.run_angle(powertodegs(40),50,Stop.COAST,True) # バケットを少し回す(初速を付けるため)
    arm_bucket.run(powertodegs(40)) # バケットを回す
    while abs(arm_bucket.speed()) >= 5: # パワーが5以下(つまりこれ以上回せなくなる)になるまで回し続ける
        pass
    arm_bucket.brake() # がっちり固定
    arm_rotate.run_angle(powertodegs(-40),250,Stop.BRAKE,True) # アームを上げる
    arm_bucket.run_angle(powertodegs(-40),30,Stop.COAST,True) # レスキューキット開放
    arm_bucket.run_angle(powertodegs(40),200,Stop.BRAKE,True) # 元の位置に戻す

# ============================================================

def avoid():
    tank.stop("brake")
    esp.clear()
    tank.drive_for_degrees(-50,-50,180)
    first_read = UARTwithESP32_LineMode(11,4)
    # 方向の指定
    if first_read[0] == 1: # 左に壁があったら
        direction_to_turn = "r"
    else:
        direction_to_turn = "l"
    print(direction_to_turn)

    if direction_to_turn == "l": # 左回り
        gyro_range11(-30,30,270)
        tank.drive_for_degrees(30,30,150,"stop")
        distance_go("r","on")
        tank.drive_for_degrees(30,30,80,"stop") # チャタリング防止
        cause = distance_go("r","off")
        tank.stop("brake")
        if cause == "t": # タッチセンサが反応した時
            direction_to_turn = "r"
            tank.stop("brake")
            gyro_range11(-30,30,180)
            distance_go("l","on")
            tank.drive_for_degrees(30,30,80,"stop") # チャタリング防止
            distance_go("l","off")
    else: # 右回り
        gyro_range11(30,-30,90)
        tank.drive_for_degrees(30,30,150,"stop")
        distance_go("l","on")
        tank.drive_for_degrees(30,30,80,"stop") # チャタリング防止
        cause = distance_go("l","off")
        print(cause)
        if cause == "t": # タッチセンサが反応した時
            direction_to_turn = "l"
            tank.stop("brake")
            gyro_range11(-30,30,180)
            distance_go("r","on")
            tank.drive_for_degrees(30,30,80,"stop") # チャタリング防止
            distance_go("r","off")

    tank.drive_for_degrees(30,30,80) # チャタリング防止
    # クソコードで草
    if direction_to_turn =="l":
        tank.drive_for_degrees(30,30,80,"stop") # 位置調整
        gyro_range11(30,-30,90) # 並行1
        cause = distance_go("r","on",True)
        # --------------------------------------------------
        if not cause == "l":
            if chattering_prevention("r",True):
                cause == "l"
            else:
                cause = distance_go("r","off",True) # 並行2
            # --------------------------------------------------
            if not cause == "l":
                if chattering_prevention("r",True):
                    cause == "l"
                else:
                    gyro_range11(30,-30,90)
                    tank.drive_for_degrees(30,30,80,"stop") # 位置調整
                    cause = distance_go("r","on",True) # 垂直1
                # --------------------------------------------------
                if not cause == "l":
                    if chattering_prevention("r",True):
                        cause == "l"
                    else:
                        cause = distance_go("r","off",True) # 垂直2
                    # --------------------------------------------------
                    if not cause == "l":
                        if chattering_prevention("r",True):
                            cause == "l"
                        else:
                            gyro_range11(30,-30,90)
                            tank.drive_for_degrees(30,30,80,"stop") # 位置調整
                            tank.drive(30,30)
                            while colorLeft.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を右のセンサが見つけるまで回る
                                pass
        # ラインに復帰
        tank.drive_for_degrees(40,40,240)
        tank.drive(-30,30)
        while colorLeft.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を右のセンサが見つけるまで回る
            pass
        tank.drive_for_degrees(-30,30,180) # 機体をラインに沿わせる
        motorLeft.brake() # 回転方向の運動を止める
        motorRight.brake()
    else: # right
        gyro_range11(-30,30,270) # 並行1
        tank.drive_for_degrees(30,30,80,"stop") # 位置調整
        cause = distance_go("l","on",True)
        # --------------------------------------------------
        if not cause == "l":
            if chattering_prevention("r",True):
                cause == "l"
            else:
                cause = distance_go("l","off",True) # 並行2
            # --------------------------------------------------
            if not cause == "l":
                if chattering_prevention("r",True):
                    cause == "l"
                else:
                    gyro_range11(-30,30,270)
                    tank.drive_for_degrees(30,30,80,"stop") # 位置調整
                    cause = distance_go("l","on",True) # 垂直1
                # --------------------------------------------------
                if not cause == "l":
                    if chattering_prevention("r",True):
                        cause == "l"
                    else:
                        cause = distance_go("l","off",True) # 垂直2
                    # --------------------------------------------------
                    if not cause == "l":
                        tank.drive_for_degrees(30,30,80,"stop") # 位置調整
                        gyro_range11(-30,30,270)
                        tank.drive(30,30)
                        while colorLeft.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を右のセンサが見つけるまで回る
                            pass
        # ラインに復帰
        tank.drive_for_degrees(40,40,240)
        tank.drive(30,-30)
        while colorRight.rgb()[1] > highest_refrection_of_Black: # 緑ないし黒を右のセンサが見つけるまで回る
            pass
        tank.drive_for_degrees(30,-30,180) # 機体をラインに沿わせる
        motorLeft.brake() # 回転方向の運動を止める
        motorRight.brake()

# --------------------------------------------------------------------------------
def chattering_prevention(direction, line_search=False,degrees=80):
    isOverBlack = False
    left_angle = motorLeft.angle()
    right_angle = motorRight.angle()
    motorLeft.run(powertodegs(basic_speed))
    motorRight.run(powertodegs(basic_speed))
    while abs(left_angle - motorLeft.angle()) <= degrees and abs(right_angle - motorRight.angle()) <= drive_pid_for_degrees:
        if line_search:
            if direction == "l" and colorLeft.rgb()[1] < highest_refrection_of_Black:
                isOverBlack = True
            elif colorRight.rgb()[1] < highest_refrection_of_Black:
                isOverBlack = True
    return isOverBlack
# --------------------------------------------------------------------------------
def distance_go(direction,turn,line=False):
    if direction == "l":
        if turn == "on": # 左があるまで進む
            tank.drive(30,30)
            esp.clear()
            while 1: # 左にあるようになるまで
                whatread = UARTwithESP32_LineMode(11,4)
                if whatread[0] == 1 or whatread[0] == 3:
                    cause = "d"
                    print(whatread)
                    break
                if whatread[1] != 0:
                    cause = "t"
                    print(whatread)
                    break
                if line and (whatread[2] == 2 or whatread[2] == 3):
                    cause = "l"
                    print(whatread)
                    break
                wait(5)
        else:
            # 左がない,まで進む
            tank.drive(30,30)
            esp.clear()
            while 1: # 左にないようになるまで
                whatread = UARTwithESP32_LineMode(11,4)
                if whatread[0] == 0 or whatread[0] == 2:
                    cause = "d"
                    break
                if whatread[1] != 0:
                    cause = "t"
                    break
                if line and (whatread[2] == 2 or whatread[2] == 3):
                    cause = "l"
                    break
                wait(5)
    else:
        if turn == "on": # 右があるまで進む
            tank.drive(30,30)
            esp.clear()
            while 1: # 右にあるようになるまで
                whatread = UARTwithESP32_LineMode(11,4)
                if whatread[0] == 2 or whatread[0] == 3:
                    cause = "d"
                    break
                if whatread[1] != 0:
                    cause = "t"
                    break
                if line and (whatread[2] == 1 or whatread[2] == 3):
                    cause = "l"
                    break
                wait(5)
        else:
            # 右がないまで進む
            tank.drive(30,30)
            esp.clear()
            while 1: # 右にないようになるまで
                whatread = UARTwithESP32_LineMode(11,4)
                if whatread[0] == 0 or whatread[0] == 1:
                    cause = "d"
                    break
                if whatread[1] != 0:
                    cause = "t"
                    break
                if line and (whatread[2] == 1 or whatread[2] == 3):
                    cause = "l"
                    break
                wait(5)
    return cause

# ============================================================

def UARTwithESP32_LineMode(mode,numbyte):
    """"
    ライントレースしてるときのUART
    通常時のと障害物回避ので2つつかえる
    """
    esp.write(mode.to_bytes(1,'big'))
    error_count = 0
    while esp.waiting() < numbyte:
        if error_count > 10: # 10回以上失敗してたら一時停止
            motorLeft.brake()
            motorRight.brake()
            ev3.speaker.say("E S P U A R T")
        wait(10)
        print("error sub")
        esp.write(mode.to_bytes(1,'big'))
        error_count += 1
    whatread = esp.read(numbyte)
    return whatread

def gyro_range11(left_power,right_power,degree):
    esp.clear()
    whatread = UARTwithESP32_LineMode(11,4)
    start_angle = whatread[3]*2
    target_angle_range_min = start_angle + degree - 5; # 目標角の範囲の最小値
    target_angle_range_max = start_angle + degree + 5; # 目標角の範囲の最大値
    tank.drive(left_power,right_power)
    # 繰り上がりとかを考慮しつつ180度回転するのを待つ
    if (target_angle_range_max <360):
        # そのままでOK
        pass
        while (1):
            whatread = UARTwithESP32_LineMode(11,4)
            angle_now = whatread[3]*2
            if (target_angle_range_min <= angle_now and angle_now < target_angle_range_max):
                break
            wait(5)
    elif (target_angle_range_max >= 360 and target_angle_range_min < 360):
        target_angle_range_max -= 360
        while (1):
            whatread = UARTwithESP32_LineMode(11,4)
            angle_now = whatread[3]*2
            if ((target_angle_range_min <= angle_now and angle_now < 360) or (0 <= angle_now and angle_now <= target_angle_range_max)):
                break
            wait(5)
    elif (target_angle_range_min >= 360):
        target_angle_range_max -= 360
        target_angle_range_min -= 360
        while (1):
            whatread = UARTwithESP32_LineMode(11,4)
            angle_now = whatread[3]*2
            if (target_angle_range_min <= angle_now and angle_now < target_angle_range_max):
                break
            wait(5)
    tank.stop("brake")
    esp.clear()

# ============================================================

def print_pico(num):
    """Raspberry Pi Picoの7セグに3桁の数字を表示する"""
    number = num.to_bytes(2,'big')
    what_to_send =  bytearray([3,number[0],number[1]])
    pico.write(what_to_send) # この後の数字を

# ============================================================
def main():
    """メインループ"""
    while 1:
        # init values
        Kp = 2.2
        Ki = 0.1
        Kd = 0.8
        last_error = 0
        error = 0
        basic_speed = 25
        hill_statue = 0
        start_time = timer.time()
        # アームを固定
        arm_rotate.hold()
        print("start")
        # Get ready!!
        ev3.speaker.beep()

        # ボタンが押されるのを待つ
        while not any(ev3.buttons.pressed()):
            pass
        while any(ev3.buttons.pressed()):
            pass
        
        # ここでESPとPicoにスタート信号を送る
        print_pico(100)

        while not any(ev3.buttons.pressed()):
            rgb_left = colorLeft.rgb() # 左のRGBをゲットだぜ
            rgb_right = colorRight.rgb() # 右のRGｂをゲットだぜ

            # PID制御
            error = rgb_left[1] - rgb_right[1] # 偏差をゲットだぜ
            u = Kp * error + Ki * (error + last_error) + Kd * (error - last_error) # 操作量をゲットだぜ
            motorLeft.run(powertodegs(basic_speed + u)) # 動かす
            motorRight.run(powertodegs(basic_speed - u))
            
            # 左の緑判定
            hsv_left = changeRGBtoHSV(rgb_left) # HSVの値をゲット
            if 140 < hsv_left[0] < 180 and hsv_left[1] > 50 and hsv_left[2] > 20:
                print("Left sensor is over green")
                onGreenMarker("l")
                continue # ループの最初に戻る(通信の内容を更新)
            #print("left hsv:  "+str(hsv_left[0])+", "+str(hsv_left[1])+", "+str(hsv_left[2]))
            #print("left rgb:  "+str(rgb_left[0])+", "+str(rgb_left[1])+", "+str(rgb_left[2]))

            # 右の緑判定
            hsv_right = changeRGBtoHSV(rgb_right) # HSVの値をゲット
            if 140 < hsv_right[0] < 180 and hsv_right[1] > 50 and hsv_right[2] > 20:
                print("Right sensor is over green")
                onGreenMarker("r")
                continue # ループの最初に戻る(通信の内容を更新)
            #print("right hsv: "+str(hsv_right[0])+", "+str(hsv_right[1])+", "+str(hsv_right[2]))
            # print("right rgb: "+str(rgb_right[0])+", "+str(rgb_right[1])+", "+str(rgb_right[2]))
            
            # ESP32との通信
            esp.write((10).to_bytes(1,'big')) # ESP32に値のリクエスト(10)を送る
            error_count = 0
            while esp.waiting() < 4: # 受信バッファに4Byteたまるまで待つ
                if error_count > 10: # 10回以上失敗してたら一時停止
                    motorLeft.brake()
                    motorRight.brake()
                    ev3.speaker.say("E S P U A R T")
                wait(10) # 待てる最大の時間は10ms
                print("error main")
                esp.write((10).to_bytes(1,'big')) # リクエストの再送
                error_count += 1
            # 4Byte読み取る
            # whatread[n]の形で使うときは型がunsigned(正の数)になる
            whatread = esp.read(4)
            #print(str(whatread[0])+", "+str(whatread[1])+", "+str(whatread[2])+", "+str(whatread[3]))

            # 直角系
            if whatread[0] != 0:
                if whatread[0] == 1:
                    # 左折コーナー
                    black("l")
                elif whatread[0] == 2:
                    # 右折コーナー
                    black("r")
                elif whatread[0] == 3:
                    black("both")
                esp.clear()
                continue # ループの最初に戻る(通信の内容を更新)
            
            # 中央のセンサが黒を読んでいない場合
            if whatread[2] == 1 and rgb_left[2] > highest_refrection_of_Black and rgb_right[2] > highest_refrection_of_Black:
                print_pico(130)
                tank.stop("brake")
                while 1:
                    pass
                lost_line()
                print_pico(100)
                wait(100)
                esp.clear()
                continue

            # 坂関係
            if hill_statue == 0:
                if whatread[3] == 0:
                    pass
                elif whatread[3] == 1:
                    basic_speed = 50
                    hill_statue = 1
                elif whatread [3] == 2:
                    basic_speed = 20
                    hill_statue = 2
                elif whatread [3] == 3: # 各速度がどえらいでかい時は一度止まろうよ
                    tank.stop("brake")
                    wait(2500)
                    basic_speed = 20
                    hill_statue = 2
                
                # レスキューキット 坂だと誤探知の可能性があるからここに置くンゴ -----
                if whatread[2] == 2 or whatread[2] == 3: # whatread[2]==3(つまりラインを見失いかつレスキューキットを発見)になることはないと思うけどそん時はレスキューキット優先、あとで進行停止しよう
                    print_pico(777)
                    rescuekit()
                    print_pico(100)
                    esp.clear() #ここでいったんクリアしたげる
                    continue # ループの最初に戻る(通信の内容を更新)
                # -----------------------------------------------------------
            elif whatread[3] == 0:
                basic_speed = 25
                hill_statue = 0

            if whatread[1]%10 > 0:
                avoid()
                continue

            # UARTも鑑みつつ13ms待つ
            if error_count == 0:
                wait(13)
            elif error_count == 1:
                wait(3)
        
        ev3.speaker.beep()
        esp.clear()
        pico.clear()
        motorLeft.hold()
        motorRight.hold()

        # ここでESPとPicoにストップ&リセット信号を送る(予定)
        
        # ボタンが離されるのを待つ
        while any(ev3.buttons.pressed()):
            pass

main()
