# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
color_sensor=ColorSensor(Port.S3)
b_motor=Motor(Port.B)
c_motor=Motor(Port.C)

# Write your program here.

def pid(kp_value,ki_value,kd_value,b,w):
   pid_value=0
   target=(b+w)/2
   kp=kp_value
   ki=ki_value
   kd=kd_value
   last_error=0
   error=target-color_sensor.reflection()
   color_sensor.reflection()
   pid_value=kp*error+ki*(error+last_error)+kd*(error-last_error)
   ev3.screen.print(pid_value)
   b_motor.dc(100-pid_value)
   c_motor.dc(-100-pid_value)
   last_error=error

while True:
   pid(1.45,0,0,9,60)