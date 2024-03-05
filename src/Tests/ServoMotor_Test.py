import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')

from robot_hat import Servo
import MPU6050Plot as MPU
# Create Servo object with PWM object
#servo_CameraYaw = Servo("P0")
servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_Steer = Servo("P2")

####Intialize Servo Motors
# Set servo to position 0, here 0 is the center position,
# angle ranges from -90 to 90

## YawAngle## Center = 10 degrees   ----> Limits = Left Right 90 / -90 (deg)
servo_CameraYaw.angle(10)
##PitchAngle## Center = -30 degrees ----> Limits = Up/Down = -90 / 0 (deg) 
servo_CameraPitch.angle(-30)
##SteeringAngle## Center = -10?  ----> Limits = Right Left 30 /-30 degrees (deg)
servo_Steer.angle(-10)



        
