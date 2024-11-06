
import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
from pynput import keyboard
from robot_hat import Servo
from robot_hat import Motors
import time

###Program Manually Controls the motors for the Car.
## Longitudnal Speed Control
## Lateral Yaw Control
## Camera Yaw and Pitch Control

# Set servo to position 0, here 0 is the center position,
# angle ranges from -90 to 90
# Run through terminal --> sudo python ..

##### Initial motor values #####
#Lateral / Longitudinal 
motor_value = 10 # Drive Value
slow_down_factor = 0.001  
steer_angle = -10 # Center
#Camera Servos
Camera_Yaw_init = 15 #YawAngle Center = 15 degrees   ----> Limits = Left Right 90 / -90 (deg)
Camera_Pitch=-40 #PitchAngle Center = -30 degrees ----> Limits = Up/Down = -90 / 0 (deg) 

##### Create Motor objects #####
motors = Motors()
motors.set_left_id(1) # Setup left and right motors (Drivers POV)?
motors.set_right_id(2)
#Lateral
servo_Steer = Servo("P2")
servo_Steer.angle(steer_angle)
#Camera
servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_CameraYaw.angle(Camera_Yaw_init)
servo_CameraPitch =(Camera_Pitch)

# Main 
def on_press(key):
    global motor_value
    global steer_angle
    try:
        motors[1].speed(motor_value) #Left
        motors[2].speed(-motor_value) #Right
        servo_Steer.angle(steer_angle)
        
        #servo_CameraPitch.angle =(Camera_Pitch)
        
# Longitudinal Manual Control
        if key == keyboard.Key.up:
            motor_value = min(100, motor_value + 2)
            time.sleep(0.0001)
            print(motor_value)
        elif key == keyboard.Key.down:
            motor_value = max(-100, motor_value - 2)
            time.sleep(0.0001)
            print(motor_value)
        elif key == keyboard.KeyCode.from_char('1'):
            # Set motor value to the maximum (100)
            motor_value = 100
            print("Full Speed")
        elif key == keyboard.KeyCode.from_char('5'):
            # Set motor value to (50)
            motor_value = 50
            print("Half Speed")
    # Reverse / Stop / Slowdown         
        elif key == keyboard.KeyCode.from_char('b'):
            motor_value = -50
            print("Reverse")
        elif key == keyboard.KeyCode.from_char('0'):
            motor_value = 0
            motors.stop()
            print("Motors Stoped")
        elif key == keyboard.KeyCode.from_char('v'):
            # Slow down motor value
            motor_value *= slow_down_factor

# Lateral Manual Control           
        elif key == keyboard.Key.left:
        # Turn car Left (up to 30)
            steer_angle = min(40, steer_angle - 2)
            Camera_Yaw = min(90, -steer_angle/2 + Camera_Yaw_init)
            servo_CameraYaw.angle(Camera_Yaw)
            #time.sleep(0.001)
            print(steer_angle)
        elif key == keyboard.Key.right:
        # Turn car Right (up to 30)
            steer_angle = max(-40, steer_angle + 2)
            Camera_Yaw = max(-90, -steer_angle/2 + Camera_Yaw_init)
            servo_CameraYaw.angle(Camera_Yaw)
            #time.sleep(0.001)
            print(steer_angle)
        elif key == keyboard.KeyCode.from_char('c'):
        # Set steering striaght
            steer_angle = -10
            
    except Exception as e:
        print(f"Error: {e}")

# Set up the keyboard listener
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()