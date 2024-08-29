
import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
from pynput import keyboard
from robot_hat import Servo
from robot_hat import Motors
import time
#Run through terminal --> sudo python ..
# Create Motor object
motors = Motors()
# Initial motor value
motor_value = 10  # You can set your desired initial value
# Slow down factor
slow_down_factor = 0.001  # Adjust this value to control the speed of slowing down
steer_angle = -10 # Center


# Create Servo object with PWM object
servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_Steer = Servo("P2")

# Setup left and right motors (Drivers POV)?
motors.set_left_id(1)
motors.set_right_id(2)
#Set up Motor Direction Speed
# motors[1].speed(motor_value) #Left
# motors[2].speed(motor_value) #Right
##SteeringAngle## Center = -10?  ----> Limits = Right Left 30 /-30 degrees (deg)
servo_Steer.angle(steer_angle)
# Main game loop
def on_press(key):
    global motor_value
    global steer_angle
    try:
        motors[1].speed(motor_value) #Left
        motors[2].speed(-motor_value) #Right
        servo_Steer.angle(steer_angle)
        # Handle key press events
        if key == keyboard.Key.up:
            motor_value = min(100, motor_value + 2)
            time.sleep(0.0001)
            #print("Up arrow pressed")
        elif key == keyboard.Key.down:
            motor_value = max(-100, motor_value - 2)
            time.sleep(0.0001)
            print(motor_value)
            #print("Down arrow pressed")
        elif key == keyboard.KeyCode.from_char('w'):
            # Set motor value to the maximum (100)
            motor_value = 100
            print("W key pressed")
        elif key == keyboard.KeyCode.from_char('5'):
            # Set motor value to the maximum (100)
            motor_value = 50
            print("5 key pressed")
        elif key == keyboard.KeyCode.from_char('b'):
            # Set motor value to the maximum (100)
            motor_value = -50
            print("b key pressed")
        elif key == keyboard.KeyCode.from_char('e'):
            # Set motor value to the maximum (100)
            motor_value = 0
            motors.stop()
        elif key == keyboard.KeyCode.from_char('s'):
            # Slow down motor value
            motor_value *= slow_down_factor
            #print("S key pressed")
            
        elif key == keyboard.Key.left:
        # Turn car Left (up to 30)
            steer_angle = min(30, steer_angle - 2)
            time.sleep(0.001)
        elif key == keyboard.Key.right:
        # Turn car Right (up to 30)
            steer_angle = max(-30, steer_angle + 2)
            time.sleep(0.001)
        elif key == keyboard.KeyCode.from_char('q'):
        # Set steering striaght
            steer_angle = -10
            
            

    except Exception as e:
        print(f"Error: {e}")

# Set up the keyboard listener
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()