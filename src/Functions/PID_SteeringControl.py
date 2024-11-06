import sys
# Adjust paths as needed
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
import csv
import cv2
import numpy as np
import time
from robot_hat import Servo, Motors
from LaneDetection import calculate_lane_center


# PID parameters
Kp = 0.1
Ki = 0.01
Kd = 0.01
deadband = 10  # Margin of error to reduce oscillations UNIT IN PIXELS

# Initialize motors and servos
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)
servo_Steer = Servo("P2")

# Center x-coordinate of the frame (target for lane center)
FRAME_CENTER_X = 320  # Assuming a 640x480 frame; adjust as necessary

# PID state variables
prev_error = 0
integral = 0

# Prepare CSV file for logging
csv_file = "pid_log_data.csv"
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Steering Angle", "Error"])  # CSV headers

# PID control function
def PID_control(lane_center_x, dt):
    global prev_error, integral

    # Calculate error as the difference between lane center and frame center
    error = lane_center_x - FRAME_CENTER_X

    # Apply deadband to reduce rapid oscillations
    if abs(error) < deadband:
        error = 0

    # Proportional term
    P = Kp * error

    # Integral term
    integral += error * dt
    I = Ki * integral

    # Derivative term
    derivative = (error - prev_error) / dt
    D = Kd * derivative

    # PID output for steering angle adjustment
    output = P + I + D
    prev_error = error

    # Clamp the output to a range suitable for servo steering
    steer_angle = max(-25, min(25, output))  # Clamp between -35 and 35 degrees Output deterimes the way the whel will turen

    # Log data to CSV file
    with open(csv_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([time.time(), steer_angle, error])

    return steer_angle

# Capture video
cap = cv2.VideoCapture(0)
motor_value = 0  # Set a constant speed for now
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Calculate the lane center
    lane_center, processed_frame = calculate_lane_center(frame)

    if lane_center:
        lane_center_x, lane_center_y = lane_center

        # Calculate time difference for PID
        current_time = time.time()
        dt = current_time - start_time
        start_time = current_time

        # Compute the steering angle using the PID control
        steer_angle = PID_control(lane_center_x, dt)
        servo_Steer.angle(steer_angle)  # Adjust the steering angle
        print(f"Steer Angle: {steer_angle}, Error: {lane_center_x - FRAME_CENTER_X}")

        # Display the processed frame
        cv2.imshow("Lane Center Calculation with PID", processed_frame)

        # Set motor speeds for forward motion
        motors[1].speed(motor_value)
        motors[2].speed(-motor_value)

    # Check for 'q' key press to exit the loop
    if cv2.waitKey(10) & 0xFF == ord('q'):
        # Ensure all motors are stopped, and windows are closed after exiting the loop
        cap.release()
        cv2.destroyAllWindows()
        motors.stop()
        break
