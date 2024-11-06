import sys
# Adjust paths as needed
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
import multiprocessing
import csv
import cv2
import numpy as np
import time
from robot_hat import Servo, Motors, modules, pin
from LaneDetection import calculate_lane_center
from Ultrasonic import read_ultrasonic_distance
from VelocityObserver_Read import real_time_velocity_plotting  # Import the velocity tracking function



# PID parameters
Kp = 0.1
Ki = 0.01
Kd = 0.01
deadband = 10  # Margin of error to reduce oscillations (in pixels)

# Initialize motors, servos, and ultrasonic sensor
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)
motor_value = 0  # Set a constant speed for now
servo_Steer = Servo("P2")
us = modules.Ultrasonic(pin.Pin("D2"), pin.Pin("D3"))

# Center x-coordinate of the frame (target for lane center)
FRAME_CENTER_X = 320  # Assuming a 640x480 frame

# PID state variables
prev_error = 0
integral = 0

# Distance threshold to stop car
STOP_DISTANCE_CM = 10

mean_tau = 0.1203
mean_alpha = 1.1349

# Prepare CSV file for logging
csv_file = "straight_line_log_data.csv"
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
    steer_angle = max(-25, min(25, output))  # Clamped between -25 and 25 degrees

    # Log data to CSV file
    with open(csv_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([time.time(), steer_angle, error])

    return steer_angle

# Function to handle lane centering and PID control
def lane_centering():
    cap = cv2.VideoCapture(0)
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Calculate the lane center
        lane_center, processed_frame = calculate_lane_center(frame)

        # Get the distance from the ultrasonic sensor
        distance = read_ultrasonic_distance()
        if distance is not None:
            print(f"Distance: {distance} cm")
            if distance < STOP_DISTANCE_CM:
                # Stop motors if too close to an obstacle
                motors.stop()
                print("Obstacle detected! Stopping car.")
                continue  # Skip PID calculation and motor adjustments when stopping

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
            # Ensure all motors are stopped and windows are closed after exiting the loop
            cap.release()
            cv2.destroyAllWindows()
            motors.stop()
            break

# Main function to start the processes
if __name__ == "__main__":
    # Create a separate process for real-time velocity plotting
    velocity_process = multiprocessing.Process(target=real_time_velocity_plotting, args=(mean_tau, mean_alpha))

    # Start the velocity tracking process
    velocity_process.start()

    # Run lane centering and PID control in the main process
    lane_centering()

    # Wait for the velocity plotting process to finish
    velocity_process.join()
