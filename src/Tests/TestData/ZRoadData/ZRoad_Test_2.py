import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')

import multiprocessing
import csv
import cv2
import numpy as np
import time
from robot_hat import Servo, Motors, modules, pin
from LaneDetection_w_Midline import calculate_lane_center  # Updated import
from Ultrasonic import read_ultrasonic_distance
from VoltageRead import VoltageRead_ADS1115
from MPU6050 import accelerometer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.linalg import expm
from scipy.integrate import quad
from pynput import keyboard

# PID parameters
Kp = 0.045
Ki = 0.0001
Kd = 0.005
deadband = 100  # Margin of error to reduce oscillations (in pixels)

# Steering parameters
STEER_CENTER = -5
STEER_LIMIT = 25

# Initialize motors, servos, and ultrasonic sensor
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)
base_speed = 0  # Base motor speed

servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_Steer = Servo("P2")
servo_CameraYaw.angle(15)
servo_CameraPitch.angle(-6)  # Start with camera pitched down
servo_Steer.angle(STEER_CENTER)

us = modules.Ultrasonic(pin.Pin("D2"), pin.Pin("D3"))

# Center x-coordinate of the frame (target for lane center)
FRAME_CENTER_X = 320  # Assuming a 640x480 frame

# PID state variables
prev_error = 0
integral = 0

# Distance threshold to stop car
STOP_DISTANCE_CM = 20

# Log data to CSV
csv_file = "lane_tracking_log.csv"
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Lane Deviation (%)", "Steering Angle", "Error (px)", "Motor Speed", "Velocity (m/s)"])  # CSV headers

# Real-time velocity tracking function
def real_time_velocity_plotting(mean_tau, mean_alpha, shared_velocity):
    A = -1 / mean_tau
    B = mean_alpha / mean_tau
    T = 0.0157
    G = float(np.exp(A * T))

    def integrand(lambda_, A, B):
        return np.exp(A * lambda_) * B

    H, _ = quad(integrand, 0, T, args=(A, B))
    H = float(H)

    x_hat = [0.0]
    while True:
        try:
            u = VoltageRead_ADS1115()  # Voltage input
            y = accelerometer()[0]  # Acceleration input (x-axis)

            # Ensure u and y are floats
            u = float(u)
            y = float(y)

            # Calculate new_x_hat
            new_x_hat = G * x_hat[-1] + H * u + 0.005 * (y - (A * x_hat[-1] + B * u))

            # Ensure new_x_hat is a float before appending to x_hat
            if isinstance(new_x_hat, (list, np.ndarray)):  # If it's a sequence, take the first element
                new_x_hat = new_x_hat[0]
            x_hat.append(float(new_x_hat))

            # Update shared_velocity with the computed value
            shared_velocity.value = float(A * x_hat[-1] + B * u)

            time.sleep(T)
        except Exception as e:
            print(f"Velocity observer error: {e}")
            shared_velocity.value = 0  # Default to zero if error occurs

# Calculate lane deviation percentage
def calculate_lane_deviation(lane_center_x, lane_width):
    deviation_pixels = abs(lane_center_x - FRAME_CENTER_X)
    return (deviation_pixels / lane_width) * 100

# Adjust motor speed based on lane deviation with gradual reduction
def adjust_speed(lane_deviation, base_speed, deviation_threshold=15, max_decrease=0.5):
    if lane_deviation > deviation_threshold:
        reduction_factor = 1 - (min(lane_deviation, 100) / 100) * max_decrease
        return base_speed * reduction_factor
    return base_speed

# PID control function
def PID_control(lane_center_x, dt):
    global prev_error, integral
    error = lane_center_x - FRAME_CENTER_X
    if abs(error) < deadband:
        error = 0
    P = Kp * error
    integral += error * dt
    I = Ki * integral
    derivative = (error - prev_error) / dt
    D = Kd * derivative
    prev_error = error
    return max(-STEER_LIMIT, min(STEER_LIMIT, P + I + D)) + STEER_CENTER, error

# Main lane centering function
def lane_centering(shared_velocity):
    cap = cv2.VideoCapture(0)
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        lane_center, processed_frame = calculate_lane_center(frame)

        if processed_frame is not None:
            cv2.imshow("Lane Center Calculation", processed_frame)

        # Check for obstacles
            distance = read_ultrasonic_distance()
        if distance is not None and distance < STOP_DISTANCE_CM:
            motors.stop()
            print("Obstacle detected. Motors stopped. Waiting for obstacle to clear...")
            time.sleep(0.1)  # Add a brief delay to avoid continuous processing
            continue  # Skip the rest of the loop and wait for the obstacle to clear

        if lane_center is None:
            print("No lane detected. Continuing...")
            motors[1].speed(base_speed)
            motors[2].speed(-base_speed)
            continue

        lane_center_x, lane_center_y = lane_center
        current_time = time.time()
        dt = current_time - start_time
        start_time = current_time

        steer_angle, error = PID_control(lane_center_x, dt)
        servo_Steer.angle(steer_angle)

        lane_deviation = calculate_lane_deviation(lane_center_x, 640)
        motor_speed = adjust_speed(lane_deviation, base_speed)

        # Print current velocity
        current_velocity = shared_velocity.value
        print(f"Velocity: {current_velocity:.2f} m/s, Lane Deviation: {lane_deviation:.2f}%, Steering Angle: {steer_angle}, Motor Speed: {motor_speed}")

        with open(csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([time.time(), lane_deviation, steer_angle, error, motor_speed, current_velocity])

        motors[1].speed(motor_speed)
        motors[2].speed(-motor_speed)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Main function
if __name__ == "__main__":
    shared_velocity = multiprocessing.Value('d', 0.0)

    # Start the velocity tracking process
    velocity_process = multiprocessing.Process(
        target=real_time_velocity_plotting, args=(0.1203, 1.1349, shared_velocity)
    )
    velocity_process.start()

    try:
        lane_centering(shared_velocity)
    finally:
        velocity_process.terminate()
        velocity_process.join()
