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
from LaneDetection import calculate_lane_center
from Ultrasonic import read_ultrasonic_distance
from VoltageRead import VoltageRead_ADS1115
from MPU6050 import accelerometer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.linalg import expm
from scipy.integrate import quad
from pynput import keyboard

# PID parameters
Kp = 0.10015
Ki = 0.0000001
Kd = 0.0000
deadband = 50  # Margin of error to reduce oscillations (in pixels)

# Steering parameters
STEER_CENTER = -5
STEER_LIMIT = 28

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
STOP_DISTANCE_CM = 30

# Log data to CSV
csv_file = "lane_tracking_log.csv"
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Lane Deviation (%)", "Steering Angle", "Error (px)", "Motor Speed"])  # CSV headers

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

# Adjust motor speed based on lane deviation
def adjust_speed(lane_deviation, base_speed, deviation_threshold=15):
    if lane_deviation > deviation_threshold:
        return base_speed * 0.5  # Reduce speed if deviation exceeds threshold
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

    steer_angle = max(-STEER_LIMIT, min(STEER_LIMIT, P + I + D)) + STEER_CENTER
    return steer_angle, error

# Main lane centering function
def lane_centering(shared_velocity):
    cap = cv2.VideoCapture(0)
    start_time = time.time()

    camera_tilt_start_time = None  # To track the camera tilt time
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Calculate the lane center
        lane_center, processed_frame = calculate_lane_center(frame)

        # If lane_center is None, just continue and adjust camera pitch
        if lane_center is None:
            print("No lane detected, car is continuing.")
            # If lane center is not detected, tilt camera up to look for the turn
            if camera_tilt_start_time is None:  # Only raise the camera once
                servo_CameraPitch.angle(-15)  # Look up to -15 degrees
                camera_tilt_start_time = time.time()  # Start the timer

            # Check if we have waited for 1 second before resetting the camera
            if time.time() - camera_tilt_start_time >= 1:
                # After 1 second, return the camera to -6 degrees
                servo_CameraPitch.angle(-6)
                camera_tilt_start_time = None  # Reset the timer

            motors[1].speed(base_speed)  # Continue at base speed
            motors[2].speed(-base_speed)  # Continue at base speed
        else:
            # Lane detected, calculate steering angle and motor speed based on lane position
            current_time = time.time()
            dt = current_time - start_time
            start_time = current_time

            steer_angle, error = PID_control(lane_center[0], dt)  # Use lane center for PID
            servo_Steer.angle(steer_angle)

            lane_deviation = calculate_lane_deviation(lane_center[0], 640)  # Assuming lane width is 640
            motor_speed = adjust_speed(lane_deviation, base_speed)
            print(f"Lane Deviation: {lane_deviation:.2f}%, Steering Angle: {steer_angle}, Motor Speed: {motor_speed}")

            # Log data to CSV
            with open(csv_file, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([time.time(), lane_deviation, steer_angle, error, motor_speed])

            motors[1].speed(motor_speed)  # Move forward with adjusted speed
            motors[2].speed(-motor_speed)  # Move forward with adjusted speed

        # Get the distance from the ultrasonic sensor
        distance = read_ultrasonic_distance()
        if distance is not None and distance < STOP_DISTANCE_CM:
            motors.stop()
            print("Obstacle detected! Stopping car.")
            motors[1].speed(0)  # Ensure motors stop
            motors[2].speed(0)
            continue  # If there's an obstacle, stop and wait for the next frame

        # Display the processed frame with lane center
        #cv2.imshow("Lane Center Calculation", processed_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            motors.stop()
            break

# Main function
if __name__ == "__main__":
    mean_tau = 0.1203
    mean_alpha = 1.1349

    shared_velocity = multiprocessing.Value('d', 0.0)

    # Create a separate process for real-time velocity plotting
    velocity_process = multiprocessing.Process(
        target=real_time_velocity_plotting, args=(mean_tau, mean_alpha, shared_velocity)
    )
    velocity_process.start()

    try:
        lane_centering(shared_velocity)
    finally:
        velocity_process.terminate()
        velocity_process.join()

