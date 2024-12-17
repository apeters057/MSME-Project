import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
import numpy as np
import cv2
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
import multiprocessing
import csv

# PID parameters
Kp = 0.055
Ki = 0.000000
Kd = 0.000
deadband = 50  # Margin of error to reduce oscillations (in pixels)

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
servo_CameraYaw.angle(15)  # Start with camera yaw centered (15 degrees)
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
    writer.writerow(["Time", "Lane Deviation (%)", "Steering Angle", "Error (px)", "Motor Speed", "Velocity"])  # CSV headers

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

# Lane deviation calculation function
def calculate_lane_deviation(lane_center_x, frame_width):
    """
    Calculate the lane deviation percentage based on the lane center position relative to the frame center.
    """
    frame_center_x = frame_width // 2  # Assuming frame width is 640, so center is at 320
    deviation_pixels = abs(lane_center_x - frame_center_x)  # Absolute difference in pixels
    deviation_percentage = (deviation_pixels / frame_center_x) * 100  # Calculate deviation as percentage
    return deviation_percentage

# Dynamic Camera Pan Function
def dynamic_camera_pan(lane_center_x, pan_angle, frame_center_x, threshold=15):
    """
    Adjust the camera pan angle based on the lane center position relative to the frame center.
    """
    # Calculate the deviation from the center of the frame
    deviation = lane_center_x - frame_center_x

    # If the midpoint is off-center, adjust the pan angle
    if abs(deviation) > 20:  # Threshold for adjusting the camera
        if deviation > 0:  # Midpoint is to the right
            pan_angle = max(0, min(30, pan_angle - 5))  # Pan right, with limits
        else:  # Midpoint is to the left
            pan_angle = max(0, min(30, pan_angle + 5))  # Pan left, with limits
    return pan_angle

# Adjust for Camera Pan
def adjust_for_camera_pan(lane_center_x, pan_angle, fov, frame_width):
    """
    Adjust the lane center for the effect of camera pan.
    """
    # Calculate the offset based on the difference from the 15-degree center.
    offset = (pan_angle - 15) / fov * frame_width  # Pan angle adjustment from center (15 degrees)
    adjusted_lane_center = lane_center_x - offset  # Adjust the lane center based on this offset
    return adjusted_lane_center

# PID control function with gradual adjustments
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

# Speed adjustment based on lane deviation
def adjust_speed(lane_deviation, base_speed, deviation_threshold=15, max_decrease=0.5):
    """
    Gradually reduce the speed based on lane deviation. The more the deviation, the slower the car goes.
    
    :param lane_deviation: The percentage of deviation from the center of the lane.
    :param base_speed: The initial motor speed.
    :param deviation_threshold: The deviation value after which speed reduction starts.
    :param max_decrease: The maximum speed reduction factor (0 to 1, where 1 is a full stop).
    
    :return: Adjusted motor speed.
    """
    if lane_deviation > deviation_threshold:
        # Gradually reduce the speed (not abruptly)
        reduction_factor = 1 - (min(lane_deviation, 100) / 100) * max_decrease  # max reduction is max_decrease
        return base_speed * reduction_factor
    return base_speed


# Function to check if the car is in a turn
def is_turn(lane_centers, threshold=30):
    """
    Check if the lane center is rapidly changing, indicating a turn.
    This is done by comparing the change in the x-coordinate of the lane center
    between consecutive frames. If the change is larger than a threshold, 
    it is considered a turn.
    """
    if len(lane_centers) < 2:
        return False
    
    # Get the x-coordinate change between the last two lane centers
    x_diff = lane_centers[-1][0] - lane_centers[-2][0]
    
    # If the change in x is larger than the threshold, it's a turn
    if abs(x_diff) > threshold:
        return True
    return False

# Function to adjust camera tilt (pitch) angle
def adjust_camera_tilt(lane_center, last_known_midpoint, servo_CameraPitch):
    """
    Adjust the camera pitch based on lane center position.
    If no lane is detected, adjust the pitch to look for the turn.
    """
    if lane_center is None:  # If no lane center detected, tilt the camera to look for the turn
        if last_known_midpoint is None:
            servo_CameraPitch.angle(-15)  # Tilt the camera downward to find the lane
        else:
            servo_CameraPitch.angle(-6)  # Default tilt for normal driving
    else:
        # If a lane is detected, keep the camera angle level
        servo_CameraPitch.angle(-6)  # Normal tilt for driving


# Main lane centering function
def lane_centering(shared_velocity):
    cap = cv2.VideoCapture(0)
    start_time = time.time()

    last_known_midpoint = None  # Store last known midpoint
    pan_angle = 15  # Start with the camera centered at 15 degrees
    fov = 62  # Assuming the camera's field of view is 60 degrees (adjust as necessary)
    lane_centers = []  # Track lane centers over time
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Call the lane center calculation function, passing the last known midpoint
        lane_center, processed_frame, last_known_midpoint = calculate_lane_center(frame, last_known_midpoint)

        # If lane_center is None, use last known midpoint for steering
        if lane_center is None:
            print("No lane detected, using last known midpoint.")
            lane_center = last_known_midpoint  # Use last known midpoint if no lane detected
            

        # If lane_center is still None after checking, skip the deviation calculation
        if lane_center is not None:
            # Add the lane center to the list
            lane_centers.append(lane_center)

            # Dynamic adjustment of pan angle (camera follows midpoint)
            pan_angle = dynamic_camera_pan(lane_center[0], pan_angle, FRAME_CENTER_X)

            # Adjust for the camera pan, considering the camera center offset
            adjusted_lane_center = adjust_for_camera_pan(lane_center[0], pan_angle, fov, 640)

            # Lane detected, calculate steering angle and motor speed based on adjusted lane position
            current_time = time.time()
            dt = current_time - start_time
            start_time = current_time

            steer_angle, error = PID_control(adjusted_lane_center, dt)  # Use adjusted lane center for PID
            servo_Steer.angle(steer_angle)

            lane_deviation = calculate_lane_deviation(lane_center[0], 640)  # Use lane center's x-coordinate
            motor_speed = adjust_speed(lane_deviation, base_speed)  # Adjust speed based on lane deviation
            print(f"Lane Deviation: {lane_deviation:.2f}%, Steering Angle: {steer_angle}, Motor Speed: {motor_speed}, Velocity: {shared_velocity.value}")


            # Log data to CSV
            with open(csv_file, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([time.time(), lane_deviation, steer_angle, error, motor_speed, shared_velocity.value])

            motors[1].speed(motor_speed)  # Move forward with adjusted speed
            motors[2].speed(-motor_speed)  # Move forward with adjusted speed

        # If lane_center is None, the car will not move and will use the last known midpoint
        else:
            print("Lane not detected, continuing with the last known midpoint.")
            # Adjust camera tilt based on lane center and last known midpoint
            adjust_camera_tilt(lane_center, last_known_midpoint, servo_CameraPitch)

        # Get the distance from the ultrasonic sensor
        distance = read_ultrasonic_distance()
        if distance is not None and distance < STOP_DISTANCE_CM:
            motors.stop()
            print("Obstacle detected! Stopping car.")
            motors[1].speed(0)  # Ensure motors stop
            motors[2].speed(0)
            continue  # If there's an obstacle, stop and wait for the next frame



        # Display the processed frame with lane center
        cv2.imshow("Lane Center Calculation", processed_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            motors.stop()
            break


if __name__ == "__main__":
    # Create a shared velocity object for inter-process communication
    shared_velocity = multiprocessing.Value('d', 0.0)

    # Create a separate process for real-time velocity tracking
    velocity_process = multiprocessing.Process(
        target=real_time_velocity_plotting, args=(0.1203, 1.1349, shared_velocity)
    )
    velocity_process.start()

    try:
        lane_centering(shared_velocity)
    finally:
        velocity_process.terminate()
        velocity_process.join()

