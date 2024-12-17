import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
import multiprocessing
import csv
import cv2
import numpy as np
import time
from scipy.integrate import quad
from robot_hat import Servo, Motors, modules, pin
from Ultrasonic import read_ultrasonic_distance
from VoltageRead import VoltageRead_ADS1115
from MPU6050 import accelerometer

# PID parameters for lane-centering control
Kp = .035
Ki = 0.00019
Kd = 0.0551
deadband = 0  # Deadband to ignore small errors

# Steering parameters
STEER_CENTER = -5  # Default steering center angle
STEER_LIMIT = 25  # Maximum allowable steering angle

# Initialize motors, servos, and ultrasonic sensor
motors = Motors()
motors.set_left_id(1)  # Set left motor ID
motors.set_right_id(2)  # Set right motor ID
base_speed = 25  # Base speed for the car

# Servo setup
servo_CameraYaw = Servo("P0")
servo_CameraPitch = Servo("P1")
servo_Steer = Servo("P2")
servo_CameraYaw.angle(15)  # Initial yaw angle for the camera
servo_CameraPitch.angle(-8)  # Initial pitch angle for the camera
servo_Steer.angle(STEER_CENTER)  # Set steering to center

# Ultrasonic sensor for obstacle detection
us = modules.Ultrasonic(pin.Pin("D2"), pin.Pin("D3"))

# Camera frame parameters
FRAME_WIDTH = 320  # Reduced resolution width
FRAME_HEIGHT = 240  # Reduced resolution height
FRAME_CENTER_X = FRAME_WIDTH // 2  # Horizontal center of the frame
STOP_DISTANCE_CM = 20  # Stop distance for obstacles (in cm)

# Road width and deviation parameters
ROAD_WIDTH_PIXELS = 512.27  # Calculated road width in pixels
MAX_DEVIATION_PIXELS = 0.15 * ROAD_WIDTH_PIXELS  # 15% of road width in pixels

# Speed parameters
initial_boost_duration = 0.3  # Boost duration in seconds
#boosted_speed = 38  # Speed during the initial boost
#stable_speed = 35.5  # Speed after stabilization
boosted_speed = 48  # Speed during the initial boost
stable_speed = 48  # Speed after stabilization
ramp_up_increment = 4  # Speed increment for ramp-up
max_ramp_up_speed = boosted_speed + 5  # Maximum speed during ramp-up

# CSV log file to record runtime data
csv_file = "lane_tracking_green_log_4.csv"
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Lane Deviation (%)", "Steering Angle", "Error (px)", "Motor Speed", "Velocity (m/s)"])

# Initialize PID state variables
prev_error = 0
integral = 0

# Function to detect and calculate the center of a green line in the frame
def find_green_line_center(frame):
    # Define HSV color range for detecting green
    LHG, LSG, LVG = 40, 138, 111  # Lower bounds
    UHG, USG, UVG = 60, 255, 255  # Upper bounds

    lower_green = np.array([LHG, LSG, LVG])
    upper_green = np.array([UHG, USG, UVG])

    # Define ROI: y = 100 to 200
    roi = frame[100:200, :]

    # Convert ROI to HSV for color-based segmentation
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Create mask for the green line
    green_mask = cv2.inRange(hsv_roi, lower_green, upper_green)

    # Apply Gaussian blur to smooth the edges
    green_blur = cv2.GaussianBlur(green_mask, (13, 13), 0)

    # Find contours in the green mask
    contours, _ = cv2.findContours(green_blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area to ignore small noise
    valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

    if not valid_contours:
        # No valid green line found
        return None, frame

    # Find the largest contour by area (assume this is the green line)
    largest_contour = max(valid_contours, key=cv2.contourArea)

    # Calculate the center of the largest contour
    M = cv2.moments(largest_contour)
    if M['m00'] == 0:
        return None, frame

    cx = int(M['m10'] / M['m00'])  # Center x-coordinate relative to ROI
    cy = int(M['m01'] / M['m00'])  # Center y-coordinate relative to ROI

    # Adjust coordinates to match the full frame
    cx_full = cx  # X-coordinate remains the same
    cy_full = cy + 100  # Add the y-offset from the ROI

    # Draw the ROI on the full frame for visualization
    roi_color = (255, 0, 0)  # Blue color for ROI box
    cv2.rectangle(frame, (0, 100), (frame.shape[1], 200), roi_color, 2)

    # Draw the center of the green contour on the full frame
    green_center = (cx_full, 150)  # Set y-coordinate to 150 for the set point
    cv2.circle(frame, green_center, 7, (0, 255, 0), -1)  # Green dot

    return green_center, frame


# Function to calculate velocity using real-time observer model
def real_time_velocity_plotting(mean_tau, mean_alpha, shared_velocity):
    # State-space model parameters
    A = -1 / mean_tau
    B = mean_alpha / mean_tau
    T = 0.0157  # Sampling time

    def integrand(lambda_, A, B):
        return np.exp(A * lambda_) * B

    # Calculate H using numerical integration
    H, _ = quad(integrand, 0, T, args=(A, B))
    H = float(H)

    # Observer state
    x_hat = [0.0]
    while True:
        try:
            # Read voltage and acceleration
            u = VoltageRead_ADS1115()  # Voltage input
            y = accelerometer()[0]  # Acceleration input (x-axis)
            u = float(u)
            y = float(y)

            # Update observer state
            new_x_hat = np.exp(A * T) * x_hat[-1] + H * u + 0.0065 * (y - (A * x_hat[-1] + B * u))
            x_hat.append(float(new_x_hat))

            # Update shared velocity
            shared_velocity.value = float(A * x_hat[-1] + B * u)
            time.sleep(T)
        except Exception as e:
            # Set velocity to 0 in case of error
            shared_velocity.value = 0

# PID control function for steering adjustment
def PID_control(lane_center_x, dt):
    global prev_error, integral
    # Calculate error
    error = lane_center_x - FRAME_CENTER_X

    # Clamp error to 15% of road width
    #if abs(error) > MAX_DEVIATION_PIXELS:
        #error = MAX_DEVIATION_PIXELS if error > 0 else -MAX_DEVIATION_PIXELS

    if abs(error) < deadband:
        error = 0  # Ignore errors within deadband

    # PID calculations
    P = Kp * error
    integral += error * dt
    I = Ki * integral
    derivative = (error - prev_error) / dt
    D = Kd * derivative

    # Update previous error and calculate steering angle
    prev_error = error
    return max(-STEER_LIMIT, min(STEER_LIMIT, P + I + D)) + STEER_CENTER, error

# Main lane-centering function
def lane_centering(shared_velocity):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)  # Set camera width
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)  # Set camera height

    # Apply initial boost
    time.sleep(1)
    motors[1].speed(boosted_speed)
    motors[2].speed(-boosted_speed)
    time.sleep(initial_boost_duration)
    base_speed = stable_speed

    start_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect lane center
        lane_center, processed_frame = find_green_line_center(frame)

        # Check for obstacles using the ultrasonic sensor
        if processed_frame is not None:
            distance = read_ultrasonic_distance()
            if distance is not None and distance < STOP_DISTANCE_CM:
                motors.stop()
                time.sleep(0.1)
                continue

        # Handle no lane detected
        if lane_center is None:
            motors[1].speed(base_speed)
            motors[2].speed(-base_speed)
            continue

        # Extract x-coordinate of lane center
        lane_center_x, _ = lane_center

        # PID control
        current_time = time.time()
        dt = current_time - start_time
        start_time = current_time

        steer_angle, error = PID_control(lane_center_x, dt)
        servo_Steer.angle(steer_angle)

        # Calculate lane deviation percentage
        lane_deviation = abs(lane_center_x - FRAME_CENTER_X) / ROAD_WIDTH_PIXELS * 100

        # Adjust motor speed based on deviation
        motor_speed = base_speed * (1 - min(lane_deviation / 100, 0.7))

        # Ramp up speed if velocity is too low
        current_velocity = shared_velocity.value
        if current_velocity <= 0:
            motor_speed = min(motor_speed + ramp_up_increment, max_ramp_up_speed)

        # Log data to CSV
        with open(csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([time.time(), lane_deviation, steer_angle, error, motor_speed, current_velocity])

        # Update motor speeds
        motors[1].speed(motor_speed)
        motors[2].speed(-motor_speed)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Main function
if __name__ == "__main__":
    # Shared variable for velocity between processes
    shared_velocity = multiprocessing.Value('d', 0.0)

    # Start velocity observer process
    velocity_process = multiprocessing.Process(target=real_time_velocity_plotting, args=(0.1203, 1.1349, shared_velocity))
    velocity_process.start()

    try:
        lane_centering(shared_velocity)
    finally:
        velocity_process.terminate()
        velocity_process.join()


