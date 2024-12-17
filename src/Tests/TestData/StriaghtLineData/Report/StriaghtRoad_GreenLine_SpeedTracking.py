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
from robot_hat import Servo, Motors
from VoltageRead import VoltageRead_ADS1115
from MPU6050 import accelerometer
from Ultrasonic import read_ultrasonic_distance  # Import ultrasonic function

# PID parameters for lane-centering control
Kp = 0.1
Ki = 0.0000
Kd = 0.000
STEER_CENTER = -5
STEER_LIMIT = 25

# Camera frame parameters
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_CENTER_X = FRAME_WIDTH // 2

# Initialize motors and servos
motors = Motors()
motors.set_left_id(1)
motors.set_right_id(2)
servo_Steer = Servo("P2")
servo_Steer.angle(STEER_CENTER)  # Set steering to center

# CSV header
csv_file = "velocity_straightline_tracking_2.5[m]_5.csv"
csv_header = [
    "Time (s)", "Lane Deviation (%)", "Steering Angle",
    "Error (px)", "Measured Velocity (m/s)", "Average Velocity (m/s)", "Expected Velocity (m/s)"
]

0.# Expected velocity mapping for motor input
input_to_velocity = {
    50: 0.2481,
    70: 0.3711,
    80: 0.4373
}

def calculate_expected_velocity(motor_input):
    """
    Calculate the expected velocity based on motor input.
    """
    return input_to_velocity.get(motor_input, 0)  # Default to 0 if input not in mapping

# Ultrasonic validation process
def ultrasonic_validation(stop_signal, threshold=20, consecutive_count=3):
    """
    Process to continuously validate ultrasonic readings and signal a stop if an obstacle is detected.
    """
    while True:
        readings = []
        for _ in range(consecutive_count):
            distance = read_ultrasonic_distance()
            if distance is not None:
                readings.append(distance)
            else:
                readings.append(float('inf'))  # Treat None as a very large value

        # Check if all readings are below the threshold
        if all(r < threshold for r in readings):
            stop_signal.value = 1  # Signal to stop the motors
            print("Confirmed obstacle detected. Signaling stop.")
            break

# Velocity Observer Function
def real_time_velocity_tracking(mean_tau, mean_alpha, shared_velocity):
    A = -1 / mean_tau
    B = mean_alpha / mean_tau
    T = 0.0157  # Fixed time step

    # Precompute H
    def integrand(lambda_, A, B):
        return np.exp(A * lambda_) * B

    H, _ = quad(integrand, 0, T, args=(A, B))
    H = float(H)

    x_hat = [0.0]
    accel_data = []  # For smoothing accelerometer values

    while True:
        try:
            u = VoltageRead_ADS1115()  # Motor voltage
            y = accelerometer()[0]  # X-axis acceleration
            u = float(u)
            y = float(y)

            # Apply a moving average filter to reduce noise
            accel_data.append(y)
            if len(accel_data) > 5:  # Keep the last 5 readings
                accel_data.pop(0)
            y_smoothed = sum(accel_data) / len(accel_data)

            # Observer update
            G = np.exp(A * T)
            new_x_hat = G * x_hat[-1] + H * u + 0.0065 * (y_smoothed - (A * x_hat[-1] + B * u))
            x_hat.append(new_x_hat)

            shared_velocity.value = float(A * x_hat[-1] + B * u)

            # Debug: Log fixed time step and key variables
            print(f"Fixed Timestep: {T:.6f}, Smoothed Acceleration: {y_smoothed:.6f}, Measured Velocity: {shared_velocity.value:.6f}")

            time.sleep(T)
        except Exception as e:
            print(f"Error in velocity tracking: {e}")
            shared_velocity.value = 0  # Default to 0 on failure

# Function to detect the center of the green line
def find_green_line_center(frame):
    lower_green = np.array([40, 138, 111])  # HSV lower bound for green
    upper_green = np.array([60, 255, 255])  # HSV upper bound for green

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    green_blur = cv2.GaussianBlur(green_mask, (13, 13), 0)

    contours, _ = cv2.findContours(green_blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

    if not valid_contours:
        return None, frame

    largest_contour = max(valid_contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    if M['m00'] == 0:
        return None, frame

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    cv2.circle(frame, (cx, cy), 7, (0, 255, 0), -1)
    return (cx, cy), frame

# PID control function
def PID_control(lane_center_x, dt, prev_error, integral):
    error = lane_center_x - FRAME_CENTER_X
    P = Kp * error
    integral += error * dt
    I = Ki * integral
    derivative = (error - prev_error) / dt
    D = Kd * derivative
    prev_error = error

    steer_angle = max(-STEER_LIMIT, min(STEER_LIMIT, P + I + D)) + STEER_CENTER
    return steer_angle, error, integral, prev_error

# Main lane-following function
def lane_following(shared_velocity, stop_signal):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("Initializing camera and components...")
#     time.sleep(1.2)  # Delay for initialization
    print("Starting motors...")

    speed = 50  # Motor speed
    motors[1].speed(speed)
    motors[2].speed(-speed)

    velocity_samples = []
    data_log = []
    start_time = time.time()

    while True:
        if stop_signal.value == 1:
            print("Stop signal received. Stopping motors.")
            motors.stop()
            break

        ret, frame = cap.read()
        if not ret:
            continue

        lane_center, _ = find_green_line_center(frame)
        if lane_center is None:
            continue

        # Record velocity
        current_velocity = shared_velocity.value
        velocity_samples.append(current_velocity)

        # PID control for steering
        lane_center_x, _ = lane_center
        steer_angle, error, _, _ = PID_control(lane_center_x, 0.0157, 0, 0)
        servo_Steer.angle(steer_angle)

        # Compute lane deviation and expected velocity
        lane_deviation = abs(lane_center_x - FRAME_CENTER_X) / FRAME_WIDTH * 100
        expected_velocity = calculate_expected_velocity(speed)

        elapsed_time = time.time() - start_time
        data_log.append([
            elapsed_time, lane_deviation, steer_angle, error,
            current_velocity, None, expected_velocity  # Average velocity placeholder
        ])

    cap.release()
    cv2.destroyAllWindows()

    # Compute average velocity after the loop
    total_time = time.time() - start_time
    average_velocity = sum(velocity_samples) / len(velocity_samples) if velocity_samples else 0

    # Update average velocity in the logged data
    for row in data_log:
        row[5] = average_velocity

    # Write data to CSV
    with open(csv_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)
        writer.writerows(data_log)

    print(f"Average Velocity: {average_velocity:.6f} m/s")
    print(f"Total Distance Traveled: {average_velocity * total_time:.2f} meters")

# Main function
if __name__ == "__main__":
    shared_velocity = multiprocessing.Value('d', 0.0)
    stop_signal = multiprocessing.Value('i', 0)

    velocity_process = multiprocessing.Process(target=real_time_velocity_tracking, args=(0.125, 1.125049, shared_velocity))
    ultrasonic_process = multiprocessing.Process(target=ultrasonic_validation, args=(stop_signal,))

    velocity_process.start()
    ultrasonic_process.start()

    try:
        lane_following(shared_velocity, stop_signal)
    finally:
        velocity_process.terminate()
        ultrasonic_process.terminate()
        velocity_process.join()
        ultrasonic_process.join()
