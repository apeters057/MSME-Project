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
Kp = 0.038
Ki = 0.000003
Kd = 0.00925
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

# CSV log file
csv_file = "velocity_straightline_tracking.csv"
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Lane Deviation (%)", "Steering Angle", "Error (px)", "Measured Velocity (m/s)", "Expected Velocity (m/s)"])

# Expected velocity mapping for motor input
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

# Function to validate ultrasonic readings
def validate_ultrasonic_reading(threshold=10, consecutive_count=3):
    """
    Validate ultrasonic sensor readings by checking for consecutive consistent values.
    """
    readings = []
    for _ in range(consecutive_count):
        distance = read_ultrasonic_distance()
        if distance is not None:
            readings.append(distance)
        time.sleep(0.05)  # Small delay between readings

    # Check if all readings are below the threshold
    return all(r < threshold for r in readings)

# Velocity Observer Function
def real_time_velocity_tracking(mean_tau, mean_alpha, shared_velocity):
    A = -1 / mean_tau
    B = mean_alpha / mean_tau
    T = 0.0157

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
            new_x_hat = np.exp(A * T) * x_hat[-1] + H * u + 0.00615 * (y_smoothed - (A * x_hat[-1] + B * u))
            x_hat.append(new_x_hat)

            shared_velocity.value = float(A * x_hat[-1] + B * u)
            time.sleep(T)
        except Exception:
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

def lane_following(shared_velocity):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # Add a delay to ensure the camera and other components are fully initialized
    print("Initializing camera and components...")
    time.sleep(1.2)  # Delay for 1.2 seconds
    print("Starting motors...")
    
    speed = 50  # Set motor speed for the test (adjust to 50, 70, or 80)
    motors[1].speed(speed)
    motors[2].speed(-speed)

    prev_error = 0
    integral = 0
    start_time = time.time()

    total_distance = 0  # Initialize total distance traveled
    last_time = start_time

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Check for obstacles using raw ultrasonic readings
        distance = read_ultrasonic_distance()
        if distance is None:
            #print("Ultrasonic sensor returned None, continuing...")
            continue  # Ignore None readings and keep driving
        if distance < 20:  # Stop if obstacle is within 10 cm
            print(f"Obstacle detected at {distance:.2f} cm. Stopping motors.")
            motors.stop()
            time.sleep(0.1)
            break

        lane_center, processed_frame = find_green_line_center(frame)
        if lane_center is None:
            continue

        lane_center_x, _ = lane_center
        current_time = time.time()
        elapsed_time = current_time - start_time
        dt = current_time - last_time  # Time step for this iteration
        last_time = current_time

        # Calculate the distance traveled in this time step
        current_velocity = shared_velocity.value
        total_distance += current_velocity * dt  # Increment total distance

        # PID control for steering
        steer_angle, error, integral, prev_error = PID_control(lane_center_x, dt, prev_error, integral)
        servo_Steer.angle(steer_angle)

        # Compute lane deviation
        lane_deviation = abs(lane_center_x - FRAME_CENTER_X) / FRAME_WIDTH * 100

        # Calculate expected velocity based on motor input
        expected_velocity = calculate_expected_velocity(speed)

        # Log data
        with open(csv_file, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([elapsed_time, lane_deviation, steer_angle, error, current_velocity, expected_velocity])

        # Display processed frame
        #cv2.imshow("Lane Following", processed_frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # Print total distance traveled
    print(f"Total Distance Traveled: {total_distance:.2f} meters")

# Main function
if __name__ == "__main__":
    shared_velocity = multiprocessing.Value('d', 0.0)

    # Start velocity tracking process
    velocity_process = multiprocessing.Process(target=real_time_velocity_tracking, args=(0.120, 1.0535049, shared_velocity))#AVg tau = .1203 avg alpha = 1.1349 ,, Pwm 50 , tau = 0.0665 alpha = 1.05	
    velocity_process.start()

    try:
        lane_following(shared_velocity)
    finally:
        velocity_process.terminate()
        velocity_process.join()
