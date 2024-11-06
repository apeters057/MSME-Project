import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.linalg import expm
from scipy.integrate import quad
import time
from pynput import keyboard  # To listen for a keypress

sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

from robot_hat import Servo, Motors
from VoltageRead import VoltageRead_ADS1115
from MPU6050 import accelerometer


# Continuous State Space Model
mean_tau = 0.1203  # Calculated From Matlab --> tau = .632 @ Vss
mean_alpha = 1.1349  # Calculated From Matlab --> alpha = Speed / Vss
A = -1 / mean_tau
B = mean_alpha / mean_tau

# Discrete State Space Model
T = 0.0157  # CONT UPDATE
G = float(expm(A * T))


def integrand(lambda_, A, B):
    return expm(A * lambda_) * B


H, _ = quad(integrand, 0, T, args=(A, B))  # Compute the integral
H = float(H)

# Sensor data functions
def get_Acceleration():
    accel_data = accelerometer()
    accel_X = accel_data[0]  # Forward and Back Data
    return float(accel_X)


def get_Voltage():
    voltage_motor = VoltageRead_ADS1115()
    return float(voltage_motor)


# Initial Conditions
x_sys = [0.0]
y_sys = [0.0]
x_hat = [0.0]
y_hat = [0.0]
time_data = [0.0]  # Record real-time elapsed in seconds
L = 0.005  # Manually tune

# Start time for real-time axis
start_time = time.time()
stop_requested = False  # Global flag to stop animation

# Initialize the plot
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='IMU Velocity')
line2, = ax.plot([], [], label='Estimated Velocity')
ax.set_xlim(0, 10)  # Initial limit for x-axis (will adjust dynamically)
ax.set_ylim(-1, 1)  # Initial limit for y-axis (will adjust dynamically)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity')
ax.legend()


def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2


def update(frame):
    global stop_requested
    if stop_requested:
        return line1, line2  # Stop updating the plot if stop requested

    # Real Time Data
    u = get_Voltage()
    y = get_Acceleration()

    # Get elapsed time
    elapsed_time = time.time() - start_time
    time_data.append(elapsed_time)

    # Update lists with current data
    y_sys.append(y)

    # System state update (Velocity)
    new_x_sys = G * x_sys[-1] + H * u
    x_sys.append(float(new_x_sys))

    # Observer state update (Estimated Velocity)
    new_x_hat = G * x_hat[-1] + H * u + L * (y_sys[-1] - y_hat[-1])
    x_hat.append(float(new_x_hat))
    y_hat.append(float(A * x_hat[-1] + B * u))

    # Update the plot data
    line1.set_data(time_data, x_sys)
    line2.set_data(time_data, x_hat)

    # Dynamically adjust x-axis to keep expanding with time
    ax.set_xlim(0, max(time_data))

    # Dynamically adjust y-axis based on the min/max of both datasets
    y_min = min(min(x_sys), min(x_hat)) - 0.1  # Add margin for clarity
    y_max = max(max(x_sys), max(x_hat)) + 0.1  # Add margin for clarity
    ax.set_ylim(y_min, y_max)

    return line1, line2


def on_press(key):
    """Function to listen for 'q' key to stop the animation."""
    global stop_requested
    try:
        if key.char == 'q':
            print("Stop requested by user")
            stop_requested = True
            return False  # Stop the listener
    except AttributeError:
        pass


# Start listening for 'q' key to stop the animation
listener = keyboard.Listener(on_press=on_press)
listener.start()

# Start the real-time animation
ani = FuncAnimation(fig, update, frames=np.arange(1000), init_func=init, blit=True, interval=T * 1000)

try:
    # Show the plot and keep it open even after 'q' is pressed
    plt.show(block=True)
except KeyboardInterrupt:
    print("Exiting cleanly.")

# Keep the listener alive, allowing 'q' to stop the animation without closing the plot
listener.join()
