import csv
import matplotlib.pyplot as plt

# Load data from CSV
time_log = []
steering_angle_log = []
error_log = []

with open("StraightLine_Log_data.csv", mode="r") as file:
    reader = csv.DictReader(file)
    start_time = None
    for row in reader:
        time = float(row["Time"])
        steering_angle = float(row["Steering Angle"])
        error = float(row["Error"])

        # Adjust time to start at 0
        if start_time is None:
            start_time = time
        time_log.append(time - start_time)
        steering_angle_log.append(steering_angle)
        error_log.append(error)

# Define the deadband range
deadband = 60 #Units Pixels

# Plot steering angle and error over time
plt.figure(figsize=(10, 5))

# Steering Angle Plot
plt.subplot(2, 1, 1)
plt.plot(time_log, steering_angle_log, label="Steering Angle (degrees)")
plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)  # Setpoint line
plt.xlabel("Time (s)")
plt.ylabel("Steering Angle")
plt.title("Steering Angle vs. Time")
plt.legend()

# Error Plot with Deadband
plt.subplot(2, 1, 2)
plt.plot(time_log, error_log, label="Error (pixels)", color='red')
plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)  # Setpoint line

# Add deadband shading
plt.fill_between(time_log, -deadband, deadband, color='yellow', alpha=0.3, label=f"Deadband ±{deadband}")

plt.xlabel("Time (s)")
plt.ylabel("Error")
plt.title("Error vs. Time with Deadband")
plt.legend()

plt.tight_layout()
plt.show()
