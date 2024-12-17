import csv
import matplotlib.pyplot as plt

# File to read
csv_file = "/home/apeters/project/env/src/Tests/StriaghtLineData/Report/velocity_straightline_tracking.csv"

# Initialize lists to store data
time_log = []
velocity_log = []
expected_velocity_log = []

# Read data from CSV
with open(csv_file, mode="r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        time_log.append(float(row["Time (s)"]))
        velocity_log.append(float(row["Measured Velocity (m/s)"]))
        expected_velocity_log.append(float(row["Expected Velocity (m/s)"]))

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(time_log, velocity_log, label="Measured Velocity", linewidth=2)
plt.plot(time_log, expected_velocity_log, label="Expected Velocity", linestyle="--", linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Measured Velocity vs Expected Velocity")
plt.legend()
plt.grid(True)
plt.show()
 #AVg tau = .1203 avg alpha = 1.1349 ,, Pwm 50 , tau = 0.0665 alpha = 1.05
