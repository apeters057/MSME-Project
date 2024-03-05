import mpu6050
import time

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

# Define a function to read the sensor data
def accelerometer():
    # Read the accelerometer values
    accel_data = mpu6050.get_accel_data()

    # Read temp
    #temperature = mpu6050.get_temp()

    return accel_data 
def gyroscope():
    gyro_data = mpu6050.get_gyro_data()
    return gyro_data

while True:

    # Read the sensor data
    accel_data = accelerometer()
    accel_X = accel_data['x']
    accel_Y = accel_data['y']
    accel_Z = accel_data['z']
    
    gyroscope_data = gyroscope()
    gyro_data = gyroscope()
    gyro_X = gyro_data['x']
    gyro_Y = gyro_data['y']
    gyro_Z = gyro_data['z']
    
    # Print the sensor data
    print(f"Accelerometer: X={accel_X}, Y={accel_Y}, Z={accel_Z}")
#     print("Accelerometer data:", accelerometer_data)
#     print("Gyroscope data:", gyroscope_data)
   # print("Temp:", temperature)

    # Wait for 1 second
    time.sleep(.1)