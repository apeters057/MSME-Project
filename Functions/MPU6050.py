import mpu6050
import time

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

# Define a function to read the sensor data
def accelerometer():
    # Read the accelerometer values
    accel_data = mpu6050.get_accel_data()
    accel_X = accel_data['x']
    accel_Y = accel_data['y']
    accel_Z = accel_data['z']
    # Read temp
    #temperature = mpu6050.get_temp()
    return accel_X ,accel_Y ,accel_Z

def gyroscope():
    
    gyro_data = mpu6050.get_gyro_data()
    gyro_X = gyro_data['x']
    gyro_Y = gyro_data['y']
    gyro_Z = gyro_data['z']
    return gyro_X ,gyro_Y ,gyro_Z

#while True:
    #accel_data = accelerometer()
    #print("Accelerometer Data:", accel_data)