import sys
sys.path.append('/home/apeters/project/env/src')


from MPU6050 import accelerometer 
from MPU6050 import gyroscope
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import count
 
plt.style.use('fivethirtyeight')
plt.style.use('dark_background')
fig, ax = plt.subplots(2, 3, figsize=(15, 5))  # 1 row, 3 columns

x_data, y_data, z_data = [], [], []
x_gyro_data, y_gyro_data, z_gyro_data = [], [], []
x_val =[]
index = count()
def update(frame):
    
    x_val.append(next(index))
    accel_data = accelerometer()
    gyro_data = gyroscope()
    x_data.append(accel_data[0])
    y_data.append(accel_data[1])
    z_data.append(accel_data[2])
    x_gyro_data.append(gyro_data[0])
    y_gyro_data.append(gyro_data[1])
    z_gyro_data.append(gyro_data[2])
    
##Plots
    #Accelerometer
    ax[0, 0].clear()
    ax[0, 0].plot(x_val,x_data, label='X-axis',color ='red')
    ax[0, 0].legend()
    ax[0, 0].set_title('X-axis Acceleration')
    
    ax[0, 1].clear()
    ax[0, 1].plot(x_val,y_data, label='Y-axis',color ='blue')
    ax[0, 1].legend()
    ax[0, 1].set_title('Y-axis Acceleration')
    
    ax[0, 2].clear()
    ax[0, 2].plot(x_val,z_data, label='Z-axis',color = 'green')
    ax[0, 2].legend()
    ax[0, 2].set_title('Z-axis Acceleration')
    
    #Gyroscope
    ax[1, 0].clear()
    ax[1, 0].plot(x_val,x_gyro_data, label='X-axis',color = 'red')
    ax[1, 0].legend()
    ax[1, 0].set_title('X-axis Gyro')
    
    ax[1, 1].clear()
    ax[1, 1].plot(x_val,y_gyro_data, label='Y-axis',color = 'blue')
    ax[1, 1].legend()
    ax[1, 1].set_title('Y-axis Gyro')
    
    ax[1, 2].clear()
    ax[1, 2].plot(x_val,z_gyro_data, label='Z-axis',color = 'green')
    ax[1, 2].legend()
    ax[1, 2].set_title('Z-axis Gyro')
    
ani = FuncAnimation(fig, update, interval=1000)
plt.tight_layout()
plt.show()