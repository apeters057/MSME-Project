import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
from scipy.integrate import quad

sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

from pynput import keyboard
import threading
import time
import csv
import mpu6050
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115 , VoltageRead_Battery 
from MPU6050 import accelerometer ,gyroscope ,velocity


#Continous State Space Model
mean_tau = 0.1203 # Calculated From Matlab --> tau = .632 @ Vss
mean_alpha = 1.1349 #Calculated From Matlab --> alpha = Speed / Vss
A = -1 / mean_tau 
B = mean_alpha / mean_tau
C = A
D = B

#Discrete State Space Model - Derived Equations to go from Cont -> Discrete (Basash Notes)
T =0.0157 #########CONT UPDATE
#Discretization
G = float(expm(A*T))
#G = expm(A*T).item()

def integrand(lambda_, A, B):
   return expm(A * lambda_) * B

H, _ = quad(integrand, 0 , T, args=(A,B))#compute the integral
#H = H.item()
H = float(H)
#G = float(G)



#Convert G & H to scalars
# if isinstance(G,no.ndarray):
#     g = np.asscalar(G)
# if isinstance(H,np.ndarray):
#     H = np.asscalar(H)

def get_Acceleration():
    accel_data = accelerometer()
    accel_X = accel_data[0] # Foward and Back Data
    return float(accel_X)
def get_Voltage ():
    voltage_motor = VoltageRead_ADS1115()
    return float(voltage_motor)

#Initial Conditions
x_sys = [0.0]
y_sys = [0.0]
x_hat = []
y_hat = [0.0]

L = 0.005 # Manually Tune... Higher value = Model Lower ... Lower Value = Towards Imu Data

x_sys.append(0) #Intial System State
x_hat.append(0) #Intial Observer State
y_hat.append(0) #initial Estimated Output

try:
    while True:
        #Real Time Data
        u = get_Voltage()
        y = get_Acceleration()
        
        #Update lists with current data
        y_sys.append(y)
        
        #System state update (Velocity)
        new_x_sys = G * x_sys[-1] + H * u
        x_sys.append(float(new_x_sys))
        
        # Observer state update (Estimated Velocity)
        new_x_hat = G * x_hat[-1] + H * u + L * (y_sys[-1] - y_hat[-1])
        x_hat.append(float(new_x_hat))
        y_hat.append(float(A * x_hat[-1] + B * u))
        
        time.sleep(T)
        
except KeyboardInterrupt:

    pass

#convert the lists to numpy arrays for manipulation
x_sys = np.array(x_sys)
y_sys = np.array(y_sys)
x_hat = np.array(x_hat)
y_hat = np.array(y_hat)

#Plot Results
plt.plot (x_sys, label='IMU Velocity')
plt.plot (x_hat, label='Estimated Velocity')
plt.xlabel('Time Steps')
plt.ylabel('Velocity')
plt.legend()
plt.show()





