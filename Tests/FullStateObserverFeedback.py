import sys
sys.path.append('/home/apeters/project/env/src/Functions')
sys.path.append('/home/apeters/project/env/src')
sys.path.append('/home/apeters/project/env/lib/python3.9/site-packages')
sys.path.append('/home/apeters/project/env/src/robot-hat')

from pynput import keyboard
import numpy as np
from scipy.integrate import quad
import threading
import time
import csv
import mpu6050
from robot_hat import Servo ,Motors
from VoltageRead import VoltageRead_ADS1115 , VoltageRead_Battery
from MPU6050 import accelerometer ,gyroscope ,velocity


#Continous State Space Model
mean_tau = 0.0159 # Calculated From Matlab --> tau = .632 @ Vss
mean_alpha = 1.1349 #Calculated From Matlab --> alpha = Speed / Vss
A = -1 / mean_tau 
B = mean_alpha / mean_tau
C = A
D = B

#Discrete State Space Model
T =0.0157 #########CONT UPDATE
G = np.exp(A*T)
f = lambda lambda_: np.exp(A*lambda_)*B
H, _ = quad(f, 0, T) #compute the integral

#Simulation Variables
final_time = 5  ########CONT UPDATE
t = np.arange(0,final_time + T,T)
num_steps = len(t)
u = np.ones_like(t) * 0.27  # Input signal (Voltage)

#System
x_sys = np.zeros(num_steps)
y_sys = np.zeros(num_steps)

#Observer
x_hat = np.zeros(num_steps)
y_hat = np.zeros(num_steps)
L = 0.001 #Tune

for k in range(num_steps-1):

    #System States
    x_sys[k+1] = G * x_sys[k] + H * u[k]
    y_sys[k] = C * x_sys[k] + D * u[k]

    #Observer States
    x_hat[k+1] = G * x_hat[k] + H * u[k] + L * (y_sys[k] - y_hat[k])
    y_hat[k+1] = C * x_hat[k] + D * u[k]
    
    




