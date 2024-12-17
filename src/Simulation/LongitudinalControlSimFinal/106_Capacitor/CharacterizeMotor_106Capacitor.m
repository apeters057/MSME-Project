%Users / anthonypeters / Documents / MATLAB / MSME / Project /
%CharacterizeMotor
clear all;clc;close all;
%% Read Data   
%Test1 = readmatrix("Test5_UnitStep_PWM50.csv");
Test1 = readmatrix("106_UnitStep_PWM80_Test3.csv");
time = Test1(:,1);
start_signal = Test1(:,2);
motor_voltage = Test1(:,3);
accel_X = Test1(:,4);
accel_Y = Test1(:,5);
battery_voltage = Test1(:,6);

%Valid Test Data - Data where Start_Signal is 1
distance = 2.13; %m % = 6.9 feet
distances=[0.5 , 1.0 , 1.5, distance];
driving = find(start_signal == 1);

Time_driving = time(driving,:);
t = Time_driving - Time_driving(1);
speed_avg = distance / max(t); %m/s
motor_voltage_driving = motor_voltage(driving,:);
accelX_driving = accel_X(driving,:);
accelY_driving = accel_Y(driving,:);
battery_voltage_driving = battery_voltage(driving,:);
t_dist = distances ./ speed_avg ;
%% Plot Data
% Plot Output vs Time ( Motor Voltage)
% Plot Input vs Time (Speed )
figure(1);
subplot(2, 1, 1);
plot(t,motor_voltage_driving,'LineWidth', 2)
xaxis([0 0.5])
title('System Response - 10 uF - PWM 80%', 'FontWeight', 'bold','FontSize',15);
xlabel('Time [s]','FontWeight', 'bold','FontSize',13)
ylabel('Voltage [V]','FontWeight', 'bold','FontSize',13)
grid on
subplot(2, 1, 2);
plot(t,accelX_driving,'LineWidth', 2)
title('Foward Car Acceleration', 'FontWeight', 'bold','FontSize',15)
xlabel('Time [s]','FontWeight', 'bold','FontSize',13)
ylabel('Acceleration [m/s^2]','FontWeight', 'bold','FontSize',13)
grid on

figure(2);
plot(t, cumsum([diff(t) ; 0].*accelX_driving))

