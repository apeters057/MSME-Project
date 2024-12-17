%Users / anthonypeters / Documents / MATLAB / MSME / Project /
%Compile all Motor Data
clc;clear all ;clear;
directory = '/Users/anthonypeters/Documents/MATLAB/MSME/Project/106_Capacitor';
baseFileName = '106_UnitStep_PWM70_Test';
files = dir(fullfile(directory, [baseFileName, '*.csv']));
allData = cell(1, numel(files));

for i = 1:numel(files)
    
    currentFileName = fullfile(directory, files(i).name);
    t_Data = readmatrix(currentFileName); % Adjust loading method according to your file format
    allData{i} = t_Data;
    
end

%% Capture & Label Sampled Data
speed_total=0;
distance = 2.13; %m % = 6.9 feet
for i = 1:numel(files)
%
    time{i} = allData{i}(:,1);
    start_signal{i} = allData{i}(:,2);
    motor_voltage{i} = allData{i}(:,3);
    accel_X{i} = allData{i}(:,4);
    accel_Y{i} = allData{i}(:,5);
    battery_voltage{i} = allData{i}(:,6);
%
    driving{i} = find(start_signal{i} == 1);
%
    motor_voltage_driving{i} = motor_voltage{i}(driving{i},:);
    accelX_driving{i} = accel_X{i}(driving{i},:);
    accelY_driving{i} = accel_Y{i}(driving{i},:);
    battery_voltage_driving{i} = battery_voltage{i}(driving{i},:);
%
    Time_driving{i} = time{i}(driving{i},:);
    t{i} = Time_driving{i} - Time_driving{i}(1);
    
end

%% Calculate Averages

%%%ReSizeMatrixes to take averages
motor_voltage_driving_maxRows = max(cellfun(@(x) size(x, 1), motor_voltage_driving));
motor_voltage_driving_maxCols = max(cellfun(@(x) size(x, 2), motor_voltage_driving));
motor_voltage_driving_Sum = zeros(motor_voltage_driving_maxRows, motor_voltage_driving_maxCols);

accelX_driving_maxRows = max(cellfun(@(x) size(x, 1), accelX_driving));
accelX_driving_maxCols = max(cellfun(@(x) size(x, 2), accelX_driving));
accelX_driving_Sum = zeros(accelX_driving_maxRows, accelX_driving_maxCols);

accelY_driving_maxRows = max(cellfun(@(x) size(x, 1), accelY_driving));
accelY_driving_maxCols = max(cellfun(@(x) size(x, 2), accelY_driving));
accelY_driving_Sum = zeros(accelY_driving_maxRows, accelY_driving_maxCols);

battery_voltage_driving_maxRows = max(cellfun(@(x) size(x, 1),battery_voltage_driving));
battery_voltage_driving_maxCols = max(cellfun(@(x) size(x, 2), battery_voltage_driving));
battery_voltage_driving_Sum = zeros(battery_voltage_driving_maxRows, battery_voltage_driving_maxCols);

t_maxRows = max(cellfun(@(x) size(x, 1),t));
t_maxCols = max(cellfun(@(x) size(x, 2), t));
t_Sum = zeros(t_maxRows, t_maxCols);% Initialize a variable to store the total sum

for i = 1:numel(files)
%
    motor_voltage_driving_Data = imresize(motor_voltage_driving{i}, [motor_voltage_driving_maxRows, motor_voltage_driving_maxCols]);
    motor_voltage_driving_Sum = motor_voltage_driving_Sum + motor_voltage_driving_Data;
%
    accelX_driving_Data = imresize(accelX_driving{i}, [accelX_driving_maxRows, accelX_driving_maxCols]);
    accelX_driving_Sum = accelX_driving_Sum + accelX_driving_Data;
    accelY_driving_Data = imresize(accelX_driving{i}, [accelY_driving_maxRows, accelY_driving_maxCols]);
    accelY_driving_Sum = accelY_driving_Sum + accelY_driving_Data;
%
    battery_voltage_driving_Data = imresize(battery_voltage_driving{i}, [battery_voltage_driving_maxRows, battery_voltage_driving_maxCols]);
    battery_voltage_driving_Sum = battery_voltage_driving_Sum + battery_voltage_driving_Data;
%
    t_Data = imresize(t{i}, [t_maxRows, t_maxCols]);
    t_Sum = t_Sum + t_Data;
%
    speed{i} = distance / max(t{i}); %m/s
    speed_total =(speed_total +speed{i}) ;
end


motor_voltage_driving_average = motor_voltage_driving_Sum / numel(files);
accelX_driving_average =accelX_driving_Sum / numel(files);
accelY_driving_average =accelY_driving_Sum / numel(files);
battery_voltage_driving_average = battery_voltage_driving_Sum /numel(files);
t_average = t_Sum / numel(files);
speed_avg = speed_total/i ;

figure(1);
subplot(2, 1, 1);
plot(t_average,motor_voltage_driving_average)
title('Avg Motor Voltage @ 70% PWM  (10 uF)', 'FontWeight', 'bold');
xlabel('Avg Time [s]')
ylabel('Avg Voltage [V]')
subplot(2, 1, 2);
plot(t_average,accelX_driving_average)
title('Avg. Foward Car Acceleration @ 70% PWM  (10 uF)', 'FontWeight', 'bold');
xlabel('Avg. Time [s]')
ylabel('Avg. Velocity [m/s]')













