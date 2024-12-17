clc;
clear;
close all;

%% Define Road Geometry
x_trajectory = linspace(0, 3.5, 300); % Fine-grained X positions
y_trajectory = [zeros(1, 100), linspace(0, .3, 100), .3 * ones(1, 100)]; % Road Y positions

road_width = 0.215; % Width of the road in meters
offset = (0.15 * road_width); % 15% of the road width offset

x_road_left = x_trajectory;
x_road_right = x_trajectory;
y_road_left = y_trajectory - road_width / 2;
y_road_right = y_trajectory + road_width / 2;

y_road_dashed_left = y_trajectory - offset;
y_road_dashed_right = y_trajectory + offset;

%% Initialize Vehicle Parameters and PID Controller
Lf = 0.05; % Distance from center to front axle
L = 0.1; % Length of the wheelbase
dt = 0.015; % Time step
v = 0.25; % Initial speed in m/s
v_max = 0.4; % Maximum speed
v_min = 0.25; % Minimum speed
x = x_trajectory(1); % Vehicle's initial x position
y = y_trajectory(1); % Vehicle's initial y position
phi = 0; % Initial heading angle

% PID Controller Gains
Ky = 3.50; % Proportional gain
Ki = 2.5; % Integral gain
Kd = 3.8; % Derivative gain

error_sum = 0; % Integral term
prev_error = 0; % Previous error
prev_error_dot = 0; % Previous derivative of lateral error

look_ahead_distance = 0.02 % 20 cm ahead

simulation_time = 10.8; % Total simulation time (seconds)
time_steps = floor(simulation_time / dt);
x_positions = zeros(1, time_steps);
y_positions = zeros(1, time_steps);
steering_inputs = zeros(1, time_steps); 
speed_values = zeros(1, time_steps); 
deviations_from_center = zeros(1, time_steps);
distance_traveled = zeros(1, time_steps);

%% Simulation Loop
for i = 1:time_steps
    [~, closest_idx] = min((x_trajectory - x).^2 + (y_trajectory - y).^2);
    
    look_ahead_idx = closest_idx;
    for idx = closest_idx:length(x_trajectory)
        if sqrt((x_trajectory(idx) - x)^2 + (y_trajectory(idx) - y)^2) > look_ahead_distance
            look_ahead_idx = idx;
            break;
        end
    end
    target_x = x_trajectory(look_ahead_idx);
    target_y = y_trajectory(look_ahead_idx);

    % Lateral error terms
    error = target_y - y; % Proportional term
    error_sum = error_sum + error * dt; % Integral term
    error_dot = (error - prev_error) / dt; % Derivative term
    prev_error_dot = error_dot;

    % Look-ahead rate of change in trajectory
    if look_ahead_idx < length(y_trajectory)
        yd_dot = (y_trajectory(look_ahead_idx + 1) - y_trajectory(look_ahead_idx)) / dt;
    else
        yd_dot = 0;
    end

    % Control Law for Steering Angle
    sigma = atan((L / Lf) * tan(asin((1 / v) * (yd_dot + Kd * error_dot + Ky * error + Ki * error_sum))));

    % Limit steering angle
    max_steering_angle = pi / 6; % ~50 degrees
    sigma = max(-max_steering_angle, min(max_steering_angle, sigma));
    alpha = 0.25; % Smoothing factor (0 < alpha < 1)
    if i > 1
       sigma = alpha * sigma + (1 - alpha) * steering_inputs(i - 1); % Smooth steering
    end
    % Adjust speed based on curvature
    if closest_idx < length(x_trajectory) - 1
        dx = x_trajectory(closest_idx + 1) - x_trajectory(closest_idx);
        dy = y_trajectory(closest_idx + 1) - y_trajectory(closest_idx);
        curvature = abs(dy / dx);
    else
        curvature = 0;
    end
    if curvature > 0.01
        v = max(v_min, v - 0.00031); % Slow down
    else
        v = min(v_max, v + 0.00031); % Speed up
    end

    % Update vehicle position and heading
    prev_x = x;
    prev_y = y;
    x = x + v * cos(phi) * dt;
    y = y + v * sin(phi) * dt;
    phi = phi + (v / L) * tan(sigma) * dt;

    % Record positions
    x_positions(i) = x;
    y_positions(i) = y;
    steering_inputs(i) = sigma;
    speed_values(i) = v;

    % Deviation from centerline
    deviation = abs(y - target_y);
    deviations_from_center(i) = deviation;

    % Cumulative distance
    distance_traveled(i) = distance_traveled(max(i - 1, 1)) + sqrt((x - prev_x)^2 + (y - prev_y)^2);

    prev_error = error;

    % Exit if vehicle goes out of bounds
    if y >= y_trajectory(end) + 3
        break;
    end
end

%% Plot Road and Results
% Adjust font size for all plots
fontSize = 30;
tickFontSize = 24; % Font size for axis numbers (ticks)

figure('Position', [100, 100, 1400, 900]); % Larger figure size
hold on;

% Road geometry and vehicle dynamics
fill([x_road_left, fliplr(x_road_right)], [y_road_left, fliplr(y_road_right)], [0.9, 0.9, 0.9], 'EdgeColor', 'none', 'DisplayName', 'Road Surface');
plot(x_road_left, y_road_left, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Road Left Boundary');
plot(x_road_right, y_road_right, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Road Right Boundary');
plot(x_trajectory, y_trajectory, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Desired Trajectory');
plot(x_trajectory, y_road_dashed_left, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Offset Left');
plot(x_trajectory, y_road_dashed_right, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Offset Right');
plot(x_positions, y_positions, 'b-', 'LineWidth', 2, 'DisplayName', 'Car Trajectory');

xlabel('X (meters)', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Y (meters)', 'FontSize', fontSize, 'FontWeight', 'bold');
title({'Z-Road Road Geometry and Vehicle Dynamics', 'Kp = 3.5 Ki = 2.5 Kd = 3.8'}, ...
      'FontSize', fontSize + 2, 'FontWeight', 'bold');
legend('FontSize', fontSize, 'Location', 'best');
axis equal;
grid on;

% Adjust tick font size
ax = gca;
ax.FontSize = tickFontSize;

%% Steering Input vs. Distance Traveled
figure('Position', [100, 100, 1400, 900]); % Larger figure size
plot(distance_traveled, rad2deg(steering_inputs), 'b-', 'LineWidth', 1.5);
title({'Steering Input vs. Distance Traveled - Z Road', '\alpha = 0.25'}, 'FontSize', fontSize + 2, 'FontWeight', 'bold');
xlabel('Distance Traveled (m)', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Steering Angle (degrees)', 'FontSize', fontSize, 'FontWeight', 'bold');
grid on;

% Adjust tick font size
ax = gca;
ax.FontSize = tickFontSize;

%% Speed Control vs. Distance Traveled
figure('Position', [100, 100, 1400, 900]); % Larger figure size
plot(distance_traveled, speed_values, 'r-', 'LineWidth', 1.5);
title('Speed Control vs. Distance Traveled - Z Road', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
xlabel('Distance Traveled (m)', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Speed (m/s)', 'FontSize', fontSize, 'FontWeight', 'bold');
grid on;

% Adjust tick font size
ax = gca;
ax.FontSize = tickFontSize;

%% Position Deviation from Centerline
figure('Position', [100, 100, 1400, 900]); % Larger figure size
plot(distance_traveled, deviations_from_center, 'b-', 'LineWidth', 1.5);
yline(offset, 'r--', 'LineWidth', 1.5, 'DisplayName', '15% Deviation Threshold');
title('Position Deviation from Centerline - Z Road', 'FontSize', fontSize + 2, 'FontWeight', 'bold');
xlabel('Distance Traveled (m)', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Car Deviation (m)', 'FontSize', fontSize, 'FontWeight', 'bold');
legend('Lane Deviation', '15% Deviation Threshold', 'FontSize', fontSize, 'Location', 'best');
grid on;

% Adjust tick font size
ax = gca;
ax.FontSize = tickFontSize;



