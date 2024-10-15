clc,clear,close all
% Step 1: Load the data from the text file
fileID = fopen('gp_task_1_data.txt', 'r');
% Assuming the file has two columns: [distance,yaw_angle]
data = textscan(fileID, '%f,%f', 'Delimiter', '\n');
fclose(fileID);

% Extract yaw angle and distance
distances = data{1};   % In meters or the unit of your data
yaw_angles = data{2};  % In degrees

subplot(311)
plot(distances,'r','linewidth',2);grid on
title('distance')
subplot(312)
plot(yaw_angles,'b','linewidth',2);grid on
title('angle')

% Step 2: Initialize position
x = 0;  % Starting x-coordinate
y = 0;  % Starting y-coordinate

% Convert yaw angles from degrees to radians for trigonometric calculations
yaw_angles_rad = deg2rad(yaw_angles);

% Initialize arrays to store path coordinates
x_coords = zeros(length(distances), 1);
y_coords = zeros(length(distances), 1);

% Step 3: Calculate the car's path
for i = 1:length(distances)
    % Calculate change in position (delta x, delta y)
    delta_x = distances(i) * cos(yaw_angles_rad(i));
    delta_y = distances(i) * sin(yaw_angles_rad(i));
    
    % Update current position
    x = x + delta_x;
    y = y + delta_y;
    
    % Store the coordinates
    x_coords(i) = x;
    y_coords(i) = y;
end

% Step 4: Plot the car's path
subplot(313)
plot(x_coords, y_coords,'linewidth',2);  % Line plot with markers
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Car Path Based on Yaw Angle and Distance');
grid on;
axis equal;  % Ensures equal scaling on both axes