% Load the data into MATLAB
% Replace 'data.csv' with the path to your CSV file
filename = 'DATALOG6.txt';
data = readtable(filename);

% Extract columns from the table
Time = data{:, 'Time_ms_'}; % Time in milliseconds
MotorCurrent = data{:, 'MotorCurrent'}; % Motor current
Position = data{:, 'Position'}; % Position
Velocity = data{:, 'Velocity'}; % Velocity
Acceleration = data{:, 'Acceleration'}; % Acceleration

% Create a figure with subplots
figure;

% Plot Motor Current over Time
subplot(4, 1, 1); 
plot(Time, MotorCurrent, 'b');
xlabel('Time (ms)');
ylabel('Motor Current (mA)');
title('Motor Current Over Time');
grid on;

% Plot Position over Time
subplot(4, 1, 2);
plot(Time, Position, 'r');
xlabel('Time (ms)');
ylabel('Position');
title('Position Over Time');
grid on;

% Plot Velocity over Time
subplot(4, 1, 3);
plot(Time, Velocity, 'g');
xlabel('Time (ms)');
ylabel('Velocity');
title('Velocity Over Time');
grid on;

% Plot Acceleration over Time
subplot(4, 1, 4);
plot(Time, Acceleration, 'm');
xlabel('Time (ms)');
ylabel('Acceleration');
title('Acceleration Over Time');
grid on;

% Adjust layout for better readability
tight_layout;
