clear,clc
% Specify the CSV file to read
filename = 'Log_encoder_vel2.txt'; % Replace with your actual file name

% Read the CSV file and display available column names
data = readtable(filename, 'Delimiter', ',', 'ReadVariableNames', true);
disp('Available variable names in the CSV:');
disp(data.Properties.VariableNames);

% Manually correct the variable names based on actual CSV header
time_var = data.Properties.VariableNames{1}; % Assume first column is time
position_var = data.Properties.VariableNames{2}; % Assume second column is position
vel_var = data.Properties.VariableNames{3};

% Extract time and position data dynamically
time_ms = data.(time_var);
position = data.(position_var);
vel = data.(vel_var);

% Convert time from milliseconds to seconds
time_seconds = time_ms / 1000;

% Plot the data
figure;
subplot(2,1,1);
plot(time_seconds,position,'-x')
xlabel('Time (s)');
ylabel('Encoder Position (count)');
grid on;

subplot(2,1,2); 
plot(time_seconds,vel,'-o')
ylabel('velocity (counts/s)')

% Display basic statistics
fprintf('Data Stats:\n');
fprintf('Total Records: %d\n', height(data));
fprintf('Start Time: %.2f s\n', time_seconds(1));
fprintf('End Time: %.2f s\n', time_seconds(end));
fprintf('Position Range: %d to %d\n', min(position), max(position));
fprintf('max velocity: %d\n',max(vel));
