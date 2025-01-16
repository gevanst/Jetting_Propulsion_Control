clear,clc

filename = 'Log_encoder_vel3.txt';

data = readtable(filename, 'Delimiter', ',', 'ReadVariableNames', true);
disp('Available variable names in the CSV:');
disp(data.Properties.VariableNames);

time_var = data.Properties.VariableNames{1};
position_var = data.Properties.VariableNames{2}; 
vel_var = data.Properties.VariableNames{3};
acc_var = data.Properties.VariableNames{4};


time_ms = data.(time_var);
position = data.(position_var);
vel = data.(vel_var);
acc = data.(acc_var);
time_seconds = time_ms / 1000;

figure;
subplot(3,1,1);
plot(time_seconds,position,'-x')
xlabel('Time (s)');
ylabel('Encoder Position (count)');
grid on;

subplot(3,1,2); 
plot(time_seconds,vel,'-o')
ylabel('velocity (counts/s)')

subplot(3,1,3); 
plot(time_seconds,acc,'-o')
ylabel('acc (counts/s^2)')

fprintf('Data Stats:\n');
fprintf('Total Records: %d\n', height(data));
fprintf('Start Time: %.2f s\n', time_seconds(1));
fprintf('End Time: %.2f s\n', time_seconds(end));
fprintf('Position Range: %d to %d\n', min(position), max(position));
fprintf('max velocity: %d\n',max(vel));
