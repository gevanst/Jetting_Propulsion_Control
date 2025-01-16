clear, clc

filename = "Test_Log_05Hz_256pts_5.txt";
data = readtable(filename, 'Delimiter', ',', 'ReadVariableNames', true);
disp('Available variable names in the CSV:');
disp(data.Properties.VariableNames);
%Time (ms), Position (count), Velocity (count/s), Acceleration (count/s2), Desired Vel, Desired Acc, control signal, motorCurrent (raw)

figure(1);
subplot(3,1,1);
plot(data.Time_ms_/1000,data.Position_count_,'-x')
ylabel('Encoder Position (count)');
grid on;

subplot(3,1,2); 
plot(data.Time_ms_/1000,data.Velocity_count_s_,data.Time_ms_/1000,data.DesiredVel)
ylabel('velocity (counts/s)')
legend('measured v', 'desired v')
grid on;

subplot(3,1,3); 
plot(data.Time_ms_/1000, data.Acceleration_count_s2_, data.Time_ms_/1000, data.DesiredAcc)
ylabel('acc (counts/s^2)')
xlabel('Time (s)');
legend('measured a','desired a')
grid on;

figure(2);
subplot(2,1,1)
plot(data.Time_ms_/1000, data.controlSignal)
xlabel('Time (s)')
ylabel('control signal')
grid on;

subplot(2,1,2)
plot(data.Time_ms_/1000, data.motorCurrent_raw_)
ylabel('Motor Current Raw ADC')
grid on;


