clear, clc

filename = "Test_Log_05Hz_256pts_OSC_83.txt";
data = readtable(filename, 'Delimiter', ',', 'ReadVariableNames', true);
disp('Available variable names in the CSV:');
disp(data.Properties.VariableNames);
%Time (ms), Position (count), Velocity (count/s), Acceleration (count/s2), Desired Vel, Desired Acc, control signal, motorCurrent (raw)

figure(1);
subplot(5,1,1);
plot(data.Time_ms_/1000,data.Position_count_)
ylabel('Encoder Position (count)');
title(':Kp=0.10,Kd=0.00,Ki=0.00');
grid on;

subplot(5,1,2); 
plot(data.Time_ms_/1000,data.Velocity_count_s_,data.Time_ms_/1000,data.DesiredVel)
ylabel('velocity (counts/s)')
legend('measured v', 'desired v')
ylim([-200 4096])
grid on;

subplot(5,1,3); 
plot(data.Time_ms_/1000, data.Acceleration_count_s2_, data.Time_ms_/1000, data.DesiredAcc)
ylabel('acc (counts/s^2)')
xlabel('Time (s)');
legend('measured a','desired a')
ylim([-10000 10000])
grid on;

subplot(5,1,4)
plot(data.Time_ms_/1000, data.controlSignal)
xlabel('Time (s)')
ylabel('control signal')
ylim([5000 25000])
grid on;

subplot(5,1,5)
plot(data.Time_ms_/1000, data.motorCurrent_raw_)
ylabel('Motor Current Raw ADC')
grid on;


