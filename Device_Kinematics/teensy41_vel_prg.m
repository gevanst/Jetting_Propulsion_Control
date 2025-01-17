clear,clc

res = 4096; %encoder resolution
f_j = 0.5; %jetting frequecny in hz
ang_vel = 2*pi*f_j; %rad/s
num_p = 4096/16; % number of points in vel program

pos_p = round(linspace(0,res,num_p));
vel_p = ang_vel*(res/(2*pi))*ones(1,num_p); %convert from rad/s to encoder count/s
acc_p = zeros(1,num_p);

pos_p = round(linspace(0,res,num_p));
time = linspace(0,1/f_j,num_p);
vel_p = 2048 + (((ang_vel*sin(2*pi*f_j*time)))*(res/(2*pi)));
acc_p = [6433, diff(vel_p) / (time(2) - time(1))];

figure;
subplot(3,1,1)
plot(pos_p)
xlabel('position')

subplot(3,1,2)
plot(vel_p)
xlabel('velocity')

subplot(3,1,3)
plot(acc_p)
xlabel('acceleration')


prog = [pos_p' round(vel_p') round(acc_p')];

output_file = 'prog_05Hz_256pts_OSC.txt';
writematrix(prog, output_file, 'Delimiter', ',', 'WriteMode', 'overwrite');
