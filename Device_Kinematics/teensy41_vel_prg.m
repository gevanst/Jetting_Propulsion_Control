clear,clc

res = 4096; %encoder resolution
f_j = 0.5; %jetting frequecny in hz
ang_vel = 2*pi*f_j; %rad/s
num_p = 4096/16; % number of points in vel program

pos_p = round(linspace(0,res,num_p));
vel_p = ang_vel*(res/(2*pi))*ones(1,num_p); %convert from rad/s to encoder count/s
acc_p = zeros(1,num_p);

prog = [pos_p' vel_p' acc_p'];

output_file = 'prog_05Hz_256pts.txt';
writematrix(prog, output_file, 'Delimiter', ',', 'WriteMode', 'overwrite');
