clear,clc

%%%% Define physical cam parameters **************[Check these]****************
dcam = 0.5*2.54;     %Crank radius cm
lcam = 1.92*2.54;    %Crank second arm length cm

%%%% Angular position/velocity for constant rotational freq
n = 2000;
freq = 2;                       %Drive shaft rot frequency (Hz)
theta = linspace(0,2*pi,n);     %Angular position of drive shaft/cam
time = linspace(0,1/freq,n);    %Time vector
dt = mean(diff(time));
t_interp = time(1:(n-1))+dt/2;
thetadotmid = diff(theta)./diff(time);
thetadot = interp1(t_interp,thetadotmid,time,'linear','extrap');

%%%% Work backwards from desired plunger velocity profile, generate trapezoidal (impulsive) velocity program
tf = 1/(2*freq);
t1 = 1/(12*freq);
t2 = 5/(12*freq);
[~,ind1] = min(abs(time-t1));
[~,ind2] = min(abs(time-t2));
[~,indf] = min(abs(time-tf));

Hdotc = 24/5*dcam*freq;             %Constant plunger velocity 
Hd1 = Hdotc*time(1:ind1)/t1;
Hd2 = Hdotc*ones(1,ind2-ind1);
Hd3 = Hdotc*(time(indf)-time(ind2+1:indf))/t1;
HM = [time',[-Hd1,-Hd2,-Hd3,Hd1(2:end),Hdotc,Hd2,Hd3]'];

V_max = 0.05; % m/s
SF = 0.01;
Hd =  -Hdotc* tanh(SF * sin(2 * pi * freq * time)) / tanh(SF);
HM2 = [time', Hd'];

params = [lcam,dcam];

plot(HM(:,1),HM(:,2),HM2(:,1),HM2(:,2))

theta0 = 0.01; %pi;
% [tout,ThetaSim] = ode45(@CrankFunc,[0,time],theta0,[],params,HM);
[tout,ThetaSim] = ode89(@CrankFunc,time,theta0,[],params,HM2); % trying 89 for more stiff model solves 
tint2 = (tout(1:end-1)+tout(2:end))/2;
thetadotsim = diff(ThetaSim)./diff(tout);
Thetadotsim = interp1(tint2,thetadotsim,tout,'linear','extrap');%anglular velocity of crank
thetadot2sim = diff(Thetadotsim)./diff(tout);
Thetadot2sim = interp1(tint2,thetadot2sim,tout,'linear','extrap'); %angular acceleration of crank

% denom = (lcam^2-dcam^2*sin(ThetaSim).^2).^(1/2);
% num = dcam*sin(ThetaSim).*cos(ThetaSim);
% HdotCheck = -dcam*Thetadotsim.*(sin(ThetaSim) + num./denom);


%%%% Create non-dim time vector and other properly sized vectors for upload to arduino
na = 200;
tint3 = linspace(0,tout(end),na);
pprog = interp1(tout,ThetaSim,tint3);
vprog0 = interp1(tout,Thetadotsim,tint3);
% aprog0 = interp1(tout,Thetadot2sim,tint3);
fcut = 0.2;              %Cutoff at half the sampling rate
[Bb,Ba] = butter(1,fcut);
vprog = filter(Bb,Ba,vprog0);
% aprog = filter(Bb,Ba,aprog0);
aprog0 = diff(vprog)./diff(tint3);
aprog = interp1((tint3(1:end-1)+tint3(2:end))/2,aprog0,tint3);

denom = (lcam^2-dcam^2*sin(pprog).^2).^(1/2);
num = dcam*sin(pprog).*cos(pprog);
HdotCheck = -dcam*vprog.*(sin(pprog) + num./denom);

figure(3)
plot(tout,ThetaSim/pi,'linewidth',1.5)
xlabel('Time (s)','fontname','times','fontsize',16)
ylabel('Crankshaft Angular Position (rad/\pi)','fontname','times','fontsize',16)
title('Impulsive Program','fontname','times','fontsize',16)

figure(4)
plot(tout,Thetadotsim,'linewidth',1.5)
hold on
plot(tint3,vprog,'o')
xlabel('Time (s)','fontname','times','fontsize',16)
ylabel('Crankshaft Angular Velocity \omega (rad/s)','fontname','times','fontsize',16)
title('Impulsive Program','fontname','times','fontsize',16)

figure(5)
plot(tint3,HdotCheck,'linewidth',1.5)
xlabel('Time (s)','fontname','times','fontsize',16)
ylabel('Plunger Velocity (m/s)','fontname','times','fontsize',16)
title('Impulsive Program','fontname','times','fontsize',16)

figure(6)
plot(tout,Thetadot2sim,'linewidth',1.5)
hold on
plot(tint3,aprog,'o')
xlabel('Time (s)','fontname','times','fontsize',16)
ylabel('Crankshaft Angular Acceleration (rad/s)','fontname','times','fontsize',16)
title('Impulsive Program','fontname','times','fontsize',16)
set(gca,'ylim',[-1000,1000])


%%%% For writing .txt file to SD card for Dodisicus
timeND = linspace(0,1000,na);
pprogND = pprog*1000/(2*pi);        %Position at each non-dim time, (units are 1/1000th of a rotation)
vprogND = vprog*1000/(2*pi*freq);   
aprogND = aprog*1000/(2*pi*freq^2);
% FName = 'F:\VelProg.txt';
% FName = 'VelProg_SF_03_2hz.txt';
% C = [timeND;pprogND;vprogND;aprogND];
% C = int16(C);
% writematrix(C,FName)


