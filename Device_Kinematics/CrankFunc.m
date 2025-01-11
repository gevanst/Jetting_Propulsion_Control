function [thetadot] = CrankFunc(t,theta,params,HM)

% Function which calculates the necessary crank rotational velocity based
% on a provided plunger desired linear velocity and the current position of 
% the crank. The function is based on a geometric model (no deformation)

% Inputs -  t       - The current value of time
%           theta   - The current crank angular position (rad)
%           params  - Crank geometry parameters
%           HM      - Desired plunger height program, 1st column time vector, 2nd column velocity vector corresponding to time values in the 1st collumn 

% Outputs - thetadot - Necessary rotational velocity of the crank (rad/s)

lcam = params(1);
dcam = params(2);

Hdot = interp1(HM(:,1),HM(:,2),t);


denom = (lcam^2-dcam^2*sin(theta).^2).^(1/2);
num = dcam*sin(theta).*cos(theta);
thetadot = Hdot/(-dcam*(sin(theta) + num./denom));