% This is a script that tests the code of numerical integration of
% orientation of a quadcopter.
%
% Author:      Haohan Zhang
% Affiliation: ROAR Lab @ Columbia
% Date:        1/23/2016

clear
clc
close all

% comment one of the next two lines to declare your rotation representation
id = 'space-three 1-2-3';
%id = 'body-two 1-2-1';  

% initialization
t = 10; % time length: 10s
ic_b = [pi/6 pi/4 pi/3]; % initial conditions, in body frame
options = odeset('RelTol',1e-4,'AbsTol',1e-8*ones(1,3)); % solver options

% call the main function to integrate angles, plot rotations and animate
% the quadcopter using 'drawnow'.
[T,Y] = OrientationIntegration(id,t,ic_b,options);






