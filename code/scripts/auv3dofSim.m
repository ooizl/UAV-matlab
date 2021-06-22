% auv3dofSim.m     e.anderlini@ucl.ac.uk     07/05/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of a REMUS 100 AUV in 3 degrees of 
% freedom under simple PID control.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Initialization:
% Load the previously generated parameters (see setUpDynamicModel3dof.m):
load('remus3dof.mat');

% Define the time step and total simulation time:
tStep = 0.01;       % time step length (s)
tEnd  = 100;        % end time (s)

% Set the initial conditions:
ics = zeros(8,1);

%% Control settings:
% Desired paramters:
u_d = 1.5;    % desired forward speed (m/s)
z_d = 1;    % desired depth (m)

% PID control settings:
kp_u = 200;
ki_u = 5;
kd_u = 10;
kp_z = 50;
ki_z = 3;
kd_z = 6;
kp_t = 20;
ki_t = 2;
kd_t = 4;

% Set the time step for the PID controller:
dtPID = 0.1;

% Define the saturation limits:
lim.torque = 100;
lim.angle  = deg2rad(15);

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'auv3dof';
% Load the Simulink file:vek
load_system(sfile);

%% Run the first shot:
sout = sim(sfile,'StopTime',num2str(tEnd));

%% Close the Simulink file:
close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('states').Values.Data;
v = sout.get('logsout').getElement('velocity').Values.Data;
u = sout.get('logsout').getElement('input').Values.Data;
p = sout.get('logsout').getElement('angle').Values.Data;

% Plot the AUV's motions:
plotMotions(t,x(:,1:3),v);
% Plot the AUV's control input:
plotControl(t,u(:,1),rad2deg(u(:,2)),x(:,7));