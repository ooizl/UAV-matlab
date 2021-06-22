% setupDynamicModel3dof.m     e.anderlini@ucl.ac.uk     07/05/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is run to generate the parameters for the simulation of an
% AUV in 3 DOF.
%
% The parameters of the model for the REMUS 100 are taken from
% Hall & Anstee (2011). "Trim Calculation Methods for a Dynamical Model of
% the REMUS 100 Autonomous Underwater Vehicle", DSTO-TR-2576.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Input the parameters for the model:
% Propeller values:
prop.mf = 0.37958;     % [kg]
prop.Kn = 0.5;         % [kg m2 s-1]
prop.Jm = 1;           % [kg m2]
prop.wp = 0.2;
prop.tp = 0.1;
prop.l = 6.604;
prop.q = 16.51;

% Actuation:
%act.Tnn = 6.279e-4;   % [kg m rad-2]
%act.Qnn = -1.121e-5;  % [kg m2 rad-2]
%act.Zuuds = -9.64;    % [kg m-1 rad-1]
%act.Muuds = -6.15;    % [kg rad-1]

% Inertia values:
ine.m = 0.37958;        % [kg]
ine.W = 299;          % [N]
ine.WB = 0;   %-0.34*9.81;  % [N]
ine.zg = 0.0043;      % [m]
ine.Iyy = 3.45;       % [kg m2]

% Surge cofficients:
surge.Xud = -0.93;    % [kg]
surge.Xuu = -2.972;   % [kg m-1]
surge.Xwq = -35.5;    % [kg]
surge.Xqq = -1.93;    % [kg m rad-2]

% Heave coefficients:
heave.Zwd = -35.5;    % [kg]
heave.Zqd = -1.93;    % [kg m rad-1]
heave.Zuw = -28.6;    % [kg m-1]
heave.Zuq = -5.22;    % [kg rad-1]
heave.Zww = -1310;    % [kg m-1]
heave.Zqq = -0.632;   % [kg m rad-2]

% Pitch coefficients:
pitch.Mwd = -1.93;    % [kg m]
pitch.Mqd = -4.88;    % [kg m2 rad-1]
pitch.Muw = 24;       % [kg]
pitch.Muq = -2;       % [kg m rad-1]
pitch.Mww = 3.18;     % [kg]
pitch.Mqq = -188;     % [kg m2 rad-2]

%% Assemble the desired vectors:
propulsion = [prop.Jm,prop.Kn,prop.mf,prop.l,prop.q,prop.wp,prop.tp];
%actuators = [act.Tnn,act.Qnn,act.Zuuds,act.Muuds];
rigid_body = [ine.m,ine.Iyy,ine.zg,ine.W,ine.WB];
X = [surge.Xuu,surge.Xwq,surge.Xqq];
Z = [heave.Zuw,heave.Zuq,heave.Zww,heave.Zqq];
M = [pitch.Muw,pitch.Muq,pitch.Mww,pitch.Mqq];

%% Calculate the inverse of the two mass matrix subcomponents:
% Define the two mass matrix components:
A1 = [ine.m-surge.Xud,0,ine.m*ine.zg;...
    0,ine.m-heave.Zwd,-heave.Zqd;...
    ine.m*ine.zg,-pitch.Mwd,ine.Iyy-pitch.Mqd];
A2 = [prop.Jm,0;0,prop.mf];

% Compute the inverse matrices:
A1i = pinv(A1);
A2i = pinv(A2); 

%% Save the required data to file:
auv.propulsion = propulsion;
%auv.actuators  = actuators;
auv.rigid_body = rigid_body;
auv.X = X;
auv.Z = Z;
auv.M = M;
auv.Ainv = A1i;
save('data/remus3dof.mat','auv');