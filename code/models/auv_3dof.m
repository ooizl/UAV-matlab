function auv_3dof(block)
% remus_dynamics.m     e.anderlini@ucl.ac.uk     07/05/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is a Level-2 Matlab S-function for the modelling of the dynamics of
% an AUV in 3 DOF.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    setup(block);
end

%% Set up the block:
function setup(block)
    % Register number of input and output ports:
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 2;

    % Setup functional port properties to dynamically inherited:
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Size the input ports correctly:
    block.InputPort(1).Dimensions        = 2;    % control input vector
    % Specify whether there is direct feedthrough:
    block.InputPort(1).DirectFeedthrough = false;
    block.InputPort(1).SamplingMode = 'Sample';
    
    % Size the output port correctly:
    block.OutputPort(1).Dimensions   = 8;        % continuous states
    % Set the output ports' sampling mode:
    block.OutputPort(1).SamplingMode = 'Sample';
    % Size the output port correctly:
    block.OutputPort(2).Dimensions   = 3;        % inertial velocity
    % Set the output ports' sampling mode:
    block.OutputPort(2).SamplingMode = 'Sample';
    
    % Register the continuous states:
    block.NumContStates = 8;
    
    % Define the number of parameters:
    block.NumDialogPrms = 8;

    % Set block sample time to continuous:
    block.SampleTimes = [0,0];
    % Set the block simStateCompliance to default:
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods:
    block.RegBlockMethod('PostPropagationSetup', @PostPropagationSetup);
    block.RegBlockMethod('InitializeConditions', @InitialConditions);
    block.RegBlockMethod('Outputs',              @Output);    
    block.RegBlockMethod('Derivatives',          @Derivative);
end

%% Set up the dynamic work vector:
function PostPropagationSetup(block)
    % Setup Dwork:
    block.NumDworks                = 1;
    block.Dwork(1).Name            = 'inertial_velocity'; 
    block.Dwork(1).Dimensions      = 3;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
end

%% Initialisitation:
function InitialConditions(block)
    % Initialise the continuous states:
    block.ContStates.Data = block.DialogPrm(8).Data; % initial conditions
    % Initialise the dynamic work vector:
    cos_theta = block.DialogPrm(8).Data(3);
    sin_theta = block.DialogPrm(8).Data(3);
    J = [cos_theta,sin_theta,0;-sin_theta,cos_theta,0;0,0,1];
    block.Dwork(1).Data = J*block.DialogPrm(8).Data(1:3);
end

%% Output the continuous states:
function Output(block)   
    % Output the continuous states:
    block.OutputPort(1).Data = block.ContStates.Data;
    % Output the inertial velocity:
    block.OutputPort(2).Data = block.Dwork(1).Data;
end

%% Compute the derivative of the continuous states:
% dx/dt = f(x);
function Derivative(block)
    % Extract the input parameters:
    prp  = block.DialogPrm(1).Data;
    act  = block.DialogPrm(2).Data;
    rbd  = block.DialogPrm(3).Data;
    X    = block.DialogPrm(4).Data;
    Z    = block.DialogPrm(5).Data;
    M    = block.DialogPrm(6).Data;
    Ainv = block.DialogPrm(7).Data;
    
    % Extract the input vector:
    in = block.InputPort(1).Data;
    
    % Extract the continuous states:
    x = block.ContStates.Data;
    
    % To make the code clearer, copy all desired parameters and states:
    % Input vector:
    Q     = in(1);
    delta = in(2);
    
    % State vector:
    theta = x(3);
    u     = x(4);
    w     = x(5);
    q     = x(6);
    n     = x(7);
    up    = x(8);
    
    % Parameters:
    Jm  = prp(1);
    Kn  = prp(2);
    mf  = prp(3);
    CDl = prp(4);
    CDq = prp(5);
    wp  = prp(6);
    tp  = prp(7);
    Tnn  = act(1);
    Qnn  = act(2);
    Zuud = act(3);
    Muud = act(4);
    m    = rbd(1);
    zg   = rbd(3);
    W    = rbd(4);
    WB   = rbd(5);
    Xuu = X(1);
    Xwq = X(2);
    Xqq = X(3);
    Zuw = Z(1);
    Zuq = Z(2);
    Zww = Z(3);
    Zqq = Z(4);
    Muw = M(1);
    Muq = M(2);
    Mww = M(3);
    Mqq = M(4);
    
    % Speed up the calculations: pre-compute recurring terms:
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    n_n = n*abs(n);
    u_u = u*abs(u);
    u2d = u^2*delta;
    w_w = w*abs(w);
    q_q = q*abs(q);
    q2 = q^2;
    u_w = u*w;
    u_q = u*q;
    w_q = w*q;
    
    % Compute the right-hand side of the position derivative vector:
    J = [cos_theta,sin_theta,0;-sin_theta,cos_theta,0;0,0,1];
    dxdt1 = J*[u;w;q];
    
    % Compute the right-hand side of the velocity derivative vector:
    % [du/dt;dw/dt;dq/dt] = f_2(t);
    f = [-WB*sin_theta+Xuu*u_u+(Xwq-m)*w_q+Xqq*q2+(1-tp)*Tnn*n_n;...
        WB*cos_theta+Zuw*u_w+(Zuq+m)*u_q+Zww*w_w+Zqq*q_q+m*zg*q2+Zuud*u2d;...
        -W*zg*sin_theta+Muw*u_w+Muq*u_q+Mww*w_w-m*zg*w_q+Mqq*q_q+Muud*u2d];
    dxdt2 = Ainv*f;
    
    % Compute the right-hand side of the propulsion vector:
    % [dn/dt;du_p/dt] = f3(t);
    dxdt3 = [(Q-Kn*n-Qnn*n_n)/Jm;...
        (Tnn*n_n-CDl*up-CDq*abs(up)*(u-(1-wp)*u))/mf];
    
    % Store the inertial velocity as a work vector:
    block.Dwork(1).Data = dxdt1;
    
    % Compute the derivative vector dx/dt:
    block.Derivatives.Data = [dxdt1;dxdt2;dxdt3];
end