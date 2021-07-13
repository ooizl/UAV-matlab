     function transformation(block)
    setup(block);
  
end


function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 4;
  block.NumOutputPorts = 3;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

   % Size the input ports correctly:
  block.InputPort(1).Dimensions        = 4;    % control input vector
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(3).Dimensions        = 1;
  block.InputPort(4).Dimensions        = 1;
 
  
  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(3).DatatypeID  = 0;  % double
  block.InputPort(3).Complexity  = 'Real';
  block.InputPort(4).DatatypeID  = 0;  % double
  block.InputPort(4).Complexity  = 'Real';

  % Specify whether there is direct feedthrough:
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode = 'Sample';
  block.InputPort(2).DirectFeedthrough = false;
  block.InputPort(2).SamplingMode = 'Sample';
  block.InputPort(3).DirectFeedthrough = false;
  block.InputPort(3).SamplingMode = 'Sample';
  block.InputPort(4).DirectFeedthrough = false;
  block.InputPort(4).SamplingMode = 'Sample';
  
  % Size the output port correctly:
  block.OutputPort(1).Dimensions   = 1;        % x
  % Set the output ports' sampling mode:
  block.OutputPort(1).SamplingMode = 'Sample';
  % Size the output port correctly:
  block.OutputPort(2).Dimensions   = 1;        % y
  % Set the output ports' sampling mode:
  block.OutputPort(2).SamplingMode = 'Sample';
    % Size the output port correctly:
  block.OutputPort(3).Dimensions   = 1;        % z
  % Set the output ports' sampling mode:
  block.OutputPort(3).SamplingMode = 'Sample';
  
  % Override the output port properties.
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';
  
  % Register the parameters.
  block.NumDialogPrms     = 0;
  
  block.SampleTimes = [0 0];
  
  block.OperatingPointCompliance = 'Default';

  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  block.RegBlockMethod('Outputs', @Outputs);
  block.RegBlockMethod('Terminate', @Terminate);
end




function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  for i = 1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode  = fd;
  end
end


function Outputs(block)
  
  f1 = block.InputPort(1).Data(1);
  f2 = block.InputPort(1).Data(2);
  f3 = block.InputPort(1).Data(3);
  f4 = block.InputPort(1).Data(4);
  phi = block.InputPort(2).Data;
  theta = block.InputPort(3).Data;
  psi = block.InputPort(4).Data;
  %The rotational transformation matrix
  R = [cos(psi)*cos(theta) -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi) sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
       sin(psi)*cos(theta) -cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi) -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
       -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
  %Body frame
  fx =0;
  fy =0;
  fz = f1+f2+f3+f4;
  
  Fb=[fx;fy;fz];
  %inertia frame
  Fi = R * Fb;
  
  block.OutputPort(1).Data = Fi(1); %FX
  block.OutputPort(2).Data = Fi(2); %FY
  block.OutputPort(3).Data = Fi(3); %FZ
 
end


function Terminate(block)


end
 