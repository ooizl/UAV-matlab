function auv_6dof(block)
    setup(block);
  
end


function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 8;
  block.NumOutputPorts = 6;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  % Override the output port properties.
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';

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
  
  f1 = block.InputPort(1).Data;
  f2 = block.InputPort(2).Data;
  f3 = block.InputPort(3).Data;
  f4 = block.InputPort(4).Data;
  m1 = block.InputPort(4).Data;
  m2 = block.InputPort(4).Data;
  m3 = block.InputPort(4).Data;
  m4 = block.InputPort(4).Data;
  
  block.OutputPort(1).Data = w1^2 * 2; %kf = 2
  block.OutputPort(2).Data = w2^2 * 2;
  block.OutputPort(3).Data = w3^2 * 2;
  block.OutputPort(4).Data = w4^2 * 2;
  block.OutputPort(5).Data = w1^2 * 3; %km = 3
  block.OutputPort(6).Data = w1^2 * 3;
  block.OutputPort(7).Data = w1^2 * 3;
  block.OutputPort(8).Data = w1^2 * 3;
end


function Terminate(block)


end
 
