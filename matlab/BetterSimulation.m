function BetterSimulation(block)
setup(block);

function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 11;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).SamplingMode = 'Sample';
block.InputPort(2).DirectFeedthrough = false;

% Override output port properties
% Angles (only actual output)
block.OutputPort(1).Dimensions       = 1;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(2).Dimensions       = 1;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';
block.OutputPort(3).Dimensions       = 1;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';
% First angle derivatives
block.OutputPort(4).Dimensions       = 1;
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';
block.OutputPort(5).Dimensions       = 1;
block.OutputPort(5).DatatypeID  = 0; % double
block.OutputPort(5).Complexity  = 'Real';
block.OutputPort(5).SamplingMode = 'Sample';
block.OutputPort(6).Dimensions       = 1;
block.OutputPort(6).DatatypeID  = 0; % double
block.OutputPort(6).Complexity  = 'Real';
block.OutputPort(6).SamplingMode = 'Sample';
% Rotor speeds
block.OutputPort(7).Dimensions       = 1;
block.OutputPort(7).DatatypeID  = 0; % double
block.OutputPort(7).Complexity  = 'Real';
block.OutputPort(7).SamplingMode = 'Sample';
block.OutputPort(8).Dimensions       = 1;
block.OutputPort(8).DatatypeID  = 0; % double
block.OutputPort(8).Complexity  = 'Real';
block.OutputPort(8).SamplingMode = 'Sample';
% Second angle derivatives
block.OutputPort(9).Dimensions       = 1;
block.OutputPort(9).DatatypeID  = 0; % double
block.OutputPort(9).Complexity  = 'Real';
block.OutputPort(9).SamplingMode = 'Sample';
block.OutputPort(10).Dimensions       = 1;
block.OutputPort(10).DatatypeID  = 0; % double
block.OutputPort(10).Complexity  = 'Real';
block.OutputPort(10).SamplingMode = 'Sample';
block.OutputPort(11).Dimensions       = 1;
block.OutputPort(11).DatatypeID  = 0; % double
block.OutputPort(11).Complexity  = 'Real';
block.OutputPort(11).SamplingMode = 'Sample';

block.NumContStates = 8;

% Register parameters
block.NumDialogPrms     = 8;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Derivatives', @Derivatives);

%end setup


function Start(block)
deg = pi / 180;
phi0 = block.DialogPrm(1).Data * deg;
eps0 = block.DialogPrm(2).Data * deg;
lamb0 = block.DialogPrm(3).Data * deg;
dphi0 = block.DialogPrm(4).Data * deg;
deps0 = block.DialogPrm(5).Data * deg;
dlamb0 = block.DialogPrm(6).Data * deg;
wf0 = block.DialogPrm(7).Data;
wb0 = block.DialogPrm(8).Data;
block.ContStates.Data = [phi0, eps0, lamb0, dphi0, deps0, dlamb0, wf0, wb0];
%end Start


function Outputs(block)

block.OutputPort(1).Data = block.ContStates.Data(1);
block.OutputPort(2).Data = block.ContStates.Data(2);
block.OutputPort(3).Data = block.ContStates.Data(3);
block.OutputPort(4).Data = block.ContStates.Data(4);
block.OutputPort(5).Data = block.ContStates.Data(5);
block.OutputPort(6).Data = block.ContStates.Data(6);
block.OutputPort(7).Data = block.ContStates.Data(7);
block.OutputPort(8).Data = block.ContStates.Data(8);

dx = ComputeDerivativesFromBlockData(block);

block.OutputPort(9).Data = dx(4);
block.OutputPort(10).Data = dx(5);
block.OutputPort(11).Data = dx(6);

%end Outputs


function Derivatives(block)

block.Derivatives.Data = ComputeDerivativesFromBlockData(block);

%end Derivatives

function dx = ComputeDerivativesFromBlockData(block)
vf = block.InputPort(1).Data;
vb = block.InputPort(2).Data;

phi = block.ContStates.Data(1);
eps = block.ContStates.Data(2);
lamb = block.ContStates.Data(3);
dphi = block.ContStates.Data(4);
deps = block.ContStates.Data(5);
dlamb = block.ContStates.Data(6);
wf = block.ContStates.Data(7);
wb = block.ContStates.Data(8);

x = [phi, eps, lamb, dphi, deps, dlamb, wf, wb];
u = [vf, vb];
dx = system_f(x, u)';


