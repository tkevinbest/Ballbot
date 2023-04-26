clear
close all

% This function uses matlab codegen to compile the MPC run method. 

% These functions are just dummy variables so matlab can get the sizes
% right

% Configure controller
dt = 0.005; % Real time sample rate
timeHorizon = 2.5; 
N_horizon = 51; % Nodes
t_horizon = linspace(0, timeHorizon, N_horizon); 
T_simulation = 3; % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Define obstacle
pObs = [.25;.55];
rObs = .1;

Q = diag([50, 1,5,1]); 
R = 1; 
curZ = zeros(4,1); 
zExpected = repmat(curZ, 1, N_horizon); 
uExpected = zeros(1,N_horizon); 
MPCconfig = Control.MPC.setup(N_horizon, timeHorizon, t_sim, zExpected, uExpected);

zDesiredTraj = repmat(curZ, 1, MPCconfig.N_horizon); 
uDesiredTraj = repmat(0, 1, MPCconfig.N_horizon); 

% Call codegen and make magic
codegen -report +Control/+MPC/Run.m -args {Q, R, curZ, zDesiredTraj, uDesiredTraj, MPCconfig, pObs, rObs}

