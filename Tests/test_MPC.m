clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.;

% Define desired states
qdes = [desX/rk;0];
qdot_des = [0; 0];
zDesired = interleave2(qdes, qdot_des, 'row');
uDesired = 0;

% Define initial conditions
q0 = [-.6/rk;0];
qdot0 = [0;0];
z0 = interleave2(q0, qdot0, 'row'); 

% Define obstacle
pObs = [.25;.6];
rObs = .05;

%% Run MPC around nominal state
% Configure controller
dt = 0.05; % Real time sample rate
timeHorizon = 2.5; 
N_horizon = 51; % Nodes
t_horizon = linspace(0, timeHorizon, N_horizon); 

% Configure simulation
T_simulation = 3; % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Configure MPC 
zExpected = repmat(z0, 1, N_horizon); 
uExpected = zeros(1,N_horizon); 
MPCconfig = Control.MPC.setup(N_horizon, timeHorizon, t_sim, zExpected, uExpected);

% Allocate space to save off the trajectory
u_store = NaN(1, length(t_sim)); 
z_store = NaN(4, length(t_sim)); 

% Loop simulation
curZ = z0; 
zstarHist = [];
ustarHist = [];
for ix = 1:length(t_sim)
    % Get the current time
    curTime = t_sim(ix); 

    % Grab current section of desired trajectory
    % For a constant setpoint, just repeat desired 
    zDesiredTraj = repmat(zDesired, 1, MPCconfig.N_horizon); 
    uDesiredTraj = repmat(uDesired, 1, MPCconfig.N_horizon); 

    % Solve optimal control
    Q = diag([50, 1,25,1]); 
    R = .05;
    [curU, zstar, ustar, MPCconfig, MPCfailed] = Control.MPC.Run(Q, R, curZ, zDesiredTraj, uDesiredTraj, MPCconfig, pObs, rObs);

    zstarHist(:,:,ix) = zstar; 
    ustarHist(:,ix) = ustar'; 
    tHist(:,ix) = curTime + t_horizon;

    % Run simulation with said control 
    forceFunc = @(t,z) curU;  
    q0 = curZ([1,3],:);
    qdot0 = curZ([2,4],:);
    [t,q, qdot, z, ~] = Ballbot.runSimulation(q0, qdot0, forceFunc, curTime + [0,dt], false);

    % Update initial conditions
    curZ = z(end,:)';

    % Store the trajectories
    u_store(ix) = curU; 
    z_store(:,ix) = z(1,:)'; 

%     Plot progress
%     Control.MPC.plotTrajectoriesAndPrediction(t_sim(1:ix), z_store(:,1:ix)', u_store(1:ix), zstarHist, ustarHist, tHist); 
%     drawnow; 

    if MPCfailed
        t_sim = t_sim(1:ix-1); 
        z_store = z_store(:,1:ix-1); 
        u_store = u_store(1:ix-1); 
        break;
    end
end

% Plot the trajectories
figure()
Ballbot.plotTrajectories(t_sim, z_store', u_store); 

%% Animate the solution
q_store = [z_store(1,:); z_store(3,:)];
Ballbot.animate(t_sim, q_store', 'TestMPC.mp4', pObs, rObs);




