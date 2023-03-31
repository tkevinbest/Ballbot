clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.0;

% Define desired states
qdes = [desX/rk;0];
qdot_des = [0; 0];
zDesired = interleave2(qdes, qdot_des, 'row');
uDesired = 0;

% Define initial conditions
q0 = [0;-pi/6];
qdot0 = [0;0];
z0 = interleave2(q0, qdot0, 'row'); 

%% Run MPC around nominal state
% Configure controller
dt = 0.05; % Real time sample rate
timeHorizon = 3; 

% Configure simulation
T_simulation = 4; % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Configure MPC 
MPCconfig = Control.MPC.setup(dt, timeHorizon, t_sim, z0);

% Allocate space to save off the trajectory
u_store = zeros(1, length(t_sim)); 
z_store = zeros(4, length(t_sim)); 

% Loop simulation
curZ = z0; 
for ix = 1:length(t_sim)
    % Get the current time
    curTime = t_sim(ix); 

    % Grab current section of desired trajectory
    % For a constant setpoint, just repeat desired 
    zDesiredTraj = repmat(zDesired, 1, MPCconfig.N_horizon); 
    uDesiredTraj = repmat(uDesired, 1, MPCconfig.N_horizon); 

    % Solve optimal control
    Q = diag([10, 1, .5,.5]); 
    R = 1; 
    [curU, zstar, ustar, MPCconfig] = Control.MPC.run(Q, R, curZ, zDesiredTraj, uDesiredTraj, MPCconfig); 

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
end

% Plot the trajectories
Ballbot.plotTrajectories(t_sim, z_store', u_store); 

%% Animate the solution
q_store = [z_store(1,:); z_store(3,:)];
Ballbot.animate(t_sim, q_store', 'TestMPC.mp4');




