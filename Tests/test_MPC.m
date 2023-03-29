clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 0.0;

% Define desired states
qdes = [desX/rk;0];
qdot_des = [0; 0];

% Define initial conditions
q0 = [-pi;-pi/16];
qdot0 = [0;0];
z0 = interleave2(q0, qdot0, 'row'); 

% Define points to linearize about -> later this will be a trajectory
qstar = qdes;
qdotstar = qdot_des; 
ustar = 0;
zstar = interleave2(qstar, qdotstar, 'row'); 

%% Run MPC around nominal state
% Configure controller
dt = 0.05; % Real time sample rate
timeHorizon = 2.5; 

% Configure simulation
T_simulation = 4; % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Convert desired points to trajectories
zstarTraj = repmat(zstar, 1, N_sim);
ustarTraj = repmat(ustar, 1, N_sim);

% Configure MPC 
MPCconfig = Control.MPC.setup(dt, timeHorizon, ...
    zstarTraj, ustarTraj, t_sim);

% Allocate space to save off the trajectory
u_store = zeros(1, length(t_sim)); 
z_store = zeros(4, length(t_sim)); 

% Loop simulation
for ix = 1:length(t_sim)
    % Get the current time
    curTime = t_sim(ix); 

    % Calculate deviation from nominal states
    ztilde = z0 - zstar;

    % Solve optimal control
    Q = diag([10, 1, .5,.5])*eye(4); 
    R = 1; 
    u = Control.MPC.run(Q, R, ztilde, curTime, MPCconfig); 

    % Run simulation with said control 
    forceFunc = @(t,z) u; 
    q0 = z0([1,3],:);
    qdot0 = z0([2,4],:);
    [t,q, qdot, z, ~] = Ballbot.runSimulation(q0, qdot0, forceFunc, curTime + [0,dt], false);

    % Update initial conditions
    z0 = z(end,:)';

    % Store the trajectories
    u_store(ix) = u; 
    z_store(:,ix) = z(1,:)'; 
end

% Plot the trajectories
Ballbot.plotTrajectories(t_sim, z_store', u_store); 

%% Animate the solution
q_store = [z_store(1,:); z_store(3,:)];
Ballbot.animate(t_sim, q_store', 'TestMPC.mp4');




