clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.25;

% Number of nodes
N = 26; 

% Constraints on states
qLim = [
        -5/rk, 1.5/rk;
        -deg2rad(30), deg2rad(30)
    ];
qdotLim = 5*[
        -15, 15;
        -2, 2
    ];
uLim = [-10, 10]; 

% Obstacles

%% Run Multiple Shooting Trajectory Optimization
% This calculates the nominal optimal trajectory
q0 = [-pi;0];
qdot0 = [0;0];
z0 = interleave2(q0, qdot0, 'row');
qdes = [desX/rk;0];
qdot_des = [0; 0];

tic
[ustar, qstar, qdotstar, tstar, Tfstar, finalCost, fminconout] = ...
        Control.optimize_trajectory_MS_mex(q0, qdot0, qdes, qdot_des, N, qLim, qdotLim, uLim);
solveTime = toc;
disp(['MS Optimization with ', num2str(N),' nodes completed in ',num2str(solveTime,'%.2f'),' seconds.'])
zstar = interleave2(qstar, qdotstar, 'row'); 

%% Run MPC around nominal trajectory
% Configure controller
dt = 0.025; % Real time sample rate
timeHorizon = 2; 

% Configure simulation
T_simulation = max(tstar); % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Convert desired points to trajectories
zstarTraj = interp1(tstar', zstar', t_sim')'; 
ustarTraj = interp1(tstar', ustar', t_sim')'; 

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
    ztilde = z0 - zstarTraj(:,ix);

    % Solve optimal control
    Q = diag([10, 1, .5,.5])*eye(4); 
    R = 1; 
    tic;
    u = Control.MPC.run(Q, R, ztilde, curTime, MPCconfig); 
    mpcTime(ix) = toc;

    % Run simulation with said control 
    forceFunc = @(t,z) u + ustarTraj(ix); 
    q0 = z0([1,3],:);
    qdot0 = z0([2,4],:);
    [t,q, qdot, z, ~] = Ballbot.runSimulation(q0, qdot0, forceFunc, curTime + [0,dt], false);

    % Update initial conditions
    z0 = z(end,:)';

    % Store the trajectories
    u_store(ix) = u + ustarTraj(ix); 
    z_store(:,ix) = z(1,:)'; 
end

disp(['Avg. MPC Compute Time: ', num2str(mean(mpcTime)), ' s'   ])

% Plot the trajectories
Ballbot.plotTrajectories(t_sim, z_store', u_store); 

%% Animate the solution
q_store = [z_store(1,:); z_store(3,:)];
Ballbot.animate(t_sim, q_store', 'TestMS_MPC.mp4');




