clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.0;

% Number of nodes
N = 26; 

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
        Control.optimize_trajectory_MS_mex(q0, qdot0, qdes, qdot_des, N);
solveTime = toc;
disp(['MS Optimization with ', num2str(N),' nodes completed in ',num2str(solveTime,'%.2f'),' seconds.'])
zstar = interleave2(qstar, qdotstar, 'row'); 

%% Run MPC around nominal trajectory
% Configure controller
dt = 0.05; % Real time sample rate
timeHorizon = 3; 

% Configure simulation
T_simulation = max(tstar); % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Convert desired points to trajectories
zstarTraj = interp1(tstar', zstar', t_sim')'; 
ustarTraj = interp1(tstar', ustar', t_sim')'; 

% Configure MPC 
MPCconfig = Control.MPC.setup(dt, timeHorizon, t_sim, z0);

% Add horizon's length buffer to the end of the trajectory
zstarTraj = [zstarTraj, repmat(zstarTraj(:,end),1,MPCconfig.N_horizon)];
ustarTraj = [ustarTraj, repmat(ustarTraj(:,end),1,MPCconfig.N_horizon)];

% Allocate space to save off the trajectory
u_store = zeros(1, length(t_sim)); 
z_store = zeros(4, length(t_sim)); 
mpcTime = zeros(length(t_sim),1); 

% Loop simulation
curZ = z0; 
for ix = 1:length(t_sim)
    % Get the current time
    curTime = t_sim(ix); 

    % Grab current section of desired trajectory
    % For a constant setpoint, just repeat desired 

    zDesiredTraj_thisHorizon = zstarTraj(:,ix:ix+MPCconfig.N_horizon-1); 
    uDesiredTraj_thisHorizon = ustarTraj(:,ix:ix+MPCconfig.N_horizon-1); 

    % Solve optimal control
    Q = diag([1000, 1, 100,.5]); 
    R = 1; 
    tic
    [curU, zstar, ustar, MPCconfig] = Control.MPC.run(Q, R, curZ, ...
                                                        zDesiredTraj_thisHorizon, ...
                                                        uDesiredTraj_thisHorizon, MPCconfig); 
    mpcTime(ix) = toc;

    % Run simulation with said control 
    forceFunc = @(t,z) curU; %*(rand()*.2 + .9); 
    q0 = curZ([1,3],:);
    qdot0 = curZ([2,4],:);
    [t,q, qdot, z, ~] = Ballbot.runSimulation(q0, qdot0, forceFunc, curTime + [0,dt], false);

    % Update initial conditions
    curZ = z(end,:)';

    % Store the trajectories
    u_store(ix) = curU; 
    z_store(:,ix) = z(1,:)'; 
end

disp(['Avg. MPC Compute Time: ', num2str(mean(mpcTime)), ' s'   ])

% Plot the trajectories
Ballbot.plotTrajectories(t_sim, z_store', u_store); 

%% Animate the solution
q_store = [z_store(1,:); z_store(3,:)];
Ballbot.animate(t_sim, q_store', 'TestMS_MPC.mp4');




