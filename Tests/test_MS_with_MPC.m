clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.0;

% Number of nodes
N = 26; 

% Define obstacle
pObs = [.0;.58];
rObs = .1;

%% Run Multiple Shooting Trajectory Optimization
% This calculates the nominal optimal trajectory
q0 = [-1/rk;0];
qdot0 = [0;0];
z0 = interleave2(q0, qdot0, 'row');
qdes = [desX/rk;0];
qdot_des = [0; 0];

tic
[uPlan, qstar, qdotstar, tstar, Tfstar, finalCost, fminconout] = ...
        Control.optimize_trajectory_MS_mex(q0, qdot0, qdes, qdot_des, N);
solveTime = toc;
disp(['MS Optimization with ', num2str(N),' nodes completed in ',num2str(solveTime,'%.2f'),' seconds.'])
zPlan = interleave2(qstar, qdotstar, 'row'); 

%% Run MPC around nominal trajectory
% Configure controller
dt = 0.01; % Real time sample rate
timeHorizon = 2.5; 
N_horizon = 51; % Nodes
t_horizon = 0:dt:timeHorizon; 

% Configure simulation
T_simulation = max(tstar)+.1; % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Convert desired points to trajectories
zstarTraj = interp1(tstar', zPlan', t_sim')'; 
ustarTraj = interp1(tstar', uPlan', t_sim')'; 
indicesBeyondPlan = any(isnan(zstarTraj)); 
zstarTraj(:,indicesBeyondPlan) = repmat(zPlan(:,end),1,sum(indicesBeyondPlan));
ustarTraj(:,indicesBeyondPlan) = repmat(uPlan(end),1,sum(indicesBeyondPlan));

% Configure MPC 
MPCconfig = Control.MPC.setup(N_horizon, timeHorizon, t_sim, z0);

% Add horizon's length buffer to the end of the trajectory
zstarTraj = [zstarTraj, repmat(zstarTraj(:,end),1,MPCconfig.N_horizon)];
ustarTraj = [ustarTraj, repmat(zeros(size(ustarTraj(:,end))),1,MPCconfig.N_horizon)];

% Allocate space to save off the trajectory
u_store = zeros(1, length(t_sim)); 
z_store = zeros(4, length(t_sim)); 
mpcTime = zeros(length(t_sim),1); 

% Loop simulation
curZ = z0; 
zstarHist = [];
ustarHist = [];
for ix = 1:length(t_sim)
    % Get the current time
    curTime = t_sim(ix); 

    % Grab current section of desired trajectory
    % For a constant setpoint, just repeat desired 

    zDesiredTraj_thisHorizon = zstarTraj(:,ix:ix+MPCconfig.N_horizon-1); 
    uDesiredTraj_thisHorizon = ustarTraj(:,ix:ix+MPCconfig.N_horizon-1); 

    % Solve optimal control
    Q = diag([10, 0, 1,0]); 
    R = .02; 
    tic
    [curU, zstar, ustar, MPCconfig, MPCfailed] = Control.MPC.run(Q, R, curZ, ...
                                                        zDesiredTraj_thisHorizon, ...
                                                        uDesiredTraj_thisHorizon, MPCconfig,...
                                                         pObs, rObs); 
    zstarHist(:,:,ix) = zstar; 
    ustarHist(:,ix) = ustar'; 
    tHist(:,ix) = curTime + t_horizon;
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

    % Plot progress
    % Control.MPC.plotTrajectoriesAndPrediction(t_sim(1:ix), z_store(:,1:ix)', u_store(1:ix), zstarHist, ustarHist, tHist); 
    % drawnow; 

    if MPCfailed
        t_sim = t_sim(1:ix-1); 
        z_store = z_store(:,1:ix-1); 
        u_store = u_store(1:ix-1); 
        break;
    end
end

disp(['Avg. MPC Compute Time: ', num2str(mean(mpcTime)), ' s'   ])

% Plot the trajectories
Ballbot.plotTrajectories(t_sim, z_store', u_store); 

%% Animate the solution
q_store = [z_store(1,:); z_store(3,:)];
Ballbot.animate(t_sim, q_store', 'TestMS_MPC.mp4', pObs, rObs);




