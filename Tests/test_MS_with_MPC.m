clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.0;

% Number of nodes
N = 26; 

% Define obstacle
pObs = [.25;.5];
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

% Call plot
z = interleave2(qstar, qdotstar, 'row'); 
Ballbot.plotTrajectories(tstar, zPlan', uPlan); 

% Animate ballbot
Ballbot.animate(tstar, qstar'); 
figure()

%% Run MPC around nominal trajectory
% Configure controller
dt = 0.01; % Real time sample rate
timeHorizon = 2.25; 
N_horizon = 26; % Nodes
t_horizon = linspace(0, timeHorizon, N_horizon); 

% Configure simulation
T_simulation = max(tstar)+1; % Length of the simulation
t_sim = 0:dt:T_simulation; 
N_sim = length(t_sim); 

% Configure MPC 
% Calculate expected trajectory for the first iteration
zDesiredTraj_thisHorizon = interp1(tstar', zPlan', t_horizon)';
uDesiredTraj_thisHorizon = interp1(tstar', uPlan', t_horizon);
indicesBeyondPlan = any(isnan(zDesiredTraj_thisHorizon)); 
zDesiredTraj_thisHorizon(:,indicesBeyondPlan) = repmat(zPlan(:,end),1,sum(indicesBeyondPlan));
uDesiredTraj_thisHorizon(indicesBeyondPlan) = repmat(uPlan(end),1,sum(indicesBeyondPlan));
MPCconfig = Control.MPC.setup(N_horizon, timeHorizon, t_sim, zDesiredTraj_thisHorizon, uDesiredTraj_thisHorizon);

% Allocate space to save off the trajectory
u_store = zeros(1, length(t_sim)); 
z_store = zeros(4, length(t_sim)); 
mpcTime = zeros(length(t_sim),1); 

% Loop simulation
curZ = z0; 
zstarHist = [];
ustarHist = [];
tHist = [];
for ix = 1:length(t_sim)
    % Get the current time
    curTime = t_sim(ix); 

    % Calculate nominal trajectories at each horizon point
    zDesiredTraj_thisHorizon = interp1(tstar', zPlan', curTime + t_horizon)';
    uDesiredTraj_thisHorizon = interp1(tstar', uPlan', curTime + t_horizon);
    indicesBeyondPlan = any(isnan(zDesiredTraj_thisHorizon)); 
    zDesiredTraj_thisHorizon(:,indicesBeyondPlan) = repmat(zPlan(:,end),1,sum(indicesBeyondPlan));
    uDesiredTraj_thisHorizon(indicesBeyondPlan) = repmat(uPlan(end),1,sum(indicesBeyondPlan));


    % Solve optimal control
    Q = diag([200, 0, 1,0]); 
    R = .2; 
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
    forceFunc = @(t,z) curU ;
    q0 = curZ([1,3],:);
    qdot0 = curZ([2,4],:);
    [t,q, qdot, z, ~] = Ballbot.runSimulation(q0, qdot0, forceFunc, curTime + [0,dt], false);
    
    % Update initial conditions
    curZ = z(end,:)';

    % Store the trajectories
    u_store(ix) = curU; 
    z_store(:,ix) = z(1,:)'; 

    % Plot progress
%     Control.MPC.plotTrajectoriesAndPrediction(t_sim(1:ix), z_store(:,1:ix)', u_store(1:ix), zstarHist, ustarHist, tHist); 
%     drawnow; 

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




