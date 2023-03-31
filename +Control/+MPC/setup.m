function MPCconfig = setup(dt, timeHorizon, t_trajectory, z0)
MPCconfig.timeHorizon = timeHorizon;
t_horizon = 0:dt:timeHorizon; 
MPCconfig.N_horizon = length(t_horizon); 
MPCconfig.dt_NominalTrajectory = mode(diff(t_trajectory)); 

% Configure MPC functions
if MPCconfig.N_horizon ~= Control.MPC.getNumPtsForGeneratedCode()
    disp('Horizon changed -> Generating MPC Functions')
    Control.MPC.genFunctions(MPCconfig.N_horizon); 
    rehash % Makes sure new functions are loaded before use
    disp('Done!')
end

% Estimate the expected state and control trajectories for the upcoming
% horizon
MPCconfig.zTrajExpected = repmat(z0, 1, MPCconfig.N_horizon);
MPCconfig.uTrajExpected = repmat(0, 1, MPCconfig.N_horizon);
end