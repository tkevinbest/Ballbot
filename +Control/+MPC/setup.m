function MPCconfig = setup(N_horizon, timeHorizon, t_trajectory, z0)
MPCconfig.timeHorizon = timeHorizon;
MPCconfig.N_horizon = N_horizon; 
MPCconfig.dt_NominalTrajectory = mode(diff(t_trajectory)); 

% Configure MPC functions
if MPCconfig.N_horizon ~= Control.MPC.getNumPtsForGeneratedCode()
    disp('Horizon nodes changed -> Generating MPC Functions')
    Control.MPC.genFunctions(MPCconfig.N_horizon); 
    pause(5); % Pause to make sure all functions are done writing
    rehash % Makes sure new functions are loaded before use
    disp('Done!')
end

% Estimate the expected state and control trajectories for the upcoming
% horizon
MPCconfig.zTrajExpected = repmat(z0, 1, MPCconfig.N_horizon);
MPCconfig.uTrajExpected = repmat(0, 1, MPCconfig.N_horizon);
end