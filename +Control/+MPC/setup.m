function MPCconfig = setup(N_horizon, timeHorizon, t_trajectory, zExpected, uExpected)
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
MPCconfig.zTrajExpected = zExpected;
MPCconfig.uTrajExpected = uExpected;
MPCconfig.previousDV = zeros(Control.MPC.get_N_decisionVars,1);
MPCconfig.planMade = false; 
end