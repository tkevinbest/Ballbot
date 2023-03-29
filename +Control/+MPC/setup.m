function MPCconfig = setup(dt, timeHorizon, zstar_traj, ustar_traj, t_trajectory)
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

% Get the linearized dynamics about desired trajectory
for ix = 1:length(t_trajectory)
    MPCconfig.A_dyn(:,:,ix) = Ballbot.A_lin_symb(zstar_traj(:,ix), ustar_traj(:,ix));
    MPCconfig.B_dyn(:,:,ix) = Ballbot.B_lin_symb(zstar_traj(:,ix), ustar_traj(:,ix));
end

% Add an additional N_horizon on the end of the trajectory
for ix = length(t_trajectory)+1:length(t_trajectory)+MPCconfig.N_horizon
    MPCconfig.A_dyn(:,:,ix) = Ballbot.A_lin_symb(zstar_traj(:,end), ustar_traj(:,end));
    MPCconfig.B_dyn(:,:,ix) = Ballbot.B_lin_symb(zstar_traj(:,end), ustar_traj(:,end));
end
end