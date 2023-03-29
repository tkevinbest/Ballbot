function [u, ztildestar, utildestar] = run(Q, R, ztilde0, curTime, MPCconfig)
% Grab the linearized dynamics for the current horizon
horizonStartIndex = round(curTime/MPCconfig.dt_NominalTrajectory)+1; 
horizonEndIndex = round(horizonStartIndex + MPCconfig.N_horizon)-1; 
A_dyn = MPCconfig.A_dyn(:,:,horizonStartIndex:horizonEndIndex);
B_dyn = MPCconfig.B_dyn(:,:,horizonStartIndex:horizonEndIndex);

% Solve optimal control
H = Control.MPC.H_func(Q, R, MPCconfig.timeHorizon); 
c = Control.MPC.c_func(Q, R, MPCconfig.timeHorizon); 
Aeq = Control.MPC.Aeq_func(A_dyn, B_dyn, MPCconfig.timeHorizon); 
beq = Control.MPC.beq_func(A_dyn, B_dyn, ztilde0, MPCconfig.timeHorizon); 
options = optimoptions("quadprog","Display","none");
[xtildestar, fval] = quadprog(H, c, [],[], Aeq, beq, [],[],[],options); 
ztildestar = Control.MPC.extract_zN_from_DV(xtildestar); 
utildestar = Control.MPC.extract_uN_from_DV(xtildestar); 

% Return the first control output
u = utildestar(1); 
end

