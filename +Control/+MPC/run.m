function [u, zstar, ustar, MPCconfig] = run(Q, R, curZ, zDesired, uDesired, MPCconfig)

% Solve optimal control
H = Control.MPC.H_func(Q, R, MPCconfig.timeHorizon, zDesired, uDesired); 
c = Control.MPC.c_func(Q, R, MPCconfig.timeHorizon, zDesired, uDesired); 
Aeq = Control.MPC.Aeq_func(MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, MPCconfig.timeHorizon); 
beq = Control.MPC.beq_func(MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, curZ, MPCconfig.timeHorizon); 
options = optimoptions("quadprog","Display","none");
[xtildestar, fval] = quadprog(H, c, [],[], Aeq, beq, [],[],[],options); 
zstar = Control.MPC.extract_zN_from_DV(xtildestar); 
ustar = Control.MPC.extract_uN_from_DV(xtildestar); 

% Return the first control output
u = ustar(1); 

% Update the expected trajectories
MPCconfig.zTrajExpected = zstar;
MPCconfig.uTrajExpected = ustar';
end

