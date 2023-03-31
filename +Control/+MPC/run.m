function [u, zstar, ustar, MPCconfig] = run(Q, R, curZ, zDesired, uDesired, MPCconfig)
% Grab constraints from file
[qLim, qdotLim, uLim] = Ballbot.defineConstraints(Control.OptimizationType.MPC);
zLim = interleave2(qLim, qdotLim, 'row');

% Solve optimal control
H = Control.MPC.H_func(Q, R, MPCconfig.timeHorizon, zDesired, uDesired); 
c = Control.MPC.c_func(Q, R, MPCconfig.timeHorizon, zDesired, uDesired); 
Aeq = Control.MPC.Aeq_func(MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, MPCconfig.timeHorizon); 
beq = Control.MPC.beq_func(MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, curZ, MPCconfig.timeHorizon); 
A = Control.MPC.A_func(zLim, uLim);
b = Control.MPC.b_func(zLim, uLim); 
options = optimoptions("quadprog","Display","none");
[xtildestar, fval, exitFlag, output] = quadprog(H, c, A, b, Aeq, beq, [],[],[],options); 
if exitFlag < 0
    error(['MPC Failed. Quadprog exit flag ',num2str(exitFlag)]);
end
zstar = Control.MPC.extract_zN_from_DV(xtildestar); 
ustar = Control.MPC.extract_uN_from_DV(xtildestar); 

% Return the first control output
u = ustar(1); 

% Update the expected trajectories
MPCconfig.zTrajExpected = zstar;
MPCconfig.uTrajExpected = ustar';
end

