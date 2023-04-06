function [u, zstar, ustar, MPCconfig, MPCfailed] = run(Q, R, curZ, zDesired, uDesired, MPCconfig, pObs, rObs)
% Grab constraints from file
[qLim, qdotLim, uLim] = Ballbot.defineConstraints(Control.OptimizationType.MPC);
zLim = interleave2(qLim, qdotLim, 'row');

% Solve optimal control
% H = Control.MPC.H_func(Q, R, MPCconfig.timeHorizon, zDesired, uDesired); 
% c = Control.MPC.c_func(Q, R, MPCconfig.timeHorizon, zDesired, uDesired); 
% Aeq = Control.MPC.Aeq_func(MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, MPCconfig.timeHorizon); 
% beq = Control.MPC.beq_func(MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, curZ, MPCconfig.timeHorizon); 
% A = Control.MPC.A_func(zLim, uLim, MPCconfig.zTrajExpected, pObs, rObs);
% b = Control.MPC.b_func(zLim, uLim, MPCconfig.zTrajExpected, pObs, rObs); 
[H, c, Aeq, beq, A, b] = Control.MPC.getQP_funcs( ...
    Q, R, MPCconfig.timeHorizon, curZ, zDesired, uDesired, ...
    MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, zLim, uLim,...
    pObs, rObs...
    );
options = optimoptions("quadprog","Display","none", 'ConstraintTolerance',1e-8);
[xtildestar, fval, exitFlag, output] = quadprog(H, c, A, b, Aeq, beq, [],[],[],options); 
if exitFlag < 0
    xtildestar = zeros(Control.MPC.get_N_decisionVars(),1);
    disp(['MPC Failed. Quadprog exit flag ',num2str(exitFlag)]);
end
MPCfailed = exitFlag < 0; 

zstar = Control.MPC.extract_zN_from_DV(xtildestar); 
ustar = Control.MPC.extract_uN_from_DV(xtildestar); 

% Return the first control output
u = ustar(1); 

% Update the expected trajectories
MPCconfig.zTrajExpected = zstar;
MPCconfig.uTrajExpected = ustar';

end

