function [u, zstar, ustar, MPCconfig, MPCfailed] = run(Q, R, curZ, zDesired, uDesired, MPCconfig, pObs, rObs)
% Grab constraints from file
[qLim, qdotLim, uLim] = Ballbot.defineConstraints(Control.OptimizationType.MPC);
zLim = interleave2(qLim, qdotLim, 'row');

% Solve optimal control
[H, c, H_PCC, c_PCC, Aeq, beq, A, b] = Control.MPC.getQP_funcs( ...
    Q, R, MPCconfig.timeHorizon, curZ, zDesired, uDesired, ...
    MPCconfig.zTrajExpected, MPCconfig.uTrajExpected, zLim, uLim,...
    pObs, rObs,...
    MPCconfig.previousDV, 1 ...
    );

if MPCconfig.planMade
    H_used = H_PCC;
    c_used = c_PCC;
else
    H_used = H;
    c_used = c;
end

options = optimoptions("quadprog","Display",'off');
[xtildestar, fval, exitFlag, output] = quadprog(H_used, c_used, A, b, Aeq, beq, [],[],[],options); 
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
MPCconfig.previousDV = xtildestar; 
MPCconfig.planMade = true; 

end

