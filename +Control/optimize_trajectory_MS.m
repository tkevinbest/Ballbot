function [ustar, qstar, qdotstar, tstar, Tfstar, finalCost, fminconout] ...
            = optimize_trajectory_MS(q0, qdot0, qdes, qdot_des, N)
consts.q0 = q0;
consts.qdot0 = qdot0;
consts.qf = qdes;
consts.qdotf = qdot_des; 

% Grab constraints from file
[qlim, qdotlim, ulim] = Ballbot.defineConstraints(Control.OptimizationType.MS);

% Make constraints more conservative for initial TO
qlim = .9*qlim;
qdotlim = .9*qdotlim;
ulim = .9*ulim; 

% Define initial guess for decision vector
% Handle nan cases for unspecified final conditions
u0 = [ulim(2); zeros(N-2, 1); ulim(1)];
qdes(isnan(qdes)) = 0;
qdot_des(isnan(qdot_des)) = 0; 
q_guess = [repmat(q0,1,ceil(N/2)),repmat(qdes,1,floor(N/2))];
qdot_guess = [repmat(qdot0,1,ceil(N/2)),repmat(qdot_des,1,floor(N/2))];
Tf0 = 1.5;
x0 = packDecisionVector(u0, q_guess, qdot_guess, Tf0);

Aeq = [];
beq = [];

% Inequality constraints
A = [];
b = [];

% Upper and lower bounds
uLimitsPos = ones(N,1)*ulim(2);
uLimitsNeg = ones(N,1)*ulim(1);

qLimitsPos = repmat(qlim(:,2),1,N);
qLimitsNeg = repmat(qlim(:,1),1,N);
qdotLimitsNeg = repmat(qdotlim(:,1),1,N); 
qdotLimitsPos = repmat(qdotlim(:,2),1,N); 
lb = packDecisionVector(uLimitsNeg, qLimitsNeg, qdotLimitsNeg, 0.01);
ub = packDecisionVector(uLimitsPos, qLimitsPos, qdotLimitsPos, inf);

options = optimoptions("fmincon",...
    "algorithm","sqp",...
    "SpecifyObjectiveGradient",true,...
    "MaxFunctionEvaluations",3e5, ...
    "MaxIterations",5e4,...
    "Display","iter");

[xstar, finalCost, exitflag, fminconout] = fmincon(@costFunc, x0, A, b, Aeq, beq, lb, ub, @(x)nonlcon(x,consts), options);
[ustar, qstar, qdotstar, Tfstar, N] = unpackDecisionVector(xstar);

% Calculate optimal time vector
tstar = linspace(0, Tfstar, N);

end


function [cost, gradF] = costFunc(x)
[u, q, qdot, Tf, N] = unpackDecisionVector(x);

cost = Tf;
gradF = [zeros(length(x)-1,1);1];

end

function [c, ceq] = nonlcon(x, consts)
% Calculates the nonlinear defect constraints and boundary constraints for
% the problem. 

[u, q, qdot, Tf, N] = unpackDecisionVector(x);

% Create time vector and number of shots
tVec = linspace(0, Tf, N); 
numShots = N-1;
ceq_defect_mat = zeros(numShots,4);

% Parse out the end of shot states
q_next = q(:,2:end);
qdot_next = qdot(:,2:end); 

% Grab times for start and end of shots
startTimes = tVec(1:end-1);
endTimes = tVec(2:end); 

parfor ix = 1:numShots
    % Simulate the shot
    [~, qsim, qdotsim] = Ballbot.runSimulation(q(:, ix), qdot(:,ix), @(t,q)controllerFunc(t,u,N, Tf), ...
        [startTimes(ix), endTimes(ix)], false);

    % Create a row of the defect constraint across all states and state
    % derivatives
    ceq_defect_mat(ix,:) = [(qsim(end,:) - q_next(:,ix)'), (qdotsim(end,:) - qdot_next(:,ix)')];
end
% Flatten these defect constraints into a vector
ceq_defect = reshape(ceq_defect_mat,[],1);

% Assemble defect equality constraints and obstacle inequality constraint
c = [];

% Boundary constraints


ceq_boundary_ic = [q(:,1) - consts.q0; ...
                qdot(:,1)-consts.qdot0];

% Check to see which terminal constraints we have
ceq_boundary_fc = [q(:,end) - consts.qf;...
                qdot(:,end) - consts.qdotf];
ceq_boundary_fc(isnan(ceq_boundary_fc)) = []; 

% Concatenate our final equality constraint vector. 
ceq = [ceq_defect; ceq_boundary_ic; ceq_boundary_fc];
end

function F = controllerFunc(t, uVec, N, Tf)
tVec = linspace(0,Tf,N);
F = interp1(tVec, uVec, t); 
end


function x = packDecisionVector(u, q, qdot, Tf)
x = [u;q(1,:)'; q(2,:)';qdot(1,:)';qdot(2,:)';Tf];
end


function [u, q, qdot, Tf, N] = unpackDecisionVector(x)
% Decision vector: [u1 .... uN, 
%                   q11 ... q1N, q21 ... 12N ... q2N,
%                   qdot11 ... qdot2N,
%                   Tf]';
% Length of decision vector is N + N*numStates + N*numStates + 1 = ...
% 1+N*(2*numStates+1)

numStates = 2;
N = (length(x)-1)/(2*numStates+1);
u = x(1:N);
q = [x(N+1:2*N)'; x(2*N+1:3*N)'];
qdot = [x(3*N+1:4*N)'; x(4*N+1:5*N)'];
Tf = x(5*N+1);
end