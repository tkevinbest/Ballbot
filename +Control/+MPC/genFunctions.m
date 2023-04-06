function genFunctions(N_horizon_pts)
num_states = 4;
num_inputs = 1; 

syms T_horizon real
dt = T_horizon/(N_horizon_pts-1); 
syms N_horizon_pts_out real

% Decision variables
zN = sym('zN',[num_states, N_horizon_pts], 'real');
uN = sym('uN', [num_inputs, N_horizon_pts], 'real');

% Define the decision vector 
x = [uN(:); zN(:)];
syms N_decision_vars;

% Define how to extract from the decision vector
uN_extracted = x(1:N_horizon_pts); 
zN_extracted = reshape(x(N_horizon_pts + 1:end), num_states, []); 

% Desired and expected trajectories
zN_des = sym('zN_des',[num_states, N_horizon_pts], 'real');
zN_exp = sym('zN_exp',[num_states, N_horizon_pts], 'real');
uN_des = sym('uN_des', [num_inputs, N_horizon_pts], 'real');
uN_exp = sym('uN_exp', [num_inputs, N_horizon_pts], 'real');

zN_tilde = zN - zN_exp; 
uN_tilde = uN - uN_exp; 

% Calculate dynamics
for ix = 1:N_horizon_pts
    A(:,:,ix) = Ballbot.A_lin_symb(zN_exp(:,ix), uN_exp(:,ix)); 
    B(:,:,ix) = Ballbot.B_lin_symb(zN_exp(:,ix), uN_exp(:,ix)); 
    F(:,ix) = Ballbot.F_lin_symb(zN_exp(:,ix), uN_exp(:,ix)); 
end

for ix = 1:size(A,3)
    dz(:,ix) = A(:,:,ix)*zN_tilde(:,ix) + B(:,:,ix)*uN_tilde(:,ix) + F(:,ix); 
end

% Define defect constraints through trapezoidal integration
dz_prev = dz(:,1:end-1);
dz_next = dz(:,2:end); 
zNext = zN(:, 2:end);
zPrev = zN(:, 1:end-1); 
defect_constraints = reshape(zNext - zPrev - (dz_prev + dz_next) * dt/2.0,[],1); 

% Define initial condition constraint
z0 = sym('z0',[num_states,1], 'real');
ic_constraint = zN(:,1) - z0; 

% Group all equality constraints
equality_constraints = [defect_constraints; ic_constraint]; 

% Obstacle Constraints
pObs = sym('pObs',[2,1], 'real');
syms rObs real;

% Linearize top of pendulum location about expected trajectory
% zN_ExcursionFromExpected = zN - zN_exp; 
% for ix = 1:N_horizon_pts
%     expectedDistToCoM = Ballbot.calcDistFrom_CoM(zN_exp(:,ix), pObs); 
%     linearizedDistToCoM(ix,1) = expectedDistToCoM + Ballbot.calcDistFrom_CoM_jacobian(zN_exp(:,ix), pObs) * zN_ExcursionFromExpected(:,ix);
% end
q = sym('q',[2,1]);
BallbotCoMSymb = Ballbot.calcCOM_location(q); 
BallbotCoMJac = jacobian(BallbotCoMSymb, q); 
for ix = 1:N_horizon_pts
    curqNominal = [zN_exp(1,ix); zN_exp(3,ix)];
    curqActual = [zN(1,ix); zN(3,ix)];
    approximateCoMLocation = Ballbot.calcCOM_location(curqNominal) + ...
                             subs(BallbotCoMJac, q, curqNominal) * (curqActual - curqNominal);
    trueCoMLocation = Ballbot.calcCOM_location(curqActual);
    heightLimit = pObs(2)-rObs + 2*(trueCoMLocation(1)-pObs(1))^2;
    heightLimit_approx = subs(heightLimit, curqActual, curqNominal) + subs(jacobian(heightLimit,curqActual), curqActual, curqNominal) * (curqActual - curqNominal); 
    obstacleConstraint(ix) = approximateCoMLocation(2) - heightLimit_approx; 
end

% Remove first obstacle constraint because we can't really control the IC
obstacleConstraint(1) = [];

% Inequality constraints
zLim = sym('zLim', [num_states, 2], 'real');
uLim = sym('uLim', [num_inputs, 2], 'real'); 

stateLowerLimits = reshape(zLim(:,1) - zN(:,2:end),[],1);
stateUpperLimits = reshape(zN(:,2:end) - zLim(:,2), [],1); 
torqueLowerLimits = reshape(uLim(:,1) - uN, [],1); 
torqueUpperLimits = reshape(uN - uLim(:,2), [],1); 
inequality_constraints = [stateLowerLimits; stateUpperLimits; torqueLowerLimits; torqueUpperLimits; obstacleConstraint(:)]; 


% Define cost function - just squared error for now
Q = sym('Q',[num_states, num_states], 'real');
R = sym('R',[num_inputs, num_inputs], 'real');
zError = zN - zN_des; 
uError = uN - uN_des; 
zN_error_flat = reshape(zError,[],1); 
uN_error_flat = reshape(uError,[],1);
J = zN_error_flat.'* kron(eye(N_horizon_pts), Q) * zN_error_flat + ...
    uN_error_flat.'* kron(eye(N_horizon_pts), R) * uN_error_flat;

% Calculate QP Matrices
H = hessian(J, x); 
c = subs(jacobian(J,x),x, zeros(size(x))).'; 
Aeq = jacobian(equality_constraints, x); 
beq = -subs(equality_constraints, x, zeros(size(x))); 
A = jacobian(inequality_constraints, x); 
b = -subs(inequality_constraints, x, zeros(size(x))); 

% Export matlab functions
simplifyAndWriteMatlabFunction( subs(N_horizon_pts_out, N_horizon_pts_out, N_horizon_pts),'File', './+Control/+MPC/getNumPtsForGeneratedCode');
matlabFunction(H, c, Aeq, beq, A, b, 'File', './+Control/+MPC/getQP_funcs', 'Vars',{Q, R, T_horizon, z0, zN_des, uN_des,zN_exp, uN_exp, zLim, uLim, pObs, rObs},...
    "Outputs",{'H','c','Aeq','beq','A','b'},'Sparse',true, 'Optimize',false);
simplifyAndWriteMatlabFunction(uN_extracted,'File', './+Control/+MPC/extract_uN_from_DV', 'Vars', {x});
simplifyAndWriteMatlabFunction(zN_extracted,'File', './+Control/+MPC/extract_zN_from_DV', 'Vars', {x});
matlabFunction(subs(N_decision_vars, N_decision_vars, length(x)),'File', './+Control/+MPC/get_N_decisionVars');
end

function outVal = simplifyAndWriteMatlabFunction(varargin)
outVal = matlabFunction(simplify(varargin{1}), varargin{2:end});
end