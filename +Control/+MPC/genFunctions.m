function genFunctions(N_horizon_pts)
num_states = 4;
num_inputs = 1; 

syms T_horizon
dt = T_horizon/(N_horizon_pts-1); 
syms N_horizon_pts_out

% Decision variables
zN = sym('zN',[num_states, N_horizon_pts]);
uN = sym('uN', [num_inputs, N_horizon_pts]);

% Desired and expected trajectories
zN_des = sym('zN_des',[num_states, N_horizon_pts]);
zN_exp = sym('zN_exp',[num_states, N_horizon_pts]);
uN_des = sym('uN_des', [num_inputs, N_horizon_pts]);
uN_exp = sym('uN_exp', [num_inputs, N_horizon_pts]);

zN_tilde = zN - zN_exp; 
uN_tilde = uN - uN_exp; 

% Calculate dynamics
for ix = 1:N_horizon_pts
    A(:,:,ix) = Ballbot.A_lin_symb(zN_exp(:,ix), uN_exp(:,ix)); 
    B(:,:,ix) = Ballbot.B_lin_symb(zN_exp(:,ix), uN_exp(:,ix)); 
    F(:,ix) = Ballbot.F_lin_symb(zN_exp(:,ix), uN_exp(:,ix)); 
end

for ix = 1:size(A,3)
    dZ(:,ix) = A(:,:,ix)*zN_tilde(:,ix) + B(:,:,ix)*uN_tilde(:,ix) + F(:,ix); 
end

% Define defect constraints through trapezoidal integration
zNext = zN(:, 2:end);
zPrev = zN(:, 1:end-1); 
dz_prev = dZ(:,1:end-1); 
dz_next = dZ(:,2:end); 
defect_constraints = reshape(zNext - zPrev - (dz_prev+dz_next)/2 * dt,[],1); 

% Define initial condition constraint
z0 = sym('z0',[num_states,1]);
ic_constraint = zN(:,1) - z0; 

% Group all equality constraints
equality_constraints = [defect_constraints; ic_constraint]; 

% Define cost function - just squared error for now
Q = sym('Q',[num_states, num_states]);
R = sym('R',[num_inputs, num_inputs]);
zError = zN - zN_des; 
uError = uN - uN_des; 
zN_error_flat = reshape(zError,[],1); 
uN_error_flat = reshape(uError,[],1);
J = zN_error_flat.'* kron(eye(N_horizon_pts), Q) * zN_error_flat + ...
    uN_error_flat.'* kron(eye(N_horizon_pts), R) * uN_error_flat;

% Define the decision vector 
x = [uN(:); zN(:)];

% Define how to extract from the decision vector
uN_extracted = x(1:N_horizon_pts); 
zN_extracted = reshape(x(N_horizon_pts + 1:end), num_states, []); 

% Calculate QP Matrices
H = hessian(J, x); 
c = subs(jacobian(J,x),x, zeros(size(x)))'; 
Aeq = jacobian(equality_constraints, x); 
beq = -subs(equality_constraints, x, zeros(size(x))); 

% Export matlab functions
matlabFunction(H, 'File', './+Control/+MPC/H_func', 'Vars', {Q, R, T_horizon, zN_des, uN_des});
matlabFunction(c, 'File', './+Control/+MPC/c_func', 'Vars', {Q, R, T_horizon, zN_des, uN_des});
matlabFunction(Aeq, 'File', './+Control/+MPC/Aeq_func', 'Vars', {zN_exp, uN_exp, T_horizon});
matlabFunction(beq, 'File', './+Control/+MPC/beq_func', 'Vars', {zN_exp, uN_exp, z0, T_horizon});
matlabFunction(uN_extracted,'File', './+Control/+MPC/extract_uN_from_DV', 'Vars', {x});
matlabFunction(zN_extracted,'File', './+Control/+MPC/extract_zN_from_DV', 'Vars', {x});
matlabFunction(subs(N_horizon_pts_out, N_horizon_pts_out, N_horizon_pts),'File', './+Control/+MPC/getNumPtsForGeneratedCode');
end