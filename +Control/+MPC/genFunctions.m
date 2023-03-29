function genFunctions(N_horizon_pts)
num_states = 4;
num_inputs = 1; 

syms T_horizon
dt = T_horizon/(N_horizon_pts-1); 
syms N_horizon_pts_out

% Decision variables
zN = sym('zN',[num_states, N_horizon_pts]);
uN = sym('uN', [num_inputs, N_horizon_pts]);

% Calculate dynamics
A = sym('A',[num_states, num_states,N_horizon_pts]);
B = sym('B',[num_states, num_inputs,N_horizon_pts]);
for ix = 1:size(A,3)
    dZ(:,ix) = A(:,:,ix)*zN(:,ix) + B(:,:,ix)*uN(:,ix); 
end

% Define defect constraints through explicit Euler
zNext = zN(:, 2:end);
zPrev = zN(:, 1:end-1); 
dz_prev = dZ(:,1:end-1); 
defect_constraints = reshape(zNext - zPrev - dz_prev * dt,[],1); 

% Define initial condition constraint
z0 = sym('z0',[num_states,1]);
ic_constraint = zN(:,1) - z0; 

% Group all equality constraints
equality_constraints = [defect_constraints; ic_constraint]; 

% Define cost function - just squared error for now
Q = sym('Q',[num_states, num_states]);
R = sym('R',[num_inputs, num_inputs]);
zN_flat = reshape(zN,[],1); % Contains [z1; z2; ...; zN]
uN_flat = uN(:);
J = zN_flat.'* kron(eye(N_horizon_pts), Q) * zN_flat + ...
    uN_flat.'* kron(eye(N_horizon_pts), R) * uN_flat;

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
matlabFunction(H, 'File', './+Control/+MPC/H_func', 'Vars', {Q, R, T_horizon});
matlabFunction(c, 'File', './+Control/+MPC/c_func', 'Vars', {Q, R, T_horizon});
matlabFunction(Aeq, 'File', './+Control/+MPC/Aeq_func', 'Vars', {A, B, T_horizon});
matlabFunction(beq, 'File', './+Control/+MPC/beq_func', 'Vars', {A, B, z0, T_horizon});
matlabFunction(uN_extracted,'File', './+Control/+MPC/extract_uN_from_DV', 'Vars', {x});
matlabFunction(zN_extracted,'File', './+Control/+MPC/extract_zN_from_DV', 'Vars', {x});
matlabFunction(subs(N_horizon_pts_out, N_horizon_pts_out, N_horizon_pts),'File', './+Control/+MPC/getNumPtsForGeneratedCode');
end