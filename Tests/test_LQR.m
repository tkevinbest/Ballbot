clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 0.0;

% Define desired states
q0 = [-pi;0];
qdot0 = [0;0];
qdes = [desX/rk;0];
qdot_des = [0; 0];

T_motion = 4; 
qstar = qdes;
qdotstar = qdot_des; 
ustar = 0;

%% Run LQR to stabilize a desired point
dt = 0.01; % Real time sample rate
t_sim = 0:dt:T_motion; % Run simulation for 25% longer than the expected trajectory to see what happens

% Create z vector, keeps life tidy
zstar = interleave2(qstar, qdotstar, 'row'); 
zdes = interleave2(qdes, qdot_des, 'row');

% Get the linearized dynamics about desired position
A_dyn = Ballbot.A_lin_symb(zstar, ustar); 
B_dyn = Ballbot.B_lin_symb(zstar, ustar); 

% Solve optimal control
Q = diag([20, 1, .5,.5]); 
R = 1; 
k = lqr(A_dyn, B_dyn, Q, R); 
forceFunc = @(t,z) -k*z; 

% Run simulation
[t, q, qdot] = Ballbot.runSimulation(q0, qdot0, forceFunc, t_sim, true); 

%% Animate the solution
Ballbot.animate(t_sim, q, 'TestLQR.mp4');

%% Validate the solution using lsim
sys = ss(A_dyn-B_dyn*k, [0;0;0;0], eye(4),0);
[y,tout] = lsim(sys, zeros(size(t_sim)), t_sim, interleave2(q0, qdot0,'row'));
figure
plot(t,y);





