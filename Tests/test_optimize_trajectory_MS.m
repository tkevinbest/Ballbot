clear
close all

% Load ballbot params
[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.25;

% Number of nodes
N = 11; 

% Constraints
qLim = [
        -1/rk, 1.5/rk;
        -deg2rad(25), deg2rad(25)
    ];
qdotLim = [
        -15, 15;
        -2, 2
    ];
uLim = [-10, 10]; 

% Run MS
q0 = [0;0];
qdot0 = [0;0];
qdes = [desX/rk;NaN];
qdot_des = [NaN; 0];

tic
[ustar, qstar, qdotstar, tstar, Tfstar, finalCost, fminconout] = ...
        Control.optimize_trajectory_MS(q0, qdot0, qdes, qdot_des, N, qLim, qdotLim, uLim);
solveTime = toc;
disp(['MS Optimization with ', num2str(N),' nodes completed in ',num2str(solveTime),' seconds.'])

% Call plot
z = interleave2(qstar, qdotstar, 'row'); 
Ballbot.plotTrajectories(tstar, z', ustar); 

% Animate ballbot
Ballbot.animate(tstar, qstar','MultipleShootingTest_notMex.mp4'); 