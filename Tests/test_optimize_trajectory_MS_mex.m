clear
close all

[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

% Desired X position
desX = 1.0;

% Number of nodes
N = 26; 

% Run MS
q0 = [-.5/rk;0];
qdot0 = [0;0];
qdes = [desX/rk;0];
qdot_des = [0; 0];

tic
[ustar, qstar, qdotstar, tstar, Tfstar, finalCost, fminconout] = ...
        Control.optimize_trajectory_MS_mex(q0, qdot0, qdes, qdot_des, N);
solveTime = toc;
disp(['MS Optimization with ', num2str(N),' nodes completed in ',num2str(solveTime,'%.2f'),' seconds.'])

% Call plot
z = interleave2(qstar, qdotstar, 'row'); 
Ballbot.plotTrajectories(tstar, z', ustar); 

% Animate ballbot
Ballbot.animate(tstar, qstar','testMS.mp4'); 

%% Save still frames
Ballbot.saveStillFrame(qstar(:,[1,6,14,18,N]),'MultipeShootingTest_mex');