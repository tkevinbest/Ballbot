% Test simulation
clear
close all

% Define initial conditions
q0 = [6;0];
qdot0 = [0;0];

% Define simulation timespan
tspan = linspace(0,2,10000); 

% Plot? T/F
plotIt = true; 

% Call simulation
[t,q, qdot, z, torque] = Ballbot.runSimulation(q0, qdot0, @forceFunc, tspan, plotIt);

% Check energy
Ballbot.plotEnergy(t, z, torque); 

% Animate ballbot
Ballbot.animate(t, q,'HelloWorld_Ballbot.mp4'); 


% Define control function
function F = forceFunc(t,q)
F = interp1([0, .4, .6, .9, 1, 5],[2, 1, -8,-7, 3, 0], t);
% if t<.5
%     F = 1;
% elseif t < 1
%     F = -5;
% else
%     F = 3;
% end
% F = (t+1)/2.5*sin(2*pi*t);
end