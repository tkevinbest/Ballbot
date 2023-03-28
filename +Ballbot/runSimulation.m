function [tout, qout, qdotOut, z, torque] = runSimulation(q0, qdot0, torqueFuncHandle, tspan, plotIt)
q0_firstOrder = interleave2(q0, qdot0, 'row');
[tout, z] = ode45(@(t,q)ballbotPlanarODE(t,q,torqueFuncHandle), tspan, q0_firstOrder);

qout = [z(:,1),z(:,3)];
qdotOut = [z(:,2),z(:,4);];

% Recover input trajectory
torque = zeros(size(tout)); 
for ix = 1:length(tout)
    torque(ix) = torqueFuncHandle(tout(ix), z(ix,:)'); 
end

if plotIt
    Ballbot.plotTrajectories(tout, z, torque); 
end


end


function qdot_firstOrder = ballbotPlanarODE(t,z, torqueFuncHandle)
% Extract the states
q = [z(1);z(3)];
qdot = [z(2);z(4);];

% Get control torque
T = torqueFuncHandle(t, z); 

M = Ballbot.M(q);
f = Ballbot.f(q, qdot, T); 

qddot = M\f;
qdot_firstOrder = interleave2(qdot, qddot, 'row'); 
qdot_firstOrder = reshape(qdot_firstOrder,[],1);
end