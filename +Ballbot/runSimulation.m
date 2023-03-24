function [tout, qout, qdotOut, qout_first_order, torque] = runSimulation(q0, qdot0, torqueFuncHandle, tspan, plotIt)
q0_firstOrder = interleave2(q0, qdot0, 'row');
[tout, qout_first_order] = ode45(@(t,q)ballbotPlanarODE(t,q,torqueFuncHandle), tspan, q0_firstOrder);

qout = [qout_first_order(:,1),qout_first_order(:,3)];
qdotOut = [qout_first_order(:,2),qout_first_order(:,4);];

% Recover input trajectory
torque = zeros(size(tout)); 
for ix = 1:length(tout)
    torque(ix) = torqueFuncHandle(tout(ix), qout(ix,:)); 
end

if plotIt
    Ballbot.plotTrajectories(tout, qout_first_order, torque); 
end


end


function qdot_firstOrder = ballbotPlanarODE(t,q_firstOrder, torqueFuncHandle)
% Extract the states
q = [q_firstOrder(1);q_firstOrder(3)];
qdot = [q_firstOrder(2);q_firstOrder(4);];

% Get control torque
T = torqueFuncHandle(t, q); 

M = Ballbot.M(q);
f = Ballbot.f(q, qdot, T); 

qddot = M\f;
qdot_firstOrder = interleave2(qdot, qddot, 'row'); 
qdot_firstOrder = reshape(qdot_firstOrder,[],1);
end