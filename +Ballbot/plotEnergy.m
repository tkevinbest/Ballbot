function plotEnergy(tout, qout_first_order, torque)
if size(qout_first_order,2) > 4
    qout_first_order = qout_first_order.';
end
[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.loadBallbotParams();

for ix = 1:length(tout)
    phi = qout_first_order(ix,1); 
    phidot = qout_first_order(ix,2); 
    theta = qout_first_order(ix,3); 
    thetadot = qout_first_order(ix, 4); 
    wheelVelocity(ix) = rk/rw*(phidot)+thetadot; 
    Ta(ix) = Ballbot.Ta(phidot, thetadot, theta, phi, tout(ix)); 
    Tk(ix) = Ballbot.Tk(phidot, thetadot, theta, phi, tout(ix)); 
    Tw(ix) = Ballbot.Tw(phidot, thetadot, theta, phi, tout(ix)); 
    Va(ix) = Ballbot.Va(phidot, thetadot, theta, phi, tout(ix)); 
    Vw(ix) = Ballbot.Vw(phidot, thetadot, theta, phi, tout(ix)); 
    power(ix) = ([rk/rw*torque(ix), -(1+rk/rw)*torque(ix)] + [0, torque(ix)])*[phidot; thetadot];
end
totalBallbotEnergy = Ta+Tk+Tw+Va+Vw;
figure
plot(tout, totalBallbotEnergy - totalBallbotEnergy(1), 'LineWidth',2)
ylabel({'Energy','(J)'}, 'interpreter','latex','FontName','times')
xlabel('Time $t$ (s)', 'interpreter','latex','FontName','times')
set(gca, 'FontName', 'times')

% Calculate input power
dt = mode(diff(tout)); 
energy = cumtrapz(power) * dt; 
hold on
plot(tout, energy, '--', 'LineWidth',2)
legend('System','Input')
legend boxoff
end