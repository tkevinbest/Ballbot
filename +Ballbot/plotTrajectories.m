function plotTrajectories(tout, qout_first_order, torque)
numFirstOrderStates = size(qout_first_order, 2); 
tiledlayout(numFirstOrderStates/2 + 1,numFirstOrderStates/2,'TileSpacing','compact')
stateNames = Ballbot.getStateNames();
for ix = 1:numFirstOrderStates
    ax(ix) = nexttile();
    plot(tout, qout_first_order(:,ix),'LineWidth',2);
    ylabel(stateNames{ix}, 'interpreter','latex','FontName','times')
    xlabel('Time $t$ (s)', 'interpreter','latex','FontName','times')
    set(gca, 'FontName', 'times')
end

nexttile([1,2])
plot(tout, torque,'LineWidth',2);
ylabel({'Control Input','$T$ (Nm)'}, 'interpreter','latex','FontName','times')
xlabel('Time $t$ (s)', 'interpreter','latex','FontName','times')
set(gca, 'FontName', 'times')

end