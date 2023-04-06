function plotTrajectoriesAndPrediction(tout, z, torque, zMPC_hist, torqueMPC_hist, tHist)
numFirstOrderStates = size(z, 2); 
tiledlayout(numFirstOrderStates/2 + 1,numFirstOrderStates/2,'TileSpacing','compact')
stateNames = Ballbot.getStateNames();
for ix = 1:numFirstOrderStates
    ax(ix) = nexttile();
    plot(tout, z(:,ix),'LineWidth',2);
    hold on
    for iy = 1:size(zMPC_hist,3)
        if iy < size(zMPC_hist,3)
            color = [.2,.2,.2,.2];
        else
            color = [1,0,0,1];
        end

        plot(tHist(:,iy), zMPC_hist(ix,:,iy),"Color", color)
    end

    ylabel(stateNames{ix}, 'interpreter','latex','FontName','times')
    xlabel('Time $t$ (s)', 'interpreter','latex','FontName','times')
    set(gca, 'FontName', 'times')
end

nexttile([1,2])
plot(tout, torque,'LineWidth',2);
hold on
for iy = 1:size(zMPC_hist,3)
    if iy < size(zMPC_hist,3)
        color = [.2,.2,.2,.2];
    else
        color = [1,0,0,1];
    end
    plot(tHist(:,iy), torqueMPC_hist(:,iy),"Color", color)
end
ylabel({'Control Input','$T$ (Nm)'}, 'interpreter','latex','FontName','times')
xlabel('Time $t$ (s)', 'interpreter','latex','FontName','times')
set(gca, 'FontName', 'times')

end