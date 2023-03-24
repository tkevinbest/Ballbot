function rCOM = calcCOM_location(q)
[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.loadBallbotParams();

rBC = [q(:,1)*rk, rk*ones(size(q,1),1)]; 
rCOM = rBC + l*[sin(q(:,2)), cos(q(:,2))];
end