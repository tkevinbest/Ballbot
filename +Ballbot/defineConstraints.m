function [qLim, qdotLim, uLim] = defineConstraints()
[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();
qLim = [
        -5/rk, 1.5/rk;
        -deg2rad(30), deg2rad(30)
    ];
qdotLim = 5*[
        -15, 15;
        -2, 2
    ];
uLim = [-10, 10]; 
end