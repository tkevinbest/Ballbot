function [qLim, qdotLim, uLim] = defineConstraints(type)
[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

switch type
    case Control.OptimizationType.MS
        qLim = [
                -1/rk, 1.5/rk;
                -deg2rad(30), deg2rad(30)
            ];
        qdotLim = 5*[
                -15, 15;
                -2, 2
            ];
        uLim = [-10, 10]; 
    case Control.OptimizationType.MPC
        qLim = [
                -3/rk, 3/rk;
                -deg2rad(90), deg2rad(90)
            ];
        qdotLim = [
                -500, 500;
                -20, 20
            ];
        uLim = 20*[-20, 20]; 
end

end