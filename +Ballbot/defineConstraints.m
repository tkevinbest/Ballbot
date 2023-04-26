function [qLim, qdotLim, uLim] = defineConstraints(type)
[mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

switch type
    case Control.OptimizationType.MS
        qLim = [
                -1/rk, 1.5/rk;
                -deg2rad(30), deg2rad(30)
            ];
        qdotLim = [
                -75, 75;
                -10, 10
            ];
        uLim = [-10, 10]; 
    case Control.OptimizationType.MPC
        qLim = [
                -3/rk, 3/rk;
                -deg2rad(45), deg2rad(45)
            ];
        qdotLim = 1.5*[
                -75, 75;
                -10, 10
            ];
        uLim = 5*[-10, 10]; 
end

end