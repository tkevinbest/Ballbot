function [mk, mw, ma, rk, rw, ra,l, omegaK, omegaW, omegaA, g] = defineParams()
% Ballbot parameters -> From Table 2.1
mk = 2.29; 
mw = 3;
ma = 9.2;
rk = 0.125;
rw = 0.06;
ra = .1;
l = 0.339;
omegaK = 0.0239;
omegaW = 0.00236;
omegaA = 4.76;

% General constants
g = 9.81;
end