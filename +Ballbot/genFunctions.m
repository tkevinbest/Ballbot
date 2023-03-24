% Calculate Ballbot Dynamics
% Kevin Best, March 2023

function genFunctions()

% Load relevant ballbot params
[mk, mw, ma, rk, rw, ra, l, omegaK, omegaW, omegaA, g] = Ballbot.defineParams();

syms t
syms phix(t) phiz(t)
syms thetax(t) thetaz(t)
syms psiz(t)
syms Tx
phidotx = diff(phix,t);
phidotz = diff(phiz,t);
thetadotx = diff(thetax,t);
thetadotz = diff(thetaz,t);
psidotz = diff(psiz, t); 

% Generalized coordinates
q = [phix; thetax];
qdot = [phidotx; thetadotx];

% Energy of the ball
Tk = 1/2*mk*(rk*phidotx)^2 + 1/2*omegaK*phidotx^2;
Vk = 0;

% Energy of the actuating wheel
Tw = 1/2*mw*( ...
    (rk*phidotx)^2 + 2 * (rk+rw)*cos(thetax)*thetadotx*(rk*phidotx) + (rk+rw)^2*thetadotx^2) ...
    + 1/2*omegaW*(rk/rw*(phidotx - thetadotx) - thetadotx)^2;
Vw = mw*g*(rk+rw)*cos(thetax); 

% Energy of the body
Ta = 1/2*ma*( ...
    (rk*phidotx)^2 + 2*l*cos(thetax)*thetadotx*(rk*phidotx) + l^2*thetadotx^2)...
    + 1/2*omegaA*thetadotx^2 ;
Va = ma*g*l*cos(thetax);

% Non potential forces
Fnp1 = [
        rk/rw*Tx   ;
        -1*(1 + rk/rw)*Tx
        ];
Fnp2 = [0; Tx];
% Fnp_xy = [-Tf]

Tyz = Tk + Tw + Ta;
Vyz = Vk + Vw + Va; 
Fnp = Fnp1 + Fnp2;


%% Calculate lagrangian
L = Tyz - Vyz; 
dL_d_qdot = jacobian(L,qdot).';

%LHS of EL eqn
EL_eq = diff(dL_d_qdot,t) - jacobian(L,q).' - Fnp; 

% Substitutions to eliminate dependence on t and diffs
syms ddthetax_ ddphix_ dthetax_ dphix_ thetax_ phix_ 
EL_eq = subs(EL_eq, {diff(thetax(t),t,t), diff(phix(t),t,t)}, {ddthetax_, ddphix_});
EL_eq = subs(EL_eq, {diff(thetax(t),t), diff(phix(t),t)}, {dthetax_, dphix_});
EL_eq = subs(EL_eq, {thetax(t), phix(t)}, {thetax_, phix_});

% Write as robot equations
M = jacobian(EL_eq, [ddphix_; ddthetax_]); % Verified that this matches 2.22
f = -subs(EL_eq, {ddphix_; ddthetax_},{0;0});

% These don't explicitly depend on time - come on matlab
M = subs(M, t, 0);
f = subs(f, t, 0); 

% Save functions
matlabFunction(M,'File','./+Ballbot/M.m','Vars',{[phix_; thetax_], t});
matlabFunction(f,'File','./+Ballbot/f.m','Vars',{[phix_; thetax_],[dphix_; dthetax_], Tx, t});

% Get energy functions
Tk_subsd = subs(Tk, {phidotx, thetadotx, thetax, phix}, {dphix_, dthetax_, thetax_, phix_});
Tw_subsd = subs(Tw, {phidotx, thetadotx, thetax, phix}, {dphix_, dthetax_, thetax_, phix_});
Vw_subsd = subs(Vw, {phidotx, thetadotx, thetax, phix}, {dphix_, dthetax_, thetax_, phix_});
Ta_subsd = subs(Ta, {phidotx, thetadotx, thetax, phix}, {dphix_, dthetax_, thetax_, phix_});
Va_subsd = subs(Va, {phidotx, thetadotx, thetax, phix}, {dphix_, dthetax_, thetax_, phix_});
argumentVars = {dphix_, dthetax_, thetax_, phix_, t};
matlabFunction(Tk_subsd,'File','./+Ballbot/Tk','Vars',argumentVars);
matlabFunction(Tw_subsd, 'File','./+Ballbot/Tw','Vars',argumentVars); 
matlabFunction(Vw_subsd, 'File','./+Ballbot/Vw','Vars',argumentVars); 
matlabFunction(Ta_subsd, 'File','./+Ballbot/Ta','Vars',argumentVars); 
matlabFunction(Va_subsd, 'File','./+Ballbot/Va','Vars',argumentVars); 

end
