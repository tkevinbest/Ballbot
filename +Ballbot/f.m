function f = f(in1,in2,Tx,t)
%F
%    F = F(IN1,IN2,Tx,T)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    27-Mar-2023 10:21:10

dthetax_ = in2(2,:);
thetax_ = in1(2,:);
t2 = sin(thetax_);
t3 = Tx.*(2.5e+1./1.2e+1);
f = [t3+dthetax_.^2.*t2.*4.59225e-1;t2.*3.6039978e+1-t3];
