function B_lin_symb = B_lin_symb(in1,u1)
%B_lin_symb
%    B_lin_symb = B_lin_symb(IN1,U1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    30-Mar-2023 16:53:04

z3 = in1(3,:);
t2 = cos(z3);
t3 = t2.^2;
t4 = t2.*1.129604210453735e+38;
t5 = t3.*1.710923478483023e+39;
t6 = -t5;
t7 = t4+t6+1.255930905379549e+40;
t8 = 1.0./t7;
B_lin_symb = [0.0;t8.*(t2.*3.638355293061304e+34+4.696031349948555e+35).*2.133333333333333e+5;0.0;t8.*(t2.*2.842465072704144e+36+1.518887952159449e+36).*(-2.730666666666667e+3)];
