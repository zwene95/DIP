%% Compute first and second order time derivatives of the measurement functions
syms t x(t) y(t) z(t) real;
assume(x(t),'real');
assume(y(t),'real');
assume(z(t),'real');
g1 = atan2(y,x); 
g2 = atan2(-z,sqrt(x^2 + y^2));
jac_g1 = diff(g1,t);
jac_g2 = diff(g2,t)