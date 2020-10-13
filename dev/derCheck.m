clc; clear;
syms x y z u v w xi yi zi ui vi wi real;

par = [x y z u v w xi yi zi ui vi wi]';

rDOO = [x;y;z];
vDOO = [u;v;w];
rVOO = [xi;yi;zi];
vVOO = [ui;vi;wi];

rVDO = rVOO - rDOO;
vVDO = vVOO - vDOO;

rVDO_dp =  dot(rVDO,rVDO);

wVDO =  cross(rVDO,vVDO) / rVDO_dp;

J = jacobian(wVDO,par);
H1 = hessian(wVDO(1),par);
H2 = hessian(wVDO(2),par);
H3 = hessian(wVDO(3),par);

% f_dx = diff(wVDO,x);
par0 = [0 0 0 0 0 0 50 0 -50 0 0 0];

subs(J,par',par0)
subs(H1,par',par0)
subs(H2,par',par0)
subs(H3,par',par0)
% subs(wVDO,par',par0)
