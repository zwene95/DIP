%%
% x_dot = a(x) + b(x) * u ;
% y = c(x);
A = [   0 , 0 , 0 , 1 , 0 , 0
        0 , 0 , 0 , 0 , 1 , 0
        0 , 0 , 0 , 0 , 0 , 1
        0 , 0 , 0 , 0 , 0 , 0
        0 , 0 , 0 , 0 , 0 , 0
        0 , 0 , 0 , 0 , 0 , 0
    ];
B = [zeros(3);eye(3)];

clear jac vec;
jac = sym('J',[2,6]);
syms x y z u v w real;
states = [x y z u v w]'; %sym('x',[6,1],'real');
u = sym('a',[3,1],'real');
n_x = length(states);
g1 = atan2(states(2),states(1));
g2 = atan2(-states(3),sqrt(states(1)^2 + states(2)^2));
g = [g1;g2];
jac(1,:) = jacobian(g1,states);
jac(2,:) = jacobian(g2,states);

f = A*states+B*u;

%%
Lf_g_0 = g;

%%
Lf_g_1 = jacobian(Lf_g_0,states) * f;
% La_c_1 = jacobian(Lf_c_0,x) * A*x;
% Lb_c_1 = jacobian(Lf_c_0,x) * B;

%%
Lf_g_2 = jacobian(Lf_g_1,states) * f;
% La_c_2 = jacobian(La_c_1,x) * A*x;
% Lb_c_2 = jacobian(La_c_1,x) * B;

%%
Lf_g_3 = jacobian(Lf_g_2,states) * f;

%%
Lf_g_4 = jacobian(Lf_g_3,states) * f;

%%
Lf_g_5 = jacobian(Lf_g_4,states) * f;

% Vereinfachung mit: simplify(Lf_g_1)