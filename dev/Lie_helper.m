% x_dot = a(x) + b(x) * u ;
% y = c(x);
a = [   0 , 0 , 0 , 1 , 0 , 0
        0 , 0 , 0 , 0 , 1 , 0
        0 , 0 , 0 , 0 , 0 , 1
        0 , 0 , 0 , 0 , 0 , 0
        0 , 0 , 0 , 0 , 0 , 0
        0 , 0 , 0 , 0 , 0 , 0
    ];
b = [zeros(3);eye(3)];


clear jac vec h1 h2;
jac = sym('J',[2,6]);
vec = sym('x',[6,1],'real');
n_x = length(vec);
c1 = atan2(vec(2),vec(1));
c2 = atan2(-vec(3),sqrt(vec(1)^2 + vec(2)^2));
jac(1,:) = jacobian(c1,vec);
jac(2,:) = jacobian(c2,vec);


La_c = jac * a;
Lb_c = jac * b;

dLa_c_1 = [
            jacobian(La_c(1,:),vec)
            jacobian(La_c(2,:),vec)
];

La_c2 = dLa_c_1 * a;
Lb_c2 = dLa_c_1 * b;


dLa_c_2 = [
            jacobian(La_c2(1,:),vec)
            jacobian(La_c2(2,:),vec)
            jacobian(La_c2(3,:),vec)
            jacobian(La_c2(4,:),vec)
            jacobian(La_c2(5,:),vec)
            jacobian(La_c2(6,:),vec)
            jacobian(La_c2(7,:),vec)
            jacobian(La_c2(8,:),vec)
            jacobian(La_c2(9,:),vec)
            jacobian(La_c2(10,:),vec)
            jacobian(La_c2(11,:),vec)
            jacobian(La_c2(12,:),vec)
            
];

La_c3 = dLa_c_2 * a;
Lb_c3 = dLa_c_2 * b;

dLa_c_3 = jacobian(La_c3(1,:),vec);
for i = 2:length(La_c3)
    dLa_c_3 = [dLa_c_3; jacobian(La_c3(i,:),vec)];
end
La_c4 = dLa_c_3 * a;
Lb_c4 = dLa_c_3 * b;


dLa_c_4 = jacobian(La_c4(1,:),vec);
for i = 2:length(La_c4)
    dLa_c_4 = [dLa_c_4; jacobian(La_c4(i,:),vec)];
end
La_c5 = dLa_c_4 * a;
Lb_c5 = dLa_c_4 * b;


dLa_c_5 = jacobian(La_c5(1,:),vec);
for i = 2:length(La_c5)
    dLa_c_5 = [dLa_c_5; jacobian(La_c5(i,:),vec)];
end
La_c6 = dLa_c_5 * a;
Lb_c6 = dLa_c_5 * b;


% dLa_c_6 = jacobian(La_c6(1,:),vec);
% for i = 2:length(La_c6)
%     dLa_c_6 = [dLa_c_6; jacobian(La_c6(i,:),vec)];
% end
% La_c7 = dLa_c_6 * a;
% Lb_c7 = dLa_c_6 * b;


