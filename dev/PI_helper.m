% x_dot = A*x;
% y = h(x);
A = [   0 , 0 , 0 , 1 , 0 , 0
        0 , 0 , 0 , 0 , 1 , 0
        0 , 0 , 0 , 0 , 0 , 1
        0 , 0 , 0 , 0 , 0 , 0
        0 , 0 , 0 , 0 , 0 , 0
        0 , 0 , 0 , 0 , 0 , 0
    ];
b = [zeros(3);eye(3)];


% Pseudo-linearized measurement function
% 0 = H*x;
sym('az','real');
sym('el','real');

H = [
        1   , 0 , -sin(az)*cot(el)  , 0 , 0 , 0
        0   , 1 , -cos(az)*cot(el)  , 0 , 0 , 0
    ];

C = [
        H
        H*A
        H*A^2
        H*A^3
        H*A^4
        H*A^5
    ];

gram = C'*C;
clear jac vec h1 h2;
jac = sym('J',[2,6]);
vec = sym('x',[6,1],'real');
n_x = length(vec);
c1 = atan2(vec(2),vec(1));
c2 = atan2(-vec(3),sqrt(vec(1)^2 + vec(2)^2));
jac(1,:) = jacobian(c1,vec);
jac(2,:) = jacobian(c2,vec);




