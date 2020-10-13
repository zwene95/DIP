%% Compute the Jacobian analytically
% vec = [x y z u v w];


clear jac vec h1 h2;
jac = sym('J',[2,6]);
vec = sym('x',[6,1],'real');
h1 = atan2(vec(2),vec(1));
h3 = asin(vec(2)/sqrt(vec(2)^2 + vec(1)^2));
h2 = atan2(-vec(3),sqrt(vec(1)^2 + vec(2)^2));
jac(1,:) = jacobian(h1,vec);
jac(2,:) = jacobian(h2,vec);
jac


% clear jac vec h1 h2;
% jac = sym('J',[2,6]);
% vec = [sym('x',[3,1],'real'); sym('y',[3,1],'real')];
% h1 = atan2(vec(5) - vec(2),vec(4) - vec(1));
% h2 = atan2(-(vec(6) - vec(3)),sqrt((vec(4)-vec(1))^2 + (vec(5)-vec(2))^2));
% jac(1,:) = jacobian(h1,vec);
% jac(2,:) = jacobian(h2,vec);
% jac'

% clear jac vec h1 h2;
% hes = sym('J',[12,6]);
% vec = [sym('x',[3,1],'real'); sym('y',[3,1],'real')];
% h1 = atan2(vec(5) - vec(2),vec(4) - vec(1));
% h2 = atan2(-(vec(6) - vec(3)),sqrt((vec(4)-vec(1))^2 + (vec(5)-vec(2))^2));
% hes(1:6,:) = hessian(h1,vec);
% hes(7:12,:) = hessian(h2,vec);
% hes

% clear jac vec h1 h2;
% jac = sym('J',[2,3]);
% vec = sym('x',[3,1],'real');
% h1 = atan2(vec(2),vec(1));
% h2 = atan2(-vec(3),sqrt(vec(1)^2 + vec(2)^2));
% jac(1,:) = jacobian(h1,vec);
% jac(2,:) = jacobian(h2,vec);
% jac'