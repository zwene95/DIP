function [x_S, P_S] = UT_SC(x_C,P_C)
%UT_C2S Unscented Transform from Spherical into Cartesian Coordinates
%   UT is required for the covariance transformation
% Inputs:
%   Cartesian state vector: q_C = [x y z x_dot y_dot z_dot]' 
%   Cartesian covariance: P_C
% Outputs:
%   Spherical state vector: x_S = [r b e r_dot b_dot e_dot]'
%   Spherical covariance P_S

% Define Measurement equations: x_S = S(x_C)
f_SC = @(q)[...
     sqrt(q(1)^2 + q(2)^2 + q(3)^2)
     atan2(q(2),q(1))
     atan2(-q(3),sqrt(q(1)^2 + q(2)^2))
    (q(1)*q(4) + q(2)*q(5) + q(3)*q(6)) / sqrt(q(1)^2 + q(2)^2 + q(3)^2)
    (q(1)*q(5) - q(2)*q(4)) / (q(1)^2 + q(2)^2)
    (q(1)*q(3)*q(4) + q(2)*q(3)*q(5) - (q(1)^2 + q(2)^2)*q(6)) / (sqrt(q(1)^2 + q(2)^2) * (q(1)^2 + q(2)^2 + q(3)^2))];

n_x = numel(x_C);                                                           % numer of states
alpha = 1e-3;                                                               % default, tunable
ki = 0;                                                                     % default, tunable
beta = 2;                                                                   % default, tunable
lambda = alpha^2*(n_x+ki)-n_x;                                              % scaling factor
c = n_x + lambda;                                                           % scaling factor
Wm=[lambda/c 0.5/c+zeros(1,2*n_x)];                                         % weights for means
Wc = Wm;                                                                    % weights for covariance
Wc(1)=Wc(1)+(1-alpha^2+beta);                                               
c=sqrt(c);
X=sigmas(x_C,P_C,c);                                                        % sigma points around x
[x_S,P_S] = ut(f_SC,X,Wm,Wc,n_x);                                                % unscented transformation

P_S = real(P_S);


end

function [y,P]=ut(f,X,Wm,Wc,n_q)
%Unscented Transformation
%Input:
%        f: nonlinear map
%        X: sigma points
%       Wm: weights for mean
%       Wc: weights for covraiance
%        n: numer of outputs of f
%        R: additive covariance
%Output:
%        y: transformed mean
%        Y: transformed smapling points
%        P: transformed covariance
%       Y1: transformed deviations

L=size(X,2);
y=zeros(n_q,1);
Y=zeros(n_q,L);
for k=1:L
    Y(:,k)=f(X(:,k));
    y=y+Wm(k)*Y(:,k);
end
Y1=Y-y(:,ones(1,L));
P=Y1*diag(Wc)*Y1';
end

function X=sigmas(x,P,c)
%Sigma points around reference point
%Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
%Output:
%       X: Sigma points

A = c*chol(P)';
Y = x(:,ones(1,numel(x)));
X = [x Y+A Y-A];
end