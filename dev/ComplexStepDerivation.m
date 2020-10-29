function [jac, jac_cs] = ComplexStepDerivation()
%COMPLEXSTEPDERIVATION Development tool for complex step derivation

%   Detailed explanation goes here
clear f df df_cs X_pert;

% Data
X = [1 2 3 4 5]';

% Analytic derivation
syms xsym [5 1] real;
f = @(xsym) xsym(1) + xsym(2)^2 + xsym(3)^3;
df = jacobian(f(xsym),xsym);
jac = subs(df,xsym,X);

% Complex step derivation
df_cs = @(x,h) imag(f(x)) / h;
h = 1e-5;
N = numel(X);
jac_cs = nan([1 N]);
for k=1:N    
    X_pert      = X;
    X_pert(k)   = X_pert(k) + 1i*h;    
    jac_cs(k)   = df_cs(X_pert,h);    
end

