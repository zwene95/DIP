function [ dy_dx ] = finiteDifferences( fun, x )
% FINITEDIFFERENCES compute central finite differences
%   This function implements a generic, central finite difference scheme,
%   which can be used to compute jacobians of function 'fun', with respect
%   to independent variables 'x'. 'fun' has to be a function handle, which
%   can be evaluated as fun(x).
%
%   if y = fun(x) is a matrix, a three dimensional derivative is returned,
%   where the second dimension is that of the derivatives with respect to
%   x.
%
%   see also: ExtendedKalmanFilter
%
%
% -------------------------------------------------------------------------
%   This file comes as part of the accompanying material of the lecture
%   'system identification' by the Institute of Flight System Dynamics of
%   the Technical University of Munich.
%
%   All use of this file for purposes not related to the lecture 'system
%   identification' are prohibited, especially commercial use of the
%   presented material.
%
%
% COPYRIGHT © 2016, Institute of Flight System Dynamics (FSD)
%                   Technische Universität München (TUM)
% Author            Christoph Göttlicher

% dummy-evaluation to obtain dimensions
y = fun(x);
n_y = size(y, 1);
n_x = length(x);

% get sample size
N = size(y, 2);

% pre-allocate memory
dy_dx = nan(n_y, n_x, N);

% set step size
h = 1e-6;
for i=1:n_x
    % initialize perturbed parameter values
    x_plus = x;
    x_minus = x;
    x_plus(i) = x(i) + h;
    x_minus(i) = x(i) - h;
    
    % evaluate function for perturbed values
    y_plus = fun(x_plus);
    y_minus = fun(x_minus);
    
    % assemble results
    dy_dx(:,i,:) = (y_plus - y_minus) / (2*h);
end

end

% --- EOF                                       COPYRIGHT © 2016, FSD - TUM