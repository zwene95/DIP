function x_kPlus1 = ode4_step(f, delta_t, t, x_k, u_k, u_kPlus1)
%ODE4_STEP fourth order Runge-Kutta step
%   This implements one integration step in the fourth order Runge-Kutta
%   integration scheme
%
%   see also: ExtendedKalmanFilter, UnscentedKalmanFilter
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

% compute intermediate terms
f_1 = feval(f, x_k,                 u_k,                    t);
f_2 = feval(f, x_k+0.5*delta_t*f_1, 0.5*(u_k + u_kPlus1),   t+0.5*delta_t);
f_3 = feval(f, x_k+0.5*delta_t*f_2, 0.5*(u_k + u_kPlus1),   t+0.5*delta_t);
f_4 = feval(f, x_k+delta_t*f_3,     u_kPlus1,               t+delta_t);

% combine to obtain state at next time-step
x_kPlus1 = x_k + (delta_t/6)*(f_1 + 2*f_2 + 2*f_3 + f_4);

end

% --- EOF                                       COPYRIGHT © 2016, FSD - TUM