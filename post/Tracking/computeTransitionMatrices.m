function [Phi, Psi] = computeTransitionMatrices(Delta_t, A)         
%COMPUTETRANSITIONMATRICES compute state transition matrices
%   compute state transition matrices with first 5 elements of infinite
%   sum. This is preferred over the integral formulation, since the sum
%   always converges, whereas it is not guarantueed, that A is invertible.
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
                  
% initialize transition matrices
Phi = eye(size(A));                                                         
Psi = eye(size(A)) * Delta_t;                                                

A_pow = eye(size(A));                                                   
Delta_t_pow = 1;                                                         
                                                                            
for i = 1:5    
    % compute next power of A               - A^i
    A_pow = A_pow * A;
    
    % compute next power of Delta_t         - delta_t^i / (i!)
    Delta_t_pow = Delta_t_pow * Delta_t / i;  
    
    % compute Phi                           - sum (A * Delta_t)^i / (i!)
    Phi = Phi + A_pow * Delta_t_pow;                 
    
    % compute Psi                           - sum A^(i-1) Delta_t^i /(i!)
    Psi = Psi + A_pow * Delta_t_pow * Delta_t / (i + 1);         
end                                                                         
                                                                            
end

% --- EOF                                       COPYRIGHT © 2016, FSD - TUM