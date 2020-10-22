%% Observability Gramian Analysis

A =  [  
    0 , 0 , 0 ,  1 ,  0 , 0
    0 , 0 , 0 ,  0 ,  1 , 0
    0 , 0 , 0 ,  0 ,  0 , 1
    0 , 0 , 0 ,  0 ,  0 , 0
    0 , 0 , 0 ,  0 ,  0 , 0
    0 , 0 , 0 ,  0 ,  0 , 0
];

B = [
    0 , 0 , 0
    0 , 0 , 0
    0 , 0 , 0
    1 , 0 , 0
    0 , 1 , 0
    0 , 0 , 1
];

syms dt [1] real;
syms az real;
syms el real;
syms a [6 1] real;

H = [
    1 0 sin(el) * cot(az) 0 0 0
    0 1 cos(el) * cot(az) 0 0 0
];

[phi psi] = computeTransitionMatrices(A, dt);

H_t = H * phi;

Gramian = H_t' * H_t;

phi_rr = phi(1:3,1:3);
phi_rv = phi(1:3,4:6);
phi_vv = phi(4:6,4:6);
phi_vr = phi(4:6,1:3);



%% Compute State Transition Matrix
function [Phi, Psi] = computeTransitionMatrices(A, Delta_t)         
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
