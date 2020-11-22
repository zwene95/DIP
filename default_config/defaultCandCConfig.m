function [ CCConfig ] = defaultCandCConfig()
% Default Cost and Constraint Configuration

CCConfig = struct();

% Cost configuration
CCConfig.Missdistance.Cost          = 0;
CCConfig.Time.Cost                  = 0;
CCConfig.TargetViolation.Cost       = 1;
% CCConfig.ObserverCov.Cost           = 1;
% CCConfig.ObserverRMSE.Cost          = 0;
% Constraint configuration
CCConfig.Thrust.Constraint          = 0;
CCConfig.Hit.Constraint             = 0;
CCConfig.FoV.Constraint             = 1;
% Scaling configuration
CCConfig.Missdistance.Scaling       = 5e-03;
CCConfig.Time.Scaling               = 7.5e-01;
CCConfig.ObserverCov.Scaling        = 5e-05;
CCConfig.ObserverRMSE.Scaling       = 0e-04;
CCConfig.TargetViolation.Scaling    = 1e-04;
CCConfig.Hit.Scaling                = 1e+02;

end
