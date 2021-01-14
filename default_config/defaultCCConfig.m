function [ CCConfig ] = defaultCCConfig()
% Default Cost and Constraint Configuration

CCConfig = struct();

% Costs
CCConfig.Cost.Missdistance          = 1;
CCConfig.Cost.Time                  = 0;
CCConfig.Cost.TargetViolation       = 1;
CCConfig.Cost.LosRate               = 0;
% Constraints
CCConfig.Constraint.Thrust          = 0;
CCConfig.Constraint.Hit             = 0;
CCConfig.Constraint.FoV             = 1;
% Scalings
CCConfig.Scaling.Missdistance       = 5e-03;
CCConfig.Scaling.Time               = 7.5e-01;
CCConfig.Scaling.ObserverCov        = 5e-05;
CCConfig.Scaling.ObserverRMSE       = 0e-04;
CCConfig.Scaling.TargetViolation    = 1e-04;
CCConfig.Scaling.Hit                = 1e+02;
CCConfig.Scaling.LosRate            = 1e+00;
% Parameters
CCConfig.Parameter.LosRateThresh    = 5*pi/180;                             % Line-of-sight (LOS) rate threshold for LosRateCost

end
