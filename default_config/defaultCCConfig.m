function [ CCConfig ] = defaultCCConfig()
% Default Cost and Constraint Configuration

CCConfig = struct();

% Cost configuration
CCConfig.Cost.Missdistance          = 1;
CCConfig.Cost.Time                  = 0;
CCConfig.Cost.TargetViolation       = 1;
CCConfig.Cost.LosRate               = 0;
% Constraint configuration
CCConfig.Constraint.Thrust          = 0;
CCConfig.Constraint.Hit             = 0;
CCConfig.Constraint.FoV             = 1;
% Scaling configuration
CCConfig.Scaling.Missdistance       = 5e-03;
CCConfig.Scaling.Time               = 7.5e-01;
CCConfig.Scaling.ObserverCov        = 5e-05;
CCConfig.Scaling.ObserverRMSE       = 0e-04;
CCConfig.Scaling.TargetViolation    = 1e-04;
CCConfig.Scaling.Hit                = 1e+02;
CCConfig.Scaling.LosRate            = 1e+00;

end
