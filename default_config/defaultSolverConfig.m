function [ Solver ] = defaultSolverConfig()
% Default solver config

Solver = struct();

Solver.gridSize         = 200;                           % 200
Solver.maxIter          = 500;                           % 500
Solver.Parallel         = 0;
Solver.GPU              = 0;
Solver.CostScalingTime  = 5e-1;                         % 1e-0
Solver.CostScalingCov   = 1e-4;                         % 1e-4/
Solver.CostScalingRMSE  = 1e-0;                         % 1e-6
Solver.CostScalingMiss  = 5e-2;                         % 5e-2/1e-2

end
