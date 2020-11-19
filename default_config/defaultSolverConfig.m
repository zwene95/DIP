function [ Solver ] = defaultSolverConfig()
% Default solver config

Solver = struct();

Solver.GridSize         = 200;                           % 200
Solver.MaxIter          = 500;                           % 500
Solver.Parallel         = 0;
Solver.GPU              = 0;
Solver.CostWeightTime  = 5e-1;                         % 1e-0
Solver.CostWeightCov   = 1e-4;                         % 1e-4/
Solver.CostWeightRMSE  = 1e-0;                         % 1e-6
Solver.CostWeightMiss  = 5e-2;                         % 5e-2/1e-2

end
