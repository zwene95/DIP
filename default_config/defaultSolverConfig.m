function [ Solver ] = defaultSolverConfig()
% Default solver config

Solver = struct();

Solver.gridSize                   = 200;                           % 200
Solver.maxIter                    = 500;                           % 500
Solver.CostScaling                = 1e-0;                         % 1e-0
Solver.TimeCostScaling            = 1e-0;                         % 1e-0
Solver.ObsCostScaling             = 1e-5;                         % 1e-0
Solver.GPU                        = 0;

end
