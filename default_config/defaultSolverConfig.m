function [ Solver ] = defaultSolverConfig()
% Default Solver Configuration

Solver = struct();

Solver.GridSize         = 200;                           % 200
Solver.maxIter          = 500;                           % 500
Solver.Parallel         = 0;
Solver.GPU              = 0;

end
