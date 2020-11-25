function [ Solver ] = defaultSolverConfig()
% Default Solver Configuration

Solver = struct();

Solver.GridSize         = 200;                           % 200
Solver.maxIter          = 500;                           % 500
Solver.BackwarEuler     = 0;
Solver.Parallel         = 1;
Solver.GPU              = 0;

end
