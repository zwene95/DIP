function [ Solver ] = defaultSolverConfig()
% Default Solver Configuration

Solver = struct();

Solver.GridSize         = 200;                           % 200
Solver.MaxIter          = 500;                           % 500
Solver.BackwarEuler     = 0;
Solver.Parallel         = 1;

end
