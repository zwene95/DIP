function obj = observabilityCostObject(problem)
%OBSERVABILITYCOSTOBJECT Summary of this function goes here
%   Creates an object for the observabilityCostFcn(obj,varargin)
    
    obj.NPhases                     = length(problem.Phases);
    obj.NTimeStepsPerPhase          = length(problem.RealTime);
    obj.NOutputs                    = length(problem.OutputNames);
    obj.OutputNames                 = problem.OutputNames;
    obj.NStates                     = length(problem.StateNames);
    obj.StateNames                  = problem.StateNames;
    obj.NControls                   = length(problem.ControlNames);
    obj.ControlNames                = problem.ControlNames;

end

