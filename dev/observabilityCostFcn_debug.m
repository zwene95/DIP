%% Debug observabilityCostFcn
clc;clear problem obj;
load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\Test3_obs\problem.mat');
obj = observabilityCostObject(problem);
str = observabilityCostFcn(         ...
    obj,                            ...
    problem.OutputValues,           ...
    problem.StateValues,            ...
    problem.ControlValues,          ...
    [problem.Parameters(1).Value
     problem.Parameters(2).Value]   ...
);
