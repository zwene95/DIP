%% Debug observabilityCostFcn
clc;clear myObj;
% load('D:\GoogleDrive\UNI\Master\Masterarbeit\DIP_git\Results\Test3_obs\problem.mat');
myObj = observabilityCostObject;
myObj.Problem = problem;

% myObj.observabilityCostFcn(         ...
%     problem.OutputValues,           ... 
%     problem.StateValues,            ... 
%     problem.ControlValues,          ... 
%     [problem.Parameters(1).Value        
%     problem.Parameters(2).Value]    ...
% )
