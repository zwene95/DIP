function [Setup, Results] = StationairyScenario(SetupSrc, varargin)
%StationairyScenario Stationary scenario to demonstrate observer behavior

if nargin == 1
    % Rerun setup file
    Setup = SetupSrc;
else
    Setup.DefenderConfig    = defaultDefenderConfig;
    Setup.InvaderConfig     = defaultInvaderConfig;
    Setup.TargetConfig      = defaultTargetConfig;
    Setup.PostOptions       = defaultPostOptions('StationairyScenario');
    Setup.ObserverConfig    = defaultObserverConfig;
end

nDat = 500;
dt = 10e-03;
Time = cumsum(ones(1,nDat)*dt);
pDOO = Setup.DefenderConfig.pDOO_0;
pIOO = Setup.InvaderConfig.pIOO_0;
Setup.Scenario.pTOO = Setup.TargetConfig.pTOO;

Setup.Solver.GridSize = nDat;

Results.Time = Time;
Results.Defender.States.Pos = repmat(pDOO,[1,nDat]);
Results.Defender.States.Vel = zeros(3,nDat);
Results.Defender.States.Acc = zeros(3,nDat);
Results.Invader.States.Pos  = repmat(pIOO,[1,nDat]);
Results.Invader.States.Vel  = zeros(3,nDat);

%% Post processing
% Create path so save results
if Setup.PostOptions.Save
    % Save Jpg
    mkdir(Setup.PostOptions.Path);
    if Setup.PostOptions.Jpg
        Setup.PostOptions.PathJpg = [Setup.PostOptions.Path, 'JPG'];
        mkdir(Setup.PostOptions.PathJpg);
    end
    % Save Fig
    if Setup.PostOptions.Fig
        Setup.PostOptions.PathFig = [Setup.PostOptions.Path, 'FIG'];
        mkdir(Setup.PostOptions.PathFig);
    end
    
    % Save Setup and Results to mat file
    save([Setup.PostOptions.Path,'Setup.mat'], 'Setup');
    save([Setup.PostOptions.Path,'Results.mat'], 'Results');
    disp(['Results saved in ',Setup.PostOptions.Path]);
end
% Post process results
PostObservabilityAnalysis(Setup,Results);
% plotDef_vBOOabs_aBOOabs(Setup, Results);

end
