function [] = DIP_post(Setup, Problem)
% Post process DefenderInvaderProblem

c = Setup.PostOptions.c;

close all;

% Extract results from solved problem
Results = logResults(Setup, Problem);

% Save Setup, Problem and Results to base workspace
assignin('base','Problem',Problem);
assignin('base','Setup',Setup);
assignin('base','Results',Results);

if Setup.PostOptions.Save
    % Create path for saving figures
    mkdir(Setup.PostOptions.Path);
    if Setup.PostOptions.Jpg
        Setup.PostOptions.PathJpg = [Setup.PostOptions.Path, 'JPG'];
        mkdir(Setup.PostOptions.PathJpg);
    end
    if Setup.PostOptions.Fig
        Setup.PostOptions.PathFig = [Setup.PostOptions.Path, 'FIG'];
        mkdir(Setup.PostOptions.PathFig);
    end
    
    % Save Setup, Problem and Results to mat file
    save([Setup.PostOptions.Path,'Setup.mat'], 'Setup');
    save([Setup.PostOptions.Path,'Problem.mat'], 'Problem');
    save([Setup.PostOptions.Path,'Results.mat'], 'Results');
    disp(['Results saved in ',Setup.PostOptions.Path]);
end

% Intercept trajectory
Plot_Intercept_attitude(Setup, Results);
%     Plot_Intercept_velocity(setup, problem, c);
%     Plot_Intercept_acceleration(setup, problem, c);
%     Plot_Intercept_animated(setup, problem, c);

% Intercept trajectories with LOS line
%     Plot_Intercept_LOS(setup, problem, c);

% Plot defender parameters
plotDef_vBOOabs(Setup, Results);                                            % Plot absolute defender aelocity
%     plotDef_Thrust(setup, problem, c);                                    % Plot defender thrust

%%%     plotDef_aBOOabs(setup, problem, c);                                     % Plot absolute defender acceleration

% Plot invader parameters
%     Plot_Invader(setup, problem, c);

% Plot missdistance over time
Plot_Missdistance(Setup, Results);

% Plot defender attitude and seeker angles
if Setup.ModelOptions.Defender.SixDoF
    Plot_Seeker(Setup, Results);
    %         plotDef_Attitude(setup, problem, c);
end

% Plot LOS angles
% 	Plot_LOS(setup, problem, c);

% Obervability Cost Function
PostObservabilityAnalysis(Setup, Results);


% Open the plot gui
%     try
%         falcon.gui.plot.show(problem,'AskSaveOnClose',false);
%     catch
%         fprintf('Please check your graphics driver and Java setting for the FALCON.m plot GUI to work.\n');
%     end

% EoF
end
