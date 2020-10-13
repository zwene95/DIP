function [] = DIP_post(setup, problem)
% Post process DefenderInvaderProblem

    
    c = setup.postOptions.c;
    
    close all;    
    
    % Extract results from solved problem
    results = logResults(setup, problem);
    
    if setup.postOptions.Save
        mkdir(setup.postOptions.Path);    
        if setup.postOptions.Jpg
            setup.postOptions.PathJpg = [setup.postOptions.Path, 'JPG'];
            mkdir(setup.postOptions.PathJpg);
        end
        if setup.postOptions.Fig
            setup.postOptions.PathFig = [setup.postOptions.Path, 'FIG'];
            mkdir(setup.postOptions.PathFig);
        end     
    end
    
    % Add folder with plot configs
    addpath([pwd,'\Post']);
    
    % Save setup and problem to mat file
    if setup.postOptions.Save == true
        % Save initialization setup
        save([setup.postOptions.Path,'setup.mat'], 'setup');
        save([setup.postOptions.Path,'problem.mat'], 'problem');
        save([setup.postOptions.Path,'results.mat'], 'results');
        disp(['Results saved in ',setup.postOptions.Path]);        
    end  
    
    % Save setup and problem to base workspace
    assignin('base','problem',problem);
    assignin('base','setup',setup);
    assignin('base','results',results);
    
    % Intercept trajectory
    Plot_Intercept_attitude(setup, problem, c);
    Plot_Intercept_velocity(setup, problem, c);
    Plot_Intercept_acceleration(setup, problem, c);
    Plot_Intercept_animated(setup, problem, c);
    
    % Intercept trajectories with LOS line
%     Plot_Intercept_LOS(setup, problem, c);
        
    % Plot defender parameters
%     plotDef_vBOOabs(setup, problem, c);                                     % Plot absolute defender aelocity
    plotDef_Thrust(setup, problem, c);                                      % Plot defender thrust
    
    %%%     plotDef_aBOOabs(setup, problem, c);                                     % Plot absolute defender acceleration    
        
    % Plot invader parameters
%     Plot_Invader(setup, problem, c);
    
    % Plot missdistance over time
    Plot_Missdistance(setup, problem, c);      
    
    % Plot defender attitude and seeker angles
    if setup.modelOptions.defender.SixDoF
%         Plot_LOS_Seeker(setup, problem, c);
%         plotDef_Attitude(setup, problem, c);
    end
    
    % Plot LOS angles
	Plot_LOS(setup, problem, c);    
    
       
    if setup.modelOptions.observer
        plotObs_pIOO(setup, problem, results);
    end
    
    % Open the plot gui
    try
        falcon.gui.plot.show(problem,'AskSaveOnClose',false);
    catch
        fprintf('Please check your graphics driver and Java setting for the FALCON.m plot GUI to work.\n');
    end
    
% EoF
end
