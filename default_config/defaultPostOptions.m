function [ postOptions ] = defaultPostOptions(varargin)

    % Create inputs parser object
    p = inputParser;
    addOptional(p, 'ScenarioName', 'DefaultScenario' , @ischar);
    parse(p,varargin{:});
    
    postOptions = struct();
    postOptions.ScenarioName =          p.Results.ScenarioName;
    postOptions.Save =                  true;
    postOptions.Path =                  strcat(pwd,'\Results\',p.Results.ScenarioName,'\');
    postOptions.Jpg =                   true;
    postOptions.Fig =                   true;
    
    
    
    % Konstanten fuer die Plots
    postOptions.c.m2km            =   1/1000;
    postOptions.c.rad2deg         =   180/pi;
    postOptions.c.FS_axes         =   16;
    postOptions.c.FS_plot         =   18;
    postOptions.c.FS_subtitle     =   14;  
    postOptions.c.FS_title        =   18;
    postOptions.c.FS_linienbreite =   2.5;
    postOptions.c.Pos_Groesse_SVGA=   [0, 0, 800, 600];
    postOptions.c.Pos_Groesse_WXGA=   [0, 0, 1280, 800];
    postOptions.c.Pos_Groesse_HD  =   [0, 0, 1920, 1080];
    postOptions.c.Pos_Groesse_VGA =   [0, 0, 640, 480];
    postOptions.c.Pos_Groesse_A4  =   [0, 0, 29.7, 21];                                 % [cm] Groesse eines A4 Blattes, quer

end
