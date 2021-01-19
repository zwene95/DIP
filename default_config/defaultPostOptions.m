function [ PostOptions ] = defaultPostOptions(varargin)

    % Create inputs parser object
    p = inputParser;
    addOptional(p, 'ScenarioName', 'DefaultScenario' , @ischar);
    parse(p,varargin{:});
    
    PostOptions = struct();
    PostOptions.ScenarioName        = p.Results.ScenarioName;
    PostOptions.Save                = true;
    PostOptions.Path                = strcat(pwd,'\results\',p.Results.ScenarioName,'\');
    PostOptions.Jpg                 = true;
    PostOptions.Fig                 = true;    
       
    % Konstanten fuer die Plots
    PostOptions.c.m2km              = 1/1000;
    PostOptions.c.rad2deg           = 180/pi;
    PostOptions.c.Title             = {'Interpreter','latex','FontSize',18,'FontWeight','Bold'};
    PostOptions.c.Subtitle          = {'Interpreter','latex','FontSize',16,'FontWeight','Bold'};
    PostOptions.c.Legend            = {'Interpreter','latex','FontSize',16};
	PostOptions.c.Axes              = {'TickLabelInterpreter','latex','FontSize',14};
    PostOptions.c.Label             = {'Interpreter','latex','FontSize',14};
    PostOptions.c.Tick              = {'Interpreter','latex','FontSize',10};
    PostOptions.c.Interpreter       = 'latex';
    PostOptions.c.Linewidth         = 2.5;
    PostOptions.c.Pos_Groesse_SVGA  = [0, 0, 800, 600];
    PostOptions.c.Pos_Groesse_WXGA  = [0, 0, 1280, 800];
    PostOptions.c.Pos_Groesse_HD    = [0, 0, 1920, 1080];
    PostOptions.c.Pos_Groesse_VGA   = [0, 0, 640, 480];
    PostOptions.c.Pos_Groesse_A4    = [0, 0, 29.7, 21];                                 % [cm] Groesse eines A4 Blattes, quer

end
