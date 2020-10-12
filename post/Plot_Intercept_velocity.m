function Plot_Intercept_velocity(setup, problem, c)

    N = 50;                                                                 % number of attitude vectors to be displayed
    step = setup.solver.gridSize / N;                                              % stepsize for attitude plot
    N_LOS = 10;                                                             % number of attitude vectors LOS to be displayed
    step_LOS = setup.solver.gridSize / N_LOS;                                          % stepsize for LOS plot

    % Position
    pDOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
            ];
        
    pIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
            ];
        
    % LOS coordinates    
    X = [pDOO(1,1:step_LOS:end);pIOO(1,1:step_LOS:end)];
    Y = [pDOO(2,1:step_LOS:end);pIOO(2,1:step_LOS:end)];
    Z = [pDOO(3,1:step_LOS:end);pIOO(3,1:step_LOS:end)];   
        
        
        
    % Get velocity    
    switch setup.modelOptions.optimize
        case 'inv'
            T_x = problem.ControlValues(find(ismember(problem.ControlNames,'T_x_inv'),1),1:step:end);
            T_y = problem.ControlValues(find(ismember(problem.ControlNames,'T_y_inv'),1),1:step:end);
            T_z = - problem.ControlValues(find(ismember(problem.ControlNames,'T_z_inv'),1),1:step:end);
        case 'def'            
            T_x = problem.StateValues(find(ismember(problem.StateNames,'u'),1),1:step:end);
            T_y = problem.StateValues(find(ismember(problem.StateNames,'v'),1),1:step:end);
            T_z = - problem.StateValues(find(ismember(problem.StateNames,'w'),1),1:step:end);
    end    

    figname = 'Intercept + Velocity';

    % Plot invader
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    hold on
    plot3(pIOO(1,1),pIOO(2,1),pIOO(3,1),'rX');                                          % Initial invader position        
    if strcmp(setup.modelOptions.optimize,'inv')
        p1 = quiver3(pIOO(1,:),pIOO(2,:),pIOO(3,:),T_x,T_y,T_z,'r','Linewidth',1.5);    % 3D invader trajectory with attitude
    else
        p1 = plot3(pIOO(1,:),pIOO(2,:),pIOO(3,:),'r','Linewidth',1.5);                  % 3D invader trajectory
    end

    % Plot defender
    plot3(pDOO(1,1),pDOO(2,1),pDOO(3,1),'gX')                                           % Initial defender position        
    if strcmp(setup.modelOptions.optimize,'def')
        p2 = plot3(pDOO(1,:),pDOO(2,:),pDOO(3,:),'g','Linewidth',1.5);
        p2a = quiver3(pDOO(1,1:step:end),pDOO(2,1:step:end),pDOO(3,1:step:end),T_x,T_y,T_z,'g','Linewidth',1.5);    % 3D defender trajectory with velocity
    else
        p2 = plot3(pDOO(1,:),pDOO(2,:),pDOO(3,:),'g','Linewidth',1.5);                  % 3D defender trajectory
    end
        
    % Plot target
    pTOO = setup.scenario.pTOO;
    p3 = plot3(pTOO(1), pTOO(2), -pTOO(3),'mX');
    
    % Plot LOS lines
    p4 = plot3(X,Y,Z,'-k','LineWidth',0.1);

    hold off

    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    legend([p1 p2 p2a p3 p4(1)],'Invader', 'Defender', 'Defender Velocity',...
        'Target', 'LOS');

    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);
    axis image;
    view(45,45);

    % Save plots
    if setup.postOptions.Save
        if setup.postOptions.Jpg
            saveas(gcf,fullfile(setup.postOptions.PathJpg,figname),'jpg');
        end
        if setup.postOptions.Fig
            savefig(fullfile(setup.postOptions.PathFig,figname));
        end
    end

end