function Plot_Intercept_LOS(setup, problem, c)

    N = 10;                                                                 % number of LOS to be displayed
    step = setup.solver.gridSize / N;                                              % stepsize for LOS plot

    pDOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
            ];
    pIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
            ];   
        
    % LOS coordinates
    X = [pDOO(1,1:step:end);pIOO(1,1:step:end)];
    Y = [pDOO(2,1:step:end);pIOO(2,1:step:end)];
    Z = [pDOO(3,1:step:end);pIOO(3,1:step:end)];   
    

    figname = 'Intercept + LOS';

    % Plot invader
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA,'Renderer','opengl');
    hold on
    plot3(pIOO(1,1),pIOO(2,1),pIOO(3,1),'rX');                              % Initial invader position        
    if strcmp(setup.optimize,'inv')
        p1 = quiver3(pIOO(1,:),pIOO(2,:),pIOO(3,:),T_x,T_y,T_z,'r','Linewidth',1.5);          % 3D invader trajectory with attitude
    else
        p1 = plot3(pIOO(1,:),pIOO(2,:),pIOO(3,:),'r','Linewidth',1.5);      % 3D invader trajectory
    end

    % Plot defender
    if ~isempty(pDOO)
        plot3(pDOO(1,1),pDOO(2,1),pDOO(3,1),'gX')                           % Initial defender position
        p2 = plot3(pDOO(1,:),pDOO(2,:),pDOO(3,:),'g','Linewidth',1.5);      % 3D defender trajectory
    end
    
    % Plot LOS lines
    p3 = plot3(X,Y,Z,'-k','LineWidth',0.1);
    
    % Plot target
    pTOO = setup.targetOptions.pTOO;
    p4 = plot3(pTOO(1), pTOO(2), -pTOO(3),'kX');

    hold off

    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    legend([p1 p2, p3(1), p4],'Invader', 'Interceptor', 'LOS', 'Target')

    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);
    axis image;
    view(45,45);


    if setup.postOptions.Save
        if setup.postOptions.Jpg
            path = [setup.postOptions.Path, '\JPG'];
            mkdir(path);
            saveas(gcf,fullfile(path,figname),'jpg');
        end
        if setup.postOptions.Fig
            path = [setup.postOptions.Path, '\FIG'];
            mkdir(path);
            savefig(fullfile(path,figname));
        end
    end

end