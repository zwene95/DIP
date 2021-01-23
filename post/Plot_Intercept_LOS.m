function Plot_Intercept_LOS(Setup, Results)
    
    c = Setup.PostOptions.c;
    N = 10;                                                                 % number of LOS to be displayed
    step = Setup.Solver.GridSize / N;                                       % stepsize for LOS plot

    pDOO = Results.Defender.States.Pos;
    pIOO = Results.Invader.States.Pos;
        
    % LOS coordinates
    X = [pDOO(1,1:step:end);pIOO(1,1:step:end)];
    Y = [pDOO(2,1:step:end);pIOO(2,1:step:end)];
    Z = [-pDOO(3,1:step:end);-pIOO(3,1:step:end)];       

    Figname = 'DIP with LOS Information';

    % Plot invader
    figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA,'Renderer','opengl');
    hold on
    plot3(pIOO(1,1),pIOO(2,1),-pIOO(3,1),'rO','LineWidth',2);               % Initial invader position            
    plot3(pIOO(1,end),pIOO(2,end),-pIOO(3,end),'rX','LineWidth',2);         % Final invader position            
    p1 = plot3(pIOO(1,:),pIOO(2,:),-pIOO(3,:),'r','Linewidth',1.5);         % 3D invader trajectory    

    % Plot defender
    if ~isempty(pDOO)
        plot3(pDOO(1,1),pDOO(2,1),-pDOO(3,1),'gO','LineWidth',2)            % Initial defender position
        plot3(pDOO(1,end),pDOO(2,end),-pDOO(3,end),'gX','LineWidth',2)      % Final defender position
        p2 = plot3(pDOO(1,:),pDOO(2,:),-pDOO(3,:),'g','Linewidth',1.5);     % 3D defender trajectory
    end
    
    % Plot LOS lines
    p3 = plot3(X,Y,Z,'-k','LineWidth',0.5);
    
    hold off

    xlabel('X',c.Label{:});
    ylabel('Y',c.Label{:});
    zlabel('Z',c.Label{:});

    legend([p1 p2, p3(1)],'Invader', 'Defender', 'LOS',c.Legend{:})

    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,c.Axes{:});
    title([Figname,newline],c.Title{:});
    axis image;
    view(-70,9);


    if Setup.PostOptions.Save
        if Setup.PostOptions.Jpg
            path = [Setup.PostOptions.Path, '\JPG'];
            mkdir(path);
            saveas(gcf,fullfile(path,Figname),'svg');
        end
        if Setup.PostOptions.Fig
            path = [Setup.PostOptions.Path, '\FIG'];
            mkdir(path);
            savefig(fullfile(path,Figname));
        end
    end

end