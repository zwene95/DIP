function Plot_LOS(setup, problem, c)

%     N = 10;                                                                 % number of LOS to be displayed
%     step = setup.gridSize / N;                                              % stepsize for LOS plot

    pDOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
            ];
    pIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
            ];     
        
    pDIO = pIOO - pDOO;
    
    azimuth =   atan2(pDIO(2,:),pDIO(1,:)) * c.rad2deg;
    d_azimuth = gradient(azimuth);
    elevation = atan2(-pDIO(3,:),sqrt(pDIO(1,:).^2 + pDIO(2,:).^2)) * c.rad2deg;
    d_elevation = gradient(elevation);
    t = problem.RealTime;
        
    figname = 'LOS + LOS-Rate';

    % LOS
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    
    % Plot elevation
    ax2 = subplot(2,2,1);
    plot(t,elevation,'LineWidth',2);    
    grid on;
    title('Elevation','FontSize',c.FS_subtitle);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');   
    set(gca,'Fontsize',c.FS_plot);
    xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    ylabel('$$\epsilon$$ in [$$^\circ$$]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    
    % Plot elevation rate
    ax4 = subplot(2,2,2);
    plot(t,d_elevation,'LineWidth',2);    
    grid on;
    title('Elevation Rate','FontSize',c.FS_subtitle);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'Fontsize',c.FS_plot);
    xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    ylabel('$$\dot{\epsilon}$$ in [$$^\circ$$/s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    
    % Plot azimuth
    ax1 = subplot(2,2,3);
    plot(t,azimuth,'LineWidth',2);    
    grid on;
    title('Azimuth','FontSize',c.FS_subtitle);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'Fontsize',c.FS_plot);
    xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    ylabel('$$\beta$$ in [$$^\circ$$]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    
    % Plot azimuth rate
    ax3 = subplot(2,2,4);
    plot(t,d_azimuth,'LineWidth',2);    
    grid on;    
    title('Azimuth Rate','FontSize',c.FS_subtitle);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');  
    set(gca,'Fontsize',c.FS_plot);
    xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    ylabel('$$\dot{\beta}$$ in [$$^\circ$$/s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    
    linkaxes([ax1,ax2,ax3,ax4],'x');
    
    % Plot overall title
    sgtitle(figname,'FontWeight','bold','FontSize',c.FS_title);
    
    % Save plot
    if setup.postOptions.Save
        if setup.postOptions.Jpg
            saveas(gcf,fullfile(setup.postOptions.PathJpg,figname),'jpg');
        end
        if setup.postOptions.Fig
            savefig(fullfile(setup.postOptions.PathFig,figname));
        end
    end

end