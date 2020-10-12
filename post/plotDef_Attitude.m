function plotDef_Attitude(setup, problem, c)

    % Real time
    t = problem.RealTime;
    
    % Get defender euler angles
    switch setup.modelOptions.defender.Attitude
        case 'Euler'
            phi =   problem.StateValues(find(ismember(problem.StateNames,'phi'),1),:) * c.rad2deg;
            theta = problem.StateValues(find(ismember(problem.StateNames,'theta'),1),:) * c.rad2deg;
            psi =   problem.StateValues(find(ismember(problem.StateNames,'psi'),1),:) * c.rad2deg;
        otherwise
            error('Defender attitude options not yet implemented');
    end
        
    figname = 'Defender Attitude';

    % LOS
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);    
    % Plot phi
    ax1 = subplot(3,1,1);
    plot(t,phi,'LineWidth',2);
    grid on;
    ylabel('\Phi [°]','FontSize',c.FS_axes);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);

    % Plot theta
    ax2 = subplot(3,1,2);
    plot(t,theta,'LineWidth',2);
    grid on;
    ylabel('\Theta [°]','FontSize',c.FS_axes);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    
    % Plot psi rate
    ax3 = subplot(3,1,3);
    plot(t,psi,'LineWidth',2);
    grid on;
    ylabel('\Psi [°]','FontSize',c.FS_axes);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    
    linkaxes([ax1,ax2,ax3],'x');
    
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