function Plot_Seeker(setup, problem, c)


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
    elevation = atan2(-pDIO(3,:),sqrt(pDIO(1,:).^2 + pDIO(2,:).^2)) * c.rad2deg;
    t = problem.RealTime;
    
    % Get defender euler angles
    switch setup.modelOptions.defender.Attitude
        case 'Euler'
%             phi =   problem.StateValues(find(ismember(problem.StateNames,'phi'),1),:) * c.rad2deg;
            theta = problem.StateValues(find(ismember(problem.StateNames,'theta'),1),:) * c.rad2deg;
            psi =   problem.StateValues(find(ismember(problem.StateNames,'psi'),1),:) * c.rad2deg;
        otherwise
            error('Defender attitude options not yet implemented');
    end
        
    figname = 'Seeker Body Angles';

    
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    
    % Plot elevation
    ax1 = subplot(2,1,1);
    hold on; grid on;
    p1 = plot(t,elevation - theta,'LineWidth',2);
    p2 = plot(t, ones(length(t)) * setup.defenderConfig.FoV(1)/2, '--r', 'LineWidth', 2);
    plot(t, -ones(length(t)) * setup.defenderConfig.FoV(1)/2, '--r', 'LineWidth', 2);
    title('Elevation','FontSize',c.FS_subtitle);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'Fontsize',c.FS_plot);
    xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    ylabel('$$\epsilon_B$$ in [$$^\circ$$]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    legend([p1 p2(1)],'$$\epsilon_B$$', 'FOV','Interpreter','latex');
    
    % Plot azimuth
    ax2 = subplot(2,1,2);
    hold on; grid on;
    p1 = plot(t,azimuth - psi,'LineWidth',2);
    p2 = plot(t, ones(length(t)) * setup.defenderConfig.FoV(2)/2, '--r', 'LineWidth', 2);
    plot(t, -ones(length(t)) * setup.defenderConfig.FoV(2)/2, '--r', 'LineWidth', 2);    
    title('Azimuth','FontSize',c.FS_subtitle);   
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'Fontsize',c.FS_plot);
    legend([p1 p2(1)],'$$\beta_B$$', 'FOV','Interpreter','latex');
    xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    ylabel('$$\beta_B$$ in [$$^\circ$$]','FontSize',c.FS_axes, 'Interpreter', 'latex');
    
    linkaxes([ax1,ax2],'x');
    
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