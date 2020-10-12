function plotObs_pIOO(setup, problem, results)

    c = setup.postOptions.c;
    
    pIOO_true   = results.invader.states.pos;
    pIOO_obs    = results.observer.pIOO_obs;

    % Real time
    time = problem.RealTime;
    
    % Create figure
    figname = 'Observed invader position';
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);    
    
    % POS X
    ax1 = subplot(3,1,1); hold on; grid on;
        true   = plot(time,pIOO_true(1,:), 'g', 'LineWidth',2);
        obs    = plot(time,pIOO_obs(1,:), '.r', 'LineWidth',2);
    ylabel('X [m]','FontSize',c.FS_axes);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);
        
    % POS Y
    ax2 = subplot(3,1,2); hold on; grid on;
        plot(time,pIOO_true(2,:), 'g', 'LineWidth',2);
        plot(time,pIOO_obs(2,:), '.r', 'LineWidth',2);
    ylabel('Y [m]','FontSize',c.FS_axes);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    
    % POS Z
    ax3 = subplot(3,1,3); hold on; grid on;
        plot(time,pIOO_true(3,:), 'g', 'LineWidth',2);
        plot(time,pIOO_obs(3,:), '.r', 'LineWidth',2);
    ylabel('Z [m]','FontSize',c.FS_axes);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    
    linkaxes([ax1,ax2,ax3],'x');
    legend([true(1) obs(1)], {'True invader position','Observed invader position',});
    
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