function plotObs_vIOO(setup, problem, results)

    c = setup.postOptions.c;
    
    vIOO_true   = results.invader.states.vel;
    vIOO_obs    = results.observer.vIOO_obs;

    % Real time
    time = problem.RealTime;
    
    % Create figure
    figname = 'Observed Invader Velocity';
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);    
    
    % POS X
    ax1 = subplot(3,1,1); hold on; grid on;
        true   = plot(time,vIOO_true(1,:), 'g', 'LineWidth',2);
        obs    = plot(time,vIOO_obs(1,:), '.r', 'LineWidth',2);
        ylabel('u [m/s]','FontSize',c.FS_axes, 'Interpreter',c.interpreter);
        set(gca,'XMinorTick','on');
        set(gca,'YMinorTick','on');            
        
    % POS Y
    ax2 = subplot(3,1,2); hold on; grid on;
        plot(time,vIOO_true(2,:), 'g', 'LineWidth',2);
        plot(time,vIOO_obs(2,:), '.r', 'LineWidth',2);
        ylabel('v [m/s]','FontSize',c.FS_axes, 'Interpreter',c.interpreter);
        set(gca,'XMinorTick','on');
        set(gca,'YMinorTick','on');   
    
    % POS Z
    ax3 = subplot(3,1,3); hold on; grid on;
        plot(time,vIOO_true(3,:), 'g', 'LineWidth',2);
        plot(time,vIOO_obs(3,:), '.r', 'LineWidth',2);
        ylabel('w [m/s]','FontSize',c.FS_axes, 'Interpreter',c.interpreter);
        set(gca,'XMinorTick','on');
        set(gca,'YMinorTick','on');    
        
    sgtitle([figname],'FontWeight','bold','FontSize',c.FS_title, 'Interpreter',c.interpreter);
    linkaxes([ax1,ax2,ax3],'x');
    legend([true(1) obs(1)], {'True invader velocity','Observed invader velocity',});
    
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