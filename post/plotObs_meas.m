function plotObs_meas(setup, problem, results)

    c = setup.postOptions.c;
    
    z_true  = results.observer.meas.true * c.rad2deg;
    z_est   = results.observer.meas.est * c.rad2deg;

    % Real time
    time = problem.RealTime;
    
    % Create figure
    figname = 'Observed invader position';
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA); 
    
    
    % Plot true and estimated LOS angles    
    ax1 = subplot(2,1,1); hold on; grid on;
        ptrue   =   plot(time,z_true(1,:),'-g','LineWidth',2);        
        pest    =   plot(time,z_est(1,:),'--r','LineWidth',2);    
        title('Azimuth', 'FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
        xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', c.interpreter);
        ylabel('$$\beta$$ in [$$^\circ$$]','FontSize',c.FS_axes, 'Interpreter', c.interpreter );
    ax2 = subplot(2,1,2); hold on; grid on;   
        plot(time,z_true(2,:),'-g','LineWidth',2); grid on;        
        plot(time,z_est(2,:),'--r','LineWidth',2);                 
        title('Elevation','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
        xlabel('Time [s]','FontSize',c.FS_axes, 'Interpreter', c.interpreter );
        ylabel('$$\epsilon$$ in [$$^\circ$$]','FontSize',c.FS_axes, 'Interpreter', c.interpreter );
    legend([ptrue(1) pest(1)], {'True', 'Estimation'});        
    sgtitle('True and Estimated Measurements','FontSize',c.FS_title, 'Interpreter', c.interpreter );
    linkaxes([ax1,ax2],'x');

    
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