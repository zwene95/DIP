function plotObs_cov(setup, problem, results)

    c = setup.postOptions.c;
    
    P_trace = results.observer.covariance.combined;
    P_trace_pos = results.observer.covariance.pos;
    P_trace_vel = results.observer.covariance.vel;    
    
    pIOO_true = [
        results.invader.states.pos - results.defender.states.pos
        results.invader.states.vel - results.defender.states.vel
    ];
    pIOO_obs = [
        results.observer.pIOO_obs
        results.observer.vIOO_obs
    ];
    
    err_vec = pIOO_true - pIOO_obs;
    
%     NEES = err_vec

    % Real time
    time = problem.RealTime;
    
    % Create figure
    figname = 'Observability Indices';
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA); 
    hold on;
    ax1 = subplot(3,3,1);
        plot(time(2:end),vecnorm(err_vec(1:6,2:end)),'g','LineWidth',2);grid on;
        title('RMSE Combined','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );       
        ylabel('[-]','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax2 = subplot(3,3,2);
        plot(time(2:end),vecnorm(err_vec(1:3,2:end)),'g','LineWidth',2);grid on;
        title('RMSE Position','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );       
        ylabel('[m]','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax3 = subplot(3,3,3);
        plot(time(2:end),vecnorm(err_vec(4:6,2:end)),'g','LineWidth',2);grid on;
        title('RMSE Velocity','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );       
        ylabel('[m/s]','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax4 = subplot(3,3,4);
        plot(time(2:end),P_trace(2:end),'g','LineWidth',2);grid on;
        title('Covariance Trace Combined','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax5 = subplot(3,3,5);
        plot(time(2:end),P_trace_pos(2:end),'g','LineWidth',2);grid on;
        title('Covariance Trace Position','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax6 = subplot(3,3,6);
        plot(time(2:end),P_trace_vel(2:end),'g','LineWidth',2);grid on;
        title('Covariance Trace Velocity','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax7 = subplot(3,3,7);
%         plot(time(2:end),NEES(2:end),'g','LineWidth',2);grid on;
        title('NEES Combined','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax8 = subplot(3,3,8);
%         plot(time(2:end),NEES_pos(2:end),'g','LineWidth',2);grid on;
        title('NEES Position','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );
    ax9 = subplot(3,3,9);
%         plot(time(2:end),NEES_vel(2:end),'g','LineWidth',2);grid on;
        title('NEES Velocity','FontSize',c.FS_subtitle, 'Interpreter', c.interpreter );       
        
    sgtitle(figname,'FontSize',c.FS_title, 'Interpreter', c.interpreter );
    linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9],'x');   
    
    
    
    
    
    
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