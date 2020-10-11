function plotDef_aBOOabs(setup, problem, c)
        
    aBOO =  [   problem.StateDotValues(find(ismember(problem.StateDotNames,'u_dot'),1),:)
                problem.StateDotValues(find(ismember(problem.StateDotNames,'v_dot'),1),:)
                - problem.StateDotValues(find(ismember(problem.StateDotNames,'w_dot'),1),:)
            ];
    
    aBOO_abs = vecnorm(aBOO);
    
    figname = 'Absolute Defender Acceleration';

    % Plot Interceptor
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    plot(problem.RealTime, aBOO_abs);
    xlabel('Time [s]');
    ylabel('Acceleration [m/s^2]');
    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title, 'Interpreter', 'none');

    if setup.postOptions.Save
        if setup.postOptions.Jpg
            saveas(gcf,fullfile(setup.postOptions.PathJpg,figname),'jpg');
        end
        if setup.postOptions.Fig
            savefig(fullfile(setup.postOptions.PathFig,figname));
        end
    end   

end