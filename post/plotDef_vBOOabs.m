function plotDef_vBOOabs(setup, problem, c)

    
    vBOO =  [   problem.StateValues(find(ismember(problem.StateNames,'u'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'v'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'w'),1),:)
            ];
    
    vBOO_abs = vecnorm(vBOO);
    
    figname = 'Absolute Defender Velocity';

    % Plot Interceptor
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    plot(problem.RealTime, vBOO_abs, 'LineWidth', 2);    
    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title, 'Interpreter', 'none');
    xlabel('Time [s]','FontSize',c.FS_axes,'Interpreter', 'latex');
    ylabel('Velocity [m/s]','FontSize',c.FS_axes,'Interpreter', 'latex');

    if setup.postOptions.Save
        if setup.postOptions.Jpg
            saveas(gcf,fullfile(setup.postOptions.PathJpg,figname),'jpg');
        end
        if setup.postOptions.Fig
            savefig(fullfile(setup.postOptions.PathFig,figname));
        end
    end   

end