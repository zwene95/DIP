function Plot_Defender(setup, problem, c)

    
    vBOO =  [   problem.StateValues(find(ismember(problem.StateNames,'u'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'v'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'w'),1),:)
            ];
        
    aBOO =  [   problem.StateDotValues(find(ismember(problem.StateDotNames,'u_dot'),1),:)
            problem.StateDotValues(find(ismember(problem.StateDotNames,'v_dot'),1),:)
            - problem.StateDotValues(find(ismember(problem.StateDotNames,'z_dot'),1),:)
        ];
    
    V_abs = vecnorm(vBOO);
    a_abs = vecnorm(aBOO);
    
    figname = 'Absolute Defender Velocity';

    % Plot Interceptor
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    plot(problem.RealTime, V_abs);
    xlabel('Time [s]');
    ylabel('Velocity [m/s]','Interpreter', 'none');
    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title, 'Interpreter', 'none');

    if setup.postOptions.Save
        if setup.postOptions.Jpg
            path = [setup.postOptions.Path, '\JPG'];
            mkdir(path);
            saveas(gcf,fullfile(path,figname),'jpg');
        end
        if setup.postOptions.Fig
            path = [setup.postOptions.Path, '\FIG'];
            mkdir(path);
            savefig(fullfile(path,figname));
        end
    end   

end