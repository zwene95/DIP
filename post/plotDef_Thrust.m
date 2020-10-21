function plotDef_Thrust(setup, problem, c)
        
    
    if setup.modelOptions.defender.SixDoF
        if setup.modelOptions.defender.MotorLag
            c1 = problem.StateValues(find(ismember(problem.StateNames,'w1'),1),:);
            c2 = problem.StateValues(find(ismember(problem.StateNames,'w2'),1),:);
            c3 = problem.StateValues(find(ismember(problem.StateNames,'w3'),1),:);
            c4 = problem.StateValues(find(ismember(problem.StateNames,'w4'),1),:);
        else
            c1 = problem.ControlValues(find(ismember(problem.ControlNames,'w1'),1),:);
            c2 = problem.ControlValues(find(ismember(problem.ControlNames,'w2'),1),:);
            c3 = problem.ControlValues(find(ismember(problem.ControlNames,'w3'),1),:);
            c4 = problem.ControlValues(find(ismember(problem.ControlNames,'w4'),1),:);
        end
    T = ( c1.^2 + c2.^2 + c3.^2 + c4.^2 ) * setup.modelOptions.defender.m * 9.80665 * setup.defenderConfig.T2W_max / 4 / setup.modelOptions.defender.RPM_max^2;
    % 3DoF
    else
        controls =  [   
            problem.ControlValues(find(ismember(problem.ControlNames,'T_x'),1),:)
            problem.ControlValues(find(ismember(problem.ControlNames,'T_y'),1),:)
            problem.ControlValues(find(ismember(problem.ControlNames,'T_z'),1),:)
       ]; 
        T = vecnorm(controls) * setup.defenderOptions.m * 9.80665;
    end

    figname = 'Defender Thrust';

    % Plot Interceptor
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    plot(problem.RealTime, T, 'LineWidth', 2);    
    xlabel('Time [s]');
    ylabel('Thrust [N]');
    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
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