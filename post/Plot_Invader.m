function Plot_Invader(setup, problem, c)

    switch setup.modelOptions.invader.Type
        case 'Quad1'
            vIOO =  [   problem.StateDotValues(find(ismember(problem.StateDotNames,'x_inv_dot'),1),:)
                        problem.StateDotValues(find(ismember(problem.StateDotNames,'y_inv_dot'),1),:)
                        - problem.StateDotValues(find(ismember(problem.StateDotNames,'z_inv_dot'),1),:)
                    ];            
            vI_abs = sqrt( vIOO(1,:).^2 + vIOO(2,:).^2 + vIOO(3,:).^2);
        case 'Quad2'
            vIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'u_inv'),1),:)
                        problem.StateValues(find(ismember(problem.StateNames,'v_inv'),1),:)
                        - problem.StateValues(find(ismember(problem.StateNames,'w_inv'),1),:)
                    ];
            vI_abs = sqrt( vIOO(1,:).^2 + vIOO(2,:).^2 + vIOO(3,:).^2);
        otherwise
            error('Invader option not implemented!');
    end
    
    figname = 'Absolute Invader Velocity';

    % Plot Invader
    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    plot(problem.RealTime, vI_abs);
    xlabel('Time [s]');
    ylabel('Velocity [m/s]','Interpreter', 'none');
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