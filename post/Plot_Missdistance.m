function Plot_Missdistance(setup, problem, c)

    pDOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
                    problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
                    - problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
                ];
        
    pIOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
                problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
                - problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
            ];

    pVDO = pIOO - pDOO;
    missdistance = vecnorm(pVDO);
    captureDistance = vecnorm(pIOO(1:2,end));

    figname = 'Missdistance';

    figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
    plot(problem.RealTime, missdistance,'LineWidth',2);

    xlabel('Time [s]')
    ylabel('Missdistance [m]')
    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,'FontSize',c.FS_plot);
    title([figname,newline],'FontWeight','bold','FontSize',c.FS_title);
    legend(sprintf('Missdistance = %.3fm',min(missdistance)));
%     legend(sprintf('Missdistance = %.3fm\nCaptureDistance = %.3fm',min(missdistance),captureDistance));
    fprintf('Missdistance = %.3fm \n',min(missdistance));
    fprintf('Capturedistance = %.3fm \n',captureDistance);
    % axis image;

    
    if setup.postOptions.Save
        if setup.postOptions.Jpg
            saveas(gcf,fullfile(setup.postOptions.PathJpg,figname),'jpg');
        end
        if setup.postOptions.Fig
            savefig(fullfile(setup.postOptions.PathFig,figname));
        end
    end 

end