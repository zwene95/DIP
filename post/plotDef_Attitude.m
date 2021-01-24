function plotDef_Attitude(Setup,Results)

    c = Setup.PostOptions.c;

    % Real time
    t = Results.Time;
    
    % Get defender euler angles
    switch Setup.ModelOptions.Defender.Attitude
        case 'Euler'
            Phi     = Results.Defender.States.Att(1,:) * c.rad2deg;
            Theta   = Results.Defender.States.Att(2,:) * c.rad2deg;
            Psi     = Results.Defender.States.Att(3,:) * c.rad2deg;
        otherwise
            error('Defender attitude options not yet implemented');
    end
    
    Figname = 'Defender Attitude';

    % LOS
    figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);    
    % Plot phi
    ax1 = subplot(3,1,1);
    plot(t,Phi,'k','LineWidth',2);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,c.Axes{:});
    grid on;
    xlabel('Time [s]',c.Label{:});
    ylabel('$$\Phi$$ [deg]',c.Label{:});    
    title([Figname],c.Title{:});

    % Plot theta
    ax2 = subplot(3,1,2);
    plot(t,Theta,'k','LineWidth',2);
    grid on;
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,c.Axes{:});
    xlabel('Time [s]',c.Label{:});
    ylabel('$$\Theta$$ [deg]',c.Label{:});
    
    % Plot psi rate
    ax3 = subplot(3,1,3);
    plot(t,Psi,'k','LineWidth',2);
    set(gca,'XMinorTick','on');
    set(gca,'YMinorTick','on');
    set(gca,c.Axes{:});
    grid on;
    xlabel('Time [s]',c.Label{:});
    ylabel('$$\Psi$$ [deg]',c.Label{:});    
    
    linkaxes([ax1,ax2,ax3],'x');
    
    % Save plot
    if Setup.PostOptions.Save
        if Setup.PostOptions.Jpg
            saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'jpg');
        end
        if Setup.PostOptions.Fig
            savefig(fullfile(Setup.PostOptions.PathFig,Figname));
        end
    end

end