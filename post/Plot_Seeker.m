function Plot_Seeker(Setup, Results)

c = Setup.PostOptions.c;

pDOO = Results.Defender.States.Pos;
pIOO = Results.Invader.States.Pos;
pDIO = pIOO - pDOO;

Azimuth =   atan2(pDIO(2,:),pDIO(1,:)) * c.rad2deg;
Elevation = atan2(-pDIO(3,:),sqrt(pDIO(1,:).^2 + pDIO(2,:).^2)) * c.rad2deg;
t = Results.Time;

% Get defender euler angles
switch Setup.ModelOptions.Defender.Attitude
    case 'Euler'        
        Theta = Results.Defender.States.Att(2,:) * c.rad2deg;
        Psi =   Results.Defender.States.Att(3,:) * c.rad2deg;
    otherwise
        error('Defender attitude options not yet implemented');
end

% FoV boundaries
FoV_el = ones(1,2) * Setup.DefenderConfig.FoV(1)/2;
FoV_az = ones(1,2) * Setup.DefenderConfig.FoV(2)/2;

% Create figure
Figname = 'Seeker Body Angles';
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);

% Plot azimuth
ax1 = subplot(2,1,1);
hold on; grid on;
plot(t,Azimuth - Psi,'k','LineWidth',2);
plot([t(1) t(end)], +FoV_az, '--r', 'LineWidth', 2);
plot([t(1) t(end)], -FoV_az, '--r', 'LineWidth', 2);
title('Azimuth',c.Subtitle{:});
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca,c.Axes{:});
%     legend([p1 p2(1)],'$$\beta_B$$', 'FOV','Interpreter','latex');
xlabel('Time [s]',c.Label{:});
ylabel('$$\beta_B$$ in [$$^\circ$$]',c.Label{:});

% Plot elevation
ax2 = subplot(2,1,2);
hold on; grid on;
plot(t,Elevation - Theta,'k','LineWidth',2);
plot([t(1) t(end)], +FoV_el, '--r', 'LineWidth', 2);
plot([t(1) t(end)], -FoV_el, '--r', 'LineWidth', 2);
title('Elevation',c.Subtitle{:});
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
xlabel('Time [s]',c.Label{:});
ylabel('$$\epsilon_B$$ in [$$^\circ$$]',c.Label{:});


linkaxes([ax1,ax2],'x');

% Plot overall title
sgtitle(Figname,c.Title{:});

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