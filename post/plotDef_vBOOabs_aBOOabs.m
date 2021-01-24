function plotDef_vBOOabs_aBOOabs(Setup,Results)
% Plot absolute defender velocity and acceleration
c = Setup.PostOptions.c;
vBOO =  Results.Defender.States.Vel;
aBOO =  Results.Defender.States.Acc;

vBOO_abs = vecnorm(vBOO);
aBOO_abs = vecnorm(aBOO);

% Create figure
Figname = 'Absolute Defender Velocity and Acceleration';
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);
sgtitle(Figname,c.Title{:});

% Plot velocity
ax1 = subplot(2,1,1);
plot(Results.Time, vBOO_abs,'k', 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
ylim([0,inf]);
title(['Abolute Velocity'],c.Subtitle{:});
xlabel('Time [s]',c.Label{:});
ylabel('Velocity [m/s]',c.Label{:});

ax2 = subplot(2,1,2);
plot(Results.Time, aBOO_abs,'k', 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
ylim([0,inf]);
title(['Abolute Acceleration'],c.Subtitle{:});
xlabel('Time [s]',c.Label{:});
ylabel('Acceleration $$[m/s^2]$$',c.Label{:});

linkaxes([ax1,ax2],'x');


if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'jpg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,Figname));
    end
end

end

