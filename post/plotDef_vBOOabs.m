function plotDef_vBOOabs(Setup, Results)
% Plot absolute defender velocity
c = Setup.PostOptions.c;
vBOO =  Results.Defender.States.Vel;

vBOO_abs = vecnorm(vBOO);

Figname = 'Absolute Defender Velocity';

% Plot Interceptor
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);
plot(Results.Time, vBOO_abs, 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
title([Figname],c.Title{:});
xlabel('Time [s]',c.Label{:});
ylabel('Velocity [m/s]',c.Label{:});


if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'jpg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,Figname));
    end
end

end