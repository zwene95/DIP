function plotDef_aBOOabs(Setup, Results)
% Plot absolute defender acceleration
c = Setup.PostOptions.c;
aBOO =  Results.Defender.States.Acc;

aBOO_abs = vecnorm(aBOO);

Figname = 'Absolute Defender Acceleration';

% Plot Interceptor
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);
plot(Results.Time, aBOO_abs, 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
title([Figname,newline],c.Title{:});
xlabel('Time [s]',c.Label{:});
ylabel('Acceleration $$[m/s^2]$$',c.Label{:});


if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'jpg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,Figname));
    end
end

end