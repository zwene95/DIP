function plotDef_Thrust(Setup, Results)

c = Setup.PostOptions.c;


if Setup.ModelOptions.Defender.SixDoF
    if Setup.ModelOptions.Defender.MotorLag
        c1 = Results.Defender.States.Motor.W1;
        c2 = Results.Defender.States.Motor.W2;
        c3 = Results.Defender.States.Motor.W3;
        c4 = Results.Defender.States.Motor.W4;
    else
        c1 = Results.Defender.Controls.W1;
        c2 = Results.Defender.Controls.W2;
        c3 = Results.Defender.Controls.W3;
        c4 = Results.Defender.Controls.W4;        
    end
    T = ( c1.^2 + c2.^2 + c3.^2 + c4.^2 ) * Setup.ModelOptions.Defender.m * 9.80665 * Setup.DefenderConfig.T2W_max / 4 / Setup.ModelOptions.Defender.RPM_max^2;
    % 3DoF
else
    controls = [
        Results.Defender.Controls.Tx
        Results.Defender.Controls.Ty
        Results.Defender.Controls.Tz];
    T = vecnorm(controls) * Setup.ModelOptions.Defender.m * 9.80665;
end

Figname = 'Defender Thrust';

% Plot Interceptor
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);
plot(Results.Time, T, 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca,c.Axes{:});
title([Figname],c.Title{:});
xlabel('Time [s]',c.Label{:});
ylabel('Thrust [N]',c.Label{:});



if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'jpg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,Figname));
    end
end

end