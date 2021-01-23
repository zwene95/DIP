function plotDefenderParameter(Setup,Results)
% Plot absolute defender velocity and acceleration

c = Setup.PostOptions.c;

% Defender absolute velocity and acceleration
vBOO =  Results.Defender.States.Vel;
aBOO =  Results.Defender.States.Acc;
vBOO_abs = vecnorm(vBOO);
aBOO_abs = vecnorm(aBOO);

% Defender thrust
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

% Create figure
Figname = 'Defender Parameters';
figure('Tag',Figname,'name', Figname,'Position', c.Pos_Groesse_SVGA);
% sgtitle(Figname,c.Title{:});

% Plot velocity
ax1 = subplot(3,1,1);
plot(Results.Time, vBOO_abs, 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
ylim([0,inf]);
title(['Defender Abolute Velocity'],c.Subtitle{:});
xlabel('Time [s]',c.Label{:});
ylabel('Velocity [m/s]',c.Label{:});

ax2 = subplot(3,1,2);
plot(Results.Time, aBOO_abs, 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca, c.Axes{:});
ylim([0,inf]);
title(['Defender Abolute Acceleration'],c.Subtitle{:});
xlabel('Time [s]',c.Label{:});
ylabel('Acceleration $$[m/s^2]$$',c.Label{:});

ax3 = subplot(3,1,3);
plot(Results.Time, T, 'LineWidth', 2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca,c.Axes{:});
title(['Defender Total Thrust'],c.Subtitle{:});
xlabel('Time [s]',c.Label{:});
ylabel('Thrust [N]',c.Label{:});

linkaxes([ax1,ax2,ax3],'x');


if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,Figname),'svg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,Figname));
    end
end

end

