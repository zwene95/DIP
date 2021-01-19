function Plot_Missdistance(Setup, Results)

c = Setup.PostOptions.c;

pDOO =  Results.Defender.States.Pos;

pIOO =  Results.Invader.States.Pos;

pVDO = pIOO - pDOO;
Missdistance = vecnorm(pVDO);
Capturedistance = vecnorm(pIOO(1:2,end));

figname = 'Missdistance';

figure('Tag',figname,'name', figname,'Position', c.Pos_Groesse_SVGA);
plot(Results.Time, Missdistance,'LineWidth',2);
grid on;
set(gca,'XMinorTick','on');
set(gca,'YMinorTick','on');
set(gca,c.Axes{:});
xlabel('Time [s]',c.Label{:})
ylabel('Missdistance [m]',c.Label{:})
title([figname,newline],c.Title{:});
legend(sprintf('Missdistance = %.3fm',min(Missdistance)),c.Legend{:});
%     legend(sprintf('Missdistance = %.3fm\nCaptureDistance = %.3fm',min(missdistance),captureDistance));
fprintf('Missdistance = %.3fm \n',min(Missdistance));
fprintf('Distance of intercept = %.3fm \n',Capturedistance);
% axis image;


if Setup.PostOptions.Save
    if Setup.PostOptions.Jpg
        saveas(gcf,fullfile(Setup.PostOptions.PathJpg,figname),'jpg');
    end
    if Setup.PostOptions.Fig
        savefig(fullfile(Setup.PostOptions.PathFig,figname));
    end
end

end