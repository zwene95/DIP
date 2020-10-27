%% Atan approximation

x = linspace(-1,1,200);


f = @(x) x - x.^3 / 3 + x.^5/5 - x.^7/7;
g = @(x) atan(x);

figure; hold on; grid on;legend;
plot(x,f(x),'-r','DisplayName','Approx');
plot(x,g(x),'--g','DisplayName','atan');

figure; grid on;
err = sqrt(f(x).^2 + g(x).^2) * 180/pi;
plot(x,err,'--g','DisplayName','error');