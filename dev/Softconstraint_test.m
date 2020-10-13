%% Softconstraint function test
syms z p f(z,p);

f(z,p) = (1 - tanh(z./p*3));
df = diff(f,z);
ddf = diff(f,z,2);

x = linspace(0,10000);
x_th = 2500;

subplot(3,1,1);
plot(x,f(x,x_th));grid on;
title('f');
subplot(3,1,2);
plot(x,df(x,x_th));grid on;
title('df');
subplot(3,1,3);
plot(x,ddf(x,x_th));grid on;
title('ddf');


%% TargetViolationCostFcn

syms z p f(z,p);
% f(z,p) = ( 1 + tanh(-z^2./p^2*4));
f(z,p) = ( 1 + tanh( (-z^2 + p^2 ) * 1000 ) ) * (p^2 - z^2) / 2;
df = diff(f,z);
ddf = diff(f,z,2);

r = linspace(0,160,1000);
r_th = 100;

subplot(3,1,1);
plot(r,f(r,r_th));grid on;
title('f');
subplot(3,1,2);
plot(r,df(r,r_th));grid on;
title('df');
subplot(3,1,3);
plot(r,ddf(r,r_th));grid on;
title('ddf');

%% Escape maneuver

syms z p f(z,p);
f(z,p) = ( 1 + tanh( (-z + p ) * 100 ) ) / 2;
df = diff(f,z);
ddf = diff(f,z,2);

r = linspace(0,160,1000);
r_th = 10;

subplot(3,1,1);
plot(r,f(r,r_th));grid on;
title('f');
subplot(3,1,2);
plot(r,df(r,r_th));grid on;
title('df');
subplot(3,1,3);
plot(r,ddf(r,r_th));grid on;
title('ddf');

%% Stop maneuver

syms z p f(z,p);
% f(z,p) = ( 1 - tanh( (-z + p ) * 1 ) ) / 2;
f(z,p) = ( 1 - tanh( (-z^2 + p^2 ) * 1 ) ) / 2;
df = diff(f,z);
ddf = diff(f,z,2);

r = linspace(0,160,1000);
r_th = 50;

subplot(3,1,1);
plot(r,f(r,r_th));grid on;
title('f');
subplot(3,1,2);
plot(r,df(r,r_th));grid on;
title('df');
subplot(3,1,3);
plot(r,ddf(r,r_th));grid on;
title('ddf');

%% Reverse maneuver

syms z p f(z,p);
f(z,p) = tanh( (z - p ) * 1000 );
df = diff(f,z);
ddf = diff(f,z,2);

r = linspace(0,160,1000);
r_th = 50;

subplot(3,1,1);
plot(r,f(r,r_th));grid on;
title('f');
subplot(3,1,2);
plot(r,df(r,r_th));grid on;
title('df');
subplot(3,1,3);
plot(r,ddf(r,r_th));grid on;
title('ddf');

%% Quiver plot
hold on;
v1 = [1;1;-1];
v2 = [1;1;0];
v3 = @(x,y) cross(x,y);
o = zeros(3,1);
V = [v1,v2,v3(v1,v2)];
quiver3(o,o,o,v1,v2,v3(v1,v2));
xlabel('x');
ylabel('y');
zlabel('z');

%%
pTOO =  [   problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)];
rI = vecnorm(pTOO);
r2I = rI.^2;
r2T = problem.Parameters(find(ismember({problem.Parameters.Name},'captureRadius'),1),:).Value^2;

figure;
f = 1+tanh(-r2I./r2T*4);
plot(rI,f);grid on;
    
