r_man = @(r,s)r/(sqrt(8)*sin(s*pi/180))*(1+2*sin(s*pi/180)^2);
% r_man_lin = @(r,s)r/(2*(s*pi/180))*(1+(s*pi/180)^2);
V = @(r,T2W)sqrt(r*T2W*9.81);