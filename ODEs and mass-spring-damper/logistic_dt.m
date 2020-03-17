
%Ex4.4
close all
clear
clc
g = @(u, t) 0.1*(1-u/500)*u;
U_0 = 100;
dt = 20;
T = 100;
[u0, t0] = ode_FE(g, U_0, dt, T);
k=1;
i=1;
while i==1
    dt=2^(-k)*dt
[u, t] = ode_FE(g, U_0, dt, T);
plot(t0, u0, 'b-',t, u, 'r-');
xlabel('t'); ylabel('N(t)');
fprintf('dt: %g\n', dt);
user = input('Continue?', 's')
if strcmp(user,'yes')
    u0 = u;
    t0 = t;
    else
        i = 0;
    end
    k = k + 1;
end
function [u, time] = ode_FE(f, U_0, dt, T)
N=floor(T/dt)
u(1) = U_0;
for i = 1:N;
    u(i+1) = u(i) +dt * f(u(i),T);
end
u;
time = [0:dt:T];
end
