clear 
close all
clc
omega = 2;
P = 2*pi/omega;
dt = P/20;
dt1 = P/2000;
T = 4*P;
N_t = floor(round(T/dt));
t = linspace(0, N_t*dt, N_t+1);

[t, u , X_0,]=backward(N_t, dt, omega)
[t1, u1 , X_01,]=backward(N_t, dt1, omega)

plot(t, u, 'r-', t, X_0*cos(omega*t), 'k');
legend('numerical', 'exact', 'Location', 'northwest');
title ('Position via Backward Euler scheme with 20 steps/period and X_0cos(\omega t)')
xlabel('t');

figure
plot(t1, u1, 'r-', t, X_01*cos(omega*t), 'k');
legend('numerical', 'exact', 'Location', 'northwest');
title ('Position via Backward Euler scheme with 2000 steps/period and X_0cos(\omega t)')
xlabel('t');
function [t, u, X_0] = backward(N_t, dt, omega)
t = linspace(0, N_t*dt, N_t+1);
u = zeros(N_t+1, 1);
v = zeros(N_t+1, 1);

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;
for n = 2:N_t+1
    u(n) = (1.0/(1+(dt*omega)^2)) * (dt*v(n-1) + u(n-1));
    v(n) = (1.0/(1+(dt*omega)^2)) * (-dt*omega^2*u(n-1) + v(n-1));
end
end


