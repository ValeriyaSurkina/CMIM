clear all
clc
close all

omega = 2;
P = 2*pi/omega;
dt = P/100;
T = 3*P;
N_t = floor(round(T/dt));
t = linspace(0, N_t*dt, N_t+1);

u = zeros(N_t+1, 1);
v = zeros(N_t+1, 1);

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;

% Step equations forward in time
for n = 1:N_t
    u(n+1) = u(n) + dt*v(n);
    v(n+1) = v(n) - dt*omega^2*u(n);
end


[U, K] = osc_energy(u, v, omega);

plot(t, U+K, 'b-');
xlabel('t');
ylabel('U+K (Forward Euler)');
figure
plot(t, v, 'r-',t,u , 'b-');
xlabel('t');
ylabel('u,v (Forward Euler)');
% Euler-Cromer

u = zeros(N_t+1,1);
v = zeros(N_t+1,1);

% Initial condition
u(1) = 2;
v(1) = 0;
figure 

% Step equations forward in time
for n = 1:N_t
    v(n+1) = v(n) - dt*omega^2*u(n);
    u(n+1) = u(n) + dt*v(n+1);
end
hold on
plot(t, v, 'r-',t, u , 'b-');
xlabel('t');
ylabel('u,v (Cromer)');

[U1, K1] = osc_energy(u, v, omega);

figure 
plot(t, U1+K1, 'b-');
xlabel('t');
ylabel('U+K (Cromer)');
