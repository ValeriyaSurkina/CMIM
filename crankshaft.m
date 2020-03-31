% Hands on solution for Simple Mechanism task
clear all
clc
close all

a = 0.1;
b = 0.2;
omega=1;
t=0:0.01:10;
phi = pi/6+omega*t;

% set a reasonable starting point
u0 = [0; b + a];
u0_dot=[1;2;3];
J = @(u) jacobian(u, b);
eps = 1e-9;
U=[];
U_dot=[];

for i=1:length(phi)
% create function handles
F = @(u) constraint(u, a, b, phi(i));
x=NR_method(F, J, u0, eps);
U=[U x];
theta=(U(1,i));
F_dot = @(u)constraint_dot(u, a, b, phi(i),theta);
J_dot = @(u)jacobian_dot(phi(i),theta, a, b);
x_dot=NR_method(F_dot, J_dot, u0_dot, eps);
U_dot=[U_dot x_dot];
end

phi_dot=(U_dot(1,:));
theta_dot=(U_dot(2,:));
d_dot=(U_dot(3,:));
theta=(U(1,:));
d=U(2,:);

figure 
plot(t, rad2deg(theta))
title('Angle \theta over time');
xlabel('Time t (s)')
ylabel('Angle \theta (\circ) ')

figure 
plot(t, d)
title('Displacement d over time');
xlabel('Time t (s)')
ylabel('Displacement d (m) ')

figure 
plot(t, d_dot)
title('Velocity $\dot{d}$ over time', 'Interpreter','latex');
xlabel('Time t (s)')
ylabel('Velocity $\dot{d}$ (m/s)', 'Interpreter','latex')

figure 
plot(t, theta_dot)
title('Angular velocity $\dot{\theta}$ over time', 'Interpreter','latex');
xlabel('Time t (s)')
ylabel('Angular velocity $\dot{d}$ (rad/s)', 'Interpreter','latex')



   
function P = constraint_dot(u, a, b, phi,theta)
phi_dot = u(1);
theta_dot = u(2);
d_dot=u(3);

P = [-a * phi_dot*sin(phi) - b *theta_dot* sin(theta) - d_dot;
    a * phi_dot*cos(phi) - b*theta_dot*cos(theta)];
end


function P = jacobian_dot(phi,theta, a, b)
P = [-a*sin(phi), -b*sin(theta), -1 
    a*cos(phi), -b*cos(theta), 0];
end



function P = constraint(u, a, b, phi)
theta = u(1);
d = u(2);

P = [a * cos(phi) + b * cos(theta) - d;
    a * sin(phi) - b * sin(theta)];
end

function P = jacobian(u, b)
theta = u(1);
P = [-b * sin(theta), -1
    -b * cos(theta), 0];
end

