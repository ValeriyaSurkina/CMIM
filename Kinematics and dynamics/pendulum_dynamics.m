
clear all;
close all;
clc;

%% Coordinates
% ground
l=1
%%Global Coordinates
% ground
q1 = [0; 0; 0];
% bar 1
q2 = [l/2; 0 ; 0];
% bar 2
q3 = [l+l/2; 0; deg2rad(0)];

% bar 3
q4 = [l+l/2+l/2; 0 ;deg2rad(0)];
q_0 = [q1; q2; q3; q4]; % initial coordinates

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and link 1
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-l/2; 0];

% 2 connects link 1 and link 2
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [l/2; 0];
revolute(2).s_j = [-l/2; 0];

% 3 connects link 2 and link 3
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [l/2; 0];
revolute(3).s_j = [-l/2; 0];


%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

%% Dynamic Analysis
%  Define bodies
body(1).m = 0; % ground mass equals to 0 kg
body(1).l = 0; 
body(1).Ic = body(1).m * body(1).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(1).q = q1;

body(2).m = 1; % crank mass equals to 2 kg
body(2).l = l; 
body(2).Ic = body(2).m * body(2).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(2).q = q2;

body(3).m = 1; % link mass equals to 2 kg
body(3).l = l; 
body(3).Ic = body(3).m * body(3).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(3).q = q3;

body(4).m = 1; % slider mass equals to 2 kg
body(4).l = l; 
body(4).Ic = body(4).m * body(4).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(4).q =q4;

p.grav = [0; -9.81]; % gravitational acceleration

%% Get mass matrix

p.M = mass_matrix(body);
q0 = [q_0;zeros(length(q_0), 1)];
tspan = linspace(0,10,200);

sforce.f = [1; 0];
sforce.i = 1;
sforce.u_i = [0; 1];

p.bodies = body;
% g_fun= @(t, q, dq) g_baum(revolute, simple, driving, t, q, dq);

p.C_fun = @(t, q) constraint_dynamics(revolute, simple, t, q);
p.Cq_fun = @(t, q) constraint_dynamics_dq(revolute, simple, t, q);
p.g_fun= @(t, q, dq) g_dynamics(revolute, simple, t, q, dq);

%% Time to integrate it
% Note that M is constant, but F, in general, no
% We can use one of the following:
%   ode45 from Matlab
options = odeset('Stats','on','RelTol',1e-8);
[t, u] = ode45(@c_equation, tspan, q0, options, p);
q = u';

% Vizualization 


for i = 1:length(tspan)
b1j1 = [q(4,i);q(5,i)] + rot(q(6,i))*[l/2;0];
b1j2 = [q(4,i);q(5,i)] + rot(q(6,i))*[-l/2;0];
b2j2 = [q(7,i);q(8,i)] + rot(q(9,i))*[l/2;0];
b2j3 = [q(7,i);q(8,i)] + rot(q(9,i))*[-l/2;0];
b3j3 = [q(10,i);q(11,i)] + rot(q(12,i))*[l/2;0];
b3j4 = [q(10,i);q(11,i)] + rot(q(12,i))*[-l/2;0];
end

plot(0,0, 'o')
hold on
plot([b1j1(1), b1j2(1)], [b1j1(2), b1j2(2)]);
plot([b2j2(1), b2j3(1)], [b2j2(2), b2j3(2)]);
plot([b3j3(1), b3j4(1)], [b3j3(2), b3j4(2)]);
legend('Origin','Bar 1','Bar 2','Bar 3');
title('Dynamic Analysis for Slider Crank Mechanism');   
xlabel('r_x (meters)');
ylabel('r_y (meters)');  



