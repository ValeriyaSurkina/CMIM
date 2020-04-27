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

%% Add some driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) pi/4 + 1.2 * t;
driving.d_k_t = @(t) 1.2;
driving.d_k_tt = @(t) 0;

%% Solve constraint equation using NR for position, velocity and acceleration
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
Ctt_fun= @(t, q, dq) constraint_ddt(revolute, simple, driving, t, q, dq);
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, 1, q_0, 0.1);

%% Some verification plots
figure
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Position Analysis for Pendulum')
legend('Bar 1','Bar 2','Bar 3','Origin')

%% Some verification plots
figure
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title('Velocity Analysis for Pendulum')
legend('Bar 1','Bar 2','Bar 3','Origin')

%% Some verification plots
figure
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    0, 0, '*');
axis equal
title('Acceleration Analysis for Pendulum')
legend('Bar 1','Bar 2','Bar 3','Origin')