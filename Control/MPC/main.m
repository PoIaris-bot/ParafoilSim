clear; clc; close;
addpath('..\');
load('route_PSO.mat');
% load('route_LPM.mat');

x = [1200; 600; 2000; pi / 3; 0];
u = [0; 0];
dt = 0.5;

figure(1);
plot3(route(1, :), route(2, :), route(3, :), 'r.-');
grid on; axis equal; hold on;

problem.V = 20;
problem.dt = dt;
problem.numStates = 5;
problem.numInputs = 2;
problem.numOutputs = 3;
problem.P = 30;  % 预测时域
problem.M = 20;  % 控制时域
problem.rho = 10;
problem.umin = [-0.3; -0.3];
problem.umax = [0.3; 0.3];
problem.delta_umin = [-0.1; -0.1];
problem.delta_umax = [0.1; 0.1];

idx = 0;
t = 0;
x_store = x;
u_store = u;
tic;
while idx < size(route, 2)
    [u, idx] = mpc_control(x, route, u, problem);
    u = saturationCons(u, [-0.1; -0.1], [0.1; 0.1]);
    [t, x] = updateState(x, u, t, problem);
    x_store = [x_store, x]; u_store = [u_store, u];
    % plot3(x(1), x(2), x(3), 'go');
    % pause(0.01);
end
toc;
plot3(x_store(1, :), x_store(2, :), x_store(3, :), 'b.-');
legend('desired', 'real');
figure(2);
plot(0:dt:t, u_store)
legend('$u_1$', '$u_2$', 'Interpreter', 'latex')
