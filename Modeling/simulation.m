clear; clc; close all;
%% Parameters
tfinal = 200;  %% 仿真时间

%% Simulation
sim("parafoil_model.slx", tfinal);
  
%% Visualization
figure(1);
plot3(x, y, h, 'LineWidth', 2);
axis equal; grid on;
xlim([0, 800]); xlabel('x', 'FontSize', 15);
ylim([-400, 400]); ylabel('y', 'FontSize', 15);
zlim([0, 1100]); zlabel('h', 'FontSize', 15);

figure(2);
plot(x, y, 'LineWidth', 2);
axis equal; grid on;
xlabel('x', 'FontSize', 15);
ylabel('y', 'FontSize', 15);

figure(3);
plot(x, h, 'LineWidth', 2);
grid on;
xlabel('x', 'FontSize', 15);
ylabel('h', 'FontSize', 15);

figure(4);
plot(t, psi / pi * 180, 'LineWidth', 2);
grid on;
xlim([0, tfinal]); xlabel('t', 'FontSize', 15);
ylabel('$\psi(^\circ)$', 'Interpreter', 'latex', 'FontSize', 15);

V = sqrt(dx.^2 + dy.^2 + dh.^2);
gamma = asin(dh ./ V);
figure(5);
plot(t, gamma / pi * 180, 'LineWidth', 2);
grid on;
xlim([0, tfinal]); xlabel('t', 'FontSize', 15);
ylabel('$\gamma(^\circ)$', 'Interpreter', 'latex', 'FontSize', 15);