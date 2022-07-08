clear; clc; close all;
%% Parameters
tfinal = 200;  %% 仿真时间

%% Simulation
sim("parafoil_model.slx", tfinal);

%% Compute R
V = sqrt(dx.^2 + dy.^2 + dh.^2);
gamma = asin(dh ./ V);
R = abs(V .* cos(gamma) ./ dpsi);
K = (x(end) - x(1)) / (h(end) - h(1));
fprintf(['R = ', num2str(mean(R)), '\tK = ', num2str(abs(K)), '\n']);