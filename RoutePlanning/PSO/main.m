clear; clc; close all;
%% 系统参数设置
% 参数设置
problem.x0 = 1200;
problem.y0 = 600;
problem.h0 = 2000;
problem.psi0 = pi / 3;
problem.k = 3;
problem.Rmin = 100;
%% PSO参数设置
c1 = 1;
c2 = 1;
w = 0.6;
num_iter = 200;
dim = 2;
N = 50;
epsilon = 1e-6;
% 搜索范围
x_lower = [100, -pi];
x_upper = [1000, pi];
%% 初始化种群
% 随机初始位置
x = repmat(x_lower, N, 1) + repmat((x_upper - x_lower), N, 1) .* rand(N, dim);
% 随机初始速度
v = randn(N, dim);
% 初始化适应度
x_particle = x;  % 每个个体的历史最佳位置
y_particle = fitness_value(x, problem);  % 每个个体的历史最佳适应度 
[y_global, index] = min(y_particle);  % 种群的历史最佳适应度
y_global = y_global(1);
x_global = x_particle(index(1), :);  % 种群的历史最佳位置
%% 迭代搜索
for i = 1:num_iter
    % 更新速度和位置
    v = w * v + c1 * rand * (x_particle - x) + c2 * rand * (repmat(x_global, N, 1) - x);
    x = x + v;
    % 边界位置处理
    index = find(x(:, 1) > x_upper(:, 1));
    if ~isempty(index)
        x(index, 1) = x_upper(:, 1);
    end
    index = find(x(:, 2) > x_upper(:, 2));
    if ~isempty(index)
        x(index, 2) = x_upper(:, 2);
    end

    index = find(x(:, 1) < x_lower(:, 1));
    if ~isempty(index)
        x(index, 1) = x_lower(:, 1);
    end
    index = find(x(:, 2) < x_lower(:, 2));
    if ~isempty(index)
        x(index, 2) = x_lower(:, 2);
    end
    % 更新最佳适应度
    y = fitness_value(x, problem);
    index = find(y < y_particle);
    if ~isempty(index)
        y_particle(index) = y(index);
        x_particle(index, :) = x(index, :);
    end
    [y_global, index] = min(y_particle);
    y_global = y_global(1);
    x_global = x_particle(index(1), :);

    if y_global < epsilon
        break;
    end
end
problem.Rep = x_global(1);
problem.theta_ep = x_global(2);
visualize(problem);