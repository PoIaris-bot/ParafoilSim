clear; clc; close all;
%% 轨迹参数
% 区域范围
xmin = 0; xmax = 2000;
ymin = 0; ymax = 2000;
zmin = 0; zmax = 2000;
% 投放点
x0 = 1800;
y0 = 1800;
z0 = 1800;
psi0 = -120 * pi / 180;
% 目标点
xf = 700;
yf = 400;
zf = 0;
psif = pi;

Rmin = 50;
Rth = 300;
k = 3;
% 障碍物参数
peaks.x = [450, 1500, 320, 1360, 1060, 1600];
peaks.y = [1460, 510, 950, 1650, 1085, 1100];
peaks.h = [1755, 1300, 1621, 1550, 1870, 1450];
peaks.xs = [340, 280, 280, 170, 230, 250];
peaks.ys = [340, 220, 190, 230, 250, 270];
%% RRT
Step = 50;
sample_rate = 0.3;
height_margin = 20;
max_tree_size = 500;
beta_max = 2 * atan(0.5 * Step / Rmin);
if beta_max < 0
    beta_max = beta_max + pi;
end
% 初始化
P_init = node([x0; y0]);
P_goal = node([xf; yf]);

x_root = x0 + Step * cos(psi0);
y_root = y0 + Step * sin(psi0);
P_root = node([x_root; y_root], 1, 0);
tree_nodes = P_root;

X_init = node([x0; y0; z0]);
X_root = node([
    (x0 + x_root) / 2;
    (y0 + y_root) / 2;
    z0 - Step / 2 / k;
    ], 1);
trajectory_nodes = X_root;
trail.x = X_root.x(1):(X_init.x(1) - X_root.x(1)) / 100:X_init.x(1);
trail.y = X_root.x(2):(X_init.x(2) - X_root.x(2)) / 100:X_init.x(2);
trail.z = X_root.x(3):(X_init.x(3) - X_root.x(3)) / 100:X_init.x(3);
trail.R = inf; trail.beta = 0;
trail_parts = trail;

% 搜索
x = xmin:xmax;
y = ymin:ymax;
[X, Y] = meshgrid(x, y);
Z = zeros(size(X));
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        Z(i, j) = height(X(i, j), Y(i, j), peaks);
    end
end

figure(1);
pcolor(X, Y, Z);
colormap; colorbar;
shading interp;
hold on;
theta = 0:pi / 200:2 * pi;
x = xf + Rth * cos(theta);
y = yf + Rth * sin(theta);
plot(x, y, 'r--', 'LineWidth', 1);
axis([xmin, xmax, ymin, ymax]); axis equal;
hold on;
plot([x0, x_root], [y0, y_root], 'r.-', 'LineWidth', 1, 'MarkerSize', 5);
hold on;
figure(2);
mesh(X, Y, Z);
surf(X, Y, Z);
colormap; colorbar;
shading interp;
hold on;
z = 100 * ones(size(x));
plot3(x, y, z, 'r--', 'LineWidth', 1);
xlim([xmin, xmax]); ylim([ymin, ymax]); zlim([zmin, zmax]); axis equal;
hold on;

i = 2;
is_path_found = false;
while i < max_tree_size
    % 随机采样
    if rand < sample_rate
        P_rand = P_goal;
    else
        x_rnd = [
            xmin + rand * (xmax - xmin);
            ymin + rand * (ymax - ymin);
            ];
        P_rand = node(x_rnd);
    end
    % 寻找邻节点
    dist = distance(P_rand, tree_nodes);
    [dist_near, index] = min(dist);
    P_near = tree_nodes(index(1));
    % 生成新节点
    x_new = P_near.x + Step / dist_near(1) * (P_rand.x - P_near.x);
    P_new = node(x_new);
    % 判断是否与障碍物冲突
    if P_near.parent == 0
        P_parent = P_init;
    else
        P_parent = tree_nodes(P_near.parent);
    end
    trail = trajectory(P_parent, P_near, P_new, trajectory_nodes(P_near.id), k, Step);
    if is_feasible(trail, height_margin, beta_max, peaks)
        P_new.id = i; P_new.parent = P_near.id;
        tree_nodes = [tree_nodes, P_new];
        X_new = node([
            (P_near.x + P_new.x) / 2;
            trail.z(1);
            ], i);
        trajectory_nodes = [trajectory_nodes, X_new];
        trail_parts = [trail_parts, trail];

        % 搜索树生长过程
        figure(1);
        plot([P_near.x(1), P_new.x(1)], [P_near.x(2), P_new.x(2)], 'r.-', 'LineWidth', 1, 'MarkerSize', 5);
        hold on;

        figure(2);
        plot3(trail.x, trail.y, trail.z, 'r', 'LineWidth', 1);
        hold on;

        if distance(P_goal, P_new) < Rth
            is_path_found = true;
            break;
        end
        i = i + 1;
    end
end
figure(2);
xlim([xmin, xmax]); ylim([ymin, ymax]); zlim([zmin, zmax]);

% 生成路径
if is_path_found
    path = P_new.x;
    parent = P_new.parent;
    path3d = [
        P_new.x(1), trail_parts(end).x;
        P_new.x(2), trail_parts(end).y;
        trail_parts(end).z(1) - Step / 2 / k, trail_parts(end).z;
        ];
    while parent > 0
        path3d = [path3d, [
            trail_parts(parent).x;
            trail_parts(parent).y;
            trail_parts(parent).z;
            ]];
        P_parent = tree_nodes(parent);
        path = [path, P_parent.x];

        parent = P_parent.parent;
    end
    path = [path, P_init.x];
    figure(1);
    plot(path(1, :), path(2, :), 'g.-', 'LineWidth', 1.5, 'MarkerSize', 5);
    hold on;
    figure(2);
    plot3(path3d(1, :), path3d(2, :), path3d(3, :), 'g', 'LineWidth', 1.5);
    hold on;

    %% PSO
    % 轨迹参数设置
    xa = path3d(1, 1);
    ya = path3d(2, 1);
    za = path3d(3, 1);
    psia = angle(path3d(1, 2), path3d(2, 2), path3d(1, 1), path3d(2, 1));
    tau = 1;

    problem.xa = xa;
    problem.ya = ya;
    problem.za = za;
    problem.psia = psia;

    problem.xf = xf;
    problem.yf = yf;
    problem.zf = zf;

    problem.k = k;
    problem.Rmin = Rmin;
    problem.tau = tau;
    % PSO参数设置
    c1 = 1;
    c2 = 1;
    w = 0.6;
    num_iter = 500;
    dim = 2;
    N = 50;
    epsilon = 1e-6;
    % 搜索范围
    x_lower = [Rmin, -pi];
    x_upper = [Rth, pi];
    % 初始化种群
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
    % 迭代搜索
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
    Rep = x_global(1);
    theta_ep = x_global(2);
    % 可视化
    xO1 = xa + Rmin * cos(psia - pi / 2);
    yO1 = ya + Rmin * sin(psia - pi / 2);

    xO2 = xf + (Rep - Rmin) * cos(theta_ep);
    yO2 = yf + (Rep - Rmin) * sin(theta_ep);

    xbc = xO2 - xO1;
    ybc = yO2 - yO1;

    if xbc == 0
        angle_bc = sign(ybc) * pi / 2;
    else
        angle_bc = ((1 - sign(xbc)) / 2) .* sign(ybc) * pi + atan(ybc / xbc);
    end

    beta1 = -(angle_bc - psia);
    if beta1 < 0
        beta1 = beta1 + 2 * pi;
    end

    gamma1 = psia + pi / 2 - beta1:pi / 100:psia + pi / 2;
    x1 = xO1 + Rmin * cos(gamma1);
    y1 = yO1 + Rmin * sin(gamma1);
    z1 = za + Rmin * (gamma1 - (psia + pi / 2)) / k;
    figure(1);
    plot(x1, y1, 'g', 'LineWidth', 1.5);
    hold on
    figure(2);
    plot3(x1, y1, z1, 'g', 'LineWidth', 1.5);
    hold on

    xb = xO1 + Rmin * cos(psia + pi / 2 - beta1);
    yb = yO1 + Rmin * sin(psia + pi / 2 - beta1);
    zb = za + Rmin * (-beta1) / k;

    beta2 = -(theta_ep - angle_bc - pi / 2);
    if beta2 < 0
        beta2 = beta2 + 2 * pi;
    end
    xc = xO2 + Rmin * cos(beta2 + theta_ep);
    yc = yO2 + Rmin * sin(beta2 + theta_ep);
    zc = zb - sqrt(xbc^2 + ybc^2) / k;
    
    figure(1);
    plot([xb, xc], [yb, yc], 'g', 'LineWidth', 1.5);
    hold on
    figure(2);
    plot3([xb, xc], [yb, yc], [zb, zc], 'g', 'LineWidth', 1.5);
    hold on

    zd = zc + Rmin * (-beta2) / k;
    gamma2 = pi / 2 + psia - beta1 - beta2:pi / 100:pi / 2 + psia - beta1;
    x2 = xO2 + Rmin * cos(gamma2);
    y2 = yO2 + Rmin * sin(gamma2);
    z2 = zd + Rmin * (gamma2 - (pi / 2 + psia - beta1 - beta2)) / k;

    figure(1);
    plot(x2, y2, 'g', 'LineWidth', 1.5);
    hold on
    figure(2);
    plot3(x2, y2, z2, 'g', 'LineWidth', 1.5);
    hold on

    beta3 = -(asin(Rmin / (Rep - Rmin)) - theta_ep);
    if beta3 < 0
        beta3 = beta3 + 2 * pi;
    end
    gamma3 = - 2 * tau * pi + pi / 2 + psia - beta1 - beta2 - beta3:pi / 100:pi / 2 + psia - beta1 - beta2;
    x3 = xf + Rep * cos(gamma3);
    y3 = yf + Rep * sin(gamma3);
    z3 = zd + Rep * (gamma3 - (pi / 2 + psia - beta1 - beta2)) / k;

    figure(1);
    plot(x3, y3, 'g', 'LineWidth', 1.5);
    hold on
    figure(2);
    plot3(x3, y3, z3, 'g', 'LineWidth', 1.5);
    hold on

    ze = zd + Rep * (-beta3 - 2 * tau * pi) / k;
    beta4 = asin(Rmin / (Rep - Rmin)) + pi / 2;
    gamma4 = pi / 2 + psia - beta1 - beta2 - beta3 - beta4:pi / 100:pi / 2 + psia - beta1 - beta2 - beta3;
    x4 = xf + ((Rep - Rmin) * cos(beta4 - pi / 2)) + Rmin * cos(gamma4);
    y4 = yf + ((Rep - Rmin) * sin(beta4 - pi / 2)) + Rmin * sin(gamma4);
    z4 = ze + Rmin * (gamma4 - (pi / 2 + psia - beta1 - beta2 - beta3)) / k;

    figure(1);
    plot(x4, y4, 'g', 'LineWidth', 1.5);
    hold on
    figure(2);
    plot3(x4, y4, z4, 'g', 'LineWidth', 1.5);
    hold on

    z5 = ze + Rmin * (-beta4) / k;
    x5 = xf + ((Rep - Rmin) * cos(beta4 - pi / 2)) + Rmin * cos(pi / 2 + psia - beta1 - beta2 - beta3 - beta4);
    y5 = yf + ((Rep - Rmin) * sin(beta4 - pi / 2)) + Rmin * sin(pi / 2 + psia - beta1 - beta2 - beta3 - beta4);
    zg = z5 - sqrt((x5 - xf)^2 + (y5 - yf)^2) / k;

    figure(1);
    plot([x5, xf], [y5, yf], 'g', 'LineWidth', 1.5);
    hold on
    figure(2);
    plot3([x5, xf], [y5, yf], [z5, zg], 'g', 'LineWidth', 1.5);
end