function [dae] = daeFun(x, u, problem)
% 状态空间方程
V = problem.V;
gamma = problem.gamma;
dae = [
    V * cos(gamma) * cos(x(:, 4));
    V * cos(gamma) * sin(x(:, 4));
    V * sin(gamma) * ones(size(x(:, 3)));
    u;
];
end

