function [mayer, lagrange] = costFun(xf, tf, x, u, problem)
% 目标函数
k1 = problem.k1;
k2 = problem.k2;
k3 = problem.k3;
J1 = xf(1)^2 + xf(2)^2 + xf(3)^2;
J2 = cos(xf(4)) + 1;
J3 = u.^2;
mayer = k1 * J1 + k2 * J2;
lagrange = k3 * J3;
end
