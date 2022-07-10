function [x_dot] = state_space_func(t, x, u, problem)
V = problem.V;
x_dot = zeros(size(x));
x_dot(1) = V * cos(x(5)) * cos(x(4)) + 2 * rand;
x_dot(2) = V * cos(x(5)) * sin(x(4));
x_dot(3) = V * sin(x(5));
x_dot(4) = u(1);
x_dot(5) = u(2);
end