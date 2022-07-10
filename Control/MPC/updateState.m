function [t, x] = updateState(x, u, t, problem)
dt = problem.dt;
[~, x] = ode45(@(t, x) state_space_func(t, x, u, problem), [t, t + dt], x);
x = x(end, :)';
t = t + dt;
end

