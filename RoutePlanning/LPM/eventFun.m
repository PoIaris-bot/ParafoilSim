function [ret] = eventFun(x0, xf, problem)
% 边界条件
ret = [
    x0' - problem.x0';
    xf' - problem.xf';
];
end

