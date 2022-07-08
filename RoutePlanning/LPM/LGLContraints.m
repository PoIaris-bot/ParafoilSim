function [c, ceq] = LGLContraints(x, problem)
states = reshape(x(problem.indexStates), problem.N + 1, problem.numStates);
inputs = reshape(x(problem.indexInputs), problem.N + 1, problem.numInputs);

tmp1 = problem.D * states;
tmp1 = tmp1(:);

dae = problem.dae(states, inputs, problem);
tmp2 = (problem.tf - problem.t0) / 2 * dae;

tmp3 = tmp2 - tmp1;

x0 = states(1, :);
xf = states(end, :);
tmp4 = problem.event(x0, xf, problem);

c = [];
ceq = [
    tmp3;
    tmp4;
];
end

