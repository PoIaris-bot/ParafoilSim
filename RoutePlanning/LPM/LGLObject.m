function [ret] = LGLObject(x, problem)
states = reshape(x(problem.indexStates), problem.N + 1, problem.numStates);
inputs = reshape(x(problem.indexInputs), problem.N + 1, problem.numInputs);

xf = states(end, :);

[Mayer, Lagrange] = problem.cost(xf, problem.tf, states, inputs, problem);
tmp = (problem.tf - problem.t0) / 2 * problem.weights' * Lagrange;
ret = tmp + Mayer;
end

