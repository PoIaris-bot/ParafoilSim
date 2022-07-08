clear; clc;
%% 参数设置
problem.N = 20;
problem.x0 = [1200, 600, 2000, pi / 3];
problem.xf = [0, 0, 0, pi];
problem.V = 20;
k = 3;
problem.gamma = -atan(1 / k);

problem.k1 = 0.01;
problem.k2 = 20;
problem.k3 = 1e8;
%
problem.xmin = [];
problem.xmax = [];
problem.umin = -0.2;
problem.umax = 0.2;

problem.t0 = 0;
problem.tf = problem.x0(3) / abs(problem.V * sin(problem.gamma));

problem.numStates = 4;  % 状态变量
problem.numInputs = 1;  % 控制变量

problem.numVariable = (problem.N + 1) * (problem.numStates + problem.numInputs);
problem.indexStates = 1:(problem.N + 1) * problem.numStates;
problem.indexInputs = (problem.N + 1) * problem.numStates + 1:problem.numVariable;

problem.points = LGLPoints(problem.N);
problem.weights = LGLWeights(problem.points);
problem.D = LGLDMatrix(problem.points);
problem.cost = @costFun;
problem.dae = @daeFun;
problem.event = @eventFun;

x0 = [
    rand((problem.N + 1) * problem.numStates, 1);
    zeros((problem.N + 1) * problem.numInputs, 1)
];

optNLP = optimset( ...
    'LargeScale', 'off', ...
    'GradObj','off', ...
    'GradConstr','off', ...
    'DerivativeCheck', 'off', ...
    'Display', 'iter', ...
    'TolX', 1e-9, ...
    'TolFun', 1e-6, ...
    'TolCon', 1e-6, ...
    'MaxFunEvals', 1e4, ...
    'DiffMinChange', 1e-5, ...
    'Algorithm', 'interior-point' ...
 );
tic;
A = []; b = [];
Aeq = []; beq = [];
lb = []; ub = [];

[x, fval, exitflag, output, lambda, grad, hessian] = fmincon( ...
    @(x) LGLObject(x, problem), x0, A, b, Aeq, beq, lb, ub, @(x) LGLContraints(x, problem), optNLP ...
);
toc;

%% 
states = reshape(x(problem.indexStates), problem.N + 1, problem.numStates);
inputs = reshape(x(problem.indexInputs), problem.N + 1, problem.numInputs);
N = 300;
pointsPlot = linspace(-1, 1, N)';
statesPlot = zeros(N, problem.numStates);
for i = 1:problem.numStates
    currentState = states(:, i);
    currentStatePlot = LagrangeInterpolation(pointsPlot, problem.points, currentState);
    statesPlot(:, i) = currentStatePlot;
end

figure(1);
plot3(statesPlot(:, 1), statesPlot(:, 2), statesPlot(:, 3));
grid on;
xlabel('x', 'FontSize', 15); ylabel('y', 'FontSize', 15); zlabel('h', 'FontSize', 15);
axis equal;

timePlot = 0.5 * (problem.tf - problem.t0) * pointsPlot + 0.5 * (problem.tf + problem.t0);
figure(2);
plot(timePlot, statesPlot(:, 4));
xlabel('t', 'FontSize', 15);
ylabel('$\psi(rad)$', 'Interpreter', 'latex', 'FontSize', 15);

pointsPlot = 0.5 * (problem.tf - problem.t0) * problem.points + 0.5 * (problem.tf + problem.t0);
figure(3)
plot(pointsPlot, inputs);
xlabel('t', 'FontSize', 15);
ylabel('u', 'FontSize', 15);

