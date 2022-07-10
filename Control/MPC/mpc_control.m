function [u, idx] = mpc_control(x, route, u, problem)
V = problem.V;
dt = problem.dt;
numStates = problem.numStates;
numInputs = problem.numInputs;
numOutputs = problem.numOutputs;
umin = problem.umin;
umax = problem.umax;
delta_umin = problem.delta_umin;
delta_umax = problem.delta_umax;
P = problem.P;  % 预测时域
M = problem.M;  % 控制时域
rho = problem.rho;

Q = eye(P * numOutputs);
R = eye(M * numInputs);

dist = distance(x, route);
[~, idx] = min(dist);
xr = route(:, idx);
xi = [
    x - xr;
    u;
];

psi = x(4);
gamma = x(5);
a = [
    1, 0, 0, -dt * V * cos(gamma) * sin(psi), -dt * V * sin(gamma) * cos(psi);
    0, 1, 0, dt * V * cos(gamma) * cos(psi), -dt * V * sin(gamma) * sin(psi);
    0, 0, 1, 0, dt * V * cos(gamma);
    0, 0, 0, 1, 0;
    0, 0, 0, 0, 1;
];
b = [
    0, 0;
    0, 0;
    0, 0;
    dt, 0;
    0, dt;
];

A = [
    a, b;
    zeros(numInputs, numStates), eye(numInputs);
];

B = [
    b;
    eye(numInputs);
];

C = [eye(numOutputs), zeros(numOutputs, numStates - numOutputs + numInputs)];

PHI_cell = cell(P, 1);
for i = 1:P
    PHI_cell{i, 1} = C * A^i;
end
PHI = cell2mat(PHI_cell);

THETA_cell = cell(P, M);
for i = 1:P
    for j = 1:M
        if j <= i
            THETA_cell{i, j} = C * A^(i - j) * B;
        else
            THETA_cell{i, j} = zeros(numOutputs, numInputs);
        end
    end
end
THETA = cell2mat(THETA_cell);

H = [
    THETA' * Q * THETA + R, zeros(numInputs * M, 1);
    zeros(1, numInputs * M), rho;
];

E = PHI * xi;
g = [E' * Q * THETA, 0];

At = zeros(M);
for i = 1:M
    At(i, 1:i) = 1;
end
AI = kron(At, eye(numInputs));

Ut = kron(ones(M, 1), u);
Umin = kron(ones(M, 1), umin);
Umax = kron(ones(M, 1), umax);
delta_Umin = kron(ones(M, 1), delta_umin);
delta_Umax = kron(ones(M, 1), delta_umax);
Acons = [
    AI, zeros(numInputs * M, 1);
    -AI, zeros(numInputs * M, 1);
];
bcons = [
    Umax - Ut;
    -Umin + Ut;
];

lb = [delta_Umin; 0];
ub = [delta_Umax; 1];

options = optimoptions('quadprog', 'Display', 'final', 'MaxIterations', 100, 'TolFun', 1e-8);
delta_U = quadprog(H, g, Acons, bcons, [], [], lb, ub, [], options);
% delta_U = quadprog(H, g, [], [], [], [], [], [], [], [], options);
delta_u = delta_U(1:numInputs);

u = delta_u + xi(numStates + 1:end);
end
