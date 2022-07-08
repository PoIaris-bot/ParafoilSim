function [D] = LGLDMatrix(points)
N = length(points) - 1;
n = N + 1;
if n == 0
    D = [];
    return;
end
x = points;
y = lepoly(n - 1, x);
D = (x ./ y) * y' - (1 ./ y) * (x .* y)';  % compute L_{n - 1}(x_j)(x_k - x_j) / L_{n - 1}(x_k);
% 1 / d_{kj} for k != j
D = D + eye(n);  % add the identity matrix so that 1 ./ D can be operated
D = 1 ./ D;
D = D - eye(n);
D(1, 1) = -n * (n - 1) / 4;
D(n, n) = -D(1, 1);
end
