function [points] = LGLPoints(N)
% Compute the initial guess of the interior LGL points
theta = (4 * (1:N) - 1) * pi / (4 * N + 2);
sigma = -(1 - (N - 1) / (8 * N^3) - (39 - 28 ./ sin(theta).^2) / (384 * N^4)) .* cos(theta);
ze = (sigma(1:N - 1) + sigma(2:N)) / 2;
ep = eps * 10;  % error tolerance for stopping iteration
ze1 = ze + ep + 1;

while max(abs(ze1 - ze)) >= ep  % Newton's iteration procedure
    ze1 = ze;
    [dy, y] = lepoly(N, ze);
    ze = ze - (1 - ze .* ze) .* dy ./ (2 * ze .* dy - N * (N + 1) * y);
end

points = [-1, ze, 1]';  % column vector
end

