function [weights] = LGLWeights(points)
N = length(points) - 1;
[~, y] = lepoly(N, points(2:end - 1));
weights = [
    2 / (N * (N + 1));
    2 ./ (N * (N + 1) * y.^2);
    2 / (N * (N + 1))
]; 
end

