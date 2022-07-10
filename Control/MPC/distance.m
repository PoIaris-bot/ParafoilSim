function [dist] = distance(x, path)
n = size(path, 2);
dist = zeros(n, 1);

for i = 1:n
    dist(i) = sqrt((x(1:3) - path(1:3, i))' * (x(1:3) - path(1:3, i)));
end
end

