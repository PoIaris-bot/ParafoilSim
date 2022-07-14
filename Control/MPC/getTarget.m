function [xr, idx] = getTarget(x, route)
index = find(route(3, :) < x(3));
if isempty(index)
    idx = size(route, 2);
else
    dist = distance(x, route(:, index));
    [~, idx] = min(dist);
    idx = index(idx);
end
xr = route(:, idx);
end