function [dist] = distance(P, tree_nodes)
n = length(tree_nodes);
dist = zeros(n);

for i = 1:n
    dist(i) = sqrt((P.x - tree_nodes(i).x)' * (P.x - tree_nodes(i).x));
end
end

