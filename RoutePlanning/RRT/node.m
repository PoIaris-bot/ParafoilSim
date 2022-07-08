function [P] = node(x, id, parent)
P.x = x;
if nargin > 1
    P.id = id;
end
if nargin > 2
    P.parent = parent;
end
end
