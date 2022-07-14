function [u] = saturationCons(u, umin, umax)
for i = 1:length(u)
    if u(i) > umax(i)
        u(i) = umax(i);
    end
    if u(i) < umin(i)
        u(i) = umin(i);
    end
end
end