function [flag] = is_feasible(trail, height_margin, beta_max, peaks)
flag = true;
if trail.beta > beta_max
    flag = false;
    return;
end
x = trail.x; y = trail.y; z = trail.z;
for i = 1:length(x)
    h = height(x(i), y(i), peaks);
    if z(i) - h < height_margin
        flag = false;
        return;
    end
end
end
