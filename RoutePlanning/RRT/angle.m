function [theta] = angle(x1, y1, x2, y2)
v = [x2 - x1, y2 - y1];
theta = real(acos(dot(v, [1, 0]) / norm(v)));
if v(2) < 0
    theta = 2 * pi - theta;
end
end