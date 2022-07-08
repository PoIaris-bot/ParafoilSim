function [z] = height(x, y, peaks)
xp = peaks.x;
yp = peaks.y;
hp = peaks.h;
xs = peaks.xs;
ys = peaks.ys;
z = 0;
for n = 1:length(hp)
    z = z + hp(n) * exp(-((x - xp(n)) / xs(n))^2 - ((y - yp(n)) / ys(n))^2);
end
end