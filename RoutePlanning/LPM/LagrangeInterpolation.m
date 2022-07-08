function [y] = LagrangeInterpolation(x, pointX, pointY)
x = x';
pointX = pointX';
pointY = pointY';
n = size(pointX, 2);
L = ones(n, size(x, 2));
if (size(pointX, 2) ~= size(pointY, 2))
    fprintf(1,'\nERROR!\npointX and pointY must have the same number of elements\n');
    y = NaN;
else
    for i = 1:n
        for j = 1:n
            if (i ~= j)
                L(i, :) = L(i, :) .* (x - pointX(j)) / (pointX(i) - pointX(j));
            end
        end
    end
    y = 0;
    for i = 1:n
        y = y + pointY(i) * L(i, :);
    end
end
y = y';
end

