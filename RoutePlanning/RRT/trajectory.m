function [trail] = trajectory(P1, P2, P3, X2, k, Step)
x1 = P1.x(1); y1 = P1.x(2);
x2 = P2.x(1); y2 = P2.x(2);
x3 = P3.x(1); y3 = P3.x(2);
xc1 = X2.x(1); yc1 = X2.x(2); zc1 = X2.x(3);
xc2 = (x2 + x3) / 2; yc2 = (y2 + y3) / 2;

if abs((x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)) < 1e-4
    trail.R = inf;
    if (x1 - x2) * (x2 - x3) + (y1 - y2) * (y2 - y3) < 0
        trail.beta = pi;
        return;
    end
    trail.beta = 0;

    x = linspace(xc2, xc1, 100);
    y = linspace(yc2, yc1, 100);
    dz = sqrt((xc1 - xc2)^2 + (yc1 - yc2)^2) / k;
    z = linspace(zc1 - dz, zc1, 100);

    trail.x = x;
    trail.y = y;
    trail.z = z;
else
    A = [
        x1 - x2, y1 - y2;
        x2 - x3, y2 - y3;
        ];

    b = [
        0.5 * (x1^2 - x2^2 + y1^2 - y2^2);
        0.5 * (x2^2 - x3^2 + y2^2 - y3^2);
        ];

    po = A \ b;
    xo = po(1); yo = po(2);

    R = sqrt((xc1 - xo)^2 + (yc1 - yo)^2);

    alpha1 = angle(xo, yo, xc1, yc1);
    alpha2 = angle(xo, yo, xc2, yc2);

    gamma1 = alpha2; gamma2 = alpha1;
    if alpha1 > alpha2
        if alpha1 - alpha2 > pi
            gamma2 = alpha1 - 2 * pi;
        end
    else
        if alpha2 - alpha1 > pi
            gamma1 = alpha2 - 2 * pi;
        end
    end
    gamma = linspace(gamma1, gamma2, 100);
    x = xo + R * cos(gamma);
    y = yo + R * sin(gamma);
    beta = abs(gamma1 - gamma2);
    dz = R * beta / k;
    z = linspace(zc1 - dz, zc1, 100);
    
    trail.beta = beta;
    trail.R = R;
    trail.x = x;
    trail.y = y;
    trail.z = z;
end
end