function visualize(problem)
xa = problem.x0;
ya = problem.y0;
za = problem.z0;
psi0 = problem.psi0;

xf = problem.xf;
yf = problem.yf;

k = problem.k;
Rmin = problem.Rmin;
tao = problem.tao;

Rep = problem.Rep;
theta_ep = problem.theta_ep;

xO1 = xa + Rmin * cos(psi0 - pi / 2);
yO1 = ya + Rmin * sin(psi0 - pi / 2);

xO2 = xf + (Rep - Rmin) * cos(theta_ep);
yO2 = yf + (Rep - Rmin) * sin(theta_ep);

xbc = xO2 - xO1;
ybc = yO2 - yO1;

if xbc == 0
    angle_bc = sign(ybc) * pi / 2;
else
    angle_bc = ((1 - sign(xbc)) / 2) .* sign(ybc) * pi + atan(ybc / xbc);
end

beta1 = -(angle_bc - psi0);
if beta1 < 0
    beta1 = beta1 + 2 * pi;
end

gamma1 = psi0 + pi / 2 - beta1:pi / 100:psi0 + pi / 2;
x1 = xO1 + Rmin * cos(gamma1);
y1 = yO1 + Rmin * sin(gamma1);
z1 = za + Rmin * (gamma1 - (psi0 + pi / 2)) / k;
plot3(x1, y1, z1, 'g', 'LineWidth', 1.5);
axis equal; grid on;
hold on

xb = xO1 + Rmin * cos(psi0 + pi / 2 - beta1);
yb = yO1 + Rmin * sin(psi0 + pi / 2 - beta1);
zb = za + Rmin * (-beta1) / k;

beta2 = -(theta_ep - angle_bc - pi / 2);
if beta2 < 0
    beta2 = beta2 + 2 * pi;
end
xc = xO2 + Rmin * cos(beta2 + theta_ep);
yc = yO2 + Rmin * sin(beta2 + theta_ep);
zc = zb - sqrt(xbc^2 + ybc^2) / k;

plot3([xb, xc], [yb, yc], [zb, zc], 'g', 'LineWidth', 1.5);
hold on

zd = zc + Rmin * (-beta2) / k;
gamma2 = pi / 2 + psi0 - beta1 - beta2:pi / 100:pi / 2 + psi0 - beta1;
x2 = xO2 + Rmin * cos(gamma2);
y2 = yO2 + Rmin * sin(gamma2);
z2 = zd + Rmin * (gamma2 - (pi / 2 + psi0 - beta1 - beta2)) / k;
plot3(x2, y2, z2, 'g', 'LineWidth', 1.5);
hold on

beta3 = -(asin(Rmin / (Rep - Rmin)) - theta_ep);
if beta3 < 0
    beta3 = beta3 + 2 * pi;
end
gamma3 = - 2 * tao * pi + pi / 2 + psi0 - beta1 - beta2 - beta3:pi / 100:pi / 2 + psi0 - beta1 - beta2;
x3 = xf + Rep * cos(gamma3);
y3 = yf + Rep * sin(gamma3);
z3 = zd + Rep * (gamma3 - (pi / 2 + psi0 - beta1 - beta2)) / k;
plot3(x3, y3, z3, 'g', 'LineWidth', 1.5);
hold on

ze = zd + Rep * (-beta3 - 2 * tao * pi) / k;
beta4 = asin(Rmin / (Rep - Rmin)) + pi / 2;
gamma4 = pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4:pi / 100:pi / 2 + psi0 - beta1 - beta2 - beta3;
x4 = xf + ((Rep - Rmin) * cos(beta4 - pi / 2)) + Rmin * cos(gamma4);
y4 = yf + ((Rep - Rmin) * sin(beta4 - pi / 2)) + Rmin * sin(gamma4);
z4 = ze + Rmin * (gamma4 - (pi / 2 + psi0 - beta1 - beta2 - beta3)) / k;
plot3(x4, y4, z4, 'g', 'LineWidth', 1.5);
hold on

z5 = ze + Rmin * (-beta4) / k;
x5 = xf + ((Rep - Rmin) * cos(beta4 - pi / 2)) + Rmin * cos(pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4);
y5 = yf + ((Rep - Rmin) * sin(beta4 - pi / 2)) + Rmin * sin(pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4);
zg = z5 - sqrt((x5 - xf)^2 + (y5 - yf)^2) / k;
plot3([x5, xf], [y5, yf], [z5, zg], 'g', 'LineWidth', 1.5);
end

