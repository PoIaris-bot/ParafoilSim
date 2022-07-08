function visualize(problem)
x0 = problem.x0;
y0 = problem.y0;
h0 = problem.h0;
psi0 = problem.psi0;
k = problem.k;
Rmin = problem.Rmin;
Rep = problem.Rep;
theta_ep = problem.theta_ep;

xO1 = x0 + Rmin * cos(psi0 - pi / 2);
yO1 = y0 + Rmin * sin(psi0 - pi / 2);

xO2 = (Rep - Rmin) * cos(theta_ep);
yO2 = (Rep - Rmin) * sin(theta_ep);

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

n1 = beta1 * Rmin / k / 10;
gamma1 = linspace(psi0 + pi / 2 - beta1, psi0 + pi / 2, n1);
% gamma1 = psi0 + pi / 2 - beta1:pi / 50:psi0 + pi / 2;
x1 = xO1 + Rmin * cos(gamma1);
y1 = yO1 + Rmin * sin(gamma1);
h1 = h0 + Rmin * (gamma1 - (psi0 + pi / 2)) / k;
plot3(x1, y1, h1);
axis equal; grid on;
hold on

xb = xO1 + Rmin * cos(psi0 + pi / 2 - beta1);
yb = yO1 + Rmin * sin(psi0 + pi / 2 - beta1);
hb = h0 + Rmin * (-beta1) / k;

beta2 = -(theta_ep - angle_bc - pi / 2);
if beta2 < 0
    beta2 = beta2 + 2 * pi;
end
xc = xO2 + Rmin * cos(beta2 + theta_ep);
yc = yO2 + Rmin * sin(beta2 + theta_ep);
hc = hb - sqrt(xbc^2 + ybc^2) / k;
n2 = (hb - hc) / 10;
plot3(linspace(xb, xc, n2), linspace(yb, yc, n2), linspace(hb, hc, n2));
hold on

hd = hc + Rmin * (-beta2) / k;
n3 = Rmin * beta2 / k / 10;
gamma2 = linspace(pi / 2 + psi0 - beta1 - beta2, pi / 2 + psi0 - beta1, n3);
% gamma2 = pi / 2 + psi0 - beta1 - beta2:pi / 50:pi / 2 + psi0 - beta1;
x2 = xO2 + Rmin * cos(gamma2);
y2 = yO2 + Rmin * sin(gamma2);
h2 = hd + Rmin * (gamma2 - (pi / 2 + psi0 - beta1 - beta2)) / k;
plot3(x2, y2, h2);
hold on

beta3 = -(asin(Rmin / (Rep - Rmin)) - theta_ep);
if beta3 < 0
    beta3 = beta3 + 2 * pi;
end
n4 = Rep * beta3 / k / 10;
gamma3 = linspace(pi / 2 + psi0 - beta1 - beta2 - beta3, pi / 2 + psi0 - beta1 - beta2, n4);
% gamma3 = pi / 2 + psi0 - beta1 - beta2 - beta3:pi / 50:pi / 2 + psi0 - beta1 - beta2;
x3 = Rep * cos(gamma3);
y3 = Rep * sin(gamma3);
h3 = hd + Rep * (gamma3 - (pi / 2 + psi0 - beta1 - beta2)) / k;
plot3(x3, y3, h3);
hold on

he = hd + Rep * (-beta3) / k;
beta4 = asin(Rmin / (Rep - Rmin)) + pi / 2;
n5 = Rmin * beta4 / k / 10;
gamma4 = linspace(pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4, pi / 2 + psi0 - beta1 - beta2 - beta3, n5);
% gamma4 = pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4:pi / 50:pi / 2 + psi0 - beta1 - beta2 - beta3;
x4 = ((Rep - Rmin) * cos(beta4 - pi / 2)) + Rmin * cos(gamma4);
y4 = ((Rep - Rmin) * sin(beta4 - pi / 2)) + Rmin * sin(gamma4);
h4 = he + Rmin * (gamma4 - (pi / 2 + psi0 - beta1 - beta2 - beta3)) / k;
plot3(x4, y4, h4);
hold on

hf = he + Rmin * (-beta4) / k;
xf = ((Rep - Rmin) * cos(beta4 - pi / 2)) + Rmin * cos(pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4);
yf = ((Rep - Rmin) * sin(beta4 - pi / 2)) + Rmin * sin(pi / 2 + psi0 - beta1 - beta2 - beta3 - beta4);
hg = hf - sqrt(xf^2 + yf^2) / k;
n6 = (hf - hg) / 10;
plot3(linspace(xf, 0, n6), linspace(yf, 0, n6), linspace(hf, hg, n6));
end

