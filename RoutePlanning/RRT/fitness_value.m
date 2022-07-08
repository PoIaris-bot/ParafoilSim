function [y] = fitness_value(x, problem)
xa = problem.xa;
ya = problem.ya;
za = problem.za;
psia = problem.psia;

xf = problem.xf;
yf = problem.yf;
zf = problem.zf;

k = problem.k;
Rmin = problem.Rmin;
tao = problem.tao;

xO1 = xa + Rmin * cos(psia - pi / 2);
yO1 = ya + Rmin * sin(psia - pi / 2);

Rep = x(:, 1);
theta_ep = x(:, 2);
xO2 = xf + (Rep - Rmin) .* cos(theta_ep);
yO2 = yf + (Rep - Rmin) .* sin(theta_ep);

xbc = xO2 - xO1;
ybc = yO2 - yO1;

N = size(x, 1);
angle_bc = zeros(N, 1);

index = find(xbc == 0);
angle_bc(index) = sign(ybc(index)) * pi / 2;
index = find(xbc ~= 0);
angle_bc(index) = ((1 - sign(xbc(index))) / 2) .* sign(ybc(index)) * pi + atan(ybc(index) ./ xbc(index));

beta1 = -(angle_bc - psia);
index = find(beta1 < 0);
if ~isempty(index)
    beta1(index) = beta1(index) + 2 * pi;
end

beta2 = -(theta_ep - angle_bc - pi / 2);
index = find(beta2 < 0);
if ~isempty(index)
    beta2(index) = beta2(index) + 2 * pi;
end

beta3 = -(asin(Rmin ./ (Rep - Rmin)) - theta_ep);
index = find(beta3 < 0);
if ~isempty(index)
    beta3(index) = beta3(index) + 2 * pi;
end

beta4 = asin(Rmin ./ (Rep - Rmin)) + pi / 2;

y = abs(Rmin * (beta1 + beta2 + beta4) + Rep .* (beta3 + 2 * tao * pi) + sqrt(xbc.^2 + ybc.^2) + sqrt((Rep - 2 * Rmin) .* Rep) - (za - zf) * k);
end