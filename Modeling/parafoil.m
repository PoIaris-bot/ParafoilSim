function [sys, x0, str, ts, simStateCompliance] = parafoil(t, x, u, flag)
switch flag
  case 0  % Initialization
    [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes;
  case 1  % Derivatives
    sys = mdlDerivatives(t, x, u);
  case 2  % Update
    sys = mdlUpdate(t, x, u);
  case 3  % Outputs
    sys = mdlOutputs(t, x, u);
  case 4  % GetTimeOfNextVarHit
    sys = mdlGetTimeOfNextVarHit(t, x, u);
  case 9  % Terminate
    sys = mdlTerminate(t, x, u);
  otherwise  % Unexpected flags
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%% mdlInitializeSizes
function [sys, x0, str, ts, simStateCompliance] = mdlInitializeSizes
sizes = simsizes;

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0, 0, 1000, 0];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

%% mdlDerivatives
function sys = mdlDerivatives(t, x, u)
% u(1) = delta_s 对称襟翼偏转控制量
% u(2) = delta_a 非对称襟翼偏转控制量
g = 9.8;  % 重力加速度
h0 = 0;

L2D0 = 3.8;
k = 2.15;
L2D = L2D0 - k * u(1);  % 升阻比L/D

rho0 = 1.225;  % 大气密度
rho = @(h) rho0 * (1 - h / 44330).^4.256;
sigma0 = -0.9 * u(2);
sigma = sigma0 * rho(h0) / rho(x(3));

gamma0 = atan(-1 / L2D);
gamma = atan(-1 / (L2D * cos(sigma)));

Veq0 = 12;
V0 = Veq0 * sqrt(rho(h0) / rho(x(3)));
V = V0 * sqrt(cos(gamma) / (cos(sigma) * cos(gamma0)));

wind_flag = 0; 
Wx = 5 * randn * wind_flag;  % 风速x轴分量
Wy = 5 * randn * wind_flag;  % 风速y轴分量

sys = [
    V * cos(gamma) * cos(x(4)) + Wx;
    V * cos(gamma) * sin(x(4)) + Wy;
    V * sin(gamma);
    g / V * tan(sigma);
];

%% mdlUpdate
function sys = mdlUpdate(t, x, u)

sys = [];

%% mdlOutputs
function sys = mdlOutputs(t, x, u)

sys = x;

%% mdlGetTimeOfNextVarHit
function sys = mdlGetTimeOfNextVarHit(t, x, u)

sampleTime = 1;
sys = t + sampleTime;

%% mdlTerminate
function sys = mdlTerminate(t, x, u)

sys = [];
