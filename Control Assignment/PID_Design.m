%% EEE4105 Control System-I - PID Controller Design (Car Suspension)
% Design ID: 20220205057  ->  A = 3, B = 4
clc; clear; close all;
s = tf('s');
G = (s+9)/((s+2)*(s+5)*(s+11))            % Plant

%% Step 1: A, B and specifications
A = ceil(5/2)                              % second-last digit = 5
B = ceil(7/2)                              % last digit = 7
OS = B*10                                  % percent overshoot = 40
factor = min(1/A, 0.7)                     % peak-time factor = 1/3

%% Step 2: damping ratio
zeta = -log(OS/100)/sqrt(pi^2 + log(OS/100)^2)
theta_deg = acosd(zeta)

%% Step 3: uncompensated system (root locus point on zeta line)
sx = @(wn) -zeta*wn + 1j*wn*sqrt(1-zeta^2);
g  = @(wn) mod(angle(evalfr(G, sx(wn))), 2*pi) - pi;   % angle condition
wn1 = fzero(g, [10 20])
s1  = sx(wn1)
K   = 1/abs(evalfr(G, s1))                 % magnitude condition
Tp_old = pi/imag(s1)
T_unc = feedback(K*G, 1);
disp('---- Uncompensated system ----'); stepinfo(T_unc)
ess_unc = 1 - dcgain(T_unc)

figure(1); rlocus(G); sgrid(zeta, 0); grid on;
title('Uncompensated Root Locus with \zeta = 0.28 line');

%% Step 4: desired compensated pole
s_new  = s1/factor
Tp_new = pi/imag(s_new)

%% Step 5: PD compensator
phi = mod(pi - angle(evalfr(G, s_new)), 2*pi);  % angle the PD zero must add
phi_deg = rad2deg(phi)
zc = imag(s_new)/tan(phi) - real(s_new)
Kd = 1/abs(evalfr((s+zc)*G, s_new))
T_pd = feedback(Kd*(s+zc)*G, 1);
disp('---- PD compensated system ----'); stepinfo(T_pd)

figure(2); rlocus((s+zc)*G); sgrid(zeta, 0); grid on;
title('PD-Compensated Root Locus');

%% Step 6-7: PI compensator and final PID
zi = 0.1;
Kp = Kd*(zc + zi)
Ki = Kd*zc*zi
C  = pid(Kp, Ki, Kd)
T  = feedback(C*G, 1)
disp('---- PID compensated system ----'); stepinfo(T)
ess_pid = 1 - dcgain(T)
closed_loop_poles = pole(T)

%% Step 8: step response comparison
figure(3); step(T_unc, T_pd, T, 1); grid on;
legend('Uncompensated', 'PD', 'PID');
title('Unit Step Response Comparison');
