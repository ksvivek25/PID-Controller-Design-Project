
%% Part (a): Uncompensated System Analysis

% Given open-loop system
s = tf('s');
G = (200) / (s*(s + 6)*(s + 10));  % Uncompensated transfer function
T = feedback(G, 1);         % Closed-loop transfer function

% Transient response analysis
figure;
step(T);
title('Step Response - Uncompensated System');
grid on;

% Calculate performance metrics
S = stepinfo(T);
Kv = dcgain(s*G);    % Velocity error constant using dcgain for s*G

fprintf('\n--- Uncompensated System Performance ---\n');
fprintf('Rise Time: %.4f sec\n', S.RiseTime);
fprintf('Settling Time: %.4f sec\n', S.SettlingTime);
fprintf('Overshoot: %.2f %%\n', S.Overshoot);
fprintf('Kv (Velocity Error Constant): %.4f\n', Kv);

%% Part (b): Root Locus and Desired Pole Selection
figure;
rlocus(G);
title('Root Locus - Uncompensated System');
grid on;

OS = 0.15;
zeta = -log(OS)/sqrt(pi^2 + log(OS)^2);
wn = 4 / (zeta * S.SettlingTime);
wn_new = 2 * wn;

sigma = zeta * wn_new;
wd = wn_new * sqrt(1 - zeta^2);
p_desired = -sigma + 1i*wd;

hold on;
plot(real(p_desired), imag(p_desired), 'rx', 'MarkerSize', 10);

%% Part (c): Lead Compensator Design

% Choose compensator zero and pole based on angle method
C_lead = (s + 4) / (s + 20);   % Adjusted based on angle criterion
G_lead = C_lead * G;

% Verify new root locus
figure;
rlocus(G_lead);
title('Root Locus with Lead Compensator');
grid on;

% Use rlocfind to choose gain placing pole at p_desired
[K_lead, ~] = rlocfind(G_lead);
T_lead = feedback(K_lead * G_lead, 1);

% Lag Compensator Design (10x Kv improvement)
C_lag = (s + 0.05) / (s + 0.005);
C_total = K_lead * C_lead * C_lag;
G_comp = C_total * G;
T_comp = feedback(G_comp, 1);

% Step Response Comparison
figure;
step(T, 'b', T_lead, 'r', T_comp, 'g');
legend('Uncompensated', 'Lead Only', 'Lead-Lag Compensated');
title('Step Response Comparison');
grid on;

% Ramp Response Comparison
Ramp_uncomp = feedback(G/s, 1);
Ramp_comp = feedback(G_comp/s, 1);

figure;
step(Ramp_uncomp, 'b', Ramp_comp, 'g');
legend('Uncompensated', 'Lead-Lag Compensated');
title('Ramp Response Comparison');
grid on;

% Show Final Performance
S_comp = stepinfo(T_comp);
Kv_final = dcgain(s * G_comp);

fprintf('\n--- Compensated System Performance ---\n');
fprintf('Rise Time: %.4f sec\n', S_comp.RiseTime);
fprintf('Settling Time: %.4f sec\n', S_comp.SettlingTime);
fprintf('Overshoot: %.2f %%\n', S_comp.Overshoot);
fprintf('Kv (Final): %.4f\n', Kv_final);
