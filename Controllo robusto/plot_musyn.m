%% === ANALISI CONTROLLORE MU-SYNTHESIS ===

%% 1. ESTRAZIONE DATI DAL WORKSPACE
t = out.zr_musyn.Time; 

% --- Profilo Stradale ---
zr = squeeze(out.zr_musyn.Data); zr = zr(:); 

% --- Dati Modello Non Lineare (K_musyn) ---
zs_ddot_mu = squeeze(out.zs_ddot_musyn.Data); zs_ddot_mu = zs_ddot_mu(:);
delta_s_mu = squeeze(out.delta_s_musyn.Data); delta_s_mu = delta_s_mu(:);
delta_t_mu = squeeze(out.delta_t_musyn.Data); delta_t_mu = delta_t_mu(:);
u1_mu      = squeeze(out.u1__musyn.Data);      u1_mu = u1_mu(:);
u2_mu      = squeeze(out.u2__musyn.Data);      u2_mu = u2_mu(:);

% --- Dati Modello Lineare (K_musyn) ---
zs_ddot_lin = squeeze(out.zs_ddot_lin.Data);  zs_ddot_lin = zs_ddot_lin(:);
delta_s_lin = squeeze(out.delta_s_lin.Data);  delta_s_lin = delta_s_lin(:);
delta_t_lin = squeeze(out.delta_t_lin.Data);  delta_t_lin = delta_t_lin(:);
u1_lin      = squeeze(out.u1_lin.Data);       u1_lin = u1_lin(:);
u2_lin      = squeeze(out.u2_lin.Data);       u2_lin = u2_lin(:);

step_time = params.t0;  
step_amp  = params.A;   

%% 2. CALCOLO METRICHE (Solo Mu-Synthesis)
zs_mu  = zr + delta_t_mu + delta_s_mu;
zu_mu  = zr + delta_t_mu;

idx_step = find(t >= step_time);
t_eval   = t(idx_step);
zs_eval  = zs_mu(idx_step);

% Errore a Regime
zs_ss_val = mean(zs_mu(end-50:end)); 
e_ss   = abs(step_amp - zs_ss_val);

% Sovraelongazione
max_zs = max(zs_eval);
overshoot = 0;
if max_zs > step_amp
    overshoot = ((max_zs - step_amp) / step_amp) * 100;
end

% Tempo di Assestamento (5%)
banda = 0.05 * step_amp;
idx_out = find((zs_eval < (step_amp - banda)) | (zs_eval > (step_amp + banda)), 1, 'last');
if isempty(idx_out)
    t_assest = 0;
else
    t_assest = t_eval(idx_out) - step_time;
end

max_acc = max(abs(zs_ddot_mu)); rms_acc = sqrt(mean(zs_ddot_mu.^2));
max_u1  = max(abs(u1_mu));      rms_u1  = sqrt(mean(u1_mu.^2));
max_u2  = max(abs(u2_mu));      rms_u2  = sqrt(mean(u2_mu.^2));

%% 3. STAMPA TABELLA
fprintf('\n=========================================================\n');
fprintf('       PRESTAZIONI CONTROLLORE MU-SYNTHESIS (NON-LINEARE)    \n');
fprintf('=========================================================\n');
fprintf('Tempo di Assestamento [s]    | %14.3f \n', t_assest);
fprintf('Sovraelongazione [%%]         | %14.2f \n', overshoot);
fprintf('Errore a Regime [m]          | %14.4f \n', e_ss);
fprintf('-----------------------------|----------------\n');
fprintf('Picco Accelerazione [m/s^2]  | %14.2f \n', max_acc);
fprintf('RMS Accelerazione [m/s^2]    | %14.2f \n', rms_acc);
fprintf('-----------------------------|----------------\n');
fprintf('Picco Sforzo U1 [N]          | %14.1f \n', max_u1);
fprintf('Picco Sforzo U2 [N]          | %14.1f \n', max_u2);
fprintf('=========================================================\n');

%% 4. PLOT GRAFICI
% Tracking Posizioni Assolute
figure('Name', 'Inseguimento Disturbo (musyn)', 'Position', [50, 50, 600, 400]);
plot(t, zr, 'k--', 'LineWidth', 1.5); hold on;
plot(t, zu_mu, 'b-', 'LineWidth', 1.2);
plot(t, zs_mu, 'r-', 'LineWidth', 2);
xlabel('Tempo [s]'); ylabel('Posizione [m]');
title('Inseguimento Disturbo (\mu-synthesis): Posizioni Assolute');
legend('Strada (z_r)', 'Ruota (z_u)', 'Cassa (z_s)', 'Location', 'best'); grid on;

% Lineare vs Non-Lineare
figure('Name', 'Lineare vs Non-Lineare (musyn)', 'Position', [100, 100, 800, 600]);
subplot(3,1,1);
plot(t, zs_ddot_lin, 'b--', 'LineWidth', 1.5); hold on;
plot(t, zs_ddot_mu, 'r', 'LineWidth', 1.2);
title('Accelerazione Scocca (Comfort)'); ylabel('m/s^2'); grid on;
legend('Lineare', 'Non-Lineare', 'Location', 'best');
subplot(3,1,2);
plot(t, delta_s_lin, 'b--', 'LineWidth', 1.5); hold on;
plot(t, delta_s_mu, 'r', 'LineWidth', 1.2);
title('Corsa Sospensione (\delta_s)'); ylabel('m'); grid on;
subplot(3,1,3);
plot(t, u1_lin, 'b--', 'LineWidth', 1.5); hold on;
plot(t, u1_mu, 'r', 'LineWidth', 1.2);
title('Sforzo Attuatore 1'); ylabel('N'); xlabel('Tempo [s]'); grid on;