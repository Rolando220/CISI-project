%% === CONFRONTO CONTROLLORI ===

%% 1. ESTRAZIONE DATI DAL WORKSPACE (Timeseries)
% Estrazione vettore tempo
t = out.zr_mixsyn.Time; % Assicurati che il nome corrisponda a quello del tuo blocco strada

% --- Profilo Stradale ---
zr = squeeze(out.zr_mixsyn.Data); 
zr = zr(:); 

% --- Dati Modello Non Lineare (K_mixsyn) ---
zs_ddot_mix = squeeze(out.zs_ddot_mixsyn.Data); zs_ddot_mix = zs_ddot_mix(:);
delta_s_mix = squeeze(out.delta_s_mixsyn.Data); delta_s_mix = delta_s_mix(:);
delta_t_mix = squeeze(out.delta_t_mixsyn.Data); delta_t_mix = delta_t_mix(:);
u1_mix      = squeeze(out.u1_mixsyn.Data);      u1_mix = u1_mix(:);
u2_mix      = squeeze(out.u2_mixsyn.Data);      u2_mix = u2_mix(:);

% --- Dati Modello Lineare (K_mixsyn) ---
zs_ddot_lin = squeeze(out.zs_ddot_lin.Data);    zs_ddot_lin = zs_ddot_lin(:);
delta_s_lin = squeeze(out.delta_s_lin.Data);    delta_s_lin = delta_s_lin(:);
delta_t_lin = squeeze(out.delta_t_lin.Data);    delta_t_lin = delta_t_lin(:);
u1_lin      = squeeze(out.u1_lin.Data);         u1_lin = u1_lin(:);
u2_lin      = squeeze(out.u2_lin.Data);         u2_lin = u2_lin(:);

% --- Dati Modello Non Lineare (K_hinfsyn) ---
zs_ddot_hinf = squeeze(out.zs_ddot_hinf.Data);  zs_ddot_hinf = zs_ddot_hinf(:);
delta_s_hinf = squeeze(out.delta_s_hinf.Data);  delta_s_hinf = delta_s_hinf(:);
delta_t_hinf = squeeze(out.delta_t_hinf.Data);  delta_t_hinf = delta_t_hinf(:);
u1_hinf      = squeeze(out.u1__hinf.Data);      u1_hinf = u1_hinf(:);
u2_hinf      = squeeze(out.u2__hinf.Data);      u2_hinf = u2_hinf(:);

% --- Dati Modello Non Lineare (PID) ---
zs_ddot_pid = squeeze(out.zs_ddot_PID.Data);    zs_ddot_pid = zs_ddot_pid(:);
delta_s_pid = squeeze(out.delta_s_PID.Data);    delta_s_pid = delta_s_pid(:);
delta_t_pid = squeeze(out.delta_t_PID.Data);    delta_t_pid = delta_t_pid(:);
u1_pid      = squeeze(out.u1_PID.Data);         u1_pid = u1_pid(:);
u2_pid      = squeeze(out.u2_PID.Data);         u2_pid = u2_pid(:);

% Parametri del gradino per calcolo metriche
step_time = params.t0;  % Istante in cui inizia il gradino [s]
step_amp  = params.A;   % Ampiezza del gradino stradale [m]

% %% 2. CALCOLO DELLA CINEMATICA ASSOLUTA (Sul modello Non Lineare Ottimo)
% zu_mix = zr + delta_t_mix;       % Posizione assoluta ruota
% zs_mix = zu_mix + delta_s_mix;   % Posizione assoluta carrozzeria

%% 3. CALCOLO DELLA CINEMATICA ASSOLUTA E METRICHE
% Ricostruzione posizione assoluta cassa (zs = zr + delta_t + delta_s)
zs_mix  = zr + delta_t_mix + delta_s_mix;
zs_hinf = zr + delta_t_hinf + delta_s_hinf;
zs_pid  = zr + delta_t_pid + delta_s_pid;

% Array per automatizzare i calcoli: 1=mixsyn, 2=hinfsyn, 3=PID
zs_all      = [zs_mix, zs_hinf, zs_pid];
zs_ddot_all = [zs_ddot_mix, zs_ddot_hinf, zs_ddot_pid];
u1_all      = [u1_mix, u1_hinf, u1_pid];
u2_all      = [u2_mix, u2_hinf, u2_pid]; % Aggiunto u2

% Inizializzazione vettori risultati
t_assest = zeros(1,3); overshoot = zeros(1,3); e_ss = zeros(1,3);
max_acc = zeros(1,3);  rms_acc = zeros(1,3);
max_u1 = zeros(1,3);   rms_u1 = zeros(1,3);
max_u2 = zeros(1,3);   rms_u2 = zeros(1,3);    % Inizializzazione u2

idx_step = find(t >= step_time);
t_eval   = t(idx_step);

for i = 1:3
    zs_curr = zs_all(:, i);
    zs_eval = zs_curr(idx_step);
    
    % A. Errore a Regime
    zs_ss_val = mean(zs_curr(end-50:end)); 
    e_ss(i)   = abs(step_amp - zs_ss_val);
    
    % B. Sovraelongazione (Overshoot)
    max_zs = max(zs_eval);
    if max_zs > step_amp
        overshoot(i) = ((max_zs - step_amp) / step_amp) * 100;
    else
        overshoot(i) = 0; 
    end
    
    % C. Tempo di Assestamento (5%)
    banda = 0.05 * step_amp;
    lim_sup = step_amp + banda;
    lim_inf = step_amp - banda;
    idx_out = find((zs_eval < lim_inf) | (zs_eval > lim_sup), 1, 'last');
    if isempty(idx_out)
        t_assest(i) = 0;
    else
        t_assest(i) = t_eval(idx_out) - step_time;
    end
    
    % D. Metriche Automotive (Comfort ed Energia per U1 e U2)
    max_acc(i) = max(abs(zs_ddot_all(:, i)));
    rms_acc(i) = sqrt(mean(zs_ddot_all(:, i).^2));
    
    max_u1(i)  = max(abs(u1_all(:, i)));
    rms_u1(i)  = sqrt(mean(u1_all(:, i).^2));
    
    max_u2(i)  = max(abs(u2_all(:, i)));
    rms_u2(i)  = sqrt(mean(u2_all(:, i).^2));
end

%% 4. STAMPA TABELLA RIASSUNTIVA A SCHERMO
fprintf('\n=======================================================================================\n');
fprintf('                CONFRONTO PRESTAZIONI CONTROLLORI (MODELLO NON-LINEARE)                \n');
fprintf('=======================================================================================\n');
fprintf('METRICA                      | H-inf (mixsyn) | H-inf (hinfsyn) | PI Strutturato |\n');
fprintf('-----------------------------|----------------|-----------------|-----------------|\n');
fprintf('Tempo di Assestamento [s]    | %14.3f | %15.3f | %15.3f |\n', t_assest(1), t_assest(2), t_assest(3));
fprintf('Sovraelongazione [%%]         | %14.2f | %15.2f | %15.2f |\n', overshoot(1), overshoot(2), overshoot(3));
fprintf('Errore a Regime [m]          | %14.4f | %15.4f | %15.4f |\n', e_ss(1), e_ss(2), e_ss(3));
fprintf('-----------------------------|----------------|-----------------|-----------------|\n');
fprintf('Picco Accelerazione [m/s^2]  | %14.2f | %15.2f | %15.2f |\n', max_acc(1), max_acc(2), max_acc(3));
fprintf('RMS Accelerazione [m/s^2]    | %14.2f | %15.2f | %15.2f |\n', rms_acc(1), rms_acc(2), rms_acc(3));
fprintf('-----------------------------|----------------|-----------------|-----------------|\n');
fprintf('Picco Sforzo (U1) [N]        | %14.1f | %15.1f | %15.1f |\n', max_u1(1), max_u1(2), max_u1(3));
fprintf('RMS Sforzo U1 (Energia) [N]  | %14.1f | %15.1f | %15.1f |\n', rms_u1(1), rms_u1(2), rms_u1(3));
fprintf('-----------------------------|----------------|-----------------|-----------------|\n');
fprintf('Picco Sforzo (U2) [N]        | %14.1f | %15.1f | %15.1f |\n', max_u2(1), max_u2(2), max_u2(3));
fprintf('RMS Sforzo U2 (Energia) [N]  | %14.1f | %15.1f | %15.1f |\n', rms_u2(1), rms_u2(2), rms_u2(3));
fprintf('=======================================================================================\n');

%% 5. PLOT GRAFICI

% --- PLOT TRACKING (Posizioni Assolute) ---
figure('Name', 'Inseguimento del Riferimento', 'Position', [50, 50, 600, 400]);
plot(t, zr, 'k--', 'LineWidth', 1.5); hold on;
plot(t, zu_mix, 'b-', 'LineWidth', 1.2);
plot(t, zs_mix, 'r-', 'LineWidth', 2);
xlabel('Tempo [s]'); ylabel('Posizione [m]');
title('Inseguimento Disturbo (mixsyn): Posizioni Assolute');
legend('Strada (z_r)', 'Ruota (z_u)', 'Cassa (z_s)', 'Location', 'best'); grid on;

% --- 1) CONFRONTO: Modello Lineare vs Non-Lineare (K_mixsyn) ---
figure('Name', '1. Lineare vs Non-Lineare (mixsyn)', 'Position', [100, 100, 800, 600]);
subplot(3,1,1);
plot(t, zs_ddot_lin, 'b--', 'LineWidth', 1.5); hold on;
plot(t, zs_ddot_mix, 'r', 'LineWidth', 1.2);
title('Accelerazione Scocca (Comfort)'); ylabel('m/s^2'); grid on;
legend('Lineare', 'Non-Lineare', 'Location', 'best');

subplot(3,1,2);
plot(t, delta_s_lin, 'b--', 'LineWidth', 1.5); hold on;
plot(t, delta_s_mix, 'r', 'LineWidth', 1.2);
title('Corsa Sospensione (\delta_s)'); ylabel('m'); grid on;

subplot(3,1,3);
plot(t, u1_lin, 'b--', 'LineWidth', 1.5); hold on;
plot(t, u1_mix, 'r', 'LineWidth', 1.2);
title('Sforzo Attuatore 1'); ylabel('N'); xlabel('Tempo [s]'); grid on;

% --- 2) CONFRONTO: K_mixsyn vs K_hinf (Entrambi Non-Lineari) ---
figure('Name', '2. mixsyn vs hinfsyn (Non-Lineare)', 'Position', [150, 150, 800, 600]);
subplot(3,1,1);
plot(t, zs_ddot_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, zs_ddot_hinf, 'k--', 'LineWidth', 1.5);
title('Accelerazione Scocca (Comfort)'); ylabel('m/s^2'); grid on;
legend('H-\infty (mixsyn)', 'H-\infty (hinfsyn)', 'Location', 'best');

subplot(3,1,2);
plot(t, delta_s_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, delta_s_hinf, 'k--', 'LineWidth', 1.5);
title('Corsa Sospensione (\delta_s)'); ylabel('m'); grid on;

subplot(3,1,3);
plot(t, u1_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, u1_hinf, 'k--', 'LineWidth', 1.5); 
title('Sforzo Attuatore 1'); ylabel('N'); xlabel('Tempo [s]'); grid on;

% --- 3) CONFRONTO: K_mixsyn vs K_PID_tuned (Entrambi Non-Lineari) ---
figure('Name', '3. Ottimo (mixsyn) vs Pratico (PID)', 'Position', [200, 200, 800, 600]);
subplot(3,1,1);
plot(t, zs_ddot_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, zs_ddot_pid, 'g--', 'LineWidth', 1.5);
title('Accelerazione Scocca (Comfort)'); ylabel('m/s^2'); grid on;
legend('H-\infty Ottimo', 'PID Strutturato', 'Location', 'best');

subplot(3,1,2);
plot(t, delta_s_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, delta_s_pid, 'g--', 'LineWidth', 1.5);
title('Corsa Sospensione (\delta_s)'); ylabel('m'); grid on;

subplot(3,1,3);
plot(t, u1_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, u1_pid, 'g--', 'LineWidth', 1.5);
title('Sforzo Attuatore 1'); ylabel('N'); xlabel('Tempo [s]'); grid on;


