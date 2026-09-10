%% =========================================================================
%% ESTRAZIONE DATI E CONFRONTO CONTROLLORI (REPORT)
%% =========================================================================


% 1. ESTRAZIONE DATI DAL WORKSPACE (Timeseries generate da Simulink)
t = out.zs_ddot_passivo.Time; 

% --- Sospensione Passiva (Baseline) ---
zs_ddot_pas = squeeze(out.zs_ddot_passivo.Data);  zs_ddot_pas = zs_ddot_pas(:);
delta_s_pas = squeeze(out.delta_s_passivo.Data);  delta_s_pas = delta_s_pas(:);
delta_t_pas = squeeze(out.delta_t_passivo.Data);  delta_t_pas = delta_t_pas(:);
u1_pas      = zeros(size(t)); % Il passivo non ha motori
u2_pas      = zeros(size(t));

% --- H-infinity Ottimo (mixsyn) ---
zs_ddot_mix = squeeze(out.zs_ddot_mixsyn.Data); zs_ddot_mix = zs_ddot_mix(:);
delta_s_mix = squeeze(out.delta_s_mixsyn.Data); delta_s_mix = delta_s_mix(:);
delta_t_mix = squeeze(out.delta_t_mixsyn.Data); delta_t_mix = delta_t_mix(:);
u1_mix      = squeeze(out.u1_mixsyn.Data);      u1_mix = u1_mix(:);
u2_mix      = squeeze(out.u2_mixsyn.Data);      u2_mix = u2_mix(:);

% --- H-infinity Ottimo (hinfsyn - Verifica) ---
zs_ddot_hinf = squeeze(out.zs_ddot_hinf.Data);  zs_ddot_hinf = zs_ddot_hinf(:);
delta_s_hinf = squeeze(out.delta_s_hinf.Data);  delta_s_hinf = delta_s_hinf(:);
delta_t_hinf = squeeze(out.delta_t_hinf.Data);  delta_t_hinf = delta_t_hinf(:);
u1_hinf      = squeeze(out.u1__hinf.Data);      u1_hinf = u1_hinf(:);
u2_hinf      = squeeze(out.u2__hinf.Data);      u2_hinf = u2_hinf(:);

% --- H-infinity Strutturato (PID Filtrato) ---
zs_ddot_pid = squeeze(out.zs_ddot_PID.Data);    zs_ddot_pid = zs_ddot_pid(:);
delta_s_pid = squeeze(out.delta_s_PID.Data);    delta_s_pid = delta_s_pid(:);
delta_t_pid = squeeze(out.delta_t_PID.Data);    delta_t_pid = delta_t_pid(:);
u1_pid      = squeeze(out.u1_PID.Data);         u1_pid = u1_pid(:);
u2_pid      = squeeze(out.u2_PID.Data);         u2_pid = u2_pid(:);

%% 2. CALCOLO METRICHE AUTOMOTIVE (RMS, Picchi, Errore a Regime, Tempo di Assestamento)
% Array per calcoli iterativi: 1=Passivo, 2=mixsyn, 3=hinfsyn, 4=PID
acc_all   = [zs_ddot_pas, zs_ddot_mix, zs_ddot_hinf, zs_ddot_pid];
dt_all    = [delta_t_pas, delta_t_mix, delta_t_hinf, delta_t_pid];
ds_all    = [delta_s_pas, delta_s_mix, delta_s_hinf, delta_s_pid];
u1_all    = [u1_pas, u1_mix, u1_hinf, u1_pid];
u2_all    = [u2_pas, u2_mix, u2_hinf, u2_pid];

% Inizializzazione
rms_acc = zeros(1,4); max_acc = zeros(1,4);
rms_dt  = zeros(1,4); max_dt  = zeros(1,4);
e_ss_ds = zeros(1,4); 
t_assest_ds  = zeros(1,4); % Tempo assestamento corsa (Autolivellamento)
t_assest_acc = zeros(1,4); % Tempo assestamento accelerazione (Comfort)
max_u1  = zeros(1,4); rms_u1  = zeros(1,4);
max_u2  = zeros(1,4); rms_u2  = zeros(1,4);

% Calcolo delle soglie di assestamento (5% del picco passivo)
banda_assest_ds  = 0.05 * max(abs(delta_s_pas));
banda_assest_acc = 0.05 * max(abs(zs_ddot_pas));

for i = 1:4
    % Metriche Comfort
    rms_acc(i) = sqrt(mean(acc_all(:, i).^2));
    max_acc(i) = max(abs(acc_all(:, i)));
    
    % Tempo di assestamento al 5% (Comfort - Accelerazione)
    idx_out_acc = find(abs(acc_all(:, i)) > banda_assest_acc, 1, 'last');
    if isempty(idx_out_acc) || idx_out_acc == length(t)
        t_assest_acc(i) = NaN; 
    else
        t_assest_acc(i) = t(idx_out_acc);
    end
    
    % Metriche Tenuta di strada
    rms_dt(i) = sqrt(mean(dt_all(:, i).^2));
    max_dt(i) = max(abs(dt_all(:, i)));
    
    % Metriche Autolivellamento
    idx_tail = round(0.9 * length(t)):length(t);
    e_ss_ds(i) = mean(ds_all(idx_tail, i)); % Errore a regime
    
    % Tempo di assestamento al 5% (Autolivellamento - Corsa Sospensione)
    idx_out_ds = find(abs(ds_all(:, i)) > banda_assest_ds, 1, 'last');
    if isempty(idx_out_ds) || idx_out_ds == length(t)
        t_assest_ds(i) = NaN; 
    else
        t_assest_ds(i) = t(idx_out_ds);
    end
    
    % Metriche Sforzo Attuatori
    rms_u1(i) = sqrt(mean(u1_all(:, i).^2));
    max_u1(i) = max(abs(u1_all(:, i)));
    rms_u2(i) = sqrt(mean(u2_all(:, i).^2));
    max_u2(i) = max(abs(u2_all(:, i)));
end

%% 3. STAMPA TABELLA RIASSUNTIVA (Da incollare nel report)
fprintf('\n=================================================================================================\n');
fprintf('                             CONFRONTO PRESTAZIONI CONTROLLORI (SIMULINK)                      \n');
fprintf('=================================================================================================\n');
fprintf('METRICA                      | Passivo (Base) | H-inf (mixsyn) | H-inf (hinfsyn) | PI (Strutturato)|\n');
fprintf('-----------------------------|----------------|----------------|-----------------|-----------------|\n');
fprintf('COMFORT\n');
fprintf('RMS Accelerazione [m/s^2]    | %14.4f | %14.4f | %15.4f | %15.4f |\n', rms_acc(1), rms_acc(2), rms_acc(3), rms_acc(4));
fprintf('Picco Accelerazione [m/s^2]  | %14.4f | %14.4f | %15.4f | %15.4f |\n', max_acc(1), max_acc(2), max_acc(3), max_acc(4));
fprintf('Tempo Assest. 5%% (Accel.)[s] | %14.2f | %14.2f | %15.2f | %15.2f |\n', t_assest_acc(1), t_assest_acc(2), t_assest_acc(3), t_assest_acc(4));
fprintf('-----------------------------|----------------|----------------|-----------------|-----------------|\n');
fprintf('TENUTA DI STRADA\n');
fprintf('RMS Deformazione Gomma [m]   | %14.4f | %14.4f | %15.4f | %15.4f |\n', rms_dt(1), rms_dt(2), rms_dt(3), rms_dt(4));
fprintf('Picco Deformaz. Gomma [m]    | %14.4f | %14.4f | %15.4f | %15.4f |\n', max_dt(1), max_dt(2), max_dt(3), max_dt(4));
fprintf('-----------------------------|----------------|----------------|-----------------|-----------------|\n');
fprintf('AUTOLIVELLAMENTO\n');
fprintf('Errore a Regime delta_s [m]  | %14.5f | %14.5f | %15.5f | %15.5f |\n', e_ss_ds(1), e_ss_ds(2), e_ss_ds(3), e_ss_ds(4));
fprintf('Tempo Assest. 5%% (Corsa) [s] | %14.2f | %14.2f | %15.2f | %15.2f |\n', t_assest_ds(1), t_assest_ds(2), t_assest_ds(3), t_assest_ds(4));
fprintf('-----------------------------|----------------|----------------|-----------------|-----------------|\n');
fprintf('SFORZO ATTUATORI\n');
fprintf('Picco Sforzo (U1) [N]        | %14.1f | %14.1f | %15.1f | %15.1f |\n', max_u1(1), max_u1(2), max_u1(3), max_u1(4));
fprintf('RMS Sforzo U1 (Energia) [N]  | %14.1f | %14.1f | %15.1f | %15.1f |\n', rms_u1(1), rms_u1(2), rms_u1(3), rms_u1(4));
fprintf('Picco Sforzo (U2) [N]        | %14.1f | %14.1f | %15.1f | %15.1f |\n', max_u2(1), max_u2(2), max_u2(3), max_u2(4));
fprintf('=================================================================================================\n');
%% 4. PLOT GRAFICI (Organizzati per il Report)

% --- Figura 1: COMFORT E TENUTA DI STRADA (Passivo vs H-inf) ---
figure('Name', 'Fig 1: Passivo vs H-inf', 'Position', [50, 50, 800, 600]);

subplot(2,1,1);
plot(t, zs_ddot_pas, 'k', 'LineWidth', 1.2); hold on;
plot(t, zs_ddot_mix, 'r', 'LineWidth', 1.5);
title('Comfort: Accelerazione della Scocca (Passivo vs Ottimo)');
ylabel('$\ddot{z}_s$ [m/s$^2$]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('Sospensione Passiva', 'H-\infty (mixsyn)', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, delta_t_pas, 'k', 'LineWidth', 1.2); hold on;
plot(t, delta_t_mix, 'r', 'LineWidth', 1.5);
title('Tenuta di Strada: Deformazione dello Pneumatico (Passivo vs Ottimo)');
ylabel('$\delta_t$ [m]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('Sospensione Passiva', 'H-\infty (mixsyn)', 'Location', 'best');
grid on;

% --- Figura 2: COMFORT E TENUTA DI STRADA (H-inf vs PI Strutturato) ---
figure('Name', 'Fig 2: H-inf vs PI', 'Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(t, zs_ddot_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, zs_ddot_pid, 'b--', 'LineWidth', 1.5);
title('Comfort: Confronto tra Controllori Attivi');
ylabel('$\ddot{z}_s$ [m/s$^2$]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('H-\infty (mixsyn)', 'PI Strutturato', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, delta_t_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, delta_t_pid, 'b--', 'LineWidth', 1.5);
title('Tenuta di Strada: Confronto tra Controllori Attivi');
ylabel('$\delta_t$ [m]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('H-\infty (mixsyn)', 'PI Strutturato', 'Location', 'best');
grid on;

% --- Figura 3: SFORZI DI CONTROLLO (Attuatore 1 e Attuatore 2) ---
figure('Name', 'Fig 3: Sforzi Attuatori', 'Position', [150, 150, 800, 600]);

subplot(2,1,1);
plot(t, u1_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, u1_pid, 'b--', 'LineWidth', 1.5);
yline(3000, 'k--', 'Limite Fisico'); yline(-3000, 'k--');
title('Sforzo Attuatore 1 (Comfort e Livellamento)');
ylabel('$u_1$ [N]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('H-\infty (mixsyn)', 'PI Strutturato', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, u2_mix, 'r', 'LineWidth', 1.5); hold on;
plot(t, u2_pid, 'b--', 'LineWidth', 1.5);
yline(3000, 'k--', 'Limite Fisico'); yline(-3000, 'k--');
title('Sforzo Attuatore 2 (Tenuta di Strada)');
ylabel('$u_2$ [N]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('H-\infty (mixsyn)', 'PI Strutturato', 'Location', 'best');
grid on;

% --- Figura 4: AUTOLIVELLAMENTO (Corsa della Sospensione da sola) ---
figure('Name', 'Fig 4: Autolivellamento', 'Position', [200, 200, 800, 400]);
plot(t, delta_s_pas, 'k', 'LineWidth', 1.2); hold on;
plot(t, delta_s_mix, 'r', 'LineWidth', 1.5);
plot(t, delta_s_pid, 'b--', 'LineWidth', 1.5);
title('Autolivellamento: Corsa della Sospensione');
ylabel('$\delta_s$ [m]', 'Interpreter', 'latex'); xlabel('Tempo [s]');
legend('Passiva', 'H-\infty (mixsyn)', 'PI Strutturato', 'Location', 'best');
grid on;