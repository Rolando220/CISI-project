%% --- ESTRAZIONE DATI DALLA SIMULAZIONE ---
% Ricavo il vettore tempo (uguale per tutti)
t = out.zs_ddot_passivo.Time;

% Dati PASSIVO
zs_pass = squeeze(out.zs_ddot_passivo.Data);
dt_pass = squeeze(out.delta_t_passivo.Data);
ds_pass = squeeze(out.delta_s_passivo.Data);

% Dati LQG INTEGRALE
zs_LQG = squeeze(out.zs_ddot_Int.Data);
dt_LQG = squeeze(out.delta_t_Int.Data);
ds_LQG = squeeze(out.delta_s_Int.Data);
u1_LQG = squeeze(out.u1_Int.Data);
u2_LQG = squeeze(out.u2_Int.Data);

% Dati H2[cite: 12]
zs_H2 = squeeze(out.zs_ddot_H2.Data);
dt_H2 = squeeze(out.delta_t_H2.Data);
ds_H2 = squeeze(out.delta_s_H2.Data);
u1_H2 = squeeze(out.u1_H2.Data);
u2_H2 = squeeze(out.u2_H2.Data);

%% --- 1. PLOT DEI RISULTATI A CONFRONTO ---
figure('Name', 'Confronto Passivo vs LQG vs H2', 'Position', [100, 100, 1200, 800]);

% 1. Comfort (zs_ddot)
subplot(2,2,1);
plot(t, zs_pass, 'k', t, zs_LQG, 'b', t, zs_H2, 'r', 'LineWidth', 1.5);
title('Comfort: Accelerazione Carrozzeria (zs_{ddot})');
ylabel('[m/s^2]'); grid on;
legend('Passivo', 'LQG Integrale', 'H_2', 'Location', 'best');

% 2. Road Holding (delta_t)
subplot(2,2,2);
plot(t, dt_pass, 'k', t, dt_LQG, 'b', t, dt_H2, 'r', 'LineWidth', 1.5);
title('Road Holding: Deformazione Pneumatico (\delta_t)');
ylabel('[m]'); grid on;
legend('Passivo', 'LQG Integrale', 'H_2', 'Location', 'best');

% 3. Corsa Sospensione (delta_s)
subplot(2,2,3);
plot(t, ds_pass, 'k', t, ds_LQG, 'b', t, ds_H2, 'r', 'LineWidth', 1.5);
title('Corsa Sospensione (\delta_s)');
xlabel('Tempo [s]'); ylabel('[m]'); grid on;
legend('Passivo', 'LQG Integrale', 'H_2', 'Location', 'best');

% 4. Control Effort (Forze Attuatori u1, u2)
subplot(2,2,4);
% Modificati gli stili di linea per distinguere controllore (colore) e attuatore (stile)
plot(t, u1_LQG, 'b-', t, u1_H2, 'r-', 'LineWidth', 1.5); hold on;
plot(t, u2_LQG, 'b--', t, u2_H2, 'r--', 'LineWidth', 1.5);
title('Control Effort: Forze Attuatori (u_1, u_2)');
legend('u_1 (LQG)', 'u_1 (H_2)', 'u_2 (LQG)', 'u_2 (H_2)', 'Location', 'best');
xlabel('Tempo [s]'); ylabel('[N]'); grid on;

%% --- 2. STAMPA DATI STATISTICI (KPI) A CONFRONTO ---
fprintf('\n=========================================================================\n');
fprintf('                CONFRONTO RISULTATI STATISTICI SIMULAZIONE\n');
fprintf('=========================================================================\n\n');

% COMFORT
fprintf('--- COMFORT (Accelerazione zs_ddot) [Target Max < 2.5 m/s^2] ---\n');
fprintf('  PASSIVO      -> RMS: %.4f m/s^2 | Var: %.4f | Max: %+.4f | Min: %+.4f\n', rms(zs_pass), var(zs_pass), max(zs_pass), min(zs_pass));
fprintf('  LQG INTEGR.  -> RMS: %.4f m/s^2 | Var: %.4f | Max: %+.4f | Min: %+.4f\n', rms(zs_LQG), var(zs_LQG), max(zs_LQG), min(zs_LQG));
fprintf('  H2           -> RMS: %.4f m/s^2 | Var: %.4f | Max: %+.4f | Min: %+.4f\n\n', rms(zs_H2), var(zs_H2), max(zs_H2), min(zs_H2));

% ROAD HOLDING
fprintf('--- ROAD HOLDING (Deformazione delta_t) [Target Limite < 0.02 m] ---\n');
fprintf('  PASSIVO      -> RMS: %.5f m | Var: %.6f | Max: %+.5f | Min: %+.5f\n', rms(dt_pass), var(dt_pass), max(dt_pass), min(dt_pass));
fprintf('  LQG INTEGR.  -> RMS: %.5f m | Var: %.6f | Max: %+.5f | Min: %+.5f\n', rms(dt_LQG), var(dt_LQG), max(dt_LQG), min(dt_LQG));
fprintf('  H2           -> RMS: %.5f m | Var: %.6f | Max: %+.5f | Min: %+.5f\n\n', rms(dt_H2), var(dt_H2), max(dt_H2), min(dt_H2));

% CORSA SOSPENSIONE
fprintf('--- CORSA SOSPENSIONE (delta_s) [Target Fine Corsa +/- 0.08 m] ---\n');
fprintf('  PASSIVO      -> RMS: %.5f m | Var: %.6f | Max: %+.5f | Err. Finale: %+.5f m\n', rms(ds_pass), var(ds_pass), max(ds_pass), ds_pass(end));
fprintf('  LQG INTEGR.  -> RMS: %.5f m | Var: %.6f | Max: %+.5f | Err. Finale: %+.5f m\n', rms(ds_LQG), var(ds_LQG), max(ds_LQG), ds_LQG(end));
fprintf('  H2           -> RMS: %.5f m | Var: %.6f | Max: %+.5f | Err. Finale: %+.5f m\n\n', rms(ds_H2), var(ds_H2), max(ds_H2), ds_H2(end));

% SFORZO DI CONTROLLO
fprintf('--- SFORZO DI CONTROLLO (Attuatori u1, u2) [Target < 3000 N] ---\n');
fprintf('  LQG INTEGR.  -> u1 RMS: %6.1f N | Var: %8.1f | u1 Max: %+.1f | u1 Min: %+.1f\n', rms(u1_LQG), var(u1_LQG), max(u1_LQG), min(u1_LQG));
fprintf('               -> u2 RMS: %6.1f N | Var: %8.1f | u2 Max: %+.1f | u2 Min: %+.1f\n', rms(u2_LQG), var(u2_LQG), max(u2_LQG), min(u2_LQG));
fprintf('  H2           -> u1 RMS: %6.1f N | Var: %8.1f | u1 Max: %+.1f | u1 Min: %+.1f\n', rms(u1_H2), var(u1_H2), max(u1_H2), min(u1_H2));
fprintf('               -> u2 RMS: %6.1f N | Var: %8.1f | u2 Max: %+.1f | u2 Min: %+.1f\n\n', rms(u2_H2), var(u2_H2), max(u2_H2), min(u2_H2));