%% --- ESTRAZIONE DATI DALLA SIMULAZIONE ---
% Ricavo il vettore tempo (uguale per tutti)
t = out.zs_ddot_passivo.Time;

% Dati PASSIVO
zs_pass = squeeze(out.zs_ddot_passivo.Data);
dt_pass = squeeze(out.delta_t_passivo.Data);
ds_pass = squeeze(out.delta_s_passivo.Data);

% Dati K_rob 
zs_krob = squeeze(out.zs_ddot_krob.Data);
dt_krob = squeeze(out.delta_t_krob.Data);
ds_krob = squeeze(out.delta_s_krob.Data);
u1_krob = squeeze(out.u1_krob.Data);
u2_krob = squeeze(out.u2_krob.Data);

% Dati K_red
zs_kred = squeeze(out.zs_ddot_kred.Data);
dt_kred = squeeze(out.delta_t_kred.Data);
ds_kred = squeeze(out.delta_s_kred.Data);
u1_kred = squeeze(out.u1_kred.Data);
u2_kred = squeeze(out.u2_kred.Data);

%% --- 1. PLOT DEI RISULTATI A CONFRONTO ---
figure('Name', 'Confronto Passivo vs LQG senza kred. vs LQG con kred.', 'Position', [100, 100, 1200, 800]);

% 1. Comfort (zs_ddot)
subplot(2,2,1);
plot(t, zs_pass, 'k', t, zs_krob, 'b', t, zs_kred, 'r', 'LineWidth', 1.5);
title('Comfort: Accelerazione Carrozzeria (zs_{ddot})');
ylabel('[m/s^2]'); grid on;
legend('Passivo', 'K_rob', 'K_red', 'Location', 'best');

% 2. Road Holding (delta_t)
subplot(2,2,2);
plot(t, dt_pass, 'k', t, dt_krob, 'b', t, dt_kred, 'r', 'LineWidth', 1.5);
title('Road Holding: Deformazione Pneumatico (\delta_t)');
ylabel('[m]'); grid on;
legend('Passivo', 'K_rob', 'K_red', 'Location', 'best');

% 3. Corsa Sospensione (delta_s)
subplot(2,2,3);
plot(t, ds_pass, 'k', t, ds_krob, 'b', t, ds_kred, 'r', 'LineWidth', 1.5);
title('Corsa Sospensione (\delta_s)');
xlabel('Tempo [s]'); ylabel('[m]'); grid on;
legend('Passivo', 'K_rob', 'K_red', 'Location', 'best');

% 4. Control Effort (Forze Attuatori u1, u2)
subplot(2,2,4);
plot(t, u1_krob, 'b--', t, u1_kred, 'b-', 'LineWidth', 1.5); hold on;
plot(t, u2_krob, 'r--', t, u2_kred, 'r-', 'LineWidth', 1.5);
title('Control Effort: Forze Attuatori (u_1, u_2)');
legend('u_1 (K_rob)', 'u_1 (K_red)', 'u_2 (K_rob)', 'u_2 (K_red)', 'Location', 'best');
xlabel('Tempo [s]'); ylabel('[N]'); grid on;

%% -- STAMPA DATI STATISTICI (KPI) A CONFRONTO ---
fprintf('\n=========================================================================\n');
fprintf('                CONFRONTO RISULTATI STATISTICI SIMULAZIONE\n');
fprintf('=========================================================================\n\n');

% COMFORT
fprintf('--- COMFORT (Accelerazione zs_ddot) [Target Max < 2.5 m/s^2] ---\n');
fprintf('  PASSIVO     -> Varianza: %.4f | RMS: %.4f m/s^2 | Max: %+.4f | Min: %+.4f\n', var(zs_pass), rms(zs_pass), max(zs_pass), min(zs_pass));
fprintf('  K_rob       -> Varianza: %.4f | RMS: %.4f m/s^2 | Max: %+.4f | Min: %+.4f\n', var(zs_krob), rms(zs_krob), max(zs_krob), min(zs_krob));
fprintf('  K_red       -> Varianza: %.4f | RMS: %.4f m/s^2 | Max: %+.4f | Min: %+.4f\n\n', var(zs_kred), rms(zs_kred), max(zs_kred), min(zs_kred));

% ROAD HOLDING
fprintf('--- ROAD HOLDING (Deformazione delta_t) [Target Limite < 0.02 m] ---\n');
fprintf('  PASSIVO     -> Varianza: %.6f | RMS: %.5f m | Max: %+.5f | Min: %+.5f\n', var(dt_pass), rms(dt_pass), max(dt_pass), min(dt_pass));
fprintf('  K_rob       -> Varianza: %.6f | RMS: %.5f m | Max: %+.5f | Min: %+.5f\n', var(dt_krob), rms(dt_krob), max(dt_krob), min(dt_krob));
fprintf('  K_red       -> Varianza: %.6f | RMS: %.5f m | Max: %+.5f | Min: %+.5f\n\n', var(dt_kred), rms(dt_kred), max(dt_kred), min(dt_kred));

% CORSA SOSPENSIONE
fprintf('--- CORSA SOSPENSIONE (delta_s) [Target Fine Corsa +/- 0.08 m] ---\n');
fprintf('  PASSIVO     -> RMS: %.5f m | Max: %+.5f | Err. Finale: %+.5f m\n', rms(ds_pass), max(ds_pass), ds_pass(end));
fprintf('  K_rob       -> RMS: %.5f m | Max: %+.5f | Err. Finale: %+.5f m\n', rms(ds_krob), max(ds_krob), ds_krob(end));
fprintf('  K_red       -> RMS: %.5f m | Max: %+.5f | Err. Finale: %+.5f m\n\n', rms(ds_kred), max(ds_kred), ds_kred(end));

% SFORZO DI CONTROLLO
fprintf('--- SFORZO DI CONTROLLO (Attuatori u1, u2) [Target < 3000 N] ---\n');
fprintf('  K_rob   -> u1 RMS: %6.1f N | u1 Max: %+.1f | u1 Min: %+.1f\n', rms(u1_krob), max(u1_krob), min(u1_krob));
fprintf('          -> u2 RMS: %6.1f N | u2 Max: %+.1f | u2 Min: %+.1f\n', rms(u2_krob), max(u2_krob), min(u2_krob));
fprintf('  K_red   -> u1 RMS: %6.1f N | u1 Max: %+.1f | u1 Min: %+.1f\n', rms(u1_kred), max(u1_kred), min(u1_kred));
fprintf('          -> u2 RMS: %6.1f N | u2 Max: %+.1f | u2 Min: %+.1f\n\n', rms(u2_kred), max(u2_kred), min(u2_kred));