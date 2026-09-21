%% --- ESTRAZIONE DATI DALLA SIMULAZIONE ---
% Ricavo il vettore tempo (uguale per tutti)
t = out.zs_ddot_passivo.Time;

% Dati PASSIVO
zs_pass = squeeze(out.zs_ddot_passivo.Data);
dt_pass = squeeze(out.delta_t_passivo.Data);
ds_pass = squeeze(out.delta_s_passivo.Data);

% Dati LQG BASE (Senza Integratore)
zs_noInt = squeeze(out.zs_ddot_H2.Data);
dt_noInt = squeeze(out.delta_t_H2.Data);
ds_noInt = squeeze(out.delta_s_H2.Data);
u1_noInt = squeeze(out.u1_H2.Data);
u2_noInt = squeeze(out.u2_H2.Data);

% Dati LQG INTEGRALE
zs_Int = squeeze(out.zs_ddot_Int.Data);
dt_Int = squeeze(out.delta_t_Int.Data);
ds_Int = squeeze(out.delta_s_Int.Data);
u1_Int = squeeze(out.u1_Int.Data);
u2_Int = squeeze(out.u2_Int.Data);

%% --- 1. PLOT DEI RISULTATI A CONFRONTO ---
figure('Name', 'Confronto Passivo vs LQG senza Int. vs LQG con Int.', 'Position', [100, 100, 1200, 800]);

% 1. Comfort (zs_ddot)
subplot(2,2,1);
plot(t, zs_pass, 'k', t, zs_noInt, 'b', t, zs_Int, 'r', 'LineWidth', 1.5);
title('Comfort: Accelerazione Carrozzeria (zs_{ddot})');
ylabel('[m/s^2]'); grid on;
legend('Passivo', 'LQG no Int', 'LQG Integrale', 'Location', 'best');

% 2. Road Holding (delta_t)
%figure('Name', 'Confronto Passivo vs LQG senza Int. vs LQG con Int.', 'Position', [100, 100, 1200, 800]);
subplot(2,2,2);
plot(t, dt_pass, 'k', t, dt_noInt, 'b', t, dt_Int, 'r', 'LineWidth', 1.5);
title('Road Holding: Deformazione Pneumatico (\delta_t)');
ylabel('[m]'); grid on;
legend('Passivo', 'LQG no Int', 'LQG Integrale', 'Location', 'best');

% 3. Corsa Sospensione (delta_s)
subplot(2,2,3);
%figure('Name', 'Confronto Passivo vs LQG senza Int. vs LQG con Int.', 'Position', [100, 100, 1200, 800]);
plot(t, ds_pass, 'k', t, ds_noInt, 'b', t, ds_Int, 'r', 'LineWidth', 1.5);
title('Corsa Sospensione (\delta_s)');
xlabel('Tempo [s]'); ylabel('[m]'); grid on;
legend('Passivo', 'LQG no Int', 'LQG Integrale', 'Location', 'best');

% 4. Control Effort (Forze Attuatori u1, u2)
subplot(2,2,4);
%figure('Name', 'Confronto Passivo vs LQG senza Int. vs LQG con Int.', 'Position', [100, 100, 1200, 800]);
% Uso linee tratteggiate per il base e continue per l'integrale per distinguerle
plot(t, u1_noInt, 'b--', t, u1_Int, 'b-', 'LineWidth', 1.5); hold on;
plot(t, u2_noInt, 'r--', t, u2_Int, 'r-', 'LineWidth', 1.5);
title('Control Effort: Forze Attuatori (u_1, u_2)');
legend('u_1 (no Int)', 'u_1 (Integrale)', 'u_2 (no Int)', 'u_2 (Integrale)', 'Location', 'best');
xlabel('Tempo [s]'); ylabel('[N]'); grid on;

%% --- 2. STAMPA DATI STATISTICI (KPI) A CONFRONTO ---
fprintf('\n=========================================================================\n');
fprintf('                CONFRONTO RISULTATI STATISTICI SIMULAZIONE\n');
fprintf('=========================================================================\n\n');

% COMFORT
fprintf('--- COMFORT (Accelerazione zs_ddot) [Target Max < 2.5 m/s^2] ---\n');
fprintf('  PASSIVO      -> RMS: %.4f m/s^2 | Max: %+.4f | Min: %+.4f\n', rms(zs_pass), max(zs_pass), min(zs_pass));
fprintf('  LQG NO INT.  -> RMS: %.4f m/s^2 | Max: %+.4f | Min: %+.4f\n', rms(zs_noInt), max(zs_noInt), min(zs_noInt));
fprintf('  LQG INTEGR.  -> RMS: %.4f m/s^2 | Max: %+.4f | Min: %+.4f\n\n', rms(zs_Int), max(zs_Int), min(zs_Int));

% ROAD HOLDING
fprintf('--- ROAD HOLDING (Deformazione delta_t) [Target Limite < 0.02 m] ---\n');
fprintf('  PASSIVO      -> RMS: %.5f m | Max: %+.5f | Min: %+.5f\n', rms(dt_pass), max(dt_pass), min(dt_pass));
fprintf('  LQG NO INT.  -> RMS: %.5f m | Max: %+.5f | Min: %+.5f\n', rms(dt_noInt), max(dt_noInt), min(dt_noInt));
fprintf('  LQG INTEGR.  -> RMS: %.5f m | Max: %+.5f | Min: %+.5f\n\n', rms(dt_Int), max(dt_Int), min(dt_Int));

% CORSA SOSPENSIONE
fprintf('--- CORSA SOSPENSIONE (delta_s) [Target Fine Corsa +/- 0.08 m] ---\n');
fprintf('  PASSIVO      -> RMS: %.5f m | Max: %+.5f | Err. Finale: %+.5f m\n', rms(ds_pass), max(ds_pass), ds_pass(end));
fprintf('  LQG NO INT.  -> RMS: %.5f m | Max: %+.5f | Err. Finale: %+.5f m\n', rms(ds_noInt), max(ds_noInt), ds_noInt(end));
fprintf('  LQG INTEGR.  -> RMS: %.5f m | Max: %+.5f | Err. Finale: %+.5f m\n\n', rms(ds_Int), max(ds_Int), ds_Int(end));

% SFORZO DI CONTROLLO
fprintf('--- SFORZO DI CONTROLLO (Attuatori u1, u2) [Target < 3000 N] ---\n');
fprintf('  LQG NO INT.  -> u1 RMS: %6.1f N | u1 Max: %+.1f | u1 Min: %+.1f\n', rms(u1_noInt), max(u1_noInt), min(u1_noInt));
fprintf('               -> u2 RMS: %6.1f N | u2 Max: %+.1f | u2 Min: %+.1f\n', rms(u2_noInt), max(u2_noInt), min(u2_noInt));
fprintf('  LQG INTEGR.  -> u1 RMS: %6.1f N | u1 Max: %+.1f | u1 Min: %+.1f\n', rms(u1_Int), max(u1_Int), min(u1_Int));
fprintf('               -> u2 RMS: %6.1f N | u2 Max: %+.1f | u2 Min: %+.1f\n\n', rms(u2_Int), max(u2_Int), min(u2_Int));