%% --- ESTRAZIONE DATI DALLA SIMULAZIONE ---
t = out.zs_ddot.Time;
zs_ddot = squeeze(out.zs_ddot.Data);
delta_t = squeeze(out.delta_t.Data);
delta_s = squeeze(out.delta_s.Data);
u1      = squeeze(out.u1.Data);
u2      = squeeze(out.u2.Data);

%% --- 1. PLOT DEI RISULTATI ---
figure('Name', 'Risultati Simulazione Quarter-Car', 'Position', [100, 100, 1000, 800]);

subplot(2,2,1);
plot(t, zs_ddot, 'b', 'LineWidth', 1.5);
title('Comfort: Accelerazione Carrozzeria (zs_{ddot})');
ylabel('[m/s^2]'); grid on;

subplot(2,2,2);
plot(t, delta_t, 'r', 'LineWidth', 1.5);
title('Road Holding: Deformazione Pneumatico (\delta_t)');
ylabel('[m]'); grid on;

subplot(2,2,3);
plot(t, delta_s, 'k', 'LineWidth', 1.5);
title('Corsa Sospensione (\delta_s)');
xlabel('Tempo [s]'); ylabel('[m]'); grid on;

subplot(2,2,4);
plot(t, u1, 'g', t, u2, 'm', 'LineWidth', 1.5);
title('Control Effort: Forze Attuatori');
legend('u1 (Carrozzeria)', 'u2 (Ruota)');
xlabel('Tempo [s]'); ylabel('[N]'); grid on;

%% --- 2. CALCOLO DATI STATISTICI (KPI) CON RANGE TIPICI ---
fprintf('\n--- RISULTATI STATISTICI SIMULAZIONE ---\n');

fprintf('COMFORT (Accelerazione zs_ddot) [Target: Picco < 2.5 m/s^2 (eccellente < 1.5), RMS < 0.5]:\n');
fprintf('  RMS   = %.4f m/s^2\n', rms(zs_ddot));
fprintf('  Picco = %.4f m/s^2\n', max(abs(zs_ddot)));

fprintf('\nROAD HOLDING (Deformazione pneumatico delta_t) [Target: Picco < 0.02 m (per non staccare la ruota)]:\n');
fprintf('  RMS   = %.5f m\n', rms(delta_t));
fprintf('  Picco = %.5f m\n', max(abs(delta_t)));

fprintf('\nCORSA SOSPENSIONE (delta_s) [Target: Picco < 0.08 m (limite fisico fine corsa)]:\n');
fprintf('  RMS   = %.5f m\n', rms(delta_s));
fprintf('  Picco = %.5f m\n', max(abs(delta_s)));

fprintf('\nSFORZO DI CONTROLLO (Attuatori) [Target: Picco < 3000 N (limite attuatori commerciali)]:\n');
fprintf('  u1: RMS = %.2f N, Picco = %.2f N\n', rms(u1), max(abs(u1)));
fprintf('  u2: RMS = %.2f N, Picco = %.2f N\n', rms(u2), max(abs(u2)));