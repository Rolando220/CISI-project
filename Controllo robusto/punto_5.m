%% =========================================================
%% PUNTO 5: MONTE CARLO SUL SISTEMA LINEARIZZATO (Rumore Bianco)
%% =========================================================
fprintf('\n=== Generazione 15 Impianti Campionati (Monte Carlo) ===\n');

% 1. Configurazione del Controllore
K_test = -K_red; 
K_test.u = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'}; 
K_test.y = {'u1_cmd', 'u2_cmd'};

% 2. Chiusura dell'anello incerto (da w_in alle uscite)
CL_unc = connect(P_esteso, K_test, {'w_in'}, {'zs_ddot', 'delta_s', 'delta_t'});

% 3. Campionamento del set di incertezza (15 impianti)
num_campioni = 15;
rng('default'); % Per riproducibilità
CL_campioni = usample(CL_unc, num_campioni);
CL_nominale = CL_unc.NominalValue;

% 4. Generazione del Disturbo Stradale (Scenario 4: Fuoristrada)
dt = 0.01;
t_sim = 0:dt:10;
varianza = 0.30;
% Creazione del rumore bianco band-limited
w_in_noise = sqrt(varianza/dt) * randn(length(t_sim), 1);

% 5. Simulazione e Plot 
figure('Name', 'Robustezza Linearizzata (Strada 4)', 'Position', [100, 100, 900, 600], 'Color', 'w');

% --- Plot 1: Comfort (Accelerazione) ---
subplot(2,1,1); hold on; grid on;
for i = 1:num_campioni
    [y_acc, ~] = lsim(CL_campioni(1,1,i), w_in_noise, t_sim);
    if i == 1
        % Plottiamo la prima riga e diamole un nome per la legenda
        p_camp1 = plot(t_sim, y_acc, 'Color', [0 0.4470 0.7410 0.4], 'LineWidth', 1, 'DisplayName', '15 Impianti Perturbati');
    else
        % Le altre le disegniamo ma le nascondiamo dalla legenda
        plot(t_sim, y_acc, 'Color', [0 0.4470 0.7410 0.4], 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end
% Simuliamo e plottiamo l'impianto nominale (perfetto) in rosso sopra agli altri
[y_acc_nom, ~] = lsim(CL_nominale(1,1), w_in_noise, t_sim);
p_nom1 = plot(t_sim, y_acc_nom, 'r--', 'LineWidth', 2, 'DisplayName', 'Impianto Nominale');

title('Robustezza Comfort ($\ddot{z}_s$) - Scenario 4: Sterrato', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('Accelerazione [m/s$^2$]', 'Interpreter', 'latex');
legend([p_camp1, p_nom1], 'Location', 'best');

% --- Plot 2: Tenuta di Strada (\delta_t) ---
subplot(2,1,2); hold on; grid on;
for i = 1:num_campioni
    [y_dt, ~] = lsim(CL_campioni(3,1,i), w_in_noise, t_sim);
    if i == 1
        p_camp2 = plot(t_sim, y_dt, 'Color', [0.4660 0.6740 0.1880 0.4], 'LineWidth', 1, 'DisplayName', '15 Impianti Perturbati');
    else
        plot(t_sim, y_dt, 'Color', [0.4660 0.6740 0.1880 0.4], 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end
[y_dt_nom, ~] = lsim(CL_nominale(3,1), w_in_noise, t_sim);
p_nom2 = plot(t_sim, y_dt_nom, 'r--', 'LineWidth', 2, 'DisplayName', 'Impianto Nominale');

title('Robustezza Tenuta di Strada ($\delta_t$) - Scenario 4: Sterrato', 'Interpreter', 'latex', 'FontSize', 12);
ylabel('Deformazione Gomma [m]');
xlabel('Tempo [s]');
legend([p_camp2, p_nom2], 'Location', 'best');

fprintf('Grafici generati con successo.\n');

% =========================================================
% Calcolo Analitico RMS e Picchi (Dimostrazione di Robustezza)
% =========================================================
rms_acc_campioni     = zeros(1, num_campioni);
rms_dt_campioni      = zeros(1, num_campioni);
estensione_max_dt    = zeros(1, num_campioni);
compressione_max_dt  = zeros(1, num_campioni);

for i = 1:num_campioni
    % Simulazione Comfort
    [y_acc, ~] = lsim(CL_campioni(1,1,i), w_in_noise, t_sim);
    rms_acc_campioni(i) = rms(y_acc);
    
    % Simulazione Tenuta di Strada
    [y_dt, ~] = lsim(CL_campioni(3,1,i), w_in_noise, t_sim);
    rms_dt_campioni(i)  = rms(y_dt);
    estensione_max_dt(i)   = max(y_dt); % Picco Positivo (Rischio Lift-off se > 0.0218)
    compressione_max_dt(i) = min(y_dt); % Picco Negativo (Schiacciamento)
end

fprintf('\n--- RISULTATI MONTE CARLO SUI 15 IMPIANTI INCERTI ---\n');
fprintf('COMFORT:\n');
fprintf('  RMS Accelerazione       (Min / Max): %.4f / %.4f m/s^2\n', min(rms_acc_campioni), max(rms_acc_campioni));
fprintf('TENUTA DI STRADA:\n');
fprintf('  RMS Deformazione        (Min / Max): %.4f / %.4f m\n', min(rms_dt_campioni), max(rms_dt_campioni));
fprintf('  Estensione Max (Lift-off) (Peggior Caso): %.4f m  <-- DEVE ESSERE < 0.0218 m\n', max(estensione_max_dt));
fprintf('  Compressione Max        (Peggior Caso): %.4f m\n', min(compressione_max_dt));
fprintf('-----------------------------------------------------\n\n');