% =========================================================================
%  MU-ANALYSIS PER SOSPENSIONE QUARTER-CAR
%
%  Questo script richiede che siano presenti nel workspace:
%  - K_hinf (Il controllore sintetizzato)
%  - P_esteso (Il generalized plant incerto)
%  - I pesi di performance (WP, Wu, WT) 
%
%   Si deve dunque lanciare uncertain_plant.m e MixeedSensitivity.m
% =========================================================================


%% MU_ANALISI MIXSYN
fprintf('\n=========================================================\n');
fprintf(' MU-ANALYSIS MIXSYN \n');
fprintf('=========================================================\n');


% N-DELTA STRUCTURE AND PERTURBED MODEL (Np)
% Si inverte il segno del controllore per garantire la retroazione negativa (u = K * (-y) = -K * y)
K_hinf_neg = -K_mixsyn;

% etichette al controllore
K_hinf_neg.u = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'}; 
K_hinf_neg.y = {'u1_cmd', 'u2_cmd'};                             

% Chiusura dell'anello 
Np_unweighted = connect(P_esteso, K_hinf_neg, {'w_in'}, ...
                        {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot', 'u1_cmd', 'u2_cmd'});


% Pesi sulle sucite (esattamente come in MixedSensitivity) 
W_perf = blkdiag(WP, Wu);
Np = W_perf * Np_unweighted;

omega = logspace(-1, 3, 100); % Griglia di frequenze [rad/s]

% =========================================================================
% NOMINAL STABILITY (NS)
% Verifica che il sistema nominale (Delta = 0) sia stabile.
N_nom = Np.NominalValue; % Estrae il sistema senza incertezze (Analisi di N11)
poles_N = pole(N_nom);

fprintf('NOMINAL STABILITY (NS)\n');
fprintf('Max Re(p) = %.6f\n', max(real(poles_N)));
if all(real(poles_N) < 0)
    fprintf('  -> NS SODDISFATTA: tutti i poli hanno parte reale negativa.\n\n');
else
    fprintf('  -> NS NON SODDISFATTA: rilevati poli instabili!\n\n');
end
% =========================================================================

% =========================================================================
% NOMINAL PERFORMANCE (NP)
% Verifica che ||Wp * S||_inf < 1 per il sistema nominale.
sv_N = sigma(N_nom, omega); %Estrazione valori singolari (Analisi di N11 pesata)
muNPinf = max(max(sv_N));

fprintf('NOMINAL PERFORMANCE (NP)\n');
fprintf('||Wp*S||_inf (picco nominale): %.4f\n', muNPinf);
if muNPinf < 1
    fprintf('  -> NP SODDISFATTA (Picco < 1).\n\n');
else
    fprintf('  -> NP NON SODDISFATTA (Picco >= 1).\n\n');
end
% =========================================================================

% =========================================================================
% ROBUST STABILITY (RS)
% Robstab per calcolare il margine di stabilità rispetto 
fprintf('ROBUST STABILITY (RS)\n');
opts_rob = robOptions('Sensitivity','On');
[stabmarg, wcu_RS, info_RS] = robstab(Np_unweighted, opts_rob); %(Analisi di N11-Delta)

fprintf('Margine di Stabilità Robusta: Lower = %.4f, Upper = %.4f\n', ...
    stabmarg.LowerBound, stabmarg.UpperBound);
if stabmarg.LowerBound > 1
    fprintf('  -> RS GARANTITA (Margine > 1). Il sistema tollera tutte le incertezze.\n');
else
    fprintf('  -> RS NON GARANTITA. Il sistema diventa instabile al %d%% dell''incertezza massima.\n', ...
        round(stabmarg.LowerBound * 100));
end

fprintf('\nSensibilità del margine alle singole incertezze:\n');
disp(info_RS.Sensitivity);
% =========================================================================

% =========================================================================
%  ROBUST PERFORMANCE (RP)
fprintf('ROBUST PERFORMANCE (RP) \n');
gamma_target = 1;
[perfmarg, wcu_RP,info_RP] = robgain(Np, gamma_target); %(Si considera tutta N)

fprintf('Margine di Performance Robusta: Lower = %.4f, Upper = %.4f\n', ...
    perfmarg.LowerBound, perfmarg.UpperBound);
if perfmarg.LowerBound >= 1
    fprintf('  -> RP SODDISFATTA.\n\n');
else
    fprintf('  -> RP NON SODDISFATTA. Rilassare i pesi WP per garantire RP.\n\n');
end
% =========================================================================




%% MU-ANALYSIS HINFSTRUCT (PI)
fprintf('\n=========================================================\n');
fprintf(' MU-ANALYSIS HINFSTRUCT  PI \n');
fprintf('=========================================================\n');

% Inversione di segno e assegnazione I/O
K_pi_neg = -K_PID_tuned;
K_pi_neg.u = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'}; 
K_pi_neg.y = {'u1_cmd', 'u2_cmd'};                             

% Chiusura dell'anello (Np_unweighted per N11)
Np_pi_unw = connect(P_esteso, K_pi_neg, {'w_in'}, ...
                        {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot', 'u1_cmd', 'u2_cmd'});

% Sistema pesato (Np per l'intera matrice N)
Np_pi = W_perf * Np_pi_unw;
% =========================================================================

% =========================================================================
% NOMINAL STABILITY (NS) PI 
N_nom_pi = Np_pi.NominalValue;
poles_N_pi = pole(N_nom_pi);
fprintf('NOMINAL STABILITY (NS)\n');
fprintf('Max Re(p) = %.6f\n', max(real(poles_N_pi)));
if all(real(poles_N_pi) < 0)
    fprintf('  -> NS SODDISFATTA: tutti i poli hanno parte reale negativa.\n\n');
else
    fprintf('  -> NS NON SODDISFATTA: rilevati poli instabili!\n\n');
end
% =========================================================================

% =========================================================================
% NOMINAL PERFORMANCE (NP) PI 
sv_N_pi = sigma(N_nom_pi, omega); 
muNPinf_pi = max(max(sv_N_pi));
fprintf('\n=== NOMINAL PERFORMANCE (NP) ===\n');
fprintf('||Wp*S||_inf (picco nominale): %.4f\n', muNPinf_pi);
if muNPinf_pi < 1
    fprintf('  -> NP SODDISFATTA (Picco < 1).\n\n');
else
    fprintf('  -> NP NON SODDISFATTA (Picco >= 1).\n\n');
end
% =========================================================================

% =========================================================================
% ROBUST STABILITY (RS) PI 
fprintf('\n=== ROBUST STABILITY (RS) ===\n');
[stabmarg_pi, ~, info_RS_pi] = robstab(Np_pi_unw, opts_rob);
fprintf('Margine di Stabilità Robusta: Lower = %.4f, Upper = %.4f\n', ...
    stabmarg_pi.LowerBound, stabmarg_pi.UpperBound);
if stabmarg_pi.LowerBound > 1
    fprintf('  -> RS GARANTITA (Margine > 1). Il sistema tollera tutte le incertezze.\n');
else
    fprintf('  -> RS NON GARANTITA. Il sistema diventa instabile al %d%% dell''incertezza massima.\n', ...
        round(stabmarg_pi.LowerBound * 100));
end
fprintf('\nSensibilità del margine alle singole incertezze:\n');
disp(info_RS_pi.Sensitivity);
% =========================================================================

% =========================================================================
% ROBUST PERFORMANCE (RP) PI 
fprintf('\n=== ROBUST PERFORMANCE (RP) ===\n');
[perfmarg_pi, ~, ~] = robgain(Np_pi, 1);
fprintf('Margine di Performance Robusta: Lower = %.4f, Upper = %.4f\n', ...
    perfmarg_pi.LowerBound, perfmarg_pi.UpperBound);

if perfmarg_pi.LowerBound >= 1
    fprintf('  -> RP SODDISFATTA.\n\n');
else
    fprintf('  -> RP NON SODDISFATTA. Rilassare i pesi WP per garantire RP.\n\n');
end
% =========================================================================


%% SUMMARY 

% =========================================================================
% MIXSYN 
fprintf('=== SUMMARY MIXSYN  ===\n');
fprintf('NS : %s\n', ternary(all(real(poles_N)<0), 'OK', 'NON soddisfatta'));
fprintf('NP : %s (Picco = %.4f)\n', ternary(muNPinf<1, 'OK', 'NON soddisfatta'), muNPinf);
fprintf('RS : %s (Margine = %.4f)\n', ternary(stabmarg.LowerBound>1, 'OK', 'NON soddisfatta'), stabmarg.LowerBound);
fprintf('RP : %s (Margine = %.4f)\n', ternary(perfmarg.LowerBound>1, 'OK', 'NON soddisfatta'), perfmarg.LowerBound);
% =========================================================================

% =========================================================================
% HINFSTRUCT (PI)
fprintf('\n=== SUMMARY HINFSTRUCT (PI) ===\n');
fprintf('NS : %s\n', ternary(all(real(poles_N_pi)<0), 'OK', 'NON soddisfatta'));
fprintf('NP : %s (Picco = %.4f)\n', ternary(muNPinf_pi<1, 'OK', 'NON soddisfatta'), muNPinf_pi);
fprintf('RS : %s (Margine = %.4f)\n', ternary(stabmarg_pi.LowerBound>1, 'OK', 'NON soddisfatta'), stabmarg_pi.LowerBound);
fprintf('RP : %s (Margine = %.4f)\n', ternary(perfmarg_pi.LowerBound>1, 'OK', 'NON soddisfatta'), perfmarg_pi.LowerBound);
% =========================================================================


% ── Funzione di supporto  ─────────────
function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end


%% =========================================================================
%  GRAFICI FINALI: MU-PLOT (SCALA LINEARE)
% =========================================================================
fprintf('\n=== Generazione Grafici Mu-Analisi ===\n');

% 1. Conversione in modelli incerti in frequenza (ufrd) per evitare cicli
opts_plot = robOptions('Display', 'off');
Np_mix_g     = ufrd(Np, omega);
Np_mix_unw_g = ufrd(Np_unweighted, omega);
Np_pi_g      = ufrd(Np_pi, omega);
Np_pi_unw_g  = ufrd(Np_pi_unw, omega);

% 2. Calcolo dei vettori mu per il MIXSYN (Inverso del LowerBound)
fprintf('Calcolo curve in frequenza per MIXSYN (attendere qualche secondo)...\n');
opts_plot = robOptions('Display', 'off');

% Inizializziamo i vettori a zero
mu_RS_mix = zeros(1, length(omega));
mu_RP_mix = zeros(1, length(omega));

% Vettore NP (sigma funziona già su tutto il vettore omega)
mu_NP_mix = max(sigma(N_nom, omega), [], 1); 

for i = 1:length(omega)
    % Valutazione puntuale usando ufrd per MANTENERE le incertezze
    sys_unw_w = ufrd(Np_unweighted, omega(i));
    sys_w     = ufrd(Np, omega(i));
    
    sm_mix = robstab(sys_unw_w, opts_plot);
    pm_mix = robgain(sys_w, 1, opts_plot);
    
    mu_RS_mix(i) = 1 / sm_mix.LowerBound;
    mu_RP_mix(i) = 1 / pm_mix.LowerBound;
end

% 3. Calcolo dei vettori mu per il PI (hinfstruct)
fprintf('Calcolo curve in frequenza per HINFSTRUCT PI (attendere qualche secondo)...\n');
mu_RS_pi = zeros(1, length(omega));
mu_RP_pi = zeros(1, length(omega));

mu_NP_pi = max(sigma(N_nom_pi, omega), [], 1);

for i = 1:length(omega)
    sys_unw_pi_w = ufrd(Np_pi_unw, omega(i));
    sys_pi_w     = ufrd(Np_pi, omega(i));
    
    sm_pi = robstab(sys_unw_pi_w, opts_plot);
    pm_pi = robgain(sys_pi_w, 1, opts_plot);
    
    mu_RS_pi(i) = 1 / sm_pi.LowerBound;
    mu_RP_pi(i) = 1 / pm_pi.LowerBound;
end


%% 4. Creazione della Figura affiancata
figure('Name','Mu-Analisi: mixsyn vs hinfstruct','Position',[100, 100, 1000, 450]);

% --- Subplot 1: MIXSYN ---
subplot(2,1,1);
h1 = semilogx(omega, mu_NP_mix(:), 'b-.', 'LineWidth', 1.5); hold on;
h2 = semilogx(omega, mu_RS_mix(:), 'g--', 'LineWidth', 1.5);
h3 = semilogx(omega, mu_RP_mix(:), 'r', 'LineWidth', 1.5);
% Sostituito yline con semilogx per avere oggetti omogenei
h4 = semilogx(omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold off; grid on;
title('Mu-Analisi: Ottimo H_\infty (mixsyn)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frequenza [rad/s]', 'FontSize', 10);
ylabel('Valore Singolare Strutturato \mu', 'FontSize', 10);
legend('\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia Critica (\mu = 1)', 'Location', 'best');
ylim([0, 1.8]); % Fissiamo il limite Y per un confronto equo

% --- Subplot 2: HINFSTRUCT (PI) ---
subplot(2,1,2);
h5 = semilogx(omega, mu_NP_pi(:), 'b-.', 'LineWidth', 1.5); hold on;
h6 = semilogx(omega, mu_RS_pi(:), 'g--', 'LineWidth', 1.5);
h7 = semilogx(omega, mu_RP_pi(:), 'r', 'LineWidth', 1.5);
% Sostituito yline con semilogx per avere oggetti omogenei
h8 = semilogx(omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold off; grid on;
title('Mu-Analisi: PI Strutturato (hinfstruct)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frequenza [rad/s]', 'FontSize', 10);
legend('\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia Critica (\mu = 1)', 'Location', 'best');
ylim([0, 1.8]);

fprintf('\nGrafici generati con successo!\n');


















% %% =========================================================================
% %  12. GRAFICI FINALI: NOMINAL vs WORST-CASE PERFORMANCE
% % =========================================================================
% fprintf('\n=== Generazione Grafici ===\n');
% 
% figure('Name','Analisi di Robustezza in Frequenza','Position',[100, 100, 700, 500]);
% 
% % La funzione wcsigma chiamata senza argomenti di uscita traccia 
% % automaticamente la curva Nominale e la curva Worst-Case!
% wcsigma(Np, omega);
% 
% % Aggiungiamo la nostra linea critica di 0 dB (mu = 1)
% hold on;
% yline(0, 'k:', 'LineWidth', 2, 'DisplayName', 'Soglia Critica 0 dB (\mu = 1)');
% hold off;
% 
% grid on;
% title('Mu-Analisi: Nominal vs Worst-Case Performance');




