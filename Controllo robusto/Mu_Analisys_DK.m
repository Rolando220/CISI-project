% =========================================================================
%  MU-ANALYSIS PER SOSPENSIONE QUARTER-CAR
%
%  Questo script richiede che siano presenti nel workspace:
%  - K_mu (Il controllore sintetizzato)
%  - P_esteso (Il generalized plant incerto)
%  - I pesi di performance (WP, Wu, WT) 
%
%   Si deve dunque lanciare mu_synthesis.m
% =========================================================================


fprintf('\n=========================================================\n');
fprintf(' MU-ANALYSIS MUSYN \n');
fprintf('=========================================================\n');

% N-DELTA STRUCTURE AND PERTURBED MODEL (Np)
% Si inverte il segno del controllore per garantire la retroazione negativa (u = -K * y)
K_mu_neg = -K_mu;

% Etichette al controllore
K_mu_neg.u = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'}; 
K_mu_neg.y = {'u1_cmd', 'u2_cmd'};                             

% Chiusura dell'anello 
Np_unweighted = connect(P_esteso, K_mu_neg, {'w_in'}, ...
                        {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot', 'u1_cmd', 'u2_cmd'});

% Pesi sulle uscite
W_perf = blkdiag(WP, Wu);
Np = W_perf * Np_unweighted;
omega = logspace(-1, 3, 100); % Griglia di frequenze [rad/s]

% =========================================================================
% NOMINAL STABILITY (NS)
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
% NOMINAL PERFORMANCE (NP)
sv_N = sigma(N_nom, omega); 
muNPinf = max(max(sv_N));
fprintf('NOMINAL PERFORMANCE (NP)\n');
fprintf('||Wp*S||_inf (picco nominale): %.4f\n', muNPinf);
if muNPinf < 1
    fprintf('  -> NP SODDISFATTA (Picco < 1).\n\n');
else
    fprintf('  -> NP NON SODDISFATTA (Picco >= 1).\n\n');
end

% =========================================================================
% ROBUST STABILITY (RS)
fprintf('ROBUST STABILITY (RS)\n');
opts_rob = robOptions('Sensitivity','On');
[stabmarg, wcu_RS, info_RS] = robstab(Np_unweighted, opts_rob); 
fprintf('Margine di Stabilità Robusta: Lower = %.4f, Upper = %.4f\n', ...
    stabmarg.LowerBound, stabmarg.UpperBound);
if stabmarg.LowerBound > 1
    fprintf('  -> RS GARANTITA (Margine > 1). Il sistema tollera tutte le incertezze.\n\n');
else
    fprintf('  -> RS NON GARANTITA. Il sistema diventa instabile al %d%%.\n\n', ...
        round(stabmarg.LowerBound * 100));
end

% =========================================================================
%  ROBUST PERFORMANCE (RP)
fprintf('ROBUST PERFORMANCE (RP) \n');
gamma_target = 1;
[perfmarg, wcu_RP,info_RP] = robgain(Np, gamma_target); 
fprintf('Margine di Performance Robusta: Lower = %.4f, Upper = %.4f\n', ...
    perfmarg.LowerBound, perfmarg.UpperBound);
if perfmarg.LowerBound >= 1
    fprintf('  -> RP SODDISFATTA.\n\n');
else
    fprintf('  -> RP NON SODDISFATTA.\n\n');
end

%% =========================================================================
%  SUMMARY E GRAFICI FINALI (SCALA LINEARE E WORST-CASE)
% =========================================================================
fprintf('=== SUMMARY MU-SYNTHESIS ===\n');
fprintf('NS : %s\n', ternary(all(real(poles_N)<0), 'OK', 'NON soddisfatta'));
fprintf('NP : %s (Picco = %.4f)\n', ternary(muNPinf<1, 'OK', 'NON soddisfatta'), muNPinf);
fprintf('RS : %s (Margine = %.4f)\n', ternary(stabmarg.LowerBound>1, 'OK', 'NON soddisfatta'), stabmarg.LowerBound);
fprintf('RP : %s (Margine = %.4f)\n', ternary(perfmarg.LowerBound>1, 'OK', 'NON soddisfatta'), perfmarg.LowerBound);

fprintf('\n=== Generazione Grafici Mu-Analisi ===\n');
opts_plot = robOptions('Display', 'off');

% Calcolo curve in frequenza
mu_RS_mu = zeros(1, length(omega));
mu_RP_mu = zeros(1, length(omega));
mu_NP_mu = max(sigma(N_nom, omega), [], 1); 

for i = 1:length(omega)
    sys_unw_w = ufrd(Np_unweighted, omega(i));
    sys_w     = ufrd(Np, omega(i));
    
    sm_mix = robstab(sys_unw_w, opts_plot);
    pm_mix = robgain(sys_w, 1, opts_plot);
    
    mu_RS_mu(i) = 1 / sm_mix.LowerBound;
    mu_RP_mu(i) = 1 / pm_mix.LowerBound;
end

% Plot Mu-Analisi singolo
figure('Name','Mu-Analisi: musyn','Position',[100, 100, 800, 450]);
h1 = semilogx(omega, mu_NP_mu(:), 'b-.', 'LineWidth', 1.5); hold on;
h2 = semilogx(omega, mu_RS_mu(:), 'g--', 'LineWidth', 1.5);
h3 = semilogx(omega, mu_RP_mu(:), 'r', 'LineWidth', 1.5);
h4 = semilogx(omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold off; grid on;
title('Mu-Analisi: Sintesi Robusta Strutturata (\mu-synthesis)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Frequenza [rad/s]', 'FontSize', 10);
ylabel('Valore Singolare Strutturato \mu', 'FontSize', 10);
legend('\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia Critica (\mu = 1)', 'Location', 'best');
ylim([0, 1.8]);

% Plot Worst-Case Sigma
figure('Name','Analisi di Robustezza in Frequenza','Position',[150, 150, 700, 500]);
wcsigma(Np, omega);
hold on;
yline(0, 'k:', 'LineWidth', 2, 'DisplayName', 'Soglia Critica 0 dB (\mu = 1)');
hold off;
grid on;
title('Mu-Analisi: Nominal vs Worst-Case Performance');

function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end

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