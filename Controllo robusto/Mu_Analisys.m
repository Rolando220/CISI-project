% =========================================================================
%  MU-ANALYSIS PER SOSPENSIONE QUARTER-CAR
%
%  Questo script richiede che siano presenti nel workspace:
%  - K_mixsyn, K_hinf, K_PID_tuned (I controllori sintetizzati)
%  - P_esteso (Il generalized plant incerto)
%  - I pesi di performance (WP, Wu, WT) 
% =========================================================================
clear Np* mu_*; % Pulizia preventiva delle variabili di loop

% Pesi e Griglia di Frequenze condivisi
W_perf = blkdiag(WP, Wu);
omega = logspace(-1, 3, 100);

%% MU-ANALISI 
% La funzione esegui_mu_analisi restituisce le strutture Np necessarie per i plot

[Np_mix, Np_unw_mix, margini_mix]   = esegui_mu_analisi(K_mixsyn, 'MIXSYN (Full-Order)', P_esteso, W_perf, omega);
[Np_hinf, Np_unw_hinf, margini_hinf] = esegui_mu_analisi(K_hinf, 'HINFSYN (Full-Order)', P_esteso, W_perf, omega);
[Np_pi, Np_unw_pi, margini_pi]      = esegui_mu_analisi(K_PID_tuned, 'PI STRUTTURATO', P_esteso, W_perf, omega);


%% SUMMARY 
fprintf('\n=========================================================\n');
fprintf(' === SUMMARY FINALE DEI MARGINI ===\n');
fprintf('=========================================================\n');
stampa_summary('MIXSYN', margini_mix);
stampa_summary('HINFSYN', margini_hinf);
stampa_summary('PI STRUTTURATO', margini_pi);


%% GENERAZIONE GRAFICI 

% Calcolo grafici
fprintf('\n=== Calcolo curve in frequenza per i Plot (attendere...) ===\n');

[mu_NP_mix, mu_RS_mix, mu_RP_mix]   = calcola_mu_frequenza(Np_mix, Np_unw_mix, omega);
[mu_NP_hinf, mu_RS_hinf, mu_RP_hinf] = calcola_mu_frequenza(Np_hinf, Np_unw_hinf, omega);
[mu_NP_pi, mu_RS_pi, mu_RP_pi]      = calcola_mu_frequenza(Np_pi, Np_unw_pi, omega);


% Crea un'unica figura principale
fig_main = figure('Name','Mu-Analisi: Confronto Controllori', 'Position', [100, 100, 800, 500]);

% Crea il gruppo di schede (Tabs)
tgroup = uitabgroup('Parent', fig_main);

% --- Scheda 1: MIXSYN ---
tab1 = uitab('Parent', tgroup, 'Title', 'MIXSYN (Full-Order)');
ax1 = axes('Parent', tab1); % Associa gli assi a questa scheda
semilogx(ax1, omega, mu_NP_mix(:), 'b-.', 'LineWidth', 1.5); hold(ax1, 'on');
semilogx(ax1, omega, mu_RS_mix(:), 'g--', 'LineWidth', 1.5);
semilogx(ax1, omega, mu_RP_mix(:), 'r', 'LineWidth', 1.5);
semilogx(ax1, omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold(ax1, 'off'); grid(ax1, 'on');
title(ax1, 'Mu-Analisi: Ottimo H_\infty (mixsyn)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax1, 'Frequenza [rad/s]', 'FontSize', 10);
ylabel(ax1, 'Valore Singolare \mu', 'FontSize', 10);
legend(ax1, '\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia (\mu = 1)', 'Location', 'best');
ylim(ax1, [0, 1.8]);

% --- Scheda 2: HINFSYN ---
tab2 = uitab('Parent', tgroup, 'Title', 'HINFSYN (Full-Order)');
ax2 = axes('Parent', tab2);
semilogx(ax2, omega, mu_NP_hinf(:), 'b-.', 'LineWidth', 1.5); hold(ax2, 'on');
semilogx(ax2, omega, mu_RS_hinf(:), 'g--', 'LineWidth', 1.5);
semilogx(ax2, omega, mu_RP_hinf(:), 'r', 'LineWidth', 1.5);
semilogx(ax2, omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold(ax2, 'off'); grid(ax2, 'on');
title(ax2, 'Mu-Analisi: Ottimo H_\infty (hinfsyn)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax2, 'Frequenza [rad/s]', 'FontSize', 10);
ylabel(ax2, 'Valore Singolare \mu', 'FontSize', 10);
legend(ax2, '\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia (\mu = 1)', 'Location', 'best');
ylim(ax2, [0, 1.8]);

% --- Scheda 3: HINFSTRUCT (PI) ---
tab3 = uitab('Parent', tgroup, 'Title', 'PI Strutturato');
ax3 = axes('Parent', tab3);
semilogx(ax3, omega, mu_NP_pi(:), 'b-.', 'LineWidth', 1.5); hold(ax3, 'on');
semilogx(ax3, omega, mu_RS_pi(:), 'g--', 'LineWidth', 1.5);
semilogx(ax3, omega, mu_RP_pi(:), 'r', 'LineWidth', 1.5);
semilogx(ax3, omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold(ax3, 'off'); grid(ax3, 'on');
title(ax3, 'Mu-Analisi: PI Strutturato (hinfstruct)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax3, 'Frequenza [rad/s]', 'FontSize', 10);
ylabel(ax3, 'Valore Singolare \mu', 'FontSize', 10);
legend(ax3, '\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia (\mu = 1)', 'Location', 'best');
ylim(ax3, [0, 1.8]);

fprintf('\nGrafici generati con successo in un''unica finestra a schede!\n');

%% FUNZIONI LOCALI DI SUPPORTO


function [Np, Np_unweighted, margini] = esegui_mu_analisi(K_ctrl, nome_ctrl, P_esteso, W_perf, omega)
    fprintf('\n---------------------------------------------------------\n');
    fprintf(' MU-ANALYSIS: %s \n', nome_ctrl);
    fprintf('---------------------------------------------------------\n');

    % Chiusura Anello
    K_neg = -K_ctrl;
    K_neg.u = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'}; 
    K_neg.y = {'u1_cmd', 'u2_cmd'};                             
    Np_unweighted = connect(P_esteso, K_neg, {'w_in'}, {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot', 'u1_cmd', 'u2_cmd'});
    Np = W_perf * Np_unweighted;

    % Nominal Stability (NS)
    N_nom = Np.NominalValue;
    poles_N = pole(N_nom);
    NS_ok = all(real(poles_N) < 0);
    fprintf(' NS: %s (Max Re(p) = %.4f)\n', ternary(NS_ok, 'OK', 'FAIL'), max(real(poles_N)));

    % Nominal Performance (NP)
    muNPinf = max(max(sigma(N_nom, omega)));
    NP_ok = muNPinf < 1;
    fprintf(' NP: %s (Picco = %.4f)\n', ternary(NP_ok, 'OK', 'FAIL'), muNPinf);

    % Robust Stability (RS) con Sensibilità
    opts_rob = robOptions('Display', 'off', 'Sensitivity', 'on'); % Attivata la sensibilità!
    [sm, ~, info_RS] = robstab(Np_unweighted, opts_rob);
    RS_ok = sm.LowerBound > 1;
    fprintf(' RS: %s (Margine = %.4f)\n', ternary(RS_ok, 'OK', 'FAIL'), sm.LowerBound);
    
    % Stampa della sensibilità
    fprintf('     Sensibilità del margine alle singole incertezze:\n');
    disp(info_RS.Sensitivity);

    % Robust Performance (RP)
    pm = robgain(Np, 1, robOptions('Display', 'off'));
    RP_ok = pm.LowerBound >= 1;
    fprintf(' RP: %s (Margine = %.4f)\n', ternary(RP_ok, 'OK', 'FAIL'), pm.LowerBound);

    % Salvataggio margini per il summary
    margini.NS = NS_ok;
    margini.NP = NP_ok; 
    margini.muNPinf = muNPinf;
    margini.RS = RS_ok; 
    margini.RS_val = sm.LowerBound;
    margini.RP = RP_ok; 
    margini.RP_val = pm.LowerBound;
end

function [mu_NP, mu_RS, mu_RP] = calcola_mu_frequenza(Np, Np_unw, omega)
    opts_plot = robOptions('Display', 'off');
    mu_RS = zeros(1, length(omega));
    mu_RP = zeros(1, length(omega));
    mu_NP = max(sigma(Np.NominalValue, omega), [], 1); 

    for i = 1:length(omega)
        sm = robstab(ufrd(Np_unw, omega(i)), opts_plot);
        pm = robgain(ufrd(Np, omega(i)), 1, opts_plot);
        mu_RS(i) = 1 / sm.LowerBound;
        mu_RP(i) = 1 / pm.LowerBound;
    end
end

function stampa_summary(nome, m)
    fprintf('%-18s | NS: %-4s | NP: %-4s (%.2f) | RS: %-4s (%.2f) | RP: %-4s (%.2f)\n', ...
        nome, ...
        ternary(m.NS, 'OK', 'FAIL'), ...
        ternary(m.NP, 'OK', 'FAIL'), m.muNPinf, ...
        ternary(m.RS, 'OK', 'FAIL'), m.RS_val, ...
        ternary(m.RP, 'OK', 'FAIL'), m.RP_val);
end

function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end