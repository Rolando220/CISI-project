%   =========================================================
%       SINTESI ROBUSTA (mu-sintesi) - MIMO Quarter-Car
%   =========================================================

%% Caricamento Parametri e Impianto Incerto
clear all; close all; clc;
run('A_quarter_car_parameters.m');
run('A_uncertain_Plant.m');

s = tf('s');

%% Definizione dei Pesi di Performance e Controllo

% Comfort
w_n = 6.5; zeta = 0.5; Gain = 1.5;
wP1 = Gain * (2 * zeta * w_n * s) / (s^2 + 2 * zeta * w_n * s + w_n^2);
%wP1 = 0.01;

% Autolivellamento: peso a bassa freq (quasi integratore) per schiacciare delta_s
Gain_livellamento = 0.5;
wP2 = Gain_livellamento / (s + 0.001);

% Tenuta di Strada
w_n_ruota = 63; zeta_ruota = 0.6; Gain_ruota = 1.2;
filtro_ruota = Gain_ruota * (2 * zeta_ruota * w_n_ruota * s) / (s^2 + 2 * zeta_ruota * w_n_ruota * s + w_n_ruota^2);
wP3 = 0.1 + filtro_ruota;

% Ruota
wP4 = 0.01;

% Assemblaggio WP
WP = blkdiag(wP1, wP2, wP3, wP4);
WP = tf(WP);
WP.InputName  = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};
WP.OutputName = {'z_p1', 'z_p2', 'z_p3', 'z_p4'};

% Pesi sugli Attuatori (Wu)
wu_LF = 1/50000; wu_HF = 1/1000; w_taglio = 30;
wu = wu_HF * (s + w_taglio * (wu_LF/wu_HF)) / (s + w_taglio);

% Assemblaggio Wu
Wu = blkdiag(wu, wu);
Wu = tf(Wu);
Wu.InputName  = {'u1_cmd', 'u2_cmd'};
Wu.OutputName = {'z_u1', 'z_u2'};

%% Nodi Sommatore per le Misure e RUMORE FITTIZIO

% 3 rumori indipendenti 
Wn = 0.01 * eye(3); 
Wn = tf(Wn);
Wn.InputName = {'n_zs', 'n_ds', 'n_zu'};
Wn.OutputName = {'noise_zs', 'noise_ds', 'noise_zu'};

% Somma rumore fittizio alle misurazioni (reazione negativa)
Sv_zs = sumblk('v_zs = -zs_ddot - noise_zs');
Sv_ds = sumblk('v_ds = -delta_s - noise_ds');
Sv_zu = sumblk('v_zu = -zu_ddot - noise_zu');

%% Costruzione Impianto Generalizzato Incerto
P_gen_unc = connect(P_esteso, WP, Wu, Wn, Sv_zs, Sv_ds, Sv_zu, ...
                     {'w_in', 'n_zs', 'n_ds', 'n_zu', 'u1_cmd', 'u2_cmd'}, ...
                     {'z_p1','z_p2','z_p3','z_p4','z_u1','z_u2','v_zs','v_ds','v_zu'});


%% SINTESI H-INFINITY ROBUSTA (D-K Iteration via musyn)
% P_gen_unc ha 10 uscite (le prime 6 sono z, le ultime 4 sono v)
% e 3 ingressi (il primo è w_in, gli ultimi 2 sono u1_cmd, u2_cmd)
% NMEAS = numero di misure (v) lette dal controllore = 3
% NCONT = numero di segnali di controllo (u) inviati agli attuatori = 2

NMEAS = 3;
NCONT = 2;

fprintf('\nAvvio della mu-sintesi (D-K iteration)...\n');


% Opzioni per musyn: MixedMU='on' sfrutta la struttura (reale/complessa) delle incertezze.
opts_musyn = musynOptions('Display','short', 'MixedMU','on', 'MaxIter',10, 'TolPerf',0.01);

[K_rob, CLperf, info_mu] = musyn(P_gen_unc, NMEAS, NCONT, opts_musyn);

K_rob.InputName = {'zs_ddot', 'delta_s', 'zu_ddot'};
K_rob.OutputName = {'u1_cmd', 'u2_cmd'};

fprintf('\n=== Risultati della mu-sintesi ===\n');
fprintf('Ordine del controllore K_rob: %d\n', order(K_rob));
fprintf('Valore finale di mu (Robust Performance bound): %.4f\n', CLperf);

% Analisi della Robust Performance (RP) 
if CLperf < 1
    fprintf('-> SUCCESSO: La Robust Performance e'' GARANTITA (mu < 1)!\n');
else
    fprintf('-> ATTENZIONE: La Robust Performance NON e'' garantita per il caso peggiore (mu >= 1).\n');
    fprintf('   Tuttavia, potrebbe comunque funzionare bene nella pratica.\n');
end
fprintf('==================================\n');


%% RIDUZIONE DELL'ORDINE DEL CONTROLLORE

fprintf('\n=== Riduzione dell''Ordine del Controllore ===\n');

% Visualizzazione dell'importanza (energia) di ogni stato
figure('Name', 'Valori Singolari di Hankel di K_rob', 'Color', 'w');
hsv = hankelsv(K_rob);
bar(hsv);
set(gca, 'YScale', 'log');
title('Valori Singolari di Hankel (Importanza degli stati)');
xlabel('Numero dello stato'); ylabel('Energia (Scala logaritmica)');
grid on;

% Scelta ordine ridotto
ordine_ridotto = 21; 
K_red = balred(K_rob, ordine_ridotto);

K_red.InputName = {'zs_ddot', 'delta_s', 'zu_ddot'};
K_red.OutputName = {'u1_cmd', 'u2_cmd'};

fprintf('Controllore ridotto da %d a %d stati.\n', order(K_rob), order(K_red));

% Verifica controllore ridotto (K_red)
CL_unc_red = lft(P_gen_unc, K_red);

% RP per K_red
opts_rob = robOptions('Sensitivity','Off', 'Display', 'off');
[perfmarg_red, ~, ~] = robgain(CL_unc_red, 1, opts_rob);

mu_RP_red = 1 / perfmarg_red.LowerBound;
fprintf('Picco mu_RP (Controllore Ridotto (K_red) a %d stati): %.4f\n', ordine_ridotto, mu_RP_red);

if perfmarg_red.LowerBound > 1
    fprintf('-> SUCCESSO: La Robust Performance e'' ANCORA GARANTITA (mu < 1)!\n');
else
    fprintf('-> ATTENZIONE: Rimossi troppi stati, RP non più garantita (mu >= 1).\n');
end
fprintf('==============================================\n');


%% ANALISI IN FREQUENZA (Bode - Passivo vs Robusto)
fprintf('\n=== Generazione Grafici di Bode (Analisi Fisica) ===\n');
% Vettore di frequenze
w_vec = logspace(-1, 3, 1000); 

% Impianto nominale 
Plant_nominale = P_esteso.NominalValue;

% F.d.T. Passiva (solo da w_in a comfort e tenuta per i plot)
G_passiva = Plant_nominale({'zs_ddot', 'delta_t'}, 'w_in');

% Connessione a ciclo chiuso tramite connect (gestisce automaticamente le 3 misure)
G_a_rob = connect(Plant_nominale, -K_rob, 'w_in', {'zs_ddot', 'delta_t'});
G_a_red = connect(Plant_nominale, -K_red, 'w_in', {'zs_ddot', 'delta_t'});

% Estrazione delle singole Funzioni di Trasferimento
% Comfort (Canale 1: w_in -> zs_ddot)
G_p_comfort   = G_passiva(1, 1);
G_rob_comfort = G_a_rob(1, 1);
G_red_comfort = G_a_red(1, 1);

% Tenuta di Strada (Canale 2: w_in -> delta_t)
G_p_tenuta   = G_passiva(2, 1);
G_rob_tenuta = G_a_rob(2, 1);
G_red_tenuta = G_a_red(2, 1);

% --- PLOT 1: COMFORT VIBRAZIONALE ---
figure('Name', 'Bode - Comfort Vibrazionale', 'Color', 'w');
bodemag(G_p_comfort, 'k--', G_rob_comfort, 'r', G_red_comfort, 'b:', w_vec);
grid on;
legend('Passiva', 'K\_rob (Full Order)', 'K\_red (Ridotto)', 'Location', 'southwest');
title('Amplificazione Disturbo: w_{in} \rightarrow Accelerazione Cassa (Comfort)');

% --- PLOT 2: TENUTA DI STRADA ---
figure('Name', 'Bode - Tenuta di Strada', 'Color', 'w');
bodemag(G_p_tenuta, 'k--', G_rob_tenuta, 'r', G_red_tenuta, 'b:', w_vec);
grid on;
legend('Passiva', 'K\_rob (Full Order)', 'K\_red (Ridotto)', 'Location', 'southwest');
title('Amplificazione Disturbo: w_{in} \rightarrow \delta_t (Tenuta di Strada)');


%% MU-ANALISI (NP, RS, RP)

fprintf('\n=== Avvio Mu-Analisi per K_rob e K_red ===\n');

omega = logspace(-1, 3, 100);
W_perf = blkdiag(WP, Wu);

[Np_rob, Np_unw_rob, margini_rob] = esegui_mu_analisi(K_rob, 'K_rob (Full-Order)', P_esteso, W_perf, omega);
[Np_red, Np_unw_red, margini_red] = esegui_mu_analisi(K_red, 'K_red (Ridotto)', P_esteso, W_perf, omega);

% Summary 
fprintf('\n=========================================================\n');
fprintf(' === SUMMARY FINALE DEI MARGINI ===\n');
fprintf('=========================================================\n');
stampa_summary('K_rob (Full-Order)', margini_rob);
stampa_summary('K_red (Ridotto)', margini_red);

% Calcolo curve in frequenze (per PLOT) 
fprintf('\n=== Calcolo curve in frequenza per i Plot (attendere...) ===\n');
[mu_NP_rob, mu_RS_rob, mu_RP_rob] = calcola_mu_frequenza(Np_rob, Np_unw_rob, omega);
[mu_NP_red, mu_RS_red, mu_RP_red] = calcola_mu_frequenza(Np_red, Np_unw_red, omega);

fig_main = figure('Name','Mu-Analisi: Confronto Controllori', 'Position', [100, 100, 800, 500], 'Color', 'w');
tgroup = uitabgroup('Parent', fig_main);

% --- PLOT 1: K_rob (Full-Order) ---
tab1 = uitab('Parent', tgroup, 'Title', 'K_rob (Full-Order)');
ax1 = axes('Parent', tab1);
semilogx(ax1, omega, mu_NP_rob(:), 'b-.', 'LineWidth', 1.5); hold(ax1, 'on');
semilogx(ax1, omega, mu_RS_rob(:), 'g--', 'LineWidth', 1.5);
semilogx(ax1, omega, mu_RP_rob(:), 'r', 'LineWidth', 1.5);
semilogx(ax1, omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold(ax1, 'off'); grid(ax1, 'on');
title(ax1, 'Mu-Analisi: K_{rob} (Full-Order)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax1, 'Frequenza [rad/s]', 'FontSize', 10);
ylabel(ax1, 'Valore Singolare \mu', 'FontSize', 10);
legend(ax1, '\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia (\mu = 1)', 'Location', 'best');
ylim(ax1, [0, 1.8]); % Adatta questo limite se le curve lo superano

% --- PLOT 2: K_red (Ridotto) ---
tab2 = uitab('Parent', tgroup, 'Title', 'K_red (Ridotto)');
ax2 = axes('Parent', tab2);
semilogx(ax2, omega, mu_NP_red(:), 'b-.', 'LineWidth', 1.5); hold(ax2, 'on');
semilogx(ax2, omega, mu_RS_red(:), 'g--', 'LineWidth', 1.5);
semilogx(ax2, omega, mu_RP_red(:), 'r', 'LineWidth', 1.5);
semilogx(ax2, omega, ones(size(omega)), 'k-', 'LineWidth', 1.5); 
hold(ax2, 'off'); grid(ax2, 'on');
title(ax2, 'Mu-Analisi: K_{red} (Ridotto)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel(ax2, 'Frequenza [rad/s]', 'FontSize', 10);
ylabel(ax2, 'Valore Singolare \mu', 'FontSize', 10);
legend(ax2, '\mu_{NP}', '\mu_{RS}', '\mu_{RP}', 'Soglia (\mu = 1)', 'Location', 'best');
ylim(ax2, [0, 1.8]); % Adatta questo limite se le curve lo superano



%% FUNZIONI DI SUPPORTO 

function [Np, Np_unweighted, margini] = esegui_mu_analisi(K_ctrl, nome_ctrl, P_esteso, W_perf, omega)
    fprintf('\n---------------------------------------------------------\n');
    fprintf(' MU-ANALYSIS: %s \n', nome_ctrl);
    fprintf('---------------------------------------------------------\n');
    
    K_neg = -K_ctrl;
    if size(K_neg, 2) == 4
        K_neg.u = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'}; 
    else
        K_neg.u = {'zs_ddot', 'delta_s', 'zu_ddot'}; 
    end
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
    fprintf(' NP: %s (Picco mu = %.4f)\n', ternary(NP_ok, 'OK', 'FAIL'), muNPinf);
    
    % Robust Stability (RS)
    opts_rob = robOptions('Display', 'off', 'Sensitivity', 'on'); 
    [sm, ~, info_RS] = robstab(Np_unweighted, opts_rob);
    mu_RS_val = 1 / sm.LowerBound;
    RS_ok = mu_RS_val < 1;
    fprintf(' RS: %s (Picco mu = %.4f)\n', ternary(RS_ok, 'OK', 'FAIL'), mu_RS_val);
    
    fprintf('     Sensibilita'' di mu alle singole incertezze:\n');
    disp(info_RS.Sensitivity);
    
    % Robust Performance (RP)
    pm = robgain(Np, 1, robOptions('Display', 'off'));
    mu_RP_val = 1 / pm.LowerBound;
    RP_ok = mu_RP_val < 1;
    fprintf(' RP: %s (Picco mu = %.4f)\n', ternary(RP_ok, 'OK', 'FAIL'), mu_RP_val);
    
    % Salvataggio margini per il summary
    margini.NS = NS_ok;
    margini.NP = NP_ok; 
    margini.muNPinf = muNPinf;
    margini.RS = RS_ok; 
    margini.muRSinf = mu_RS_val;
    margini.RP = RP_ok; 
    margini.muRPinf = mu_RP_val;
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
    fprintf('%-18s | NS: %-4s | NP: %-4s (mu=%.2f) | RS: %-4s (mu=%.2f) | RP: %-4s (mu=%.2f)\n', ...
        nome, ...
        ternary(m.NS, 'OK', 'FAIL'), ...
        ternary(m.NP, 'OK', 'FAIL'), m.muNPinf, ...
        ternary(m.RS, 'OK', 'FAIL'), m.muRSinf, ...
        ternary(m.RP, 'OK', 'FAIL'), m.muRPinf);
end

function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end







