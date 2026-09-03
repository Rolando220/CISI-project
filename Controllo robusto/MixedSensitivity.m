%% Sintesi Mixed-Sensitivity H-infinito (MIMO) 

s = tf('s');

%% MODELLO NOMINALE DEL QUARTER-CAR

m_s = parameters.m_s;
m_u = parameters.m_u;
k_s = parameters.ks0;
b_s  = parameters.bs;
k_t  = parameters.kt;
b_t  = parameters.bt;

A = [ 0, 1, 0, -1;
     -k_s/m_s, -b_s/m_s, 0, b_s/m_s;
      0, 0, 0, 1;
      k_s/m_u, b_s/m_u, -k_t/m_u, -(b_s+b_t)/m_u ];

B = [ 0,       0,      0;
      1/m_s,   0,      0;
      0,       0,     -1;
     -1/m_u,   1/m_u,  b_t/m_u ];

C = [ -k_s/m_s, -b_s/m_s, 0, b_s/m_s;   % y1: zs_ddot
       1, 0, 0, 0;                      % y2: delta_s
       0, 0, 1, 0;                      % y3: delta_t
       k_s/m_u, b_s/m_u, -k_t/m_u, -(b_s+b_t)/m_u ]; % y4: zu_ddot

D = [ 1/m_s,   0,      0;
      0,       0,      0;
      0,       0,      0;
     -1/m_u,   1/m_u,  b_t/m_u ];

G = ss(A, B, C, D);
G.InputName  = {'u1', 'u2','w_dist'};
G.OutputName = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};

%%  ANALISI ZERI E POLI DI TRASMISSIONE 
Poles_MIMO = pole(G);
Zeros_MIMO = tzero(G);
fprintf('=== Analisi Impianto ===\n');
fprintf('Poli di trasmissione:\n'); disp(Poles_MIMO);
fprintf('Zeri di trasmissione:\n'); disp(Zeros_MIMO);

%% PERFORMANCE E PESI 

% Filtro del 1° ordine sulle posizioni (Tracking a regime per delta_s) ---
M  = 2.0;    % Picco massimo tipico per robustezza
wB = 5;      % Banda passante ~1 Hz
A_err = 1e-3;% Errore a regime (0.1%)
wP_track = (s/M + wB) / (s + wB*A_err); 

% Filtro Passa-Banda per il Comfort (Accelerazione zs_ddot) ---
% Normativa ISO 2631: Massima sensibilità umana tra 1 Hz e 10 Hz
f_low  = 1;  % Hz
f_high = 10; % Hz
w_low  = 2 * pi * f_low;  
w_high = 2 * pi * f_high;
Gain_acc = 0.1; %Guadagno nella banda critica (più è alto, più l'algoritmo penalizza l'accelerazione)
% Filtro Passa-Banda
wP_acc = Gain_acc * ( (s/w_low) / (s/w_low + 1) ) * ( 1 / (s/w_high + 1) );

% Assegnazione Pesi 
wP_relax = 0.1;

wP1 = wP_acc;         % zs_ddot: Penalizzata nella banda critica(1-10 Hz)
wP2 = wP_track;       % delta_s: Tracking a regime
wP3 = wP_relax;       % delta_t: Limite base
wP4 = wP_relax;       % zu_ddot: Limite base

WP = blkdiag(wP1, wP2, wP3, wP4);

% Peso sullo sforzo di controllo Wu 
wu_val = 0.05 / 3000; 
Wu = blkdiag(wu_val, wu_val);

% Peso  WT 
% Filtro passa-alto: penalizza T alle alte frequenze (sopra i 50 rad/s)
% A basse frequenze vale ~0.05, ad alte frequenze vale 100.
wT_base = (s + 50) / (0.01*s + 100); 

% Stesso peso WT a tutti e 4 i canali in uscita
WT = blkdiag(wT_base, wT_base, wT_base, wT_base);

fprintf('Pesi impostati');

%%  H-INFINITY SYNTHESIS via mixsyn()

% Estrazione della parte di impianto governata solo dagli attuatori (u1, u2)
G_uy = G(:, 1:2); 

[K_mixsyn, CL, gamma, info] = mixsyn(G_uy, WP, Wu, WT);
K_mixsyn = zpk(K_mixsyn);

fprintf('\n=== H-infinity Synthesis ===\n');
fprintf('Norma H-inf -> gamma = %.4f\n', gamma);
fprintf('Ordine Controllore: %d\n', order(K_mixsyn));

% Funzioni a ciclo chiuso (basate sull'impianto di controllo G_uy)
S_cl  = inv(eye(4) + G_uy*K_mixsyn);           % output sensitivity
KS_cl = K_mixsyn * inv(eye(4) + G_uy*K_mixsyn);       % control sensitivity
T_cl  = G_uy*K_mixsyn * inv(eye(4) + G_uy*K_mixsyn);  % complementary sensitivity

% CONTROLLO DELLE PERFORMANCE (H-infinity norms)
[norm_WPS,  w_peak_WPS ] = hinfnorm(minreal(WP  * S_cl));
[norm_WuKS, w_peak_WuKS] = hinfnorm(minreal(Wu  * KS_cl));
[norm_T,    w_peak_T   ] = hinfnorm(minreal(T_cl));

fprintf('\n=== H-infinity Performance Norms ===\n');
fprintf('||WP*S||_inf   = %.4f  (peak at w = %.2f rad/s)\n', norm_WPS,  w_peak_WPS);
fprintf('||Wu*KS||_inf  = %.4f  (peak at w = %.2f rad/s)\n', norm_WuKS, w_peak_WuKS);
fprintf('||T||_inf      = %.4f  (peak at w = %.2f rad/s)\n', norm_T,    w_peak_T);

% VERIFICA STABILITÀ NOMINALE (NS) via loopsens
% Si calcolano tutte le fdt interne (S, T, KS, SG, ecc.)
L_sens = loopsens(G_uy, K_mixsyn);

% Verifica Poli tutti a parte reale negativa 
if all(real(pole(L_sens.So)) < 0) && all(real(pole(L_sens.To)) < 0) && ...
   all(real(pole(L_sens.Si)) < 0) && all(real(pole(L_sens.Ti)) < 0) && ...
   all(real(pole(L_sens.PSi)) < 0) && all(real(pole(L_sens.CSo)) < 0)
    fprintf('\n-> Stabilità Nominale (NS) CONFERMATA: Nessuna cancellazione instabile.\n');
else 
    fprintf('\n-> ATTENZIONE: Il sistema NON è Nominalmente Stabile!\n');
end


% VERIFICA PERFORMANCE: Singular Value Plots
% figure('Name','Sensitivity vs Bound','Position',[50 50 700 450]);
% sigmaplot(zpk(inv(WP)), zpk(minreal(S_cl)));
% legend('1/W_P (performance bound)', '\sigma(S) (achieved)');
% title('Sensitivity S: achieved vs bound 1/W_P'); grid on;
% 
% figure('Name','Control Sensitivity vs Bound','Position',[50 50 700 450]);
% sigmaplot(zpk(inv(Wu)), zpk(minreal(KS_cl)));
% legend('1/W_u (control bound)', '\sigma(KS) (achieved)');
% title('Control sensitivity KS: achieved vs bound 1/W_u'); grid on;


% CLOSED-LOOP STEP RESPONSE
t = 0:0.01:5;
% Gradino unitario come riferimento alla sola corsa della sospensione (canale 2)
r = zeros(length(t), 4);
r(:, 2) = 0.05; % Gradino di 5 cm 

[y_out, t_out] = lsim(minreal(zpk(T_cl)), r, t);
[u_out, ~    ] = lsim(minreal(zpk(KS_cl)), r, t);

figure('Name','Closed-Loop Step Response','Position',[50 50 900 600]);
subplot(2,1,1);
plot(t_out, y_out(:,2), 'LineWidth', 1.5); hold on;
yline(0.05, 'k--');
xlabel('t [s]'); ylabel('\delta_s [m]');
title('Output y: Inseguimento gradino (5 cm) su \delta_s'); grid on;

subplot(2,1,2);
plot(t_out, u_out(:,1), 'LineWidth', 1.5); hold on;
plot(t_out, u_out(:,2), 'LineWidth', 1.5);
yline(3000,'r--','Max Force'); yline(-3000,'r--');
xlabel('t [s]'); ylabel('Forza Attuatori [N]');
legend('u_1','u_2');
title('Control effort u'); grid on;


%%  H-INFINITY SYNTHESIS via hinfsyn()

fprintf('\n=== Sintesi Alternativa con hinfsyn ===\n');

% Impianto Generalizzato P (augw)
% augw(Impianto, W1_performance, W2_control, W3_complementary)
P_gen = augw(G_uy, WP, Wu, WT);

% hinfsyn(Pianto_Generalizzato, numero_misure, numero_controlli)
% 4 misure (y) e 2 controlli (u1, u2)
NMEAS = 4;
NCONT = 2;

[K_hinf, CL_hinf, gamma_hinf, info_hinf] = hinfsyn(P_gen, NMEAS, NCONT);
K_hinf = zpk(K_hinf);

fprintf('Achieved H-inf norm gamma (hinfsyn) = %.4f\n', gamma_hinf);
fprintf('Controller order (hinfsyn): %d\n', order(K_hinf));

% Confronto mixsyn e hinfsyn 
if abs(gamma - gamma_hinf) < 1e-4
    fprintf('-> SUCCESSO: mixsyn e hinfsyn hanno prodotto lo stesso gamma!\n');
else
    fprintf('-> DIFFERENZA: gamma_mixsyn = %.4f, gamma_hinfsyn = %.4f\n', gamma, gamma_hinf);
end




%% H-INFINITY CON STRUTTURA FISSA via hinfstruct() (PI Filtrato)
fprintf('\n=== Sintesi H-infinity con hinfstruct  ===\n');

% Wu (limite attuatori 3000 N ) 
Wu_struct = blkdiag(1/3000, 1/3000); 

% Definizione Parametri ( P e I)
Kp = realp('Kp', zeros(2,4));
Ki = realp('Ki', zeros(2,4));

% Struttura PI con Integratore "Leaky" (polo in -0.001) 
K_PI = Kp + Ki/(s + 0.001);

% Filtro Passa-Basso del 1° Ordine (garantisce D = 0 ) 
% Frequenza 150 rad/s (~24 Hz) per non rovinare la fase utile
wf = 150; 
LPF_single = tf(wf, [1, wf]);
LPF_matrix = blkdiag(LPF_single, LPF_single);

% Struttura finale: Il PI attraverso il filtro
K_struct = LPF_matrix * K_PI;

% Impianto Generalizzato
P_gen_struct = augw(G_uy, WP, Wu_struct, WT);
CL0 = lft(P_gen_struct, K_struct);

% hinfstruct
rng('default');
opt = hinfstructOptions('Display', 'final', 'RandomStart', 10);
[CL_pid, gamma_pid, info_pid] = hinfstruct(CL0, opt);

% Estrazione del controllore in State-Space
tuned_params = getBlockValue(CL_pid);
K_PID_tuned = ss(replaceBlock(K_struct, tuned_params));
K_PID_tuned = minreal(K_PID_tuned);

fprintf('\n Norma H-inf (hinfstruct) -> gamma = %.4f\n', gamma_pid);
fprintf('Ordine Controllore (hinfstruct): %d\n', order(K_PID_tuned));

