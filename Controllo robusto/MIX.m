%% Mixed-Sensitivity H-infinity Synthesis: MIMO Quarter-Car
%
% Design objective: minimize ||[WP*S; Wu*KS]||_inf < gamma
% Basato fedelmente sul template del docente.

clear all; close all; clc;

% --- 1. CARICAMENTO PARAMETRI E CREAZIONE IMPIANTO INCERTO ---
run('quarter_car_parameters.m');
run('uncertain_Plant.m');

s = tf('s');

%% =========================================================
%  MIMO PLANT (Quarter-Car)
%% =========================================================
% Parametri
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

C = [ -k_s/m_s, -b_s/m_s, 0, b_s/m_s;                   % y1: zs_ddot (Comfort)
       1, 0, 0, 0;                                      % y2: delta_s (Livellamento)
       0, 0, 1, 0;                                      % y3: delta_t (Tenuta strada)
       k_s/m_u, b_s/m_u, -k_t/m_u, -(b_s+b_t)/m_u ];    % y4: zu_ddot

D = [ 1/m_s,   0,      0;
      0,       0,      0;
      0,       0,      0;
     -1/m_u,   1/m_u,  b_t/m_u ];

% G = ss(A, B, C, D);
% G.InputName  = {'u1', 'u2', 'w_dist'};
% G.OutputName = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};
% 
% % Impianto di sintesi (visto dal controllore): solo u1 e u2
% G_uy = G(:, 1:2);
% 
% % Analisi Poli e Zeri
% Poles_MIMO = pole(G_uy);
% Zeros_MIMO = tzero(G_uy);
% fprintf('Transmission poles:\n'); disp(Poles_MIMO);
% fprintf('Transmission zeros:\n'); disp(Zeros_MIMO);

%% =========================================================
%  PERFORMANCE AND CONTROL WEIGHTS
%% =========================================================
M = 2.0; 

% 1. Comfort (zs_ddot): IL SEGRETO E' IL FILTRO PASSA-BANDA. 
% L'accelerazione a regime è SEMPRE zero. Il peso deve essere zero a w=0, 
% altrimenti l'algoritmo impazzisce cercando di attenuare un segnale già nullo!
% Usiamo un passa-banda centrato tra 5 e 50 rad/s.
% wP1 = 0.4 * (s / (s + 1)) * (50 / (s + 5)); 

% Filtro passa-banda Risonante (Stretto e mirato)
w_n = 3;        % Centro della valle (3 rad/s, la frequenza del tuo test!)
zeta = 0.5;     % Larghezza: più è piccolo, più la valle è stretta (0.2 è un "cecchino")
Gain = 2.5;     % Profondità della valle (quanto vogliamo schiacciare l'errore)

wP1 = Gain * (2 * zeta * w_n * s) / (s^2 + 2 * zeta * w_n * s + w_n^2);

% 2. Autolivellamento (delta_s): Rallentiamo la banda a 0.1 rad/s.
% Se gli chiedi di livellare l'auto in 1 secondo (wB=1), richiede 100.000 N.
% Con wB = 0.1 (circa 10 secondi), lo sforzo crolla nei limiti fisici.
% A_track = 1e-4; 
% wB_track = 0.05; 
% wP2 = (s/M + wB_track) / (s + wB_track*A_track);
wP2=0.01; 

% 3. Tenuta di Strada (delta_t)
% Banda passante da 15 a 20 rad/s 
% A_road = 10.0;  
% wB_road = 20.0; 
% wP3 = (s/M + wB_road) / (s + wB_road*A_road);
wP3 = 0.01;

% 4. Ruota (zu_ddot): Peso costante di relax
wP4 = 0.01;

WP = blkdiag(wP1, wP2, wP3, wP4);

% Sforzo di controllo (Wu): limite fisso a 30000 N (scalato)
wu_LF = 1/40000;
wu_HF = 1/1000;     % Multa salatissima per le reazioni nervose
w_taglio = 13;      % Frequenza di taglio (inizia a frenare dopo i 2-3 Hz)

% Creazione del filtro passa-alto per il peso
wu = wu_HF * (s + w_taglio * (wu_LF/wu_HF)) / (s + w_taglio);
Wu = blkdiag(wu, wu);


% --- ROBUSTEZZA (WT) ---
% e garantire la Stabilità Robusta (RS) contro le incertezze.
% wT_base = 2 * (s + 20) / (0.01*s + 200);
% WT = blkdiag(wT_base, wT_base, wT_base, wT_base);
WT = [];

%% Costruzione plant

Plant_nom = ss(A,B,C,D);
Plant_nom.InputName  = {'u1_force','u2_force','w_dist'};
Plant_nom.OutputName = {'zs_ddot','delta_s','delta_t','zu_ddot'};

Sv1 = sumblk('v1 = -zs_ddot');
Sv2 = sumblk('v2 = -delta_s');
Sv3 = sumblk('v3 = -delta_t');
Sv4 = sumblk('v4 = -zu_ddot');

WP = tf(WP);
WP.InputName  = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};
WP.OutputName = {'z_p1', 'z_p2', 'z_p3', 'z_p4'};

Wu = tf(Wu);
Wu.InputName  = {'u1_cmd', 'u2_cmd'};
Wu.OutputName = {'z_u1', 'z_u2'};

Actuator1_nom = G_a1_nom; Actuator1_nom.InputName='u1_cmd'; Actuator1_nom.OutputName='u1_force';
Actuator2_nom = G_a2_nom; Actuator2_nom.InputName='u2_cmd'; Actuator2_nom.OutputName='u2_force';

P_esteso_nom = connect(Plant_nom, Actuator1_nom, Actuator2_nom, G_d, ...
                        {'w_in','u1_cmd','u2_cmd'}, ...
                        {'zs_ddot','delta_s','delta_t','zu_ddot'});

P_gen_nom = connect(P_esteso_nom, WP, Wu, Sv1, Sv2, Sv3, Sv4, ...
                     {'w_in','u1_cmd','u2_cmd'}, ...
                     {'z_p1','z_p2','z_p3','z_p4','z_u1','z_u2','v1','v2','v3','v4'});




%% =========================================================
%  H-INFINITY SYNTHESIS via mixsyn()
%  Nota: usiamo [] per WT come nel file del professore.
%% =========================================================
G_uy = P_esteso_nom(:, {'u1_cmd','u2_cmd'});
[K_mixsyn, CL, gamma, ~] = mixsyn(G_uy, WP, Wu, WT);
K_mixsyn = zpk(K_mixsyn);

fprintf('\nAchieved H-inf norm gamma = %.4f\n', gamma);
fprintf('Controller order: %d\n', order(K_mixsyn));

% Closed-loop sensitivity functions
S_cl  = inv(eye(4) + G_uy*K_mixsyn);       % output sensitivity   (I+GK)^-1
KS_cl = K_mixsyn * inv(eye(4) + G_uy*K_mixsyn);   % control sensitivity  K*(I+GK)^-1
T_cl  = G_uy*K_mixsyn * inv(eye(4)+G_uy*K_mixsyn);% comp. sensitivity    GK*(I+GK)^-1

%% =========================================================
%  PERFORMANCE VERIFICATION: Singular Value Plots
%% =========================================================
figure('Name','Sensitivity vs Bound','Position',[50 50 700 450]);
sigmaplot(zpk(inv(WP)), zpk(minreal(S_cl)));
legend('1/W_P (performance bound)', '\sigma(S) (achieved)');
title('Sensitivity S: achieved vs bound 1/W_P'); grid on;

figure('Name','Control Sensitivity vs Bound','Position',[50 50 700 450]);
sigmaplot(zpk(inv(Wu)), zpk(minreal(KS_cl)));
legend('1/W_u (control bound)', '\sigma(KS) (achieved)');
title('Control sensitivity KS: achieved vs bound 1/W_u'); grid on;

%% =========================================================
%  PERFORMANCE VERIFICATION: H-infinity norms
%% =========================================================
[norm_WPS,  w_peak_WPS ] = hinfnorm(minreal(WP  * S_cl));
[norm_WuKS, w_peak_WuKS] = hinfnorm(minreal(Wu  * KS_cl));

fprintf('\n=== H-infinity Performance Norms ===\n');
fprintf('||WP*S||_inf   = %.4f  (peak at w = %.2f rad/s)  %s\n', ...
        norm_WPS,  w_peak_WPS,  check(norm_WPS));
fprintf('||Wu*KS||_inf  = %.4f  (peak at w = %.2f rad/s)  %s\n', ...
        norm_WuKS, w_peak_WuKS, check(norm_WuKS));

% %% =========================================================
% %  CLOSED-LOOP STEP RESPONSE (Test Fisici Reali)
% %% =========================================================
% t = 0:0.01:15;
% 
% % 1. TEST DI AUTOLIVELLAMENTO (Gradino di Carico F_load = 1000 N)
% % Il carico entra opposto all'attuatore u1: G_load = -G_uy(:,1)
% T_load2y = S_cl * (-G_uy(:,1));
% T_load2u = -KS_cl * (-G_uy(:,1));
% 
% r_load = 1000 * ones(length(t), 1);
% [y_load, ~] = lsim(minreal(zpk(T_load2y)), r_load, t);
% [u_load, ~] = lsim(minreal(zpk(T_load2u)), r_load, t);
% 
% figure('Name','Test 1: Autolivellamento (Carico 1000 N)','Position',[50 50 900 600]);
% subplot(2,1,1);
% plot(t, y_load(:,2), 'LineWidth', 1.5, 'Color', 'b');
% xlabel('t [s]'); ylabel('\delta_s [m]');
% title('Autolivellamento: Corsa sospensione (deve riassestarsi a 0)'); grid on;
% subplot(2,1,2);
% plot(t, u_load(:,1), 'LineWidth', 1.5); hold on;
% plot(t, u_load(:,2), 'LineWidth', 1.5);
% xlabel('t [s]'); ylabel('Forza Attuatori [N]');
% legend('u_1','u_2'); title('Sforzo di controllo'); grid on;
% 
% % 2. TEST DELLA BUCA STRADALE (Gradino di posizione = Impulso di velocità)
% % L'ingresso 3 di G è la velocità della strada
% T_w2y = S_cl * G(:,3);
% 
% % Simuliamo un impulso scalato per 5 cm (0.05)
% [y_bump, t_bump] = impulse(minreal(zpk(T_w2y)), t);
% y_bump = y_bump * 0.05;
% 
% figure('Name','Test 2: Buca Stradale (0.05 m)','Position',[100 100 900 600]);
% subplot(2,1,1);
% plot(t_bump, y_bump(:,1), 'LineWidth', 1.5, 'Color', 'k');
% xlabel('t [s]'); ylabel('Acc. Cassa [m/s^2]');
% title('Comfort: Accelerazione cassa dopo buca'); grid on;
% subplot(2,1,2);
% plot(t_bump, y_bump(:,3), 'LineWidth', 1.5, 'Color', 'g');
% xlabel('t [s]'); ylabel('\delta_t [m]');
% title('Tenuta di Strada: Deformazione pneumatico'); grid on;

%% =========================================================
%%  SINTESI ALTERNATIVA via hinfsyn()
%% =========================================================
fprintf('\n=== Sintesi Alternativa con hinfsyn ===\n');

% Impianto Generalizzato P (augw)
% augw(Impianto, W1_performance, W2_control, W3_complementary)
% NOTA BENE: Usiamo [] per W3, esattamente come in mixsyn, per coerenza.
% P_gen = augw(G_uy, WP, Wu, WT);

% hinfsyn(Impianto_Generalizzato, numero_misure, numero_controlli)
% Abbiamo 4 uscite lette dal controllore (y) e 2 ingressi di controllo (u)
NMEAS = 4;
NCONT = 2;

opts = hinfsynOptions('Display','on');
[K_hinf, CL_hinf, gamma_hinf, info_hinf] = hinfsyn(P_gen_nom, NMEAS, NCONT, [0.3, 2], opts);
K_hinf = zpk(K_hinf);

fprintf('Achieved H-inf norm gamma (hinfsyn) = %.4f\n', gamma_hinf);
fprintf('Controller order (hinfsyn): %d\n', order(K_hinf));

% Confronto mixsyn e hinfsyn 
% (Dovrebbero essere identici a meno di tolleranze numeriche)
if abs(gamma - gamma_hinf) < 1e-3
    fprintf('-> SUCCESSO: mixsyn e hinfsyn hanno prodotto lo stesso gamma!\n');
else
    fprintf('-> DIFFERENZA: gamma_mixsyn = %.4f, gamma_hinfsyn = %.4f\n', gamma, gamma_hinf);
end

%% =========================================================
%% H-INFINITY CON STRUTTURA FISSA (PI Industriale per Comfort)
%% Confronto leale con hinfsyn
%% =========================================================
fprintf('\n=== Sintesi H-infinity con PI Strutturato (Comfort) ===\n');

% 1. IL GUINZAGLIO: Limite fisso per proteggere gli attuatori
% Wu_strict = blkdiag(1/45000, 1/45000); 
% Wu_strict = tf(Wu_strict);
% Wu_strict.InputName  = {'u1_cmd', 'u2_cmd'};
% Wu_strict.OutputName = {'z_u1', 'z_u2'};

% 2. IMPIANTO GENERALIZZATO (Lo stesso identico usato per hinfsyn!)
% P_gen_struct = connect(P_esteso_nom, WP, Wu, Sv1, Sv2, Sv3, Sv4, ...
%                      {'w_in','u1_cmd','u2_cmd'}, ...
%                      {'z_p1','z_p2','z_p3','z_p4','z_u1','z_u2','v1','v2','v3','v4'});

% 3. COSTRUZIONE DELLA STRUTTURA (Azione di tipo Skyhook)
% A. Proporzionale: Matrice 2x4 (legge tutti i sensori)
Kp = realp('Kp', zeros(2,4)); 

% B. Integrale: Agisce sull'accelerazione (zs_ddot). 
% Integrare l'accelerazione fornisce un feedback di velocità (effetto Skyhook).
Ki_base = realp('Ki_base', [0; 0]); 
Selector_I = [1, 0, 0, 0]; % Accende SOLO il canale 1 (zs_ddot)
I_action = (Ki_base / (s + 0.001)) * Selector_I;

% C. Assemblaggio
K_custom = Kp + I_action;

% D. Filtro Passa-Basso a 150 rad/s (per rendere il controllore realistico e proprio)
wf = 39; 
LPF_single = tf(wf, [1, wf]);
LPF_matrix = blkdiag(LPF_single, LPF_single);

K_struct = LPF_matrix * K_custom;

% 4. OTTIMIZZAZIONE HINFSTRUCT
CL0 = lft(P_gen_nom, K_struct);

rng('default'); % Per riproducibilità
opt = hinfstructOptions('Display', 'final', 'RandomStart', 15);
[CL_pid, gamma_pid, info_pid] = hinfstruct(CL0, opt);

% Estrazione del controllore tunato
tuned_params = getBlockValue(CL_pid);
K_PID_tuned = minreal(ss(replaceBlock(K_struct, tuned_params)));

fprintf('\nNorma H-inf (PI Strutturato Comfort) -> gamma = %.4f\n', gamma_pid);

%% =========================================================
%  HELPER FUNCTION
%% =========================================================
function s = check(n)
    if n < 1
        s = '<-- OK (< 1)';
    else
        s = '<-- VIOLATED (> 1)';
    end
end

% %% =========================================================
% %% H-INFINITY CON STRUTTURA FISSA (PI Industriale Realistico)
% %% Confronto con mixsyn 
% %% =========================================================
% fprintf('\n=== Sintesi H-infinity con PI Strutturato ===\n');
% 
% % 1. IL GUINZAGLIO: 
% % ATTENZIONE: Per fare un confronto leale con mixsyn, usa lo stesso 
% % limite che hai messo in Wu per mixsyn (es. 1/45000). 
% Wu_strict = blkdiag(1/45000, 1/45000); 
% P_gen_struct = augw(G_uy, WP, Wu_strict, WT);
% 
% % 2. COSTRUZIONE DELLA STRUTTURA (Focus su Dinamica e Comfort)
% 
% % A. Proporzionale: Matrice 2x4 (2 attuatori x 4 sensori)
% % Togliamo il "Selector_P" che lo accecava. Ora lasciamo che la matrice 
% % Kp legga tutte le 4 variabili: y = [zs_ddot, delta_s, delta_t, zu_ddot]
% Kp = realp('Kp', zeros(2,4)); 
% 
% % B. Integrale: Spostato sull'accelerazione della cassa (zs_ddot)
% % MAGIA FISICA: Integrare l'accelerazione in Simulink genera la velocità. 
% % Questa azione I equivale a uno smorzatore viscoso "Skyhook" ideale!
% Ki_base = realp('Ki_base', [0; 0]); 
% Selector_I = [1, 0, 0, 0]; % Accende SOLO il canale 1 (zs_ddot), spegne il resto
% I_action = (Ki_base / (s + 0.001)) * Selector_I;
% 
% % C. Assemblaggio del controllore
% K_custom = Kp + I_action;
% 
% % D. Filtro Passa-Basso a 50 rad/s (~8 Hz) per simulare il ritardo fisico degli attuatori
% % Questo filtro è vitale per garantire la Robust Stability (RS) nella mu-analisi
% wf = 100; 
% LPF_single = tf(wf, [1, wf]);
% LPF_matrix = blkdiag(LPF_single, LPF_single);
% 
% K_struct = LPF_matrix * K_custom;
% 
% % 3. OTTIMIZZAZIONE
% CL0 = lft(P_gen_struct, K_struct);
% 
% rng('default');
% opt = hinfstructOptions('Display', 'final', 'RandomStart', 10);
% [CL_pid, gamma_pid, info_pid] = hinfstruct(CL0, opt);
% 
% % Estrazione
% tuned_params = getBlockValue(CL_pid);
% K_PID_tuned = minreal(ss(replaceBlock(K_struct, tuned_params)));
% 
% fprintf('\nNorma H-inf (PI Strutturato) -> gamma = %.4f\n', gamma_pid);
% 
% 
% %% =========================================================
% %% H-INFINITY CON STRUTTURA FISSA (PI per Autolivellamento)
% %% =========================================================
% fprintf('\n=== Sintesi H-infinity con PI Strutturato (Autolivellamento) ===\n');
% 
% % 1. DEFINIZIONE PESI SPECIFICI PER IL LIVELLAMENTO
% % Autolivellamento (delta_s): Rallentiamo la banda a 0.05 rad/s.
% % Se gli chiedi di livellare l'auto in 1 secondo (wB=1), richiede 100.000 N.
% % Con wB = 0.05 (circa 20 secondi), lo sforzo crolla nei limiti fisici.
% A_track = 1e-4; 
% wB_track = 0.05; 
% wP2_liv = (s/M + wB_track) / (s + wB_track*A_track);
% 
% % Assembliamo il nuovo peso globale (mantenendo gli altri inalterati)
% WP_liv = blkdiag(wP1, wP2_liv, wP3, wP4);
% 
% % Creazione dell'impianto generalizzato con i pesi di livellamento
% P_gen_liv = augw(G_uy, WP_liv, Wu_strict, WT);
% 
% % 2. COSTRUZIONE DELLA STRUTTURA "ACCECATA" (Solo Posizioni)
% % A. Proporzionale: Matrice 2x2 (2 attuatori x 2 sensori di posizione)
% Kp_liv_base = realp('Kp_liv_base', zeros(2,2));
% 
% % Moltiplichiamo per una matrice che "spegne" le accelerazioni (col 1 e 4)
% % e lascia passare solo delta_s (col 2) e delta_t (col 3)
% Selector_P_liv = [0, 1, 0, 0; 
%                   0, 0, 1, 0];
% Kp_liv = Kp_liv_base * Selector_P_liv; 
% 
% % B. Integrale: Solo per l'autolivellamento (delta_s)
% Ki_liv_base = realp('Ki_liv_base', [0; 0]); 
% Selector_I_liv = [0, 1, 0, 0]; % Accende solo il canale 2 (delta_s)
% I_action_liv = (Ki_liv_base / (s + 0.001)) * Selector_I_liv;
% 
% % C. Assemblaggio del controllore
% K_custom_liv = Kp_liv + I_action_liv;
% 
% % D. Filtro Passa-Basso a 50 rad/s (~8 Hz) 
% wf_liv = 50; 
% LPF_single_liv = tf(wf_liv, [1, wf_liv]);
% LPF_matrix_liv = blkdiag(LPF_single_liv, LPF_single_liv);
% 
% K_struct_liv = LPF_matrix_liv * K_custom_liv;
% 
% % 3. OTTIMIZZAZIONE
% CL0_liv = lft(P_gen_liv, K_struct_liv);
% 
% rng('default');
% opt_liv = hinfstructOptions('Display', 'final', 'RandomStart', 10);
% [CL_pid_liv, gamma_pid_liv, info_pid_liv] = hinfstruct(CL0_liv, opt_liv);
% 
% % Estrazione
% tuned_params_liv = getBlockValue(CL_pid_liv);
% K_PID_tuned_liv = minreal(ss(replaceBlock(K_struct_liv, tuned_params_liv)));
% 
% fprintf('\nNorma H-inf (PI Strutturato Autolivellamento) -> gamma = %.4f\n', gamma_pid_liv);
% 
% 
% %% =========================================================
% %  HELPER FUNCTION
% %% =========================================================
% function s = check(n)
%     if n < 1
%         s = '<-- OK (< 1)';
%     else
%         s = '<-- VIOLATED (> 1)';
%     end
% end