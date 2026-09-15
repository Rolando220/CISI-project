%% =========================================================================
%  PUNTO 3: MU-SYNTHESIS PER SOSPENSIONE QUARTER-CAR
% =========================================================================
clear all; close all; clc;

% --- 1. CARICAMENTO PARAMETRI E CREAZIONE IMPIANTO INCERTO ---
run('quarter_car_parameters.m');
run('uncertain_Plant.m');

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

%% =========================================================
%  PERFORMANCE AND CONTROL WEIGHTS
%% =========================================================
M = 2.0; 

% 1. Comfort (zs_ddot): IL SEGRETO E' IL FILTRO PASSA-BANDA. 
% L'accelerazione a regime è SEMPRE zero. Il peso deve essere zero a w=0, 
% altrimenti l'algoritmo impazzisce cercando di attenuare un segnale già nullo!
% Usiamo un passa-banda centrato tra 5 e 50 rad/s.
wP1 = 1.2 * (s / (s + 5)) * (50 / (s + 50)); 

% 2. Autolivellamento (delta_s): Rallentiamo la banda a 0.1 rad/s.
% Se gli chiedi di livellare l'auto in 1 secondo (wB=1), richiede 100.000 N.
% Con wB = 0.1 (circa 10 secondi), lo sforzo crolla nei limiti fisici.
% A_track = 1e-4; 
% wB_track = 0.05; 
% wP2 = (s/M + wB_track) / (s + wB_track*A_track);
wP2=0.01; 

% 3. Tenuta di Strada (delta_t)
% Banda passante da 15 a 20 rad/s 
A_road = 10.0;  
wB_road = 20.0; 
wP3 = (s/M + wB_road) / (s + wB_road*A_road);

% 4. Ruota (zu_ddot): Peso costante di relax
wP4 = 0.01;

WP = blkdiag(wP1, wP2, wP3, wP4);

% Sforzo di controllo (Wu): limite fisso a 30000 N (scalato)
wu = 1/40000;
Wu = blkdiag(wu, wu);


% --- ROBUSTEZZA (WT) ---
% e garantire la Stabilità Robusta (RS) contro le incertezze.
wT_base = 2 * (s + 20) / (0.01*s + 200);
WT = blkdiag(wT_base, wT_base, wT_base, wT_base);

% --- 3. COSTRUZIONE GENERALIZED PLANT PER LA SINTESI ---
% 1. Assegniamo i nomi agli I/O dei pesi di performance come fa il prof
WP = tf(WP);
WP.InputName  = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};
WP.OutputName = {'z_p1', 'z_p2', 'z_p3', 'z_p4'};

Wu = tf(Wu);
Wu.InputName  = {'u1_cmd', 'u2_cmd'};
Wu.OutputName = {'z_u1', 'z_u2'};

% Nota: Omettiamo il peso WT perché, come si vede nella teoria della mu-sintesi,
% la robustezza è già garantita dai blocchi di incertezza dentro P_esteso.

% 2. Creiamo i nodi sommatori per il segnale di errore 'v'
% Il comando musyn si aspetta una retroazione positiva (u = K*v). 
% Per ottenere la nostra retroazione negativa (u = -K*y), usiamo sumblk 
% per invertire il segno delle misure in uscita, esattamente come fa il prof.
Sv1 = sumblk('v1 = -zs_ddot');
Sv2 = sumblk('v2 = -delta_s');
Sv3 = sumblk('v3 = -delta_t');
Sv4 = sumblk('v4 = -zu_ddot');

% 3. Assembliamo il Generalized Plant tramite connect
% Ingressi (3): disturbo stradale reale (w_in) e comandi attuatori (u)
% Uscite (10): performance pesate (z) e misure per il controllore (v)
P_gen_unc = connect(P_esteso, WP, Wu, Sv1, Sv2, Sv3, Sv4, ...
                    {'w_in', 'u1_cmd', 'u2_cmd'}, ...
                    {'z_p1', 'z_p2', 'z_p3', 'z_p4', 'z_u1', 'z_u2', 'v1', 'v2', 'v3', 'v4'});

% --- 4. MU-SYNTHESIS (DK-ITERATION VIA MUSYN) ---
% Impostazioni prese dallo script del professore
NMEAS = 4;
NCONT = 2;
opts_musyn = musynOptions('Display','short', 'MixedMU','on', 'MaxIter',10, 'TolPerf',0.01);

fprintf('\n=========================================================\n');
fprintf(' INIZIO MU-SYNTHESIS (Attendere, puo'' richiedere minuti...) \n');
fprintf('=========================================================\n');

[K_mu, CLperf, info_mu] = musyn(P_gen_unc, NMEAS, NCONT, opts_musyn);

fprintf('\n-> Sintesi conclusa. Valore di mu (RP bound) = %.4f\n\n', CLperf);

if CLperf > 1
    warning('Il valore di mu e'' > 1. Il sistema NON garantisce le prestazioni robuste. Abbassare "Gain_acc" e riprovare.');
end

% Conversione in zpk/ss per simulink
K_mu = minreal(ss(K_mu));

% --- 5. MU-ANALYSIS A POSTERIORI ---
% (Incolla qui l'intero blocco copiato da "Mu_Analisys.m" del tuo collega,
% avendo cura di sostituire 'K_hinf' con 'K_mu' in tutto il blocco)