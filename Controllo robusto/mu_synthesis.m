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

% --- 2. DEFINIZIONE DEI PESI (Da MixedSensitivity.m) ---
s = tf('s');
wP_track = (s/2.0 + 5) / (s + 5*1e-3); 

% Filtro per il Comfort (Accelerazione zs_ddot)
w_low = 2*pi*1;  w_high = 2*pi*10;
Gain_acc = 0.1; 
wP_acc = Gain_acc * ( (s/w_low) / (s/w_low + 1) ) * ( 1 / (s/w_high + 1) );

wP_relax = 0.1;
WP = blkdiag(wP_acc, wP_track, wP_relax, wP_relax);

wu_val = 0.05 / 3000; 
Wu = blkdiag(wu_val, wu_val);

wT_base = (s + 50) / (0.01*s + 100); 
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