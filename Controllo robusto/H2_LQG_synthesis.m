%   =========================================================
%              H2 ( & LQG )  SYNTHESIS: QUARTER-CAR 
%   =========================================================

%% Caricamento Parametri e Impianto Incerto
clear all; close all; clc;

run('A_quarter_car_parameters.m');
run('A_uncertain_Plant.m');
P_esteso_nom = P_esteso.NominalValue;
s = tf('s');

%% Modello Linearizzato Quarter-Car

% Estrazione parametri 
m_s = parameters.m_s;
m_u = parameters.m_u;
ks0 = parameters.ks0;
bs  = parameters.bs;
kt  = parameters.kt;
bt  = parameters.bt;

% Stati x = [delta_s; zs_dot; delta_t; zu_dot]
A = [ 0,            1,          0,           -1;
     -ks0/m_s,     -bs/m_s,     0,            bs/m_s;
      0,            0,          0,            1;
      ks0/m_u,      bs/m_u,    -kt/m_u,     -(bs+bt)/m_u ];

B_u = [ 0,       0;
        1/m_s,   0;
        0,       0;
       -1/m_u,   1/m_u ];

B_w = [ 0; 0; -1; bt/m_u ]; % Ingresso disturbo stradale w = zr_dot

% Uscite misurate: y_meas = [zs_ddot; delta_s; zu_ddot]
C_y = [ -ks0/m_s, -bs/m_s,  0,       bs/m_s;        % Accelerazione cassa (IMU)
         1,        0,       0,       0;             % Escursione sospensione (LVDT)
         ks0/m_u,  bs/m_u, -kt/m_u, -(bs+bt)/m_u ]; % Accelerazione ruota (IMU)

D_yu = [ 1/m_s,   0;
         0,       0;
        -1/m_u,   1/m_u ];

D_yw = [ 0; 0; bt/m_u ];

% Dimensioni del plant base
n = size(A, 1);
m = size(B_u, 2);
p = size(C_y, 1);

%% Sintesi Filtro di Kalman (Condiviso tra noInt e Int)
% Il filtro di Kalman conosce il disturbo B_w e lo tratta come rumore
W_kf = 0.05;                        % varianza disturbo stradale w 
V_kf = diag([1e-3, 1e-5, 1e-3]);    % Rumore sensori: IMU_s, LVDT, IMU_u

Sys_Est = ss(A, [B_u, B_w], C_y, [D_yu, D_yw]);
[kf_sys, K_e, P_est] = kalman(Sys_Est, W_kf, V_kf);

%% Sintesi LQG (Senza Integratore)
Q_lqr = diag([1e5, 1e4, 1e4, 1e2]); % Pesi su: [delta_s, zs_dot, delta_t, zu_dot]
R_lqr = diag([1e-4, 1e-4]);         % Penalità sforzo di controllo u1, u2

K_lqr = lqr(A, B_u, Q_lqr, R_lqr);

% Assemblaggio Controllore LQG noInt
Ac_noInt = A - B_u*K_lqr - K_e*C_y + K_e*D_yu*K_lqr;
K_LQG_noInt = ss(Ac_noInt, K_e, -K_lqr, zeros(m, p));
K_LQG_noInt.InputName = {'zs_ddot', 'delta_s', 'zu_ddot'}; 
K_LQG_noInt.OutputName = {'u1_cmd', 'u2_cmd'};

%% Sintesi LQG con Azione Integrale 
% Focus:  garantire errore a regime nullo su delta_s a fronte di carichi statici
C_int = [1, 0, 0, 0];   % Estrazione stato delta_s per integrarlo

% Sistema Aumentato
A_aug = [A, zeros(n, 1);
         C_int, 0];
B_aug = [B_u; zeros(1, m)];

% Pesi per LQR Aumentato
Q_aug = blkdiag(Q_lqr, 5e6);    % Peso alto sull'errore integrale di delta_s (forza a portarlo a zero)
R_aug = R_lqr;

K_lqr_aug = lqr(A_aug, B_aug, Q_aug, R_aug);
K_r = K_lqr_aug(:, 1:n);   % Guadagno sugli stati stimati
K_i = K_lqr_aug(:, n+1);   % Guadagno sullo stato integrale

% Assemblaggio Controllore LQG Integrale
% Usiamo la stima di Kalman per gli stati e la misura diretta (y_meas(2)) per l'integrale
Ac_int = [ (A - B_u*K_r - K_e*C_y + K_e*D_yu*K_r),  (-B_u*K_i + K_e*D_yu*K_i);
           zeros(1, n),                                 0 ]; 
Bc_int = [ K_e;
           0, 1, 0 ]; % Integra direttamente il sensore LVDT (delta_s misurato)

K_LQG_Int = ss(Ac_int, Bc_int, [-K_r, -K_i], zeros(m, p));
K_LQG_Int.InputName = {'zs_ddot','delta_s', 'zu_ddot'}; 
K_LQG_Int.OutputName = {'u1_cmd', 'u2_cmd'};

disp('Sintesi completata. Controllori K_LQG_noInt e K_LQG_Int pronti per Simulink.');



%% 2. Costruzione Impianto Generalizzato per H2 (PESI COSTANTI E 3 MISURE)

% -- Pesi sulle Prestazioni (WP) - COSTANTI (Rimaniamo a 4 variabili da ottimizzare) --
wP1 = 100;    % Peso su accelerazione cassa (zs_ddot) - Comfort
wP2 = 10;     % Peso su autolivellamento (delta_s)
wP3 = 1000;   % Peso su tenuta di strada (delta_t) - Vogliamo ottimizzarla anche se non la misuriamo!
wP4 = 1;      % Peso su accelerazione ruota (zu_ddot)

WP = diag([wP1, wP2, wP3, wP4]);
WP = ss(WP);  
WP.InputName  = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};
WP.OutputName = {'z_p1', 'z_p2', 'z_p3', 'z_p4'};

% -- Pesi sullo Sforzo di Controllo (Wu) - COSTANTI --
w_u1 = 0.1;
w_u2 = 0.1;

Wu = diag([w_u1, w_u2]);
Wu = ss(Wu);
Wu.InputName  = {'u1_cmd', 'u2_cmd'};
Wu.OutputName = {'z_u1', 'z_u2'};

% -- Rumore sui Sensori (Wn) - SOLO 3 SENSORI REALI --
% Creiamo rumore solo per i 3 sensori che abbiamo in Simulink
Wn = 0.01 * eye(3); 
Wn = ss(Wn);
Wn.InputName = {'n_zs', 'n_ds', 'n_zu'};
Wn.OutputName = {'noise_zs', 'noise_ds', 'noise_zu'};

% Nodi Sommatore (Retroazione negativa + Rumore)
% Rimuoviamo Sv3 (quello relativo a delta_t)
Sv_zs = sumblk('v_zs = -zs_ddot - noise_zs');
Sv_ds = sumblk('v_ds = -delta_s - noise_ds');
Sv_zu = sumblk('v_zu = -zu_ddot - noise_zu');

% -- Interconnessione Finale (Impianto Generalizzato) --
% Inseriamo solo i 3 rumori e le 3 uscite misurate (v_zs, v_ds, v_zu)
P_gen_H2 = connect(P_esteso_nom, WP, Wu, Wn, Sv_zs, Sv_ds, Sv_zu, ...
                   {'w_in', 'n_zs', 'n_ds', 'n_zu', 'u1_cmd', 'u2_cmd'}, ...
                   {'z_p1','z_p2','z_p3','z_p4','z_u1','z_u2', 'v_zs', 'v_ds', 'v_zu'});

%% 3. Sintesi Ottima H2
% P_gen_H2 ha ora:
% Ingressi esogeni (w): 4  (w_in + 3 rumori n)
% Ingressi controllo (u): 2 (u1_cmd, u2_cmd)
% Uscite performance (z): 6 (4 prestazioni + 2 sforzi)
% Uscite misurate (v): 3    (v_zs, v_ds, v_zu)

NMEAS = 3; % <-- Corretto: 3 sensori letti dal controllore
NCONT = 2; % 2 attuatori comandati

[K_H2, CL_H2, gamma_H2] = h2syn(P_gen_H2, NMEAS, NCONT);

% Rinomino I/O del controllore per compatibilità esatta con i nomi di Simulink
K_H2.InputName = {'zs_ddot', 'delta_s', 'zu_ddot'}; 
K_H2.OutputName = {'u1_cmd', 'u2_cmd'};

% Invertiamo il segno per il formato standard u = -K*y
K_H2 = -K_H2; 

disp(['Sintesi H2 completata. Norma H2 ottima (gamma): ', num2str(gamma_H2)]);





%% Analisi in Frequenza: Comfort e Tenuta di Strada
disp('Generazione Grafici di Bode (Comfort e Tenuta di Strada)...');

% 1. IMPIANTO PASSIVO 
G_passiva = P_esteso_nom({'zs_ddot', 'delta_t'}, 'w_in');

% 2. COSTRUZIONE DEI SISTEMI AD ANELLO CHIUSO (COLLEGAMENTO DIRETTO)
G_LQG = connect(P_esteso_nom, K_LQG_Int, 'w_in', {'zs_ddot', 'delta_t'});
G_H2  = connect(P_esteso_nom, K_H2,      'w_in', {'zs_ddot', 'delta_t'});


w_vec = logspace(-1, 3, 1000); % Vettore di frequenze comune

% --- Comfort Vibrazionale (w_in -> zs_ddot) ---
figure('Name', 'Bode - Comfort Vibrazionale', 'Color', 'w');
bodemag(G_passiva(1,1), 'k--', G_LQG(1,1), 'b', G_H2(1,1), 'r', w_vec);
grid on;
legend('Passiva', 'Attiva (LQG Integrale)', 'Attiva (H_2)', 'Location', 'southwest');
title('Amplificazione Disturbo: w_{in} \rightarrow zs\_ddot (Comfort)');

% Estrazione Dati Comfort
[mag_p_c, ~, ~]   = bode(G_passiva(1,1), w_vec);
[mag_lqg_c, ~, ~] = bode(G_LQG(1,1), w_vec);
[mag_h2_c, ~, ~]  = bode(G_H2(1,1), w_vec);

mag_p_c_dB   = 20*log10(squeeze(mag_p_c));
mag_lqg_c_dB = 20*log10(squeeze(mag_lqg_c));
mag_h2_c_dB  = 20*log10(squeeze(mag_h2_c));

% --- Tenuta di strada (w_in -> delta_t) ---
figure('Name', 'Bode - Tenuta di Strada', 'Color', 'w');
bodemag(G_passiva(2,1), 'k--', G_LQG(2,1), 'b', G_H2(2,1), 'r', w_vec);
grid on;
legend('Passiva', 'Attiva (LQG Integrale)', 'Attiva (H_2)', 'Location', 'southwest');
title('Amplificazione Disturbo: w_{in} \rightarrow \delta_t (Tenuta di Strada)');

% Estrazione Dati Tenuta di Strada
[mag_p_t, ~, ~]   = bode(G_passiva(2,1), w_vec);
[mag_lqg_t, ~, ~] = bode(G_LQG(2,1), w_vec);
[mag_h2_t, ~, ~]  = bode(G_H2(2,1), w_vec);

mag_p_t_dB   = 20*log10(squeeze(mag_p_t));
mag_lqg_t_dB = 20*log10(squeeze(mag_lqg_t));
mag_h2_t_dB  = 20*log10(squeeze(mag_h2_t));

% --- Calcolo Risultati e Attenuazioni ---
% Ricerca indici di picco (Cassa: 4-15 rad/s | Ruota: 40-80 rad/s)
idx_cassa = find(w_vec > 4 & w_vec < 15);
idx_ruota = find(w_vec > 40 & w_vec < 80);

[~, loc_c_cassa] = max(mag_p_c_dB(idx_cassa)); peak_c_cassa = idx_cassa(loc_c_cassa);
[~, loc_c_ruota] = max(mag_p_c_dB(idx_ruota)); peak_c_ruota = idx_ruota(loc_c_ruota);

[~, loc_t_cassa] = max(mag_p_t_dB(idx_cassa)); peak_t_cassa = idx_cassa(loc_t_cassa);
[~, loc_t_ruota] = max(mag_p_t_dB(idx_ruota)); peak_t_ruota = idx_ruota(loc_t_ruota);

disp(' ');
disp('=================================================================');
disp('   ANALISI FREQUENZIALE: ATTENUAZIONE RISPETTO AL PASSIVO');
disp('   (Valori positivi = Miglioramento | Valori negativi = Peggioramento)');
disp('=================================================================');

disp('--- 1. COMFORT VIBRAZIONALE (w_in -> zs_ddot) ---');
disp(['> Picco Cassa a ', num2str(w_vec(peak_c_cassa), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_c_dB(peak_c_cassa) - mag_lqg_c_dB(peak_c_cassa), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_c_dB(peak_c_cassa) - mag_h2_c_dB(peak_c_cassa), '%+0.2f'), ' dB']);
disp(['> Picco Ruota a ', num2str(w_vec(peak_c_ruota), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_c_dB(peak_c_ruota) - mag_lqg_c_dB(peak_c_ruota), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_c_dB(peak_c_ruota) - mag_h2_c_dB(peak_c_ruota), '%+0.2f'), ' dB']);
disp(' ');

disp('--- 2. TENUTA DI STRADA (w_in -> delta_t) ---');
disp(['> Picco Cassa a ', num2str(w_vec(peak_t_cassa), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_t_dB(peak_t_cassa) - mag_lqg_t_dB(peak_t_cassa), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_t_dB(peak_t_cassa) - mag_h2_t_dB(peak_t_cassa), '%+0.2f'), ' dB']);
disp(['> Picco Ruota a ', num2str(w_vec(peak_t_ruota), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_t_dB(peak_t_ruota) - mag_lqg_t_dB(peak_t_ruota), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_t_dB(peak_t_ruota) - mag_h2_t_dB(peak_t_ruota), '%+0.2f'), ' dB']);
disp('=================================================================');