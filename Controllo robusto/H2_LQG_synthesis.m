%% Sintesi Controllo LQG e H2

% Estrazione parametri 
m_s = parameters.m_s;
m_u = parameters.m_u;
ks0 = parameters.ks0;
bs  = parameters.bs;
kt  = parameters.kt;
bt  = parameters.bt;

%% Modello Linearizzato Quarter-Car
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




%% Sintesi H2 (Comfort e Road Holding)
% Costruzione dell'Impianto Generalizzato P per h2syn.
% Il vettore dei disturbi esogeni w viene aumentato per includere i rumori 
% di misura sui sensori.

% Pesi di Performance (Comfort vs Road Holding) ---
W_vel  = 2000;      % Comfort (zs_dot)
W_defl = 1000;      % Tenuta di strada (delta_t)

% Pesi di controllo 
W_u1   = 0.01;      
W_u2   = 0.1;      

% Rumore virtuale sui sensori 
W_n1 = 0.1;   % IMU cassa
W_n2 = 0.01;  % LVDT
W_n3 = 0.1;   % IMU ruota

% Matrici per le Uscite di Prestazione z 
% Minimizza z = [W_vel*zs_dot; W_defl*delta_t; W_u1*u1; W_u2*u2]
C_z = [ W_vel  * [0, 1, 0, 0];  % Estrae zs_dot (2° stato)
        W_defl * [0, 0, 1, 0];  % Estrae delta_t (3° stato)
        zeros(2, n) ];          % Spazio per le penalità di controllo u1, u2
        
% Nessun feedthrough diretto del disturbo stradale
D_zw_strada = zeros(4, 1);
                
% Matrice comandi u
D_zu = [ 0, 0;
         0, 0;
         W_u1, 0;
         0, W_u2 ];
         
% Aumento del Disturbo w con i rumori di misura 
B_w_aug = [B_w, zeros(n, p)];
D_zw_aug = [D_zw_strada, zeros(4, p)]; 
D_yw_aug = [D_yw, diag([W_n1, W_n2, W_n3])];

% Costruzione Impianto Generalizzato P 
A_P = A;
B_P = [B_w_aug, B_u];
C_P = [C_z; 
       C_y];
D_P = [D_zw_aug, D_zu;
       D_yw_aug, D_yu];
       
P_sys = ss(A_P, B_P, C_P, D_P);

% Sintesi del Controllore Ottimo H2 
nmeas = p;  % numero uscite misurate y (le ultime 3 uscite di P_sys)
ncont = m;  % numero comandi u (gli ultimi 2 ingressi di P_sys)

[K_H2, CL_H2, gamma_H2] = h2syn(P_sys, nmeas, ncont);

% Rinomino I/O per facilitare l'uso del blocco in Simulink
K_H2.InputName = {'zs_ddot', 'delta_s', 'zu_ddot'}; 
K_H2.OutputName = {'u1_cmd', 'u2_cmd'};

disp(['Sintesi H2 completata. Norma H2 ottima (gamma): ', num2str(gamma_H2)]);




%% Analisi in Frequenza: Comfort e Tenuta di Strada
disp('Generazione Grafici di Bode (Comfort e Tenuta di Strada)...');

C_dt = [0, 0, 1, 0];
D_dt_w = 0;
D_dt_u = [0, 0];

Sys_Eval = ss(A, [B_w, B_u], [C_y; C_dt], [D_yw, D_yu; D_dt_w, D_dt_u]);
Sys_Eval.InputName  = {'w_dist', 'u1_cmd', 'u2_cmd'};
Sys_Eval.OutputName = {'zs_ddot', 'delta_s', 'zu_ddot', 'delta_t'};

% Chiusura Anello di controllo (w_dist -> [zs_ddot, delta_t])
G_passiva = Sys_Eval({'zs_ddot', 'delta_t'}, 'w_dist'); 
G_LQG     = connect(Sys_Eval, K_LQG_Int, 'w_dist', {'zs_ddot', 'delta_t'});
G_H2      = connect(Sys_Eval, K_H2, 'w_dist', {'zs_ddot', 'delta_t'});

w_vec = logspace(-1, 3, 1000); % Vettore di frequenze comune

% Comfort (w -> zs_ddot)
figure('Name', 'Bode - Comfort Vibrazionale', 'Color', 'w');
bodemag(G_passiva(1,1), 'k--', G_LQG(1,1), 'b', G_H2(1,1), 'r', w_vec);
grid on;
legend('Passiva', 'Attiva (LQG Integrale)', 'Attiva (H_2)', 'Location', 'southwest');
title('Amplificazione Disturbo Stradale: w \rightarrow zs\_ddot (Comfort)');

% Estrazione Dati Comfort
[mag_p_c, ~, ~]   = bode(G_passiva(1,1), w_vec);
[mag_lqg_c, ~, ~] = bode(G_LQG(1,1), w_vec);
[mag_h2_c, ~, ~]  = bode(G_H2(1,1), w_vec);

mag_p_c_dB   = 20*log10(squeeze(mag_p_c));
mag_lqg_c_dB = 20*log10(squeeze(mag_lqg_c));
mag_h2_c_dB  = 20*log10(squeeze(mag_h2_c));

% Tenuta di strada (w -> delta_t)
figure('Name', 'Bode - Tenuta di Strada', 'Color', 'w');
bodemag(G_passiva(2,1), 'k--', G_LQG(2,1), 'b', G_H2(2,1), 'r', w_vec);
grid on;
legend('Passiva', 'Attiva (LQG Integrale)', 'Attiva (H_2)', 'Location', 'southwest');
title('Amplificazione Disturbo Stradale: w \rightarrow \delta_t (Tenuta di Strada)');

% Estrazione Dati Tenuta di Strada
[mag_p_t, ~, ~]   = bode(G_passiva(2,1), w_vec);
[mag_lqg_t, ~, ~] = bode(G_LQG(2,1), w_vec);
[mag_h2_t, ~, ~]  = bode(G_H2(2,1), w_vec);

mag_p_t_dB   = 20*log10(squeeze(mag_p_t));
mag_lqg_t_dB = 20*log10(squeeze(mag_lqg_t));
mag_h2_t_dB  = 20*log10(squeeze(mag_h2_t));

% Calcolo Risultati 
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

disp('--- 1. COMFORT VIBRAZIONALE (w -> zs_ddot) ---');
disp(['> Picco Cassa a ', num2str(w_vec(peak_c_cassa), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_c_dB(peak_c_cassa) - mag_lqg_c_dB(peak_c_cassa), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_c_dB(peak_c_cassa) - mag_h2_c_dB(peak_c_cassa), '%+0.2f'), ' dB']);
disp(['> Picco Ruota a ', num2str(w_vec(peak_c_ruota), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_c_dB(peak_c_ruota) - mag_lqg_c_dB(peak_c_ruota), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_c_dB(peak_c_ruota) - mag_h2_c_dB(peak_c_ruota), '%+0.2f'), ' dB']);
disp(' ');

disp('--- 2. TENUTA DI STRADA (w -> delta_t) ---');
disp(['> Picco Cassa a ', num2str(w_vec(peak_t_cassa), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_t_dB(peak_t_cassa) - mag_lqg_t_dB(peak_t_cassa), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_t_dB(peak_t_cassa) - mag_h2_t_dB(peak_t_cassa), '%+0.2f'), ' dB']);
disp(['> Picco Ruota a ', num2str(w_vec(peak_t_ruota), '%.2f'), ' rad/s:']);
disp(['     Attenuazione LQG: ', num2str(mag_p_t_dB(peak_t_ruota) - mag_lqg_t_dB(peak_t_ruota), '%+0.2f'), ' dB']);
disp(['     Attenuazione H2 : ', num2str(mag_p_t_dB(peak_t_ruota) - mag_h2_t_dB(peak_t_ruota), '%+0.2f'), ' dB']);
disp('=================================================================');