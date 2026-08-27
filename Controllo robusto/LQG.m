%% 1. ESTRAZIONE PARAMETRI DAL WORKSPACE
% Preleviamo i parametri fisici dalla struct caricata precedentemente
m_s = parameters.m_s;
m_u = parameters.m_u;
ks0 = parameters.ks0;
bs  = parameters.bs;
kt  = parameters.kt;
bt  = parameters.bt;

%% 2. COSTRUZIONE DEL MODELLO LINEARIZZATO (Quarter-Car)
% Stati: x = [delta_s; zs_dot; delta_t; zu_dot]
% Ingressi: u_plant = [u1; u2; w_dist] (w_dist = zr_dot)
A = [ 0,                    1,           0,                    -1;
     -ks0/m_s,             -bs/m_s,      0,                     bs/m_s;
      0,                    0,           0,                     1;
      ks0/m_u,              bs/m_u,     -kt/m_u,               -(bs+bt)/m_u ];

B = [ 0,       0,      0;
      1/m_s,   0,      0;
      0,       0,     -1;
     -1/m_u,   1/m_u,  bt/m_u ];

% Uscite: [accel_zs; delta_s; delta_t; accel_zu]
C = [ -ks0/m_s, -bs/m_s,  0,       bs/m_s;   
       1,        0,       0,       0;        
       0,        0,       1,       0;        
       ks0/m_u,  bs/m_u, -kt/m_u, -(bs+bt)/m_u ]; 

D = [ 1/m_s,   0,      0;
      0,       0,      0;
      0,       0,      0;
     -1/m_u,   1/m_u,  bt/m_u ];

Plant = ss(A, B, C, D);
Plant.StateName = {'delta_s', 'zs_dot', 'delta_t', 'zu_dot'};
Plant.InputName = {'u1_force', 'u2_force', 'w_dist'};
Plant.OutputName = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};

%% 3. MODELLISTICA ATTUATORI CON RITARDO DI PADE' (1° Ordine)
s = tf('s');
Pade_1 = (1 - (tau_1/2)*s) / (1 + (tau_1/2)*s);
G_a1_ss = ss((omega_n1^2 / (s^2 + 2*zeta_1*omega_n1*s + omega_n1^2)) * Pade_1);
G_a1_ss.InputName = 'u1_cmd';  G_a1_ss.OutputName = 'u1_force';

Pade_2 = (1 - (tau_2/2)*s) / (1 + (tau_2/2)*s);
G_a2_ss = ss((1 / (1 + tau_a*s)) * Pade_2);
G_a2_ss.InputName = 'u2_cmd';  G_a2_ss.OutputName = 'u2_force';

%% 4. CREAZIONE DEL PLANT AUMENTATO NOMINALE
Sys_Nom = connect(Plant, G_a1_ss, G_a2_ss, {'u1_cmd', 'u2_cmd', 'w_dist'}, {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

%% 5. PREPARAZIONE MATRICI PER SINTESI LQG (SOLO 2 IMU)
A_nom = Sys_Nom.A;
B_tot = Sys_Nom.B;
C_tot = Sys_Nom.C;
D_tot = Sys_Nom.D;

B_u = B_tot(:, 1:2);
B_w = B_tot(:, 3);

% SELEZIONIAMO SOLO LE 2 IMU (Righe 1 e 4 di C_tot)
idx_meas = [1, 4];
C_y = C_tot(idx_meas, :);
D_yu = D_tot(idx_meas, 1:2);
D_yw = D_tot(idx_meas, 3);

n = size(A_nom, 1); % 9 stati
m = size(B_u, 2);   % 2 ingressi
p = size(C_y, 1);   % 2 uscite misurate

%% 6. SINTESI CONTROLLO LQG (SENZA INTEGRATORE)
% 6.1 LQR: Feedback di stato
Q_lqr = zeros(n, n);
Q_lqr(1,1) = 1;      % delta_s (Corsa Sospensione): Peso bassissimo. L'ammortizzatore deve essere libero di lavorare
Q_lqr(2,2) = 5e5;    % zs_dot (Comfort): Peso alto. Smorza la velocità verticale della carrozzeria
Q_lqr(3,3) = 5e6;    % delta_t (Road Holding): Peso altissimo. Mantiene lo pneumatico schiacciato a terra
Q_lqr(4,4) = 1e3;    % zu_dot (Wheel Hop): Peso basso. Frena le vibrazioni libere della ruota
% NOTA: Gli stati da 5 a 9 (ritardi e dinamica attuatori) rimangono a zero.


R_lqr = eye(m) * 1e-3; 
K_lqr = lqr(A_nom, B_u, Q_lqr, R_lqr);

% 6.2 KALMAN FILTER (Osservatore a 2 sensori)
W_kf = 0.01;              
V_kf = diag([0.001, 0.001]); % Matrice 2x2 per le due IMU
Sys_Est = ss(A_nom, [B_u, B_w], C_y, [D_yu, D_yw]);
[kf_sys, K_e, P_est] = kalman(Sys_Est, W_kf, V_kf);

% 6.3 ASSEMBLAGGIO LQG BASE
Ac = A_nom - B_u * K_lqr - K_e * C_y + K_e * D_yu * K_lqr;
K_LQG_noInt = ss(Ac, K_e, -K_lqr, zeros(m, p));
K_LQG_noInt.InputName = {'zs_ddot', 'zu_ddot'}; 
K_LQG_noInt.OutputName = {'u1_cmd', 'u2_cmd'};
disp('Controllore LQG Base (2 IMU) assemblato.');

%% 7. SINTESI CONTROLLO LQG CON AZIONE INTEGRALE (Su Stima Kalman)
C_int = C_tot(2, :); % Vogliamo integrare delta_s

A_aug = [A_nom, zeros(n, 1);
         C_int, 0];
B_aug = [B_u;
         zeros(1, m)];
         
% 7.2 Pesi LQR Aumentati (Forma Estesa)
Q_aug = zeros(n+1, n+1);
Q_aug(1,1)   = 1;    % delta_s (Corsa Sospensione): Aiuta leggermente l'integratore nel transitorio
Q_aug(2,2)   = 5e5;    % zs_dot (Comfort): Smorza la velocità della carrozzeria
Q_aug(3,3)   = 5e6;    % delta_t (Road Holding): Mantiene la ruota incollata a terra
Q_aug(4,4)   = 1e3;    % zu_dot (Wheel Hop): Smorza i saltellamenti della ruota
% Gli stati da 5 a 9 (dinamica attuatori e ritardi di Padé) rimangono a zero.



Q_aug(10,10) = 1e6;    % STATO INTEGRALE (int_delta_s): Più è alto, più l'auto cerca di tornare a delta_s = 0 velocemente.

R_aug = eye(m) * 1e-3; % Sforzo di controllo (Costo energia attuatori)

K_lqr_aug = lqr(A_aug, B_aug, Q_aug, R_aug);
K_r = K_lqr_aug(:, 1:n);  
K_i = K_lqr_aug(:, n+1);  

% Assemblaggio Controllore (L'integratore usa la STIMA del Kalman C_int*x_hat)
Ac_int = [ (A_nom - B_u*K_r - K_e*C_y + K_e*D_yu*K_r),  (-B_u*K_i + K_e*D_yu*K_i);
           C_int,                                       0 ]; 
Bc_int = [ K_e;
           zeros(1, p) ]; 

K_LQG_Int = ss(Ac_int, Bc_int, [-K_r, -K_i], zeros(m, p));
K_LQG_Int.InputName = {'zs_ddot', 'zu_ddot'}; 
K_LQG_Int.OutputName = {'u1_cmd', 'u2_cmd'};
disp('Controllore LQG Integrale (2 IMU) assemblato.');

%% 8. CHIUSURA ANELLI LINEARI E VERIFICA STABILITA'
CL_sys_base = connect(Sys_Nom, K_LQG_noInt, 'w_dist', {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});
CL_sys_int  = connect(Sys_Nom, K_LQG_Int, 'w_dist', {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

if max(real(pole(CL_sys_base))) < 0 && max(real(pole(CL_sys_int))) < 0
    disp('-> Entrambi i sistemi ad anello chiuso sono Asintoticamente Stabili.');
else
    disp('-> ERRORE: Almeno un sistema chiuso è instabile.');
end

%% 9. ANALISI IN FREQUENZA E ROBUSTEZZA (Dominio Frequenza)
disp('--- Generazione Grafici di Bode e Margini di Robustezza ---');

% Attenuazione Disturbi (w_dist -> zs_ddot)
G_passiva = Sys_Nom('zs_ddot', 'w_dist');
G_attiva  = CL_sys_int('zs_ddot', 'w_dist');

figure('Name', 'Attenuazione Disturbi (Bode)');
bode(G_passiva, 'b', G_attiva, 'r', {0.1, 1000}); grid on;
legend('Passiva (Senza Controllo)', 'Attiva (LQG Integrale)', 'Location', 'best');
title('Trasmissibilita Disturbo Stradale: w \rightarrow zs\_ddot'); 

% Margini di Robustezza
G_meas = Sys_Nom({'zs_ddot', 'zu_ddot'}, {'u1_cmd', 'u2_cmd'}); 
L_loop = - (K_LQG_Int * G_meas); 

figure('Name', 'Margini Robustezza - u1');
margin(L_loop(1,1)); title('Margini di Robustezza - Attuatore u_1 (Carrozzeria)'); grid on;

figure('Name', 'Margini Robustezza - u2');
margin(L_loop(2,2)); title('Margini di Robustezza - Attuatore u_2 (Ruota)'); grid on;