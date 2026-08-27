% clear all; close all; clc;

%% DEFINIZIONE PARAMETRI NOMINALI 
m_s = 350;
m_u = 50;
ks0 = 20000;
alpha = 10^5;
bs = 1500;
kt = 180000;
bt = 150;

% Parametri attuatori nominali
omega_n1 = 80;       
zeta_1 = 0.7;        
tau_1 = 0.005; 
tau_a = 0.025;       
tau_2 = 0.010;       

%% COSTRUZIONE DEL MODELLO LINEARIZZATO (Quarter-Car)
% Stati: x = [delta_s; zs_dot; delta_t; zu_dot]
% Ingressi: u_plant = [u1; u2; zr_dot]

A = [ 0,                    1,           0,                    -1;
     -ks0/m_s,             -bs/m_s,      0,                     bs/m_s;
      0,                    0,           0,                     1;
      ks0/m_u,              bs/m_u,     -kt/m_u,               -(bs+bt)/m_u ];

B = [ 0,       0,      0;
      1/m_s,   0,      0;
      0,       0,     -1;
     -1/m_u,   1/m_u,  bt/m_u ];

% Uscite: [accel_zs; delta_s; delta_t; accel_zu]
    % accel_zs = zs_ddot = -ks0/ms*x1 - bs/ms*x2 + bs/ms*x4 + 1/ms*u1

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

%% MODELLISTICA ATTUATORI CON RITARDO DI PADE' (1° Ordine)
s = tf('s');
Pade_1 = (1 - (tau_1/2)*s) / (1 + (tau_1/2)*s);
G_a1_ss = ss((omega_n1^2 / (s^2 + 2*zeta_1*omega_n1*s + omega_n1^2)) * Pade_1);
G_a1_ss.InputName = 'u1_cmd';  G_a1_ss.OutputName = 'u1_force';

Pade_2 = (1 - (tau_2/2)*s) / (1 + (tau_2/2)*s);
G_a2_ss = ss((1 / (1 + tau_a*s)) * Pade_2);
G_a2_ss.InputName = 'u2_cmd';  G_a2_ss.OutputName = 'u2_force';

%% 4. CREAZIONE DEL PLANT AUMENTATO NOMINALE
Sys_Nom = connect(Plant, G_a1_ss, G_a2_ss, {'u1_cmd', 'u2_cmd', 'w_dist'}, {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

disp('Stati effettivi del modello Nominale (Sys_Nom):');
disp(order(Sys_Nom)); 


%% 5. PREPARAZIONE MATRICI PER SINTESI LQG
% Estraiamo le matrici dal modello nominale a 9 stati (Sys_Nom)
A_nom = Sys_Nom.A;
B_tot = Sys_Nom.B;
C_tot = Sys_Nom.C;
D_tot = Sys_Nom.D;

% Dividiamo gli ingressi: 
% u_ctrl = [u1_cmd, u2_cmd] (colonne 1 e 2)
% w_dist = [w_dist]         (colonna 3)
B_u = B_tot(:, 1:2);
B_w = B_tot(:, 3);

% Dividiamo le uscite. Nel Simulink misuriamo solo le accelerazioni (IMU):
% y_meas = [zs_ddot, zu_ddot] -> righe 1 e 4 di C_tot e D_tot
idx_meas = [1, 4];
C_y = C_tot(idx_meas, :);
D_yu = D_tot(idx_meas, 1:2);
D_yw = D_tot(idx_meas, 3);

n = size(A_nom, 1); % Numero di stati (9)
m = size(B_u, 2);   % Numero di ingressi di controllo (2)
p = size(C_y, 1);   % Numero di uscite misurate (2)

%% 6. SINTESI CONTROLLO LQG (SENZA INTEGRATORE)
% 6.1 LQR: Feedback di stato (Progettato su A_nom e B_u)
% --- TUNING BILANCIATO: COMFORT + TENUTA DI STRADA ---
Q_lqr = zeros(n, n);
Q_lqr(1,1) = 1;    % Peso moderato su delta_s: la sospensione può muoversi ma non troppo
Q_lqr(2,2) = 5e5;    % Peso alto su zs_dot: frena i sobbalzi della carrozzeria (Comfort)
Q_lqr(3,3) = 5e6;    % Peso MOLTO ALTO su delta_t: schiaccia la ruota a terra (Road Holding)
Q_lqr(4,4) = 1e3;    % Peso basso su zu_dot: lascia vibrare liberamente la ruota

% Sforzo di controllo
R_lqr = eye(m) * 1e-3; % R_lqr equilibrato: permette forze realistiche (es. 500-1500 N)

K_lqr = lqr(A_nom, B_u, Q_lqr, R_lqr);


% 6.2 KALMAN FILTER (Osservatore)
% Definiamo i rumori di processo (W) e di misura (V)
W_kf = 0.05;             % Varianza del disturbo stradale w (scalare, entra da B_w)
V_kf = 0.001 * eye(p);    % Varianza del rumore sui 2 sensori IMU

% Il comando kalman richiede il sistema con ingressi [u, w]
Sys_Est = ss(A_nom, [B_u, B_w], C_y, [D_yu, D_yw]);
[kf_sys, K_e, P_est] = kalman(Sys_Est, W_kf, V_kf);

% 6.3 ASSEMBLAGGIO DEL CONTROLLORE LQG (Corretto per D_yu non nulla)
% Poiché gli attuatori con Padé hanno un legame algebrico diretto (D_yu ~= 0),
% dobbiamo inserire il termine di compensazione + K_e * D_yu * K_lqr 
% affinché il Kalman Filter non auto-annulli i comandi!

Ac = A_nom - B_u * K_lqr - K_e * C_y + K_e * D_yu * K_lqr;
Bc = K_e;
Cc = -K_lqr;
Dc = zeros(m, p);

K_LQG_noInt = ss(Ac, Bc, Cc, Dc);
K_LQG_noInt.InputName = {'zs_ddot', 'zu_ddot'};
K_LQG_noInt.OutputName = {'u1_cmd', 'u2_cmd'};

disp('Controllore LQG (Senza Integratore) assemblato con compensazione D!');
disp(['Ordine del controllore: ', num2str(order(K_LQG_noInt))]);

%% 7. VERIFICA STABILITA' E SIMULAZIONE LINEARE (Closed-Loop)
% Colleghiamo in anello chiuso l'impianto nominale (Sys_Nom) e il controllore (K_LQG_noInt)
% Gli ingressi esterni al sistema chiuso rimangono solo il disturbo stradale w_dist.
% Le uscite restano quelle di performance e di misura.

CL_sys = connect(Sys_Nom, K_LQG_noInt, 'w_dist', {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

% 7.1 Stability Check (Controllo Autovalori)
poli_CL = pole(CL_sys);
max_real_part = max(real(poli_CL));

disp('--- VERIFICA STABILITA LINEARE ---');
if max_real_part < 0
    disp(' Il sistema ad anello chiuso è Asintoticamente Stabile!');
else
    disp('ERRORE: Il sistema è INSTABILE! (Almeno un polo ha parte reale positiva)');
end

% 7.2 Simulazione Veloce Lineare (Risposta al gradino stradale)
% Simuliamo un piccolo gradino sulla strada (es. un dislivello di 1 cm = 0.01 m)
% Poiché w è la velocità della strada (zr_dot), un gradino di velocità
% corrisponde a una rampa di posizione. Per semplicità valutiamo 
% la risposta impulsiva (che equivale a un gradino di posizione stradale).

t_sim = 0:0.01:5; % Simuliamo per 5 secondi
figure('Name', 'Analisi Lineare LQG (No Int)');

% Risposta impulsiva su w_dist (equivale al gradino su z_r)
[y_out, t_out] = impulse(CL_sys, t_sim);

subplot(2,1,1);
plot(t_out, y_out(:, 2), 'b', 'LineWidth', 1.5); % delta_s (colonna 2)
title('Risposta Corsa Sospensione (\delta_s)');
ylabel('[m]'); grid on;

subplot(2,1,2);
plot(t_out, y_out(:, 1), 'r', 'LineWidth', 1.5); % zs_ddot (colonna 1)
title('Accelerazione Carrozzeria (Comfort)');
ylabel('[m/s^2]'); xlabel('Tempo [s]'); grid on;


%% 8. SINTESI CONTROLLO LQG CON AZIONE INTEGRALE (Sensore Reale)

% 8.1 Sistema Aumentato per il calcolo dei guadagni LQR
% Questa parte (la sintesi ottima teorica) rimane identica, 
% perché l'LQR si basa sul modello matematico, non sui sensori.
C_int = C_tot(2, :); 
A_aug = [A_nom, zeros(n, 1);
         C_int, 0];
B_aug = [B_u;
         zeros(1, m)];
         
% 8.2 Pesi LQR Aumentati
Q_aug = zeros(n+1, n+1);
Q_aug(1,1) = 1e5;    % delta_s
Q_aug(2,2) = 5e5;    % zs_dot 
Q_aug(3,3) = 5e6;    % delta_t 
Q_aug(4,4) = 1e3;    % zu_dot
Q_aug(10,10) = 1e6;  % PESO INTEGRATORE (Ridotto a 1e6 per un transitorio più dolce)

R_aug = eye(m) * 1e-3;

% Calcolo guadagno aumentato e separazione 
K_lqr_aug = lqr(A_aug, B_aug, Q_aug, R_aug);
K_r = K_lqr_aug(:, 1:n);  
K_i = K_lqr_aug(:, n+1);  

% 8.3 L'Osservatore di Kalman è già stato calcolato (K_e ora è 9x3)

% 8.4 Assemblaggio del Controllore Dinamico (La VERA Modifica)
% - In Ac_int l'ultima riga è di zeri: l'integratore non usa più la stima x_hat
% - In Bc_int l'ultima riga è [0, 1, 0]: l'integratore si nutre DIRETTAMENTE del sensore delta_s

Ac_int = [ (A_nom - B_u*K_r - K_e*C_y + K_e*D_yu*K_r),  (-B_u*K_i + K_e*D_yu*K_i);
           zeros(1, n),                                 0 ];

Bc_int = [ K_e;
           0, 1, 0 ]; 

Cc_int = [ -K_r, -K_i ];
Dc_int = zeros(m, p); % p ora vale 3

K_LQG_Int = ss(Ac_int, Bc_int, Cc_int, Dc_int);
K_LQG_Int.InputName = {'zs_ddot', 'delta_s', 'zu_ddot'};
K_LQG_Int.OutputName = {'u1_cmd', 'u2_cmd'};

disp('Controllore LQG CON INTEGRATORE (Cablato sul Sensore) assemblato ');
disp(['Ordine del nuovo controllore: ', num2str(order(K_LQG_Int))]);

%% 9. VERIFICA STABILITA' E SIMULAZIONE LINEARE (LQG CON INTEGRATORE)
% Chiudiamo l'anello con il nuovo controllore
CL_sys_int = connect(Sys_Nom, K_LQG_Int, 'w_dist', {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

% 9.1 Stability Check (Controllo Autovalori)
poli_CL_int = pole(CL_sys_int);
max_real_part_int = max(real(poli_CL_int));

disp('--- VERIFICA STABILITA LINEARE (CON INTEGRATORE) ---');
if max_real_part_int < 0
    disp('SUCCESSO: Il sistema ad anello chiuso con integratore è Asintoticamente Stabile!');
else
    disp('ERRORE: Il sistema con integratore è INSTABILE! (Almeno un polo ha parte reale positiva)');
end

% 9.2 Simulazione Veloce Lineare (Risposta al gradino stradale)
% Usiamo lo stesso t_sim della sezione 7 per poterli confrontare a occhio
figure('Name', 'Analisi Lineare LQG (CON Integratore)');

% Risposta impulsiva su w_dist (che equivale a un gradino su z_r)
[y_out_int, t_out_int] = impulse(CL_sys_int, t_sim);

subplot(2,1,1);
plot(t_out_int, y_out_int(:, 2), 'g', 'LineWidth', 1.5); % delta_s (colonna 2)
title('Risposta Corsa Sospensione (\delta_s) - CON INTEGRATORE');
ylabel('[m]'); 
grid on;

subplot(2,1,2);
plot(t_out_int, y_out_int(:, 1), 'r', 'LineWidth', 1.5); % zs_ddot (colonna 1)
title('Accelerazione Carrozzeria (Comfort) - CON INTEGRATORE');
ylabel('[m/s^2]'); 
xlabel('Tempo [s]'); 
grid on;

