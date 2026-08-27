% clear all; close all; clc;

%% 1. PARAMETRI NOMINALI 
m_s = 350; m_u = 50;
ks0 = 20000; alpha = 10^5; bs = 1500;
kt = 180000; bt = 150;

% Attuatori
omega_n1 = 80; zeta_1 = 0.7; tau_1 = 0.005; 
tau_a = 0.025; tau_2 = 0.010;       

%% 2. COSTRUZIONE MODELLO (Quarter-Car)
A = [ 0, 1, 0, -1;
     -ks0/m_s, -bs/m_s, 0, bs/m_s;
      0, 0, 0, 1;
      ks0/m_u, bs/m_u, -kt/m_u, -(bs+bt)/m_u ];
B = [ 0, 0, 0;
      1/m_s, 0, 0;
      0, 0, -1;
     -1/m_u, 1/m_u, bt/m_u ];
C = [ -ks0/m_s, -bs/m_s, 0, bs/m_s;   
       1, 0, 0, 0;        
       0, 0, 1, 0;        
       ks0/m_u, bs/m_u, -kt/m_u, -(bs+bt)/m_u ]; 
D = [ 1/m_s, 0, 0;
      0, 0, 0;
      0, 0, 0;
     -1/m_u, 1/m_u, bt/m_u ];

Plant = ss(A, B, C, D);
Plant.StateName = {'delta_s', 'zs_dot', 'delta_t', 'zu_dot'};
Plant.InputName = {'u1_force', 'u2_force', 'w_dist'};
Plant.OutputName = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};

%% 3. MODELLISTICA ATTUATORI
s = tf('s');
G_a1_ss = ss((omega_n1^2 / (s^2 + 2*zeta_1*omega_n1*s + omega_n1^2)) * ((1 - (tau_1/2)*s) / (1 + (tau_1/2)*s)));
G_a1_ss.InputName = 'u1_cmd';  G_a1_ss.OutputName = 'u1_force';

G_a2_ss = ss((1 / (1 + tau_a*s)) * ((1 - (tau_2/2)*s) / (1 + (tau_2/2)*s)));
G_a2_ss.InputName = 'u2_cmd';  G_a2_ss.OutputName = 'u2_force';

%% 4. PLANT AUMENTATO NOMINALE
Sys_Nom = connect(Plant, G_a1_ss, G_a2_ss, {'u1_cmd', 'u2_cmd', 'w_dist'}, {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

%% 5. PREPARAZIONE MATRICI LQG (SOLO 2 IMU)
A_nom = Sys_Nom.A; B_tot = Sys_Nom.B; C_tot = Sys_Nom.C; D_tot = Sys_Nom.D;
B_u = B_tot(:, 1:2); B_w = B_tot(:, 3);

% p=2: Misuriamo solo accelerazioni (Righe 1 e 4)
idx_meas = [1, 4];
C_y = C_tot(idx_meas, :);
D_yu = D_tot(idx_meas, 1:2);
D_yw = D_tot(idx_meas, 3);

n = size(A_nom, 1); m = size(B_u, 2); p = size(C_y, 1);

%% 6. SINTESI LQG BASE (No Integratore)
Q_lqr = diag([1, 5e5, 5e6, 1e3, 0, 0, 0, 0, 0]); 
R_lqr = eye(m) * 1e-3;
K_lqr = lqr(A_nom, B_u, Q_lqr, R_lqr);

W_kf = 0.05; 
V_kf = diag([0.001, 0.001]); % Rumore su 2 IMU
Sys_Est = ss(A_nom, [B_u, B_w], C_y, [D_yu, D_yw]);
[kf_sys, K_e, P_est] = kalman(Sys_Est, W_kf, V_kf);

Ac = A_nom - B_u * K_lqr - K_e * C_y + K_e * D_yu * K_lqr;
K_LQG_noInt = ss(Ac, K_e, -K_lqr, zeros(m, p));
K_LQG_noInt.InputName = {'zs_ddot', 'zu_ddot'};
K_LQG_noInt.OutputName = {'u1_cmd', 'u2_cmd'};

%% 8. SINTESI LQG CON INTEGRATORE (Sulla stima di delta_s)
C_int = C_tot(2, :); % Vogliamo integrare delta_s
A_aug = [A_nom, zeros(n, 1); C_int, 0];
B_aug = [B_u; zeros(1, m)];

Q_aug = diag([1e4, 5e5, 5e6, 1e3, 0, 0, 0, 0, 0, 1e6]); % 1e6 peso integratore
K_lqr_aug = lqr(A_aug, B_aug, Q_aug, eye(m) * 1e-3);
K_r = K_lqr_aug(:, 1:n);  
K_i = K_lqr_aug(:, n+1);  

% L'integratore usa lo stato interno stimato (C_int * x_hat)
Ac_int = [ (A_nom - B_u*K_r - K_e*C_y + K_e*D_yu*K_r),  (-B_u*K_i + K_e*D_yu*K_i);
           C_int,                                       0 ];
Bc_int = [ K_e; zeros(1, p) ];

K_LQG_Int = ss(Ac_int, Bc_int, [-K_r, -K_i], zeros(m, p));
K_LQG_Int.InputName = {'zs_ddot', 'zu_ddot'};
K_LQG_Int.OutputName = {'u1_cmd', 'u2_cmd'};

disp('Entrambi i controllori a 2 IMU assemblati con successo!');