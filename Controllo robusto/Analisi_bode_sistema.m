%% --- ANALISI MODALE E IN FREQUENZA DEL SISTEMA PASSIVO ---

% 1. Definizione Parametri Nominali (Traccia 1)
m_s = parameters.m_s;
m_u = parameters.m_u;
ks0 = parameters.ks0;
bs  = parameters.bs;
kt  = parameters.kt;
bt  = parameters.bt;

% 2. Matrice della Dinamica A (Sistema Quarter-Car)
% Stati: x = [delta_s; zs_dot; delta_t; zu_dot]
A = [ 0,            1,          0,           -1;
     -ks0/m_s,     -bs/m_s,     0,            bs/m_s;
      0,            0,          0,            1;
      ks0/m_u,      bs/m_u,    -kt/m_u,     -(bs+bt)/m_u ];

% 3. Analisi Modale Teorica (Estrazione Frequenze Naturali)
disp('======================================================');
disp('   ANALISI MODALE (Autovalori della matrice A)');
disp('======================================================');
[Wn, Z, P] = damp(A);
% Wn = Frequenze naturali [rad/s]
% Z  = Coefficienti di smorzamento

% 4. Costruzione Sistema Passivo per Bode
B_w = [ 0; 0; -1; bt/m_u ]; % Ingresso disturbo w
C_out = [ -ks0/m_s, -bs/m_s, 0, bs/m_s;  % 1. Accelerazione cassa (zs_ddot)
           0,        0,      1, 0      ]; % 2. Deformazione pneumatico (delta_t)
D_out = [ 0; 0 ];

Sys_passivo = ss(A, B_w, C_out, D_out);
Sys_passivo.InputName = {'w'};
Sys_passivo.OutputName = {'zs\_ddot', 'delta\_t'};

% 5. Tracciamento Diagramma di Bode
w_vec = logspace(-1, 3, 1000);

figure('Name', 'Bode Sistema Passivo - Verifica Risonanze', 'Color', 'w');
bode(Sys_passivo, w_vec);
grid on;
title('Analisi in Frequenza: Disturbo (w) \rightarrow Uscite (Passivo)');