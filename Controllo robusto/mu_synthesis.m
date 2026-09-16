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

C = [ -k_s/m_s, -b_s/m_s, 0, b_s/m_s;                   % y1: zs_ddot
       1, 0, 0, 0;                                      % y2: delta_s
       0, 0, 1, 0;                                      % y3: delta_t
       k_s/m_u, b_s/m_u, -k_t/m_u, -(b_s+b_t)/m_u ];    % y4: zu_ddot

D = [ 1/m_s,   0,      0;
      0,       0,      0;
      0,       0,      0;
     -1/m_u,   1/m_u,  b_t/m_u ];

%% =========================================================
%  PERFORMANCE AND CONTROL WEIGHTS
%% =========================================================
M = 2.0; 
s = tf('s');

% 1. Comfort (zs_ddot): IL SEGRETO E' IL FILTRO PASSA-BANDA. 
% L'accelerazione a regime è SEMPRE zero. Il peso deve essere zero a w=0, 
% altrimenti l'algoritmo impazzisce cercando di attenuare un segnale già nullo!
% Usiamo un passa-banda centrato tra 5 e 50 rad/s.
% wP1 = 0.4 * (s / (s + 1)) * (50 / (s + 5)); 

% Filtro passa-banda Risonante (Stretto e mirato)
w_n = 3;        % Centro della valle (3 rad/s, la frequenza del tuo test!)
zeta = 0.5;     % Larghezza: più è piccolo, più la valle è stretta (0.2 è un "cecchino")
Gain = 0.5;     % Profondità della valle (quanto vogliamo schiacciare l'errore)

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

wT_base = 2 * (s + 20) / (0.01*s + 200);
WT = blkdiag(wT_base, wT_base, wT_base, wT_base);

% --- 3. COSTRUZIONE GENERALIZED PLANT ---
WP = tf(WP);
WP.InputName  = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};
WP.OutputName = {'z_p1', 'z_p2', 'z_p3', 'z_p4'};

Wu = tf(Wu);
Wu.InputName  = {'u1_cmd', 'u2_cmd'};
Wu.OutputName = {'z_u1', 'z_u2'};

% Nodi per le misure
Sv1 = sumblk('v1 = -zs_ddot');
Sv2 = sumblk('v2 = -delta_s');
Sv3 = sumblk('v3 = -delta_t');
Sv4 = sumblk('v4 = -zu_ddot');

% Ingressi: w_in (disturbo fisico) e u1, u2
% Uscite: z (prestazioni pesate) e v (misure)
P_gen_unc = connect(P_esteso, WP, Wu, Sv1, Sv2, Sv3, Sv4, ...
                    {'w_in', 'u1_cmd', 'u2_cmd'}, ...
                    {'z_p1', 'z_p2', 'z_p3', 'z_p4', 'z_u1', 'z_u2', 'v1', 'v2', 'v3', 'v4'});

% --- 4. MU-SYNTHESIS (DK-ITERATION VIA MUSYN) ---

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





% Plant_nom = ss(A,B,C,D);
% Plant_nom.InputName  = {'u1_force','u2_force','w_dist'};
% Plant_nom.OutputName = {'zs_ddot','delta_s','delta_t','zu_ddot'};
% 
% Actuator1_nom = G_a1_nom; Actuator1_nom.InputName='u1_cmd'; Actuator1_nom.OutputName='u1_force';
% Actuator2_nom = G_a2_nom; Actuator2_nom.InputName='u2_cmd'; Actuator2_nom.OutputName='u2_force';
% 
% P_esteso_nom = connect(Plant_nom, Actuator1_nom, Actuator2_nom, G_d, ...
%                         {'w_in','u1_cmd','u2_cmd'}, ...
%                         {'zs_ddot','delta_s','delta_t','zu_ddot'});
% 
% P_gen_nom = connect(P_esteso_nom, WP, Wu, Sv1, Sv2, Sv3, Sv4, ...
%                      {'w_in','u1_cmd','u2_cmd'}, ...
%                      {'z_p1','z_p2','z_p3','z_p4','z_u1','z_u2','v1','v2','v3','v4'});
% 
% [K_nom, CL_nom, gamma_nom] = hinfsyn(P_gen_nom, NMEAS, NCONT);
% gamma_nom
%
% scale_list = [1 0.5 0.3 0.2 0.15 0.125 0.1];
% for k = scale_list
%     wP1_k = k * 5*(s/(s+2))*(20/(s+20));
%     WP_k  = tf(blkdiag(wP1_k, wP2, wP3, wP4));
%     WP_k.InputName  = {'zs_ddot','delta_s','delta_t','zu_ddot'};
%     WP_k.OutputName = {'z_p1','z_p2','z_p3','z_p4'};
% 
%     P_gen_nom_k = connect(P_esteso_nom, WP_k, Wu, Sv1,Sv2,Sv3,Sv4, ...
%         {'w_in','u1_cmd','u2_cmd'}, ...
%         {'z_p1','z_p2','z_p3','z_p4','z_u1','z_u2','v1','v2','v3','v4'});
% 
%     [~,~,gamma_k] = hinfsyn(P_gen_nom_k, NMEAS, NCONT);
%     fprintf('k = %.3f  ->  gamma_nom = %.3f\n', k, gamma_k);
% end