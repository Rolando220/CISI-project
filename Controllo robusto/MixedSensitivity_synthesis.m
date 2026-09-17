%% Mixed-Sensitivity H-infinity Synthesis: MIMO Quarter-Car
%
% Design objective: minimize ||[WP*S; Wu*KS]||_inf < gamma

clear all; close all; clc;

% --- CARICAMENTO PARAMETRI E CREAZIONE IMPIANTO INCERTO ---
run('A_quarter_car_parameters.m');
run('A_uncertain_Plant.m');

s = tf('s');

%% =========================================================
%  MIMO PLANT (Quarter-Car)
%% =========================================================
% Parametri del modello nominale
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
       1, 0, 0, 0;                                      % y2: delta_s 
       0, 0, 1, 0;                                      % y3: delta_t (Tenuta strada)
       k_s/m_u, b_s/m_u, -k_t/m_u, -(b_s+b_t)/m_u ];    % y4: zu_ddot

D = [ 1/m_s,   0,      0;
      0,       0,      0;
      0,       0,      0;
     -1/m_u,   1/m_u,  b_t/m_u ];

%% =========================================================
%  PERFORMANCE AND CONTROL WEIGHTS
%% =========================================================

% Accelerazione della cassa sospesa: comfort
% L'accelerazione ha guadagno nullo a regime permanente. Per questo motivo
% il relativo peso viene scelto passa-banda, così da concentrare la
% penalizzazione nella banda di risonanza senza imporre vincoli a 0 rad/s.

% Peso passa-banda centrato sulla risonanza della carrozzeria.
% w_n    : frequenza centrale [rad/s]
% zeta   : smorzamento, determina l'ampiezza della banda
% Gain   : guadagno massimo del peso

w_n = 6.5;      
zeta = 0.6;     
Gain = 1.5;     

wP1 = Gain * (2 * zeta * w_n * s) / (s^2 + 2 * zeta * w_n * s + w_n^2);

% Autolivellamento (delta_s): peso costante di relax
wP2=0.01; 

% Tenuta di strada (delta_t)

w_n_ruota = 63;      
zeta_ruota = 0.6;    
Gain_ruota = 1.5;    

filtro_ruota = Gain_ruota * (2 * zeta_ruota * w_n_ruota * s) / (s^2 + 2 * zeta_ruota * w_n_ruota * s + w_n_ruota^2);
wP3 = 0.2 + filtro_ruota;

% Ruota (zu_ddot): peso costante di relax
wP4 = 0.01;

WP = blkdiag(wP1, wP2, wP3, wP4);

% Peso dello sforzo di controllo (Wu):
wu_LF = 1/40000;
wu_HF = 1/1000;     % Penalizzazione delle componenti ad alta frequenza
w_taglio = 80;      % Frequenza di transizione [rad/s]


% Penalizzazione dello sforzo di controllo:
% basso peso alle basse frequenze e peso maggiore alle alte frequenze,
% per limitare comandi rapidi e reazioni nervose degli attuatori.

wu = wu_HF * (s + w_taglio * (wu_LF/wu_HF)) / (s + w_taglio);
Wu = blkdiag(wu, wu);


% Nessun peso esplicito sulla sensibilità complementare: la sintesi mixsyn
% considera quindi solo il peso di prestazione e quello dello sforzo.

WT = [];

%% Costruzione del modello nominale e dell'impianto generalizzato

% Il modello ha:
%   3 ingressi fisici: u1_force, u2_force, w_dist
%   4 uscite: accelerazione cassa, corsa sospensione,
%             corsa pneumatico, accelerazione massa non sospesa

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
%% =========================================================
G_uy = P_esteso_nom(:, {'u1_cmd','u2_cmd'});
[K_mixsyn, CL, gamma, ~] = mixsyn(G_uy, WP, Wu, WT);
K_mixsyn = zpk(K_mixsyn);

fprintf('\nAchieved H-inf norm gamma = %.4f\n', gamma);
fprintf('Controller order: %d\n', order(K_mixsyn));

% Funzioni di sensibilità in anello chiuso con retroazione negativa
S_cl  = inv(eye(4) + G_uy*K_mixsyn);                % output sensitivity   (I+GK)^-1
KS_cl = K_mixsyn * inv(eye(4) + G_uy*K_mixsyn);     % control sensitivity  K*(I+GK)^-1
T_cl  = G_uy*K_mixsyn * inv(eye(4)+G_uy*K_mixsyn);  % comp. sensitivity    GK*(I+GK)^-1

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

%% =========================================================
%%  SINTESI ALTERNATIVA via hinfsyn()
%% =========================================================
fprintf('\n=== Sintesi Alternativa con hinfsyn ===\n');

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
if abs(gamma - gamma_hinf) < 1e-3
    fprintf('-> SUCCESSO: mixsyn e hinfsyn hanno prodotto lo stesso gamma!\n');
else
    fprintf('-> DIFFERENZA: gamma_mixsyn = %.4f, gamma_hinfsyn = %.4f\n', gamma, gamma_hinf);
end

%% =========================================================
%% H-INFINITY CON STRUTTURA FISSA (PI Industriale per Comfort)
%% =========================================================
fprintf('\n=== Sintesi H-infinity con PI Strutturato (Comfort) ===\n');

% Costruzione della struttura
% Azione proporzionale: matrice 2x4 che utilizza tutti i sensori
Kp = realp('Kp', zeros(2,4)); 

% Azione integrale filtrata sull'accelerazione della cassa.
% Il polo a 0.001 rad/s limita il guadagno alle frequenze prossime a zero,
% evitando il comportamento non limitato dell'integratore ideale.

Ki_base = realp('Ki_base', [0; 0]);
Selector_I = [1, 0, 0, 0]; % Seleziona il canale dell'accelerazione della cassa
I_action = (Ki_base / (s + 0.001)) * Selector_I;

% Assemblaggio
K_custom = Kp + I_action;

% Filtro passa-basso a 80 rad/s
wf = 80; 
LPF_single = tf(wf, [1, wf]);
LPF_matrix = blkdiag(LPF_single, LPF_single);

K_struct = LPF_matrix * K_custom;

% Ottimizzazione HINFSTRUCT
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

%% =========================================================
%%  ANALISI IN FREQUENZA MULTIPLA (Passivo vs H-inf vs PI)
%% =========================================================
fprintf('\n=== Generazione Grafici di Bode Multipli ===\n');

% Vettore di frequenze
w_vec = logspace(-1, 3, 1000); 

% Modello passivo: impianto senza attuazione e senza retroazione
G_passiva = Plant_nom(:, 'w_dist');

% Calcolo delle matrici di sensibilità e delle F.d.T. per tutti i controllori
% (Convenzione reazione negativa: S = inv(I + G*K))

% Controllore mixsyn (Full-Order)
S_mixsyn   = inv(eye(4) + G_uy * K_mixsyn);
G_a_mixsyn = minreal(S_mixsyn * G_passiva);

% Controllore hinfsyn (Full-Order)
S_hinf   = inv(eye(4) + G_uy * K_hinf);
G_a_hinf = minreal(S_hinf * G_passiva);

% Controllore PI strutturato (hinfstruct)
S_pid   = inv(eye(4) + G_uy * K_PID_tuned);
G_a_pid = minreal(S_pid * G_passiva);

% Estrazione delle singole F.d.T.
% Comfort: primo canale di uscita, w_in -> zs_ddot
G_p_comfort    = G_passiva(1, 1);
G_mix_comfort  = G_a_mixsyn(1, 1);
G_hinf_comfort = G_a_hinf(1, 1);
G_pid_comfort  = G_a_pid(1, 1);

% Tenuta di strada: terzo canale di uscita, w_in -> delta_t
G_p_tenuta    = G_passiva(3, 1);
G_mix_tenuta  = G_a_mixsyn(3, 1);
G_hinf_tenuta = G_a_hinf(3, 1);
G_pid_tenuta  = G_a_pid(3, 1);

% --- COMFORT VIBRAZIONALE ---
figure('Name', 'Bode - Confronto Controllori (Comfort)', 'Color', 'w');
% mixsyn in rosso continuo, hinfsyn in blu tratteggiato sovrapposto, PI in verde
bodemag(G_p_comfort, 'k--', G_mix_comfort, 'r', G_hinf_comfort, 'b:', G_pid_comfort, 'g', w_vec);
grid on;
legend('Passiva', 'mixsyn (Full-Order)', 'hinfsyn (Full-Order)', 'PI Strutturato', 'Location', 'southwest');
title('Amplificazione Disturbo: w_{in} \rightarrow Accelerazione Cassa (Comfort)');

% --- TENUTA DI STRADA ---
figure('Name', 'Bode - Confronto Controllori (Tenuta)', 'Color', 'w');
bodemag(G_p_tenuta, 'k--', G_mix_tenuta, 'r', G_hinf_tenuta, 'b:', G_pid_tenuta, 'g', w_vec);
grid on;
legend('Passiva', 'mixsyn (Full-Order)', 'hinfsyn (Full-Order)', 'PI Strutturato', 'Location', 'southwest');
title('Amplificazione Disturbo: w_{in} \rightarrow \delta_t (Tenuta di Strada)');