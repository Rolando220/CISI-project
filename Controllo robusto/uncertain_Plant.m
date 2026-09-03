%% --- 1. MODELLAZIONE INCERTEZZE CONCENTRATE (ATTUATORI) ---
s = tf('s');

% Parametri nominali attuatori
tau_1_nom = 0.005; % 5 ms
tau_2_nom = 0.010; % 10 ms
omega_n1 = 80; zeta_1 = 0.7; tau_a = 0.025;

% Creiamo un array di possibili ritardi (+/- 20%)
tau_1_array = linspace(tau_1_nom*0.8, tau_1_nom*1.2, 10);
tau_2_array = linspace(tau_2_nom*0.8, tau_2_nom*1.2, 10);

% Inizializziamo array di funzioni di trasferimento
G_a1_array = tf(zeros(1,1,10));
G_a2_array = tf(zeros(1,1,10));

% Popoliamo gli array con i modelli perturbati (usando Padé 1° ordine)
for i = 1:10
    Pade1 = (1 - (tau_1_array(i)/2)*s) / (1 + (tau_1_array(i)/2)*s);
    Pade2 = (1 - (tau_2_array(i)/2)*s) / (1 + (tau_2_array(i)/2)*s);
    
    G_a1_array(:, :, i) = (omega_n1^2 / (s^2 + 2*zeta_1*omega_n1*s + omega_n1^2)) * Pade1;
    G_a2_array(:, :, i) = (1 / (1 + tau_a*s)) * Pade2;
end

% Modelli nominali
Pade1_nom = (1 - (tau_1_nom/2)*s) / (1 + (tau_1_nom/2)*s);
Pade2_nom = (1 - (tau_2_nom/2)*s) / (1 + (tau_2_nom/2)*s);
G_a1_nom = (omega_n1^2 / (s^2 + 2*zeta_1*omega_n1*s + omega_n1^2)) * Pade1_nom;
G_a2_nom = (1 / (1 + tau_a*s)) * Pade2_nom;


% Trasformo in Frequency Response Data (per W_L2)
w_vec = logspace(-1, 3, 200); % Vettore di frequenze da 0.1 a 1000 rad/s
G_a1_frd = frd(G_a1_array, w_vec);
G_a1_nom_frd = frd(G_a1_nom, w_vec);
G_a2_frd = frd(G_a2_array, w_vec);
G_a2_nom_frd = frd(G_a2_nom, w_vec);

% --- UCOVER: Calcolo del peso moltiplicativo sui dati FRD ---
order_W = 2; % Ordine del filtro peso
[~, info_u1] = ucover(G_a1_frd, G_a1_nom_frd, order_W, 'InputMult');
[~, info_u2] = ucover(G_a2_frd, G_a2_nom_frd, order_W, 'InputMult');

W_L1 = tf(info_u1.W1); % Peso incertezza attuatore 1
W_L2 = tf(info_u2.W1); % Peso incertezza attuatore 2


% Grafico di verifica
figure('Name', 'Incertezza Moltiplicativa Attuatori');

subplot(2,1,1);
bodemag((G_a1_array - G_a1_nom)/G_a1_nom, 'b--', W_L1, 'r-', {0.1, 1000});
title('Copertura Incertezza Attuatore 1 (W_{L1})');
legend('Errore Impianti Perturbati', 'Filtro Peso W_{L1}');

subplot(2,1,2);
bodemag((G_a2_array - G_a2_nom)/G_a2_nom, 'b--', W_L2, 'r-', {0.1, 1000});
title('Copertura Incertezza Attuatore 2 (W_{L2})');
legend('Errore Impianti Perturbati', 'Filtro Peso W_{L2}');

%% --- 2. MODELLAZIONE INCERTEZZE STRUTTURATE (PLANT) ---

% Parametri nominali fissi
m_u = 50;       
k_t = 150000;   
b_t = 150;      

% Parametri Incerte Strutturate (Generano in automatico la forma LFT)
m_s_unc  = ureal('m_s', 350, 'Percentage', 20);       
k_s0_unc = ureal('k_s0', 20000, 'Percentage', 10);    
b_s_unc  = ureal('b_s', 1500, 'Percentage', 10);      

% Costruzione Matrici (MATLAB riconosce che sono incerte)
A_unc = [ 0,                    1,                0,                     -1;
         -k_s0_unc/m_s_unc,    -b_s_unc/m_s_unc,  0,                      b_s_unc/m_s_unc;
          0,                    0,                0,                      1;
          k_s0_unc/m_u,         b_s_unc/m_u,     -k_t/m_u,               -(b_s_unc+b_t)/m_u ];

B_unc = [ 0,          0,         0;
          1/m_s_unc,  0,         0;
          0,          0,        -1;
         -1/m_u,      1/m_u,     b_t/m_u ];

C_unc = [ -k_s0_unc/m_s_unc, -b_s_unc/m_s_unc,  0,        b_s_unc/m_s_unc;   
           1,                 0,                0,        0;        
           0,                 0,                1,        0;        
           k_s0_unc/m_u,      b_s_unc/m_u,     -k_t/m_u, -(b_s_unc+b_t)/m_u ]; 

D_unc = [ 1/m_s_unc,  0,         0;
          0,          0,         0;
          0,          0,         0;
         -1/m_u,      1/m_u,     b_t/m_u ];

Plant_unc = uss(A_unc, B_unc, C_unc, D_unc);
Plant_unc.InputName = {'u1_force', 'u2_force', 'w_dist'};
Plant_unc.OutputName = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};

%% --- 3. PESI SUI DISTURBI E RUMORI DI MISURA ---

% 3.1 Peso sul Disturbo Stradale (Gd)
% Il profilo stradale ha energia a bassa frequenza. 
% Usiamo un Low-Pass Filter (LPF) come descritto nella teoria.
G_d = 0.1 / (s/(2*pi*5) + 1); % Frequenza di taglio a 5 Hz (~30 rad/s)
G_d.InputName = 'w_in';       % Ingresso esogeno matematico (rumore bianco)
G_d.OutputName = 'w_dist';    % Uscita fisica (velocità della strada che entra nel Plant)

% % 3.2 Peso sui Rumori di Misura (Gn)
% % Il rumore dei sensori IMU si concentra ad alta frequenza.
% % Usiamo un High-Pass Filter (HPF) per sporcare le misurazioni.
% G_n = 0.01 * (s/(2*pi*1) + 1) / (s/(2*pi*100) + 1); 
% G_n.InputName = 'noise_in';
% G_n.OutputName = 'n_sens';

%% --- 4. INCERTEZZA CONCENTRATA (MOLTIPLICATIVA IN INGRESSO) ---
% Definiamo il blocco Delta_L matematico normalizzato (||Delta||_inf <= 1)
Delta_L1 = ultidyn('Delta_L1', [1 1]); 
Delta_L2 = ultidyn('Delta_L2', [1 1]);

% Attuatori Incerti (G_nom * (1 + W_L * Delta_L))
Actuator1_unc = G_a1_nom * (1 + W_L1 * Delta_L1);
Actuator1_unc.InputName = 'u1_cmd';
Actuator1_unc.OutputName = 'u1_force';

Actuator2_unc = G_a2_nom * (1 + W_L2 * Delta_L2);
Actuator2_unc.InputName = 'u2_cmd';
Actuator2_unc.OutputName = 'u2_force';

%% --- 5. INTERCONNESSIONE DEL GENERALIZED PLANT ---
% Sommiamo il rumore di misura alle uscite "pulite" del sistema (le accelerazioni)
Sum_IMU1 = sumblk('y_imu1_meas = zs_ddot + n_sens');
Sum_IMU2 = sumblk('y_imu2_meas = zu_ddot + n_sens');

% Creiamo il Processo Esteso (N-Delta) connettendo tutti i blocchi
% Input del sistema esteso: [w_in; u1_cmd; u2_cmd]
% Output del sistema esteso: [zs_ddot; delta_s; delt_t; zu_ddot]
P_esteso = connect(Plant_unc, Actuator1_unc, Actuator2_unc, G_d, ...
                   {'w_in', 'u1_cmd', 'u2_cmd'}, ...
                   {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

disp('Processo Esteso Incerto assemblato con successo!');