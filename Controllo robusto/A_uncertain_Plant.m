%% MODELLAZIONE INCERTEZZE CONCENTRATE (ATTUATORI) ---
s = tf('s');

% Parametri nominali attuatori
tau_1_nom = parameters.tau_1; % 5 ms
tau_2_nom = parameters.tau_2; % 10 ms
omega_n1 = parameters.omega_n1; zeta_1 = parameters.zeta_1; tau_a = parameters.tau_a;

% Array di possibili ritardi (+/- 20%)
tau_1_array = linspace(tau_1_nom*0.8, tau_1_nom*1.2, 10);
tau_2_array = linspace(tau_2_nom*0.8, tau_2_nom*1.2, 10);

% Inizializzazione array di funzioni di trasferimento
G_a1_array = tf(zeros(1,1,10));
G_a2_array = tf(zeros(1,1,10));

% Popolamento array con modelli perturbati (usando Padé 1° ordine)
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


% Trasformazione in Frequency Response Data (FRD per W_L2)
w_vec = logspace(-1, 3, 200); % Vettore di frequenze da 0.1 a 1000 rad/s
G_a1_frd = frd(G_a1_array, w_vec);
G_a1_nom_frd = frd(G_a1_nom, w_vec);
G_a2_frd = frd(G_a2_array, w_vec);
G_a2_nom_frd = frd(G_a2_nom, w_vec);

% UCOVER: Calcolo del peso moltiplicativo sui dati FRD 
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

%% MODELLAZIONE INCERTEZZE STRUTTURATE (PLANT) 

% Parametri nominali
m_u = parameters.m_u;       
k_t = parameters.kt;
k_s0 = parameters.ks0;
b_t = parameters.bt;
b_s = parameters.bs;

% Parametri Incerti 
m_s_unc  = ureal('m_s', 350, 'Percentage', 6);       
% k_s0_unc = ureal('k_s0', 20000, 'Percentage', 10);    
% b_s_unc  = ureal('b_s', 1500, 'Percentage', 10);      

% Costruzione Matrici Incerte 
A_unc = [ 0,                    1,                0,                     -1;
         -k_s0/m_s_unc,    -b_s/m_s_unc,  0,                      b_s/m_s_unc;
          0,                    0,                0,                      1;
          k_s0/m_u,         b_s/m_u,     -k_t/m_u,               -(b_s+b_t)/m_u ];

B_unc = [ 0,          0,         0;
          1/m_s_unc,  0,         0;
          0,          0,        -1;
         -1/m_u,      1/m_u,     b_t/m_u ];

C_unc = [ -k_s0/m_s_unc, -b_s/m_s_unc,  0,        b_s/m_s_unc;   
           1,                 0,                0,        0;        
           0,                 0,                1,        0;        
           k_s0/m_u,      b_s/m_u,     -k_t/m_u, -(b_s+b_t)/m_u ]; 

D_unc = [ 1/m_s_unc,  0,         0;
          0,          0,         0;
          0,          0,         0;
         -1/m_u,      1/m_u,     b_t/m_u ];

Plant_unc = uss(A_unc, B_unc, C_unc, D_unc);
Plant_unc.InputName = {'u1_force', 'u2_force', 'w_dist'};
Plant_unc.OutputName = {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'};

%% PESO SUI DISTURBI 

% Peso sul Disturbo Stradale (Gd)
% Il profilo stradale ha energia a bassa frequenza. 
% Low-Pass Filter (LPF) 
G_d = 1 / (s/(2*pi*5) + 1); % Frequenza di taglio a 5 Hz (~30 rad/s)
G_d.InputName = 'w_in';       % Ingresso Rumore Bianco 
G_d.OutputName = 'w_dist';    % Uscita fisica (velocità della strada che entra nel Plant)


%% INCERTEZZA CONCENTRATA (MOLTIPLICATIVA IN INGRESSO) 
% Blocco Delta_L normalizzato (||Delta||_inf <= 1)
Delta_L1 = ultidyn('Delta_L1', [1 1]); 
Delta_L2 = ultidyn('Delta_L2', [1 1]);

% Attuatori Incerti (G_nom * (1 + W_L * Delta_L))
Actuator1_unc = G_a1_nom * (1 + W_L1 * Delta_L1);
Actuator1_unc.InputName = 'u1_cmd';
Actuator1_unc.OutputName = 'u1_force';

Actuator2_unc = G_a2_nom * (1 + W_L2 * Delta_L2);
Actuator2_unc.InputName = 'u2_cmd';
Actuator2_unc.OutputName = 'u2_force';

%% INTERCONNESSIONE PER IMPIANTO GENERALIZZATO 

% Crezione Processo Esteso (N-Delta) connettendo tutti i blocchi
% Input del sistema esteso: [w_in; u1_cmd; u2_cmd]
% Output del sistema esteso: [zs_ddot; delta_s; delt_t; zu_ddot]
P_esteso = connect(Plant_unc, Actuator1_unc, Actuator2_unc, G_d, ...
                   {'w_in', 'u1_cmd', 'u2_cmd'}, ...
                   {'zs_ddot', 'delta_s', 'delta_t', 'zu_ddot'});

disp('Processo Esteso Incerto assemblato con successo!');