clear all
close all
clc

parameters.m_s = 350;
parameters.m_u = 50;
parameters.ks0 = 20000;
parameters.alpha = 10^5;
parameters.bs = 1500;
parameters.kt = 180000;
parameters.bt = 150;

omega_n1 = 80;       
zeta_1 = 0.7;        
tau_1 = 0.005;       

tau_a = 0.025;       
tau_2 = 0.010;       


%% Disturbo strada 

%parametri 

% Parametri 
% Parametri Strada
params.A   = 0.05;   % altezza bump 5 cm (non 20!)
params.L   = 1.0;    % lunghezza bump 1 m
params.v   = 30/3.6; % velocità veicolo 30 km/h in m/s
params.f   = 2;      % frequenza sinusoide [Hz]
params.t0  = 1.0;    % istante inizio ostacolo [s]
params.tau = 0.03;   % tempo di salita gradino [s]



%% Inizializzazione 
%Tempo di campionamento (il più veloce - 100Hz) 
parameters.dt_ekf=0.01; 

% Stato inziale 
% x0_hat=[0 ; 0; 0; 0]; 
% 
% % Incertezza iniziale 
% P0 = eye(4)*10;

% Covarianza del Rumore di Processo (Q) 
parameters.Q= 0.01; 

parameters.R_imu = 0.01; 








