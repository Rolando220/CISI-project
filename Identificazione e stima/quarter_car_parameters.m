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
params.A   = 0.015;   % altezza bump 1.5 cm
params.L   = 0.5;    % lunghezza bump 50 cm
params.v   = 10;     % velocità veicolo 36 km/h
params.f   = 3;      % frequenza sinusoide [Hz] 
params.t0  = 1.0;    % istante gradino [s]
params.tau = 0.05;   % tempo di salita gradino [s]


%% Inizializzazione 
%Tempo di campionamento (il più veloce - 100Hz) 
parameters.dt_ekf=0.01; 

% Stato inziale 
% x0_hat=[0 ; 0; 0; 0]; 
% 
% % Incertezza iniziale 
% P0 = eye(4)*10;

% Covarianza del Rumore di Processo (Q) 
parameters.Q= 0.1; 

parameters.R_imu = 0.01; 








