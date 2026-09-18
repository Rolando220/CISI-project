%% ANALISI PRESTAZIONI BUMP (Ostacolo Isolato)
clc;
fprintf('=========================================================\n');
fprintf(' === ANALISI RISPOSTA AL BUMP (Picchi e Assestamento) ===\n');
fprintf('=========================================================\n\n');

% Estrazione vettore tempo
t = out.zs_ddot_passivo.Time;
t_bump = 1.0; % Istante in cui la ruota incontra il dosso

% Estrazione dati Accelerazione Cassa
acc_p   = squeeze(out.zs_ddot_passivo.Data);
acc_rob = squeeze(out.zs_ddot_krob.Data);
acc_red = squeeze(out.zs_ddot_kred.Data);

% Estrazione dati Deformazione Gomma (Tenuta di strada)
dt_p   = squeeze(out.delta_t_passivo.Data);
dt_rob = squeeze(out.delta_t_krob.Data);
dt_red = squeeze(out.delta_t_kred.Data);

%% 1. DEFINIZIONE SOGLIE DI ASSESTAMENTO
% Definiamo quando il sistema è considerato "tornato a riposo"
soglia_acc = 0.15;   % Tolleranza accelerazione [m/s^2]
soglia_dt  = 0.0005; % Tolleranza deformazione gomma [m] (0.5 mm)

% Funzione anonima per trovare il tempo di assestamento dopo il bump
% Trova l'ultimo istante in cui il segnale supera la soglia, e sottrae t_bump
calc_Ts = @(data, soglia) t(find(abs(data) > soglia, 1, 'last')) - t_bump;

%% 2. CALCOLI E STAMPE: COMFORT VIBRAZIONALE (zs_ddot)
fprintf('--- COMFORT (Accelerazione Cassa zs_ddot) ---\n');

% Passivo
max_acc_p = max(acc_p); min_acc_p = min(acc_p);
ts_acc_p = calc_Ts(acc_p, soglia_acc);
fprintf('PASSIVO  -> Max: %+.4f | Min: %+.4f | Tempo Assestamento: %.2f s\n', max_acc_p, min_acc_p, ts_acc_p);

% K_rob
max_acc_rob = max(acc_rob); min_acc_rob = min(acc_rob);
ts_acc_rob = calc_Ts(acc_rob, soglia_acc);
fprintf('K_rob    -> Max: %+.4f | Min: %+.4f | Tempo Assestamento: %.2f s\n', max_acc_rob, min_acc_rob, ts_acc_rob);

% K_red
max_acc_red = max(acc_red); min_acc_red = min(acc_red);
ts_acc_red = calc_Ts(acc_red, soglia_acc);
fprintf('K_red    -> Max: %+.4f | Min: %+.4f | Tempo Assestamento: %.2f s\n\n', max_acc_red, min_acc_red, ts_acc_red);

%% 3. CALCOLI E STAMPE: TENUTA DI STRADA (delta_t)
fprintf('--- ROAD HOLDING (Deformazione Gomma delta_t) ---\n');

% Passivo
max_dt_p = max(dt_p); min_dt_p = min(dt_p);
ts_dt_p = calc_Ts(dt_p, soglia_dt);
fprintf('PASSIVO  -> Max: %+.5f | Min: %+.5f | Tempo Assestamento: %.2f s\n', max_dt_p, min_dt_p, ts_dt_p);

% K_rob
max_dt_rob = max(dt_rob); min_dt_rob = min(dt_rob);
ts_dt_rob = calc_Ts(dt_rob, soglia_dt);
fprintf('K_rob    -> Max: %+.5f | Min: %+.5f | Tempo Assestamento: %.2f s\n', max_dt_rob, min_dt_rob, ts_dt_rob);

% K_red
max_dt_red = max(dt_red); min_dt_red = min(dt_red);
ts_dt_red = calc_Ts(dt_red, soglia_dt);
fprintf('K_red    -> Max: %+.5f | Min: %+.5f | Tempo Assestamento: %.2f s\n', max_dt_red, min_dt_red, ts_dt_red);

fprintf('\n=========================================================\n');