%% filter_stats_table.m
% Calcola e visualizza le statistiche degli errori EKF e Particle Filter
% Include il calcolo dell'RMS a regime (escludendo il transitorio iniziale)
clc;

%% ── 0. Parametri di Analisi ─────────────────────────────────────────────────
t_regime = 2.0; % Tempo di transitorio (in secondi) da scartare per l'RMS a regime

%% ── 1. Nomi dei segnali e variabili workspace ───────────────────────────────
signal_names = {'e\_delta\_s', 'e\_zs\_dot', 'e\_delta\_t', 'e\_zu\_dot'};
units        = {'[m]',         '[m/s]',       '[m]',         '[m/s]'};
ekf_vars = {'ekf_e_delta_s', 'ekf_e_zs_dot', 'ekf_e_delta_t', 'ekf_e_zu_dot'};
pf_vars  = {'pf_e_delta_s',  'pf_e_zs_dot',  'pf_e_delta_t',  'pf_e_zu_dot'};

%% ── 2. Estrazione dati dal workspace ────────────────────────────────────────
function [data, time] = extract_signal(varname)
    if evalin('base', 'exist(''out'', ''var'')')
        out_data = evalin('base', 'out');
        if isprop(out_data, varname)
            ts = out_data.(varname); 
            data = ts.Data(:);
            time = ts.Time(:); % Estraiamo anche il vettore tempo
        else
            warning('Segnale "%s" non trovato dentro "out".', varname);
            data = zeros(100, 1);
            time = linspace(0, 10, 100)';
        end
    else
        error('La variabile "out" non è presente nel Workspace!');
    end
end

%% ── 3. Funzione statistiche (incluso RMS a regime) ──────────────────────────
function s = compute_stats(v, t, t_regime)
    v = v(:);  
    t = t(:);
    
    s.min      = min(v);
    s.max      = max(v);
    s.p2p      = max(v) - min(v);
    s.mean     = mean(v);
    s.variance = var(v);
    s.std      = std(v);
    s.median   = median(v);
    s.rms      = rms(v);
    s.ms       = mean(v.^2);
    
    % Calcolo RMS a Regime
    idx_regime = (t >= t_regime);
    if any(idx_regime)
        s.rms_reg = rms(v(idx_regime));
    else
        s.rms_reg = NaN; % Se la simulazione dura meno di t_regime
    end
end

%% ── 4. Calcola le statistiche per tutti gli 8 segnali ──────────────────────
stat_fields = {'min','max','p2p','mean','variance','std','median','rms','rms_reg','ms'};
stat_labels = {'Min','Max','Peak-to-Peak','Mean','Variance','Std Dev','Median','RMS (Totale)','RMS (Regime)','Mean Square'};
n_signals = length(signal_names);
n_stats   = length(stat_fields);
ekf_stats = cell(n_signals, n_stats);
pf_stats  = cell(n_signals, n_stats);

for i = 1:n_signals
    [ekf_data, ekf_time] = extract_signal(ekf_vars{i});
    [pf_data, pf_time]   = extract_signal(pf_vars{i});
    
    s_ekf = compute_stats(ekf_data, ekf_time, t_regime);
    s_pf  = compute_stats(pf_data, pf_time, t_regime);
    
    for j = 1:n_stats
        ekf_stats{i,j} = s_ekf.(stat_fields{j});
        pf_stats{i,j}  = s_pf.(stat_fields{j});
    end
end

%% ── 5. Costruisci la tabella MATLAB ─────────────────────────────────────────
row_names = stat_labels';
col_names = {};
col_data  = zeros(n_stats, n_signals * 2);

for i = 1:n_signals
    base = strrep(signal_names{i}, '\', '');  
    col_names{end+1} = ['EKF_' strrep(base,'_','')];
    col_names{end+1} = ['PF_'  strrep(base,'_','')];
    
    for j = 1:n_stats
        col_data(j, (i-1)*2 + 1) = ekf_stats{i,j};
        col_data(j, (i-1)*2 + 2) = pf_stats{i,j};
    end
end

T = array2table(col_data, 'RowNames', row_names, 'VariableNames', col_names);
disp('══════════════════════════════════════════════════════════════════════')
disp('   TABELLA COMPARATIVA ERRORI — EKF vs Particle Filter')
disp('══════════════════════════════════════════════════════════════════════')
disp(T)

%% ── 6. Salva nel workspace e su file ────────────────────────────────────────
assignin('base', 'stats_table', T);
assignin('base', 'ekf_stats',   ekf_stats);
assignin('base', 'pf_stats',    pf_stats);
writetable(T, 'filter_error_stats.csv', 'WriteRowNames', true);
fprintf('\nTabella salvata in: filter_error_stats.csv\n');
fprintf('Tabella disponibile nel workspace come: stats_table\n\n');

%% ── 7. Stampa tabelle separate per leggibilità ──────────────────────────────
fprintf('\n%s\n', repmat('─',1,76));
fprintf('  EKF — Statistiche errori\n');
fprintf('%s\n', repmat('─',1,76));
fprintf('  %-16s  %-14s  %-14s  %-14s  %-14s\n', ...
    'Statistica', signal_names{1}, signal_names{2}, signal_names{3}, signal_names{4});
fprintf('%s\n', repmat('─',1,76));
for j = 1:n_stats
    fprintf('  %-16s', stat_labels{j});
    for i = 1:n_signals
        fprintf('  %+12.6e', ekf_stats{i,j});
    end
    fprintf('\n');
end

fprintf('\n%s\n', repmat('─',1,76));
fprintf('  PF — Statistiche errori\n');
fprintf('%s\n', repmat('─',1,76));
fprintf('  %-16s  %-14s  %-14s  %-14s  %-14s\n', ...
    'Statistica', signal_names{1}, signal_names{2}, signal_names{3}, signal_names{4});
fprintf('%s\n', repmat('─',1,76));
for j = 1:n_stats
    fprintf('  %-16s', stat_labels{j});
    for i = 1:n_signals
        fprintf('  %+12.6e', pf_stats{i,j});
    end
    fprintf('\n');
end

%% ── 8. Figure: confronto RMS Totale vs RMS a Regime ─────────────────────────
ekf_rms     = zeros(1, n_signals);
pf_rms      = zeros(1, n_signals);
ekf_rms_reg = zeros(1, n_signals);
pf_rms_reg  = zeros(1, n_signals);

for i = 1:n_signals
    ekf_rms(i)     = ekf_stats{i, strcmp(stat_fields,'rms')};
    pf_rms(i)      = pf_stats{i,  strcmp(stat_fields,'rms')};
    ekf_rms_reg(i) = ekf_stats{i, strcmp(stat_fields,'rms_reg')};
    pf_rms_reg(i)  = pf_stats{i,  strcmp(stat_fields,'rms_reg')};
end

x = 1:n_signals;
labels_clean = strrep(signal_names, '\', '');

figure('Name','Confronto EKF vs PF — Analisi RMS', 'NumberTitle','off', ...
       'Position',[100 100 900 400]);

% Subplot 1: RMS Totale (include il transitorio)
subplot(1,2,1);
bar(x, [ekf_rms; pf_rms]', 0.6);
set(gca, 'XTick', x, 'XTickLabel', labels_clean);
legend('EKF','PF','Location','northwest');
title('RMS Totale (Transitorio incluso)');
ylabel('RMS');
grid on;

% Subplot 2: RMS a Regime (esclude i primi 2 secondi)
subplot(1,2,2);
bar(x, [ekf_rms_reg; pf_rms_reg]', 0.6);
set(gca, 'XTick', x, 'XTickLabel', labels_clean);
legend('EKF','PF','Location','northwest');
title(sprintf('RMS a Regime (dopo %.1f s)', t_regime));
ylabel('RMS');
grid on;

sgtitle('Confronto prestazioni EKF vs Particle Filter');