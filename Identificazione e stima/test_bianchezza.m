% 1. Estrazione dei dati dell'innovazione da Simulink
inn = squeeze(out.innovazione_out.Data); 

% Impostiamo quanti ritardi (lags) guardare per il test
max_lag = 1000;

figure('Name', 'Test di Bianchezza di Anderson', 'Color', 'w');
nomi_sensori = {'Innovazione IMU Massa Sospesa (100 Hz)', 'Innovazione IMU Ruota (50 Hz)'};

for i = 1:2
    subplot(2, 1, i);
    
    % --- MODIFICA CRUCIALE: Pulizia dei NaN ---
    % Estraiamo la riga del sensore corrente
    inn_grezza = inn(i, :);
    
    if i == 2 
        % Siamo sul sensore Ruota (50Hz), ne prendiamo uno ogni due
        % Assumendo che il dato valido sia sempre alla posizione 1, 3, 5...
        inn_pulita = inn_grezza(1:2:end); 
    else
        % Siamo sul sensore Carrozzeria (100Hz), prendiamo tutto
        inn_pulita = inn_grezza;
    end
    % Ora rimuoviamo eventuali NaN residui (se presenti)
    inn_pulita = inn_pulita(~isnan(inn_pulita));

    % Calcoliamo il VERO numero di campioni fisici per questo sensore
    N_reale = length(inn_pulita);
    
    % Calcolo dei limiti di Anderson specifici per questo sensore
    bound = 1.96 / sqrt(N_reale);
    % -----------------------------------------
    
    % Assicuriamoci che l'innovazione sia a media nulla usando i dati puliti
    inn_centered = inn_pulita - mean(inn_pulita);
    
    % Calcolo dell'autocorrelazione normalizzata
    [r, lags] = xcorr(inn_centered, max_lag/i, 'coeff');
    
    % Estraiamo solo i ritardi positivi (lag > 0)
    idx_pos = find(lags > 0);
    lags_pos = lags(idx_pos);
    r_pos = r(idx_pos);
    
    % Disegniamo il grafico
    stem(lags_pos, r_pos, 'b', 'filled', 'MarkerSize', 4);
    hold on; grid on;
    
    % Disegniamo i limiti di Anderson calcolati per N_reale
    yline(bound, 'r--', 'LineWidth', 1.5, 'DisplayName', '+95% Bound');
    yline(-bound, 'r--', 'LineWidth', 1.5, 'DisplayName', '-95% Bound');
    yline(0, 'k', 'LineWidth', 1);
    
    % Estetica e Titoli
    title(nomi_sensori{i}, 'FontSize', 11, 'FontWeight', 'bold');
    xlabel('Ritardo k (Lags)');
    ylabel('Autocorrelazione');
    ylim([-max(0.15, bound*1.5), max(0.15, bound*1.5)]);
    
    % Calcolo percentuale per il report
    punti_fuori = sum(abs(r_pos) > bound);
    percentuale_fuori = (punti_fuori / length(lags_pos)) * 100;
    
    subtitle(sprintf('Campioni: %d | Punti fuori: %.1f%% (Superato se < 5%%)', N_reale, percentuale_fuori));
end