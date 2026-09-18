%% Stima Empirica 2D della RAS per MIXSYN, H-INF e PI STRUTTURATO
%------------------------------------------------------------------------
%   Si verifichi nel rispettivo file simulink che i disturbi esogeni 
%   siano posti a zero 
%   Si imposti nel rispettivo file simulink l'opzione "Fast Restart"
%------------------------------------------------------------------------

clear x0_delta_s x0_zs_dot x0_delta_t x0_zu_dot;
clc;

nome_modello = 'MixedSensitivity_quarter_car'; 
tolleranza = 1e-3;       % Soglia per considerare l'errore asintotico nullo (1 mm)
tempo_simulazione = 10;  % Tempo sufficiente per smaltire il transitorio iniziale

% a zero gli stati non esplorati
assignin('base', 'x0_delta_t', 0);
assignin('base', 'x0_zu_dot',  0);

% Definizione griglia 2D 
delta_s_vec = linspace(-0.15, 0.15, 15); 
zs_dot_vec  = linspace(-2.0, 2.0, 15);   

% Matrici per salvare i risultati (1 = Stabile, 0 = Instabile)
RAS_mixsyn = zeros(length(zs_dot_vec), length(delta_s_vec));
RAS_hinf   = zeros(length(zs_dot_vec), length(delta_s_vec));
RAS_pid    = zeros(length(zs_dot_vec), length(delta_s_vec));

disp('================================================================');
disp('   INIZIO VALUTAZIONE RAS 2D (MIXSYN, H-INF, PI STRUTTURATO)');
disp('================================================================');

% Creazione Waitbar
h_wait = waitbar(0, 'Inizializzazione...', 'Name', 'Calcolo RAS in corso', ...
                'CreateCancelBtn', 'setappdata(gcbf,''canceling'',1)');
setappdata(h_wait, 'canceling', 0);

num_totale = length(delta_s_vec) * length(zs_dot_vec);
iter = 0;
abortito = false;

for i = 1:length(delta_s_vec)
    for j = 1:length(zs_dot_vec)
        
        % Controllo se l'utente ha premuto "Cancel" sulla waitbar
        if getappdata(h_wait, 'canceling')
            disp('*** VALUTAZIONE ANNULLATA DALL''UTENTE ***');
            abortito = true;
            break;
        end
        
        iter = iter + 1;
        
        val_ds = delta_s_vec(i);
        val_dz = zs_dot_vec(j);
        
        % Aggiorna la waitbar
        waitbar(iter/num_totale, h_wait, sprintf('Test %d di %d\ndelta_s=%.3f, zs_dot=%.3f', iter, num_totale, val_ds, val_dz));
        
        % condizioni iniziali per l'iterazione corrente
        assignin('base', 'x0_delta_s', val_ds); 
        assignin('base', 'x0_zs_dot',  val_dz); 
        
        fprintf('Test punto [%d/%d]: delta_s = %5.3f m, zs_dot = %5.3f m/s ... ', ...
                iter, num_totale, val_ds, val_dz);
        
        try
            % Esecuzione modello Simulink
            out = sim(nome_modello, 'StopTime', num2str(tempo_simulazione), 'ReturnWorkspaceOutputs', 'on');
            
            % Estrazione Dati
            data_mixsyn = out.delta_s_mixsyn.Data; 
            data_hinf   = out.delta_s_hinf.Data;
            data_pid    = out.delta_s_PID.Data;
            
            % Verifica MIXSYN
            err_mix = max(abs(data_mixsyn(end-50:end))); 
            if err_mix < tolleranza && ~any(isnan(data_mixsyn))
                RAS_mixsyn(j, i) = 1; 
            end
            
            % Verifica H-INF
            err_hinf = max(abs(data_hinf(end-50:end))); 
            if err_hinf < tolleranza && ~any(isnan(data_hinf))
                RAS_hinf(j, i) = 1; 
            end
            
            % Verifica PI Strutturato
            err_pid = max(abs(data_pid(end-50:end)));
            if err_pid < tolleranza && ~any(isnan(data_pid))
                RAS_pid(j, i) = 1; 
            end
            
            fprintf('OK\n');
        catch
            % Se la simulazione crasha
            fprintf('INSTABILE/CRASH\n');
        end
    end
    
    if abortito
        break; 
    end
end

% Chiude la waitbar alla fine 
if ishandle(h_wait)
    delete(h_wait);
end

if ~abortito
    disp('Simulazioni completate! Generazione Grafici...');

    % Plot risultati 
    [X, Y] = meshgrid(delta_s_vec, zs_dot_vec);

    figure('Name', 'Confronto RAS - Controllori H-inf', 'Position', [100, 100, 1400, 400]);

    % --- PLOT 1: MIXSYN ---
    subplot(1,3,1);
    gscatter(X(:), Y(:), RAS_mixsyn(:), 'rg', 'x.', 15, 15);
    title('RAS - MixSyn');
    xlabel('Corsa Iniziale \delta_s [m]');
    ylabel('Velocità Cassa \cdot{z}_s [m/s]');
    legend('Instabile', 'Stabile', 'Location', 'best');
    grid on; axis square;

    % --- PLOT 2: H-INF STANDARD ---
    subplot(1,3,2);
    gscatter(X(:), Y(:), RAS_hinf(:), 'rg', 'x.', 15, 15);
    title('RAS - H_\infty (hinfsyn)');
    xlabel('Corsa Iniziale \delta_s [m]');
    ylabel('Velocità Cassa \cdot{z}_s [m/s]');
    legend('Instabile', 'Stabile', 'Location', 'best');
    grid on; axis square;

    % --- PLOT 3: PI STRUTTURATO ---
    subplot(1,3,3);
    gscatter(X(:), Y(:), RAS_pid(:), 'rg', 'x.', 15, 15);
    title('RAS - PI Strutturato');
    xlabel('Corsa Iniziale \delta_s [m]');
    ylabel('Velocità Cassa \cdot{z}_s [m/s]');
    legend('Instabile', 'Stabile', 'Location', 'best');
    grid on; axis square;
end