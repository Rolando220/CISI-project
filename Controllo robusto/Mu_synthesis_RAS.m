%% Stima Empirica 2D della RAS per Controllori Robusti (DK-Iteration)
%------------------------------------------------------------------------
%   Si verifichi nel rispettivo file simulink che i disturbi esogeni 
%   siano posti a zero 
%   Si imposti nel rispettivo file simulink l'opzione "Fast Restart"
%------------------------------------------------------------------------

clear x0_delta_s x0_zs_dot x0_delta_t x0_zu_dot;
clc;

warning('off', 'all');

nome_modello = 'Mu_synthesis_quarter_car'; 

tolleranza = 1e-3;       % Soglia per considerare l'errore asintotico nullo
tempo_simulazione = 20;  % Tempo di simulazione in evoluzione libera
timeout_sim = 10;        

% a zero gli stati non esplorati
assignin('base', 'x0_delta_t', 0);
assignin('base', 'x0_zu_dot',  0);

% Definizione griglia 2D 
delta_s_vec = linspace(-0.15, 0.15, 15); 
zs_dot_vec  = linspace(-2.0, 2.0, 15);   

% Matrici per salvataggio risultati (inizializzate a 0 = Instabile ; 1 = Stabile)
RAS_krob = zeros(length(zs_dot_vec), length(delta_s_vec));
RAS_kred = zeros(length(zs_dot_vec), length(delta_s_vec));

disp('================================================================');
disp('   INIZIO VALUTAZIONE RAS 2D (K_rob, K_red)');
disp('================================================================');

% Creazione della Waitbar con pulsante Annulla
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
        
        % Condizioni iniziali per l'iterazione corrente
        assignin('base', 'x0_delta_s', val_ds); 
        assignin('base', 'x0_zs_dot',  val_dz); 
        
        fprintf('Test punto [%d/%d]: delta_s = %5.3f m, zs_dot = %5.3f m/s ... ', ...
                iter, num_totale, val_ds, val_dz);
        
        try
            % Esecuzione modello Simulink con timeout
            out = sim(nome_modello, 'StopTime', num2str(tempo_simulazione), ...
                      'ReturnWorkspaceOutputs', 'on', ...
                      'TimeOut', timeout_sim); 
            
            % Estrazione Dati
            data_krob = out.delta_s_krob.Data;
            data_kred = out.delta_s_kred.Data;
            
            % Verifica K_rob 
            err_krob = max(abs(data_krob(end-50:end))); 
            if err_krob < tolleranza && ~any(isnan(data_krob))
                RAS_krob(j, i) = 1; % Segniamo come Stabile
            end
            
            % Verifica K_red 
            err_kred = max(abs(data_kred(end-50:end)));
            if err_kred < tolleranza && ~any(isnan(data_kred))
                RAS_kred(j, i) = 1; % Segniamo come Stabile
            end
            
            fprintf('OK\n');
        catch
           
            fprintf('Error\n');
        end
    end
    
    if abortito
        break; 
    end
end

warning('on', 'all');

% Chiusura waitbar
if ishandle(h_wait)
    delete(h_wait);
end

if ~abortito
    disp('Simulazioni completate! Generazione Grafici...');

    % Plot risultati 
    [X, Y] = meshgrid(delta_s_vec, zs_dot_vec);

    figure('Name', 'Confronto RAS - Controllori Robusti (DK-Iteration)', 'Position', [150, 150, 1000, 450]);

    % --- PLOT 1: K_rob (Completo) ---
    subplot(1,2,1);
    gscatter(X(:), Y(:), RAS_krob(:), 'rg', 'x.', 15, 15);
    title('RAS - K_{rob} (Ordine Completo)');
    xlabel('Corsa Iniziale \delta_s [m]');
    ylabel('Velocità Cassa \cdot{z}_s [m/s]');
    if all(RAS_krob(:) == 0)
        legend('Instabile', 'Location', 'best');
    elseif all(RAS_krob(:) == 1)
        legend('Stabile', 'Location', 'best');
    else
        legend('Instabile', 'Stabile', 'Location', 'best');
    end
    grid on; axis square;

    % --- PLOT 2: K_red (Ridotto) ---
    subplot(1,2,2);
    gscatter(X(:), Y(:), RAS_kred(:), 'rg', 'x.', 15, 15);
    title('RAS - K_{red} (Ordine Ridotto)');
    xlabel('Corsa Iniziale \delta_s [m]');
    ylabel('Velocità Cassa \cdot{z}_s [m/s]');
    if all(RAS_kred(:) == 0)
        legend('Instabile', 'Location', 'best');
    elseif all(RAS_kred(:) == 1)
        legend('Stabile', 'Location', 'best');
    else
        legend('Instabile', 'Stabile', 'Location', 'best');
    end
    grid on; axis square;
end