%% Stima Empirica 2D della RAS per LQG (Integrale)
%------------------------------------------------------------------------
%   Si verifichi nel rispettivo file simulink che i disturbi esogeni 
%   siano posti a zero 
%   Si imposti nel rispettivo file simulink l'opzione "Fast Restart"
%------------------------------------------------------------------------
clear x0_delta_s x0_zs_dot x0_delta_t x0_zu_dot;
clc;

nome_modello = 'H2_LQG_quarter_Car'; 

tolleranza = 1e-3;       % Soglia di tolleranza a regime (1 mm)
tempo_simulazione = 10;  % Tempo di simulazione in evoluzione libera

% a zero gli stati non esplorati 
assignin('base', 'x0_delta_t', 0);
assignin('base', 'x0_zu_dot',  0);

% Definizione griglia 2D 
delta_s_vec = linspace(-0.15, 0.15, 15); 
zs_dot_vec  = linspace(-2.0, 2.0, 15);   

% Matrice per salvare i risultati (1 = Stabile, 0 = Instabile)
RAS_int = zeros(length(zs_dot_vec), length(delta_s_vec));

disp('================================================================');
disp('   INIZIO VALUTAZIONE RAS 2D (LQG con Integratore)');
disp('   ATTENZIONE: Assicurati che w e F_load siano a zero in Simulink!');
disp('================================================================');

% Creazione della Waitbar 
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
           
            out = sim(nome_modello, 'StopTime', num2str(tempo_simulazione), 'ReturnWorkspaceOutputs', 'on');
            
            % Estrazione Dati 
            data_int = out.delta_s_Int.Data;
            
            % Verifica LQG Con Integratore
            err_int = max(abs(data_int(end-50:end))); 
            if err_int < tolleranza && ~any(isnan(data_int))
                RAS_int(j, i) = 1; 
            end
            
            fprintf('OK\n');
        catch
            fprintf('INSTABILE/CRASH\n');
        end
    end
    
    if abortito
        break; 
    end
end

% Chiude la waitbar alla fine o all'annullamento
if ishandle(h_wait)
    delete(h_wait);
end

if ~abortito
    disp('Simulazioni completate! Generazione Grafici...');

    % Plot dei risultati
    [X, Y] = meshgrid(delta_s_vec, zs_dot_vec);

    figure('Name', 'Valutazione RAS - LQG Integrale', 'Position', [300, 200, 600, 500]);
    gscatter(X(:), Y(:), RAS_int(:), 'rg', 'x.', 15, 15);
    title('RAS - LQG (Con Azione Integrale)');
    xlabel('Corsa Iniziale \delta_s [m]');
    ylabel('Velocità Cassa \cdot{z}_s [m/s]');
    legend('Instabile', 'Stabile', 'Location', 'best');
    grid on; axis square;
end