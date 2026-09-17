%% Stima Empirica Globale della RAS per i 4 stati del Quarter-Car
clear x0_delta_s x0_zs_dot x0_delta_t x0_zu_dot;

% --- IMPOSTAZIONI GENERALI ---
nome_modello = 'CR_quarter_car_NL'; 
tolleranza = 1e-3;       % Tolleranza errore a regime (1 mm)
tempo_simulazione = 30;  % Tempo per smaltire il windup
risultati_RAS = zeros(1,4); % Vettore per salvare i risultati

% Inizializziamo tutte le variabili a 0 nel workspace
assignin('base', 'x0_delta_s', 0);
assignin('base', 'x0_zs_dot',  0);
assignin('base', 'x0_delta_t', 0);
assignin('base', 'x0_zu_dot',  0);

% Definiamo i parametri dei test per ogni variabile: {NomeVariabile, NomeStampato, Step, MaxTest, UnitàDiMisura}
test_cases = {
    'x0_delta_s', 'Corsa Sospensione (delta_s)', 0.01, 0.20, 'm';
    'x0_zs_dot',  'Velocità Cassa (zs_dot)',     0.2,  10.0, 'm/s';
    'x0_delta_t', 'Deform. Pneumatico (delta_t)',0.005,0.10, 'm';
    'x0_zu_dot',  'Velocità Ruota (zu_dot)',     0.5,  20.0, 'm/s'
};

disp('================================================================');
disp('   INIZIO VALUTAZIONE RAS - CONTROLLORE LQG INTEGRALE');
disp('================================================================');

for i = 1:4
    var_name = test_cases{i,1};
    disp_name = test_cases{i,2};
    step_inc = test_cases{i,3};
    max_val = test_cases{i,4};
    um = test_cases{i,5};
    
    limite_attuale = 0;
    disp(['-> Test in corso per: ', disp_name, ' ...']);
    
    for val_test = 0 : step_inc : max_val
        
        assignin('base', var_name, val_test); 
        
        try
            out = sim(nome_modello, 'StopTime', num2str(tempo_simulazione));
            delta_s_data = out.delta_s_Int.Data; 
            errore_finale = mean(abs(delta_s_data(end-100:end)));
            
            if errore_finale < tolleranza
                limite_attuale = val_test; 
            else
                break; % Limite trovato, esci dal sotto-ciclo
            end
        catch
            break; % Crash numerico, limite superato
        end
    end
    
    % Salva il risultato e ripristina la variabile a 0 per il test successivo
    risultati_RAS(i) = limite_attuale;
    assignin('base', var_name, 0); 
    
    disp(['   [COMPLETATO] Limite trovato: +/- ', num2str(limite_attuale), ' ', um]);
end

disp(' ');
disp('================================================================');
disp('   RISULTATI FINALI PER TABELLA LATEX');
disp('================================================================');
for i = 1:4
    disp([test_cases{i,2}, ' : da -', num2str(risultati_RAS(i)), ' a +', num2str(risultati_RAS(i)), ' ', test_cases{i,5}]);
end