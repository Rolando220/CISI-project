%% Stima Empirica Globale della RAS per MU-SYNTHESIS
clear x0_delta_s x0_zs_dot x0_delta_t x0_zu_dot;

% --- IMPOSTAZIONI GENERALI ---
nome_modello = 'quarter_car_musyn';
tolleranza = 1e-3;       % Tolleranza errore a regime (1 mm)
tempo_simulazione = 30;  % Tempo per smaltire il windup
risultati_mu = zeros(1,4);

% Inizializziamo tutte le variabili a 0 nel workspace
assignin('base', 'x0_delta_s', 0);
assignin('base', 'x0_zs_dot',  0);
assignin('base', 'x0_delta_t', 0);
assignin('base', 'x0_zu_dot',  0);

test_cases = {
    'x0_delta_s', 'Corsa Sospensione', 0.01, 0.20, 'm';
    'x0_zs_dot',  'Velocita Cassa',    0.2,  10.0, 'm/s';
    'x0_delta_t', 'Deform. Pneumatico',0.005,0.10, 'm';
    'x0_zu_dot',  'Velocita Ruota',    0.5,  20.0, 'm/s'
};

disp('================================================================');
disp('   INIZIO VALUTAZIONE RAS - MU-SYNTHESIS');
disp('================================================================');

for i = 1:4
    var_name = test_cases{i,1};
    disp_name = test_cases{i,2};
    step_inc = test_cases{i,3};
    max_val = test_cases{i,4};
    um = test_cases{i,5};
    
    limite_mu = 0;
    
    disp(['-> Test in corso per: ', disp_name, ' ...']);
    
    for val_test = 0 : step_inc : max_val
        assignin('base', var_name, val_test); 
        
        try
            % Lancia Simulink
            out = sim(nome_modello, 'StopTime', num2str(tempo_simulazione));
            
            % Controlla se negli ultimi 100 istanti l'auto è effettivamente ferma
            err_mu = mean(abs(out.delta_s_musyn.Data(end-100:end)));
            
            if err_mu < tolleranza
                limite_mu = val_test; % Se è ferma, salva questo valore come sicuro
            else
                break; % Se non si ferma, esce dal loop e tiene buono il valore precedente
            end
            
        catch
            % Se Simulink esplode numericamente a causa della non linearità
            break; 
        end
    end
    
    risultati_mu(i) = limite_mu;
    assignin('base', var_name, 0); % Resetta la variabile a 0 per il test successivo
    
    fprintf('   [COMPLETATO] Limite trovato: +/- %.3f %s\n', limite_mu, um);
end

disp(' ');
disp('================================================================');
disp('   RISULTATI FINALI PER TABELLA LATEX');
disp('================================================================');
for i = 1:4
    fprintf('%-30s | MU: +/- %.3f %s\n', test_cases{i,2}, risultati_mu(i), test_cases{i,5});
end