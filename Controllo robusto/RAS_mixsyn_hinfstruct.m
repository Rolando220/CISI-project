%% Stima Empirica Globale della RAS per MIXSYN e HINFSTRUCT (PI)
clear x0_delta_s x0_zs_dot x0_delta_t x0_zu_dot;

% --- IMPOSTAZIONI GENERALI ---
nome_modello = 'quarter_car_MixedSensitivity'; 
tolleranza = 1e-3;       % Tolleranza errore a regime (1 mm)
tempo_simulazione = 30;  % Tempo per smaltire il windup

risultati_mixsyn = zeros(1,4);
risultati_pid    = zeros(1,4);

% Inizializziamo tutte le variabili a 0 nel workspace
assignin('base', 'x0_delta_s', 0);
assignin('base', 'x0_zs_dot',  0);
assignin('base', 'x0_delta_t', 0);
assignin('base', 'x0_zu_dot',  0);

test_cases = {
    'x0_delta_s', 'Corsa Sospensione (delta_s)', 0.01, 0.20, 'm';
    'x0_zs_dot',  'Velocità Cassa (zs_dot)',     0.2,  10.0, 'm/s';
    'x0_delta_t', 'Deform. Pneumatico (delta_t)',0.005,0.10, 'm';
    'x0_zu_dot',  'Velocità Ruota (zu_dot)',     0.5,  20.0, 'm/s'
};


disp('================================================================');
disp('   INIZIO VALUTAZIONE RAS - MIXSYN E PI STRUTTURATO');
disp('================================================================');

for i = 1:4
    var_name = test_cases{i,1};
    disp_name = test_cases{i,2};
    step_inc = test_cases{i,3};
    max_val = test_cases{i,4};
    um = test_cases{i,5};
    
    limite_mixsyn = 0;
    limite_pid = 0;
    fail_mixsyn = false;
    fail_pid = false;
    
    disp(['-> Test in corso per: ', disp_name, ' ...']);
    
    for val_test = 0 : step_inc : max_val
        assignin('base', var_name, val_test); 
        
        try
            % Simuliamo tutti i controllori in un colpo solo
            out = sim(nome_modello, 'StopTime', num2str(tempo_simulazione));
            
            % 1. Verifica stabilità MIXSYN
            if ~fail_mixsyn
                err_mix = mean(abs(out.delta_s_mixsyn.Data(end-100:end)));
                if err_mix < tolleranza
                    limite_mixsyn = val_test; 
                else
                    fail_mixsyn = true; 
                end
            end
            
            % 2. Verifica stabilità HINFSTRUCT (PI)
            if ~fail_pid
                err_pid = mean(abs(out.delta_s_PID.Data(end-100:end)));
                if err_pid < tolleranza
                    limite_pid = val_test; 
                else
                    fail_pid = true; 
                end
            end
            
            % Se entrambi i controllori sono diventati instabili, fermiamo la ricerca
            if fail_mixsyn && fail_pid
                break; 
            end
            
        catch
            % Se Simulink va in crash matematico, significa che l'instabilità 
            % di un loop ha bloccato il solutore per tutto il modello.
            break; 
        end
    end
    
    risultati_mixsyn(i) = limite_mixsyn;
    risultati_pid(i) = limite_pid;
    assignin('base', var_name, 0); 
    
    fprintf('   [COMPLETATO] Limite MIXSYN: +/- %.3f %s | Limite PI: +/- %.3f %s\n', ...
        limite_mixsyn, um, limite_pid, um);
end

disp(' ');
disp('================================================================');
disp('   RISULTATI FINALI PER TABELLA LATEX');
disp('================================================================');
for i = 1:4
    fprintf('%-30s | MIXSYN: +/- %.3f %s | PI: +/- %.3f %s\n', ...
        test_cases{i,2}, risultati_mixsyn(i), test_cases{i,5}, risultati_pid(i), test_cases{i,5});
end