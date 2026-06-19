x_hat_pred = squeeze(out.RTS_data.Data(:,1,:));
P_pred = squeeze(out.RTS_data.Data(:,2:5,:));
Fk = squeeze(out.RTS_data.Data(:,6:9,:));
x_hat_corr = squeeze(out.RTS_data.Data(:,10,:));
P_corr = squeeze(out.RTS_data.Data(:,11:14,:));


N = size(x_hat_pred,2);

x_reg = zeros(4, N);
P_reg = zeros(4, 4, N);

x_reg(:, N)   = x_hat_corr(:, N);
P_reg(:,:, N) = P_corr(:,:, N);

for k=(N-1):-1:1

    Ck = P_corr(:,:,k) * Fk(:,:,k)' / P_pred(:,:,k+1);

    x_reg(:,k) = x_hat_corr(:,k) + Ck * (x_reg(:, k+1) - x_hat_pred(:, k+1));

    P_reg(:,:,k) = P_corr(:,:,k) + Ck * (P_reg(:,:,k+1) - P_pred(:,:,k+1)) * Ck';

end


x_true = squeeze(out.x_true.Data(:,1,:));

% Assumo che il vettore tempo sia salvato in out.tout. 
% (Se usi un altro formato, cambialo in out.RTS_data.Time)
t = out.x_true.Time; 

% Assumo che il vettore tempo t sia già caricato (es. t = out.RTS_data.Time;)

% Nomi fisici degli stati per avere dei titoli chiari sui grafici
nomi_stati = {'Deflessione Sospensione (\delta_s) [m]', ...
              'Vel. Massa Sospesa (\dot{z}_s) [m/s]', ...
              'Deflessione Pneumatico (\delta_t) [m]', ...
              'Vel. Massa Non Sospesa (\dot{z}_u) [m/s]'};

% Ciclo per generare 4 figure distinte
for i = 1:4
    % Apre una NUOVA finestra per ogni stato
    figure('Name', nomi_stati{i}, 'Color', 'w');
    hold on; grid on;
    
    % PLOT 1: Stato Vero -> Linea nera spessa
    plot(t, x_true(i, :), 'k', 'LineWidth', 1.5, 'DisplayName', 'Stato Vero (Plant)');
    
    % PLOT 2: Stato Stimato EKF -> Linea blu tratteggiata
    plot(t, x_hat_corr(i, :), 'b--', 'LineWidth', 1.2, 'DisplayName', 'Stima EKF (Forward)');
    
    % PLOT 3: Stato Regolarizzato RTS -> Linea rossa continua
    plot(t, x_reg(i, :), 'r', 'LineWidth', 1.5, 'DisplayName', 'Regolarizzato RTS (Backward)');
    
    % Estetica del grafico singolo
    title(nomi_stati{i}, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Tempo [s]');
    ylabel('Ampiezza');
    legend('Location', 'best');
    xlim([t(1) t(end)]);
end