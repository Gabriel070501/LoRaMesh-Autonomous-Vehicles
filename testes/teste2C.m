% Parâmetros
N = 1000;
n_fixos = 10;
A_km2 = 1;
T = 30;                          % tempo total (s)
alcance_min = 50;
alcance_max = 250;
vel_min = 5;
vel_max = 20;
duracao_tx = 50;                % duração da transmissão em ms
slot_max = 1000;                % janela total de transmissão (1s = 1000ms)

% Área
lado = sqrt(A_km2 * 1e6);

% Posições móveis
pos_movel = lado * rand(N, 2);
direcao = 2*pi * rand(N,1);
velocidade = vel_min + (vel_max - vel_min)*rand(N,1);

% Posições fixas
pos_fixo = lado * rand(n_fixos, 2);

% Total
pos_total = [pos_movel; pos_fixo];
N_total = N + n_fixos;

% Cada nó escolhe seu instante de transmissão fixo dentro de [0, 1000 - duracao]
instantes_tx = randi([0, slot_max - duracao_tx], N_total, 1);

% Estatísticas
colisoes_total = zeros(T,1);
pacotes_recebidos = 0;
media_vizinhos = zeros(T,1);

figure;
for t = 1:T
    % Atualiza apenas móveis
    pos_movel(:,1) = pos_movel(:,1) + velocidade .* cos(direcao);
    pos_movel(:,2) = pos_movel(:,2) + velocidade .* sin(direcao);
    fora = pos_movel < 0 | pos_movel > lado;
    direcao(fora(:,1) | fora(:,2)) = 2*pi * rand(sum(fora(:,1) | fora(:,2)), 1);
    pos_movel = max(min(pos_movel, lado), 0);
    pos_total = [pos_movel; pos_fixo];

    % Matriz de adjacência
    adj = zeros(N_total);
    for i = 1:N_total
        for j = i+1:N_total
            d = norm(pos_total(i,:) - pos_total(j,:));
            if d >= alcance_min && d <= alcance_max
                adj(i,j) = 1;
                adj(j,i) = 1;
            end
        end
    end

    % Para cada nó, veja se ele recebe pacotes de forma válida
    recebidos_sucesso = 0;
    colisoes = 0;

    for i = 1:N_total
        vizinhos = find(adj(i,:) == 1);
        if isempty(vizinhos)
            continue;
        end

        % Verifica sobreposição dos tempos de transmissão dos vizinhos
        tx_inicios = instantes_tx(vizinhos);
        tx_fins = tx_inicios + duracao_tx;

        % Ordenar por início
        [tx_inicios_sorted, idx] = sort(tx_inicios);
        tx_fins_sorted = tx_fins(idx);

        % Contar sobreposições
        ok = false;
        for k = 1:length(tx_inicios_sorted)
            inicio_k = tx_inicios_sorted(k);
            fim_k = tx_fins_sorted(k);

            % Checa se esse slot sobrepõe com algum anterior
            sobrepoe = false;
            for j = k+1:length(tx_inicios_sorted)
                if tx_inicios_sorted(j) < fim_k
                    sobrepoe = true;
                    break;
                end
            end

            if ~sobrepoe
                ok = true;
                break;  % recebeu de um sem colisão
            end
        end

        if ok
            recebidos_sucesso = recebidos_sucesso + 1;
        else
            colisoes = colisoes + 1;
        end
    end

    colisoes_total(t) = colisoes;
    pacotes_recebidos = pacotes_recebidos + recebidos_sucesso;
    media_vizinhos(t) = mean(sum(adj, 2));

    % Plot ao vivo
    clf; hold on;
    plot(pos_movel(:,1), pos_movel(:,2), 'bo', 'MarkerFaceColor', 'b');
    plot(pos_fixo(:,1), pos_fixo(:,2), 'ro', 'MarkerFaceColor', 'r');
    [i_idx, j_idx] = find(triu(adj));
    for k = 1:length(i_idx)
        plot([pos_total(i_idx(k),1), pos_total(j_idx(k),1)], ...
             [pos_total(i_idx(k),2), pos_total(j_idx(k),2)], 'g-');
    end
    title(sprintf('t = %ds | Colisões: %d | Média vizinhos: %.2f', ...
        t, colisoes, media_vizinhos(t)));
    xlim([0 lado]); ylim([0 lado]);
    drawnow;
end

% Estatísticas finais
total_pacotes = N_total * T;
eficiencia = pacotes_recebidos / total_pacotes;

fprintf('\nResumo da simulação:\n');
fprintf('Pacotes enviados: %d\n', total_pacotes);
fprintf('Pacotes recebidos sem colisão: %d\n', pacotes_recebidos);
fprintf('Colisões totais: %d\n', sum(colisoes_total));
fprintf('Eficiência da rede: %.2f%%\n', 100 * eficiencia);
fprintf('Média de vizinhos por nó: %.2f\n', mean(media_vizinhos));
fprintf('Nós fixos (vermelhos): %d\n', n_fixos);
