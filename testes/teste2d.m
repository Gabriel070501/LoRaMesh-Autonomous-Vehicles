% --- Parâmetros gerais ---
N = 500;                  % Número de veículos móveis
n_fixos = 5;            % Número de nós fixos (vermelhos)
A_km2 = 1344;              % Área total (km²)
T = 60;                  % Tempo total (s)
alcance_min = 400;       % Alcance mínimo LoRa (m)
alcance_max = 1000;      % Alcance máximo LoRa (m)
vel_min = 5;             % Velocidade mínima (m/s)
vel_max = 20;            % Velocidade máxima (m/s)

% --- Canais e transmissão ---
num_canais_pequenos = 64;
num_canais_grandes = 8;
total_canais = num_canais_pequenos + num_canais_grandes;

% Tempo de transmissão (em milissegundos)
t_pequeno = 26.48;        % ms
t_grande_extra = 8.4;   % ms
t_grande_normal = 33.6;  % ms
janela_ms = 1000;        % 1 segundo

% --- Área como quadrado ---
lado = sqrt(A_km2 * 1e6);  % metros

% --- Inicialização ---
pos_movel = lado * rand(N, 2);
direcao = 2*pi * rand(N,1);
velocidade = vel_min + (vel_max - vel_min)*rand(N,1);

pos_fixo = lado * rand(n_fixos, 2);
N_total = N + n_fixos;

offset_ms = randi([0, janela_ms], N_total, 1);  % Offset inicial aleatório (em ms)

% --- Estatísticas ---
colisoes_total = zeros(T,1);
pacotes_recebidos = 0;
media_vizinhos = zeros(T,1);

figure;
for t = 1:T
    % Atualizar posições móveis
    pos_movel(:,1) = pos_movel(:,1) + velocidade .* cos(direcao);
    pos_movel(:,2) = pos_movel(:,2) + velocidade .* sin(direcao);

    % Refletir nas bordas
    fora = pos_movel < 0 | pos_movel > lado;
    direcao(fora(:,1) | fora(:,2)) = 2*pi * rand(sum(fora(:,1) | fora(:,2)), 1);
    pos_movel = max(min(pos_movel, lado), 0);

    % Posição total
    pos_total = [pos_movel; pos_fixo];

    % Matriz de vizinhança
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

    % Pacotes e canais
    canais = zeros(N_total,1);
    tempos_tx = zeros(N_total,1);     % Tempo de envio (ms)
    inicios_tx = mod(offset_ms, janela_ms);  % Início da transmissão na janela

    for i = 1:N_total
        is_grande = mod(t - floor(offset_ms(i)/1000), 5) == 4;

        % Define canal
        if is_grande
            canais_possiveis = (num_canais_pequenos+1):total_canais;
            tempo_pacote = t_grande_extra;

            % Verifica se canal grande está livre entre vizinhos
            vizinhos = find(adj(i,:) == 1);
            ocupado = false;
            for c = canais_possiveis
                conflito = false;
                for v = vizinhos
                    if canais(v) == c
                        % Verifica sobreposição temporal
                        fim_i = inicios_tx(i) + tempo_pacote;
                        fim_v = inicios_tx(v) + tempos_tx(v);
                        if ~(inicios_tx(i) > fim_v || inicios_tx(v) > fim_i)
                            conflito = true;
                            break
                        end
                    end
                end
                if ~conflito
                    canais(i) = c;
                    tempos_tx(i) = tempo_pacote;
                    ocupado = true;
                    break
                end
            end

            % Se todos ocupados, vai para canal pequeno
            if ~ocupado
                canais_possiveis = 1:num_canais_pequenos;
                tempo_pacote = t_grande_normal;
                canais(i) = canais_possiveis(randi(length(canais_possiveis)));
                tempos_tx(i) = tempo_pacote;
            end
        else
            canais_possiveis = 1:num_canais_pequenos;
            tempo_pacote = t_pequeno;
            canais(i) = canais_possiveis(randi(length(canais_possiveis)));
            tempos_tx(i) = tempo_pacote;
        end
    end

    % --- Verifica colisões temporais ---
    colisoes = 0;
    recebidos_sucesso = 0;

    for i = 1:N_total
        vizinhos = find(adj(i,:) == 1);
        conflito = false;

        for v = vizinhos
            if canais(i) == canais(v)
                % Verifica sobreposição temporal
                fim_i = inicios_tx(i) + tempos_tx(i);
                fim_v = inicios_tx(v) + tempos_tx(v);
                if ~(inicios_tx(i) > fim_v || inicios_tx(v) > fim_i)
                    conflito = true;
                    break;
                end
            end
        end

        if conflito
            colisoes = colisoes + 1;
        else
            recebidos_sucesso = recebidos_sucesso + 1;
        end
    end

    colisoes_total(t) = colisoes;
    pacotes_recebidos = pacotes_recebidos + recebidos_sucesso;
    media_vizinhos(t) = mean(sum(adj, 2));

    % --- Plot ao vivo ---
    clf; hold on;
    plot(pos_movel(:,1), pos_movel(:,2), 'bo', 'MarkerFaceColor', 'b');
    plot(pos_fixo(:,1), pos_fixo(:,2), 'rs', 'MarkerFaceColor', 'r');

    [i_idx, j_idx] = find(triu(adj));
    X = [pos_total(i_idx,1), pos_total(j_idx,1), nan(length(i_idx),1)]';
    Y = [pos_total(i_idx,2), pos_total(j_idx,2), nan(length(i_idx),1)]';
   	plot(X(:), Y(:), 'g-', 'LineWidth', 0.5, 'LineStyle', ':');

    title(sprintf('t = %ds | Colisões: %d | Média de nós no alcance de retransmissão: %.2f', ...
        t, colisoes, media_vizinhos(t)));
    xlabel('Coordenada X (metros)');
    ylabel('Coordenada Y (metros)');
    xlim([0 lado]); ylim([0 lado]);
    drawnow;
end

% --- Estatísticas finais ---
total_pacotes = N_total * T;
eficiencia = pacotes_recebidos / total_pacotes;

fprintf('\nResumo da simulação:\n');
fprintf('Pacotes enviados: %d\n', total_pacotes);
fprintf('Pacotes recebidos sem colisão: %d\n', pacotes_recebidos);
fprintf('Colisões totais: %d\n', sum(colisoes_total));
fprintf('Eficiência da rede: %.2f%%\n', 100 * eficiencia);
fprintf('Média de vizinhos por nó: %.2f\n', mean(media_vizinhos));
fprintf('Nós fixos (vermelhos): %d\n', n_fixos);
