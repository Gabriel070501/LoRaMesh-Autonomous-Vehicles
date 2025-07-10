% --- Parâmetros gerais ---
N = 100;                  % Número de veículos móveis
n_fixos = 10;             % Número de nós fixos (vermelhos)
A_km2 = 13.44;            % Área total (km²)
T = 10;                   % Tempo total (s)
alcance_min = 400;        % Alcance mínimo LoRa (m)
alcance_max = 1000;       % Alcance máximo LoRa (m)
vel_min = 5;              % Velocidade mínima (m/s)
vel_max = 20;             % Velocidade máxima (m/s)

% --- Canais e transmissão ---
num_canais_pequenos = 64;
num_canais_grandes = 8;
total_canais = num_canais_pequenos + num_canais_grandes;

% Tempo de transmissão (ms)
t_pequeno = 26.48;        % ms para pacotes pequenos
t_grande_extra = 8.4;     % ms para pacotes grandes em canal extra
t_grande_normal = 33.6;   % ms para pacotes grandes em canal normal
janela_ms = 1000;         % janela de 1 segundo para envio

% --- Área ---
lado = sqrt(A_km2 * 1e6); % lado do quadrado em metros

% --- Inicialização posições e velocidades ---
pos_movel = lado * rand(N, 2);
direcao = 2*pi * rand(N,1);
velocidade = vel_min + (vel_max - vel_min) .* rand(N,1);

pos_fixo = lado * rand(n_fixos, 2);
N_total = N + n_fixos;

% --- Inicialização dos pacotes ativos ---
% Estrutura de pacotes:
% origem = nó que gerou o pacote original
% transmissor = nó que está transmitindo (original ou retransmissão)
% salto = número de retransmissões (0 = original)
% inicio_tx = início da transmissão em ms dentro da janela
% duracao_tx = duração da transmissão em ms
% canal = canal usado

pacotes_ativos = struct('origem', {}, 'transmissor', {}, 'salto', {}, ...
    'inicio_tx', {}, 'duracao_tx', {}, 'canal', {});

% --- Estatísticas ---
colisoes_total = zeros(T,1);
pacotes_recebidos = 0;
media_vizinhos = zeros(T,1);

% --- Offset inicial aleatório para sincronizar transmissões ---
offset_ms = randi([0, janela_ms], N_total, 1);

figure;

for t = 1:T
    % --- Atualiza posições móveis ---
    pos_movel(:,1) = pos_movel(:,1) + velocidade .* cos(direcao);
    pos_movel(:,2) = pos_movel(:,2) + velocidade .* sin(direcao);
    
    % Refletir nas bordas
    fora = pos_movel < 0 | pos_movel > lado;
    if any(fora(:,1) | fora(:,2))
        idx_reflete = find(fora(:,1) | fora(:,2));
        direcao(idx_reflete) = mod(direcao(idx_reflete) + pi, 2*pi);
        % Ajustar posição para dentro da área
        pos_movel = max(min(pos_movel, lado), 0);
    end

    % Junta posições móveis e fixas
    pos_total = [pos_movel; pos_fixo];

    % --- Matriz de vizinhança por distância para retransmissão ---
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

    media_vizinhos(t) = mean(sum(adj, 2));

    % --- Gerar pacotes originais para cada nó (salto = 0) ---
    novos_pacotes = struct('origem', {}, 'transmissor', {}, 'salto', {}, ...
        'inicio_tx', {}, 'duracao_tx', {}, 'canal', {});
    
    for i = 1:N_total
        % Determinar se pacote é grande ou pequeno neste segundo
        % Pacote grande a cada 5s, pequeno todo segundo
        is_grande = mod(t - floor(offset_ms(i)/1000), 5) == 4;

        if is_grande
            % Pacote grande prioriza canal extra (64+1 a 72)
            canais_possiveis = (num_canais_pequenos+1):total_canais;
            duracao_tx = t_grande_extra;
        else
            % Pacote pequeno canal normal (1 a 64)
            canais_possiveis = 1:num_canais_pequenos;
            duracao_tx = t_pequeno;
        end

        canal = canais_possiveis(randi(length(canais_possiveis)));
        inicio_tx = randi([0, janela_ms - ceil(duracao_tx)]);

        pacote = struct('origem', i, 'transmissor', i, 'salto', 0, ...
            'inicio_tx', inicio_tx, 'duracao_tx', duracao_tx, 'canal', canal);
        novos_pacotes(end+1) = pacote; %#ok<SAGROW>
    end

    % --- Atualizar lista de pacotes ativos: remove pacotes já transmitidos ---
    pacotes_ativos_novos = [];
    for k = 1:length(pacotes_ativos)
        fim_tx = pacotes_ativos(k).inicio_tx + pacotes_ativos(k).duracao_tx;
        % Mantém só pacotes que ainda estão "ativos" nesta janela
        pacotes_ativos_novos = struct('origem', {}, 'transmissor', {}, 'salto', {}, 'inicio_tx', {}, 'duracao_tx', {}, 'canal', {});

        for k = 1:length(pacotes_ativos)
            fim_tx = pacotes_ativos(k).inicio_tx + pacotes_ativos(k).duracao_tx;
            if fim_tx > 0
                pacotes_ativos_novos(end+1) = pacotes_ativos(k); %#ok<SAGROW>
            end
        end
    end
    pacotes_ativos = pacotes_ativos_novos;

    % Adiciona os novos pacotes originais
    pacotes_ativos = [pacotes_ativos, novos_pacotes];

    % --- Controle de retransmissão ---
    retransmitidos_nos_hoje = false(N_total,1);

    % Novo array para pacotes retransmitidos nesta iteração
    pacotes_retrans = struct('origem', {}, 'transmissor', {}, 'salto', {}, ...
        'inicio_tx', {}, 'duracao_tx', {}, 'canal', {});

    % Para evitar retransmissão múltipla do mesmo pacote pelo mesmo nó,
    % usamos um mapa (origem + transmissor) já feitos nesta iteração
    retransmitidos_map = containers.Map('KeyType','char','ValueType','logical');

    for idx = 1:length(pacotes_ativos)
        p = pacotes_ativos(idx);

        % Retransmite só se salto < 2
        if p.salto < 2
            % Vizinhos para retransmitir
            vizinhos = find(adj(p.transmissor,:) == 1);
            vizinhos = setdiff(vizinhos, p.transmissor); % exclui o próprio nó

            for v = vizinhos
                % Checar se já retransmitiu este pacote neste nó
                chave = sprintf('%d_%d', p.origem, v);
                if isKey(retransmitidos_map, chave)
                    continue
                end

                % Limitar retransmissão a 1 pacote por nó por iteração
                if retransmitidos_nos_hoje(v)
                    continue
                end

                % Define canal e duração igual ao pacote original
                if p.duracao_tx == t_grande_extra || p.duracao_tx == t_grande_normal
                    canais_possiveis = (num_canais_pequenos+1):total_canais;
                    duracao_tx = t_grande_extra;
                else
                    canais_possiveis = 1:num_canais_pequenos;
                    duracao_tx = t_pequeno;
                end

                canal = canais_possiveis(randi(length(canais_possiveis)));

                % Início da transmissão com offset aleatório na janela
                inicio_tx = randi([0, janela_ms - ceil(duracao_tx)]);

                % Cria pacote retransmitido
                novo_pacote = struct('origem', p.origem, 'transmissor', v, ...
                    'salto', p.salto + 1, 'inicio_tx', inicio_tx, ...
                    'duracao_tx', duracao_tx, 'canal', canal);

                pacotes_retrans(end+1) = novo_pacote; %#ok<SAGROW>

                retransmitidos_nos_hoje(v) = true;
                retransmitidos_map(chave) = true;
            end
        end
    end

    pacotes_ativos = [pacotes_ativos, pacotes_retrans];

    % --- Verificação de colisões ---
    colisoes = 0;

    % Para facilitar, vamos comparar pares de pacotes que são
    % próximos e no mesmo canal, e verificar sobreposição no tempo

    for i_p = 1:length(pacotes_ativos)
        p1 = pacotes_ativos(i_p);
        for j_p = i_p+1:length(pacotes_ativos)
            p2 = pacotes_ativos(j_p);

            % Só pode haver colisão se canais iguais
            if p1.canal ~= p2.canal
                continue
            end

            % Checar proximidade dos transmissores
            d = norm(pos_total(p1.transmissor,:) - pos_total(p2.transmissor,:));
            if d < alcance_max && d > alcance_min
                % Checar sobreposição temporal (TOA)
                fim_p1 = p1.inicio_tx + p1.duracao_tx;
                fim_p2 = p2.inicio_tx + p2.duracao_tx;

                if ~(p1.inicio_tx > fim_p2 || p2.inicio_tx > fim_p1)
                    % Colisão detectada: marcar ambos pacotes como colididos
                    % Para simplificar, vamos só contar colisões únicas para cada pacote
                    colisoes = colisoes + 1;
                end
            end
        end
    end

    % Número de pacotes sem colisão é total menos colisões
    % Simplificação: algumas colisões podem contar mais de uma vez, mas para estatística é OK
    recebidos_sucesso = length(pacotes_ativos)*2 - colisoes; % *2 pois cada colisão envolve 2 pacotes

    colisoes_total(t) = colisoes;
    pacotes_recebidos = pacotes_recebidos + max(recebidos_sucesso,0);

    % --- Plot ---
    clf; hold on;
    plot(pos_movel(:,1), pos_movel(:,2), 'bo', 'MarkerFaceColor', 'b'); % veículos azuis círculos
    plot(pos_fixo(:,1), pos_fixo(:,2), 'rs', 'MarkerFaceColor', 'r');   % nós fixos vermelhos quadrados

    % Plota conexões (arestas da matriz adj)
    [i_idx, j_idx] = find(triu(adj));
    X = [pos_total(i_idx,1), pos_total(j_idx,1), nan(length(i_idx),1)]';
    Y = [pos_total(i_idx,2), pos_total(j_idx,2), nan(length(i_idx),1)]';
    %plot(X(:), Y(:), 'g-', 'LineWidth', 0.5, 'LineStyle', ':');

    title(sprintf('t = %ds | Colisões: %d | Média vizinhos: %.2f', ...
        t, colisoes, media_vizinhos(t)));
    xlabel('Coordenada X (m)');
    ylabel('Coordenada Y (m)');
    xlim([0 lado]); ylim([0 lado]);
    drawnow;
end

% --- Estatísticas finais ---
total_pacotes = length(pacotes_ativos) * T;
eficiencia = pacotes_recebidos / total_pacotes;

fprintf('\nResumo da simulação:\n');
fprintf('Pacotes enviados (orig + retrans): %d\n', total_pacotes);
fprintf('Pacotes recebidos sem colisão (estimado): %d\n', pacotes_recebidos);
fprintf('Colisões totais: %d\n', sum(colisoes_total));
fprintf('Eficiência da rede: %.2f%%\n', 100 * eficiencia);
fprintf('Média de vizinhos por nó: %.2f\n', mean(media_vizinhos));
fprintf('Nós fixos (vermelhos): %d\n', n_fixos);
