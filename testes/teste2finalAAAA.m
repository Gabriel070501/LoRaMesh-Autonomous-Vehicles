clear; clc;

% ---------- Parâmetros Fixos ----------
packet_small_bytes = 19;
packet_large_bytes = 23;

bps_normal = 5470;   % 125kHz SF7
bps_extra  = 21875;  % 500kHz SF7

channels_normal = 64;
channels_extra  = 8;

interval = 5; % segundos da simulação
Z = 15;       % km²
A_min = 400;
A_max = 1000;

max_retx = 3;

vehicle_counts = 25:25:1000; % use até 1000 para teste rápido
collision_rates = zeros(size(vehicle_counts));

% Bits por pacote
bits_small = packet_small_bytes * 8;
bits_large = packet_large_bytes * 8;

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    f_retransmit = calcula_f(N, Z, A_min, A_max);
    r = f_retransmit * (N - 1) / N;

    % Fator de retransmissão com limite
    powers = 0:max_retx;
    retrans_factor = sum(r .^ powers);
    avg_retx = retrans_factor - 1;

    % Inicializar lista de pacotes (vetor struct vazio)
    packets = struct('start_time', {}, 'end_time', {}, 'channel', {});

    for i = 1:N
        % Gerar pacotes originais pequenos
        for p = 1:4
            pkt = gerar_pacote(bits_small, bps_normal, channels_normal, interval);
            new_pkts = replicar_pacote(pkt, avg_retx, bps_normal, channels_normal, interval);
            packets = [packets; new_pkts(:)];
        end
        % Gerar pacotes originais grandes
        for p = 1:1
            pkt = gerar_pacote(bits_large, bps_extra, channels_extra, interval, true);
            new_pkts = replicar_pacote(pkt, avg_retx, bps_extra, channels_extra, interval);
            packets = [packets; new_pkts(:)];
        end
    end

    % Verificar colisões por canal
    colisoes = 0;
    total_pacotes = length(packets);

    for ch = 1:(channels_normal + channels_extra)
        % Filtrar pacotes no canal atual
        pacotes_canal = packets([packets.channel] == ch);
        % Ordenar por tempo de início
        [~, ord] = sort([pacotes_canal.start_time]);
        pacotes_canal = pacotes_canal(ord);

        % Verificar sobreposição
        for j = 2:length(pacotes_canal)
            anterior = pacotes_canal(j - 1);
            atual    = pacotes_canal(j);
            if atual.start_time < anterior.end_time
                colisoes = colisoes + 1;
                break; % Conta apenas uma colisão por conflito
            end
        end
    end

    collision_rates(idx) = colisoes / total_pacotes;
    fprintf('N = %4d | f = %.4f | Avg Retx = %.2f | Pacotes = %5d | Colisões = %4d | Taxa = %.4f\n', ...
        N, f_retransmit, avg_retx, total_pacotes, colisoes, collision_rates(idx));
end

% ---------- Plot ----------
figure;
plot(vehicle_counts, collision_rates, 'LineWidth', 2);
grid on;
xlabel('Número de nós na rede');
ylabel('Taxa de colisão temporal (fração de pacotes colididos)');
title(sprintf('Colisão temporal com retransmissão em cascata limitada (max %d retx)', max_retx));
ylim([0 1]);

% ---------- Funções Auxiliares ----------
function pkt = gerar_pacote(bits, bps, num_canais, interval, extra)
    if nargin < 5
        extra = false;
    end
    channel_base = 0;
    if extra
        channel_base = 64;
    end
    ch = channel_base + randi(num_canais);
    t_start = rand() * interval;
    toa = bits / bps;
    pkt = struct(...
        'start_time', t_start, ...
        'end_time', t_start + toa, ...
        'channel', ch ...
    );
end

function lista = replicar_pacote(pkt, avg_retx, bps, num_canais, interval)
    num_retx = poissrnd(avg_retx);
    lista = pkt; % inclui o original
    for k = 1:num_retx
        retx_pkt = gerar_pacote((pkt.end_time - pkt.start_time) * bps, bps, num_canais, interval);
        lista(end+1) = retx_pkt;
    end
    lista = lista(:); % força vetor coluna para concatenação
end

function f = calcula_f(N, Z, A_min, A_max)
    area_km2 = Z;
    area_m2 = area_km2 * 1e6;
    densidade = N / area_m2;
    area_coroa = pi * (A_max^2 - A_min^2);
    f = densidade * area_coroa;
end
