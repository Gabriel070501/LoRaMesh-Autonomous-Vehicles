clear; clc;

% --------- Parâmetros fixos ---------
packet_small_bytes = 19;
packet_large_bytes = 23;

bps_normal = 5470;   % 125kHz SF7
bps_extra  = 21875;  % 500kHz SF7

channels_normal = 64;
channels_extra = 8;

interval = 5; % janela simulada para pacotes grandes (em segundos)

Z = 15; % área da cidade em km^2
A_min = 400;  % distância mínima para retransmissão (m)
A_max = 1000; % distância máxima para retransmissão (m)
T_sim = 60; % tempo total da simulação em segundos (exemplo)

vehicle_counts = 1000:1000:3000; % Exemplo: de 100 a 1000 veículos em passos de 100
collision_rates = zeros(size(vehicle_counts));

bits_small = packet_small_bytes * 8;
bits_large = packet_large_bytes * 8;

toa_small = bits_small / bps_normal;
toa_large = bits_large / bps_extra;

channels_small = 1:channels_normal;
channels_large = 1:channels_extra;

Z_m2 = Z * 1e6;
L = sqrt(Z_m2);

for idx = 1:length(vehicle_counts)
    
    N = vehicle_counts(idx);
    grid_size = ceil(sqrt(N));
    x_vals = linspace(0, L, grid_size);
    y_vals = linspace(0, L, grid_size);
    [X, Y] = meshgrid(x_vals, y_vals);
    pos = [X(:), Y(:)];
    pos = pos(1:N, :); % pega apenas N pontos
    dist_mat = squareform(pdist(pos));
    
    pkts_all = [];
    
    for i = 1:N
        offset_small = rand;
        offset_large = rand;
        
        for t = offset_small:1:(T_sim - 1)
            if mod(t,5) ~= 0
                ch = channels_small(randi(channels_normal));
                pkts_all = [pkts_all; i, t, toa_small, ch, 0];
            end
        end
        
        for t = offset_large:5:(T_sim - 1)
            ch = channels_large(randi(channels_extra));
            pkts_all = [pkts_all; i, t, toa_large, ch, 0];
        end
    end
    
    % Retransmissão
    pkts_retx = [];
    for p = 1:size(pkts_all,1)
        tx_node = pkts_all(p,1);
        tx_time = pkts_all(p,2);
        tx_toa  = pkts_all(p,3);
        
        neighbors = find(dist_mat(tx_node,:) >= A_min & dist_mat(tx_node,:) <= A_max);
        neighbors(neighbors == tx_node) = [];
        
        for nbr = neighbors
            delay = rand * 0.1;
            t_retx = tx_time + delay + tx_toa;
            
            if tx_toa == toa_small
                ch_retx = channels_small(randi(channels_normal));
            else
                ch_retx = channels_large(randi(channels_extra));
            end
            
            pkts_retx = [pkts_retx; nbr, t_retx, tx_toa, ch_retx, 1];
        end
    end
    
    pkts_all = [pkts_all; pkts_retx];
    pkts_all = sortrows(pkts_all, 2);
    
    % Verificar colisões
    collisions = 0;
    total_pkts = size(pkts_all,1);
    
    for i1 = 1:total_pkts-1
        for j1 = i1+1:total_pkts
            if pkts_all(j1,2) - pkts_all(i1,2) > max(toa_small, toa_large)
                break
            end
            
            node_i = pkts_all(i1,1);
            node_j = pkts_all(j1,1);
            
            if dist_mat(node_i,node_j) <= A_max && dist_mat(node_i,node_j) >= A_min
                if pkts_all(i1,4) == pkts_all(j1,4)
                    start_i = pkts_all(i1,2);
                    end_i = start_i + pkts_all(i1,3);
                    start_j = pkts_all(j1,2);
                    end_j = start_j + pkts_all(j1,3);
                    
                    if (start_i < end_j) && (start_j < end_i)
                        collisions = collisions + 1;
                    end
                end
            end
        end
    end
    
    % Estimar probabilidade de colisão
    collision_rates(idx) = collisions / (total_pkts*(total_pkts-1)/2);
    
    fprintf('N=%d, Colisões=%d, Prob=%.4f\n', N, collisions, collision_rates(idx));
end

% Plot
figure;
plot(vehicle_counts, collision_rates, '-o', 'LineWidth', 2);
grid on;
xlabel('Número de veículos na rede');
ylabel('Probabilidade de colisão');
title('Probabilidade de colisão vs Número de veículos');
ylim([0 1]);