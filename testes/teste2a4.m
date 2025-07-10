clear; clc;

% Parâmetros
packet_small_bytes = 19;
packet_large_bytes = 23;

bps_normal = 5470;   % 125kHz SF7
bps_extra  = 21875;  % 500kHz SF7

channels_normal = 64;
channels_extra = 8;

interval = 5; % segundos

Z = 1344;
A_min = 400;
A_max = 1000;

 % fração média de nós retransmitindo cada pacote

vehicle_counts = 0:20:5000;
collision_rates = zeros(size(vehicle_counts));

bits_small = packet_small_bytes * 8;
bits_large = packet_large_bytes * 8;

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    
    f_retransmit = calcula_f(N, Z, A_min, A_max);
    
    % Pacotes originais por nó em 5s
    pkts_small_orig = 4;
    pkts_large_orig = 1;
    
    % Total pacotes originais na rede em 5s
    total_small_orig = N * pkts_small_orig;
    total_large_orig = N * pkts_large_orig;
    
    % Retransmissões médias (fração f_retransmit dos outros nós retransmitindo cada pacote)
    avg_retx_per_pkt = f_retransmit * (N - 1);
    % Total pacotes considerando retransmissões
    total_small = total_small_orig * (1 + avg_retx_per_pkt);
    total_large = total_large_orig * (1 + avg_retx_per_pkt);
    
    % Bits totais a transmitir por tipo
    bits_small_total = total_small * bits_small;
    bits_large_total = total_large * bits_large;
    
    % Capacidade total da rede em 5s
    capacity_normal = bps_normal * channels_normal * interval;
    capacity_extra  = bps_extra  * channels_extra  * interval;
    
    % Como os pacotes pequenos priorizam canais normais e grandes os extras:
    % Verificar saturação em cada conjunto de canais
    
    % Se exceder capacidade dos canais normais para pequenos:
    if bits_small_total <= capacity_normal
        lost_small = 0;
    else
        lost_small = bits_small_total - capacity_normal;
    end
    
    % Se exceder capacidade dos canais extras para grandes:
    if bits_large_total <= capacity_extra
        lost_large = 0;
    else
        lost_large = bits_large_total - capacity_extra;
    end
    
    % Total perdido
    lost_total = lost_small + lost_large;
    
    % Total bits a transmitir
    total_bits = bits_small_total + bits_large_total;
    
    % Taxa de colisão = fração de bits perdidos
    collision_rates(idx) = lost_total / total_bits;
end

% Plot
figure;
plot(vehicle_counts, collision_rates, 'LineWidth', 2);
grid on;
xlabel('Número de nós na rede');
ylabel('Taxa de colisão (fração de bits perdidos)');
title(sprintf('Taxa de colisão considerando retransmissões'));
ylim([0 1]);


