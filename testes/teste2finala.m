clear; clc;

% --------- Parâmetros fixos ---------
packet_small_bytes = 19;
packet_large_bytes = 23;

bps_normal = 5470;   % taxa bps para canais 125kHz SF7
bps_extra  = 21875;  % taxa bps para canais 500kHz SF7

channels_normal = 64;
channels_extra = 8;

interval = 5; % janela de 5 segundos para cálculo

Z = 13.44; % área da cidade em km^2
A_min = 400;  % distância mínima para retransmissão (m)
A_max = 1000; % distância máxima para retransmissão (m)
max_retx = 2; % número máximo de retransmissões em cascata

vehicle_counts = 100:25:5000;
collision_rates = zeros(size(vehicle_counts));

bits_small = packet_small_bytes * 8;
bits_large = packet_large_bytes * 8;

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    
    % Calcula f baseado na distância média entre veículos para o N atual
    f_retransmit = calcula_f(N, Z, A_min, A_max); 
    
    % Pacotes originais por nó em 5s
    pkts_small_orig = 4;
    pkts_large_orig = 1;
    
    % Total pacotes originais na rede em 5s
    total_small_orig = N * pkts_small_orig;
    total_large_orig = N * pkts_large_orig;
    
    % Fator r para retransmissão em cascata
    r = f_retransmit * (N - 1) / N;
    
    if r >= 1
        warning('Fator r >= 1, retransmissão pode ser ilimitada. Ajustando para 0.99');
        r = 0.99; % evitar infinito
    end
    
    powers = 0:max_retx;
    retrans_factor = sum(r .^ powers);
    
    % Média de retransmissões por pacote
    avg_retx_per_pkt = retrans_factor - 1;
    
    % Total pacotes considerando retransmissões em cascata
    total_small = total_small_orig * retrans_factor;
    total_large = total_large_orig * retrans_factor;
    
    % Bits totais a transmitir por tipo
    bits_small_total = total_small * bits_small;
    bits_large_total = total_large * bits_large;
    
    % Capacidade total da rede em 5s
    capacity_normal = bps_normal * channels_normal * interval;
    capacity_extra  = bps_extra  * channels_extra  * interval;
    
    % Saturação e perdas por tipo de canal
    if bits_small_total <= capacity_normal
        lost_small = 0;
    else
        lost_small = bits_small_total - capacity_normal;
    end
    
    if bits_large_total <= capacity_extra
        lost_large = 0;
    else
        lost_large = bits_large_total - capacity_extra;
    end
    
    % Total perdido e taxa de colisão
    lost_total = lost_small + lost_large;
    total_bits = bits_small_total + bits_large_total;
    collision_rates(idx) = lost_total / total_bits;
end

% Plotando resultado
figure;
plot(vehicle_counts, collision_rates, 'LineWidth', 2);
grid on;
xlabel('Número de nós na rede');
ylabel('Taxa de colisão (fração de bits perdidos)');
title(sprintf('Taxa de colisão com retransmissão em cascata (Área = %.0f km²)', Z));
ylim([0 1]);

