clear; clc;

% Dados fixos
packet_small = 19 * 8; % bits
packet_large = 23 * 8; % bits

bps_normal = 5470;     % 125 kHz SF7
bps_extra  = 21875;    % 500 kHz SF7

channels_normal = 64;
channels_extra  = 8;

interval = 5; % segundos

% Retransmissões
max_retransmissions = 2;
total_transmissions_per_packet = 1 + max_retransmissions;

% Quantidade de veículos para testar
vehicle_counts = 100:100:5000;
collision_rates = zeros(size(vehicle_counts));

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    
    % Pacotes por veículo a cada 5s
    packets_small_per_vehicle = 4;
    packets_large_per_vehicle = 1;
    
    % Total de pacotes considerando retransmissão
    total_small = N * packets_small_per_vehicle * total_transmissions_per_packet * packet_small;
    total_large = N * packets_large_per_vehicle * total_transmissions_per_packet * packet_large;
    
    % Capacidade dos canais em 5s
    capacity_normal = bps_normal * channels_normal * interval;
    capacity_extra  = bps_extra  * channels_extra  * interval;
    
    % Uso dos canais
    used_small = min(total_small, capacity_normal);
    overflow_small = max(0, total_small - capacity_normal);
    
    used_large = min(total_large, capacity_extra);
    
    % Se sobrou capacidade normal, pacotes grandes extras tentam usar
    leftover_capacity_normal = max(0, capacity_normal - used_small);
    used_large_in_normal = min(overflow_small + (total_large - used_large), leftover_capacity_normal);
    
    total_used = used_small + used_large + used_large_in_normal;
    
    total_data = total_small + total_large;
    
    % Dados perdidos por falta de capacidade (colisão)
    lost = total_data - total_used;
    
    collision_rates(idx) = lost / total_data;
end

% Plotagem
figure;
plot(vehicle_counts, collision_rates, 'LineWidth', 2);
xlabel('Número de veículos');
ylabel('Taxa de colisão (perda)');
title('Taxa de colisão na rede LoRa Mesh com retransmissões');
grid on;
