clear;
clc;

% Dados fixos
packet_small = 19 * 8; % bits
packet_large = 23 * 8; % bits

bps_normal = 5470;     % 125 kHz SF7
bps_extra  = 21875;    % 500 kHz SF7

channels_normal = 64;
channels_extra  = 7;

interval = 5; % segundos

% Quantidade de veículos para testar
vehicle_counts = 100:100:5000;
collision_rates = zeros(size(vehicle_counts));

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    
    % Carga total de dados transmitidos por N veículos em 5 s
    data_per_vehicle = (4 * packet_small + 1 * packet_large); % bits por 5 s
    total_data = N * data_per_vehicle; % bits
    
    % Capacidade total da rede (todos canais, em 5 s)
    capacity_normal = bps_normal * channels_normal * interval;
    capacity_extra  = bps_extra  * channels_extra  * interval;
    
    % Alocação de pacotes
    total_small = N * 4 * packet_small;
    total_large = N * 1 * packet_large;
    
    % Verifica se os canais dão conta
    used_small = min(total_small, capacity_normal);
    overflow_small = max(0, total_small - capacity_normal);
    
    used_large = min(total_large, capacity_extra);
    % Se sobrou espaço nos canais normais, os pacotes grandes extras tentam usar
    leftover_capacity_normal = max(0, capacity_normal - used_small);
    used_large_in_normal = min(overflow_small + (total_large - used_large), leftover_capacity_normal);
    total_used = used_small + used_large + used_large_in_normal;
    
    % Dados perdidos por falta de capacidade
    lost = total_data - total_used;
    collision_rates(idx) = lost / total_data;
end

% Plotagem
figure;
plot(vehicle_counts, collision_rates, 'LineWidth', 2);
xlabel('Número de veículos');
ylabel('Taxa de colisão (fração de bits perdidos)');
title('Taxa de Colisão sem retransmissão');
grid on;