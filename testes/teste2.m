clear;
clc;

% Tamanhos dos pacotes em bits
packet_small = 19 * 8;
packet_large = 23 * 8;

% Intervalo de transmissão
interval = 5; % segundos

% Canais disponíveis
channels_normal = 64;
channels_extra  = 8;

% Spreading Factors e suas taxas estimadas em kbps (tabela acima)
SFs = 7:12;
bps_normal_kbps = [5.47, 3.13, 1.76, 0.98, 0.54, 0.29];
bps_extra_kbps  = [21.87, 12.5, 7.03, 3.91, 2.15, 1.17];

max_vehicles = zeros(1, length(SFs));

for i = 1:length(SFs)
    % Taxa de transmissão em bps
    bps_normal = bps_normal_kbps(i) * 1000;
    bps_extra  = bps_extra_kbps(i)  * 1000;
    
    % Capacidade da rede por intervalo de 5s
    cap_normal = bps_normal * channels_normal * interval;
    cap_extra  = bps_extra  * channels_extra  * interval;
    
    % Dados por veículo a cada 5s
    data_small = 4 * packet_small;
    data_large = 1 * packet_large;
    data_total = data_small + data_large;
    
    % Testa quantos veículos cabem até saturar
    max_v = floor((cap_normal + cap_extra) / data_total);
    max_vehicles(i) = max_v;
end

% Plotagem
figure;
bar(SFs, max_vehicles);
xlabel('Spreading Factor (SF)');
ylabel('Máximo de visibilidade de nós (sem saturar)');
title('Impacto do Spreading Factor na Capacidade da Rede LoRaMesh');
grid on;