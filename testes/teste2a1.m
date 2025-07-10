clear; clc;

% Configurações
SF = 7;
CR = 4/5;

BW_normal = 125e3; % Hz
BW_extra = 500e3;  % Hz

packet_small_bytes = 19;
packet_large_bytes = 23;

channels_normal = 64;
channels_extra = 8;

sim_time = 100; % segundos de simulação

vehicle_counts = 100:100:2000;
collision_rates = zeros(size(vehicle_counts));

% Função ToA
calcToA = @(payload, SF, CR, BW) ...
    (8 + max(ceil((8*payload - 4*SF + 28 + 16 - 20*0)/(4*(SF - 2*0))) * (4 + 4), 0)) * (2^SF)/BW + (8 + 4.25)*(2^SF)/BW;

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    
    events = []; % [start_time, end_time, channel]

    % Cada veículo envia 4 pacotes pequenos e 1 grande a cada 5s
    num_intervals = floor(sim_time / 5);
    
    toa_small = calcToA(packet_small_bytes, SF, CR, BW_normal);
    toa_large = calcToA(packet_large_bytes, SF, CR, BW_extra);
    
    for v = 1:N
        for interval = 0:num_intervals-1
            base_time = interval * 5;
            % 4 pacotes pequenos distribuídos aleatoriamente nos 5s
            small_times = base_time + rand(1,4) * 5;
            for t = small_times
                channel = randi(channels_normal);
                events = [events; t, t+toa_small, channel];
            end
            % 1 pacote grande, enviado aleatoriamente no intervalo
            t_large = base_time + rand()*5;
            channel = channels_normal + randi(channels_extra);
            events = [events; t_large, t_large+toa_large, channel];
        end
    end
    
    % Ordenar eventos pelo tempo inicial
    events = sortrows(events,1);
    
    colisao_count = 0;
    total_packets = size(events,1);
    
    % Verificar colisões: dois pacotes no mesmo canal com tempos que se sobrepõem
    for i = 1:total_packets
        for j = i+1:total_packets
            if events(j,1) > events(i,2)
                break; % próximo pacote começa depois do fim do atual, sem colisão
            end
            if events(i,3) == events(j,3) && events(j,1) < events(i,2)
                colisao_count = colisao_count + 1;
                break; % conta uma colisão e passa pro próximo pacote
            end
        end
    end
    
    collision_rates(idx) = colisao_count / total_packets;
    fprintf('N=%d -> Probabilidade de colisão: %.4f\n', N, collision_rates(idx));
end

% Plotagem
figure;
plot(vehicle_counts, collision_rates, '-o', 'LineWidth', 2);
xlabel('Número de veículos');
ylabel('Probabilidade de colisão');
title('Probabilidade de colisão em rede LoRa com SF7, CR4/5');
grid on;
