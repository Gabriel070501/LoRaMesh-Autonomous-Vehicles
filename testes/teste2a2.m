clear; clc;

% Parâmetros básicos
SF = 7;
CR = 4/5;
BW_normal = 125e3;
BW_extra = 500e3;

packet_small_bytes = 19;
packet_large_bytes = 23;

channels_normal = 64;
channels_extra = 8;

sim_time = 100; % segundos
vehicle_counts = 100:100:3400;
collision_rates = zeros(size(vehicle_counts));

% Função para calcular Time on Air (ToA)
calcToA = @(payload, SF, CR, BW) ...
    (8 + max(ceil((8*payload - 4*SF + 28 + 16 - 20*0)/(4*(SF - 2*0))) * (4 + 4), 0)) * (2^SF)/BW + (8 + 4.25)*(2^SF)/BW;

toa_small = calcToA(packet_small_bytes, SF, CR, BW_normal);
toa_large = calcToA(packet_large_bytes, SF, CR, BW_extra);

for idx = 1:length(vehicle_counts)
    N = vehicle_counts(idx);
    
    % Vetores para registrar ocupação dos canais: estrutura (canal x tempo)
    % Para evitar uso intenso de memória, vamos armazenar os eventos no tempo como intervalos
    
    % Listas de eventos para canais normais e extras: cada linha [start_time, end_time]
    channels_norm_events = cell(channels_normal,1);
    channels_extra_events = cell(channels_extra,1);
    
    num_intervals = floor(sim_time / 5);
    
    colisao_count = 0;
    total_packets = N * num_intervals * (4+1); % 4 pequenos + 1 grande por intervalo
    
    for interval = 0:num_intervals-1
        base_time = interval * 5;
        
        for v = 1:N
            % --- Pacotes pequenos ---
            % Distribuir 4 pacotes pequenos dentro dos 5s
            small_times = base_time + rand(1,4)*5;
            for t = small_times
                % Escolher canal menos saturado (menor número de eventos e que não esteja ocupado nesse intervalo)
                chosen_channel = 0;
                for ch = 1:channels_normal
                    evts = channels_norm_events{ch};
                    if isempty(evts)
                        % canal livre
                        chosen_channel = ch;
                        break;
                    else
                        % verificar se o canal está livre no intervalo [t, t+toa_small]
                        busy = any( (evts(:,1) < t+toa_small) & (evts(:,2) > t) );
                        if ~busy
                            chosen_channel = ch;
                            break;
                        end
                    end
                end
                
                if chosen_channel == 0
                    % Todos os canais ocupados nesse intervalo, colisão ocorre
                    colisao_count = colisao_count +1;
                    % Para simulação, colocar no canal 1 (ou qualquer) pra continuar
                    chosen_channel = 1;
                else
                    % Inserir evento no canal escolhido
                    channels_norm_events{chosen_channel} = [channels_norm_events{chosen_channel}; t, t+toa_small];
                end
            end
            
            % --- Pacote grande ---
            t = base_time + rand()*5;
            
            chosen_channel = 0;
            for ch = 1:channels_extra
                evts = channels_extra_events{ch};
                if isempty(evts)
                    chosen_channel = ch;
                    break;
                else
                    busy = any( (evts(:,1) < t+toa_large) & (evts(:,2) > t) );
                    if ~busy
                        chosen_channel = ch;
                        break;
                    end
                end
            end
            
            if chosen_channel == 0
                colisao_count = colisao_count +1;
                chosen_channel = 1;
            else
                channels_extra_events{chosen_channel} = [channels_extra_events{chosen_channel}; t, t+toa_large];
            end
        end
    end
    
    collision_rates(idx) = colisao_count / total_packets;
    fprintf('Veículos: %d, Probabilidade de colisão: %.4f\n', N, collision_rates(idx));
end

% Plot
figure;
plot(vehicle_counts, collision_rates, '-o','LineWidth',2);
xlabel('Número de veículos');
ylabel('Probabilidade de colisão');
title('Probabilidade de colisão com canais alocados conforme saturação');
grid on;