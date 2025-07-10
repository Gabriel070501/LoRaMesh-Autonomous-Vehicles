clear;
clc;

% Parâmetros
packet_total_bits = 792;     % Total de bits transmitidos por veículo a cada 5s
interval = 5;                % Tempo total por ciclo de envio

% Bitrates por canal
bps_normal = 5.47e3;         % 125 kHz @ SF7
bps_extra  = 21.87e3;        % 500 kHz @ SF7

% Canais totais
total_normals = 64;
total_extras  = 8;

% Interferência variando
interfered_normals = 0:5:40; % de 0 a 40 canais normais ocupados
interfered_extras  = 0:1:4;  % de 0 a 4 canais extras ocupados

% Matriz de resultados
results = zeros(length(interfered_extras), length(interfered_normals));

for i = 1:length(interfered_extras)
    for j = 1:length(interfered_normals)
        % Canais disponíveis
        normal_free = max(total_normals - interfered_normals(j), 0);
        extra_free  = max(total_extras - interfered_extras(i), 0);
        
        % Largura de banda total disponível por 5 segundos
        total_bandwidth = (normal_free * bps_normal + extra_free * bps_extra) * interval;
        
        % Número de veículos suportados
        results(i, j) = floor(total_bandwidth / packet_total_bits);
    end
end

% Plot
[X, Y] = meshgrid(interfered_normals, interfered_extras);
figure;
surf(X, Y, results);
xlabel('Canais Normais Interferidos');
ylabel('Canais Extras Interferidos');
zlabel('Veículos Suportados');
title('Capacidade da Rede vs Interferência Externa (Malha única)');
grid on;
