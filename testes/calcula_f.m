function f = calcula_f(N, Z, A_min, A_max)
    % N       = número total de nós
    % Z_km2   = área total em km²
    % A_min   = distância mínima para retransmitir (em metros)
    % A_max   = distância máxima para retransmitir (em metros)

    % Converter área de km² para m²
    Z_m2 = Z * 1e6;
    L = sqrt(Z_m2);  % lado do quadrado
    
    % Gerar coordenadas aleatórias uniformemente distribuídas
    Xg = rand(N, 1) * L;
    Yg = rand(N, 1) * L;

    % Matriz de distâncias NxN
    dist_mat = squareform(pdist([Xg, Yg]));

    % Calcular distância média (entre todos os pares, exceto ele mesmo)
    max_retx_limit = 5; % limite de retransmissões no alcance
    % Calcular f individual para cada nó
    f_per_node = zeros(N,1);
    veiculos_alcance_total = 0;
    for i = 1:N
        distancias = dist_mat(i,:);
        distancias(i) = inf;  % ignora ele mesmo
        
        veiculos_no_alcance = sum(distancias >= A_min & distancias <= A_max);
        veiculos_alcance_total = veiculos_alcance_total + veiculos_no_alcance;
        % Se > limite, limitar para max_retx_limit
        if veiculos_no_alcance > max_retx_limit
            veiculos_no_alcance = max_retx_limit;
        end
        
        % Fração de nós a distância entre A_min e A_max
        f_per_node(i) = veiculos_no_alcance / (N - 1);
    end

    % Média da fração
    f = mean(f_per_node);
    
    media_veiculos_alcance = veiculos_alcance_total / N;
    fprintf('Média de veículos no alcance (%.0f a %.0f m): %.2f veículos\n', A_min, A_max, media_veiculos_alcance);
end