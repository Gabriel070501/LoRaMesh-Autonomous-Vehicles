clc; clear; close all
rng(1)

% Parâmetros
N = 150;                   
T_ciclo = 20;               
alc_min = 400;              
alc_max = 1000;              
area_total = 1e6;         
num_fixos = 10;             

Lado = sqrt(area_total);

pos_movel_init = rand(N,2)*Lado;
pos_fixos = rand(num_fixos,2)*Lado;

velocidade = 15*ones(N,1);
direcao = 2*pi*rand(N,1);

% Para paralelismo
poolobj = gcp('nocreate');
if isempty(poolobj)
    parpool('local',16)
elseif poolobj.NumWorkers ~= 16
    delete(poolobj);
    parpool('local',16)
end

% Variáveis para salvar resultados
pos_movel_all = zeros(N,2,T_ciclo);
num_vizinhos_all = zeros(T_ciclo,1);
colisoes_all = zeros(T_ciclo,1);

% Inicializa posição móvel
pos_movel = pos_movel_init;
direcao_local = direcao;

% Para paralelismo, vamos calcular as posições de cada segundo sequencialmente,
% mas o cálculo pesado dentro de parfor que rodaria por exemplo as colisões simultâneas.

% Aqui, o mais pesado é o cálculo das colisões. Então, pra cada instante,
% vamos paralelizar só a verificação das colisões.

for seg = 1:T_ciclo
    % Atualiza posições móveis
    pos_movel(:,1) = pos_movel(:,1) + velocidade .* cos(direcao_local);
    pos_movel(:,2) = pos_movel(:,2) + velocidade .* sin(direcao_local);
    
    % Rebote nas bordas
    fora_x = (pos_movel(:,1) < 0) | (pos_movel(:,1) > Lado);
    direcao_local(fora_x) = pi - direcao_local(fora_x);
    pos_movel(fora_x,1) = min(max(pos_movel(fora_x,1),0),Lado);
    
    fora_y = (pos_movel(:,2) < 0) | (pos_movel(:,2) > Lado);
    direcao_local(fora_y) = -direcao_local(fora_y);
    pos_movel(fora_y,2) = min(max(pos_movel(fora_y,2),0),Lado);
    
    % Junta posições
    pos_total = [pos_movel; pos_fixos];
    N_total = size(pos_total,1);
    
    % Distâncias
    D = squareform(pdist(pos_total));
    
    % Matriz adjacente
    adj = (D >= alc_min) & (D <= alc_max);
    adj(1:N_total+1:end) = 0;
    
    % Num vizinhos médio (somente para móveis)
    num_vizinhos = sum(adj(1:N,1:N_total),2);
    num_vizinhos_all(seg) = mean(num_vizinhos);
    
    % Simulação canais
    canais = randi(72, N_total, 1);
    
    % Paraleliza cálculo colisão (cada nó verifica se tem colisão)
    colisao_flag = false(N_total,1);
    parfor i = 1:N_total
        if sum(adj(i,:) & (canais == canais(i))') > 1
            colisao_flag(i) = true;
        end
    end
    colisoes_all(seg) = sum(colisao_flag);
    
    % Salva posições para plot
    pos_movel_all(:,:,seg) = pos_movel;
end

% --- Agora plot dinâmico ---
figure('Name','Simulação com Paralelismo + Plot dinâmico','NumberTitle','off')
hold on; axis equal
xlabel('x (m)'); ylabel('y (m)')
xlim([0 Lado]); ylim([0 Lado])
h_fixos = plot(pos_fixos(:,1), pos_fixos(:,2), 'rs', 'MarkerSize',8);
h_veic = plot(pos_movel_all(:,1,1), pos_movel_all(:,2,1), 'bo', 'MarkerSize',4);
h_linhas = [];
h_text = text(0, Lado*1.02, '', 'FontSize', 12);

for seg = 1:T_ciclo
    pos_movel = pos_movel_all(:,:,seg);
    pos_total = [pos_movel; pos_fixos];
    
    % Distância para plot
    D = squareform(pdist(pos_total));
    adj = (D >= alc_min) & (D <= alc_max);
    adj(1:size(adj,1)+1:end) = 0;
    
    % Apaga linhas antigas
    if ~isempty(h_linhas)
        delete(h_linhas)
    end
    
    % Desenha linhas conexões
    [lin_i, lin_j] = find(triu(adj,1));
    h_linhas = gobjects(length(lin_i),1);
    for k = 1:length(lin_i)
        x = [pos_total(lin_i(k),1), pos_total(lin_j(k),1)];
        y = [pos_total(lin_i(k),2), pos_total(lin_j(k),2)];
        h_linhas(k) = plot(x,y,'-','Color',[0.7 0.7 0.7 0.1]);
    end
    
    set(h_veic, 'XData', pos_movel(:,1), 'YData', pos_movel(:,2));
    set(h_text, 'String', sprintf('Seg: %d   Colisões: %d   Média vizinhos: %.2f', ...
        seg, colisoes_all(seg), num_vizinhos_all(seg)));
    
    drawnow limitrate
    pause(0.1)
end
