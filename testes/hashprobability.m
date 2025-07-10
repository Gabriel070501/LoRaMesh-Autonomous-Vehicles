clear; clc;

% Número de bits do hash
b = 64;
N = 1:2000000000; % Número de IDs gerados
M = 2^b;      % Total de combinações possíveis

% Fórmula do paradoxo do aniversário
P = 1 - exp(-N .* (N - 1) / (2 * M));

% Plot
figure;
plot(N, P * 100, 'b', 'LineWidth', 2);
grid on;
xlabel('Número de identificadores gerados (n)');
ylabel('Probabilidade de colisão (%)');
title('Probabilidade de colisão para identificadores de 64 bits (FNV-1a ou TRNG)');
xlim([0 2000000000]);
ylim([0 100]);

ax = gca;
ax.XAxis.Exponent = 0;
ax.YAxis.Exponent = 0;
