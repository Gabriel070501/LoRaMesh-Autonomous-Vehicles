% ---- script principal ----
N = 1500;
Z_km2 = 1000; % área em km²
A_min = 600;  % metros
A_max = 1000; % metros

Z_m2 = Z_km2;       % converter para m²
L = sqrt(Z_m2);           % lado do quadrado
num_lados = ceil(sqrt(N));% nós por lado da grade
d = L / (num_lados - 1);  % distância média entre nós adjacentes


f_val = calcula_f(N, Z_m2, A_min, A_max);
fprintf('Fração média f = %.4f\n', f_val);