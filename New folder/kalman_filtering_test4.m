%% Filtre de Kalman 
% -----------------------------------------------------------------------------
% L'objectif est de generer une trajectoire que nous bruiterons plus tard
% afin d'appliquer le filtre de Kalman sur cette derniere et se rendre
% compte de ses performances.
% -----------------------------------------------------------------------------
clear, close all

% Declaration des variables symboliques 
syms x x_k xdot_k  real

% Paramètres du modèle
A = [1 1; 0 1];  % Matrice d'état
H = [1 0];      % Matrice d'observation

% Bruit du processus et de mesure
Q = 0.01 * eye(2);  % Covariance du bruit du processus
R = 0.1;            % Variance du bruit de mesure

% État initial et covariance
x0 = [0; 0];        % Etat init 
P0 = eye(2);        % Covariance init

% Génération de la trajectoire réelle
N = 50;            % Nombre de pas point qui decrirons notre trajectoire  
realTrajectory = zeros(2, N);

for t = 1:N
    realTrajectory(:, t) = A * x0;
    x0 = realTrajectory(:, t) + sqrtm(Q) * randn(2, 1);
end

% Génération de bruit de mesure
BruitDeMesure = sqrt(R) * randn(1, N);

% Generation de la trajectoire bruitee 
measurements = H * realTrajectory + BruitDeMesure;

% Filtrage de Kalman
filteredTrajectory = zeros(2, N);
x_hat = x0;
P_hat = P0;

for t = 1:N
    % Prédiction (premiere etape) 
    x_pred = A * x_hat;
    P_pred = A * P_hat * A' + Q;
    
    % Correction (2 eme etape) 
    K = P_pred * H' / (H * P_pred * H' + R);
    x_hat = x_pred + K * (measurements(:, t) - H * x_pred);
    P_hat = (eye(2) - K * H) * P_pred;
    
    filteredTrajectory(:, t) = x_hat;
end

% Affichage des résultats
figure;
plot(1:N, realTrajectory(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'Trajectoire Réelle');
hold on;
plot(1:N, measurements, 'rx', 'DisplayName', 'Mesures');
plot(1:N, filteredTrajectory(1, :), 'g', 'LineWidth', 2, 'DisplayName', 'Trajectoire Filtrée');
legend('Location', 'Best');
title('Génération de Trajectoire avec Filtre de Kalman');
xlabel('Temps');
ylabel('Position');
