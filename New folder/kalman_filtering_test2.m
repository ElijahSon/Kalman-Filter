%% Filtre de Kalman 
% -----------------------------------------------------------------------------
% L'objectif est de generer une trajectoire que nous bruiterons plus tard
% afin d'appliquer le filtre de Kalman sur cette derniere et se rendre
% compte de ses performances.
% -----------------------------------------------------------------------------
clear, close all

% Paramètres du modèle
A = 1; % Matrice d'état
H = 1; % Matrice d'observation
Q = 0.01; % Covariance du bruit de processus
R = 0.1; % Covariance du bruit de mesure

% Initialisation
x = 0; % État initial
P = 1; % Covariance de l'état initial

% Nombre d'itérations
num_iterations = 100;

% Générer la trajectoire et les mesures bruitées
true_trajectory = zeros(1, num_iterations);
noisy_measurements = zeros(1, num_iterations);

for k = 1:num_iterations
    % Générer la trajectoire réelle (linéaire ici pour l'exemple)
    true_trajectory(k) = A * x;
    
    % Générer une mesure bruitée
    measurement_noise = sqrt(R) * randn;
    noisy_measurements(k) = H * true_trajectory(k) + measurement_noise;
    
    % Mise à jour du filtre de Kalman
    % Prédiction de l'état
    x_hat_minus = A * x;
    % Prédiction de la covariance
    P_minus = A * P * A' + Q;
    % Calcul du gain de Kalman
    K = P_minus * H' / (H * P_minus * H' + R);
    % Mise à jour de l'état estimé
    x = x_hat_minus + K * (noisy_measurements(k) - H * x_hat_minus);
    % Mise à jour de la covariance estimée
    P = (1 - K * H) * P_minus;
end

% Tracer les résultats
figure;
plot(true_trajectory, 'g', 'LineWidth', 2, 'DisplayName', 'Trajectoire Réelle');
hold on;
plot(noisy_measurements, 'r', 'DisplayName', 'Mesures Bruitées');
plot(x_hat_minus, 'b', 'LineWidth', 2, 'DisplayName', 'Trajectoire Estimée');
legend('show');
title('Génération de Trajectoire avec Filtre de Kalman');
xlabel('Itérations');
ylabel('Valeurs');
grid on;
hold off;
