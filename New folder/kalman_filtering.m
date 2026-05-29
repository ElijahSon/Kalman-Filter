%% Filtre de Kalman 
% -----------------------------------------------------------------------------
% L'objectif est de generer une trajectoire que nous bruiterons plus tard
% afin d'appliquer le filtre de Kalman sur cette derniere et se rendre
% compte de ses performances.
% -----------------------------------------------------------------------------
clear, close all; 
% Paramètres du système
dt = 0.1;  % Intervalle de temps ou tech 
A = [1 dt; 0 1];  % Matrice de transition du système
H = [1 0];  % Matrice d'observation

% Processus de bruit blanc
Q = 0.01 * eye(2);  % Covariance du bruit du processus dictee par un ecart type 
R = 0.1;  % Covariance du bruit de mesure

% Initialisation des variables
x_true = [0; 0];  % Position réelle
x_est = [0; 0];  % Estimation de la position
P = eye(2);  % Covariance de l'estimation

% Génération de la trajectoire et application du filtre de Kalman
num_steps = 100;
true_trajectory = zeros(2, num_steps);
estimated_trajectory = zeros(2, num_steps);

for k = 1:num_steps
    % Génération de la trajectoire réelle 
    x_true = A * x_true + sqrt(Q) * randn(2, 1);

    % Mesure avec bruit
    z = H * x_true + sqrt(R) * randn;

    % Prédiction de l'état
    x_est_pred = A * x_est;
    P_pred = A * P * A' + Q;

    % Mise à jour de l'estimation
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_est_pred + K * (z - H * x_est_pred);
    P = (eye(2) - K * H) * P_pred;

    % Stockage des résultats pour trajectoire réelle et estimée
    true_trajectory(:, k) = x_true;
    estimated_trajectory(:, k) = x_est;
end

% Affichage des résultats
figure;
plot(true_trajectory(1, :), true_trajectory(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'Trajectoire Réelle');
hold on;
plot(estimated_trajectory(1, :), estimated_trajectory(2, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Trajectoire Estimée');
xlabel('Position en X');
ylabel('Position en Y');
legend('show');
title('Génération de Trajectoire avec Filtre de Kalman');
grid on 