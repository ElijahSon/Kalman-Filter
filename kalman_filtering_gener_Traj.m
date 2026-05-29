%% Kalmam Filter  
% Fils Elie BOUNGOUERES 

% Objectives:
%   - Generate a mathematical trajectory
%   - Add measurement noise to the generated trajectory
%   - Apply a Kalman Filter to recover the original trajectory
%% ------------------------------------------------------------------------
clear, close all

% Declaration des variables symboliques 
syms x x_k xdot_k  real

% Model parameters 
A = [1 1; 0 1];  % State Matrix
H = [1 0];       % Observation Matrix
q = 0.1 ;        % Standard Deviation
alpha = 03.5; 

% Process Noise for measure
Q = q^2*eye(2);  % Process Noise Covariance 
Qc = chol(Q); 
R = 0.1;         % Variance du bruit de mesure

% Initiak state and covariance
x0 = [0; 0];     % init state 
P0 = alpha*eye(2);        % Covariance init

% Real Trajectory Generation
N = 50;            % Number of step point to describe the trajectory
realTrajectory = zeros(2, N);

for t = 1:N
    realTrajectory(:, t) = A*x0;
    x0 = realTrajectory(:, t) + sqrtm(Q)*randn(2, 1);
end

% Measure Noise Generation
BruitDeMesure = sqrt(R)*randn(1, N);

% Noised Trajectory Generated 
measurements = H*realTrajectory + BruitDeMesure;

% Kalman Filtering
filteredTrajectory = zeros(2, N);
x_hat = x0;
P_hat = P0;

for t = 1:N
    % Prédiction (1st step) 
    x_pred = A * x_hat;
    P_pred = A * P_hat * A' + Q;
    
    % Correction (2nd step) 
    K = P_pred * H' / (H * P_pred * H' + R);
    x_hat = x_pred + K * (measurements(:, t) - H * x_pred);
    P_hat = (eye(2) - K * H) * P_pred;

    % Filtred trajectory signal stored in a matrix 
    filteredTrajectory(:, t) = x_hat;
end

% Plots 
figure, 
plot(1:N, realTrajectory(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'Trajectoire Réelle'), hold on, grid on
plot(1:N, measurements, 'r', 'LineWidth',3, 'DisplayName', 'Mesures');
plot(1:N, filteredTrajectory(1, :), 'g', 'LineWidth', 3, 'DisplayName', 'Trajectoire Filtrée');
legend('Location', 'Best');
title('Génération de Trajectoire avec Filtre de Kalman'), xlabel('Temps'), ylabel('Position');

