%% Kalman Filter
% Fils Elie BOUNGOUERES
%
% Objectives:
%   - Generate a mathematical trajectory
%   - Add measurement noise to the generated trajectory
%   - Apply a Kalman Filter to recover the original trajectory

%% ------------------------------------------------------------------------
clear; close all; clc;

%% Model parameters

dt = 1;                  % Time step

A = [1 dt;               % State transition matrix
     0 1];

H = [1 0];               % Observation matrix: only position is measured

q = 0.1;                 % Process noise standard deviation
Q = q^2 * eye(2);        % Process noise covariance

R = 0.1;                 % Measurement noise variance

alpha = 3.5;             % Initial covariance scaling
P0 = alpha * eye(2);     % Initial covariance matrix

N = 50;                  % Number of time steps

%% Initial states

x_true = [0; 0];         % True initial state: [position; velocity]
x_hat = [0; 0];          % Estimated initial state

P_hat = P0;              % Initial estimate covariance

%% Storage variables

realTrajectory = zeros(2, N);
measurements = zeros(1, N);
filteredTrajectory = zeros(2, N);

%% Real trajectory and measurement generation

for t = 1:N
    % True system evolution
    processNoise = q * randn(2, 1);
    x_true = A * x_true + processNoise;

    % Store real trajectory
    realTrajectory(:, t) = x_true;

    % Generate measurement: position plus measurement noise
    measurementNoise = sqrt(R) * randn;
    measurements(t) = H * x_true + measurementNoise;
end

%% Kalman Filtering

for t = 1:N
    % Prediction step
    x_pred = A * x_hat;
    P_pred = A * P_hat * A' + Q;

    % Correction step
    S = H * P_pred * H' + R;       % Innovation covariance
    K = P_pred * H' / S;           % Kalman gain

    innovation = measurements(t) - H * x_pred;
    x_hat = x_pred + K * innovation;

    P_hat = (eye(2) - K * H) * P_pred;

    % Store filtered trajectory
    filteredTrajectory(:, t) = x_hat;
end

%% Plot results

figure;

plot(1:N, realTrajectory(1, :), 'b', ...
    'LineWidth', 2, ...
    'DisplayName', 'True Trajectory');

hold on; grid on;

plot(1:N, measurements, 'r.', ...
    'MarkerSize', 18, ...
    'DisplayName', 'Noisy Measurements');

plot(1:N, filteredTrajectory(1, :), 'g', ...
    'LineWidth', 3, ...
    'DisplayName', 'Filtered Trajectory');

legend('Location', 'best');

title('Trajectory Generation with Kalman Filter');
xlabel('Time');
ylabel('Position');