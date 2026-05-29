%% Filtre de Kalman 
% -----------------------------------------------------------------------------
% L'objectif est de generer une trajectoire que nous bruiterons plus tard
% afin d'appliquer le filtre de Kalman sur cette derniere et se rendre
% compte de ses performances.
% -----------------------------------------------------------------------------
clear, close all; 

% Initialisation et generation de trajectoire 
% MRU ou MUA

Ts = 1; % Periode d'echantillonnage 

syms q phi_kmoins1 x_kmoins1 k G u_k real; 
% x_k = phi_kmoins1*x_kmoins1 + G*u_k   ; 
 
% Paramètres du système
initialState = [5; 2];  % [x, y, vx, vy] - position et vitesse initiales
timeStep = 0.05;  % Pas de temps
numSteps = 100;   % Nombre d'étapes

% Vecteurs d'état
state = initialState;

% Initialisation des vecteurs de position
xTrajectory = zeros(numSteps, 1);
yTrajectory = zeros(numSteps, 1);

% Simulation de la trajectoire
for step = 1:numSteps
    % Mettez à jour la position en fonction de la vitesse
    xTrajectory(step) = state(1);
    yTrajectory(step) = state(2);
    
    state(1) = state(1) + rand(1);  % Mise à jour de la position x
    state(2) = state(2) +  rand(1);  % Mise à jour de la position y
    
end

% trajectoire
figure, plot(xTrajectory, yTrajectory, 'r', 'LineWidth', 2);
title('Trajectoire en fonction des vecteurs d''état');
xlabel('Position X'), ylabel('Position Y'), grid on;






