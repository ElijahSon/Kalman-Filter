%% This file was made to understand what is the Kalman Filter (KF)
%
%
%
%
clear, close all; 
%%
Ts = 1; % Sample Time 
q = 0.1; % Standard Deviation 
Q = q*q*[(Ts^3)/3, (Ts^2)/2 ; (Ts^2)/2 , Ts]; % Matrice de covariance


