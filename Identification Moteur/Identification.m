clc
clear all
close all

%% Data start
load("data_1v_4-09_100hz.mat");
% Vm - entrée (tension)
% servo - angle de moteur
% omega_c - vitesse angulaire
% tsimu - temps de simulation

%% Test 1
dT = tsimu(2);

%Diff1_1 = diff(servo)/dT;
%Diff2_1 = diff(Diff1_1)/dT;

%Y = servo(1:end-2, 1);
%X = [Vm(1:end-2, 1), Diff1_1(1:end-1, 1), Diff2_1];

%A = pinv(X)*Y

%% Test 2
Diff1_2 = diff(omega_c)/dT;

Y2 = omega_c(1:end-1, 1);
X2 = [Vm(1:end-2, 1), Diff1_2];

A2 = pinv(X2)*Y2

%% Mise en valeur
%Charge et engrenage
n_g = 0.9000;
K_g = 70;

%Partie électrique du moteur
k_m = 0.0076776;    % Nm/A
n_m = 0.69;
k_t = 0.0076830;    %V/(Rad/s)

%Partie mécanique du moteur
J_m = 3.9001*10^(-07);  %kg*m^2
J_eq = 0.0017728;   %kg*m^2

%Sphère
m_s = 0.0640;   % kg
J_s = 4.1290*10^(-06);  % kg*m^2
r_s = 0.0127;   % m
g = 9.81;   % m/s^2

%Poutre
L = 0.4254; % m
r_arm = 0.0254; % m

%% Calculs
J_c = J_eq - ((K_g^2)*n_g*J_m)

R_m = ((-1*A2(2))*K_g*n_g*n_m*k_t)/(A2(1)*J_c)

B_eq = ((K_g*n_g*n_m*k_t)/(A2(1)*R_m))-((K_g^2)*n_g*J_m)-((n_g*k_m*(K_g^2)*n_m*k_t)/R_m)