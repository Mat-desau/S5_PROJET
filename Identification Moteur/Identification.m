clc
clear all
close all

%% Data start
load("data_1v_4-09_100hz.mat");
% Vm - entrée (tension)
% servo - angle de moteur
% omega_c - vitesse angulaire
% tsimu - temps de simulation

%% Calcul des coefficient Fonction de transfert
dT = tsimu(2);

Diff1 = diff(omega_c(1:end,1))/dT;

Y_iden = omega_c(1:end-1, 1);
X_iden = [Vm(1:end-2, 1), Diff1];

A_iden = pinv(X_iden)*Y_iden

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
J_s = 4.1290*10^(-06);  % kg*m^2

%% Calculs
J_c = J_eq - ((K_g^2)*n_g*J_m)

R_m = ((-1*A_iden(2))*K_g*n_g*n_m*k_t)/(A_iden(1)*J_eq)

B_eq = ((K_g*n_g*n_m*k_t)/(A_iden(1)*R_m))-((n_g*k_m*(K_g^2)*n_m*k_t)/R_m)

%% Save les valeurs 
Path = which("Identification.m");
Path = strrep(Path, 'Identification.m', 'Valeurs.mat');
save(Path, "-mat");

%% Graphique

sys = tf([A_iden(1)],[-1*A_iden(2) 1]);

[y,t] = step(sys,tsimu(1:end-1));

% Graphiques
plot(tsimu(1:end-1)-1,omega_c(1:end), 'red')
hold on
plot(t,y, 'black')












