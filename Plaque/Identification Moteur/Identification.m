clc
clear all
close all

%% Data start
load("data_1v_4-09_100hz.mat");
% Vm - entrée (tension)
% servo - angle de moteur
% omega_c - vitesse angulaire
% tsimu - temps de simulation

%On a trouver que omega_c et servo sont en degrée et non en rad
servo = deg2rad(servo);
omega_c = deg2rad(omega_c);

%% Lissage des courbes
% Création du lissage
% P = polyfit(tsimu(1:end-1), omega_c, 22);
% Omega_c = polyval(P, tsimu(1:end-1));

Min = 101;
Max = 104; %104

A = mean(omega_c(Max:end));
% A = 1.6107; % Celui à Mathis
C = [ones(length(tsimu(Min:Max)),1), tsimu(Min:Max)];
Y = real(log((A-omega_c(Min:Max))./(omega_c(Min:Max))));

Mat_A = pinv(C)*Y;
b = (-1)*Mat_A(2);
c = (Mat_A(1)/b);

Omega_c = (A)./(1+(exp((-b)*(tsimu(1:end-1)-(c)))));

% P = fit(tsimu(1:end-1), omega_c, 'logistic')
% Omega_c = (P.a)./(1+(exp((-P.b)*(tsimu(1:end-1)-(P.c)))));

% Plot pour Montrer juste la monter
% figure
% hold on
% plot(tsimu(Min:Max), Omega_c(Min:Max), 'black', 'Marker', 'o')
% plot(tsimu(Min:Max), omega_c(Min:Max), 'red', 'Marker', 'x')
% title("Méthode de lissage");
% xlabel("Temps (s)");
% ylabel("Vitesse (rad/s)");
% legend(["Lisser", "Non-Lisser"]);

% Plot pour toute
% figure
% hold on
% plot(tsimu(1:end-1), Omega_c, 'black')
% plot(tsimu(1:end-1), omega_c, 'red')
% title("Méthode de lissage");
% xlabel("Temps (s)");
% ylabel("Vitesse (rad/s)");
% legend(["Lisser", "Non-Lisser"]);

%% Calcul des coefficient Fonction de transfert
dT = tsimu(2);

Diff1 = diff(Omega_c(1:end,1))/dT;

Y_iden = Omega_c(1:end-1, 1);
X_iden = [Vm(1:end-2, 1), Diff1];

A_iden = pinv(X_iden)*Y_iden
A_iden(2) = A_iden(2)*(1.0)

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
J_s = 7.7028125*10^(-07);  % kg*m^2

%% Calculs
J_c = J_eq - ((K_g^2)*n_g*J_m)

R_m = (((-1)*A_iden(2))*K_g*n_g*n_m*k_t)/(A_iden(1)*J_eq)

B_eq = ((K_g*n_g*n_m*k_t)/(A_iden(1)*R_m))-((n_g*k_m*(K_g^2)*n_m*k_t)/R_m)

%% Graphique
sys = tf([A_iden(1)],[-1*A_iden(2) 1]);

% y = lsim(sys,Vm(1:end-1),tsimu(1:end-1));
% y_Rampe = lsim(sys,Vm(Min:Max),tsimu(Min:Max))

% Graphiques
% figure
% hold on
% plot(tsimu(1:end-1),omega_c(1:end), 'red')
% plot(tsimu(1:end-1), y, 'black')
% title("Comparaison de la fonction de transfert obtenu avec la méthode des moindres carrées et les vrai données");
% xlabel("Temps (s)");
% ylabel("Vitesse (rad/s)");
% legend(["Vrai données", "Fonction de transfert"]);

%% Erreur
%omega_c = signal dentrée bruité
%y = la valeur qui est lisser

% RMSE
RMSE = sqrt((1/length(omega_c))*(sum((Omega_c-omega_c).^2)))

% R^2
Y_ = (1/length(omega_c))*sum(omega_c)
Y__Rampe = (1/length(omega_c(Min:Max)))*sum(omega_c(Min:Max))

R_2_ALL = sum((Omega_c-Y_).^2)/(sum((omega_c-Y_).^2))
R_2_Rampe = sum((Omega_c(Min:Max)-Y__Rampe).^2)/(sum((omega_c(Min:Max)-Y__Rampe).^2))

%% Save les valeurs 
Path = which("Identification.m");
Path = strrep(Path, 'Identification.m', 'Valeurs.mat');
save(Path, "-mat");

%Assurer la fin du document
disp("Hello World")

%Y_mesurer
%Y_Estimer
%R2 obtenu

% Y_mesurer = omega_c(Min:Max)
% Y_estimer = Omega_c(Min:Max)
% R2_trouver = R_2_Rampe