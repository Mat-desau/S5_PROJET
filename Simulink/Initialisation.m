clc
clear all
close all
warning off

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

%Test d'asservissement
Test_Angle = pi/4;

%% Calcul des valeurs manquante
%Aller chercher les valeurs manquante dans l'autre fichier (R_m, B_eq, J_c, tsimu, Vm)
%Trouver le path
Path = which("Initialisation.m");
Path = strrep(Path, '/Simulink/Initialisation.m', '/Identification Moteur/Valeurs.mat');
Path = strrep(Path, '\Simulink\Initialisation.m', '\Identification Moteur\Valeurs.mat');

%Ce qu'on veut sortir
Variables = {"B_eq","J_c", "R_m", "tsimu", "Vm", "servo"};

%Sortir en structure
Iden = load(Path, Variables{:});

%Remplacer les structure par leur variables chercher
R_m = Iden.R_m;
J_c = Iden.J_c;
B_eq = Iden.B_eq;
tsimu = Iden.tsimu;
Vm = Iden.Vm;
theta_c = Iden.servo;

% Pour simulation Simulink
tsimu2 = [0:0.01:3]';
Vm2 = zeros(length(tsimu2),1);
Vm2((tsimu2 >= 1) & (tsimu2 <= 3)) = 1;

tsimu3 = [0:0.01:10]';
Vm3 = zeros(length(tsimu3),1);
Vm3((tsimu3 >= 1) & (tsimu3 <= 1.2)) = 1;


%% Matrices A,B,C,D
%On resort nos matrices A,B,C,D en fonction de J_eq et de B_eq
A = [0 1            0                                       0;
     0 0   ((5*g*r_arm)/(7*L))                              0;
     0 0            0                                       1;
     0 0            0             (((-n_m*n_g*k_t*(K_g^2)*k_m)-(R_m*B_eq))/(R_m*J_eq))];

B = [               0;
                    0;
                    0;
     ((n_m*k_t*n_g*K_g)/(R_m*J_eq))];

C = [1 0 0 0;
     0 0 1 0];

D = [0;
     0];

%% Sortir les fonction de transfert
%Fonction de base Theta_c/V_m
num_mot = [K_g*n_m*n_g*k_t];
den_mot = [R_m*J_eq  (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2) 0];
TF_mot = tf(num_mot, den_mot);
%Fonction de Omega_c/V_m
num_mot_simu = [K_g*n_m*n_g*k_t];
den_mot_simu = [R_m*J_eq  (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2)];

%Fonction de base X/theta_c
num_bille = [5*g*r_arm];
den_bille = [7*L 0 0];
TF_bille = tf(num_bille, den_bille);
%Fonction de V_x/theta_C
num_bille_simu = [5*g*r_arm];
den_bille_simu = [7*L 0];

%% Rlocus pour boucle interne
% Graphique
% figure
% rlocus(num_mot,den_mot)

%Trouver automatiquement le gain
[~,p,~]=residue(num_mot,den_mot);
Intersection = sum(p)/(length(den_mot)-1);
K_cri = (((-1)*den_mot(1)*(Intersection^2))+((-1)*den_mot(2)*(Intersection^1)))/(num_mot(1));

%Trouver pour zeta = 0.8
angle_Phi = acos(0.8);
Imaginaire = Intersection*tan(angle_Phi);
Inter = Intersection + Imaginaire*i;

K_int = real((((-1)*den_mot(1)*(Inter^2))+((-1)*den_mot(2)*(Inter^1)))/(num_mot(1)));


%% SC-3
% figure
% hold on
% lsim(TF3, Vm, tsimu);
% % On sort theta_c
% plot(tsimu, (theta_c-theta_c(1)))
% legend(["TF", "\Theta_c"])

%% Modèle Simulink Linéraire
%On a trouver que le meilleur stop time est de 11.258427 pour que ce soit le même nombre de points que la variable servo
Sim_Lin = sim('Modele_Lineaire.slx', 'stoptime', '3');

%% Modèle Simulink Non-Linéaire
Sim_Non_Lin = sim('Modele_Non_Lineaire.slx', 'stoptime', '3');

%% Modèle Simulink asservi
%Fermer la boucle (theta_c/V_m) avec K_int
FTBF_TF_mot = feedback(K_int*TF_mot, 1);
[num_FTBF_mot, den_FTBF_mot] = tfdata(FTBF_TF_mot, 'v');

%[A,B,C,D] = tf2ss(FTBF_TF_mot)

disp("Hello World")


