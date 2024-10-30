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
servo = Iden.servo;

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

%Sortir les fonction de transfert
%Fonction de Omega_c/V_m
num = [K_g*n_m*n_g*k_t];
den = [R_m*J_eq  (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2)];
%Fonction de V_x/Omega_C
num1 = [5*g*r_arm];
den1 = [7*L 0];

TF1 = tf(num, den);
TF2 = tf(num1, den1);
%[num, den] = ss2tf(A, B, C, D);
% TF1 = tf(num(1,:), den);
% TF2 = tf(num(2,:), den);

%% Rlocus pour boucle interne
figure
num_int = [K_g*n_m*n_g*k_t];
den_int = [R_m*J_eq  (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2) 0];
rlocus(num_int,den_int)

%Trouver automatiquement le gain
[r,p,k]=residue(num_int,den_int);
Intersection = sum(p)/(length(den_int)-1);
K_int = (((-1)*den_int(1)*(Intersection^2))+((-1)*den_int(2)*(Intersection^1)))/(num_int(1));

FTBF_TF1 = feedback(TF1, K_int);

%% Sortie du Simulink Linéraire
%On a trouver que le meilleur stop time est de 11.258427 pour que ce soit le même nombre de points que la variable servo
Sim_Lin = sim('Modele_Lineaire.slx',"StopTime","10");

% Validation que l'angle des servo sont les mêmes que ceux données par le fichier de donnée
% test1  = ((servo(170)-servo(160))/(tsimu(170)-tsimu(160)))
% test2  = ((Sim_Lin.Theta_C.Data(170)-Sim_Lin.Theta_C.data(160))/(Sim_Lin.Theta_C.Time(170)-Sim_Lin.Theta_C.Time(160)))

%% Sortie du Simulink Non-Linéaire
Sim_Non_Lin = sim('Modele_Non_Lineaire.slx');


