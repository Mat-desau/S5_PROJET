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

clear Iden Variables Path

% Pour simulation Simulink
% tsimu2 = [0:0.01:3]';
% Vm2 = zeros(length(tsimu2),1);
% Vm2((tsimu2 >= 1) & (tsimu2 <= 3)) = 1;
% 
% tsimu3 = [0:0.01:10]';
% Vm3 = zeros(length(tsimu3),1);
% Vm3((tsimu3 >= 1) & (tsimu3 <= 1.2)) = 1;


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
K_cri = (((-1)*den_mot(1)*(Intersection^2)) + ((-1)*den_mot(2)*(Intersection^1))) / (num_mot(1));

%Trouver pour zeta = 0.8
angle_Phi = acos(0.8);
Imaginaire = Intersection*tan(angle_Phi);
Inter = Intersection + Imaginaire*i;

K_int = real((((-1)*den_mot(1)*(Inter^2)) + ((-1)*den_mot(2)*(Inter^1))) / (num_mot(1)));

%Enlever non-utiliser
clear Imaginaire angle_Phi Intersection Inter p

%% SC-3
% figure
% hold on
% lsim(TF3, Vm, tsimu);
% % On sort theta_c
% plot(tsimu, (theta_c-theta_c(1)))
% legend(["TF", "\Theta_c"])

%% Modèle Simulink Linéraire
% Ouvrir Simulink
Sim_Lin = sim('Modele_Lineaire.slx');

%% Modèle Simulink Non-Linéaire% Ouvrir Simulink
% Ouvrir Simulink
Sim_Non_Lin = sim('Modele_Non_Lineaire.slx');

%% Modèle Simulink asservi
clc
% Boucle interne
%Test d'asservissement
Test_Angle = pi/4;

%Fermer la boucle (theta_c/V_m) avec K_int
FTBF_TF_mot = feedback(K_int*TF_mot, 1);
FTBF_TF_int_ordre4 = FTBF_TF_mot * TF_bille;

%On doit diminuer à une ordre 2
[num_temp, den_temp] = tfdata(FTBF_TF_int_ordre4, 'v');
[r, p, k] = residue(num_temp, den_temp);
poids = abs(r)./abs(real(p));
FTBF_TF_int = tf([r(3)], [1 -p(3)]) + tf([r(4)], [1 -p(4)]);
clear num_temp den_temp r p k poids

%[num_FTBF_mot, den_FTBF_mot] = tfdata(FTBF_TF_mot, 'v');
%[A,B,C,D] = tf2ss(FTBF_TF_mot)

% Boucle Externe
%Variables
M_p = 5; % Important ceci est en pourcent
t_s = 4; % s
t_r = 2; % s
t_p = 3; % s

%Calculs
Phi = atan((-pi) / log(M_p/100));
Zeta = cos(Phi);
Omega_a = pi / t_p;
Omega_n = Omega_a / sqrt(1-Zeta^2);
%Omega_n = 4 / (Zeta*t_s);
P_etoile = (Omega_n*Zeta)*(-1) + Omega_a*i; 
    % frsp = evalfr(FTBF_TF_int, P_etoile)
    % 
    % Angle_FTBF_TF_int = rad2deg(angle(frsp))
    % Mag_FTBF_TF_int = abs(frsp)
    % clear frsp

%Calcul de l'avance de phase comme dans le document
Delta_phi = 0;
Alpha = pi - Phi;
Phi_z = (Alpha + Delta_phi) / 2;
Phi_p = (Alpha - Delta_phi) / 2;
Z_temp = real(P_etoile) - (imag(P_etoile) / tan(Phi_z));
P_temp = real(P_etoile) - (imag(P_etoile) / tan(Phi_p));
%Pour faire l'étape finale
TF_PD = tf([1 -Z_temp], [1 -P_temp]);
TF_FTBO_ext = TF_PD * FTBF_TF_int;
K_a = 1 / abs(evalfr(TF_FTBO_ext, P_etoile));

%Creation des nouvelles TF
TF_PD = TF_PD * K_a;
TF_FTBO_ext = TF_FTBO_ext * K_a;
TF_FTBF_ext = feedback(TF_FTBO_ext, 1);

%Enlever non-utiliser
clear Z_temp P_temp Phi_z Phi_p Alpha Delta_phi

% figure
% step(TF_FTBF_ext)
% stepinfo(TF_FTBF_ext)
% 
% figure
% rlocus(TF_FTBO_ext, 'red')

% Ouvrir Simulink
Sim_Asservi = sim('Modele_Lineaire_Asservi.slx');

disp("Hello World")
