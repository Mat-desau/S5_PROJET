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
% Boucle interne
%Test d'asservissement
Test_Angle = pi/4;
Test_Position = 0.06; % m

%Fermer la boucle (theta_c/V_m) avec K_int
FTBF_TF_mot = feedback(K_int*TF_mot, 1);
FTBF_TF_int_ordre4 = FTBF_TF_mot * TF_bille;


% Boucle Externe
%Variables
M_p = 5; % Important ceci est en pourcent
t_s = 4; % sec
t_r = 2; % sec
t_p = 3; % sec

% Ajustement des valeurs pour Bisectrice
Ajout_Omega_a = 0.4;
Ajout_P_Etoile = 0.025;

%Calculs par bisectrice
Phi = atan((-pi) / log(M_p/100));
Zeta = cos(Phi);
Omega_a = (pi / t_p) - Ajout_Omega_a;
    % Omega_n = Omega_a / sqrt(1-Zeta^2)
    % Omega_n = (1+(1.1*Zeta)+(1.4*Zeta^2)) / t_r
Omega_n = 4 / (Zeta*t_s); % On prends celle-ci car elle a Omega_n le plus haut

% Calcul du des P étoiles pour les poles à faire passer la nouvelle fonction
P_etoile = ((((-1)*Omega_n*Zeta)+Ajout_P_Etoile) + Omega_a*i);

%Trouver notre angle pour P_etoile
frsp = evalfr(FTBF_TF_int_ordre4, P_etoile);
Angle_FTBF_TF_int = rad2deg(angle(frsp)) - 360;
clear frsp 

%Calcul de l'avance de phase comme dans le document
Delta_phi = deg2rad(-180 - Angle_FTBF_TF_int);
Alpha = pi - Phi;
Phi_z = ((Alpha + Delta_phi) / 2);
Phi_p = ((Alpha - Delta_phi) / 2);
Z_temp = real(P_etoile) - (imag(P_etoile) / tan(Phi_z));
P_temp = real(P_etoile) - (imag(P_etoile) / tan(Phi_p));

% Pour trouver le K_a
TF_PD_Bi = tf([1 -Z_temp], [1 -P_temp]);
TF_FTBO_ext_Bi = TF_PD_Bi * FTBF_TF_int_ordre4;
K_a = 1 / abs(evalfr(TF_FTBO_ext_Bi, P_etoile));

%Creation des nouvelles TF
TF_PD_Bi = TF_PD_Bi * K_a;
[TF_PD_Bi_Num, TF_PD_Bi_Den]  = tfdata(TF_PD_Bi, 'v');
TF_FTBO_ext_Bi = TF_FTBO_ext_Bi * K_a;
TF_FTBF_ext_Bi = feedback(TF_FTBO_ext_Bi, 1);

% Validation par margin  (GM > 10 et PM > 45deg)
% figure 
% margin(TF_FTBO_ext_Bi)

% Validation par step info  (Mp = 30% +- 0.1 et t_s = 5 +- 0.1)
% stepinfo(TF_FTBF_ext_Bi)

% Validation par Simulink
% Valider Omega_CD (+- 56°)
% Valider Vm (+- 10)
        
        % Afficher le Rlocus avec les points
        % figure
        % hold on
        % rlocus(TF_FTBO_ext_Bi, 'red');
        % scatter(real(P_etoile),imag(P_etoile), '^', "blue");
        % scatter(real(P_etoile),imag(-P_etoile), '^', "blue");

%Enlever non-utiliser
clear Z_temp P_temp Phi_z Phi_p Alpha K_a Zeta Delta_phi Angle_FTBF_TF_int Omega_n Omega_a Phi P_etoile Ajout_P_Etoile Ajout_Omega_a

% Calcul par Bode
% Ajustement
Ajout_BW = 0.170;
Ajout_PM = -0.1;

% Variables
PM_etoile = 45 + Ajout_PM; % deg
BW = 2.3 + Ajout_BW; % rad/s
Echelon_Saturation = 6; % cm

% La nouvelle boucle fermer ordre 4 devient la FTBO externe
FTBO_TF_ext = FTBF_TF_int_ordre4;

% Calculs
Zeta = (1/2)*sqrt(tand(PM_etoile)*sind(PM_etoile));
Omega_g_etoile = BW * ((sqrt(sqrt(1+(4*(Zeta^4)))-(2*(Zeta^2)))) / (sqrt((1-(2*Zeta^2))+sqrt((4*Zeta^4)-(4*Zeta^2)+2))))
K_etoile = 1 / abs(evalfr(FTBO_TF_ext, (Omega_g_etoile*i)));
PM = angle(evalfr(FTBO_TF_ext*K_etoile, (Omega_g_etoile*i))) - pi;
Delta_phi = deg2rad(PM_etoile) - PM;
Alpha = (1 - sin(Delta_phi)) / (1 + sin(Delta_phi));
T = 1 / (Omega_g_etoile * sqrt(Alpha));
K_a = K_etoile / sqrt(Alpha);

% Création des fonctions de transfert
TF_PD_Bode = tf([K_a*Alpha*T K_a*Alpha], [Alpha*T 1]);
[TF_PD_Bode_Num, TF_PD_Bode_Den]  = tfdata(TF_PD_Bode, 'v');
TF_FTBO_ext_Bode = TF_PD_Bode * FTBO_TF_ext;
TF_FTBF_ext_Bode = feedback(TF_FTBO_ext_Bode, 1);

% Validation margin(GM = 12 +- 1 dB et PM = 45 +- 0.1 deg) 
figure;
margin(TF_FTBO_ext_Bode);

% Validation Bandwidth (BW > 2.5)
% BW_Cal = bandwidth(TF_FTBF_ext_Bode)

% Validation de StepInfo (t_s = 4.5 +- 0.05 sec)
stepinfo(TF_FTBF_ext_Bode)

clear Alpha K_a Zeta Delta_phi PM K_etoile Omega_g_etoile

X_num = TF_PD_Bi_Num;
X_den = TF_PD_Bi_Den;
% Ouvrir Simulink
Sim_Asservi1 = sim('Modele_Lineaire_Asservi.slx');

X_num = TF_PD_Bode_Num;
X_den = TF_PD_Bode_Den;
% Ouvrir Simulink
Sim_Asservi2 = sim('Modele_Lineaire_Asservi.slx');

Max_CD= max(rad2deg(Sim_Asservi2.Theta_Cd.Data))
Max_V = max(Sim_Asservi2.Vm_out.Data)

% figure
% sgtitle("Graphiques des asservissements")
% subplot(4,1,1)
% hold on
% plot(Sim_Asservi1.Vm_out)
% plot(Sim_Asservi2.Vm_out)
% title("V_m")
% legend(["Bi", "Bode"])
% ylabel("Tension (V)")
% subplot(4,1,2)
% hold on
% plot(Sim_Asservi1.X)
% plot(Sim_Asservi2.X)
% title("X")
% legend(["Bi", "Bode"])
% ylabel("Position (m)")
% subplot(4,1,3)
% hold on
% plot(Sim_Asservi1.Theta_C.Time, rad2deg(Sim_Asservi1.Theta_C.Data))
% plot(Sim_Asservi2.Theta_C.Time, rad2deg(Sim_Asservi2.Theta_C.Data))
% title("\Theta_C")
% legend(["Bi", "Bode"])
% ylabel("Angle (deg)")
% subplot(4,1,4)
% hold on
% plot(Sim_Asservi1.Theta_Cd.Time, rad2deg(Sim_Asservi1.Theta_Cd.Data))
% plot(Sim_Asservi2.Theta_Cd.Time, rad2deg(Sim_Asservi2.Theta_Cd.Data))
% title("\Theta_C_d")
% legend(["Bi", "Bode"])
% ylabel("Angle (deg)")

disp("Hello World")