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
L_poutre = 0.4254; % m
r_arm = 0.0254; % m

%Plaque
L_plaque = 0.2750; %m

%% Calcul des valeurs manquante
%Aller chercher les valeurs manquante dans l'autre fichier (R_m, B_eq, J_c, tsimu, Vm)
%Trouver le path
Path = which("Code.m");
Path = strrep(Path, '/Simulink/Code.m', '/Identification Moteur/Valeurs.mat');
Path = strrep(Path, '\Simulink\Code.m', '\Identification Moteur\Valeurs.mat');

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

%Pour simulation Simulink
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
     0 0   ((5*g*r_arm)/(7*L_poutre))                              0;
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
%Fonction de base Theta_c/V_m (SI-1)
num_mot = [K_g*n_m*n_g*k_t];
den_mot = [R_m*J_eq  (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2) 0];
TF_mot = tf(num_mot, den_mot);
%Fonction de Omega_c/V_m
num_mot_simu = [K_g*n_m*n_g*k_t];
den_mot_simu = [R_m*J_eq  (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2)];

%Fonction de base X/theta_c
num_bille = [5*g*r_arm];
den_bille = [7*L_poutre 0 0];
TF_bille = tf(num_bille, den_bille);
%Fonction de V_x/theta_C
num_bille_simu = [5*g*r_arm];
den_bille_simu = [7*L_poutre 0];


%Multiplie ensemble (SM-7)
% TF_temp = TF_mot * TF_bille
% [p, z] = pzmap(TF_temp)
% figure
% rlocus(TF_temp, "blue")
% xlim([-100, 80])

%% Boucle interne trouver les K avec Rlocus
%Trouver automatiquement le gain (SI-3)
[~,p,~]=residue(num_mot,den_mot);
Intersection = sum(p)/(length(den_mot)-1);
K_cri = (((-1)*den_mot(1)*(Intersection^2)) + ((-1)*den_mot(2)*(Intersection^1))) / (num_mot(1));

%Trouver pour Zeta = 0.8 (SI-3)
angle_Phi = acos(0.8);
Imaginaire = Intersection*tan(angle_Phi);
Inter = Intersection + Imaginaire*i;

%Calcul de K_int (SI-3)
K_int = real((((-1)*den_mot(1)*(Inter^2)) + ((-1)*den_mot(2)*(Inter^1))) / (num_mot(1)))

%Calcul (SI-2)
% Ts = 4/Intersection
% TF_temp = feedback(K_cri*TF_mot, 1)
% minreal(TF_temp)
% Ts = 4/(38.8/2)

%Graphique (SC-3)
% figure
% hold on
% lsim(TF_mot, Vm(50:250), tsimu(50:250));
% plot(tsimu(50:250), (theta_c(50:250)-theta_c(1)))
% legend(["TF", "\Theta_c"])

%Graphique (SI-1) (SI-2)
% figure
% hold on
% rlocus(num_mot,den_mot, "blue")

%Enlever non-utiliser
clear Imaginaire angle_Phi Intersection Inter p

%% Modèle Simulink Linéraire
% Ouvrir Simulink
Sim_Lin = sim('Modele_Lineaire.slx', "StopTime", "10");

%% Modèle Simulink Non-Linéaire% Ouvrir Simulink
% Ouvrir Simulink
Sim_Non_Lin = sim('Modele_Non_Lineaire.slx', "StopTime", "10");

%% Boucle interne
%Boucle interne
%Test d'asservissement
Test_Angle = pi/4;
Test_Position = 0.06; % m

%Fermer la boucle (theta_c/V_m) avec K_int (SI-3)
FTBF_TF_mot = feedback(K_int*TF_mot, 1);
FTBF_TF_int_ordre4 = FTBF_TF_mot * TF_bille;

%Creation des nouvelles matrices interne
A_int = A;
B_int = B;
C_int = C;
D_int = D;

%Ajouter les modifications
A_int(4,3) = (-1)*(B(4,1))*K_int;
B_int(4,1) = K_int*B(4,1);

%Valeurs propre (SI-7)
% eig(A_int)
% A_int
% B_int
% C_int
% D_int

%Graphique (SI-4)
% figure
% hold on
% rlocus(num_mot,den_mot, "blue")
% r_temp = rlocus(num_mot,den_mot, K_int);
% scatter(real(r_temp(1)),imag(r_temp(1)), 100, "square", "red", "filled")
% scatter(real(r_temp(2)),imag(r_temp(2)), 100, "square", "red", "filled")
% clear r_temp

%Margin de la boucle interne (SI-5)
% figure
% margin(K_int*TF_mot)

%Rlocus (SI-10)
% figure
% rlocus(FTBF_TF_int_ordre4, "blue")

%Trouver les valeurs des fonctions (SI-8)
% [num, den] = ss2tf(A_int, B_int, C_int, D_int)
% roots(den(1,:))
% minreal(tf(num(1,:), den(1,:)))
% minreal(tf(num(2,:), den(1,:)))

%% Boucle Externe BI
%Variables
M_p = 5; % Important ceci est en pourcent
t_s = 4; % sec
t_r = 2; % sec
t_p = 3; % sec

% Ajustement des valeurs pour Bisectrice
Ajout_Omega_a = 0.4; %0.4
Ajout_Omega_n = 0.025; %0.025

%Calculs par bisectrice
Phi = atan((-pi) / log(M_p/100));
Zeta = cos(Phi);
Omega_a = (pi / t_p) - Ajout_Omega_a;
    % Omega_n = Omega_a / sqrt(1-Zeta^2)
    % Omega_n = (1+(1.1*Zeta)+(1.4*Zeta^2)) / t_r
Omega_n = 4 / (Zeta*t_s); % On prends celle-ci car elle a Omega_n le plus haut

%Calcul du des P étoiles pour les poles à faire passer la nouvelle fonction
P_etoile = ((((-1)*Omega_n*Zeta)+Ajout_Omega_n) + Omega_a*i);

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

%Pour trouver le K_a
TF_PD_Bi = tf([1 -Z_temp], [1 -P_temp]);
TF_FTBO_ext_Bi = TF_PD_Bi * FTBF_TF_int_ordre4;
K_a = 1 / abs(evalfr(TF_FTBO_ext_Bi, P_etoile));

%Creation des nouvelles TF
TF_PD_Bi = TF_PD_Bi * K_a;
[TF_PD_Bi_Num, TF_PD_Bi_Den]  = tfdata(TF_PD_Bi, 'v');
TF_FTBO_ext_Bi = TF_FTBO_ext_Bi * K_a;
TF_FTBF_ext_Bi = feedback(TF_FTBO_ext_Bi, 1);

%Validation par margin  (GM > 10 et PM > 45deg)
% figure 
% margin(TF_FTBO_ext_Bi)

%Validation par step info  (Mp = 30% +- 0.1 et t_s = 5 +- 0.1)
% stepinfo(TF_FTBF_ext_Bi)

%Validation par Simulink
%Valider Omega_CD (+- 56°)
%Valider Vm (+- 10)
        
        %Afficher le Rlocus avec les points
        % figure
        % hold on
        % rlocus(TF_FTBO_ext_Bi, 'red');
        % scatter(real(P_etoile),imag(P_etoile), '^', "blue");
        % scatter(real(P_etoile),imag(-P_etoile), '^', "blue");

%Enlever non-utiliser
clear Z_temp P_temp Phi_z Phi_p Alpha K_a Zeta Delta_phi Angle_FTBF_TF_int Omega_n Omega_a Phi P_etoile Ajout_Omega_n Ajout_Omega_a

%% Boucle Externe Bode
%Ajustement
Ajout_BW = 0.170; %0.170
Ajout_PM = -0.1; %-0.1

%Variables
PM_etoile = 45 + Ajout_PM; % deg
BW = 2.3 + Ajout_BW; % rad/s
Echelon_Saturation = 6; % cm

%La nouvelle boucle fermer ordre 4 devient la FTBO externe
FTBO_TF_ext = FTBF_TF_int_ordre4;

%Calculs
Zeta = (1/2)*sqrt(tand(PM_etoile)*sind(PM_etoile));
Omega_g_etoile = BW * ((sqrt(sqrt(1+(4*(Zeta^4)))-(2*(Zeta^2)))) / (sqrt((1-(2*Zeta^2))+sqrt((4*Zeta^4)-(4*Zeta^2)+2))));
K_etoile = 1 / abs(evalfr(FTBO_TF_ext, (Omega_g_etoile*i)));
PM = angle(evalfr(FTBO_TF_ext*K_etoile, (Omega_g_etoile*i))) - pi;
PM2 = rad2deg(PM);
Delta_phi = deg2rad(PM_etoile) - PM;
Alpha = (1 - sin(Delta_phi)) / (1 + sin(Delta_phi));
T = 1 / (Omega_g_etoile * sqrt(Alpha));
K_a = K_etoile / sqrt(Alpha);

%Création des fonctions de transfert
TF_PD_Bode = tf([K_a*Alpha*T K_a*Alpha], [Alpha*T 1]);
[TF_PD_Bode_Num, TF_PD_Bode_Den]  = tfdata(TF_PD_Bode, 'v');
TF_FTBO_ext_Bode = TF_PD_Bode * FTBO_TF_ext;
TF_FTBF_ext_Bode = feedback(TF_FTBO_ext_Bode, 1);

%Validation margin(GM = 12 +- 1 dB et PM = 45 +- 0.1 deg) 
% figure;
% margin(TF_FTBO_ext_Bode);

%Validation Bandwidth (BW > 2.5)
% BW_Cal = bandwidth(TF_FTBF_ext_Bode)

%Validation de StepInfo (t_s = 4.5 +- 0.05 sec)
% figure
% stepinfo(TF_FTBF_ext_Bode)

clear Alpha K_a Zeta Delta_phi PM K_etoile Omega_g_etoile

%% Poutre Sphère


%% Simulink
K_int

X_num = TF_PD_Bi_Num
X_den = TF_PD_Bi_Den
%Ouvrir Simulink
Sim_Asservi1 = sim('Modele_Lineaire_Asservi.slx', "StopTime", "10");

X_num = TF_PD_Bode_Num
X_den = TF_PD_Bode_Den
%Ouvrir Simulink
Sim_Asservi2 = sim('Modele_Lineaire_Asservi.slx', "StopTime", "10");

%Trouver les Max pour les validation par Simulink
% Max_CD_Bi= max(rad2deg(Sim_Asservi1.Theta_Cd.Data))
% Max_V_Bi = max(Sim_Asservi1.Vm_out.Data)
% Max_CD_Bode= max(rad2deg(Sim_Asservi2.Theta_Cd.Data))
% Max_V_Bode = max(Sim_Asservi2.Vm_out.Data)


%% Validation sur modèle non-linéraire Poutre-Sphere
X_num = TF_PD_Bi_Num;
X_den = TF_PD_Bi_Den;
%Ouvrir Simulink
Sim_Non_Lin_Asservi1 = sim("Modele_Non_Lineaire_Asservi.slx", "StopTime", "10");

X_num = TF_PD_Bode_Num;
X_den = TF_PD_Bode_Den;
%Ouvrir Simulink
Sim_Non_Lin_Asservi2 = sim("Modele_Non_Lineaire_Asservi.slx", "StopTime", "10");

%Position résultat (SE-4)
% figure
% hold on
% plot(Sim_Non_Lin_Asservi1.X.Time, Sim_Non_Lin_Asservi1.X.Data, "red");
% Valeur_Demander = (Test_Position*ones(length(Sim_Non_Lin_Asservi1.X.Time),1));
% plot(Sim_Non_Lin_Asservi1.X.Time, Valeur_Demander, "blue");
% grid on
% % t_p
% [Max_Sorti_X, IndexMax] = max(Sim_Non_Lin_Asservi1.X.Data);
% scatter(Sim_Non_Lin_Asservi1.X.Time(IndexMax), Sim_Non_Lin_Asservi1.X.Data(IndexMax), 100, "square", 'black');
% tp = Sim_Non_Lin_Asservi1.X.Time(IndexMax)
% Mp = Sim_Non_Lin_Asservi1.X.Data(IndexMax)
% % t_s
% temp = Sim_Non_Lin_Asservi1.X.Data(IndexMax:end) - (Test_Position+(Test_Position*0.02));
% [~, index_ts] = min(abs(temp));
% scatter(Sim_Non_Lin_Asservi1.X.Time(IndexMax+index_ts), Sim_Non_Lin_Asservi1.X.Data(IndexMax+index_ts), 100, "square", 'black');
% ts = Sim_Non_Lin_Asservi1.X.Time(IndexMax+index_ts)
% title("X Bisectrice")
% legend(["Réponse", "Position Demander"])
% ylabel("Position (m)")
% clear Valeur_Demander Max_Sorti_X IndexMax index_ts
% 
% figure
% hold on
% plot(Sim_Non_Lin_Asservi2.X, "red");
% Valeur_Demander = (Test_Position*ones(length(Sim_Non_Lin_Asservi2.X.Time),1));
% plot(Sim_Non_Lin_Asservi2.X.Time, Valeur_Demander, "blue");
% grid on
% % t_p
% [Max_Sorti_X, IndexMax] = max(Sim_Non_Lin_Asservi2.X.Data);
% scatter(Sim_Non_Lin_Asservi2.X.Time(IndexMax), Sim_Non_Lin_Asservi2.X.Data(IndexMax), 100, "square", 'black');
% tp = Sim_Non_Lin_Asservi2.X.Time(IndexMax)
% Mp = Sim_Non_Lin_Asservi2.X.Data(IndexMax)
% % t_s
% temp = Sim_Non_Lin_Asservi2.X.Data(IndexMax:end) - (Test_Position+(Test_Position*0.02));
% [~, index_ts] = min(abs(temp));
% scatter(Sim_Non_Lin_Asservi2.X.Time(IndexMax+index_ts), Sim_Non_Lin_Asservi2.X.Data(IndexMax+index_ts), 100, "square", 'black');
% ts = Sim_Non_Lin_Asservi2.X.Time(IndexMax+index_ts)
% title("X Bode")
% legend(["Réponse", "Position Demander"])
% ylabel("Position (m)")
% clear Valeur_Demander Max_Sorti_X IndexMax index_ts

            % F1 = figure;
            % F2 = figure;
            % F3 = figure;
            % F4 = figure;
            % 
            % %subplot(4,1,1)
            % figure(F1);
            % hold on
            % plot(Sim_Asservi1.Vm_out)
            % plot(Sim_Asservi2.Vm_out)
            % %title("V_m")
            % %legend(["Bi Linéaire", "Bode Linéaire"])
            % %ylabel("Tension (V)")
            % 
            % %subplot(4,1,2)
            % figure(F2);
            % hold on
            % plot(Sim_Asservi1.X)
            % plot(Sim_Asservi2.X)
            % %title("X")
            % %legend(["Bi Linéaire", "Bode Linéaire"])
            % %ylabel("Position (m)")
            % 
            % %subplot(4,1,3)
            % figure(F3);
            % hold on
            % plot(Sim_Asservi1.Theta_C.Time, rad2deg(Sim_Asservi1.Theta_C.Data))
            % plot(Sim_Asservi2.Theta_C.Time, rad2deg(Sim_Asservi2.Theta_C.Data))
            % %title("\Theta_C")
            % %legend(["Bi Linéaire", "Bode Linéaire"])
            % %ylabel("Angle (deg)")
            % 
            % %subplot(4,1,4)
            % figure(F4);
            % hold on
            % plot(Sim_Asservi1.Theta_Cd.Time, rad2deg(Sim_Asservi1.Theta_Cd.Data))
            % plot(Sim_Asservi2.Theta_Cd.Time, rad2deg(Sim_Asservi2.Theta_Cd.Data))
            % %title("\Theta_C_d")
            % %legend(["Bi Linéaire", "Bode Linéaire"])
            % %ylabel("Angle (deg)")
            % 
            % %subplot(4,1,1)
            % figure(F1);
            % hold on
            % plot(Sim_Non_Lin_Asservi1.Vm_out)
            % plot(Sim_Non_Lin_Asservi2.Vm_out)
            % title("V_m")
            % %legend(["Bi Non-Linéaire", "Bode Non-Linéaire"])
            % legend(["Bi Linéaire", "Bode Linéaire", "Bi Non-Linéaire", "Bode Non-Linéaire"])
            % ylabel("Tension (V)")
            % 
            % %subplot(4,1,2)
            % figure(F2);
            % hold on
            % plot(Sim_Non_Lin_Asservi1.X)
            % plot(Sim_Non_Lin_Asservi2.X)
            % title("X")
            % %legend(["Bi Non-Linéaire", "Bode Non-Linéaire"])
            % legend(["Bi Linéaire", "Bode Linéaire", "Bi Non-Linéaire", "Bode Non-Linéaire"])
            % ylabel("Position (m)")
            % 
            % %subplot(4,1,3)
            % figure(F3);
            % hold on
            % plot(Sim_Non_Lin_Asservi1.Theta_C.Time, rad2deg(Sim_Non_Lin_Asservi1.Theta_C.Data))
            % plot(Sim_Non_Lin_Asservi2.Theta_C.Time, rad2deg(Sim_Non_Lin_Asservi2.Theta_C.Data))
            % title("\Theta_C")
            % %legend(["Bi Non-Linéaire", "Bode Non-Linéaire"])
            % legend(["Bi Linéaire", "Bode Linéaire", "Bi Non-Linéaire", "Bode Non-Linéaire"])
            % ylabel("Angle (deg)")
            % 
            % %subplot(4,1,4)
            % figure(F4);
            % hold on
            % plot(Sim_Non_Lin_Asservi1.Theta_Cd.Time, rad2deg(Sim_Non_Lin_Asservi1.Theta_Cd.Data))
            % plot(Sim_Non_Lin_Asservi2.Theta_Cd.Time, rad2deg(Sim_Non_Lin_Asservi2.Theta_Cd.Data))
            % title("\Theta_C_d")
            % %legend(["Bi Non-Linéaire", "Bode Non-Linéaire"])
            % legend(["Bi Linéaire", "Bode Linéaire", "Bi Non-Linéaire", "Bode Non-Linéaire"])
            % ylabel("Angle (deg)")

%Assurer la fin du document
disp("Hello World")