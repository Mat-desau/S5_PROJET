





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
[num, den] = ss2tf(A, B, C, D);
% TF1 = tf(num(1,:), den);
TF2 = tf(num(2,:), den)
rlocus(TF2)

%%
num = [0, 0, K_g*n_m*n_g*k_t];
den = [R_m*J_eq, (R_m*B_eq+n_g*n_m*k_m*k_t*K_g.^2), 0];
[r,p,k]=residue(num,den);

Kint = 0.423;

theta_Vm = tf(num,den);

FTBF_theta_Vm = feedback(theta_Vm,Kint);

rlocus(num,den);

num1 = [5*g*r_arm];
den1 = [7*L,0,0];

X_theta = tf(num1,den1);
TFfinal = series(FTBF_theta_Vm, X_theta)

K=20;
Td =1;
numpd = K*[Td 0];
denpd = [1];
syspd = tf(numpd,denpd);
TFfinalpd = series(TFfinal, syspd);
figure
rlocus(TFfinalpd)
FTBFfinalpd = feedback(TFfinalpd,1)
figure
step(FTBFfinalpd)
figure
stepinfo(FTBFfinalpd)
%% Test résidue
numfinal = [0.4161];
denfinal = [6.918e-05 0.01078 0.4207 0 0];
[r,p,k]=residue(numfinal,denfinal);

poids = abs(r)./abs(real(p));

Tfres = tf([r(3)],[1 -p(3)]) + tf([r(4)],[1 -p(4)])

%[numres, denres] = residue(r(3:4),p(3:4), k)

%Tfres = tf(numres,denres)

systemp = Tfres*syspd
rlocus(systemp)

