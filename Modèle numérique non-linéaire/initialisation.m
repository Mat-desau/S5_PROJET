clc
clear all
close all

t = [0:0.1:100]';

%% Mise en valeur
%Charge et engrenage
n_g = 0.9000;
K_g = 70;

    %Non-Connu
    %J_c = 1;
    %B_c = 1;
    omega_c = 1;
    theta_c = 1;
    T_c = 1;

%Partie électrique du moteur
V_m = [t, ones(size(t))];
k_m = 0.0076776;    % Nm/A
n_m = 0.69;
k_t = 0.0076830;    %V/(Rad/s)

    %Non-Connu
    %R_m = 1;    % À être identifier
    T_m = 1;
    e_m = 1;
    L_m = 1;
    I_m = 1;

%Partie mécanique du moteur
J_m = 3.9001*10^(-07);  %kg*m^2
J_eq = 0.0017728;   %kg*m^2
  
    %Non-Connu
    %B_eq = 1;   % À être identifier
    B_m = 1;
    omega_m = 1;
    T_mc = 1;

%Sphère
m_s = 0.0640;   % kg
J_s = 4.1290*10^(-06);  % kg*m^2
r_s = 0.0137;   % m
g = 9.81;   % m/s^2

    %Non-Connu
    F_tra = 1;
    F_rot = 1;
    T_s = 1;

%Poutre

L = 0.4254; % m
r_arm = 0.0254; % m
    
    %Non-Connu
    h = 1;
    alpha = 1;

%% Calcul des valeurs manquante
% On sait que J_m est à la sortie du moteur et J_c est à la suite des engrenages
J_c = J_m*K_g;

B_c = 1;
R_m = 1;
B_eq = ;

%% Matrices
A = [                   0                          1                0                                              0;
                        0                          0     ((5*g*r_arm)/(7*L))                                       0;
                        0                          0                0                                              1;
     ((m_s*g*r_arm)/((J_m*(K_g^2)*n_g*L)+(J_c*L))) 0                0               (((-R_m*B_m*(K_g^2)*n_g)-(R_m*B_c)+(n_m*k_t*k_m*(K_g^2)*n_g))/((R_m*J_m*(K_g^2)*n_g)+(R_m*J_c)))]


B = [ 0;
      0;
      0;
      -((n_m*k_t*n_g*K_g)/((R_m*J_m*(K_g^2)*n_g)+(R_m*J_c)))]

C = [1 0 0 0;
     0 0 1 0]

D = [0;
     0]