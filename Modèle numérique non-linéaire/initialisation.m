clc
clear all
close all

t = [0:0.1:100]';

%Charge et engrenage

J_c = 1;
B_c = 1;
omega_c = 1;
theta_c = 1;
T_c = 1;
n_g = 1;
K_g = 1;

%Partie électrique du moteur

L_m = 1;
R_m = 1;
I_m = 1;
V_m = [t, ones(size(t))];
e_m = 1;
k_m = 1;
T_m = 1;
n_m = 1;
k_t = 1;

%Partie mécanique du moteur

J_m = 1;
B_m = 1;
omega_m = 1;
T_mc = 1;

%Sphère

m_s = 1;
J_s = 1;
r_s = 1;
F_tra = 1;
F_rot = 1;
T_s = 1;
g = 9.81;

%Poutre
alpha = 1;
L = 1;
h = 1;
r_arm = 1;


