clc
clear all 
close all

Bode = load("Bode.mat");
Lineaire = load("Lineaire.mat");

%% Position
Min1 = find(Lineaire.tsim == 68);
Max1 = find(Lineaire.tsim == 75);
Min2 = find(Bode.tsim == 23);
Max2 = find(Bode.tsim == 29);

% figure
% hold on
% plot(Lineaire.tsim(Min1:Max1,1), Lineaire.xd_balle(Min1:Max1,1))
% plot(Lineaire.tsim(Min1:Max1,1), Lineaire.x_balle(Min1:Max1,1))
% legend(["Demander" "Obtenu"])
% title("Bisectrice Position")
% ylim([-0.1 0.1])

% figure
% hold on
% plot(Lineaire.tsim(:,1), Lineaire.xd_balle(:,1))
% plot(Lineaire.tsim(:,1), Lineaire.x_balle(:,1))
% legend(["Demander" "Obtenu"])
% title("Bisectrice Position")
% ylim([-0.25 0.1])

Lin_lsim = lsiminfo(Lineaire.x_balle(Min1:Max1,1), Lineaire.tsim(1:(Max1-Min1+1),1))

M_p = (Lin_lsim.Max - Lineaire.x_balle(Max1,1))/Lineaire.x_balle(Max1,1)
Phi = atan((-pi) / log(M_p))
Zeta = cos(Phi)
Omega_n = 4/(Lin_lsim.SettlingTime * Zeta)

PM = atand((2*Zeta)/(sqrt(sqrt(1+(4*Zeta^4))-(2*Zeta^2))))
BW = Omega_n * sqrt((1-(2*Zeta^2))+sqrt((4*Zeta^4)-(4*Zeta^2)+2))

max(Lineaire.Vm(Min1:Max1,1))
max(Lineaire.ServoAngle(Min1:Max1,1))

% figure
% hold on
% plot(Bode.tsim(Min2:Max2,1), Bode.xd_balle(Min2:Max2,1))
% plot(Bode.tsim(Min2:Max2,1), Bode.x_balle(Min2:Max2,1))
% legend(["Demander" "Obtenu"])
% title("Bode Position")
% ylim([-0.05 0.1])

% figure
% hold on
% plot(Bode.tsim(:,1), Bode.xd_balle(:,1))
% plot(Bode.tsim(:,1), Bode.x_balle(:,1))
% legend(["Demander" "Obtenu"])
% title("Bode Position")
% ylim([-0.25 0.1])

Bode_lsim = lsiminfo(Bode.x_balle(Min2:Max2,1), Bode.tsim(1:(Max2-Min2+1),1))

M_p = (Bode_lsim.Max - Bode.x_balle(Max2,1))/Bode.x_balle(Max2,1)
Phi = atan((-pi) / log(M_p))
Zeta = cos(Phi)
Omega_n = 4/(Bode_lsim.SettlingTime * Zeta)

PM = atand((2*Zeta)/(sqrt(sqrt(1+(4*Zeta^4))-(2*Zeta^2))))
BW = Omega_n * sqrt((1-(2*Zeta^2))+sqrt((4*Zeta^4)-(4*Zeta^2)+2))

max(Bode.Vm(Min2:Max2,1))
max(Bode.ServoAngle(Min2:Max2,1))

%% Angle
figure
hold on
plot(Lineaire.tsim, Lineaire.ServoAngle(:,1))
plot(Lineaire.tsim, Lineaire.ServoAngle(:,2))
legend(["Demander" "Obtenu"])
title("Bisectrice Angle")

figure
hold on
plot(Bode.tsim, Bode.ServoAngle(:,1))
plot(Bode.tsim, Bode.ServoAngle(:,2))
legend(["Demander" "Obtenu"])
title("Bode Angle")

%% Tension
figure
hold on
plot(Lineaire.tsim, Lineaire.Vm(:,1))
plot(Lineaire.data_vm(:,1), Lineaire.data_vm(:,2))
legend(["Demander" "Obtenu"])
title("Bisectrice Tension")

figure
hold on
plot(Bode.tsim, Bode.Vm(:,1))
plot(Bode.data_vm(:,1), Bode.data_vm(:,2))
legend(["Demander" "Obtenu"])
title("Bode Tension")