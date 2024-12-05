clc
clear all
close all

BodeX = load("sinusxbode.mat");
BodeY = load("sinusybode.mat");
TempX = load("sinusxtemp.mat");
TempY = load("sinusytemp.mat");

%% Position
figure
title("Bode position sinus en X");
hold on
grid on
plot(BodeX.data_vm_y.time, BodeX.Xpos(:,1))
plot(BodeX.data_vm_y.time, BodeX.Xpos(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["Désirer", "Reel"]);


figure
title("Bode position sinus en Y");
hold on
grid on
plot(BodeY.data_vm_y.time, BodeY.Ypos(:,1))
plot(BodeY.data_vm_y.time, BodeY.Ypos(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["Désirer", "Reel"]);

figure
title("Temporel position sinus en X");
hold on
grid on
plot(TempX.data_vm_y.time, TempX.Xpos(:,1))
plot(TempX.data_vm_y.time, TempX.Xpos(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["Désirer", "Reel"]);

figure
title("Temporel position sinus en Y");
hold on
grid on
plot(TempY.data_vm_y.time, TempY.Ypos(:,1))
plot(TempY.data_vm_y.time, TempY.Ypos(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["Désirer", "Reel"]);


%% Angle

figure
title("Bode angle sinus en X");
hold on
grid on
plot(BodeX.data_vm_y.time, BodeX.ServoAngles(:,1))
plot(BodeX.data_vm_y.time, BodeX.ServoAngles(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

figure
title("Bode angle sinus en Y");
hold on
grid on
plot(BodeY.data_vm_y.time, BodeY.ServoAngles(:,1))
plot(BodeY.data_vm_y.time, BodeY.ServoAngles(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

figure
title("Temporel angle sinus en X");
hold on
grid on
plot(TempX.data_vm_y.time, TempX.ServoAngles(:,1))
plot(TempX.data_vm_y.time, TempX.ServoAngles(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

figure
title("Temporel angle sinus en Y");
hold on
grid on
plot(TempY.data_vm_y.time, TempY.ServoAngles(:,1))
plot(TempY.data_vm_y.time, TempY.ServoAngles(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

%% Tension

figure
title("Bode tension sinus en X");
hold on
grid on
plot(BodeX.data_vm_y.time, BodeX.MotorVoltage(:,1))
plot(BodeX.data_vm_y.time, BodeX.MotorVoltage(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

figure
title("Bode tension sinus en Y");
hold on
grid on
plot(BodeY.data_vm_y.time, BodeY.MotorVoltage(:,1))
plot(BodeY.data_vm_y.time, BodeY.MotorVoltage(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

figure
title("Temporel tension sinus en X");
hold on
grid on
plot(TempX.data_vm_y.time, TempX.MotorVoltage(:,1))
plot(TempX.data_vm_y.time, TempX.MotorVoltage(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

figure
title("Temporel tension sinus en Y");
hold on
grid on
plot(TempY.data_vm_y.time, TempY.MotorVoltage(:,1))
plot(TempY.data_vm_y.time, TempY.MotorVoltage(:,2))
xlabel("Temps (sec)");
ylabel("Distance (m)");
legend(["X", "Y"]);

close all