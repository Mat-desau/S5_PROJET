clc
close all
clear all

%% Information
% point 1 = (0,0)
% point 2 = (3,3)
% point 3 = (-3,3)
% point 4 = (-3,-3)
% point 5 = (3,-3)
% point 6 = (0,0)

%Variables
%Un huit en trajectoire
% OUT = [0.0/100  3.0/100   5.0/100   3.0/100  -3.0/100  -5.0/100  -3.0/100   0.0/100;
%        0.0/100  3.0/100   0.0/100  -3.0/100   3.0/100   0.0/100  -3.0/100   0.0/100]';

OUT = [0.0  3.0   0.0  -3.0  -3.0  -3.0   3.0   0.0;
       0.0  3.0   3.0   3.0   0.0  -3.0  -3.0   0.0]';

OUT = OUT./100;

Temps = linspace(1, 8, length(OUT(:,1)));
Position = 1:0.02:8;

%Pour les X
for n = 1:length(OUT(:,1))
    for nn = 1:length(OUT(:,1));
        C(nn, n) = (Temps(nn)).^(n-1);
    end
end

%Pour les Y
for n = 1:length(OUT(:,2))
    S(n,1) = OUT(n,2);
    S2(n,1) = OUT(n,1);
end

clear n nn

%Calcul de l'interpolation
A1 = pinv(C)*S;
A2 = pinv(C)*S2;

H1 = A1(8).*Position.^7 + A1(7).*Position.^6 + A1(6).*Position.^5 + A1(5).*Position.^4 + A1(4).*Position.^3 + A1(3).*Position.^2 + A1(2).*Position + A1(1);
H2 = A2(8).*Position.^7 + A2(7).*Position.^6 + A2(6).*Position.^5 + A2(5).*Position.^4 + A2(4).*Position.^3 + A2(3).*Position.^2 + A2(2).*Position + A2(1);

%Pour la longueur
dH1 = 7*A1(8).*Position.^6 + 6*A1(7).*Position.^5 + 5*A1(6).*Position.^4 + 4*A1(5).*Position.^3 + 3*A1(4).*Position.^2 + 2*A1(3).*Position + A1(2).*Position;
dH2 = 7*A2(8).*Position.^6 + 6*A2(7).*Position.^5 + 5*A2(6).*Position.^4 + 4*A2(5).*Position.^3 + 3*A2(4).*Position.^2 + 2*A2(3).*Position + A2(2).*Position;

H1 = H1(:);
H2 = H2(:);

%Ajustement pour le maximum
H1(H1 <= -0.08) = -0.08;
H1(H1 >=  0.08) =  0.08;
H2(H2 <= -0.08) = -0.08;
H2(H2 >=  0.08) =  0.08;

figure
hold on
plot(H2, H1, "red")
scatter(OUT(:,1), OUT(:,2), "blue", 'filled')
xlim([-0.04 0.08])
ylim([-0.05 0.05])
xlabel("Distance (m)")
ylabel("Distance (m)")
title("Trajectoire")
legend(["Trajectoire", "Points désirer"])

%% Save les valeurs 
Path = which("Trajectoire.m");
Path = strrep(Path, 'Trajectoire.m', 'Trajectoire.mat');
save(Path, "-mat");


%% Autre
% Si on veut juste de point vers point
% Diviser = 10;

% for n = 2:length(OUT(:,1))
%     if OUT(n,1) == OUT(n-1,1)
%         X = OUT(n,1)*ones(1, Diviser);
%     elseif OUT(n,1) < OUT(n-1,1)
%         X = linspace(OUT(n-1,1),OUT(n,1),Diviser);
%     elseif OUT(n,1) > OUT(n-1,1)
%         X = linspace(OUT(n-1,1),OUT(n,1), Diviser);
%     end
% 
%     if OUT(n,2) == OUT(n-1,2)
%         Y = OUT(n,2)*ones(1, Diviser);
%     elseif OUT(n,2) < OUT(n-1,2)
%         Y = linspace(OUT(n-1,2),OUT(n,2),Diviser);
%     elseif OUT(n,2) > OUT(n-1,2)
%         Y = linspace(OUT(n-1,2),OUT(n,1), Diviser);
%     end
% 
%     if n == 2
%         Xout = X;
%         Yout = Y;
%     else
%         Xout = [Xout X];
%         Yout = [Yout Y];
%     end
% end
% figure
% scatter(Xout, Yout)
% xlim([-4 4])
% ylim([-4 4])


disp("Hello World");