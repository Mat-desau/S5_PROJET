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

Diviser = 10;

OUT = [0 3 -3 -3  3 0;
       0 3  3 -3 -3 0]';

%Pour les X
for n = 1:length(OUT(:,1))
    for nn = 1:length(OUT(:,1));
        C(nn, n) = (OUT(nn, 1)).^(n-1);
    end
end

%Pour les Y
for n = 1:length(OUT(:,2))
    S(n,1) = OUT(n,2);
end

clear n nn

%Calcul de l'interpolation
A = pinv(C)*S;

Length = length(OUT(:,1));

H = @(x) A(6)*x.^5 +A(5)*x.^4 + A(4)*x.^3 + A(3)*x.^2 + A(2)*x + A(1)

figure
fplot(H, [-10 10])
hold on
%scatter(OUT(:,1), OUT(:,2))

for n = 2:Length
    x = (OUT(n-1,1)):Diviser:(OUT(n,1));
    xout = xout + x;
end

%% Save les valeurs 
Path = which("Trajectoire.m");
Path = strrep(Path, 'Trajectoire.m', 'Trajectoire.mat');
save(Path, "-mat");

disp("Hello World");