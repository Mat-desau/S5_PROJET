%% Load des valeurs
A = load("Valeurs.mat");

x_etoile = A(:,1);
y_etoile = A(:,2);
x_reel = A(:,3);
y_reel = A(:,4);

N_x = length(x_etoile);
N_y = length(y_etoile);

RMS_x = sqrt((1/N_x)*sum((x_etoile - x_reel).^2));
RMS_y = sqrt((1/N_y)*sum((y_etoile - y_reel).^2));