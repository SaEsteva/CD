%% Control Digital en Sistemas Embebidos - MSE - PRACTICA 2

pkg load signal
pkg load control

clc
clear all

%%
% 1) Crear una función en Matlab que aplique la función de identificación por 
% método de cuadrados mínimos (LS) y obtenga el vector de parámetros del modelo 
% a partir del orden deseado del sistema y las señales de entrada/salida.
%%

function [Theta] = identificacionLS(n, u, y)
  Y = y(n+1:length(y));
  Phi = [];
  for i=n:-1:1
    Phi = [Phi y(i:(length(y)+i-n-1))];
  end
  for j=(n+1):-1:1
    Phi = [Phi u(j:(length(y)+j-n-1))];
  end
  Theta = (Phi'*Phi)^(-1)*Phi'*Y;
end

%%
% 2) Crear una función en Matlab que aplique la función de identificación por 
% método de cuadrados mínimos recursivo (RLS) y obtenga el vector de parámetros 
% del modelo a partir del orden deseado del sistema y las señales de 
% entrada/salida.
%%

function [Theta P] = identificacionRLS(n, u, y, lastP, lastTheta)
  Phi = [];
  for i=n:-1:1
    Phi = [Phi y(i)];
  end
  for j=(n+1):-1:1
    Phi = [Phi u(j)];
  end
  P = lastP - lastP*Phi'*Phi*lastP/(1+Phi*lastP*Phi');
  K = lastP*Phi'/(1+Phi*lastP*Phi');
  Theta = lastTheta + K*(y(n+1)-Phi*lastTheta);
end

%%
% 3) Discretizar la planta con un período de muestreo h = 0.1s.
%%

h = 0.1;

num = [ 1 1 ];
den = [ 1 0.5 1];

Hs= tf(num, den)

Hz = c2d(Hs, h, 'zoh')

%%
% 4) Identificar los parámetros de la planta para orden 2 y 3 utilizando la 
% función LS creada anteriormente.
%%

t = 1:h:40;

[numz, denz] = tfdata(Hz, 'v')

u = rand(length(t), 1);
y = filter(numz, denz, u);

[Theta_LS_1] = identificacionLS(1, u, y)
numz_LS_1 = [Theta_LS_1(2) Theta_LS_1(3)]
denz_LS_1 = [1 -Theta_LS_1(1)]

[Theta_LS_2] = identificacionLS(2, u, y)
numz_LS_2 = [Theta_LS_2(3) Theta_LS_2(4) Theta_LS_2(5)]
denz_LS_2 = [1 -Theta_LS_2(1) -Theta_LS_2(2)]

%%
% 5)Obtener el error de ambas identificaciones utilizando la función de costo J.
%%

y_LS_1 = filter(numz_LS_1, denz_LS_1, u);
J_1 = (y-y_LS_1)'*(y-y_LS_1)/2

y_LS_2 = filter(numz_LS_2, denz_LS_2, u);
J_2 = (y-y_LS_2)'*(y-y_LS_2)/2

%%
% 6)Comparar la respuesta de los modelos obtenidos en el punto 4 con la 
% respuesta de la planta real utilizando la misma señal de entrada.
%%

u = rand(length(t), 1);
y = filter(numz, denz, u);
y_LS_1 = filter(numz_LS_1, denz_LS_1, u);
y_LS_2 = filter(numz_LS_2, denz_LS_2, u);

figure;
hold on;
plot(t, u)
plot(t, y, 'LineWidth', 3)
plot(t, y_LS_1, 'LineWidth', 3)
plot(t, y_LS_2, '--', 'LineWidth', 3)
legend('u', 'y', 'y_LS_1', 'y_LS_2')

%%
% 7) Identificar los parámetros de la planta utilizando la función RLS creada
% anteriormente. Obtener y comparar identificaciones con Pinicial = 100.I y
% Pinicial = 1000.I.
%%

N = 300;

Theta_RLS_1 = [0; 0; 0; 0; 0];
P = eye(5)*100;

% u(k-2) u(k-1) u(k) y(k-2) y(k-1) y(k)

for i=3:N
  [Theta_RLS_1 P] = identificacionRLS(2, u(i-2:i), y(i-2:i), P, Theta_RLS_1);
end

numz_RLS_1 = [Theta_RLS_1(3) Theta_RLS_1(4) Theta_RLS_1(5)]
denz_RLS_1 = [1 -Theta_RLS_1(1) -Theta_RLS_1(2)]


Theta_RLS_2 = [0; 0; 0; 0; 0];
P = eye(5)*1000;

% u(k-2) u(k-1) u(k) y(k-2) y(k-1) y(k)

for i=3:N
  [Theta_RLS_2 P] = identificacionRLS(2, u(i-2:i), y(i-2:i), P, Theta_RLS_2);
end

numz_RLS_2 = [Theta_RLS_2(3) Theta_RLS_2(4) Theta_RLS_2(5)]
denz_RLS_2 = [1 -Theta_RLS_2(1) -Theta_RLS_2(2)]

%%
% 8) Comparar la respuesta de los modelos obtenidos en el punto anterior con la
% respuesta de la planta real utilizando la misma señal de entrada.
%%

y_RLS_1 = filter(numz_RLS_1, denz_RLS_1, u);
y_RLS_2 = filter(numz_RLS_2, denz_RLS_2, u);

figure;
hold on;
plot(t, u)
plot(t, y, 'LineWidth', 3)
plot(t, y_RLS_1, 'LineWidth', 3)
plot(t, y_RLS_2, '--', 'LineWidth', 3)
legend('u', 'y', 'y_RLS_1', 'y_RLS_2')

