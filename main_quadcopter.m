clc; clear; close all;

syms x y z phi theta psi d_x d_y d_z d_phi d_theta d_psi w1 w2 w3 w4

% Parameters
Ixx = 0.01;
Iyy = Ixx;
Izz = 0.05;

k = 1;      % cosntante de empuje  
b = 1;      % constante de arrastre
l = 0.5;    % distancia entre el motor el centro de masa del quadcopter.
JR = 0.2;   % momento de inercia del motor.
m = 1.8;    % masa del quadcopter
g = 9.8;    % constante de gravedad
 

% Calculating inputs
f1 = k*w1^2;
f2 = k*w2^2;
f3 = k*w3^2;
f4 = k*w4^2;

F = (f1+f2+f3+f4);

t1 = b*w1^2;
t2 = b*w2^2;
t3 = b*w3^2;
t4 = b*w4^2;

t_phi = l*k*(-w2^2+w4^2);
t_theta = l*k*(-w1^2+w3^2);
t_psi = (t1+t2+t3+t4);

w_r = w1-w2+w3-w4;

% R = [ 
%     cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(phi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
%     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(phi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
%     -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)
%     ];

% I = diag([Ixx, Iyy, Izz]);

Eq1 = d_x;
Eq2 = d_y;
Eq3 = d_z;

Eq4 = d_phi;
Eq5 = d_theta;
Eq6 = d_psi;

Eq7 = (F/m)*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
Eq8 = (F/m)*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
Eq9 = (F/m)*(cos(phi)*cos(theta)) - g;

Eq10 = (t_phi/Ixx) + (JR*w_r*d_theta/Ixx) + (d_theta*d_psi*(Iyy - Izz)/Ixx);
Eq11 = (t_theta/Iyy) + (JR*w_r*d_phi/Iyy) + (d_theta*d_psi*(Izz - Ixx)/Iyy);
Eq12 = (t_psi/Izz) + (d_phi*d_theta*(Ixx - Iyy)/Izz);

F_PLANT = [Eq1; Eq2; Eq3; Eq4; Eq5; Eq6; Eq7; Eq8; Eq9; Eq10; Eq11; Eq12];
X = [ x y z phi theta psi d_x d_y d_z d_phi d_theta d_psi ];
U = [ w1 w2 w3 w4 ];

%% calculando los puntos de equilibrio

w1 = 0.5*sqrt(m*g/k);
w2 = 0.5*sqrt(m*g/k);
w3 = 0.5*sqrt(m*g/k);
w4 = 0.5*sqrt(m*g/k);

F_equi = [0;0;0;0;0;0;0;0;0;0;0;0] == eval(F_PLANT);
S = solve(F_equi, X);

% A_J = jacobian(F_PLANT, X)
% B_J = jacobian(F_PLANT, U)