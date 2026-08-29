clc
clear
close all

syms t
syms x(t) theta(t)
syms m M l I g

dx = diff(x,t) ;
dtheta = diff(theta,t);

ddx= diff(x,t,2);
%COM
xCOM = x + l*sin(theta);
yCOM = l*cos(theta);
%Velocity
vx = diff(xCOM,t);
vy = diff(yCOM,t);

v2 = simplify(vx^2 + vy^2);

%Energies
Twheel = (1/2)*M*dx^2;

Tbody = (1/2)*m*v2;

Trot = (1/2)*I*dtheta^2;

T = simplify(Twheel + Tbody + Trot); %TKE

V = m*g*yCOM; %PE

L = simplify(T - V); % Lagrange
pretty(L)