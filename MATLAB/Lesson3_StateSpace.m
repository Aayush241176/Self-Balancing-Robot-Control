clc
clear
close all

%% Robot Parameters

m = 0.34;        % Body mass (kg)
M = 0.09;        % Wheel assembly mass (kg)
l = 0.031;       % COM height from axle (m)
g = 9.81;        % Gravity (m/s^2)
I = 0.000953;    %Moment of Inertia about axle
D = M*m*l^2 + M*I + m*I;

a = -(g*m^2*l^2)/D;

b = (m*l^2 + I)/D;

c = (g*l*m*(M+m))/D;

d = -(l*m)/D;
% Matrix A B C D
A = [ 0 1 0 0;
    0 0 a 0;
    0 0 0 1;
    0 0 c 0 ] ;

B = [0;
    b;
    0;
    d];

C = [1 0 0 0;
    0 0 1 0];

Dmat = [0;
    0];

sys = ss(A,B,C,Dmat);

Co = ctrb(A,B);

rank(Co)

Ob = obsv(A,C);

rank(Ob)