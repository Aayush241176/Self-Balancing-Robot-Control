clc
clear

m = 0.34;
M = 0.09;
l = 0.031;
g = 9.81;
I = 0.000953;

Den = M*m*l^2 + M*I + m*I;

a = -(g*m^2*l^2)/Den;
b = (m*l^2 + I)/Den;
c = (g*l*m*(M+m))/Den;
d = -(l*m)/Den;

A = [0 1 0 0;
    0 0 a 0;
    0 0 0 1;
    0 0 c 0];

B = [0;
    b;
    0;
    d];

C = [1 0 0 0;
    0 0 1 0];

D = [0;
    0];

sys = ss(A,B,C,D);

