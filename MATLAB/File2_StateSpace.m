clc
clear

syms M m l I g F theta
syms ddx ddtheta

A = [ M+m , m*l ;
      m*l , m*l^2 + I ] ;
b = [ F ;
      m*l*g*theta];
sol= A\b ;
ddx_sol = simplify(sol(1));
ddtheta_sol = simplify(sol(2));
