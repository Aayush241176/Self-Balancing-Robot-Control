clear all
close all
clc

Robot_parameters

Q = diag([1 1 500 10]);
R = 1;

K = lqr(A,B,Q,R);
Acl = A - B*K;