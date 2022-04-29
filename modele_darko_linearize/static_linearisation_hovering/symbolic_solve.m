clear all ;
close all;
clc;

syms e1 e2 e3 w1 w2 w3 g

%% Linearisation sans vent 
% M = jacobian([1-2*e2^2-2*e3^2, 2*e3*sqrt(1-e1^2-e2^2-e3^2) + 2*e1*e2, -2*e2*sqrt(1-e1^2-e2^2-e3^2) + 2*e1*e3],[e1,e2,e3])
% q_point = 1/2*[sqrt(1-e1^2-e2^2-e3^2)*w1-e3*w2+e2*w3,e3*w1+sqrt(1-e1^2-e2^2-e3^2)*w2-e1*w3,-e2*w1+e1*w2+sqrt(1-e1^2-e2^2-e3^2)*w3];
% M2 = jacobian(q_point, [w1,w2,w3])
% M3 = jacobian(q_point, [e1,e2,e3])
% e1 = 0;
% e3 = 0;
% 
% subs(M)
% subs(M2)
% e2 = sqrt(2)/2;
% subs(M)
% subs(M2)
% w1 = 0;
% w2 = 0;
% w3 = 0;
% subs(M3)

%% Linearisation avec vent
M = jacobian([1-2*e2^2-2*e3^2 + 2*e2*sqrt(1-e1^2-e2^2-e3^2) + 2*e1*e3, 2*e3*sqrt(1-e1^2-e2^2-e3^2) + 2*e1*e2 - 2*e1*sqrt(1-e1^2-e2^2-e3^2) + 2*e2*e3, -2*e2*sqrt(1-e1^2-e2^2-e3^2) + 2*e1*e3 + 1-2*e1^2-2*e2^2],[e1,e2,e3])
q_point = 1/2*[sqrt(1-e1^2-e2^2-e3^2)*w1-e3*w2+e2*w3,e3*w1+sqrt(1-e1^2-e2^2-e3^2)*w2-e1*w3,-e2*w1+e1*w2+sqrt(1-e1^2-e2^2-e3^2)*w3];
M2 = jacobian(q_point, [w1,w2,w3])
M3 = jacobian(q_point, [e1,e2,e3])
e1 = 0;
e3 = 0;

subs(M)
subs(M2)
e2 = sqrt(2)/2;
subs(M)
subs(M2)
w1 = 0;
w2 = 0;
w3 = 0;
subs(M3)
