clear all ;
close all;
clc;

% syms ua ub theta S Sp cd0 cl0 rho eta wx wz Thetaf Thetam m g dR

% assume(ua > 0)
% 
% 
% 
% 
% eqns = [(1-(S/(4*Sp))*cd0)*2*ua -1/2*rho*S*eta*cd0*wx + 1/4*rho*S*eta*cd0*Thetaf*2*ub*wz + m*g*sin(theta) == 0,...
%        -1/2*rho*S*eta*cl0*wz - 1/4*rho*S*eta*cl0*Thetaf*2*ub*wx - S/(4*Sp)*Thetaf*cl0*2*ua*ub - m*g*cos(theta) == 0,...
%         1/2*rho*S*dR*cl0*eta*wz + 1/4*rho*S*dR*cl0*eta*Thetam*2*ub*wx + S/(4*Sp)*dR*cl0*Thetam*2*ua*ub == 0];
% S = solve(eqns,[ua, ub, theta])
% [ua_s, ub_s, theta_s,parameters,conditions] = solve(eqns,[ua, ub, theta],'ReturnConditions',true,'IgnoreAnalyticConstraints',true)

% eqns = [(1-(S/(4*Sp))*cd0)*2*ua -1/2*rho*S*eta*cd0*(cos(theta)*wx+sin(theta)*wz) + 1/4*rho*S*eta*cd0*Thetaf*2*ub*(-sin(theta)*wx+cos(theta)*wz) + m*g*sin(theta) == 0,...
%        -1/2*rho*S*eta*cl0*(-sin(theta)*wx+cos(theta)*wz) - 1/4*rho*S*eta*cl0*Thetaf*2*ub*(cos(theta)*wx+sin(theta)*wz) - S/(4*Sp)*Thetaf*cl0*2*ua*ub - m*g*cos(theta) == 0,...
%         1/2*rho*S*dR*cl0*eta*(-sin(theta)*wx+cos(theta)*wz) + 1/4*rho*S*dR*cl0*eta*Thetam*2*ub*(cos(theta)*wx+sin(theta)*wz) + S/(4*Sp)*dR*cl0*Thetam*2*ua*ub == 0];
% S = solve(eqns,[ua, ub, theta])
% [ua_s, ub_s, theta_s,parameters,conditions] = solve(eqns,[ua, ub, theta],'ReturnConditions',true,'IgnoreAnalyticConstraints',true)

syms A B theta
assume(A,'real')
assume(B,'real')
eqns = [A*cos(theta) + B*sin(theta) == 0];
% S = solve(eqns,[theta])
[theta_s,parameters,conditions] = solve(eqns, theta,'ReturnConditions',true,'IgnoreAnalyticConstraints',true)

latex(theta_s)
A = 1;
B =2;
theta1 = subs(theta_s)
double(theta1)
