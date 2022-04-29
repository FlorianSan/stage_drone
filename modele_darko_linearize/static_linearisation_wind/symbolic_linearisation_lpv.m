clear all ;
close all;
clc;

syms T1 T2 d1 d2 S Sp cd0 cl0 rho eta wx wz xif xim m g dR kf km ay py m J1 J2 J3
syms e1 real
syms e2 real
syms e3 real
assume(e1>=0 & e1<=1)
assume(e2>=0 & e2<=1)
assume(e3>=0 & e3<=1)
assume(T1 == T2)
assume(d1 == d2)

% assume(sqrt(e1^2+e2^2+e3^2) == 1)

R = [1-2*e2^2-2*e3^2, 2*e1*e2 - 2*sqrt(1-e1^2-e2^2-e3^2)*e3, 2*sqrt(1-e1^2-e2^2-e3^2)*e2 + 2*e1*e3;
    2*sqrt(1-e1^2-e2^2-e3^2)*e3 + 2*e1*e2, 1-2*e1^2-2*e3^2, 2*e2*e3 - 2*sqrt(1-e1^2-e2^2-e3^2)*e1;
    2*e1*e3 - 2*sqrt(1-e1^2-e2^2-e3^2)*e2, 2*sqrt(1-e1^2-e2^2-e3^2)*e1 + 2*e2*e3, 1-2*e1^2-2*e2^2];

Vinf_b = R'*([0;0;0] - [wx; 0; wz]);

eqns_F = (1/m)*R*[(1-(S*cd0)/(4*Sp))*(T1+T2) - 1/2*rho*S*eta*cd0*Vinf_b(1) + 1/4*rho*S*eta*cd0*xif*(d1+d2)*Vinf_b(3);
            0;
            -S/(4*Sp)*xif*cl0*(T1+T2)*(d1+d2) - 1/2*rho*S*eta*cl0*Vinf_b(3) - 1/4*rho*S*eta*cl0*xif*(d1+d2)*Vinf_b(1)];

% eqns_M = diag([J1 J2 J3])\([(km/kf)*(T1-T2) + (S*cl0*ay*xif)/(4*Sp)*(d1*T1 - d2*T2) + 1/4*rho*S*eta*ay*xif*cl0 *Vinf_b(1)*(d1-d2);
%         (S*cl0*dR*xim)/(4*Sp)*(d1*T1 + d2*T2) + 1/2*rho*S*eta*dR*cl0 *Vinf_b(3) + 1/4*rho*S*eta*dR*xim*cl0 *Vinf_b(1)*(d1+d2);
%         (py + (S*cd0*ay)/(4*Sp))*(T1-T2) + 1/4*rho*S*eta*ay*xif*cd0 *Vinf_b(3)*(d1-d2)]);

eqns_M = [(km/kf)*(T1-T2) + (S*cl0*ay*xif)/(4*Sp)*(d1*T1 - d2*T2) + 1/4*rho*S*eta*ay*xif*cl0 *Vinf_b(1)*(d1-d2);
        (S*cl0*dR*xim)/(4*Sp)*(d1*T1 + d2*T2) + 1/2*rho*S*eta*dR*cl0 *Vinf_b(3) + 1/4*rho*S*eta*dR*xim*cl0 *Vinf_b(1)*(d1+d2);
        (py + (S*cd0*ay)/(4*Sp))*(T1-T2) + 1/4*rho*S*eta*ay*xif*cd0 *Vinf_b(3)*(d1-d2)];


F_e1 = jacobian(eqns_F,e1);
F_e2 = jacobian(eqns_F,e2);
F_e3 = jacobian(eqns_F,e3);

F_T1 = jacobian(eqns_F,T1);
F_T2 = jacobian(eqns_F,T2);
F_d1 = jacobian(eqns_F,d1);
F_d2 = jacobian(eqns_F,d2);

F_wx = jacobian(eqns_F,wx);
F_wz = jacobian(eqns_F,wz);

M_e1 = jacobian(eqns_M,e1);
M_e2 = jacobian(eqns_M,e2);
M_e3 = jacobian(eqns_M,e3);

M_T1 = jacobian(eqns_M,T1);
M_T2 = jacobian(eqns_M,T2);
M_d1 = jacobian(eqns_M,d1);
M_d2 = jacobian(eqns_M,d2);

M_wx = jacobian(eqns_M,wx);
M_wz = jacobian(eqns_M,wz);

S_sol = subs(M_wz,[e1 e3],[0 0])

% chr = latex(S_sol(3))

                                    (S*ay*cl0*e2*eta*rho*xif*(1 - e2^2)^(1/2)*(d1 - d2))/2
1/2*S*cl0*dR*eta*rho*(2*e2^2 - 1)+ 1/2*S*cl0*dR*e2*eta*rho*xim*(1 - e2^2)^(1/2)*(d1 + d2)
                                           (S*ay*cd0*eta*rho*xif*(2*e2^2 - 1)*(d1 - d2))/4

% 1/4*S*ay*cl0*eta*rho*xif*(2*e2^2 - 1)*(d1 - d2)
% 1/4*S*cl0*dR*eta*rho*xim*(d1 + d2)*(2*e2^2 - 1) - S*cl0*dR*e2*eta*rho*(1 - e2^2)^(1/2)
%                                -1/2*S*ay*cd0*e2*eta*rho*xif*(1 - e2^2)^(1/2)*(d1 - d2)

% 1/m*((2*e2^2 - 1)*(S*cd0*e2*eta*rho*(1 - e2^2)^(1/2) - 1/4*S*cd0*eta*rho*xif*(d1 + d2)*(2*e2^2 - 1)) ...
% - 2*e2*(1 - e2^2)^(1/2)*(1/2*S*cl0*eta*rho*(2*e2^2 - 1) + 1/2*S*cl0*e2*eta*rho*xif*(1 - e2^2)^(1/2)*(d1 + d2)))
% 
% 1/m*((2*e2^2 - 1)*(1/2*S*cl0*eta*rho*(2*e2^2 - 1) + 1/2*S*cl0*e2*eta*rho*xif*(1 - e2^2)^(1/2)*(d1 + d2))...
% + 2*e2*(1 - e2^2)^(1/2)*(S*cd0*e2*eta*rho*(1 - e2^2)^(1/2) - 1/2*S*cd0*eta*rho*xif*(d1 + d2)*(2*e2^2 - 1)))
 

% 1/m*((2*e2^2 - 1)*(1/2*S*cd0*eta*rho*(2*e2^2 - 1) + 1/2*S*cd0*e2*eta*rho*xif*(1 - e2^2)^(1/2)*(d1 + d2))...
%     + 2*e2*(1 - e2^2)^(1/2)*(S*cl0*e2*eta*rho*(1 - e2^2)^(1/2) - 1/4*S*cl0*eta*rho*xif*(d1 + d2)*(2*e2^2 - 1)));
% 
% 1/m*(2*e2*(1 - e2^2)^(1/2)*(1/2*S*cd0*eta*rho*(2*e2^2 - 1) + 1/2*S*cd0*e2*eta*rho*xif*(1 - e2^2)^(1/2)*(d1 + d2))...
%     - (2*e2^2 - 1)*(1/2*S*cl0*2*e2*eta*rho*(1 - e2^2)^(1/2) - 1/4*S*cl0*eta*rho*xif*(d1 + d2)*(2*e2^2 - 1));

% (1/m)*(4*e2*((T1 + T2)*((S*cd0)/(4*Sp) - 1) + 1/2*S*cd0*eta*rho*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2)) - 1/4*S*cd0*eta*rho*xif*(d1 + d2)*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)))...
% + (2*e2^2 - 1)*(1/2*S*cd0*eta*rho*(4*e2*wx - wz*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2))) - 1/4*S*cd0*eta*rho*xif*(4*e2*wz + wx*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2)))*(d1 + d2))...
% + ((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2))*(1/2*S*cl0*eta*rho*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)) + (S*cl0*xif*(T1 + T2)*(d1 + d2))/(4*Sp) + 1/4*S*cl0*eta*rho*xif*(d1 + d2)*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2)))...
% - 2*e2*(1 - e2^2)^(1/2)*(1/2*S*cl0*eta*rho*(4*e2*wz + wx*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2))) + 1/4*S*cl0*eta*rho*xif*(4*e2*wx - wz*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2)))*(d1 + d2)))
%  
%  
%    
% 
% 
% (1/m)*(4*e2*(1/2*S*cl0*eta*rho*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)) + (S*cl0*xif*(T1 + T2)*(d1 + d2))/(4*Sp) + 1/4*S*cl0*eta*rho*xif*(d1 + d2)*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2)))...
% + (2*e2^2 - 1)*(1/2*S*cl0*eta*rho*(4*e2*wz + wx*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2))) + 1/4*S*cl0*eta*rho*xif*(4*e2*wx - wz*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2)))*(d1 + d2)) ...
% - ((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2))*((T1 + T2)*((S*cd0)/(4*Sp) - 1) + 1/2*S*cd0*eta*rho*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2)) - 1/4*S*cd0*eta*rho*xif*(d1 + d2)*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)))...
% + 2*e2*(1/2*S*cd0*eta*rho*(4*e2*wx - wz*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2))) - 1/4*S*cd0*eta*rho*xif*(4*e2*wz + wx*((2*e2^2)/(1 - e2^2)^(1/2) - 2*(1 - e2^2)^(1/2)))*(d1 + d2))*(1 - e2^2)^(1/2))
%  

% (1/m)*(-2*(1 - e2^2)^(1/2)*((T1 + T2)*((S*cd0)/(4*Sp) - 1) + 1/2*S*cd0*eta*rho*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2)) - 1/4*S*cd0*eta*rho*xif*(d1 + d2)*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)))...
% - 2*e2*(1/2*S*cl0*eta*rho*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)) + (S*cl0*xif*(T1 + T2)*(d1 + d2))/(4*Sp) + 1/4*S*cl0*eta*rho*xif*(d1 + d2)*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2))))


% (1/m)*(- 2*e2*(1 - e2^2)^(1/2)*((S*cl0*xif*(T1 + T2))/(4*Sp) + 1/4*S*cl0*eta*rho*xif*(wx*(2*e2^2 - 1) + 2*e2*wz*(1 - e2^2)^(1/2))) - 1/4*S*cd0*eta*rho*xif*(2*e2^2 - 1)*(wz*(2*e2^2 - 1) - 2*e2*wx*(1 - e2^2)^(1/2)))
 