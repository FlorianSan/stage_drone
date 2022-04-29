clear all ;
close all;
clc;

drone = generateDarko();


%%%%%%%%%%%%%%%%
% Modele linéarisé complet
A = [0 0 0  1 0 0   0 0 0   0 0 0;
     0 0 0  0 1 0   0 0 0   0 0 0;
     0 0 0  0 0 1   0 0 0   0 0 0;
     0 0 0  0 0 0   0 2*sqrt(2)*drone.G 0   0 0 0;
     0 0 0  0 0 0   -sqrt(2)*drone.G 0 sqrt(2)*drone.G   0 0 0;
     0 0 0  0 0 0   0 -sqrt(2)*drone.G 0   0 0 0;
     0 0 0  0 0 0   0 0 0   sqrt(2)/4 0 -sqrt(2)/4;
     0 0 0  0 0 0   0 0 0   0 sqrt(2)/4 0;
     0 0 0  0 0 0   0 0 0   sqrt(2)/4 0 sqrt(2)/4;
     0 0 0  0 0 0   0 0 0   0 0 0;
     0 0 0  0 0 0   0 0 0   0 0 0;
     0 0 0  0 0 0   0 0 0   0 0 0;
     ];
 
 B1 = [0 0 0 0;
      0 0 0 0 ;
      0 0 0 0 ;
      0 0 -1/drone.MASS*drone.Fb(3,4) -1/drone.MASS*drone.Fb(3,4) ;
      0 0 0 0 ;
      1/drone.MASS*drone.Fb(1,1) 1/drone.MASS*drone.Fb(1,1) 0 0 ;
      0 0 0 0 ;
      0 0 0 0 ;
      0 0 0 0 ;
      1/drone.INERTIA(1,1)*drone.Mb(1,1) 1/drone.INERTIA(1,1)*drone.Mb(1,2) 1/drone.INERTIA(1,1)*drone.Mb(1,3) 1/drone.INERTIA(1,1)*drone.Mb(1,4) ;
      0 0 1/drone.INERTIA(2,2)*drone.Mb(2,3) 1/drone.INERTIA(2,2)*drone.Mb(2,4) ;
      1/drone.INERTIA(3,3)*drone.Mb(3,1) 1/drone.INERTIA(3,3)*drone.Mb(3,2) 0 0 ;
      ];
  
 B2 = [0 0 0;
       0 0 0;
       0 0 0;
       -2*drone.PHI(3,3) 0 0;
       0 0 0;
       0 0 -2*drone.PHI(1,1);
       0 0 0;
       0 0 0;
       0 0 0;
       0 0 0;
       -2*drone.CENTRAGE*drone.PHI(3,3) 0 0;
       0 0 0;
      ];
       


Co = ctrb(A,B1);
unco = length(A) - rank(Co);

Q = eye(12);
R = eye(4);
[K,S,e] = lqr(A,B1,Q,R);
S_complet = [S(1:6,1:6), zeros(6,1), S(1:6,7:12); zeros(1,13); S(7:12,1:6), zeros(6,1), S(7:12,7:12)];
K_complet = [K(:,1:6), zeros(4,1), K(:,7:12)];
matrix2latex(K_complet, 'out.tex', 'alignment', 'c', 'format', '%-6.2f', 'size', 'tiny');

% eig(A-B*K)

%%%%%%%%%%%%%%%
% Ki = -ones(4,12);
% Ki = [ones(1,6),zeros(1,6); ones(1,6),zeros(1,6); ones(1,6),zeros(1,6); ones(1,6),zeros(1,6];
Ki = K;

% H = [zeros(1,3), 0.1, zeros(1,8);zeros(1,5), 0.1, zeros(1,6)];
H = ones(2,12);
H(:,8) = [0;0];

C = eye(12);
C(8,8) = 0;
% C = [C(1:7,:),ZE;C(9:12,:)];

fun = @(X)max(real(eig([[A;X(5:6,:)*C], zeros(14,2)] - [B1;zeros(2,4)]*X(1:4,:)*[C, zeros(12,2)] + [B1;zeros(2,4)]*[zeros(4,12),[1,0;1,0;0,1;0,1]])));

x0 = [Ki;H];




lb = -1000*ones(6,12);
ub = 1000*ones(6,12);

MyValue = 10e5;
options = optimoptions('fmincon','Algorithm','sqp', 'MaxFunctionEvaluations',MyValue);

solution = fmincon(fun,x0,[],[],[],[],lb,ub, [], options)

Acl = [[A;solution(5:6,:)*C], zeros(14,2)] - [B1;zeros(2,4)]*solution(1:4,:)*[C, zeros(12,2)] + [B1;zeros(2,4)]*[zeros(4,12),[1,0;1,0;0,1;0,1]];
eig(Acl)
max(real(eig(Acl)))


Ki_complet = [solution(1:4,1:6), zeros(4,1), solution(1:4,7:12)];
H_complet = [solution(5:6,1:6), zeros(2,1), solution(5:6,7:12)];

% Bcl = zeros(14);
% Ccl = zeros(14);
% Dcl = zeros(14);

% sys = ss(Acl, Bcl, Ccl, Dcl);
% pzmap(sys)

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];

yaw = deg2rad(0);
pitch = deg2rad(-90);
roll = deg2rad(0);
q_initial = eul2quat([yaw, pitch, roll])';
initial_state = [p_initial; v_initial; q_initial; omega_initial];

u_eq = drone.G*drone.MASS/(2*drone.Fb(1,1)) * [1;1;0;0];
x_eq = [[0; 0; 0]; [0; 0; 0]; [1/sqrt(2); 0; -1/sqrt(2);0]; [0; 0; 0]];
x_eq_ = [[0; 0; 0]; [0; 0; 0]; [0; -1/sqrt(2);0]; [0; 0; 0]];




% [A_lin, B_lin, C_lin, ~] = linmod('darko_model', x_eq, u_eq);
% Co_lin = ctrb(A_lin,B_lin);
% unco_lin = length(A_lin) - rank(Co_lin);
% A_lin_ = [A_lin(1:6,1:6), A_lin(1:6,8:13);A_lin(8:13,1:6), A_lin(8:13,8:13)];
% B_lin_ = [B_lin(1:6,:); B_lin(8:13,:)];
% 
% Q_lin = eye(12);
% R_lin = eye(4);
% [K_lin,S_lin,e_lin] = lqr(A_lin_,B_lin_,Q_lin,R_lin);
% 
% K_lin_control = [K_lin(:,1:6), [0;0;0;0], K_lin(:,7:12)];
