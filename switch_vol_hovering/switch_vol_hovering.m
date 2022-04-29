clear all ;
close all;
clc;

drone = generateDarko();
controlleur = generateContolleur(drone);

%%%%%%%%%%%%%%%%
%%  Modele linéarisé complet
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
 
 B = [0 0 0 0 ;
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

Co = ctrb(A,B);
unco = length(A) - rank(Co);

%%%%%%%%%%%%%%%%
%% Controlleur linearise hovering
Q = eye(12);
R = eye(4);
[K,S,e] = lqr(A,B,Q,R);
S_complet = [S(1:6,1:6), zeros(6,1), S(1:6,7:12); zeros(1,13); S(7:12,1:6), zeros(6,1), S(7:12,7:12)];
K_complet = [K(:,1:6), [0;0;0;0], K(:,7:12)];

u_eq_hover = drone.G*drone.MASS/(2*drone.Fb(1,1)) * [1;1;0;0];
x_eq_hover = [[0; 0; 0]; [0; 0; 0]; [1/sqrt(2); 0; -1/sqrt(2);0]; [0; 0; 0]];

% initial_state_lin = [[0; 0]; [0.1; 0.1]; [0; -1/sqrt(2);0]; [0.1; 0.1; 0.1]];
% x_eq_lin = [[0; 0]; [0; 0]; [0; -1/sqrt(2); 0]; [0; 0; 0]];

% delta_x0 = initial_state_lin - x_eq_lin; 


%%%%%%%%%%%%%%%%
%% Controlleur vol lineaire
v_trim = 5;
trim = utComputeTrimFlight_fmin_dyn(v_trim, drone)
p_initial = [0; 0; 0];
v_initial = [v_trim; 0; 0];
q_initial = [sqrt(1-trim(4)^2); 0; trim(4);0];
omega_initial = [0; 0; 0];

u_eq_vol = [trim(1);trim(1);trim(2);trim(2)];
x_eq_vol = [p_initial; v_initial; q_initial; omega_initial];

[A_lin, B_lin, C_lin, ~] = linmod('darko_model', x_eq_vol, u_eq_vol);

A_lin_ = [A_lin(3:6,3:6), A_lin(3:6,8:13); A_lin(8:13,3:6), A_lin(8:13,8:13)];
B_lin_ = [B_lin(3:6,:); B_lin(8:13,:)];

Q_lin = eye(10);
R_lin = eye(4);
[K_lin,S_lin,e_lin] = lqr(A_lin_,B_lin_,Q_lin,R_lin);

K_lin_control = [zeros(4,2), K_lin(:,1:4), zeros(4,1), K_lin(:,5:10)];


%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];

yaw = deg2rad(0);
pitch = deg2rad(-45);
roll = deg2rad(0);
q_initial = eul2quat([yaw, pitch, roll])';

s = 1;%etat du switch

x0 = [p_initial; v_initial; q_initial; omega_initial; s];



position_cible = [50; 0; 0];

%switch surface
a = 400;
b = 500;

% simulation horizon                                                    
T = 80;                                                                 
J = 200;                                                                 
                                                                        
% rule for jumps                                                        
% rule = 1 -> priority for jumps                                        
% rule = 2 -> priority for flows                                        
% rule = 3 -> no priority, random selection when simultaneous conditions
rule = 1;                                                               
                                                                        
%solver tolerances
RelTol = 1e-6;
MaxStep = 1e-3;