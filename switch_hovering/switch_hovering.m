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
%% Controlleur linearise
Q = eye(12);
R = eye(4);
[K,S,e] = lqr(A,B,Q,R);
S_complet = [S(1:6,1:6), zeros(6,1), S(1:6,7:12); zeros(1,13); S(7:12,1:6), zeros(6,1), S(7:12,7:12)];
K_complet = [K(:,1:6), [0;0;0;0], K(:,7:12)];

u_eq = drone.G*drone.MASS/(2*drone.Fb(1,1)) * [1;1;0;0];
x_eq = [[0; 0; 0]; [0; 0; 0]; [1/sqrt(2); 0; -1/sqrt(2);0]; [0; 0; 0]];

% initial_state_lin = [[0; 0]; [0.1; 0.1]; [0; -1/sqrt(2);0]; [0.1; 0.1; 0.1]];
% x_eq_lin = [[0; 0]; [0; 0]; [0; -1/sqrt(2); 0]; [0; 0; 0]];

% delta_x0 = initial_state_lin - x_eq_lin; 


%%%%%%%%%%%%%%%%
%% Controlleur non lineaire

f_initial = drone.MASS*drone.G;

yaw_d = deg2rad(0);
pitch_d = deg2rad(-90);
roll_d = deg2rad(0);
q_d_initial = ToQuaternion(yaw_d, pitch_d, roll_d);





%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];

yaw = deg2rad(0);
pitch = deg2rad(-45);
roll = deg2rad(0);
q_initial = eul2quat([yaw, pitch, roll])';

s = 1; %etat du switch

x0 = [p_initial; v_initial; q_initial; omega_initial; s];



position_cible = [50; 25; 12.5];

%switch surface
a = 250;
b = 300;

% simulation horizon                                                    
T = 49.999;                                                                 
J = 200;                                                                 
                                                                        
% rule for jumps                                                        
% rule = 1 -> priority for jumps                                        
% rule = 2 -> priority for flows                                        
% rule = 3 -> no priority, random selection when simultaneous conditions
rule = 1;                                                               
                                                                        
%solver tolerances
RelTol = 1e-6;
MaxStep = 1e-3;