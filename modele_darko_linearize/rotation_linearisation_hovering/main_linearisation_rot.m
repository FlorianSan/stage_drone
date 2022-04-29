clear all ;
close all;
clc;

drone = generateDarko();


%%%%%%%%%%%%%%%%
% Modele linéarisé complet
beta = 0;
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

Q = eye(12);
%Q(3,3)=100;
R = eye(4);
[K,S,e] = lqr(A,B,Q,R);
S_complet = [S(1:6,1:6), zeros(6,1), S(1:6,7:12); zeros(1,13); S(7:12,1:6), zeros(6,1), S(7:12,7:12)];
K_complet = [K(:,1:6), zeros(4,1), K(:,7:12)];
matrix2latex(K_complet, 'out.tex', 'alignment', 'c', 'format', '%-6.2f', 'size', 'tiny');

% eig(A-B*K)

%%%%%%%%%%%%%%%
%% LQI
C = [eye(3),zeros(3,9)];
D = zeros(3,4);
sys = ss(A,B,C,D);
Qi = eye(15);
Ri = eye(4);
Ni = eye(15,4);
[Ki,Si,ei] = lqi(sys,Qi,Ri, Ni);
S_complet_i = [Si(1:6,1:6), zeros(6,1), Si(1:6,7:15); zeros(1,16); Si(7:15,1:6), zeros(9,1), Si(7:15,7:15)];
K_complet_i = [Ki(:,1:6), zeros(4,1), Ki(:,7:15)];

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];

yaw = deg2rad(0);
pitch = deg2rad(-45);
roll = deg2rad(0);
q_initial = eul2quat([yaw, pitch, roll])';
initial_state = [p_initial; v_initial; q_initial; omega_initial];

u_eq = drone.G*drone.MASS/(2*drone.Fb(1,1)) * [1;1;0;0];
x_eq = [[0; 0; 0]; [0; 0; 0]; [1/sqrt(2); 0; -1/sqrt(2);0]; [0; 0; 0]];

initial_state_lin = [[0; 0]; [0.1; 0.1]; [0; -1/sqrt(2);0]; [0.1; 0.1; 0.1]];
x_eq_lin = [[0; 0]; [0; 0]; [0; -1/sqrt(2); 0]; [0; 0; 0]];

delta_x0 = initial_state_lin - x_eq_lin; 
% 




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
affichage = 0;
if (affichage == 1)
    figure(1)
    %sgtitle("Position du drone dans l'espace")
    subplot(3,1,1)
    plot(out.tout, out.positions(:,1))
    ylim([-1.5 1.5])
    grid on
    ylabel('px [m]')
    xlabel('time [s]')

    subplot(3,1,2)
    plot(out.tout, out.positions(:,2))
    ylim([-0.2 0.2])
    grid on
    ylabel('py [m]')
    xlabel('time [s]')

    subplot(3,1,3)
    plot(out.tout, out.positions(:,3))
    ylim([-1 1])
    grid on
    ylabel('pz [m]')
    xlabel('time [s]')
    
        figure(2)
    %sgtitle("Position du drone dans l'espace")
    subplot(3,1,1)
    plot(out.tout, out.orientation_deg(:,1))
    ylim([-1 0.6])
    grid on
    ylabel('Roulis [°]')
    xlabel('time [s]')

    subplot(3,1,2)
    plot(out.tout, out.orientation_deg(:,2))
    ylim([-130 -40])
    grid on
    ylabel('Tangage [°]')
    xlabel('time [s]')

    subplot(3,1,3)
    plot(out.tout, out.orientation_deg(:,3))
    ylim([-1 6])
    grid on
    ylabel('Lacet [°]')
    xlabel('time [s]')
    
            figure(3)
    %sgtitle("Position du drone dans l'espace")
    subplot(4,1,1)
    plot(out.tout, out.commande(:,1))
    ylim([600 850])
    grid on
    ylabel('\omega droite [rad/s]')
    xlabel('time [s]')

    subplot(4,1,2)
    plot(out.tout, out.commande(:,2))
    ylim([-850 -600])
    grid on
    ylabel('\omega gauche [rad/s]')
    xlabel('time [s]')

    subplot(4,1,3)
    plot(out.tout, out.commande(:,3))
    ylim([-20 35])
    grid on
    ylabel('\delta droit [deg]')
    xlabel('time [s]')
    
    subplot(4,1,4)
    plot(out.tout, out.commande(:,4))
    ylim([-20 35])
    grid on
    ylabel('\delta gauche [deg]')
    xlabel('time [s]')
end