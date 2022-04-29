clear all ;
close all;
clc;

drone = generateDarko();
controlleur = generateContolleur(drone);
% M = drone.Mb;
% F = drone.Fb;
% save('matrix.mat','M','F')
% matrix2latex(controlleur.M_k, 'out.tex', 'alignment', 'c', 'format', '%-6.2f', 'size', 'tiny');

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];


%initial controlleur state
f_initial = drone.MASS*drone.G;

yaw_d = deg2rad(0);
pitch_d = deg2rad(-90);
roll_d = deg2rad(0);
q_d_initial = ToQuaternion(yaw_d, pitch_d, roll_d);


position_cible = [5; 6; 7];

yaw = deg2rad(10);
pitch = deg2rad(-45);
roll = deg2rad(10);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; q_initial; omega_initial];

affichage = 0;

if (affichage == 1)


    % Plot 3D trajectory
    disp('Plotting results ...')
    figure(1)
    hold on
    [n,m] = size(sim1.orientation);
    for k=1:n
        ord = quatrotate(sim1.orientation(k,:),  eye(3));
        drawOrdinates(ord, sim1.positions(k,:));
    end
    plot3(sim1.positions(:,1), sim1.positions(:,2), sim1.positions(:,3));
    hold off
    %axis equal
    grid on;
    title("init proche")
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')

 figure(1)
    %sgtitle("Position du drone dans l'espace")
    subplot(3,1,1)
    plot(out.tout, out.positions(:,1))
    ylim([-0.5 6])
    grid on
    ylabel('px [m]')
    xlabel('time [s]')

    subplot(3,1,2)
    plot(out.tout, out.positions(:,2))
    ylim([-0.5 6])
    grid on
    ylabel('py [m]')
    xlabel('time [s]')

    subplot(3,1,3)
    plot(out.tout, out.positions(:,3))
    ylim([-0.5 6])
    grid on
    ylabel('pz [m]')
    xlabel('time [s]')
    
        figure(2)
    %sgtitle("Position du drone dans l'espace")
    subplot(3,1,1)
    plot(out.tout, out.orientation_deg(:,1))
    ylim([-10 15])
    grid on
    ylabel('Roulis [°]')
    xlabel('time [s]')

    subplot(3,1,2)
    plot(out.tout, out.orientation_deg(:,2))
    ylim([-100 -40])
    grid on
    ylabel('Tangage [°]')
    xlabel('time [s]')

    subplot(3,1,3)
    plot(out.tout, out.orientation_deg(:,3))
    ylim([0 30])
    grid on
    ylabel('Lacet [°]')
    xlabel('time [s]')
    
    figure(3)

    subplot(4,1,1)
    plot(out.tout, out.commande(:,1))
    ylim([100 900])
    grid on
    ylabel('\omega droite [rad/s]')
    xlabel('time [s]')

    subplot(4,1,2)
    plot(out.tout, out.commande(:,2))
    ylim([-1100 -600])
    grid on
    ylabel('\omega gauche [rad/s]')
    xlabel('time [s]')

    subplot(4,1,3)
    plot(out.tout, out.commande(:,3))
    ylim([-10 35])
    grid on
    ylabel('\delta droit [deg]')
    xlabel('time [s]')
    
    subplot(4,1,4)
    plot(out.tout, out.commande(:,4))
    ylim([-10 35])
    grid on
    ylabel('\delta gauche [deg]')
    xlabel('time [s]')
end
