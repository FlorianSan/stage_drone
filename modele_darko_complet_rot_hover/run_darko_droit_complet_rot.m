clear all ;
close all;
clc;

drone = generateDarko();
controlleur = generateContolleur(drone);

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


position_cible = [0; 0; 0];

yaw = deg2rad(0);
pitch = deg2rad(-90);
roll = deg2rad(0);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; q_initial; omega_initial];

yaw_orient = deg2rad(0);
pitch_orient = deg2rad(-45);
roll_orient = deg2rad(90);
q_d_orient = ToQuaternion(yaw_orient, pitch_orient, roll_orient);

affichage = 0;

if (affichage == 1)
    
    sim1 = sim('simulation_darko_complet_visu.slx');
    
    yaw = deg2rad(45);
    pitch = deg2rad(-80);
    roll = deg2rad(45);
    q_initial = ToQuaternion(yaw, pitch, roll);

    initial_state = [p_initial; v_initial; omega_initial; q_initial];
    
    sim2 = sim('simulation_darko_complet_visu.slx');

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

    figure(2)
    plot3(sim2.positions(:,1), sim2.positions(:,2), sim2.positions(:,3));
    grid on;
    title("init loin")
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')

    figure(3)
    subplot(3,1,1)
    plot(sim1.tout,sim1.positions(:,1), sim2.tout, sim2.positions(:,1))
    title('Position x')
    legend("init proche","init loin")

    subplot(3,1,2)
    plot(sim1.tout,sim1.positions(:,2), sim2.tout, sim2.positions(:,2))
    title('Position y')
    legend("init proche","init loin")

    subplot(3,1,3)
    plot(sim1.tout,sim1.positions(:,3), sim2.tout, sim2.positions(:,3))
    title('Position z')
    legend("init proche","init loin")
    
    figure(4)
    subplot(4,1,1)
    plot(sim1.tout,sim1.orientation(:,1), sim2.tout, sim2.orientation(:,1))
    title('q1')
    legend("init proche","init loin")

    subplot(4,1,2)
    plot(sim1.tout,sim1.orientation(:,2), sim2.tout, sim2.orientation(:,2))
    title('q2')
    legend("init proche","init loin")

    subplot(4,1,3)
    plot(sim1.tout,sim1.orientation(:,3), sim2.tout, sim2.orientation(:,3))
    title('q3')
    legend("init proche","init loin")
    
    subplot(4,1,4)
    plot(sim1.tout,sim1.orientation(:,3), sim2.tout, sim2.orientation(:,3))
    title('q4')
    legend("init proche","init loin")
end
