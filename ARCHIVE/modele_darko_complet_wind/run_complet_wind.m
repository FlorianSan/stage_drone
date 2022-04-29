clear all ;
close all;
clc;

drone = generateDarko();
controlleur = generateContolleur(drone);

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];
yaw = deg2rad(10);
pitch = deg2rad(-80);
roll = deg2rad(10);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; omega_initial; q_initial];

%initial controlleur state
f_initial = drone.MASS*drone.G;

yaw_d = deg2rad(0);
pitch_d = deg2rad(-90);
roll_d = deg2rad(0);
q_d_initial = ToQuaternion(yaw_d, pitch_d, roll_d);


position_cible = [1; 1; 1];

wind =[0;0;0];

affichage = 0;

if (affichage == 1)
    sans = sim('simulation_darko_complet_wind.slx');
    
    wind = [0;0;1];
    avec = sim('simulation_darko_complet_wind.slx');

    % Plot 3D trajectory
    disp('Plotting results ...')
    figure(1)
    plot3(sans.positions(:,1), sans.positions(:,2), sans.positions(:,3));
    %axis equal
    grid on;
    title("Sans vent")
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')

    figure(2)
    plot3(avec.positions(:,1), avec.positions(:,2), avec.positions(:,3));
    grid on;
    title("Avec vent")
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')

    figure(3)
    subplot(3,1,1)
    plot(sans.tout,sans.positions(:,1), avec.tout, avec.positions(:,1))
    title('Position x')
    legend("Sans vent","Avec vent")

    subplot(3,1,2)
    plot(sans.tout,sans.positions(:,2), avec.tout, avec.positions(:,2))
    title('Position y')
    legend("Sans vent","Avec vent")

    subplot(3,1,3)
    plot(sans.tout,sans.positions(:,3), avec.tout, avec.positions(:,3))
    title('Position z')
    legend("Sans vent","Avec vent")
    
    figure(4)
    subplot(4,1,1)
    plot(sans.tout,sans.orientation(:,1), avec.tout, avec.orientation(:,1))
    title('q1')
    legend("Sans vent","Avec vent")

    subplot(4,1,2)
    plot(sans.tout,sans.orientation(:,2), avec.tout, avec.orientation(:,2))
    title('q2')
    legend("Sans vent","Avec vent")

    subplot(4,1,3)
    plot(sans.tout,sans.orientation(:,3), avec.tout, avec.orientation(:,3))
    title('q3')
    legend("Sans vent","Avec vent")
    
    subplot(4,1,4)
    plot(sans.tout,sans.orientation(:,3), avec.tout, avec.orientation(:,3))
    title('q4')
    legend("Sans vent","Avec vent")
end
