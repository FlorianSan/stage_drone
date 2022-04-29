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
pitch = deg2rad(-90);
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



affichage = 1;

if (affichage == 1)
    out = sim('simulation_darko_modif_nu.slx');
    out2 = sim('simulation_darko.slx');

    % Plot 3D trajectory
    disp('Plotting results ...')
    figure(1)
    plot3(out.positions(:,1), out.positions(:,2), out.positions(:,3));
    %axis equal
    grid on;
    title("modif nu")
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')

    figure(2)
    plot3(out2.positions(:,1), out2.positions(:,2), out2.positions(:,3));
    grid on;
    title("Sans modif nu")
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')

    figure(3)
    subplot(3,1,1)
    plot(out.tout,out.positions(:,1), out2.tout, out2.positions(:,1))
    title('Position x')
    legend("nu'","nu")

    subplot(3,1,2)
    plot(out.tout,out.positions(:,2), out2.tout, out2.positions(:,2))
    title('Position y')
    legend("nu'",'nu')

    subplot(3,1,3)
    plot(out.tout,out.positions(:,3), out2.tout, out2.positions(:,3))
    title('Position z')
    legend("nu'",'nu')


    %% F_delta
    figure(4)
    subplot(3,1,1)
    plot(out.tout,out.f_delta(:,1), out2.tout,out2.f_delta(:,1))
    title("F_{delta} x ")
    legend("nu'",'nu')

    subplot(3,1,2)
    plot(out.tout,out.f_delta(:,2), out2.tout,out2.f_delta(:,2))
    title("F_{delta} y")
    legend("nu'",'nu')

    subplot(3,1,3)
    plot(out.tout,out.f_delta(:,3), out2.tout,out2.f_delta(:,3))
    title("F_{delta} y")
    legend("nu'",'nu')
end
