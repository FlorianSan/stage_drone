clear ;
close all;
clc;

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];
q_initial = [0.5; 0.5; 0.5; 0.5];

initial_state = [p_initial; v_initial; omega_initial; q_initial];

%initial controlleur state
f_initial = 1;
q_d_initial = [0; 1; 0; 1]/sqrt(2);


position_cible = [1; 1; 1];


drone = generateDarko();
controlleur = generateContolleur(drone);

out = sim('simulation_arc.slx');

%% Plot 3D trajectory
disp('Plotting results ...')
figure(1)
%plot3(pos_x_d, pos_y_d, pos_z_d, 'r');
%hold on;
plot3(out.positions(:,1), out.positions(:,2), out.positions(:,3));
grid on;
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')