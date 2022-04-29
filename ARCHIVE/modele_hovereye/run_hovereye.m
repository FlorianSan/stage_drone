clear all ;
close all;
clc;

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];
q_initial = [0; 0; 0; 1]/sqrt(1);

position_cible = [0.2; 0.2; 0.2];


drone = generatehovereye();
controlleur = generateContolleur();

out = sim('simulation_hovereye.slx');

%% Plot 3D trajectory
disp('Plotting results ...')
figure(1)
%plot3(pos_x_d, pos_y_d, pos_z_d, 'r');
%hold on;
plot3(out.positions(:,1), out.positions(:,2), out.positions(:,3));
%axis equal
grid on;
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')