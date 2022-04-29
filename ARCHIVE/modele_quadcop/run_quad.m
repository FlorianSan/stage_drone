clear all;
close all;
clc;

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];

yaw = deg2rad(10);
pitch = deg2rad(10);
roll = deg2rad(10);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; omega_initial; q_initial];

%initial controlleur state
f_initial = 1;

yaw_d = deg2rad(-10);
pitch_d = deg2rad(20);
roll_d = deg2rad(-10);
q_d_initial = ToQuaternion(yaw_d, pitch_d, roll_d);


%position_cible = [0.2; 0.2; 0.2];
position_cible = 0.5*[1; 1; 1];

drone = generatequad();
controlleur = generateContolleur(drone);

controlleur.a = 10;
controlleur.b = -log(1/2);  %halfway point
controlleur.c = 1; %growth rate


%  out = sim('simulation_quad_modif_nu.slx');
%  out2 = sim('simulation_quad.slx');
% 
% %% Plot 3D trajectory
% disp('Plotting results ...')
% figure(1)
% %plot3(pos_x_d, pos_y_d, pos_z_d, 'r');
% %hold on;
% plot3(out.positions(:,1), out.positions(:,2), out.positions(:,3));
% grid on;
% axis equal
% title("modif nu")
% xlabel('X [m]')
% ylabel('Y [m]')
% zlabel('Z [m]')
% 
% figure(2)
% %plot3(pos_x_d, pos_y_d, pos_z_d, 'r');
% %hold on;
% plot3(out2.positions(:,1), out2.positions(:,2), out2.positions(:,3));
% grid on;
% axis equal
% title("Sans modif nu")
% xlabel('X [m]')
% ylabel('Y [m]')
% zlabel('Z [m]')
% 
% figure(3)
% subplot(3,2,1)
% plot(out.tout,out.positions(:,1))
% title("Position x (nu')")
% 
% subplot(3,2,2)
% plot(out2.tout,out2.positions(:,1))
% title("Position x (nu)")
% 
% subplot(3,2,3)
% plot(out.tout,out.positions(:,2))
% title("Position y (nu')")
% 
% subplot(3,2,4)
% plot(out2.tout,out2.positions(:,2))
% title("Position y (nu)")
% 
% subplot(3,2,5)
% plot(out.tout,out.positions(:,3))
% title("Position z (nu')")
% 
% subplot(3,2,6)
% plot(out2.tout,out2.positions(:,3))
% title("Position z(nu)")
% 
% 
% %% F_delta
% figure(4)
% subplot(3,2,1)
% plot(out.tout,out.f_delta(:,1))
% title("F_{delta} x (nu')")
% 
% subplot(3,2,2)
% plot(out2.tout,out2.f_delta(:,1))
% title("F_{delta} x (nu)")
% 
% subplot(3,2,3)
% plot(out.tout,out.f_delta(:,2))
% title("F_{delta} y (nu')")
% 
% subplot(3,2,4)
% plot(out2.tout,out2.f_delta(:,2))
% title("F_{delta} y (nu)")
% 
% subplot(3,2,5)
% plot(out.tout,out.f_delta(:,3))
% title("F_{delta} z (nu')")
% 
% subplot(3,2,6)
% plot(out2.tout,out2.f_delta(:,3))
% title("F_{delta} z(nu)")