clear all ;
close all;
clc;

controlleur = generateContolleur();

speed = 25;

p_initial = [500; 0; 0];
v_initial = [speed; 0; 0];
omega_initial = [0; 0; 0];


yaw = deg2rad(0);
pitch = deg2rad(0);
roll = deg2rad(0);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; q_initial; omega_initial];

p1 = [0;0;0];
%p1 = p_initial;
p2 = [1000; 1000; 0];

out = sim("simulation_path_following.slx");

figure(1)
plot3(out.positions(:,1), out.positions(:,2), out.positions(:,3));
hold on
plot3([p1(1),p2(1)], [p1(2),p2(2)], [p1(3),p2(3)]);
grid on;
title("")
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
