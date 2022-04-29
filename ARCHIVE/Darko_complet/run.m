clear all ;
close all;
clc;

drone = generateDarko();

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [0; 0; 0];
omega_initial = [0; 0; 0];
yaw = deg2rad(0);
pitch = deg2rad(-90);
roll = deg2rad(0);
q_initial = ToQuaternion(yaw, pitch, roll);

initial_state = [p_initial; v_initial; omega_initial; q_initial];
initial_state2 = [v_initial; omega_initial; q_initial];