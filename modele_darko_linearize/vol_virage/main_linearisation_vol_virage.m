clear all ;
close all;
clc;

drone = generateDarko();

v_trim = 2;
R = 10; %rayon de virage

trim = utComputeTrimFlight_fmin_dyn_virage(v_trim, R, drone)
eulZYX = rad2deg(quat2eul([sqrt(1-trim(3)^2 - trim(4)^2 - trim(5)^2 );  trim(3); trim(4); trim(5)]'))

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [v_trim; 0; 0];
q_initial = [sqrt(1-trim(3)^2 - trim(4)^2 - trim(5)^2 );  trim(3); trim(4); trim(5)];
omega_initial = [0; 0; 0];

u_eq = [trim(1); trim(1); trim(2); trim(2)];
x_eq = [p_initial; v_initial; q_initial; omega_initial];

[A_lin, B_lin, C_lin, ~] = linmod('darko_model', x_eq, u_eq);
Co_lin = ctrb(A_lin,B_lin);
unco_lin = length(A_lin) - rank(Co_lin);

% %% Juste
% A_lin_ = [A_lin(3:6,3:6), A_lin(3:6, 8:13); A_lin(8:13,3:6), A_lin(8:13,8:13)];
% B_lin_ = [B_lin(3:6,:); B_lin(8:13,:)];
% Co_lin_ = ctrb(A_lin_,B_lin_);
% unco_lin_ = length(A_lin_) - rank(Co_lin_)
% Q_lin = eye(10);
% R_lin = eye(4);
% [K_lin,S_lin,e_lin] = lqr(A_lin_,B_lin_,Q_lin,R_lin);
% 
% % S_complet = [S_lin(1:6,1:6), zeros(6,1), S_lin(1:6,7:12); zeros(1,13); S_lin(7:12,1:6), zeros(6,1), S_lin(7:12,7:12)];
% K_lin_control = [zeros(4,2), K_lin(:,1:4), zeros(4,1), K_lin(:,5:10)];

A_lin_ = [A_lin(3:6,3:6), A_lin(3:6, 11:13); A_lin(11:13,3:6), A_lin(11:13,11:13)];
B_lin_ = [B_lin(3:6,:); B_lin(11:13,:)];
Co_lin_ = ctrb(A_lin_,B_lin_);
unco_lin_ = length(A_lin_) - rank(Co_lin_)
Q_lin = eye(7);
R_lin = eye(4);
[K_lin,S_lin,e_lin] = lqr(A_lin_,B_lin_,Q_lin,R_lin);

% S_complet = [S_lin(1:6,1:6), zeros(6,1), S_lin(1:6,7:12); zeros(1,13); S_lin(7:12,1:6), zeros(6,1), S_lin(7:12,7:12)];
K_lin_control = [zeros(4,2), K_lin(:,1:4), zeros(4,4), K_lin(:,5:7)];

%% controle z et eps1 et 3 
% A_lin_ = [A_lin(3:6,3:6), A_lin(3:6, 8), A_lin(3:6, 10:13); A_lin(8,3:6), A_lin(8,8), A_lin(8,10:13) ; A_lin(10:13,3:6), A_lin(10:13,8), A_lin(10:13,10:13)];
% B_lin_ = [B_lin(3:6,:); B_lin(8,:); B_lin(10:13,:)];
% Co_lin_ = ctrb(A_lin_,B_lin_);
% unco_lin_ = length(A_lin_) - rank(Co_lin_)
% Q_lin = eye(9);
% R_lin = eye(4);
% [K_lin,S_lin,e_lin] = lqr(A_lin_,B_lin_,Q_lin,R_lin);
% 
% % S_complet = [S_lin(1:6,1:6), zeros(6,1), S_lin(1:6,7:12); zeros(1,13); S_lin(7:12,1:6), zeros(6,1), S_lin(7:12,7:12)];
% K_lin_control = [zeros(4,2), K_lin(:,1:4), zeros(4,1), K_lin(:,5), zeros(4,1), K_lin(:,6:9)];


p_ = [0; 0; 0];
v_ = [v_trim; 0; 0];
omega_ = [0; 0; 0];
yaw = deg2rad(0);
pitch = deg2rad(-45);
roll = deg2rad(0);
q_ = eul2quat([yaw, pitch, roll])';

% initial_state = [p_; v_; q_; omega_];
initial_state = x_eq;
% C = [zeros(3), eye(3), zeros(3)];
% D = zeros(3,4);
% sys = ss(A_lin_,B_lin_,C,D);
% Qi = eye(12);
% Ri = eye(4);
% Ni = eye(12,4);
% [Ki,Si,ei] = lqi(sys, Qi, Ri, Ni);
% K_complet_i = [zeros(4,3), Ki(:,1:3), zeros(4,1), Ki(:,4:12)];