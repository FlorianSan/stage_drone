clear all ;
close all;
clc;

drone = generateDarko();
drone.wind = 0;
% %% DarkO initial points for simulation
% p_initial = [0; 0; 0];
% v_initial = [10; 0; 0];
% omega_initial = [0; 0; 0];
% 
% yaw = deg2rad(0);
% pitch = deg2rad(-45);
% roll = deg2rad(0);
% q_initial = eul2quat([yaw, pitch, roll])';
% initial_state = [p_initial; v_initial; q_initial; omega_initial];


% mdl = 'darko_model';
% % % open_system(mdl)
% opspec = operspec(mdl);
% opspec.Inputs(1).Min = 0;
% opspec.Inputs(2).Min = 0;
% 
% % opspec = addoutputspec(opspec,'darko_model/vx',);
% % opspec.Outputs(4).Known = 1;
% % opspec.Outputs(4).y = 10;
% opspec.Outputs(4).Min = 9;
% opspec.Outputs(4).Max = 11;
% 
% opspec.Outputs(7).Known = 0;
% opspec.Outputs(8).Known = 0;
% opspec.Outputs(9).Known = 0;
% 
% op = findop(mdl,opspec);
% 
% io(1) = linio('darko_model/cmd',1,'input');
% io(2) = linio('darko_model/cmd',2,'input');
% io(3) = linio('darko_model/cmd',3,'input');
% io(4) = linio('darko_model/cmd',4,'input');
% io(5) = linio('darko_model/speed',1,'output');
% 
% 
% sys = linearize(mdl,op,io);

v_trim = 10;
% %trimVal = utComputeTrimFlight_fmin_body(v_trim, drone)
%trim = utComputeTrimFlight_fmin_inertial(v_trim, drone)
% %trim_2 = utComputeTrimFlight(v_trim, drone)
% 
trim = utComputeTrimFlight_fmin_dyn(v_trim, drone)
% 

%% DarkO initial points for simulation
p_initial = [0; 0; 0];
v_initial = [v_trim; 0; 0];
q_initial = [sqrt(1-trim(4)^2); 0; trim(4);0];
omega_initial = [0; 0; 0];

u_eq = [trim(1);trim(1);trim(2);trim(2)];
x_eq = [p_initial; v_initial; q_initial; omega_initial];
matrix2latex(x_eq, 'out.tex', 'alignment', 'c', 'format', '%-6.2f', 'size', 'tiny');
eul = rad2deg(quat2eul(q_initial'));

initial_state = x_eq;
[A_lin, B_lin, C_lin, ~] = linmod('darko_model', x_eq, u_eq);
Co_lin = ctrb(A_lin,B_lin);
unco_lin = length(A_lin) - rank(Co_lin);


A_lin_ = [A_lin(3:6,3:6), A_lin(3:6,8:13); A_lin(8:13,3:6), A_lin(8:13,8:13)];
B_lin_ = [B_lin(3:6,:); B_lin(8:13,:)];
matrix2latex(A_lin_, 'out.tex', 'alignment', 'c', 'format', '%-6.2f', 'size', 'tiny');

Co_lin_ = ctrb(A_lin_,B_lin_);
unco_lin_ = length(A_lin_) - rank(Co_lin_)
Q_lin = eye(10);
R_lin = eye(4);
[K_lin,S_lin,e_lin] = lqr(A_lin_,B_lin_,Q_lin,R_lin);

% S_complet = [S_lin(1:6,1:6), zeros(6,1), S_lin(1:6,7:12); zeros(1,13); S_lin(7:12,1:6), zeros(6,1), S_lin(7:12,7:12)];
K_lin_control = [zeros(4,2), K_lin(:,1:4), zeros(4,1), K_lin(:,5:10)];

p_ = [0; 0; 0];
v_ = [v_trim; 0; 0];
omega_ = [0; 0; 0];
yaw = deg2rad(0);
pitch = deg2rad(-80);
roll = deg2rad(0);
q_ = eul2quat([yaw, pitch, roll])';

initial_state = [p_; v_; q_; omega_];

% C = [zeros(3), eye(3), zeros(3)];
% D = zeros(3,4);
% sys = ss(A_lin_,B_lin_,C,D);
% Qi = eye(12);
% Ri = eye(4);
% Ni = eye(12,4);
% [Ki,Si,ei] = lqi(sys, Qi, Ri, Ni);
% K_complet_i = [zeros(4,3), Ki(:,1:3), zeros(4,1), Ki(:,4:12)];

figure(1)
sgtitle("Position du drone dans l'espace")
subplot(3,1,1)
plot(out.tout, out.positions(:,1))
ylim([-100 1000])
grid on
ylabel('px [m]')
xlabel('time [s]')

subplot(3,1,2)
plot(out.tout, out.positions(:,2))
ylim([0 10])
grid on
ylabel('py [m]')
xlabel('time [s]')

subplot(3,1,3)
plot(out.tout, out.positions(:,3))
ylim([0 10])
grid on
ylabel('pz [m]')
xlabel('time [s]')