clear all ;
close all;
clc;


nb_point = 4;

sys_hinf = cell(nb_point,1);

ReqSoft = [];
ReqHard = [];

ueq = [];

for i = 0:1:nb_point-1
    w = [-1;0;0];
    
    drone = generateDarko();
    
    PHI_fv = drone.PHI(1:3,1:3);
    PHI_mv = drone.PHI(4:6,1:3);
    PHI_mw = drone.PHI(4:6,4:6);
    phi_n  = drone.PHI_n;
    rho    = drone.RHO;
    Swet   = drone.WET_SURFACE;
    Sdry   = drone.DRY_SURFACE;
    chord  = drone.CHORD;
    ws     = drone.WINGSPAN;
    xi_m = drone.ELEVON_MEFFICIENCY;
    xi_m = xi_m(2);
    xi_f = drone.ELEVON_FEFFICIENCY;
    xi_f = xi_f(2);
    p1 = drone.P_P1_CG;
    a1 = drone.P_A1_CG;
    a2 = drone.P_A2_CG;
    Sp = pi*drone.PROP_RADIUS^2;
    dR = drone.CENTRAGE;
    kf = drone.PROP_KP;
    km = drone.PROP_KM;
    
    % gravity
    g = drone.G;
    % vehicle mass
    m = drone.MASS;
    
    J = drone.INERTIA;
    
    S = Swet + Sdry;
    
    cd0 = PHI_fv(1,1);
    cl0 = PHI_fv(3,3);
    
    
    eta = norm(w);
    
    angle = deg2rad(90);
    angle_deg = rad2deg(angle);
    ua = (m*g)/(2*(1-(S/(4*Sp))*cd0));
    ub = 0;
         
    
    
    %% DarkO initial points for simulation
    p_initial = [0; 0; 0];
    v_initial = [0; 0; 0];
    omega_initial = [0; 0; 0];
    
    yaw = deg2rad(0);
    % pitch = angle;
    pitch = deg2rad(90);
    roll = deg2rad(0);
    q_initial = eul2quat([yaw, pitch, roll])';
    initial_state = [p_initial; v_initial; q_initial; omega_initial];
    
    q_eq = [cos(angle/2); 0; sin(angle/2); 0];
    u_eq = [ua;ua;ub;ub];
    x_eq = [[0; 0; 0]; [0; 0; 0]; q_eq; [0; 0; 0]];
    x_6 = [[0; 0]; [0; 0];  sin(angle/2); 0];
    x_eq_ = [[0; 0; 0]; [0; 0; 0]; [0; 0]; [0; 0; 0]];
    
    %%%%%%%%%%%%%%%%
    % Modele linéarisé complet
    eps2 = q_eq(3);
    
    dF1_de2 = (1/m)*(4*eps2*(((S*cd0)/(4*Sp) -1 )*2*ua + 1/2*rho*S*eta*cd0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)) - 1/4*rho*S*eta*cd0*xi_f*2*ub*(((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1))))...
        + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*(1/2*rho*S*eta*cl0*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1)) + 1/4*rho*S*eta*cl0*xi_f*2*ub*(((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3))) + S/(4*Sp)*xi_f*cl0*2*ua*2*ub)...
        +(eps2^2-1)*(1/2*rho*S*eta*cd0*(-((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(3) + 4*eps2*w(1)) - 1/4*rho*S*eta*cd0*xi_f*2*ub*((4*eps2*w(3) + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(1))))...
        - (2*eps2*sqrt(1-eps2^2))* (1/2*rho*S*eta*cl0*(4*eps2*w(3) + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(1)) + 1/4*rho*S*eta*cl0*xi_f*2*ub*((4*eps2*w(1) - ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(3)))));
    
    
    dF2_de1 = (1/m)*(2*sqrt(1-eps2^2)*(1/2*rho*S*eta*cl0*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1)) + 1/4*rho*S*eta*cd0*xi_f*2*ub*(((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3))) + S/(4*Sp)*xi_f*cl0*2*ua*2*ub)...
        -2*eps2*(((S*cd0)/(4*Sp) -1 )*2*ua + 1/2*rho*S*eta*cd0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)) - 1/4*rho*S*eta*cd0*xi_f*2*ub*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1))));
    
    dF2_de3 = (1/m)*(-2*sqrt(1-eps2^2)*(((S*cd0)/(4*Sp) -1 )*2*ua + 1/2*rho*S*eta*cd0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)) -1/4*rho*S*eta*cd0* xi_f*2*ub*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1)))...
        -2*eps2*(1/2*rho*S*eta*cl0*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1)) + S/(4*Sp)*xi_f*cl0*2*ua*2*ub + 1/4*rho*S*eta*cd0* xi_f*2*ub*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3))));
    
    dF3_de2 = (1/m)*(4*eps2)*(S/(4*Sp)*xi_f*cl0*2*ua*2*ub + 1/2*rho*S*eta*cl0*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1)) + 1/4*rho*S*eta*cl0*xi_f*2*ub*(((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3))))...
        - ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*(((S*cd0)/(4*Sp) -1 )*2*ua + 1/2*rho*S*eta*cd0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)) -1/4*rho*S*eta*cd0*xi_f*2*ub*(((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1))))...
        + (2*eps2^2-1)*(1/2*rho*S*eta*cl0*(4*eps2*w(3) + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(1)) + 1/4*rho*S*eta*cl0*xi_f*2*ub*(4*eps2*w(1) + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(3)))...
        + (2*eps2*sqrt(1-eps2^2))*(1/2*rho*S*eta*cd0*(-((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(3) + 4*eps2*w(1))  - 1/4*rho*S*eta*cd0*xi_f*2*ub*(4*eps2*w(3) + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(1)));
    
    
    
    dM2_de2 = 1/drone.INERTIA(2,2)*(1/2*rho*S*cl0*dR*eta*(4*eps2*w(3) + ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(1))...
        + 1/4*rho*S*eta*dR*xi_m*cl0*2*ub*(4*eps2*w(1) - ((2*eps2^2)/sqrt(1-eps2^2) - 2*sqrt(1-eps2^2))*w(3)));
    
    
    
    A = [0 0  1 0   0   0 ;
         0 0  0 1   0   0 ;
         0 0  0 0   dF1_de2    0 ;
         0 0  0 0   dF3_de2  0 ;     
         0 0  0 0   0    (1/2)*sqrt(1-eps2^2) ;
         0 0  0 0   dM2_de2    0;     
         ];
    
    
    dF1_dT1 = (1/m)*((2*eps2^2-1)*((S*cd0)/(4*Sp)-1) - ((S*cl0)/(2*Sp))*xi_f*cl0*eps2*sqrt(1-eps2^2)*2*ub);
    dF3_dT1 = (1/m)*((2*eps2*sqrt(1-eps2^2))*((S*cd0)/(4*Sp)-1) + ((S*cl0)/(4*Sp))*xi_f*(2*eps2^2-1)*2*ub);
    dF1_dT2 = dF1_dT1;
    dF3_dT2 = dF3_dT1;
    
    dF1_dd1 = (1/m)*((-2*eps2*sqrt(1-eps2^2))*(((S*cl0)/(4*Sp))*xi_f*2*ua + 1/4*rho*S*eta*xi_f*cl0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3))) - 1/4*rho*S*eta*xi_f*cd0*(2*eps2^2-1)*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1)));
    dF3_dd1 = (1/m)*((2*eps2^2-1)*(1/2*rho*S*eta*xi_f*cl0*((2*eps2^2-1)*w(1) +(2*eps2*sqrt(1-eps2^2))*w(3)) + ((S*cl0)/(4*Sp))*xi_f*2*ua ) -(1/2*rho*S*eta*xi_f*cd0*(2*eps2*sqrt(1-eps2^2))*((2*eps2^2-1)*w(3) - (2*eps2*sqrt(1-eps2^2))*w(1))) );
    dF1_dd2 = dF1_dd1;
    dF3_dd2 = dF3_dd1;
    
    dM1_dT1 = 1/drone.INERTIA(1,1)*(km/kf + (S/(4*Sp))*a1(2)*xi_f*cl0*ub);
    dM2_dT1 = 1/drone.INERTIA(2,2)*((S/(4*Sp))*dR*cl0*xi_m*ub);
    dM3_dT1 = 1/drone.INERTIA(3,3)*((S/(4*Sp))*a1(2)*cd0 + p1(2));
    
    dM1_dT2 = 1/drone.INERTIA(1,1)*(-km/kf - (S/(4*Sp))*a1(2)*xi_f*cl0*ub);
    dM2_dT2 = 1/drone.INERTIA(2,2)*((S/(4*Sp))*dR*cl0*xi_m*ub);
    dM3_dT2 = 1/drone.INERTIA(3,3)*(-(S/(4*Sp))*a1(2)*cd0 - p1(2));
    
    dM1_dd1 = 1/drone.INERTIA(1,1)*((S/(4*Sp))*a1(2)*xi_f*cl0*ua + 1/4*rho*S*eta*a1(2)*xi_f*cl0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)));
    dM2_dd1 = 1/drone.INERTIA(2,2)*((S/(4*Sp))*dR*xi_m*cl0*ua + 1/4*rho*S*eta*dR*xi_m*cl0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)));
    dM3_dd1 = 1/drone.INERTIA(3,3)*(1/4*rho*S*eta*a1(2)*xi_f*cd0*(-(2*eps2*sqrt(1-eps2^2))*w(1) +(2*eps2^2-1)*w(3)));
    
    dM1_dd2 = 1/drone.INERTIA(1,1)*(-(S/(4*Sp))*a1(2)*xi_f*cl0*ua - 1/4*rho*S*eta*a1(2)*xi_f*cl0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)));
    dM2_dd2 = 1/drone.INERTIA(2,2)*((S/(4*Sp))*dR*xi_m*cl0*ua + 1/4*rho*S*eta*dR*xi_m*cl0*((2*eps2^2-1)*w(1) + (2*eps2*sqrt(1-eps2^2))*w(3)));
    dM3_dd2 = 1/drone.INERTIA(3,3)*(-1/4*rho*S*eta*a1(2)*xi_f*cd0*(-(2*eps2*sqrt(1-eps2^2))*w(1) +(2*eps2^2-1)*w(3)));
    
     B = [0 0 ;      
          0 0 ;
          2*dF1_dT1  2*dF1_dd1 ;      
          2*dF3_dT1 2*dF3_dd1 ;      
          0 0 ;      
          2*dM2_dT1 2*dM2_dd1  ;
          ];
    
    dF1_dwx = (1/m)*((2*eps2^2-1)*(1/2*rho*S*eta*cd0*(2*eps2^2-1) + 1/2*rho*S*eta*cd0*xi_f*2*ub*eps2*sqrt(1-eps2^2))...
        + (2*eps2*sqrt(1-eps2^2))*(1/2*rho*S*eta*cl0*2*eps2*sqrt(1-eps2^2) - 1/4*rho*S*eta*cl0*xi_f*2*ub*(2*eps2^2-1)));
    dF3_dwx = (1/m)*((2*eps2*sqrt(1-eps2^2))*(1/2*rho*S*eta*cd0*(2*eps2^2-1) + 1/2*rho*S*eta*cd0*xi_f*2*ub*eps2*sqrt(1-eps2^2))...
        - (2*eps2^2-1)*(1/2*rho*S*eta*cl0*2*eps2*sqrt(1-eps2^2) - 1/4*rho*S*eta*cl0*xi_f*2*ub*(2*eps2^2-1)));
    
    dF1_dwz =(1/m)*((2*eps2^2-1)*(1/2*rho*S*eta*cd0*2*eps2*sqrt(1-eps2^2) - 1/4*rho*S*eta*cd0*xi_f*2*ub*(2*eps2^2-1))...
        - (2*eps2*sqrt(1-eps2^2))*(1/2*rho*S*eta*cl0*(2*eps2^2-1) + 1/2*rho*S*eta*cl0*xi_f*2*ub*eps2*sqrt(1-eps2^2)));
    dF3_dwz =(1/m)*((2*eps2^2-1)*(1/2*rho*S*eta*cl0*(2*eps2^2-1) + 1/2*rho*S*eta*cl0*xi_f*2*ub*eps2*sqrt(1-eps2^2))...
        + (2*eps2*sqrt(1-eps2^2))*(1/2*rho*S*eta*cd0*2*eps2*sqrt(1-eps2^2) - 1/2*rho*S*eta*cd0*xi_f*2*ub*(2*eps2^2-1)));
    
    dM2_dwx = 1/drone.INERTIA(2,2)*(1/4*rho*S*eta*dR*xi_m*cl0*(2*eps2^2-1)*2*ub - 1/2*rho*S*eta*cl0*dR*2*eps2*sqrt(1-eps2^2));
    dM2_dwz = 1/drone.INERTIA(2,2)*(1/2*rho*S*eta*dR*xi_m*cl0*eps2*sqrt(1-eps2^2)*2*ub + 1/2*rho*S*eta*cl0*dR*(2*eps2^2-1));
    
    
    
    B2 = [0 0;
          0 0;
          dF1_dwx dF1_dwz;
          dF3_dwx dF3_dwz;
          0 0;
          dM2_dwx dM2_dwz;
          ];
    
    C = eye(6);
    
    C_control = [C(1:4,:);C(6,:)];
    
    C_control_ = eye(12);
    C_control_(8,8) = 0;
    C_control_ = [C_control_(1:7,:);C_control_(9:12,:)];
    
    D = zeros(6,4);
    
    
    Co = ctrb(A,B);
    unco = length(A) - rank(Co);
    
    sys = ss(A,[B, B2],C,D);
    sys.OutputName = 'y';
    sys.InputName = {'u_cmd(1)', 'u_cmd(2)','wx', 'wz'};

    clear dF1_de2 dF2_de1 dF2_de3 dF3_de2 dM2_de2 dF1_dT1 dF3_dT1 dF1_dT2 dF3_dT2 dF1_dd1 dF3_dd1 dF1_dd2 dF3_dd2 dM1_dT1 dM2_dT1 dM3_dT1 dM1_dT2 dM2_dT2 dM3_dT2 dM1_dd1 dM2_dd1 dM3_dd1 dM1_dd2 dM2_dd2 dM3_dd2 dF1_dwx dF3_dwx dF1_dwz dF3_dwz dM2_dwx dM2_dwz
    
    F = eye(2);
    
    [At,Bt,Ct,Dt] = linmod('BF_sans_controleur');
    bouclage = ss(At,Bt,Ct,Dt);
    
    
    %%% Création de liste entrée et sortie
    u_vector = cell(1,2);
    uw_vector = cell(1,2);
    r_vector = cell(1,5);
    yw_vector = cell(1,5);
    eps_vector = cell(1,5);
    y_vector = cell(1,5);
    sys_out_vector = cell(1,6);
    
    for k = 1:2
        uw_vector(k) = cellstr(sprintf('uw%d(%d)', i, k));
        u_vector(k) = sprintfc('u(%d)', k);
    end
    for k = 1:5
        r_vector(k) = cellstr(sprintf('r%d(%d)', i, k));
        yw_vector(k) = cellstr(sprintf('yw%d(%d)', i, k));
        eps_vector(k) = cellstr(sprintf('epsw(%d%d)', i, k));
        y_vector(k) = sprintfc('y(%d)', k);
    end
    for k = 1:6
        sys_out_vector(k) = cellstr(sprintf('sys_out%d(%d)', i, k));
    end
    
   
    bouclage.InputName = [sprintfc('wx%d', i), sprintfc('wz%d', i), r_vector, u_vector];
    bouclage.OutputName = [eps_vector, uw_vector, sys_out_vector, yw_vector, y_vector];

    sys_hinf{i+1} = bouclage;

    gain_track = 10;
    Req = TuningGoal.Gain(r_vector, eps_vector, 10^(gain_track/20)); 
    
    Req_cmd = TuningGoal.Gain(r_vector, uw_vector, 10^(6/20));  

    ReqHard=[];
    ReqSoft=[Req; Req_cmd];

end

Knu=5;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% nombre d'entrées de K
Kny=2;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% nombre de sorties de K
K_init = ones(Kny,Knu);
H_init = zeros(2,Knu);

K = tunableGain('K',Kny,Knu);  % 2 outputs, 5 inputs
K.Gain.Free = ones(Kny,Knu);    % all tunable
% K.Gain.Free = [ones(4,1),zeros(4,1),ones(4,2),zeros(4,1),ones(4,1),zeros(4,3),ones(4,1),zeros(4,1)];
K.Gain.Value = K_init; % init all to 1
% K.Gain.Maximum = 0.5;
% K.Gain.Minimum = -0.5;


H = tunableGain('H',2,Knu);  % 2 outputs, 5 inputs
H.Gain.Free = ones(2,Knu);    % all tunable
% H.Gain.Free = [ones(2,1),zeros(2,1),ones(2,2),zeros(2,1),ones(2,1),zeros(2,3),ones(2,1),zeros(2,1)];
H.Gain.Value = H_init; % init all to 0

% H.Gain.Maximum = 0.5;
% H.Gain.Minimum = -0.5;

integrator = tf({1 0; 0 1}, {[1 0] 1; 1 [1 0]});
integ = F*integrator*H;

Sum = -K+integ;
Sum.InputName = 'y';
Sum.OutputName = 'u';


Nbspecif=length(sys_hinf);
Tbf=[];

for j= 1:(Nbspecif)%%%%%%%%%%%%%%%%% CONSTRUCTION DE M(s) 
    Ttemp=lft(sys_hinf{j},Sum,Kny,Knu); 
    Tbf=append(Ttemp,Tbf);
end
T0=Tbf;%%%%%%%%%%%%%%%%%%%%%% 


option=systuneOptions('Display','iter','RandomStart',8,'MaxIter',500,'UseParallel',true,'MinDecay',1e-7,'MaxRadius',1e16);
        
[T, fSoft, gHard] = systune(T0, ReqSoft, ReqHard, option);
% figure(7);
% viewGoal([Req Req_cmd], T)
K_val = getBlockValue(T,'K');
K_val = K_val.D
H_val = getBlockValue(T,'H');
H_val = H_val.D

kf = drone.PROP_KP;
m = drone.MASS;

K_val_ = [K_val(:,1), zeros(2,1), K_val(:,2:3), zeros(2,1), K_val(:,4), zeros(2,3), K_val(:,5), zeros(2,1)]
H_val_ = [H_val(:,1), zeros(2,1), H_val(:,2:3), zeros(2,1), H_val(:,4), zeros(2,3), H_val(:,5), zeros(2,1)]
x_eq_ = zeros(11,1);
u_eq_ = [ua;ub];


savefile = 'matrix_systune_plan.mat';
save(savefile, 'x_eq_', 'u_eq_', 'H_val_', 'K_val_', 'C_control_', 'F', 'kf', 'm');
%   %%%%%%%%%%%%%%%%%%%%%% ANALYSE DES RESULTAS
[A_an,B_an,C_an,D_an] = linmod('BF_sans_controleur_analyse');
sysT = ss(A_an, B_an, C_an, D_an);
S=sysT(1:5, 3:end);
S_track = sys(3:5, 1:3);
% GS=sys_tf(:, 1:4)*S;
% KS=[K_val; H_val]*S;

W1=(1/10^(gain_track/20))*tf(1,1);
invW1=inv(W1);

figure(1);
sigma(S);
hold on
sigma(invW1, 'r',fSoft(1)*invW1, 'g')

figure(2)
sigma(S(1,1))

figure(3)
step(S(1,1))

figure(4)
step(S(2,2))

% figure(5)
% step(S(3,3))
% 
% figure(6)
% step(S(4,4))
% 
% figure;
% sigma(GS,'g');
% hold on
% sigma(invW3,fSoft(2)*invW3)
% sigma(sysG);
% 
% figure(6);
% sigma(KS);
% hold on
% sigma(invW1,fSoft(1)*invW1);

% figure(2)
% hold on
% sigma(S_track(1,1));

%  matrix2latex(K_val, 'out.tex', 'alignment', 'c', 'format', '%-6.2f', 'size', 'tiny');