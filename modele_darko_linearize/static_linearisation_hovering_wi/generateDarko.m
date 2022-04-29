function drone = generateDarko()
drone = struct('MASS',               0.400,      ... % ok: 0.492
               'INERTIA',            eye(3),     ... % below
               'G',                  9.81,       ... % ok
               'P_P1_CG',            [1; 1; 1],  ... % below
               'P_P2_CG',            [1; 1; 1],  ... % below
               'P_A1_CG',            [1; 1; 1],  ... % below
               'P_A2_CG',            [1; 1; 1],  ... % below
               'INERTIA_PROP_X',     5.1116e-6,    ... % ok
               'INERTIA_PROP_N',     0,          ... % ok
               'PHI',                eye(6),     ... % below
               'RHO',                1.225,      ... % ok
               'WET_SURFACE',        0.0743,     ... % ok
               'DRY_SURFACE',        0,          ... % PROBLEME
               'PHI_n',              0,          ... % ok
               'CHORD',              0.13,       ... % ok
               'WINGSPAN',           0.55,       ... % ok
               'PROP_RADIUS',        0.125,      ... % ok
               'ELEVON_MEFFICIENCY', [0;0.93;0], ... % ok
               'ELEVON_FEFFICIENCY', [0;0.48;0], ... % ok
               'PROP_KP',            1.13728e-6,...    % ok: 1.43e-6
               'THICKNESS',          0.02   ,   ... % ok
               'PROP_KM',            2.640e-7,  ...
               'CENTRAGE',           0, ...
               'ELEVON_RATE_CHANGE', (5*pi)/3, ... % 0.2s/60° ou 0.2s/pi/3 
               'MOTOR_RATE_CHANGE',  (0.63*1000)/0.002, ... % on atteind 63% de max rpm en tau_mech = 0.002s 
               'Mb',                 zeros(3,4), ...
               'Fb',                 zeros(3,4) ...
               ); % ok
%% acording to inertia identificayiton (python + IMU measurements)
drone.INERTIA = diag([0.007018 0.002785 0.00606]);

S = drone.WET_SURFACE + drone.DRY_SURFACE;

%% according to thin airfoil phi-theory (see [1])
Cd0 = 0.025;
Cy0 = 0; 


dR = -0.1*drone.CHORD; %centrage de l'avion
drone.CENTRAGE = dR;


% Revisited Phi-theory with Prandtl's lifting-line theory
AR    = (drone.WINGSPAN^2)/drone.WET_SURFACE;

die_c = pi*AR/(1+sqrt(1+(AR^2/4)));
PHI_fv = diag([Cd0; Cy0; (die_c+Cd0)]);
PHI_mv = [0 0 0 ; 0 0 -1/drone.CHORD*dR*(die_c+Cd0); 0 1/drone.WINGSPAN*dR*Cy0 0];

PHI_mw = 1/2*([0.2792 0 0.1145; 0 1.2715 0; 0.081 0 0.0039]); %Cl Cm Cn 10% static margin
drone.PHI = [PHI_fv PHI_mv; PHI_mv PHI_mw];

%% geometric parameters

drone.P_P1_CG = [0.065; -0.155; 0];             % 10% static margin Dark0
drone.P_P2_CG = [0.065;  0.155; 0];             % 10% static margin Dark0
drone.P_A1_CG = [ 0.0; -0.155; 0];             %                   Dark0
drone.P_A2_CG = [ 0.0;  0.155; 0];             %                   Dark0



Sp = pi*drone.PROP_RADIUS^2;
Thetam = drone.ELEVON_MEFFICIENCY;
Thetaf = drone.ELEVON_FEFFICIENCY;
kf = drone.PROP_KP;
km = drone.PROP_KM;


%% matrice d'effort et moment
A_f = (1-(S/(4*Sp))*PHI_fv(1,1));
B_f = -(S/(4*Sp))*PHI_fv(3,3)*Thetaf(2,1);
%B_f = 0;

drone.Fb = [A_f A_f 0 0; 0 0 0 0; 0 0 B_f B_f];
%drone.Fb = [A_f 0 A_f 0; 0 0 0 0; 0 B_f 0 B_f];

% Calcul du moment
A_m = (km/kf);
B_m = (S/(4*Sp))*drone.P_A1_CG(2,1)*PHI_fv(3,3)*Thetaf(2,1);
C_m = (S/(4*Sp))*dR*PHI_fv(3,3)*Thetam(2,1);
D_m = (drone.P_P1_CG(2,1) + (S/(4*Sp))*drone.P_A1_CG(2,1)*PHI_fv(1,1));


drone.Mb = [A_m -A_m  B_m -B_m ; 0 0 C_m C_m ; D_m -D_m 0 0 ];
%drone.Mb = [A_m B_m -A_m -B_m ; 0 C_m 0 C_m ; D_m 0 -D_m 0 ];
end