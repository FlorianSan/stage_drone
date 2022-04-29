function [Fp, Mp] = external_force_moment( x, u, w, drone)
%% AERO aerodynamic forces and moments computation (in a wing section!)
%
%  this function computes the aerodynamic forces and moments in body axis
%    in view of Phi-theory in a wing section. it includes propwash effects 
%    due to thrust.
%
%  INPUTS:
%    state def. : x = (vl wb q)     in R(10x1)
%    thrust     : T (N)             in R( 3x1)
%    elevon def.: del (rad)         in R( 1x1)
%    motor speed: w (rad/s)         in R( 1x1)
%    drone specs: drone             struct
%    + required drone specs: 
%    |---- PHI                      in R( 6x6)
%    |---- RHO (kg/m^3)             in R( 1x1)
%    |---- WET_SURFACE (m^2)        in R( 1x1)
%    |---- DRY_SURFACE (m^2)        in R( 1x1)
%    |---- PHI_n                    in R( 1x1)
%    |---- CHORD (m)                in R( 1x1)
%    |---- WINGSPAN (m)             in R( 1x1)
%    |       (of the wing section, half of full drone)
%    |---- PROP_RADIUS (m)          in R( 1x1)
%    |---- ELEVON_MEFFICIENCY       in R( 3x1)
%    |---- ELEVON_FEFFICIENCY       in R( 3x1)
%
%  OUTPUTS:
%    Aero force : Fb (body axis)    in R( 3x1) 
%    Aero moment: Mb (body axis)    in R( 3x1) 
%
%  vl: vehicle velocity in NED axis (m/s) [3x1 Real]
%  wb: vehicle angular velocity in body axis (rad/s) [3x1 Real]
%  q:  quaternion attitude (according to MATLAB convention) [4x1 Real]
%
%  NOTE1: notice that w's sign depend on which section we are due to
%    counter-rotating propellers;
%  NOTE2: elevon sign convention is positive pictch-up deflections.
%  
%  refer to [1] for further information.
% 
%  REFERENCES
%    [1] Lustosa L.R., Defay F., Moschetta J.-M., "The Phi-theory 
%    approach to flight control design of tail-sitter vehicles"
%    @ http://lustosa-leandro.github.io 

%% data extraction from drone struct
PHI_fv = drone.PHI(1:3,1:3);
PHI_mv = drone.PHI(4:6,1:3);
PHI_mw = drone.PHI(4:6,4:6);
phi_n  = drone.PHI_n;
rho    = drone.RHO;
Swet   = drone.WET_SURFACE;
Sdry   = drone.DRY_SURFACE;
chord  = drone.CHORD;
ws     = drone.WINGSPAN;
Thetam = drone.ELEVON_MEFFICIENCY;
Thetaf = drone.ELEVON_FEFFICIENCY;
a1 = drone.P_A1_CG;
a2 = drone.P_A2_CG;


%% state demultiplexing
% p = x(1:3);
vl = x(4:6);
q  = x(7:10);
wb = x(11:13);

%% control demultiplexing 
%T1 = u(1);
%T2 = u(2);
d1 = u(3);
d2 = u(4);

%% DCM computation matrice du body vers inertiel
D = q2dcm(q);

%% freestream velocity computation in body frame
vinf = D'*(vl-w);

%% computation of total wing section area
S = Swet + Sdry;

%% computation of chord matrix
B = diag([ws; chord; ws]);

%% eta computation
eta = sqrt( norm(vinf)^2 + phi_n*norm(B*wb)^2 );

%% force computation

Fp = -1/2 * rho * S * PHI_fv * eta * vinf -1/2 * rho * S * PHI_mv * eta * B * wb ...
     + 1/4 * rho * S * PHI_fv * skew_sym(Thetaf) * (d1 + d2) * eta * vinf ...
     + 1/4 * rho * S * PHI_mv * skew_sym(Thetaf) * (d1 + d2) * B * eta * wb;
%% moment computation

Mp = -1/2 * rho * S * B * PHI_mv * eta * vinf - 1/2 * rho * S *B * PHI_mw * eta * wb ...
     + 1/4 * rho * S * skew_sym(a1) * PHI_fv * skew_sym(Thetaf)* d1 * eta * vinf ...
     + 1/4 * rho * S * skew_sym(a2) * PHI_fv * skew_sym(Thetaf)* d2 * eta * vinf ...
     + 1/4 * rho * S * skew_sym(a1) * PHI_mv * skew_sym(Thetaf)* d1 * eta * B * eta * wb ...
     + 1/4 * rho * S * skew_sym(a2) * PHI_mv * skew_sym(Thetaf)* d2 * eta * B * eta * wb ...
     + 1/4 * rho * S * B * PHI_mv * skew_sym(Thetam) * (d1 + d2) * eta * vinf ...
     + 1/4 * rho * S * B * PHI_mw * skew_sym(Thetam) * (d1 + d2) * eta * B * wb;
     
     
end
