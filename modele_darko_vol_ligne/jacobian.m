drone = generateDarko();
p = [0;0;0];
v = [1;1;1];
q  = [1/sqrt(4);1/sqrt(4);1/sqrt(4);1/sqrt(4)];
w = [0;0;0];

x = [p;v;q;w];

T1 = 1.5;
T2 = 2.5;
d1 = deg2rad(-15);
d2 = deg2rad(15);


T1pT2 = T1+T2;
T1mT2 = T1-T2;
d1pd2 = d1+d2;
d1md2 = d1-d2;

D = q2dcm(q);

%% velocityin body frame
vb = D'*v;
vbx = vb(1);
vby = vb(2);
vbz = vb(3);

J = drone.INERTIA;


ay = drone.P_A2_CG(2);
py = drone.P_P2_CG(2);
die_c = drone.PHI(3,3);
Cd0 = drone.PHI(1,1);

Swet   = drone.WET_SURFACE;
Sdry   = drone.DRY_SURFACE;
Sp = pi*drone.PROP_RADIUS^2;
Thetam = drone.ELEVON_MEFFICIENCY(2);
Thetaf = drone.ELEVON_FEFFICIENCY(2);
kf = drone.PROP_KP;
km = drone.PROP_KM;
S = Swet + Sdry;
rho    = drone.RHO;
phi_n  = drone.PHI_n;
chord  = drone.CHORD;
ws     = drone.WINGSPAN;
B = diag([ws; chord; ws]);
dR = drone.CENTRAGE ;

eta = sqrt( norm(v)^2 + phi_n*norm(B*w)^2 );

syms x y z

% M = [
% (km/kf - (Swet/(8*Sp)) * ay * die_c * Thetaf *d1pd2)*T1mT2 + (-(Swet/(8*Sp))* ay * die_c * Thetaf* T1pT2 + 1/4 * rho* S * eta * ay * die_c * Thetaf* vbx)*d1md2; 
% (Swet/(8*Sp) * dR * die_c * Thetam * T1pT2 + 1/4 * rho* S * eta * dR * die_c * Thetam* vbx )* d1pd2 + Swet/(8*Sp) * dR * die_c * Thetam * T1mT2*d1md2 + 1/2 * rho* S *  dR  * die_c * eta *vbz ;
% -(py+Swet/(4*Sp)*ay*Cd0)*T1mT2 + (1/4 * rho* S * eta * ay * Cd0 * Thetaf* vbz)*d1md2
%     ];

M = [
(km/kf - (Swet/(8*Sp)) * ay * die_c * Thetaf *y)*x + (-(Swet/(8*Sp))* ay * die_c * Thetaf* T1pT2 + 1/4 * rho* S * eta * ay * die_c * Thetaf* vbx)*z; 
(Swet/(8*Sp) * dR * die_c * Thetam * T1pT2 + 1/4 * rho* S * eta * dR * die_c * Thetam* vbx )* y + Swet/(8*Sp) * dR * die_c * Thetam * x*z + 1/2 * rho* S *  dR  * die_c * eta *vbz ;
-(py+Swet/(4*Sp)*ay*Cd0)*x + (1/4 * rho* S * eta * ay * Cd0 * Thetaf* vbz)*z
    ];
DM_num = jacobian(M, [x, y, z]);

x= T1mT2 ;
y = d1pd2;
z = d1md2;
dM_num = double(subs(DM_num));

dM = [km/kf - (Swet/(8*Sp)) * ay * die_c * Thetaf *d1pd2, -(Swet/(8*Sp))* ay * die_c * Thetaf * T1mT2 ,(-(Swet/(8*Sp))* ay * die_c * Thetaf* T1pT2 + 1/4 * rho* S * eta * ay * die_c * Thetaf* vbx);
(Swet/(8*Sp) * dR * die_c * Thetam * d1md2), Swet/(8*Sp) * dR * die_c * Thetam * T1pT2 + 1/4 * rho* S * eta * dR * die_c * Thetam* vbx, Swet/(8*Sp) * dR * die_c * Thetam * T1mT2;
-(py+Swet/(4*Sp)*ay*Cd0), 0, 1/4 * rho* S * eta * ay * Cd0 * Thetaf* vbz
     ];
 
 diff = dM_num-dM