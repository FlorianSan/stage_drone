% clear all ;
close all;
clc;

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
xi_f = drone.ELEVON_FEFFICIENCY;
a1 = drone.P_A1_CG;
a2 = drone.P_A2_CG;
Sp = pi*drone.PROP_RADIUS^2;
dR = drone.CENTRAGE;

% gravity
g = drone.G;
% vehicle mass
m = drone.MASS;

S = Swet + Sdry;

cd0 = PHI_fv(1,1);
cl0 = PHI_fv(3,3);

w = [-6;0;0];

eta = norm(w);
     
angle = -atan(w(3)/w(1) - (2*m*g)/(rho*S*cl0*eta*((xi_f(2)/xi_m(2))-1)*w(1))) ;
angle_rad= rad2deg(angle)

w_barre  = [cos(angle), 0, -sin(angle); 0, 1, 0; sin(angle), 0, cos(angle)]*(-w);
wx_barre = w_barre(1);
wz_barre = w_barre(3);


a = (2*(1-(S*cd0)/(4*Sp))*cl0)/(rho*Sp*eta*cd0*wz_barre);
b = (2*cl0*wx_barre)/(cd0*wz_barre)*(1-(S*cd0)/(4*Sp)) - (m*g*sin(angle)*cl0)/(rho*Sp*eta*cd0*wz_barre) - (S*wx_barre*cl0)/(2*Sp*wz_barre);
c = m*g*cos(angle) - (1/2)*rho*S*eta*cl0*(wz_barre + (wx_barre^2)/wz_barre) - (cl0*wx_barre*m*g*sin(angle))/(cd0*wz_barre);

r = [(-b+sqrt(b^2-4*a*c))/(2*a), (-b-sqrt(b^2-4*a*c))/(2*a)];
ua_sol = max(r);

ub = (2*m*g*sin(angle))/(rho*S*eta*cd0*xi_f(2)*wz_barre) + (wx_barre)/(xi_f(2)*wz_barre) - (4*(1-(S*cd0)/(4*Sp))*ua_sol)/(rho*S*eta*cd0*xi_f(2)*wz_barre);
ub = rad2deg(ub)

windmax = 40;
T = zeros(2*windmax*10,1);
ua1 = zeros(2*windmax*10,1);
ua2 = zeros(2*windmax*10,1);
delta = zeros(2*windmax*10,1);
theta = zeros(2*windmax*10,1);
incidence = zeros(2*windmax*10,1);
w = zeros(3,2*windmax*10);

for i = 1:2*windmax*10

    w(:,i) = [-windmax+0.1*i;0;0];
    eta2 = norm(w(:,i));
    if w(1,i) < 0
        angle = -atan(w(3,i)/w(1,i) - (2*m*g)/(rho*S*cl0*eta2*((xi_f(2)/xi_m(2))-1)*w(1,i)));
    else
        angle = -atan(w(3,i)/w(1,i) - (2*m*g)/(rho*S*cl0*eta2*((xi_f(2)/xi_m(2))-1)*w(1,i))) + pi;
    end
    
    if w(1,i) ~= 0
        w_barre  = [cos(angle), 0, -sin(angle); 0, 1, 0; sin(angle), 0, cos(angle)]*(-w(:,i));
        wx_barre = w_barre(1);
        wz_barre = w_barre(3);
        
        
        a = (2*(1-(S*cd0)/(4*Sp))*cl0)/(rho*Sp*eta2*cd0*wz_barre);
        b = (2*cl0*wx_barre)/(cd0*wz_barre)*(1-(S*cd0)/(4*Sp)) - (m*g*sin(angle)*cl0)/(rho*Sp*eta2*cd0*wz_barre) - (S*wx_barre*cl0)/(2*Sp*wz_barre);
        c = m*g*cos(angle) - (1/2)*rho*S*eta2*cl0*(wz_barre + (wx_barre^2)/wz_barre) - (cl0*wx_barre*m*g*sin(angle))/(cd0*wz_barre);
    
        r = [(-b+sqrt(b^2-4*a*c))/(2*a), (-b-sqrt(b^2-4*a*c))/(2*a)];
        ua_sol = max(r);
        ub_sol = (2*m*g*sin(angle))/(rho*S*eta2*cd0*xi_f(2)*wz_barre) + (wx_barre)/(xi_f(2)*wz_barre) - (4*(1-(S*cd0)/(4*Sp))*ua_sol)/(rho*S*eta2*cd0*xi_f(2)*wz_barre);

    else
        ua_sol = (m*g)/2*(1-(S/(4*Sp))*cd0);
        ub_sol = 0;
    end
            
    ua1(i) = r(1);
    ua2(i) = r(2);
    T(i) = ua_sol;
    delta(i) = rad2deg(ub_sol);
    incidence(i) = angle;
    theta(i) = rad2deg(angle);
end
x = - w(1,1:399);
y = incidence(1:399)';
[p,S] = polyfit(x,y,5)
(inv(S.R)*inv(S.R)')*S.normr^2/S.df
[y_fit,delta_fit] = polyval(p,x,S);

figure(1)
plot(x,y,'bo')
hold on
plot(x,y_fit,'r-')
plot(x,y_fit+2*delta_fit,'m--',x,y_fit-2*delta_fit,'m--')
title('Linear Fit of Data with 95% Prediction Interval')
legend('Data','Polynomial Fit','95% Prediction Interval')


[Tmin,I] = min(T);
deltamin = delta(I);
v_min_nrj = w(:,I);
X = ['Traction min ', num2str(Tmin),', Elevon min ',num2str(deltamin),', Vent', num2str(v_min_nrj(1))];
disp(X)

figure(2)
% tiledlayout(5,1)
% 
% ax(1) = nexttile;
% plot(-w(1,:), ua1)
% title('ua_1')
% 
% ax(2) = nexttile;
% plot(-w(1,:), ua2)
% title('ua_2')
% 
% 
% ax(3) = nexttile;
% plot(-w(1,:), T)
% title('Traction')
% 
% 
% ax(4) =nexttile;
% plot(-w(1,:), delta)
% yline(-30, "r")
% yline(30, "r")
% title('Elevon')
% 
% ax(5) = nexttile;
% plot(-w(1,:), theta)
% title('Angle')
% linkaxes(ax,'x');

% tiledlayout(3,1)
% 
% 
% ax(1) = nexttile;
% plot(-w(1,:), T)
% title('Traction')
% 
% 
% ax(2) =nexttile;
% plot(-w(1,:), delta)
% yline(-30, "r")
% yline(30, "r")
% title('Elevon')
% 
% ax(3) = nexttile;
% plot(-w(1,:), theta)
% title('Angle')
% linkaxes(ax,'x');

% tiledlayout(3,1)
% 
% 
% ax(1) = nexttile;
% plot(-w(1,1:399), T(1:399))
% title("Définition des points d'équilibre de la voilure",'FontSize',30)
% ylabel("Traction des hélices (N)", 'FontSize',15)
% 
% 
% ax(2) =nexttile;
% plot(-w(1,1:399), delta(1:399))
% yline(-30, "r")
% yline(30, "r")
% % title('Elevon')
% ylabel("Déflexion des élevons (deg)", 'FontSize',15)
% 
% ax(3) = nexttile;
% plot(-w(1,1:399), theta(1:399))
% % title("Angle d'incidence")
% linkaxes(ax,'x');
% ylabel("Angle d'incidence (deg)",'FontSize',15)
% xlabel("Composante de vent horizontal (m/s)", 'FontSize',15)

figure(2)
tiledlayout(3,1)


ax(1) = nexttile;
plot(-w(1,1:399), T(1:399))
title("Definition of wing balance points",'FontSize',30)
ylabel("Propeller traction (N)", 'FontSize',15)


ax(2) =nexttile;
plot(-w(1,1:399), delta(1:399))
yline(-30, "r")
yline(30, "r")
% title('Elevon')
ylabel("Elevon deflection (deg)", 'FontSize',15)

ax(3) = nexttile;
plot(-w(1,1:399), theta(1:399))
% title("Angle d'incidence")
linkaxes(ax,'x');
ylabel("Angle of incidence (deg)",'FontSize',15)
xlabel("Horizontal wind component (m/s)", 'FontSize',15)
