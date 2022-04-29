function trimVal = utComputeTrimFlight_fmin(V_inf_x, drone)
    Swet   = drone.WET_SURFACE;
    Sdry   = drone.DRY_SURFACE;
    Sp = pi*drone.PROP_RADIUS^2;
    Thetam = drone.ELEVON_MEFFICIENCY(2);
    Thetaf = drone.ELEVON_FEFFICIENCY(2);
    rho = drone.RHO;
    S = Swet + Sdry;
    m = drone.MASS;
    g = drone.G;
    dr = drone.CENTRAGE;
    
    PHI_fv = drone.PHI(1:3,1:3);
    Cd_0 = PHI_fv(1,1);
    die_c = PHI_fv(3,3);
    
    eta = sqrt( norm(V_inf_x)^2); %+ phi_n*norm(B*wb)^2 

    function F = setTrim(u)
        
		F(1) = (1-Swet/4*Sp)*2*u(1) - 1/2*rho*S*eta*Cd_0*(1-2*u(3)^2)*V_inf_x + 1/2*rho*S*eta*Thetaf*Cd_0*(1-2*u(3)^2)*V_inf_x*u(2) + 2*sqrt(1-u(3)^2)*u(3)*m*g;
        F(2) = -Swet/(4*Sp)*die_c*Thetaf*2*u(1)*u(2) - rho*S*eta*die_c*sqrt(1-u(3)^2)*u(3)*V_inf_x - 1/4*rho*S*eta*Thetaf*die_c*(1-2*u(3)^2)*V_inf_x + (1-2*u(3)^2)*m*g;
        F(3) = Swet/(4*Sp)*dr*die_c*Thetam*2*u(1)*u(2) + rho*S*eta*dr*die_c*sqrt(1-u(3)^2)*u(3)*V_inf_x;
    end

    function F = setTrimFzero(u)
        F = norm(setTrim(u));
    end
    T = m*g;
    delta = 0;
%     eps3 = -sqrt(2);
    eps3 = -0.01;
    
    if false
        options = optimoptions('fsolve','Algorithm','levenberg-marquardt');
        [trimVal,fval,exitFlag] = fsolve(@setTrim,[T;delta;eps3],options);
    else
        trimVal = fminsearch(@setTrimFzero, [T; delta; eps3]);
    end    
    
end